#include <Arduino.h>
#include "esp32-hal-ledc.h"
#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_BME680.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// SDA = 21, SCL = 22
#define GPIO_PIR 23
#define GPIO_LAMP 5
#define GPIO_POTENZIOMETRO 15
#define GPIO_BTN_FAN_CONTROLLER 18
#define GPIO_LED_FAN_CONTROLLER 19
#define GPIO_FAN_PWM 32
#define GPIO_FAN_TACHIMETRO 33

#define FAN_PWM_CHANNEL 0
#define FAN_PWM_FREQ 25000
#define FAN_PWM_RESOLUTION 8  // 0–255

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// BME688 a indirizzo 0x76 (o 0x77 se collegato ADDR a VCC)
Adafruit_BME680 bme;
volatile float temp = 0;
volatile float hum = 0;
volatile float gas_index = 0;
String msg_mod = "";
bool lamp_state = false;
bool mode_manual = false;   // false = automatico
bool ventola_on = false;
int speed_ventola = 0;
volatile int tach_pulse_count = 0;

void IRAM_ATTR tachimetro_interrupt() {
  tach_pulse_count++;
}

void set_fan_speed(uint8_t speed) {
  ledcWrite(GPIO_FAN_PWM, speed);
}

float read_fan_rpm() {
  static unsigned long last_time = 0;
  static int last_pulses = 0;

  unsigned long now = millis();
  if (now - last_time >= 1000) {  
    int pulses = tach_pulse_count - last_pulses;
    last_pulses = tach_pulse_count;
    last_time = now;
    // La maggior parte delle ventole 4 fili genera 2 impulsi per giro
    return (pulses / 2.0f) * 60.0f; // RPM
  }
  return -1;  // non aggiornato
}

// Funzione per convertire resistenza gas in Air Quality Index (0-100)
float gas_to_AirQualityIndex(double gas_ohm) {
  const double GAS_MIN = 10000.0;   // 10 kΩ = aria pessima
  const double GAS_MAX = 120000.0;  // 120 kΩ = aria pulita

  if (gas_ohm < GAS_MIN) gas_ohm = GAS_MIN;
  if (gas_ohm > GAS_MAX) gas_ohm = GAS_MAX;

  float aqi = (gas_ohm - GAS_MIN) / (GAS_MAX - GAS_MIN) * 100;
  return aqi;
}

// Funzione che converte AQI (%) in messaggio leggibile
String air_index_to_msg(float quality_index) {
  if      (quality_index < 20)  return "Apri tutto";     // pessima
  else if (quality_index < 40)  return "Aria stantia";   // scarsa
  else if (quality_index < 60)  return "Aria viziata";   // media
  else if (quality_index < 80)  return "Aria normale";   // buona
  else                          return "Aria fresca";    // ottima
}

void visualizza_msg_scorrevole(String msg){
  if (msg.length() >= 10){
    display.print(msg_mod);
    msg_mod.remove(0,1);
  }
  else{
    display.print(msg);
  }
}

void check_display(){
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("Errore display");
    while (true);
  }

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("Display OK");
  display.display();

  delay(1000);
}

void check_bme(){
  if (!bme.begin(0x76)) {
    Serial.println("Errore: BME688 non trovato!");
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("ERRORE BME688");
    display.setCursor(0, 16);
    display.println("Controlla I2C!");
    display.display();
    while (true);
  }
  Serial.println("BME688 trovato!");
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("BME688 OK");
  display.display();
  delay(1000);
}

void task_bme(void *pvParameters){
  for(;;) {
    if (!bme.performReading()) {
    Serial.println("Lettura fallita!");
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("Lettura fallita!");
    display.display();
    delay(1000);
    return;
    }
    else {
      temp = bme.temperature;
      hum  = bme.humidity;
      gas_index = gas_to_AirQualityIndex(bme.gas_resistance);
    }
    vTaskDelay(pdMS_TO_TICKS(2000)); 
  }
}

void task_display(void *pvParameters){
  for(;;) {
    display.clearDisplay();
    display.setCursor(0, 0);
    display.setTextSize(1);
    display.print("Temperatura:  "); display.print(temp, 1); display.println(" C");
    display.setCursor(0, 12);
    display.print("Umidita:      "); display.print(hum, 1); display.println(" %");
    display.setCursor(0, 24);
    display.print("Qualita aria: "); display.print(gas_index, 1); display.println(" %");
    display.setCursor(0, 48);
    display.setTextSize(2);

    String msg = air_index_to_msg(gas_index);
    if (msg_mod.length() < 10)
      msg_mod = msg;  

    visualizza_msg_scorrevole(msg);
    display.display();

    vTaskDelay(pdMS_TO_TICKS(500)); 
  }
}

void task_pir(void *pvParameters) {
  TickType_t last_motion_time = 0;
  bool lamp_on = false;

  for (;;) {
    int pir_state = gpio_get_level((gpio_num_t)GPIO_PIR);

    if (pir_state == 1) {
      last_motion_time = xTaskGetTickCount();
      if (!lamp_on) {
          gpio_set_level((gpio_num_t)GPIO_LAMP, 1);
          lamp_on = true;
      }
    }

    if (lamp_on &&
      (xTaskGetTickCount() - last_motion_time > pdMS_TO_TICKS(5000))) {
      gpio_set_level((gpio_num_t)GPIO_LAMP, 0);
      lamp_on = false;
    }

    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

void task_aspirazione(void *pvParameters) {
  unsigned long start_time = millis();
  bool flag_temp_heater_low = true;

  Serial.println("Ventola SPENTA - attendo riscaldamento sensore...");

  //bme688 danneggiato. Servono 4,5 minuti perché si riscaldi abbastanza da capire che aqi >= 40% 
  for (;;) {
    if ((millis() - start_time < 270000 || gas_index < 40) && flag_temp_heater_low){
      flag_temp_heater_low = false;
      continue;
    }

    bool condizione_accensione = (gas_index < 40 || hum > 75 || temp > 50);

    if (!mode_manual) {
      ventola_on = condizione_accensione;
      speed_ventola = ventola_on ? 150 : 0;
    }

    vTaskDelay(pdMS_TO_TICKS(2000));
  }
}

void task_check_btn_controllo_aspirazione(void *pvParameters){
  int button_last_state = HIGH;

  for (;;) {
    int button_state = digitalRead(GPIO_BTN_FAN_CONTROLLER);

    if (button_state == LOW && button_last_state == HIGH) {
      mode_manual = !mode_manual;
      digitalWrite(GPIO_LED_FAN_CONTROLLER, mode_manual ? HIGH : LOW);
      Serial.printf("Modalità: %s\n", mode_manual ? "MANUALE" : "AUTOMATICA");
      vTaskDelay(pdMS_TO_TICKS(200)); 
    }

    button_last_state = button_state;
    vTaskDelay(pdMS_TO_TICKS(50)); 
  }
}

void task_ventola_controller(void *pvParameters) {
  for (;;) {
    if (mode_manual) {
      int pot_value = analogRead(GPIO_POTENZIOMETRO);
      speed_ventola = map(pot_value, 0, 4095, 0, 255);
      ventola_on = speed_ventola > 10;
    }

    set_fan_speed(ventola_on ? speed_ventola : 0);

    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}


void setup_GPIO(){
  pinMode(GPIO_POTENZIOMETRO, INPUT);
  pinMode(GPIO_BTN_FAN_CONTROLLER, INPUT_PULLUP);
  pinMode(GPIO_LED_FAN_CONTROLLER, OUTPUT);

  gpio_set_direction((gpio_num_t)GPIO_PIR, GPIO_MODE_INPUT);
  gpio_set_direction((gpio_num_t)GPIO_LAMP, GPIO_MODE_OUTPUT);
  gpio_set_level((gpio_num_t)GPIO_LAMP, 0);

  ledcAttach(GPIO_FAN_PWM, FAN_PWM_FREQ, FAN_PWM_RESOLUTION);
  
  pinMode(GPIO_FAN_TACHIMETRO, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(GPIO_FAN_TACHIMETRO), tachimetro_interrupt, FALLING);
}

void setup(){
  setup_GPIO();
  Serial.begin(115200);
  Wire.begin(); //I2C
  Wire.setClock(100000);
  
  check_display();

  check_bme();

  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setGasHeater(320, 150);



  Serial.println("Inizio Task");

  xTaskCreate(task_bme, "BME", 4096, NULL, 2, NULL);
  xTaskCreate(task_display, "Display", 4096, NULL, 1, NULL);
  xTaskCreate(task_pir, "PIR", 4096, NULL, 3, NULL);
  xTaskCreate(task_aspirazione, "Ventola", 4096, NULL, 1, NULL);
  xTaskCreate(task_check_btn_controllo_aspirazione, "btn_auto", 4096, NULL, 1, NULL);
  xTaskCreate(task_ventola_controller, "ventola_manuale", 4096, NULL, 1, NULL);
}

void loop(){
}
