#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_BME680.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define GPIO_PIR 23
#define GPIO_LAMP 5
#define GPIO_POTENZIOMETRO 15
#define GPIO_VENTOLA 17
#define GPIO_BTN_SPEED_AUTO 18
#define GPIO_LED_SPEED_AUTO 19

// SDA = 21, SCL = 22
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
  bool ventola_state = false;
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

    if (gas_index != 0) {
      if (condizione_accensione && !ventola_state) {
        ventola_state = true;
        digitalWrite(GPIO_VENTOLA, HIGH);
        Serial.println("Ventola ACCESA");
      } 
      else if (!condizione_accensione && ventola_state) {
        ventola_state = false;
        digitalWrite(GPIO_VENTOLA, LOW);
        Serial.println("Ventola SPENTA");
      }
    }

    vTaskDelay(pdMS_TO_TICKS(ventola_state ? 10000 : 200));
  }
}

void task_btn_speed_auto(void *pvParameters){
  int button_last_state = HIGH;

  for (;;) {
    int button_state = digitalRead(GPIO_BTN_SPEED_AUTO);

    if (button_state == LOW && button_last_state == HIGH) {
      lamp_state = !lamp_state;  
      digitalWrite(GPIO_LED_SPEED_AUTO, lamp_state ? HIGH : LOW);
      Serial.printf("LED %s\n", lamp_state ? "ON (Manuale)" : "OFF (Automatico)");
      vTaskDelay(pdMS_TO_TICKS(200)); 
    }

    button_last_state = button_state;
    vTaskDelay(pdMS_TO_TICKS(50)); 
  }
}

void task_ventola_manuale(void *pvParameters) {
  for (;;) {
    if(lamp_state){
      int pot_value = analogRead(GPIO_POTENZIOMETRO); 
    }
    vTaskDelay(pdMS_TO_TICKS(200));
  }
}

void setup_GPIO(){
  pinMode(GPIO_POTENZIOMETRO, INPUT);
  pinMode(GPIO_BTN_SPEED_AUTO, INPUT_PULLUP);
  pinMode(GPIO_LED_SPEED_AUTO, OUTPUT);
  pinMode(GPIO_VENTOLA, OUTPUT);

  gpio_set_direction((gpio_num_t)GPIO_PIR, GPIO_MODE_INPUT);
  gpio_set_direction((gpio_num_t)GPIO_LAMP, GPIO_MODE_OUTPUT);
  gpio_set_level((gpio_num_t)GPIO_LAMP, 0);
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
  xTaskCreate(task_btn_speed_auto, "btn_auto", 4096, NULL, 1, NULL);
  xTaskCreate(task_ventola_manuale, "ventola_manuale", 4096, NULL, 1, NULL);
}

void loop(){
}
