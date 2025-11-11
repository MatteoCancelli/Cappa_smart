#include "include/Globals.h"
#include <Arduino.h>
#include "esp32-hal-ledc.h"
#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_BME680.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "config.h"


// // SDA = 21, SCL = 22
// #define GPIO_PIR 23
// #define GPIO_LAMP 5
// #define GPIO_POTENZIOMETRO 15
// #define GPIO_BTN_FAN_CONTROLLER 18
// #define GPIO_LED_FAN_CONTROLLER 19
// #define GPIO_FAN_PWM 32
// #define GPIO_FAN_TACHIMETRO 33

// #define FAN_PWM_FREQ 25000
// #define FAN_PWM_RESOLUTION 8  // 0–255

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Client MQTT
WiFiClient espClient;
PubSubClient mqttClient(espClient);

// Topic MQTT
#define TOPIC_TEMP "home/bme688/temperature"
#define TOPIC_HUM "home/bme688/humidity"
#define TOPIC_GAS "home/bme688/gas_index"
#define TOPIC_AIR_QUALITY "home/bme688/air_quality"
#define TOPIC_FAN_MODE "home/fan/mode"
#define TOPIC_FAN_SPEED "home/fan/speed"
#define TOPIC_FAN_RPM "home/fan/rpm"
#define TOPIC_PIR "home/pir/motion"

Adafruit_BME680 bme;

// // Variabili sensore (scritte da task_bme)
// volatile float temp = 0;
// volatile float hum = 0;
// volatile float gas_index = 0;

// Variabili display
String msg_mod = "";

// Variabili ventola (protette da mutex)
SemaphoreHandle_t fan_mutex;
//bool mode_manual = false;        // false = automatico, true = manuale
uint8_t target_fan_speed = 0;    // Velocità target (0-255)
volatile int tach_pulse_count = 0;


void IRAM_ATTR tachimetro_interrupt() {
  tach_pulse_count++;
}

float read_fan_rpm() {
  static unsigned long last_time = 0;
  static int last_pulses = 0;

  unsigned long now = millis();
  if (now - last_time >= 1000) {  
    int pulses = tach_pulse_count - last_pulses;
    last_pulses = tach_pulse_count;
    last_time = now;
    return (pulses / 2.0f) * 60.0f; // RPM
  }
  return -1;  // non ancora aggiornato
}

float gas_to_AirQualityIndex(double gas_ohm) {
  const double GAS_MIN = 10000.0;   // 10 kΩ = aria pessima
  const double GAS_MAX = 120000.0;  // 120 kΩ = aria pulita

  if (gas_ohm < GAS_MIN) gas_ohm = GAS_MIN;
  if (gas_ohm > GAS_MAX) gas_ohm = GAS_MAX;

  float aqi = (gas_ohm - GAS_MIN) / (GAS_MAX - GAS_MIN) * 100;
  return aqi;
}

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

void setup_wifi() {
  delay(10);
  Serial.println();
  Serial.print("Connessione a ");
  Serial.println(WIFI_SSID);

  // Mostra su display
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("Connessione WiFi...");
  display.display();

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 40) {
    delay(500);
    Serial.print(".");
    attempts++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println();
    Serial.println("WiFi connesso!");
    Serial.print("IP: ");
    Serial.println(WiFi.localIP());

    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("WiFi OK!");
    display.setCursor(0, 16);
    display.print("IP: ");
    display.println(WiFi.localIP());
    display.display();
    delay(2000);
  } else {
    Serial.println();
    Serial.println("ERRORE: WiFi non connesso!");
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("WiFi ERRORE!");
    display.setCursor(0, 16);
    display.println("Controllo cred.");
    display.display();
    delay(3000);
  }
}

void reconnect_mqtt() {
  // Non bloccare se non connesso
  if (!mqttClient.connected()) {
    Serial.print("Connessione MQTT...");
    
    if (mqttClient.connect(MQTT_CLIENT_ID, MQTT_USER, MQTT_PASSWORD)) {
      Serial.println(" OK!");
    } else {
      Serial.print(" FALLITA, rc=");
      Serial.println(mqttClient.state());
    }
  }
}

void task_mqtt_publish(void *pvParameters) {
  TickType_t last_wake_time = xTaskGetTickCount();
  const TickType_t frequency = pdMS_TO_TICKS(5000);

  for (;;) {
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("WiFi disconnesso, riconnessione...");
      setup_wifi();
      vTaskDelay(pdMS_TO_TICKS(5000));
      continue;
    }

    if (!mqttClient.connected()) {
      reconnect_mqtt();
    }

    // Loop MQTT per gestire messaggi
    mqttClient.loop();

    // Pubblica dati se connesso
    if (mqttClient.connected()) {
      char buffer[32];
      dtostrf(temp, 5, 2, buffer);
      mqttClient.publish(TOPIC_TEMP, buffer);
      dtostrf(hum, 5, 2, buffer);
      mqttClient.publish(TOPIC_HUM, buffer);
      dtostrf(gas_index, 5, 2, buffer);
      mqttClient.publish(TOPIC_GAS, buffer);
      String air_msg = air_index_to_msg(gas_index);
      mqttClient.publish(TOPIC_AIR_QUALITY, air_msg.c_str());
      uint8_t current_speed = 0;
      if (xSemaphoreTake(fan_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        current_speed = target_fan_speed;
        mqttClient.publish(TOPIC_FAN_MODE, mode_manual ? "manual" : "auto");
        xSemaphoreGive(fan_mutex);
      }
      sprintf(buffer, "%d", current_speed);
      mqttClient.publish(TOPIC_FAN_SPEED, buffer);
      float rpm = read_fan_rpm();
      if (rpm >= 0) {
        dtostrf(rpm, 6, 0, buffer);
        mqttClient.publish(TOPIC_FAN_RPM, buffer);
      }
      //mqttClient.publish(TOPIC_PIR, motion_detected ? "detected" : "clear");

      Serial.println("Dati MQTT pubblicati");
    }

    vTaskDelayUntil(&last_wake_time, frequency);
  }
}

void task_bme(void *pvParameters){
  for(;;) {
    if (!bme.performReading()) {
      Serial.println("Lettura fallita!");
      display.clearDisplay();
      display.setCursor(0, 0);
      display.println("Lettura fallita!");
      display.display();
      vTaskDelay(pdMS_TO_TICKS(1000));
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

void task_logica_automatica_ventola(void *pvParameters) {
  unsigned long start_time = millis();
  bool sensor_warmup_skipped = false;

  Serial.println("Ventola SPENTA - attendo riscaldamento sensore...");

  for (;;) {
    // Skip warmup se il sensore è già caldo (gas_index >= 40 -> ventola spenta)
    if (!sensor_warmup_skipped) {
      if (gas_index >= 40) {
        Serial.println("Sensore già caldo, skip warmup!");
        sensor_warmup_skipped = true;
      } else if (millis() - start_time >= 270000) {
        Serial.println("Warmup completato (4.5 min)");
        sensor_warmup_skipped = true;
      } else {
        // Ancora in warmup
        vTaskDelay(pdMS_TO_TICKS(2000));
        continue;
      }
    }

    // Determina se la ventola deve essere accesa (in modalità automatica)
    bool condizione_accensione = (gas_index < 40 || hum > 75 || temp > 50);

    if (xSemaphoreTake(fan_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
      if (!mode_manual) {
        // Modalità automatica: imposta velocità fissa 150 se condizioni soddisfatte
        target_fan_speed = condizione_accensione ? 150 : 0;
      }
      xSemaphoreGive(fan_mutex);
    }
    vTaskDelay(pdMS_TO_TICKS(2000));
  }
}

void task_controllo_manuale_velocita(void *pvParameters) {
  for (;;) {
    if (xSemaphoreTake(fan_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
      if (mode_manual) {
        int pot_value = analogRead(GPIO_POTENZIOMETRO);
        target_fan_speed = map(pot_value, 0, 4095, 0, 255);
      }
      xSemaphoreGive(fan_mutex);
    }

    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

void task_toggle_mode(void *pvParameters){
  int button_last_state = HIGH;

  for (;;) {
    int button_state = digitalRead(GPIO_BTN_FAN_CONTROLLER);

    if (button_state == LOW && button_last_state == HIGH) {
      if (xSemaphoreTake(fan_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        mode_manual = !mode_manual;
        digitalWrite(GPIO_LED_FAN_CONTROLLER, mode_manual ? HIGH : LOW);
        Serial.printf("Modalità: %s\n", mode_manual ? "MANUALE" : "AUTOMATICA");
        xSemaphoreGive(fan_mutex);
      }
      vTaskDelay(pdMS_TO_TICKS(200)); // debounce
    }

    button_last_state = button_state;
    vTaskDelay(pdMS_TO_TICKS(50)); 
  }
}

void task_attuatore_ventola(void *pvParameters) {
  for (;;) {
    uint8_t speed_to_set = 0;

    if (xSemaphoreTake(fan_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
      speed_to_set = target_fan_speed;
      xSemaphoreGive(fan_mutex);
      Serial.printf("Fan speed: %d PWM, %.0f RPM\n", speed_to_set, read_fan_rpm());
    }

    ledcWrite(GPIO_FAN_PWM, speed_to_set);

    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

// void setup_GPIO(){
//   pinMode(GPIO_POTENZIOMETRO, INPUT);
//   pinMode(GPIO_BTN_FAN_CONTROLLER, INPUT_PULLUP);
//   pinMode(GPIO_LED_FAN_CONTROLLER, OUTPUT);

//   gpio_set_direction((gpio_num_t)GPIO_PIR, GPIO_MODE_INPUT);
//   gpio_set_direction((gpio_num_t)GPIO_LAMP, GPIO_MODE_OUTPUT);
//   gpio_set_level((gpio_num_t)GPIO_LAMP, 0);

//   ledcAttach(GPIO_FAN_PWM, FAN_PWM_FREQ, FAN_PWM_RESOLUTION);
  
//   pinMode(GPIO_FAN_TACHIMETRO, INPUT_PULLUP);
//   attachInterrupt(digitalPinToInterrupt(GPIO_FAN_TACHIMETRO), tachimetro_interrupt, FALLING);
// }

void setup(){
  setup_GPIO();
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(100000);

  fan_mutex = xSemaphoreCreateMutex();
  
  check_display();
  check_bme();

  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setGasHeater(320, 150);

  setup_wifi();

  mqttClient.setServer(MQTT_SERVER, MQTT_PORT);
  mqttClient.setKeepAlive(60);

  Serial.println("Inizio Task");

  // Task sensori e display
  xTaskCreate(task_bme, "BME688", 4096, NULL, 2, NULL);
  xTaskCreate(task_display, "Display", 4096, NULL, 1, NULL);
  xTaskCreate(task_pir, "PIR", 4096, NULL, 3, NULL);
  
  // Task ventola: separazione responsabilità
  xTaskCreate(task_logica_automatica_ventola, "LogicaAuto", 4096, NULL, 1, NULL);
  xTaskCreate(task_controllo_manuale_velocita, "PotenzioMetro", 4096, NULL, 1, NULL);
  xTaskCreate(task_toggle_mode, "ToggleMode", 4096, NULL, 1, NULL);
  xTaskCreate(task_attuatore_ventola, "AttuatoreVentola", 4096, NULL, 2, NULL);

  xTaskCreate(task_mqtt_publish, "MQTT_Publish", 4096, NULL, 2, NULL);
}

void loop(){
  // Vuoto: tutto gestito da FreeRTOS
}