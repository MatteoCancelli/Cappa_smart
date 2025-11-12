#include "Arduino.h"
#include "include/Globals.h"
#include "esp32-hal-ledc.h"
#include <Wire.h>
//#include <Adafruit_SSD1306.h>
//#include <Adafruit_BME680.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "config.h"

// Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

SemaphoreHandle_t fan_mutex;
WiFiClient espClient;
PubSubClient mqttClient(espClient);

bool mode_manual = false;
bool ventola_on = false;
int speed_ventola = 0;
// String msg_mod = "";
uint8_t target_fan_speed = 0;    // Velocità target (0-255)
volatile int tach_pulse_count = 0;

// Adafruit_BME680 bme;
// float temp = 0;
// float hum = 0;
// float gas_index = 0;

void setup_GPIO() {
  pinMode(GPIO_POTENZIOMETRO, INPUT);
  pinMode(GPIO_BTN_FAN_CONTROLLER, INPUT_PULLUP);
  pinMode(GPIO_LED_FAN_CONTROLLER, OUTPUT);
  pinMode(GPIO_FAN_TACHIMETRO, INPUT_PULLUP);
  pinMode(GPIO_LAMP, OUTPUT);
  pinMode(GPIO_PIR, INPUT);

  //ledcSetup(FAN_PWM_CHANNEL, FAN_PWM_FREQ, FAN_PWM_RESOLUTION);
  //ledcAttachPin(GPIO_FAN_PWM, FAN_PWM_CHANNEL);
  ledcAttach(GPIO_FAN_PWM, FAN_PWM_FREQ, FAN_PWM_RESOLUTION);
}

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

// float gas_to_AirQualityIndex(double gas_ohm) {
//   const double GAS_MIN = 10000.0;   // 10 kΩ = aria pessima
//   const double GAS_MAX = 120000.0;  // 120 kΩ = aria pulita

//   if (gas_ohm < GAS_MIN) gas_ohm = GAS_MIN;
//   if (gas_ohm > GAS_MAX) gas_ohm = GAS_MAX;

//   float aqi = (gas_ohm - GAS_MIN) / (GAS_MAX - GAS_MIN) * 100;
//   return aqi;
// }

// String air_index_to_msg(float quality_index) {
//   if      (quality_index < 20)  return "Apri tutto";     // pessima
//   else if (quality_index < 40)  return "Aria stantia";   // scarsa
//   else if (quality_index < 60)  return "Aria viziata";   // media
//   else if (quality_index < 80)  return "Aria normale";   // buona
//   else                          return "Aria fresca";    // ottima
// }
