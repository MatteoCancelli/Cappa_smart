#include "include/Globals.h"
#include <Arduino.h>
#include <Wire.h>
#include "esp32-hal-ledc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "include/Aspiration.h"
#include "include/BME680.h"
#include "include/Display.h"
#include "include/PIR.h"
#include "include/TaskMQTT.h"

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
String scrolling_message = "";

SystemState system_state = {
  .temperature       = 0.0f,
  .humidity          = 0.0f,
  .air_quality_pct   = 0.0f,
  .iaq_score         = 0.0f,
  .is_motion_detected = false,
  .is_manual_mode    = false,
  .fan_speed_target  = 0,
  .fan_mutex         = NULL
};

WiFiClient espClient;
PubSubClient mqttClient(espClient);

void init_gpio()
{
  analogSetPinAttenuation(GPIO_POTENZIOMETRO, ADC_11db);

  pinMode(GPIO_POTENZIOMETRO,      INPUT);
  pinMode(GPIO_BTN_FAN_CONTROLLER, INPUT_PULLUP);
  pinMode(GPIO_LED_FAN_CONTROLLER, OUTPUT);
  pinMode(GPIO_FAN_ENABLE,         OUTPUT);
  digitalWrite(GPIO_FAN_ENABLE,    LOW);
  pinMode(GPIO_LAMP,               OUTPUT);
  pinMode(GPIO_PIR,                INPUT);

  ledcAttach(GPIO_FAN_PWM, FAN_PWM_FREQ, FAN_PWM_RESOLUTION);
}
