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
Adafruit_BME680 bme;
String msg_mod = "";

SemaphoreHandle_t fan_mutex;
bool mode_manual = false;
bool ventola_on = false;
int speed_ventola = 0;
uint8_t target_fan_speed = 0;    // Velocità target (0-255)
volatile int tach_pulse_count = 0;
bool motion_detected = false;

float temp = 0;
float hum = 0;
float gas_index = 0;

WiFiClient espClient;
PubSubClient mqttClient(espClient);

#define GPIO_PIR 23
#define GPIO_LAMP 5
#define GPIO_POTENZIOMETRO 15
#define GPIO_BTN_FAN_CONTROLLER 18
#define GPIO_LED_FAN_CONTROLLER 19
#define GPIO_FAN_PWM 32
#define GPIO_FAN_TACHIMETRO 33
// SDA = 21, SCL = 22

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
