#pragma once
#include <Arduino.h>
#include <Wire.h>
#include "esp32-hal-ledc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "include/SystemState.h"
#include "include/Aspiration.h"
#include "include/BME680.h"
#include "include/Display.h"
#include "include/PIR.h"
#include "include/TaskMQTT.h"

#define GPIO_PIR                 15
#define GPIO_LAMP                5
#define GPIO_POTENZIOMETRO       36
#define GPIO_BTN_FAN_CONTROLLER  18
#define GPIO_LED_FAN_CONTROLLER  19
#define GPIO_FAN_PWM             32
#define GPIO_FAN_ENABLE          33
// SDA = 21, SCL = 22

#define FAN_PWM_CHANNEL    0
#define FAN_PWM_FREQ       25000
#define FAN_PWM_RESOLUTION 8

class Adafruit_SSD1306;

extern Adafruit_SSD1306 display;
extern String scrolling_message;
extern SystemState system_state;

void init_gpio();
