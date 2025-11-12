#pragma once
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

// --- GPIO ---
#define GPIO_PIR 23
#define GPIO_LAMP 5
#define GPIO_POTENZIOMETRO 15
#define GPIO_BTN_FAN_CONTROLLER 18
#define GPIO_LED_FAN_CONTROLLER 19
#define GPIO_FAN_PWM 32
#define GPIO_FAN_TACHIMETRO 33
// SDA = 21, SCL = 22

// --- PWM ---
#define FAN_PWM_CHANNEL 0
#define FAN_PWM_FREQ 25000
#define FAN_PWM_RESOLUTION 8

// // --- Display ---
// #define SCREEN_WIDTH 128
// #define SCREEN_HEIGHT 64
// #define OLED_RESET -1
// extern Adafruit_SSD1306 display;

// --- BME680 ---
extern Adafruit_BME680 bme;

// --- Variabili globali ---
extern SemaphoreHandle_t fan_mutex;
extern bool mode_manual;
extern bool ventola_on;
extern int speed_ventola;
// extern String msg_mod;
extern uint8_t target_fan_speed;    // Velocità target (0-255)
extern volatile int tach_pulse_count;

// // --- Sensori ambiente ---
// extern float temp;
// extern float hum;
// extern float gas_index;

// --- Funzioni comuni ---
void setup_GPIO();
void IRAM_ATTR tachimetro_interrupt();
float read_fan_rpm();
// float gas_to_AirQualityIndex(double gas_ohm);
// String air_index_to_msg(float quality_index); 
