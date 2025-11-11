#pragma once
#include <Arduino.h>

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

// --- Variabili globali ---
extern bool mode_manual;
extern bool ventola_on;
extern int speed_ventola;

// --- Sensori ambiente ---
extern float temp;
extern float hum;
extern float gas_index;

// --- Funzioni comuni ---
void setup_GPIO();
