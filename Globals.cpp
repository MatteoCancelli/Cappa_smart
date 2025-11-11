#include "Arduino.h"
#include "include/Globals.h"
#include "esp32-hal-ledc.h"

bool mode_manual = false;
bool ventola_on = false;
int speed_ventola = 0;

float temp = 0;
float hum = 0;
float gas_index = 0;

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
