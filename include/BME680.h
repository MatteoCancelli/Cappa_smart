#pragma once
#include "include/Globals.h"
#include "include/Display.h"
#include <Adafruit_BME680.h>

extern Adafruit_BME680 bme;
extern float temp;
extern float hum;
extern float gas_index;

void check_bme();
float gas_to_AirQualityIndex(double gas_ohm);
String air_index_to_msg(float quality_index); 
void task_bme(void *pvParameters);