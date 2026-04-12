#pragma once
#include <Arduino.h>

int calcola_velocita_automatica(float gas, float humidity, float temperature);
float gas_to_AirQualityIndex(double gas_ohm);
String air_index_to_msg(float quality_index);