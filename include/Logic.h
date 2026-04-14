#pragma once
#include <Arduino.h>

int calcola_velocita_automatica(float gas, float humidity, float temperature);
float iaq_to_percentage(float iaq);
String air_index_to_msg(float quality_index);