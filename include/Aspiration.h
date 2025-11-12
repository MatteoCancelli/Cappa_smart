#pragma once
#include "include/Globals.h"

void IRAM_ATTR tachimetro_interrupt();
float read_fan_rpm();
void task_logica_automatica_ventola(void *pvParameters);
void task_controllo_manuale_velocita(void *pvParameters);
void task_toggle_mode(void *pvParameters);
void task_attuatore_ventola(void *pvParameters);