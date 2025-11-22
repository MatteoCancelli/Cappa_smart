#pragma once
#include "include/Globals.h"

void IRAM_ATTR tachimetro_interrupt();
static void ensure_fan_mutex();
float read_fan_rpm();
void attuatore_ventola();
static void controllo_manuale_velocita();
static bool warmup_delay(unsigned long start_time);
void task_toggle_mode(void *pvParameters);
void task_logica_ventola(void *pvParameters);