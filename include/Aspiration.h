#pragma once
#include "include/Globals.h"

static void ensure_fan_mutex();
void attuatore_ventola();
static void controllo_manuale_velocita();
static bool warmup_delay(unsigned long start_time);
void task_toggle_mode(void *pvParameters);
void task_logica_ventola(void *pvParameters);