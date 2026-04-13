#pragma once
#include "include/Globals.h"
#include <bsec.h>

extern Bsec bsec;

void check_bme();
void task_bme(void *pvParameters);