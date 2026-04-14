#pragma once
#include "include/Globals.h"
#include "include/SystemState.h"
#include <bsec.h>

extern Bsec bsec_sensor;

void init_environment_sensor(SystemState* state);
void task_environment_sensor(void* state);
