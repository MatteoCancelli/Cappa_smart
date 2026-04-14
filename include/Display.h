#pragma once
#include "include/Globals.h"
#include "include/SystemState.h"
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1

extern Adafruit_SSD1306 display;
extern String scrolling_message;

void init_display();
void wifi_try_msg();
void wifi_ok_msg(IPAddress local_ip);
void wifi_error_msg();
void bme_error_msg();
void bme_ok_msg();
void bme_fail_msg();
void task_display_update(void* state);
