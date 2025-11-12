#pragma once
#include "include/Globals.h"
#include "include/Display.h"
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
extern Adafruit_SSD1306 display;
extern String msg_mod;

void visualizza_msg_scorrevole(String msg);
void wifi_try_msg();
void wifi_ok_msg(IPAddress local_IP);
void wifi_error_msg();
void bme_error_msg();
void bme_ok_msg();
void check_display();
void task_display(void *pvParameters);