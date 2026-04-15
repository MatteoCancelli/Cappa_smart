#include "include/Globals.h"
#include "include/Logic.h"
#include "include/Display.h"
#include <Adafruit_SSD1306.h>

void wifi_try_msg()
{
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("Connessione WiFi...");
  display.display();
}

void wifi_ok_msg(IPAddress local_ip)
{
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("WiFi OK!");
  display.setCursor(0, 16);
  display.print("IP: ");
  display.println(local_ip);
  display.display();
}

void wifi_error_msg()
{
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("WiFi ERRORE!");
  display.setCursor(0, 16);
  display.println("Controllo cred.");
  display.display();
}

void bme_error_msg()
{
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("ERRORE BME688");
  display.setCursor(0, 16);
  display.println("Controlla I2C!");
  display.display();
}

void bme_ok_msg()
{
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("BME688 OK");
  display.display();
}

void bme_fail_msg()
{
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("Lettura fallita!");
  display.display();
}

static void display_scrolling_text(String msg)
{
  if (msg.length() >= 10)
  {
    display.print(scrolling_message);
    scrolling_message.remove(0, 1);
  }
  else
  {
    display.print(msg);
  }
}

void init_display()
{
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C))
  {
    Serial.println("Errore display");
    while (true);
  }

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("Display OK");
  display.display();
  delay(1000);
}

void task_display_update(void* pvParameters)
{
  SystemState* state = (SystemState*)pvParameters;

  for (;;)
  {
    display.clearDisplay();
    display.setTextSize(1);

    display.setCursor(0, 0);
    display.print("Temperatura:  ");
    display.print(state->temperature, 1);
    display.println(" C");

    display.setCursor(0, 12);
    display.print("Umidita:      ");
    display.print(state->humidity, 1);
    display.println(" %");

    display.setCursor(0, 24);
    display.print("Qualita aria: ");
    display.print(state->air_quality_pct, 1);
    display.println(" %");

    display.setCursor(0, 48);
    display.setTextSize(2);

    String msg = air_index_to_msg(state->iaq_score, state->iaq_accuracy);
    if (scrolling_message.length() < 10)
      scrolling_message = msg;

    display_scrolling_text(msg);
    display.display();

    vTaskDelay(pdMS_TO_TICKS(500));
  }
}
