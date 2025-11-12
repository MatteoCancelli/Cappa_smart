#include "include/Globals.h"
#include "include/Display.h"
#include <Adafruit_SSD1306.h>

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
String msg_mod = "";

void wifi_try_msg(){
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("Connessione WiFi...");
  display.display();
}

void wifi_ok_msg(IPAddress local_IP){
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("WiFi OK!");
  display.setCursor(0, 16);
  display.print("IP: ");
  display.println(local_IP);
  display.display();
}

void wifi_error_msg(){
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("WiFi ERRORE!");
  display.setCursor(0, 16);
  display.println("Controllo cred.");
  display.display();
}

void bme_error_msg(){
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("ERRORE BME688");
  display.setCursor(0, 16);
  display.println("Controlla I2C!");
  display.display();
}

void bme_ok_msg(){
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("BME688 OK");
  display.display();
}

void visualizza_msg_scorrevole(String msg){
  if (msg.length() >= 10){
    display.print(msg_mod);
    msg_mod.remove(0,1);
  }
  else{
    display.print(msg);
  }
}

void check_display(){
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
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

void task_display(void *pvParameters){
  for(;;) {
    display.clearDisplay();
    display.setCursor(0, 0);
    display.setTextSize(1);
    display.print("Temperatura:  "); display.print(temp, 1); display.println(" C");
    display.setCursor(0, 12);
    display.print("Umidita:      "); display.print(hum, 1); display.println(" %");
    display.setCursor(0, 24);
    display.print("Qualita aria: "); display.print(gas_index, 1); display.println(" %");
    display.setCursor(0, 48);
    display.setTextSize(2);

    String msg = air_index_to_msg(gas_index);
    if (msg_mod.length() < 10)
      msg_mod = msg;  

    visualizza_msg_scorrevole(msg);
    display.display();

    vTaskDelay(pdMS_TO_TICKS(500)); 
  }
}