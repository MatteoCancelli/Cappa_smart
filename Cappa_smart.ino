#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_BME680.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define PIR_GPIO 15
#define LED_GPIO 2
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// BME688 a indirizzo 0x76 (o 0x77 se hai collegato ADDR a VCC)
Adafruit_BME680 bme;
float temp, hum, gas_index;
String msg_mod = "";

// Funzione per convertire resistenza gas in Air Quality Index (0-100)
float gas_to_AirQualityIndex(double gas_ohm) {
  const double GAS_MIN = 10000.0;   // 10 kΩ = aria pessima
  const double GAS_MAX = 120000.0;  // 120 kΩ = aria pulita

  if (gas_ohm < GAS_MIN) gas_ohm = GAS_MIN;
  if (gas_ohm > GAS_MAX) gas_ohm = GAS_MAX;

  int aqi = (int)((gas_ohm - GAS_MIN) / (GAS_MAX - GAS_MIN) * 100);
  return aqi;
}

// Funzione che converte AQI (%) in messaggio leggibile
String air_index_to_msg(float gas_index) {
  if      (gas_index < 20) return "Apri tutto";     // pessima
  else if (gas_index < 40) return "Aria stantia";   // scarsa
  else if (gas_index < 60) return "Aria viziata";   // media
  else if (gas_index < 80) return "Aria normale";   // buona
  else                     return "Aria fresca";    // ottima
}

void visualizza_msg_scorrevole(String msg){
  if (msg.length() > 11){
    display.print(msg_mod);
    msg_mod.remove(0,1);
  }
  else{
    display.print(msg);
  }
}

void check_display(){
  Serial.println("=== Avvio Display ===");

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

void check_bme(){
  if (!bme.begin(0x76)) {
    Serial.println("Errore: BME688 non trovato!");
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("ERRORE BME688");
    display.setCursor(0, 16);
    display.println("Controlla I2C!");
    display.display();
    while (true);
  }
  Serial.println("BME688 trovato!");
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("BME688 OK");
  display.display();
  delay(1000);
}

void task_bme(void *pvParameters){
  for(;;) {
    if (!bme.performReading()) {
    Serial.println("Lettura fallita!");
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("Lettura fallita!");
    display.display();
    delay(1000);
    return;
    }
    else {
      temp = bme.temperature;
      hum  = bme.humidity;
      gas_index = gas_to_AirQualityIndex(bme.gas_resistance);
    }
    vTaskDelay(pdMS_TO_TICKS(2000)); 
  }
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

void task_pir(void *pvParameters){
  pinMode(PIR_GPIO, INPUT);
  pinMode(LED_GPIO, OUTPUT);

  for(;;){
    int pir_state = digitalRead(PIR_GPIO);

    if (pir_state == HIGH)
      digitalWrite(LED_GPIO, HIGH);
    else
      digitalWrite(LED_GPIO, LOW);
  }
  vTaskDelay(pdMS_TO_TICKS(200));
}


void setup(){
  Serial.begin(115200);
  Wire.begin(); //I2C
  Wire.setClock(100000);

  check_display();

  check_bme();

  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setGasHeater(300, 150);

  xTaskCreate(task_bme, "BME", 4096, NULL, 1, NULL);
  xTaskCreate(task_display, "Display", 4096, NULL, 1, NULL);
  xTaskCreate(task_pir, "PIR", 4096, NULL, 2, NULL);
  
}

void loop(){
}

