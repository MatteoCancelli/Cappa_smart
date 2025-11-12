#include "include/Globals.h"
#include "include/BME680.h"
#include "include/Display.h"
#include <Adafruit_BME680.h>

Adafruit_BME680 bme;
float temp = 0;
float hum = 0;
float gas_index = 0;

void check_bme(){
  if (!bme.begin(0x76)) {
    Serial.println("Errore: BME688 non trovato!");
    bme_error_msg();
    while (true);
  }
  Serial.println("BME688 trovato!");
  bme_ok_msg();
  delay(1000);
}

float gas_to_AirQualityIndex(double gas_ohm) {
  const double GAS_MIN = 10000.0;   // 10 kΩ = aria pessima
  const double GAS_MAX = 120000.0;  // 120 kΩ = aria pulita

  if (gas_ohm < GAS_MIN) gas_ohm = GAS_MIN;
  if (gas_ohm > GAS_MAX) gas_ohm = GAS_MAX;

  float aqi = (gas_ohm - GAS_MIN) / (GAS_MAX - GAS_MIN) * 100;
  return aqi;
}

String air_index_to_msg(float quality_index) {
  if      (quality_index < 20)  return "Apri tutto";     // pessima
  else if (quality_index < 40)  return "Aria stantia";   // scarsa
  else if (quality_index < 60)  return "Aria viziata";   // media
  else if (quality_index < 80)  return "Aria normale";   // buona
  else                          return "Aria fresca";    // ottima
}

void task_bme(void *pvParameters){
  for(;;) {
    if (!bme.performReading()) {
      Serial.println("Lettura fallita!");
      bme_fail_msg();
      vTaskDelay(pdMS_TO_TICKS(1000));
    }
    else {
      temp = bme.temperature;
      hum  = bme.humidity;
      gas_index = gas_to_AirQualityIndex(bme.gas_resistance);
    }
    vTaskDelay(pdMS_TO_TICKS(2000)); 
  }
}
