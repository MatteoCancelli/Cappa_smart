#include "include/Globals.h"
#include "include/BME680.h"
#include "include/Logic.h"
#include <bsec.h>
#include <Preferences.h>

Bsec bsec;
Preferences prefs;

#define BSEC_STATE_KEY "bsec_state"

static void load_bsec_state()
{
  prefs.begin("bsec", true);
  size_t len = prefs.getBytesLength(BSEC_STATE_KEY);
  if (len == BSEC_MAX_STATE_BLOB_SIZE)
  {
    uint8_t state[BSEC_MAX_STATE_BLOB_SIZE];
    prefs.getBytes(BSEC_STATE_KEY, state, len);
    bsec.setState(state);
    Serial.println("Stato BSEC caricato dalla flash");
  }
  else
  {
    Serial.println("Nessuno stato BSEC salvato, calibrazione da zero");
  }
  prefs.end();
}

static void save_bsec_state()
{
  uint8_t state[BSEC_MAX_STATE_BLOB_SIZE];
  bsec.getState(state);
  prefs.begin("bsec", false);
  prefs.putBytes(BSEC_STATE_KEY, state, BSEC_MAX_STATE_BLOB_SIZE);
  prefs.end();
  Serial.println("Stato BSEC salvato in flash");
}

void check_bme()
{
  bsec.begin(BME68X_I2C_ADDR_LOW, Wire);

  if (bsec.bsecStatus != BSEC_OK)
  {
    Serial.println("Errore: BME680 non trovato!");
    bme_error_msg();
    while (true);
  }

  bsec_virtual_sensor_t sensor_list[] = {
    BSEC_OUTPUT_IAQ,
    BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE,
    BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY
  };

  bsec.updateSubscription(sensor_list, 3, BSEC_SAMPLE_RATE_LP);

  load_bsec_state();

  Serial.println("BME680 + BSEC OK!");
  bme_ok_msg();
  delay(1000);
}

void task_bme(void *pvParameters)
{
  uint32_t save_counter = 0;
  
  for (;;)
  {
    if (bsec.run())
    {
      temp      = bsec.temperature;
      hum       = bsec.humidity;
      iaq_raw   = bsec.iaq;
      gas_index = iaq_to_percentage(bsec.iaq);

      save_counter++;
      if (save_counter >= 300)
      {
        save_bsec_state();
        save_counter = 0;
      }
    }
    else if (bsec.bsecStatus != BSEC_OK || bsec.bme68xStatus != BME68X_OK)
    {
      Serial.printf("Errore BSEC: %d BME: %d\n", bsec.bsecStatus, bsec.bme68xStatus);
      bme_fail_msg();
    }
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}