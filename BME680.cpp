#include "include/Globals.h"
#include "include/BME680.h"
#include "include/Logic.h"
#include <bsec.h>
#include <Preferences.h>

Bsec bsec_sensor;
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
    bsec_sensor.setState(state);
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
  bsec_sensor.getState(state);
  prefs.begin("bsec", false);
  prefs.putBytes(BSEC_STATE_KEY, state, BSEC_MAX_STATE_BLOB_SIZE);
  prefs.end();
  Serial.println("Stato BSEC salvato in flash");
}

void init_environment_sensor(SystemState* state)
{
  bsec_sensor.begin(BME68X_I2C_ADDR_LOW, Wire);

  if (bsec_sensor.bsecStatus != BSEC_OK)
  {
    Serial.println("Errore: BME688 non trovato!");
    bme_error_msg();
    while (true);
  }

  bsec_virtual_sensor_t sensor_list[] = {
    BSEC_OUTPUT_IAQ,
    BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE,
    BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY
  };

  bsec_sensor.updateSubscription(sensor_list, 3, BSEC_SAMPLE_RATE_LP);

  load_bsec_state();

  Serial.println("BME688 + BSEC OK!");
  bme_ok_msg();
  delay(1000);
}

void task_environment_sensor(void* pvParameters)
{
  SystemState* state = (SystemState*)pvParameters;
  uint32_t save_counter = 0;
  TickType_t last_wake_time = xTaskGetTickCount();

  for (;;)
  {
    if (bsec_sensor.run())
    {
      state->temperature     = bsec_sensor.temperature;
      state->humidity        = bsec_sensor.humidity;
      state->iaq_score       = bsec_sensor.iaq;
      state->iaq_accuracy    = bsec_sensor.iaqAccuracy;
      state->air_quality_pct = iaq_to_percentage(bsec_sensor.iaq);

      save_counter++;
      if (save_counter >= 300)
      {
        save_bsec_state();
        save_counter = 0;
      }
    }
    else if (bsec_sensor.bsecStatus != BSEC_OK || bsec_sensor.bme68xStatus != BME68X_OK)
    {
      Serial.printf("Errore BSEC: %d BME: %d\n", bsec_sensor.bsecStatus, bsec_sensor.bme68xStatus);
      bme_fail_msg();
    }
    vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(100));
  }
}
