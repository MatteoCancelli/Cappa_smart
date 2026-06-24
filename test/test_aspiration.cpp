/*
 * Cappa_smart - ESP32-based smart kitchen hood controller
 * Copyright (C) 2026 Matteo Cancelli
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <https://www.gnu.org/licenses/>.
 */

#include <gtest/gtest.h>
#include "Globals_test.h"
#include "include/Aspiration.h"

static uint8_t last_ledc_duty = 0;
static int mock_analog_value = 0;

static void mock_ledcWrite(uint8_t, uint32_t duty) { last_ledc_duty = duty; }
static int  mock_digitalRead(uint8_t)              { return HIGH; }
static void mock_digitalWrite(uint8_t, uint8_t)    {}
static int  mock_analogRead(uint8_t)               { return mock_analog_value; }

class AspirationTest : public ::testing::Test {
protected:
  SystemState state;
  void SetUp() override {
  hal.ledcWrite    = mock_ledcWrite;
  hal.digitalRead  = mock_digitalRead;
  hal.digitalWrite = mock_digitalWrite;
  hal.analogRead   = mock_analogRead;
  last_ledc_duty   = 0;
  mock_analog_value = 0;
  state.temperature        = 0;
  state.humidity           = 0;
  state.air_quality_pct    = 0;
  state.iaq_score          = 0;
  state.iaq_accuracy       = 0;
  state.is_motion_detected = false;
  state.is_manual_mode     = false;
  state.fan_speed_target   = 0;
  state.fan_mutex          = nullptr;
  }
};

TEST_F(AspirationTest, SetFanSpeedScrivePWM) {
  set_fan_speed(&state, 100);
  EXPECT_EQ(last_ledc_duty, 100);
}

TEST_F(AspirationTest, SetFanSpeedNonScriveSeUguale) {
  state.fan_speed_target = 100;
  set_fan_speed(&state, 100);
  EXPECT_EQ(last_ledc_duty, 0);
}

TEST_F(AspirationTest, SetFanSpeedAggiornaDato) {
  set_fan_speed(&state, 150);
  EXPECT_EQ(state.fan_speed_target, 150);
}

TEST_F(AspirationTest, PotenziometroMassimo) {
  int speed = map(4095, 0, 4095, 0, 255);
  EXPECT_EQ(speed, 255);
}

TEST_F(AspirationTest, PotenziometroMinimo) {
  int speed = map(0, 0, 4095, 0, 255);
  EXPECT_EQ(speed, 0);
}
