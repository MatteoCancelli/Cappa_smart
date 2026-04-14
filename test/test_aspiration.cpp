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
    state = { 0, 0, 0, 0, false, false, 0, nullptr };
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
