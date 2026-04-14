#include <gtest/gtest.h>
#include "Globals_test.h"
#include "include/Aspiration.h"

static uint8_t last_ledc_pin = 0;
static uint32_t last_ledc_duty = 0;
static int mock_digital_read_value = HIGH;
static uint8_t last_digital_write_pin = 0;
static uint8_t last_digital_write_val = 0;
static int mock_analog_read_value = 0;

static void mock_ledcWrite(uint8_t pin, uint32_t duty) {
  last_ledc_pin = pin;
  last_ledc_duty = duty;
}

static int mock_digitalRead(uint8_t pin) {
  return mock_digital_read_value;
}

static void mock_digitalWrite(uint8_t pin, uint8_t val) {
  last_digital_write_pin = pin;
  last_digital_write_val = val;
}

static int mock_analogRead(uint8_t pin) {
  return mock_analog_read_value;
}

class AspirationTest : public ::testing::Test {
protected:
  void SetUp() override {
    hal.ledcWrite    = mock_ledcWrite;
    hal.digitalRead  = mock_digitalRead;
    hal.digitalWrite = mock_digitalWrite;
    hal.analogRead   = mock_analogRead;

    last_ledc_pin = 0;
    last_ledc_duty = 0;
    last_digital_write_pin = 0;
    last_digital_write_val = 0;
    target_fan_speed = 0;
    mode_manual = false;
  }
};

TEST_F(AspirationTest, AttuatoreScrivePWM) {
  attuatore_ventola(100);
  EXPECT_EQ(last_ledc_duty, 100);
}

TEST_F(AspirationTest, AttuatoreNonScriveSeVelocitaUguale) {
  target_fan_speed = 100;
  attuatore_ventola(100);
  EXPECT_EQ(last_ledc_duty, 0);
}

TEST_F(AspirationTest, PotenziometroMassimo) {
  mock_analog_read_value = 4095;
  // map(4095, 0, 4095, 0, 255) = 255
  // verifica che attuatore riceva 255
  int pot = mock_analogRead(0);
  int speed = (pot * 255) / 4095;
  EXPECT_EQ(speed, 255);
}

TEST_F(AspirationTest, PotenziometroMinimo) {
  mock_analog_read_value = 0;
  int pot = mock_analogRead(0);
  int speed = (pot * 255) / 4095;
  EXPECT_EQ(speed, 0);
}