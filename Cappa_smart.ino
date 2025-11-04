#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_BME680.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// BME688 a indirizzo 0x76 (o 0x77 se hai collegato ADDR a VCC)
Adafruit_BME680 bme;

// Funzione per convertire resistenza gas in Air Quality Index (0-100)
float gas_to_AirQualityIndex(double gasOhm) {

  const double GAS_MIN = 10000.0;   // 10 kΩ = peggior aria
  const double GAS_MAX = 120000.0;  // 120 kΩ = aria pulita

  if (gasOhm < GAS_MIN) gasOhm = GAS_MIN;
  if (gasOhm > GAS_MAX) gasOhm = GAS_MAX;

  int aqi = (int)((gasOhm - GAS_MIN) / (GAS_MAX - GAS_MIN) * 100);
  return aqi;
}

void setup() {
  Serial.begin(115200);
  delay(500);

  // Avvio I2C
  Wire.begin(); // usa i pin di default dell’ESP32: SDA=21, SCL=22
  Wire.setClock(100000);

  Serial.println("=== Avvio Display ===");
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("❌ Errore display");
    while (true);
  }
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("Display OK");
  display.display();

  delay(1000);

  Serial.println("=== Avvio BME688 ===");
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("Inizializzo BME...");
  display.display();

  if (!bme.begin(0x76)) {
    Serial.println("❌ Errore: BME688 non trovato!");
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("ERRORE BME688");
    display.setCursor(0, 16);
    display.println("Controlla I2C!");
    display.display();
    while (true);
  }

  Serial.println("✅ BME688 trovato!");
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("BME688 OK!");
  display.display();
  delay(1000);

  // Impostazioni base
  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setGasHeater(300, 150);
}

void loop() {
  if (!bme.performReading()) {
    Serial.println("⚠️ Lettura fallita!");
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("Lettura fallita!");
    display.display();
    delay(1000);
    return;
  }

  float t = bme.temperature;
  float h = bme.humidity;
  float g = gas_to_AirQualityIndex(bme.gas_resistance);

  Serial.print("Temperatura:  "); Serial.print(t); Serial.println(" °C");
  Serial.print("Umidità:      "); Serial.print(h); Serial.println(" %");
  Serial.print("Qualità aria: "); Serial.print(g); Serial.println(" %");
  Serial.println("------------------");

  display.clearDisplay();
  display.setCursor(0, 0);
  display.print("Temperatura:  "); display.print(t, 2); display.println(" C");
  display.setCursor(0, 10);
  display.print("Umidita:      "); display.print(h, 2); display.println(" %");
  display.setCursor(0, 20);
  display.print("Qualita aria: "); display.print(g, 2); display.println(" %");
  display.display();

  delay(2000);
}
