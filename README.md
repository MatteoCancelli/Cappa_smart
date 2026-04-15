# Cappa_smart

Progetto per ESP32 che controlla una cappa cucina in modo automatico, leggendo qualità dell'aria, temperatura e umidità tramite BME688 con libreria BSEC, e pubblicando i dati via MQTT.

## Caratteristiche

- Calcolo IAQ (Indoor Air Quality) calibrato tramite libreria BSEC2 di Bosch
- Salvataggio dello stato di calibrazione BSEC in flash NVS - sopravvive ai riavvii
- Controllo ventola PWM con soft-start per evitare brown-out
- Modalità automatica (basata su IAQ, umidità e temperatura) e manuale (potenziometro)
- Protezione boot: MOSFET 2N7000 blocca la ventola finché l'ESP32 non è pronto
- Display OLED con messaggio scorrevole sulla qualità dell'aria
- Rilevamento movimento PIR per illuminazione automatica
- Pubblicazione dati su MQTT
- Architettura a task FreeRTOS con `SystemState` condiviso
- HAL Adapter pattern per astrazione GPIO (testabile su PC)
- Suite di test unitari con Google Test (Logic e Aspiration)

---

## Hardware richiesto

| Componente | Modello / Specifica | Wiring |
|---|---|---|
| Microcontrollore | ESP32-WROOM-32D | - |
| Sensore ambientale | BME688 (compatibile BME680) | SDA → GPIO21, SCL → GPIO22 |
| Sensore movimento | PIR | GPIO15 |
| Display | SSD1306 128x64 I2C | SDA → GPIO21, SCL → GPIO22 |
| Ventola | Noctua NF-P14s redux 1500 PWM - 12V, 2.40W (0.20A) | GPIO32 PWM |
| MOSFET | 2N7000 N-channel - interruttore GND ventola | Drain → GND ventola, Source → GND, Gate → GPIO33 |
| Resistenza pull-down | 10 kΩ | Gate 2N7000 → GND |
| Relay | SRD-05VDC-SL-C | GPIO5 → lampadina 220V |
| Step-Up | MT3608 | 5V → 12V |
| Modulo alimentazione breadboard | Output 3.3V e 5V | - |
| Potenziometro | B10K | GPIO36 (ADC1 - ADC2 condiviso con WiFi) |
| Pulsante | - | GPIO18 |
| LED + Resistenza | LED + 1 kΩ | GPIO19 |
| Condensatore elettrolitico | 4700 µF, 16V - bulk lato 5V (ingresso step-up) | tra 5V e GND, vicino al MT3608 |
| Condensatore elettrolitico | 1000 µF, 25V - bulk lato 12V (uscita step-up) | tra 12V e GND, vicino alla ventola |
| Condensatore elettrolitico | 100 µF, 10V - stabilizzazione rail 3.3V | tra 3.3V e GND, vicino all'uscita del modulo alimentazione |
| Condensatori ceramici bypass | 100 nF, 50V (MLCC "104") - ×3 min, ×5 consigliati | uno in parallelo a ciascun elettrolitico |

---

## Struttura del progetto

```
Cappa_smart/
  Cappa_smart.ino         — setup e avvio task FreeRTOS
  Aspiration.cpp          — controllo ventola, soft-start PWM, pulsante modalità
  BME680.cpp              — lettura BME688 via BSEC, salvataggio stato NVS
  Display.cpp             — display OLED SSD1306
  Globals.cpp             — variabili globali, SystemState, init GPIO
  HalInterface.cpp        — HAL Adapter pattern per GPIO (mockabile nei test)
  Logic.cpp               — logica pura: IAQ → velocità, IAQ → percentuale, IAQ → messaggio
  PIR.cpp                 — task sensore PIR e lampada
  TaskMQTT.cpp            — WiFi, connessione MQTT, pubblicazione dati
  CMakeLists.txt          — root CMake per Google Test
  config.h                — credenziali WiFi e MQTT (NO COMMIT)
  include/
    Aspiration.h
    BME680.h
    Display.h
    Globals.h
    HalInterface.h
    Logic.h
    PIR.h
    SystemState.h         — struct condivisa tra tutti i task
    TaskMQTT.h
  test/
    test_logic.cpp        — test unitari per Logic (Google Test)
    test_aspiration.cpp   — test unitari per Aspiration con mock HAL
    Globals_test.h/cpp    — stub FreeRTOS/Arduino per compilazione su PC
    CMakeLists.txt        — build dei test su PC
```

---

## Topic MQTT

| Topic | Contenuto |
|---|---|
| `home/bme680/temperature` | Temperatura °C |
| `home/bme680/humidity` | Umidità % |
| `home/bme680/air_quality_pct` | Qualità aria 0–100% (scala non lineare) |
| `home/bme680/iaq_score` | IAQ grezzo BSEC 0–500 |
| `home/bme680/iaq_accuracy` | Accuratezza BSEC 0–3 |
| `home/bme680/air_quality_msg` | Messaggio testuale qualità aria |
| `home/fan/mode` | `manual` / `auto` |
| `home/fan/speed` | Velocità ventola 0–255 |
| `home/pir/motion` | `detected` / `clear` |

---

## Scala qualità dell'aria

La percentuale è calcolata con una mappatura non lineare basata sulla scala IAQ ufficiale Bosch:

| IAQ score | Percentuale | Messaggio |
|---|---|---|
| 0–50 | 100–95% | Aria pulita |
| 51–100 | 95–75% | Aria ok |
| 101–150 | 75–50% | Odori rilevati |
| 151–200 | 50–30% | Aria stantia |
| 201–350 | 30–10% | Apri finestre |
| 351–500 | 10–0% | Ventila subito |

Con `iaq_accuracy < 2` il messaggio mostra "Calibrazione..." e la ventola automatica resta ferma.

---

## Logica automatica ventola

| Condizione | Velocità |
|---|---|
| `iaq_accuracy < 2` | 0 (aspetta calibrazione) |
| `humidity > 75%` oppure `temperature > 50°C` | 180 |
| `iaq_score <= 100` | 0 |
| `iaq_score <= 200` | 180 |
| `iaq_score > 200` | 255 |

---

## Dipendenze Arduino IDE

- **BSEC** - Bosch Sensortec (Library Manager)
- **Adafruit SSD1306**
- **Adafruit GFX Library**
- **PubSubClient** - Nick O'Leary
- **ArduinoJson** - Benoit Blanchon
- **Board package**: esp32 by Espressif Systems 3.3.7

---

## Configurazione

Crea il file `config.h` nella cartella del progetto:

```cpp
#pragma once

#define WIFI_SSID     "tuo_ssid"
#define WIFI_PASSWORD "tua_password"

#define MQTT_SERVER   "192.168.x.x"
#define MQTT_PORT     1883
#define MQTT_CLIENT_ID "cappa_smart"
#define MQTT_USER     ""
#define MQTT_PASSWORD ""
```

---

## Test unitari

I test girano sul PC con Google Test e non vengono caricati sull'ESP32.

**Prerequisiti**: CMake, Visual Studio Build Tools con C++

```bash
mkdir build && cd build
cmake .. -G "Visual Studio 18 2026"
cmake --build .
.\test\Debug\test_logic.exe
```

Copertura attuale:

- `Logic`: `iaq_to_percentage`, `air_index_to_msg`, `calcola_velocita_automatica` (inclusi casi bordo accuratezza)
- `Aspiration`: `set_fan_speed` con mock HAL (PWM, semaforo, potenziometro)

---

## Note hardware

- **Brown-out**: risolto con soft-start PWM (rampa 5 step ogni 30ms) + condensatori di bulk + MOSFET 2N7000 sul GND ventola
- **Boot ventola**: il 2N7000 con pull-down 10kΩ sul Gate mantiene la ventola spenta durante il boot; `GPIO_FAN_ENABLE` viene portato HIGH solo a fine setup
- **BSEC calibrazione**: richiede ~5 minuti per accuratezza 2, alcune ore per accuratezza 3; lo stato viene salvato in NVS ogni 15 minuti circa
- **ADC**: usare solo pin ADC1 (GPIO32–39) con WiFi attivo; ADC2 è condiviso con il modulo WiFi
- **Alimentazione**: testato con alimentatore USB 5V 2.4A; i condensatori stabilizzano i picchi di corrente durante l'accelerazione della ventola

---

## Licenza

MIT License - libero per uso personale e comunitario, attribuzione gradita.