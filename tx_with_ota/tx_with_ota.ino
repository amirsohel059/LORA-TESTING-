#include <SPI.h>
#include <LoRa.h>
#include <ESP8266WiFi.h>
#include <EEPROM.h>
#include "ota_helper.h"

// ================== NodeMCU ↔ RFM96 wiring ==================
// Hardware SPI on ESP8266:
// SCK  = D5 (GPIO14)
// MISO = D6 (GPIO12)
// MOSI = D7 (GPIO13)

static const int PIN_LORA_SS   = D8;  // GPIO15
static const int PIN_LORA_RST  = D1;  // GPIO5
static const int PIN_LORA_DIO0 = D2;  // GPIO4

// Existing button kept only to preserve structure (not used)
static const int PIN_BUTTON    = D3;  // GPIO0

// ================== OTA control ==================
// Safe manual OTA trigger button:
// Connect push button between GPIO3 (RX) and GND
static const int PIN_OTA_BUTTON = 3;   // GPIO3 / RX

// Built-in LED on NodeMCU is usually active LOW
static const int PIN_STATUS_LED = LED_BUILTIN;

// ================== WiFi for OTA ==================
const char* WIFI_SSID = "red";
const char* WIFI_PASS = "amir@1212";
const char* OTA_HOSTNAME = "DCI_REZLER";
const char* OTA_PASSWORD = "1234";

// ================== EEPROM ==================
static const int EEPROM_SIZE      = 8;
static const int EEPROM_SEQ_ADDR  = 0;

// ================== TX behavior ==================
static const uint8_t  TX_REPEATS        = 4;
static const uint16_t TX_REPEAT_GAP_MS  = 120;

// Persist sequence across reset/deep sleep wake
static uint32_t seq = 0;

// Kept only to preserve structure (not used)
static uint32_t lastPressMs = 0;

// ---------- EEPROM helpers ----------
static uint32_t loadSeq() {
  uint32_t value = 0;
  EEPROM.get(EEPROM_SEQ_ADDR, value);

  if (value == 0xFFFFFFFFUL) {
    value = 0;
  }
  return value;
}

static void saveSeq(uint32_t value) {
  EEPROM.put(EEPROM_SEQ_ADDR, value);
  EEPROM.commit();
}

// ---------- Radio send ----------
static bool sendOnePacket(const char* msg) {
  LoRa.beginPacket();
  LoRa.print(msg);
  int ret = LoRa.endPacket();
  return (ret == 1);
}

static void sendBlinkCommand() {
  seq++;
  saveSeq(seq);

  char msg[32];
  snprintf(msg, sizeof(msg), "BLINK,%lu", (unsigned long)seq);

  Serial.print("Prepared: ");
  Serial.println(msg);

  for (uint8_t i = 0; i < TX_REPEATS; i++) {
    bool ok = sendOnePacket(msg);

    Serial.print("TX ");
    Serial.print(i + 1);
    Serial.print("/");
    Serial.print(TX_REPEATS);
    Serial.print(" -> ");
    Serial.println(ok ? "OK" : "FAIL");

    delay(TX_REPEAT_GAP_MS);
  }

  Serial.print("Sent final: ");
  Serial.println(msg);
}

// ---------- Normal LoRa operation ----------
static void runNormalOperation() {
  // Power saving: disable WiFi fully
  WiFi.persistent(false);
  WiFi.mode(WIFI_OFF);
  WiFi.forceSleepBegin();
  delay(1);

  EEPROM.begin(EEPROM_SIZE);
  seq = loadSeq();

  Serial.println();
  Serial.println("Starting LoRa TX (RFM96W 433MHz)...");
  Serial.print("Last stored seq = ");
  Serial.println(seq);

  LoRa.setPins(PIN_LORA_SS, PIN_LORA_RST, PIN_LORA_DIO0);

  if (!LoRa.begin(433E6)) {
    Serial.println("LoRa begin failed! Check wiring / power / frequency.");
    delay(100);
    ESP.deepSleep(0);
  }

  // Must match receiver
  LoRa.enableCrc();
  LoRa.setSpreadingFactor(12);
  LoRa.setSignalBandwidth(62.5E3);
  LoRa.setCodingRate4(5);
  LoRa.setPreambleLength(12);
  LoRa.setSyncWord(0x12);
  LoRa.setTxPower(20);

  Serial.println("Radio configured.");
  Serial.println("Sending command now...");

  sendBlinkCommand();

  Serial.println("Going to deep sleep now. Wake again by RESET button.");
  delay(200);

  ESP.deepSleep(0);
}

void setup() {
  Serial.begin(115200);
  delay(200);

  otaInitPins(PIN_STATUS_LED, PIN_OTA_BUTTON);

  // Existing structure preserved
  pinMode(PIN_BUTTON, INPUT_PULLUP);

  if (shouldEnterOtaMode()) {
    startOtaMode(WIFI_SSID, WIFI_PASS, OTA_HOSTNAME, OTA_PASSWORD);
    return;
  }

  runNormalOperation();
}

void loop() {
  if (isOtaModeActive()) {
    handleOtaMode();
  }
}