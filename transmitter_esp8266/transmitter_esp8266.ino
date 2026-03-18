#include <SPI.h>
#include <LoRa.h>
#include <ESP8266WiFi.h>
#include <EEPROM.h>

// ================== NodeMCU ↔ RFM96 wiring ==================
// Hardware SPI on ESP8266:
// SCK  = D5 (GPIO14)
// MISO = D6 (GPIO12)
// MOSI = D7 (GPIO13)

static const int PIN_LORA_SS   = D8;  // GPIO15
static const int PIN_LORA_RST  = D1;  // GPIO5
static const int PIN_LORA_DIO0 = D2;  // GPIO4

// Kept only to preserve structure (not used now)
static const int PIN_BUTTON    = D3;  // GPIO0

// ================== EEPROM ==================
static const int EEPROM_SIZE      = 8;
static const int EEPROM_SEQ_ADDR  = 0;

// ================== TX behavior ==================
static const uint8_t  TX_REPEATS        = 4;     // send multiple times for reliability
static const uint16_t TX_REPEAT_GAP_MS  = 120;   // gap between repeated packets

// Persist sequence across reset/deep sleep wake
static uint32_t seq = 0;

// Kept only to preserve structure (not used)
static uint32_t lastPressMs = 0;

// ---------- EEPROM helpers ----------
static uint32_t loadSeq() {
  uint32_t value = 0;
  EEPROM.get(EEPROM_SEQ_ADDR, value);

  // Basic sanity check
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
  int ret = LoRa.endPacket();   // blocking send
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

void setup() {
  // Power saving: disable WiFi fully
  WiFi.persistent(false);
  WiFi.mode(WIFI_OFF);
  WiFi.forceSleepBegin();
  delay(1);

  Serial.begin(115200);
  delay(200);

  // Kept only to preserve structure
  pinMode(PIN_BUTTON, INPUT_PULLUP);

  EEPROM.begin(EEPROM_SIZE);
  seq = loadSeq();

  Serial.println();
  Serial.println("Starting LoRa TX (RFM96W 433MHz)...");
  Serial.print("Last stored seq = ");
  Serial.println(seq);

  // Set LoRa control pins
  LoRa.setPins(PIN_LORA_SS, PIN_LORA_RST, PIN_LORA_DIO0);

  if (!LoRa.begin(433E6)) {
    Serial.println("LoRa begin failed! Check wiring / power / frequency.");
    delay(100);
    ESP.deepSleep(0);   // sleep forever until reset
  }

  // ========== Long-range settings ==========
  // MUST match the receiver exactly
  LoRa.enableCrc();
  LoRa.setSpreadingFactor(12);        // longest range, slowest data rate
  LoRa.setSignalBandwidth(62.5E3);    // narrower bandwidth = better sensitivity
  LoRa.setCodingRate4(5);             // keep compatible/simple
  LoRa.setPreambleLength(12);
  LoRa.setSyncWord(0x12);

  // Increase TX power
  // RFM96W / SX1276 usually supports high power on PA_BOOST
  LoRa.setTxPower(20);

  Serial.println("Radio configured.");
  Serial.println("Sending command now...");

  // Wake by RESET -> send immediately -> sleep again
  sendBlinkCommand();

  Serial.println("Going to deep sleep now. Wake again by RESET button.");
  delay(200);   // allow UART + radio to fully finish

  // Sleep forever until reset/wake event
  ESP.deepSleep(0);
}

void loop() {
  // Not used
}