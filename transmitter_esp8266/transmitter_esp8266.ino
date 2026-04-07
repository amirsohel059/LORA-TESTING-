#include <SPI.h>
#include <LoRa.h>
#include <ESP8266WiFi.h>
#include <EEPROM.h>
#include <ArduinoOTA.h>

// ================== NodeMCU ↔ RFM96 wiring ==================
// Hardware SPI on ESP8266:
// SCK  = D5 (GPIO14)
// MISO = D6 (GPIO12)
// MOSI = D7 (GPIO13)

static const int PIN_LORA_SS = D8;    // GPIO15
static const int PIN_LORA_RST = D1;   // GPIO5
static const int PIN_LORA_DIO0 = D2;  // GPIO4

// Existing button kept unchanged
static const int PIN_BUTTON = D3;  // GPIO0

// ================== OTA control ==================
// Connect a temporary push button between GPIO3 (RX) and GND
// Hold button during power-up / reset to enter OTA mode
static const int PIN_OTA_BUTTON = 3;            // GPIO3 / RX
static const int PIN_STATUS_LED = LED_BUILTIN;  // usually active LOW on NodeMCU

// ================== WiFi for OTA ==================
const char* WIFI_SSID = "red";
const char* WIFI_PASS = "amir@1212";
const char* OTA_HOSTNAME = "REZLER_DCI";
const char* OTA_PASSWORD = "1234";

// ================== EEPROM ==================
static const int EEPROM_SIZE = 8;
static const int EEPROM_SEQ_ADDR = 0;

// ================== TX behavior ==================
static const uint8_t TX_REPEATS = 4;
static const uint16_t TX_REPEAT_GAP_MS = 120;

// Persist sequence across reset/deep sleep wake
static uint32_t seq = 0;

// ================== DCI protocol ==================
static const uint8_t DCI_MAGIC1 = 0x44;  // 'D'
static const uint8_t DCI_MAGIC2 = 0x43;  // 'C'
static const uint8_t DCI_VERSION = 0x01;

// IMPORTANT:
// Give each transmitter/receiver set a unique pair ID.
// Matching transmitter + receiver must use the same pair ID.
static const uint8_t DCI_PAIR_ID = 0x11;

static const uint8_t DCI_CMD_TRIGGER = 0x01;
static const uint8_t DCI_CMD_HEARTBEAT = 0x02;
static const uint8_t DCI_CMD_ACK = 0x03;

struct __attribute__((packed)) DciPacketV1 {
  uint8_t magic1;
  uint8_t magic2;
  uint8_t version;
  uint8_t pairId;
  uint32_t seq;
  uint8_t cmd;
  uint8_t flags;
  uint16_t reserved;
};

static_assert(sizeof(DciPacketV1) == 12, "Unexpected DciPacketV1 size");

// ================== OTA state ==================
static bool otaModeActive = false;
static uint32_t otaLedLastToggleMs = 0;
static bool otaLedState = false;

// ---------- LED helpers ----------
static void setStatusLed(bool on) {
  // NodeMCU LED is usually active LOW
  digitalWrite(PIN_STATUS_LED, on ? LOW : HIGH);
}

static void blinkStatusLedTask(uint32_t intervalMs) {
  uint32_t now = millis();
  if (now - otaLedLastToggleMs >= intervalMs) {
    otaLedLastToggleMs = now;
    otaLedState = !otaLedState;
    setStatusLed(otaLedState);
  }
}

// ---------- OTA helpers ----------
static void initOtaPins() {
  pinMode(PIN_STATUS_LED, OUTPUT);
  setStatusLed(false);

  pinMode(PIN_OTA_BUTTON, INPUT_PULLUP);
}

static bool shouldEnterOtaMode() {
  // Button pressed = LOW
  if (digitalRead(PIN_OTA_BUTTON) != LOW) {
    return false;
  }

  // Small hold check to avoid accidental entry
  uint32_t startMs = millis();
  while (millis() - startMs < 1200) {
    if (digitalRead(PIN_OTA_BUTTON) != LOW) {
      return false;
    }
    blinkStatusLedTask(120);
    delay(10);
  }

  setStatusLed(false);
  return true;
}

static void startOtaMode() {
  otaModeActive = true;

  Serial.println();
  Serial.println("OTA trigger detected.");
  Serial.println("Entering OTA mode...");

  WiFi.persistent(false);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  Serial.print("Connecting to WiFi");

  uint32_t wifiStartMs = millis();
  while (WiFi.status() != WL_CONNECTED) {
    blinkStatusLedTask(150);
    Serial.print(".");
    delay(250);

    // keep trying, but print a status every ~10 sec
    if (millis() - wifiStartMs > 10000) {
      Serial.println();
      Serial.println("Still trying to connect to WiFi...");
      wifiStartMs = millis();
    }
  }

  Serial.println();
  Serial.println("WiFi connected.");
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());

  ArduinoOTA.setHostname(OTA_HOSTNAME);

  if (OTA_PASSWORD != nullptr && OTA_PASSWORD[0] != '\0') {
    ArduinoOTA.setPassword(OTA_PASSWORD);
  }

  ArduinoOTA.onStart([]() {
    setStatusLed(true);
    Serial.println("OTA update started.");
  });

  ArduinoOTA.onEnd([]() {
    setStatusLed(false);
    Serial.println();
    Serial.println("OTA update finished.");
  });

  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    static uint32_t lastPrintMs = 0;

    // blink during OTA transfer too
    blinkStatusLedTask(80);

    uint32_t now = millis();
    if (now - lastPrintMs >= 1000) {
      lastPrintMs = now;

      unsigned int percent = 0;
      if (total > 0) {
        percent = (progress * 100U) / total;
      }

      Serial.print("OTA progress: ");
      Serial.print(percent);
      Serial.println("%");
    }
  });

  ArduinoOTA.onError([](ota_error_t error) {
    setStatusLed(false);

    Serial.print("OTA error: ");
    Serial.println((int)error);

    if (error == OTA_AUTH_ERROR) {
      Serial.println("Auth failed");
    } else if (error == OTA_BEGIN_ERROR) {
      Serial.println("Begin failed");
    } else if (error == OTA_CONNECT_ERROR) {
      Serial.println("Connect failed");
    } else if (error == OTA_RECEIVE_ERROR) {
      Serial.println("Receive failed");
    } else if (error == OTA_END_ERROR) {
      Serial.println("End failed");
    }
  });

  ArduinoOTA.begin();

  Serial.println("OTA ready.");
  Serial.print("Hostname: ");
  Serial.println(OTA_HOSTNAME);
  Serial.println("LED will blink while OTA mode is active.");
}

static void handleOtaMode() {
  ArduinoOTA.handle();
  blinkStatusLedTask(300);
  yield();
}

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

// ---------- Packet send ----------
static bool sendOnePacket(const DciPacketV1& pkt) {
  LoRa.beginPacket();
  size_t written = LoRa.write((const uint8_t*)&pkt, sizeof(pkt));
  int ret = LoRa.endPacket();  // blocking send
  return (written == sizeof(pkt) && ret == 1);
}

static void printPacket(const DciPacketV1& pkt) {
  Serial.print("Packet: ");
  Serial.print("pairId=0x");
  Serial.print(pkt.pairId, HEX);
  Serial.print(" seq=");
  Serial.print(pkt.seq);
  Serial.print(" cmd=");
  Serial.print(pkt.cmd);
  Serial.print(" flags=0x");
  Serial.print(pkt.flags, HEX);
  Serial.print(" reserved=0x");
  Serial.println(pkt.reserved, HEX);
}

static void sendTriggerCommand() {
  seq++;
  saveSeq(seq);

  DciPacketV1 pkt;
  pkt.magic1 = DCI_MAGIC1;
  pkt.magic2 = DCI_MAGIC2;
  pkt.version = DCI_VERSION;
  pkt.pairId = DCI_PAIR_ID;
  pkt.seq = seq;
  pkt.cmd = DCI_CMD_TRIGGER;
  pkt.flags = 0;
  pkt.reserved = 0;

  printPacket(pkt);

  for (uint8_t i = 0; i < TX_REPEATS; i++) {
    bool ok = sendOnePacket(pkt);

    Serial.print("TX ");
    Serial.print(i + 1);
    Serial.print("/");
    Serial.print(TX_REPEATS);
    Serial.print(" -> ");
    Serial.println(ok ? "OK" : "FAIL");

    if (i + 1 < TX_REPEATS) {
      delay(TX_REPEAT_GAP_MS);
    }
  }

  Serial.println("Trigger packet send complete.");
}

// ---------- Normal LoRa operation ----------
static void runNormalOperation() {
  // Power saving: disable WiFi fully
  WiFi.persistent(false);
  WiFi.mode(WIFI_OFF);
  WiFi.forceSleepBegin();
  delay(1);

  pinMode(PIN_BUTTON, INPUT_PULLUP);

  EEPROM.begin(EEPROM_SIZE);
  seq = loadSeq();

  Serial.println();
  Serial.println("Starting LoRa TX (RFM96W 433MHz)...");
  Serial.print("Last stored seq = ");
  Serial.println(seq);
  Serial.print("DCI pair ID = 0x");
  Serial.println(DCI_PAIR_ID, HEX);

  LoRa.setPins(PIN_LORA_SS, PIN_LORA_RST, PIN_LORA_DIO0);

  if (!LoRa.begin(433E6)) {
    Serial.println("LoRa begin failed! Check wiring / power / frequency.");
    delay(100);
    ESP.deepSleep(0);
    return;
  }

  // MUST match receiver exactly
  LoRa.enableCrc();
  LoRa.setSpreadingFactor(7);
  LoRa.setSignalBandwidth(125E3);
  LoRa.setCodingRate4(5);
  LoRa.setPreambleLength(8);
  LoRa.setSyncWord(0x12);
  LoRa.setTxPower(17);

  Serial.println("Radio configured.");
  Serial.println("Sending command now...");

  sendTriggerCommand();

  Serial.println("Going to deep sleep now. Wake again by RESET button.");
  delay(200);

  ESP.deepSleep(0);
}

void setup() {
  Serial.begin(9600);
  delay(100);

  initOtaPins();

  if (shouldEnterOtaMode()) {
    startOtaMode();
    return;
  }

  runNormalOperation();
}

void loop() {
  if (otaModeActive) {
    handleOtaMode();
  }
}