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
static const uint8_t  TX_REPEATS        = 4;
static const uint16_t TX_REPEAT_GAP_MS  = 120;

// Persist sequence across reset/deep sleep wake
static uint32_t seq = 0;

// ================== DCI protocol ==================
static const uint8_t DCI_MAGIC1  = 0x44;   // 'D'
static const uint8_t DCI_MAGIC2  = 0x43;   // 'C'
static const uint8_t DCI_VERSION = 0x01;

// IMPORTANT:
// Give each transmitter/receiver set a unique pair ID.
// Matching transmitter + receiver must use the same pair ID.
static const uint8_t DCI_PAIR_ID = 0x11;

static const uint8_t DCI_CMD_TRIGGER   = 0x01;
static const uint8_t DCI_CMD_HEARTBEAT = 0x02;
static const uint8_t DCI_CMD_ACK       = 0x03;

struct __attribute__((packed)) DciPacketV1 {
  uint8_t  magic1;
  uint8_t  magic2;
  uint8_t  version;
  uint8_t  pairId;
  uint32_t seq;
  uint8_t  cmd;
  uint8_t  flags;
  uint16_t reserved;
};

static_assert(sizeof(DciPacketV1) == 12, "Unexpected DciPacketV1 size");

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
  int ret = LoRa.endPacket();   // blocking send
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
  pkt.magic1   = DCI_MAGIC1;
  pkt.magic2   = DCI_MAGIC2;
  pkt.version  = DCI_VERSION;
  pkt.pairId   = DCI_PAIR_ID;
  pkt.seq      = seq;
  pkt.cmd      = DCI_CMD_TRIGGER;
  pkt.flags    = 0;
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

    delay(TX_REPEAT_GAP_MS);
  }

  Serial.println("Trigger packet send complete.");
}

void setup() {
  // Power saving: disable WiFi fully
  WiFi.persistent(false);
  WiFi.mode(WIFI_OFF);
  WiFi.forceSleepBegin();
  delay(1);

  Serial.begin(115200);
  delay(200);

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
  }

  // MUST match receiver exactly
  LoRa.enableCrc();
  LoRa.setSpreadingFactor(12);
  LoRa.setSignalBandwidth(62.5E3);
  LoRa.setCodingRate4(5);
  LoRa.setPreambleLength(12);
  LoRa.setSyncWord(0x12);
  LoRa.setTxPower(20);

  Serial.println("Radio configured.");
  Serial.println("Sending command now...");

  sendTriggerCommand();

  Serial.println("Going to deep sleep now. Wake again by RESET button.");
  delay(200);

  ESP.deepSleep(0);
}

void loop() {
  // Not used
}