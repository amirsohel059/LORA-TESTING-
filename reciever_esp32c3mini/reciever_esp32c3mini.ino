#include <SPI.h>
#include <LoRa.h>

// ===== ESP32-C3 Super Mini pin mapping =====
static const int PIN_LORA_SCK  = 4;   // GPIO4
static const int PIN_LORA_MISO = 5;   // GPIO5
static const int PIN_LORA_MOSI = 6;   // GPIO6
static const int PIN_LORA_SS   = 7;   // GPIO7

static const int PIN_LORA_RST  = 3;   // GPIO3
static const int PIN_LORA_DIO0 = 2;   // GPIO2

static const int PIN_RELAY     = 21;  // GPIO21

// ===== Relay logic =====
static const uint8_t RELAY_ON_LEVEL  = HIGH;
static const uint8_t RELAY_OFF_LEVEL = LOW;

// Relay ON time after valid command
static const uint32_t RELAY_ON_MS = 2000;
static uint32_t relayOffAtMs = 0;

// ================== DCI protocol ==================
static const uint8_t DCI_MAGIC1  = 0x44;   // 'D'
static const uint8_t DCI_MAGIC2  = 0x43;   // 'C'
static const uint8_t DCI_VERSION = 0x01;

// IMPORTANT:
// Must match the paired transmitter only.
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

// Duplicate / old packet protection
static uint32_t lastSeq = 0;
static bool haveLastSeq = false;

// ---------- Relay helpers ----------
static void relayOff() {
  digitalWrite(PIN_RELAY, RELAY_OFF_LEVEL);
}

static void relayOn() {
  digitalWrite(PIN_RELAY, RELAY_ON_LEVEL);
}

// ---------- Packet validation ----------
static bool isPacketValidBasic(const DciPacketV1& pkt) {
  if (pkt.magic1 != DCI_MAGIC1 || pkt.magic2 != DCI_MAGIC2) {
    Serial.println("Rejected: bad magic");
    return false;
  }

  if (pkt.version != DCI_VERSION) {
    Serial.print("Rejected: unsupported version ");
    Serial.println(pkt.version);
    return false;
  }

  if (pkt.pairId != DCI_PAIR_ID) {
    Serial.print("Rejected: pairId mismatch. RX expects 0x");
    Serial.print(DCI_PAIR_ID, HEX);
    Serial.print(", got 0x");
    Serial.println(pkt.pairId, HEX);
    return false;
  }

  if (pkt.cmd != DCI_CMD_TRIGGER) {
    Serial.print("Rejected: unsupported cmd ");
    Serial.println(pkt.cmd);
    return false;
  }

  return true;
}

static bool isPacketNew(uint32_t seq) {
  if (!haveLastSeq) {
    return true;
  }

  // Accept only newer sequence number
  if (seq <= lastSeq) {
    Serial.print("Rejected: duplicate/old seq ");
    Serial.print(seq);
    Serial.print(" (last=");
    Serial.print(lastSeq);
    Serial.println(")");
    return false;
  }

  return true;
}

static void printPacket(const DciPacketV1& pkt, int rssi, float snr) {
  Serial.print("Valid packet: ");
  Serial.print("pairId=0x");
  Serial.print(pkt.pairId, HEX);
  Serial.print(" seq=");
  Serial.print(pkt.seq);
  Serial.print(" cmd=");
  Serial.print(pkt.cmd);
  Serial.print(" flags=0x");
  Serial.print(pkt.flags, HEX);
  Serial.print(" reserved=0x");
  Serial.print(pkt.reserved, HEX);
  Serial.print(" | RSSI=");
  Serial.print(rssi);
  Serial.print(" | SNR=");
  Serial.println(snr);
}

static void handlePacket(const DciPacketV1& pkt, int rssi, float snr) {
  if (!isPacketValidBasic(pkt)) {
    return;
  }

  if (!isPacketNew(pkt.seq)) {
    return;
  }

  lastSeq = pkt.seq;
  haveLastSeq = true;

  printPacket(pkt, rssi, snr);

  relayOn();
  relayOffAtMs = millis() + RELAY_ON_MS;

  Serial.println("RELAY ON");
}

void setup() {
  Serial.begin(115200);
  delay(200);

  pinMode(PIN_RELAY, OUTPUT);
  relayOff();

  Serial.println();
  Serial.println("LoRa RX Relay (RFM96W 433MHz) starting...");
  Serial.print("DCI pair ID = 0x");
  Serial.println(DCI_PAIR_ID, HEX);

  SPI.begin(PIN_LORA_SCK, PIN_LORA_MISO, PIN_LORA_MOSI, PIN_LORA_SS);

  LoRa.setPins(PIN_LORA_SS, PIN_LORA_RST, PIN_LORA_DIO0);

  if (!LoRa.begin(433E6)) {
    Serial.println("LoRa begin failed! Check wiring / power / frequency.");
    while (true) {
      delay(1000);
    }
  }

  LoRa.enableCrc();
  LoRa.setSpreadingFactor(12);
  LoRa.setSignalBandwidth(62.5E3);
  LoRa.setCodingRate4(5);
  LoRa.setPreambleLength(12);
  LoRa.setSyncWord(0x12);

  Serial.println("LoRa RX ready. Waiting for packets...");
  Serial.println("Relay default state: OFF");
}

void loop() {
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    Serial.print("Packet received, size=");
    Serial.println(packetSize);

    if (packetSize != (int)sizeof(DciPacketV1)) {
      Serial.print("Rejected: unexpected packet size ");
      Serial.print(packetSize);
      Serial.print(", expected ");
      Serial.println(sizeof(DciPacketV1));

      while (LoRa.available()) {
        LoRa.read();
      }
      return;
    }

    DciPacketV1 pkt;
    uint8_t* p = (uint8_t*)&pkt;
    size_t idx = 0;

    while (LoRa.available() && idx < sizeof(DciPacketV1)) {
      p[idx++] = (uint8_t)LoRa.read();
    }

    if (idx != sizeof(DciPacketV1)) {
      Serial.println("Rejected: incomplete packet read");
      return;
    }

    handlePacket(pkt, LoRa.packetRssi(), LoRa.packetSnr());
  }

  if (relayOffAtMs != 0 && (int32_t)(millis() - relayOffAtMs) >= 0) {
    relayOff();
    relayOffAtMs = 0;
    Serial.println("RELAY OFF");
  }
}