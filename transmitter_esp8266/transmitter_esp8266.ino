#include <SPI.h>
#include <LoRa.h>

// NodeMCU ↔ RFM96 wiring (as per your table)
static const int PIN_LORA_SS   = D8;  // GPIO15
static const int PIN_LORA_RST  = D1;  // GPIO5
static const int PIN_LORA_DIO0 = D2;  // GPIO4

// Button (to GND)
static const int PIN_BUTTON    = D3;  // GPIO0 (INPUT_PULLUP)

static uint16_t seq = 0;
static uint32_t lastPressMs = 0;

static void sendBlinkCommand() {
  seq++;

  char msg[32];
  snprintf(msg, sizeof(msg), "BLINK,%u", (unsigned)seq);

  // Send a few times to reduce miss probability
  const int repeats = 3;
  for (int i = 0; i < repeats; i++) {
    LoRa.beginPacket();
    LoRa.print(msg);
    LoRa.endPacket();     // blocking send
    delay(60);
  }

  Serial.print("Sent: ");
  Serial.println(msg);
}

void setup() {
  Serial.begin(115200);
  delay(200);

  pinMode(PIN_BUTTON, INPUT_PULLUP);

  LoRa.setPins(PIN_LORA_SS, PIN_LORA_RST, PIN_LORA_DIO0);

  Serial.println("Starting LoRa TX (RFM96W 433MHz)...");
  if (!LoRa.begin(433E6)) {               // ✅ FIXED FOR 433 MHz
    Serial.println("LoRa begin failed! Check wiring/power.");
    while (true) delay(1000);
  }

  // Reliability-friendly defaults (optional, but good)
  LoRa.enableCrc();
  LoRa.setSpreadingFactor(10);            // 7..12 (higher = longer range, slower)
  LoRa.setSignalBandwidth(125E3);         // common default
  LoRa.setCodingRate4(5);                 // 4/5

  Serial.println("LoRa TX ready. Press button.");
}

void loop() {
  if (digitalRead(PIN_BUTTON) == LOW) {
    uint32_t now = millis();
    if (now - lastPressMs > 250) {        // debounce
      lastPressMs = now;
      sendBlinkCommand();
    }
    while (digitalRead(PIN_BUTTON) == LOW) delay(10); // wait release
  }
}