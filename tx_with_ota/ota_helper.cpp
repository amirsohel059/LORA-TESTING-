#include "ota_helper.h"
#include <ESP8266WiFi.h>
#include <ArduinoOTA.h>

static int g_statusLedPin = LED_BUILTIN;
static int g_otaButtonPin = 3;

static bool g_otaMode = false;
static unsigned long g_lastLedToggleMs = 0;
static bool g_ledState = false;
static uint8_t g_lastOtaPercent = 255;

// ---------- LED helpers ----------
static void ledOn() {
  digitalWrite(g_statusLedPin, LOW);   // active LOW
}

static void ledOff() {
  digitalWrite(g_statusLedPin, HIGH);  // active LOW
}

static void blinkLedInOtaMode() {
  unsigned long now = millis();
  if (now - g_lastLedToggleMs >= 250) {
    g_lastLedToggleMs = now;
    g_ledState = !g_ledState;
    digitalWrite(g_statusLedPin, g_ledState ? LOW : HIGH);
  }
}

void otaInitPins(int statusLedPin, int otaButtonPin) {
  g_statusLedPin = statusLedPin;
  g_otaButtonPin = otaButtonPin;

  pinMode(g_statusLedPin, OUTPUT);
  ledOff();

  pinMode(g_otaButtonPin, INPUT_PULLUP);
}

bool shouldEnterOtaMode() {
  delay(20);  // small settle time

  // Button pressed -> pin reads LOW
  if (digitalRead(g_otaButtonPin) == LOW) {
    delay(30);
    if (digitalRead(g_otaButtonPin) == LOW) {
      return true;
    }
  }
  return false;
}

void startOtaMode(const char* wifiSsid,
                  const char* wifiPass,
                  const char* otaHostname,
                  const char* otaPassword) {
  g_otaMode = true;

  Serial.println();
  Serial.println("OTA button detected.");
  Serial.println("Entering OTA mode...");

  ledOff();
  delay(100);
  ledOn();
  delay(100);
  ledOff();

  WiFi.persistent(false);
  WiFi.mode(WIFI_STA);
  WiFi.begin(wifiSsid, wifiPass);

  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    blinkLedInOtaMode();
  }

  Serial.println();
  Serial.print("WiFi connected. IP: ");
  Serial.println(WiFi.localIP());

  ArduinoOTA.setHostname(otaHostname);
  ArduinoOTA.setPassword(otaPassword);

  ArduinoOTA.onStart([]() {
    Serial.println("OTA Start");
    ledOn();
    g_lastOtaPercent = 255;
  });

  ArduinoOTA.onEnd([]() {
    Serial.println();
    Serial.println("OTA End");
    ledOff();
  });

  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    uint8_t percent = (progress * 100) / total;

    if (g_lastOtaPercent == 255 || percent >= g_lastOtaPercent + 10 || percent == 100) {
      Serial.print("OTA Progress: ");
      Serial.print(percent);
      Serial.println("%");
      g_lastOtaPercent = percent;
    }
  });

  ArduinoOTA.onError([](ota_error_t error) {
    Serial.print("OTA Error[");
    Serial.print((int)error);
    Serial.print("]: ");

    if (error == OTA_AUTH_ERROR) {
      Serial.println("Auth Failed");
    } else if (error == OTA_BEGIN_ERROR) {
      Serial.println("Begin Failed");
    } else if (error == OTA_CONNECT_ERROR) {
      Serial.println("Connect Failed");
    } else if (error == OTA_RECEIVE_ERROR) {
      Serial.println("Receive Failed");
    } else if (error == OTA_END_ERROR) {
      Serial.println("End Failed");
    } else {
      Serial.println("Unknown Error");
    }
  });

  ArduinoOTA.begin();

  Serial.println("OTA Ready");
  Serial.print("Hostname: ");
  Serial.println(otaHostname);
  Serial.println("Select the network port in Arduino IDE and upload wirelessly.");
  Serial.println("Board will remain in OTA mode until reset/power cycle.");
}

void handleOtaMode() {
  if (g_otaMode) {
    ArduinoOTA.handle();
    blinkLedInOtaMode();
  }
}

bool isOtaModeActive() {
  return g_otaMode;
}