#ifndef OTA_HELPER_H
#define OTA_HELPER_H

#include <Arduino.h>

void otaInitPins(int statusLedPin, int otaButtonPin);
bool shouldEnterOtaMode();
void startOtaMode(const char* wifiSsid,
                  const char* wifiPass,
                  const char* otaHostname,
                  const char* otaPassword);
void handleOtaMode();
bool isOtaModeActive();

#endif