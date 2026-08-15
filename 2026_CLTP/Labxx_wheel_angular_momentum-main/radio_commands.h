#ifndef RADIO_COMMANDS_H
#define RADIO_COMMANDS_H

#include <Arduino.h>

constexpr int32_t MAX_WHEEL_SPEED_RPM = 3000;
constexpr int32_t WHEEL_MAX_CURRENT = 100000;  // Register unit: 0.01 mA
constexpr float RPM_TO_RAD_PER_SEC = TWO_PI / 60.0f;
constexpr unsigned long WHEEL_STARTUP_TIMEOUT_MS = 5000;
constexpr unsigned long WHEEL_RETRY_INTERVAL_MS = 250;

using CommandHandler = void (*)(String command);

void receiveRadioCommands(CommandHandler commandHandler);
void sendMessage(const String &message);
void sendBodyAngularVelocity();

#endif  // RADIO_COMMANDS_H
