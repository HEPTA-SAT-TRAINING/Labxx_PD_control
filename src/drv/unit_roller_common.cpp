/*
 *SPDX-FileCopyrightText: 2024 M5Stack Technology CO LTD
 *
 *SPDX-License-Identifier: MIT
 */

#include "unit_roller_common.hpp"
#include "unit_rolleri2c.hpp"

bool mutexLocked = false;
void acquire_mutex()
{
    while (mutexLocked) {
        delay(1);
    }
    mutexLocked = true;
}

void release_mutex()
{
    mutexLocked = false;
}

int32_t handle_value(uint8_t byte0, uint8_t byte1, uint8_t byte2, uint8_t byte3)
{
    int32_t newNumerical = 0;
    // Combine bytes to form the encoder value
    newNumerical |= ((int32_t)byte0);  // LSB
    newNumerical |= ((int32_t)byte1 << 8);
    newNumerical |= ((int32_t)byte2 << 16);
    newNumerical |= ((int32_t)byte3 << 24);  // MSB
    return newNumerical;
}

bool connect_wheel(UnitRollerI2C &wheel, bool &isConnected,
                  unsigned long timeoutMs, unsigned long retryIntervalMs)
{
    isConnected = wheel.connect(timeoutMs, retryIntervalMs);
    return isConnected;
}

String start_wheel(UnitRollerI2C &wheel, int32_t targetSpeedRpm,
                  int32_t maxCurrent, bool &isRunning)
{
    if (targetSpeedRpm == 0) return "ERR SET_SPEED_FIRST";

    wheel.start(targetSpeedRpm, maxCurrent);
    isRunning = true;
    return "OK START " + String(targetSpeedRpm) + " rpm";
}

String stop_wheel(UnitRollerI2C &wheel, bool &isRunning)
{
    wheel.stop();
    isRunning = false;
    return "OK STOP";
}

String set_wheel_speed(UnitRollerI2C &wheel, const String &argument,
                     int32_t maxSpeedRpm, int32_t &targetSpeedRpm,
                     bool &isRunning)
{
    char *endPointer = nullptr;
    long requestedRpm = strtol(argument.c_str(), &endPointer, 10);

    if (argument.length() == 0 || *endPointer != '\0') {
        return "ERR USAGE R<rpm>";
    }
    if (requestedRpm < -maxSpeedRpm || requestedRpm > maxSpeedRpm) {
        return "ERR SPEED_RANGE -" + String(maxSpeedRpm) + ".." +
               String(maxSpeedRpm) + " rpm";
    }

    targetSpeedRpm = static_cast<int32_t>(requestedRpm);
    if (isRunning) {
        if (targetSpeedRpm == 0) return stop_wheel(wheel, isRunning);
        wheel.setSpeedRpm(targetSpeedRpm);
    }

    return "OK SPEED " + String(targetSpeedRpm) + " rpm";
}

String send_wheel_status(UnitRollerI2C &wheel, bool isConnected, bool isRunning,
                       int32_t targetSpeedRpm, float rpmToRadPerSec)
{
    String message = "STATUS ";
    message += (isRunning ? "RUNNING" : "STOPPED");
    message += " TARGET=" + String(targetSpeedRpm) + " rpm";
    message += " TARGET_OMEGA=";
    message += String(targetSpeedRpm * rpmToRadPerSec, 2);
    message += " rad/s";

    if (isConnected) {
        float actualRpm = wheel.getSpeedReadbackRpm();
        message += " ACTUAL=" + String(actualRpm, 2) + " rpm";
        message += " ACTUAL_OMEGA=";
        message += String(actualRpm * rpmToRadPerSec, 2);
        message += " rad/s";
    } else {
        message += " WHEEL=DISCONNECTED";
    }
    return message;
}
