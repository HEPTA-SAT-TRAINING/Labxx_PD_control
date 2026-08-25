/*
 *SPDX-FileCopyrightText: 2024 M5Stack Technology CO LTD
 *
 *SPDX-License-Identifier: MIT
 */

#include "unit_rolleri2c.hpp"

void UnitRollerI2C::writeBytes(uint8_t addr, uint8_t reg, uint8_t *buffer, uint8_t length)
{
    // Transient I2C failures previously dropped speed/stop commands silently.
    // Retry each write and update the driver's connection state.
    for (uint8_t attempt = 0; attempt < 3; ++attempt) {
        _wire->beginTransmission(addr);
        _wire->write(reg);
        for (int i = 0; i < length; i++) {
            _wire->write(*(buffer + i));
        }
        if (_wire->endTransmission() == 0) {
            _connected = true;
            return;
        }
        delay(2);
    }
    _connected = false;
#if defined UNIT_ROLLER_DEBUG
    Serial.print("Writing to I2C address: 0x");
    Serial.print(addr, HEX);
    Serial.print(", Register: 0x");
    Serial.print(reg, HEX);
    Serial.print(", Data: ");
    for (int i = 0; i < length; i++) {
        Serial.print("0x");
        Serial.print(*(buffer + i), HEX);
        if (i < length - 1) {
            Serial.print(", ");  // Print comma except for the last element
        }
    }
    Serial.println();  // New line after printing all data
#else
#endif
}

void UnitRollerI2C::readBytes(uint8_t addr, uint8_t reg, uint8_t *buffer, uint8_t length)
{
    for (uint8_t attempt = 0; attempt < 3; ++attempt) {
        _wire->beginTransmission(addr);
        _wire->write(reg);
        if (_wire->endTransmission(false) != 0) {
            delay(2);
            continue;
        }

        const uint8_t received = _wire->requestFrom(addr, length);
        if (received == length && _wire->available() >= length) {
            for (uint8_t i = 0; i < length; ++i) {
                buffer[i] = static_cast<uint8_t>(_wire->read());
            }
            _connected = true;
            return;
        }

        while (_wire->available()) _wire->read();
        delay(2);
    }
    _connected = false;
#if defined UNIT_ROLLER_DEBUG
    Serial.print("Reading from I2C address: 0x");
    Serial.print(addr, HEX);
    Serial.print(", Register: 0x");
    Serial.print(reg, HEX);
    Serial.print(", Length: ");
    Serial.println(length);
    for (int i = 0; i < length; i++) {
        Serial.print("0x");
        Serial.print(buffer[i], HEX);
        if (i < length - 1) {
            Serial.print(", ");  // Print comma except for the last element
        }
    }

    Serial.println();  // New line after printing all received data
#endif
}

void UnitRollerI2C::floatToBytes(float s, uint8_t *d)
{
    union {
        float value;
        uint8_t bytes[4];
    } float_union_t;
    float_union_t.value = s;
    memcpy(d, float_union_t.bytes, 4);
}

float UnitRollerI2C::bytesToFloat(uint8_t *s)
{
    union {
        float value;
        uint8_t bytes[4];
    } bytes_union_t;
    memcpy(bytes_union_t.bytes, s, 4);
    return bytes_union_t.value;
}

bool UnitRollerI2C::begin(TwoWire *wire, uint8_t addr, uint32_t speed)
{
    _wire  = wire;
    _addr  = addr;
    _speed = speed;
    _wire->begin();
    _wire->setClock(_speed);
    delay(10);
    _wire->beginTransmission(_addr);
    uint8_t error = _wire->endTransmission();
    _connected = (error == 0);
    return _connected;
}

bool UnitRollerI2C::begin(uint8_t sdaPin, uint8_t sclPin)
{
    return begin(&Wire1, sdaPin, sclPin, I2C_ADDR, 100000L, 5000, 250);
}

bool UnitRollerI2C::begin(TwoWire *wire, uint8_t sdaPin, uint8_t sclPin,
                          uint8_t addr, uint32_t speed,
                          unsigned long timeoutMs,
                          unsigned long retryIntervalMs)
{
    _wire = wire;
    _addr = addr;
    _speed = speed;
    _wire->setSDA(sdaPin);
    _wire->setSCL(sclPin);
    _wire->begin();
    _wire->setClock(_speed);
    delay(10);
    return connect(timeoutMs, retryIntervalMs);
}

bool UnitRollerI2C::connect(unsigned long timeoutMs,
                            unsigned long retryIntervalMs)
{
    const unsigned long startedAt = millis();
    do {
        _wire->beginTransmission(_addr);
        _connected = (_wire->endTransmission() == 0);
        if (_connected) {
            configure();
            return true;
        }
        delay(retryIntervalMs);
    } while (millis() - startedAt < timeoutMs);

    return false;
}

void UnitRollerI2C::configure(int32_t maxCurrent)
{
    setOutput(0);
    setMode(ROLLER_MODE_SPEED);
    setSpeedRpm(0);
    setSpeedMaxCurrent(maxCurrent);
}

void UnitRollerI2C::start(int32_t speedRpm, int32_t maxCurrent)
{
    setMode(ROLLER_MODE_SPEED);
    setSpeedMaxCurrent(maxCurrent);
    setSpeedRpm(speedRpm);
    setOutput(1);
}

void UnitRollerI2C::stop(void)
{
    setSpeedRpm(0);
    setOutput(0);
}

void UnitRollerI2C::setSpeedRpm(int32_t speedRpm)
{
    setSpeed(speedRpm * SPEED_REGISTER_SCALE);
}

int32_t UnitRollerI2C::getSpeedRpm(void)
{
    return getSpeed() / SPEED_REGISTER_SCALE;
}

int32_t UnitRollerI2C::getSpeedReadbackRpm(void)
{
    return getSpeedReadback() / SPEED_REGISTER_SCALE;
}

bool UnitRollerI2C::isConnected(void) const
{
    return _connected;
}

void UnitRollerI2C::setMode(roller_mode_t mode)
{
    uint8_t reg = I2C_MODE_REG;
    writeBytes(_addr, reg, (uint8_t *)&mode, 1);
}

void UnitRollerI2C::setOutput(uint8_t en)
{
    uint8_t reg = I2C_OUTPUT_REG;
    writeBytes(_addr, reg, (uint8_t *)&en, 1);
}

void UnitRollerI2C::setSpeed(int32_t speed)
{
    uint8_t reg = I2C_SPEED_REG;
    writeBytes(_addr, reg, (uint8_t *)&speed, 4);
}

void UnitRollerI2C::setSpeedMaxCurrent(int32_t current)
{
    uint8_t reg = I2C_SPEED_MAX_CURRENT_REG;
    writeBytes(_addr, reg, (uint8_t *)&current, 4);
}

void UnitRollerI2C::setSpeedPID(uint32_t p, uint32_t i, uint32_t d)
{
    uint8_t data[12] = {0};
    uint8_t reg      = I2C_SPEED_PID_REG;

    memcpy(data, (uint8_t *)&p, 4);
    memcpy(data + 4, (uint8_t *)&i, 4);
    memcpy(data + 8, (uint8_t *)&d, 4);
    writeBytes(_addr, reg, data, 12);
}

void UnitRollerI2C::setPos(int32_t pos)
{
    uint8_t reg = I2C_POS_REG;
    writeBytes(_addr, reg, (uint8_t *)&pos, 4);
}

void UnitRollerI2C::setPosMaxCurrent(int32_t current)
{
    uint8_t reg = I2C_POS_MAX_CURRENT_REG;
    writeBytes(_addr, reg, (uint8_t *)&current, 4);
}

void UnitRollerI2C::setPosPID(uint32_t p, uint32_t i, uint32_t d)
{
    uint8_t data[12] = {0};
    uint8_t reg      = I2C_POS_PID_REG;

    memcpy(data, (uint8_t *)&p, 4);
    memcpy(data + 4, (uint8_t *)&i, 4);
    memcpy(data + 8, (uint8_t *)&d, 4);
    writeBytes(_addr, reg, data, 12);
}

void UnitRollerI2C::setCurrent(int32_t current)
{
    uint8_t reg = I2C_CURRENT_REG;
    writeBytes(_addr, reg, (uint8_t *)&current, 4);
}

void UnitRollerI2C::setDialCounter(int32_t counter)
{
    uint8_t reg = I2C_DIAL_COUNTER_REG;
    writeBytes(_addr, reg, (uint8_t *)&counter, 4);
}

void UnitRollerI2C::setRGBMode(roller_rgb_t mode)
{
    uint8_t reg = I2C_RGB_REG + 3;
    writeBytes(_addr, reg, (uint8_t *)&mode, 1);
}

void UnitRollerI2C::setRGB(int32_t color)
{
    uint8_t reg    = I2C_RGB_REG;
    uint8_t r      = (color >> 11) & 0x1F;  // R (5 bits)
    uint8_t g      = (color >> 5) & 0x3F;   // G (6 bits)
    uint8_t b      = color & 0x1F;          // B (5 bits)
    r              = (r * 255) / 31;        // Expand to 255
    g              = (g * 255) / 63;        // Expand to 255
    b              = (b * 255) / 31;        // Expand to 255
    uint8_t bgr[3] = {b, g, r};
    writeBytes(_addr, reg, bgr, 3);
}

void UnitRollerI2C::setRGBBrightness(uint8_t brightness)
{
    uint8_t reg = I2C_RGB_BRIGHTNESS_REG;
    writeBytes(_addr, reg, (uint8_t *)&brightness, 1);
}

void UnitRollerI2C::setMotorID(uint8_t id)
{
    uint8_t reg = I2C_ID_REG;
    writeBytes(_addr, reg, (uint8_t *)&id, 1);
}

void UnitRollerI2C::setBPS(roller_bps_t bps)
{
    uint8_t reg = I2C_BPS_REG;
    writeBytes(_addr, reg, (uint8_t *)&bps, 1);
}

void UnitRollerI2C::saveConfigToFlash(void)
{
    uint8_t data = 1;
    uint8_t reg  = I2C_SAVE_FLASH_REG;
    writeBytes(_addr, reg, (uint8_t *)&data, 1);
}

void UnitRollerI2C::posRangeProtect(uint8_t en)
{
    uint8_t reg = I2C_POS_RANGE_PROTECT_REG;
    writeBytes(_addr, reg, (uint8_t *)&en, 1);
}

void UnitRollerI2C::resetStalledProtect(void)
{
    uint8_t data = 1;
    uint8_t reg  = I2C_RESET_STALLED_PROTECT_REG;
    writeBytes(_addr, reg, (uint8_t *)&data, 1);
}

void UnitRollerI2C::setKeySwitchMode(uint8_t en)
{
    uint8_t reg = I2C_KEY_SWTICH_MODE_REG;
    writeBytes(_addr, reg, (uint8_t *)&en, 1);
}

void UnitRollerI2C::setStallProtection(uint8_t en)
{
    uint8_t reg = I2C_STALL_PROTECTION_REG;
    writeBytes(_addr, reg, (uint8_t *)&en, 1);
}

uint8_t UnitRollerI2C::setI2CAddress(uint8_t addr)
{
    uint8_t data[2] = {0};
    data[0]         = I2C_ADDRESS_REG;
    _wire->beginTransmission(_addr);
    _wire->write(data[0]);
    _wire->write(addr);
    _wire->endTransmission();
    _addr = addr;
    return _addr;
}

void UnitRollerI2C::startAngleCal(void)
{
    uint8_t temp = 1;
    uint8_t reg  = START_ANGLE_CAL_REG;
    writeBytes(_addr, reg, (uint8_t *)&temp, 1);
}

void UnitRollerI2C::updateAngleCal(void)
{
    uint8_t temp = 1;
    uint8_t reg  = UPDATE_ANGLE_CAL_REG;
    writeBytes(_addr, reg, (uint8_t *)&temp, 1);
}

uint8_t UnitRollerI2C::getCalBusyStatus(void)
{
    uint8_t temp = 0;
    uint8_t reg  = GET_ANGLE_BUSY_REG;
    readBytes(_addr, reg, (uint8_t *)&temp, 1);

    return temp;
}

void UnitRollerI2C::getSpeedPID(uint32_t *p, uint32_t *i, uint32_t *d)
{
    uint8_t data[12];
    readBytes(_addr, I2C_SPEED_PID_REG, (uint8_t *)&data, 12);
    memcpy(p, (uint8_t *)&data[0], 4);
    memcpy(i, (uint8_t *)&data[4], 4);
    memcpy(d, (uint8_t *)&data[8], 4);
}

void UnitRollerI2C::getPosPID(uint32_t *p, uint32_t *i, uint32_t *d)
{
    uint8_t data[12];
    readBytes(_addr, I2C_POS_PID_REG, (uint8_t *)&data, 12);
    memcpy(p, (uint8_t *)&data[0], 4);
    memcpy(i, (uint8_t *)&data[4], 4);
    memcpy(d, (uint8_t *)&data[8], 4);
}

void UnitRollerI2C::getRGB(uint8_t *r, uint8_t *g, uint8_t *b)
{
    uint8_t data[3];
    uint8_t reg = I2C_RGB_REG;
    readBytes(_addr, reg, (uint8_t *)&data, 3);
    memcpy(r, (uint8_t *)&data[2], 1);
    memcpy(g, (uint8_t *)&data[1], 1);
    memcpy(b, (uint8_t *)&data[0], 1);
}

int32_t UnitRollerI2C::getSpeed(void)
{
    int32_t data = 0;
    uint8_t reg  = I2C_SPEED_REG;
    readBytes(_addr, reg, (uint8_t *)&data, 4);
    return data;
}

int32_t UnitRollerI2C::getSpeedMaxCurrent(void)
{
    int32_t data = 0;
    uint8_t reg  = I2C_SPEED_MAX_CURRENT_REG;
    readBytes(_addr, reg, (uint8_t *)&data, 4);
    return data;
}

int32_t UnitRollerI2C::getSpeedReadback(void)
{
    int32_t data = 0;
    uint8_t reg  = I2C_SPEED_READBACK_REG;
    readBytes(_addr, reg, (uint8_t *)&data, 4);
    return data;
}

int32_t UnitRollerI2C::getPos(void)
{
    int32_t data = 0;
    uint8_t reg  = I2C_POS_REG;
    readBytes(_addr, reg, (uint8_t *)&data, 4);
    return data;
}

int32_t UnitRollerI2C::getPosMaxCurrent(void)
{
    int32_t data = 0;
    uint8_t reg  = I2C_POS_MAX_CURRENT_REG;
    readBytes(_addr, reg, (uint8_t *)&data, 4);
    return data;
}

int32_t UnitRollerI2C::getPosReadback(void)
{
    int32_t data = 0;
    uint8_t reg  = I2C_POS_READBACK_REG;
    readBytes(_addr, reg, (uint8_t *)&data, 4);
    return data;
}

int32_t UnitRollerI2C::getCurrent(void)
{
    int32_t data = 0;
    uint8_t reg  = I2C_CURRENT_REG;
    readBytes(_addr, reg, (uint8_t *)&data, 4);
    return data;
}

int32_t UnitRollerI2C::getCurrentReadback(void)
{
    int32_t data = 0;
    uint8_t reg  = I2C_CURRENT_READBACK_REG;
    readBytes(_addr, reg, (uint8_t *)&data, 4);
    return data;
}

int32_t UnitRollerI2C::getDialCounter(void)
{
    int32_t data = 0;
    uint8_t reg  = I2C_DIAL_COUNTER_REG;
    readBytes(_addr, reg, (uint8_t *)&data, 4);

    return data;
}

int32_t UnitRollerI2C::getVin(void)
{
    int32_t data = 0;
    uint8_t reg  = I2C_VIN_REG;
    readBytes(_addr, reg, (uint8_t *)&data, 4);

    return data;
}

int32_t UnitRollerI2C::getTemp(void)
{
    int32_t data = 0;
    uint8_t reg  = I2C_TEMP_REG;
    readBytes(_addr, reg, (uint8_t *)&data, 4);

    return data;
}

uint8_t UnitRollerI2C::getSysStatus(void)
{
    uint8_t data = 0;
    uint8_t reg  = I2C_SYS_STATUS_REG;
    readBytes(_addr, reg, (uint8_t *)&data, 1);

    return data;
}

uint8_t UnitRollerI2C::getErrorCode(void)
{
    uint8_t data = 0;
    uint8_t reg  = I2C_ERROR_CODE_REG;
    readBytes(_addr, reg, (uint8_t *)&data, 1);

    return data;
}

uint8_t UnitRollerI2C::getPosRangeProtect(void)
{
    uint8_t data = 0;
    uint8_t reg  = I2C_POS_RANGE_PROTECT_REG;
    readBytes(_addr, reg, (uint8_t *)&data, 1);
    return data;
}

uint8_t UnitRollerI2C::getStallProtection(void)
{
    uint8_t data = 0;
    uint8_t reg  = I2C_STALL_PROTECTION_REG;
    readBytes(_addr, reg, (uint8_t *)&data, 1);

    return data;
}

uint8_t UnitRollerI2C::getKeySwitchMode(void)
{
    uint8_t data = 0;
    uint8_t reg  = I2C_KEY_SWTICH_MODE_REG;
    readBytes(_addr, reg, (uint8_t *)&data, 1);
    return data;
}
uint8_t UnitRollerI2C::getOutputStatus(void)
{
    uint8_t data = 0;
    uint8_t reg  = I2C_OUTPUT_REG;
    readBytes(_addr, reg, (uint8_t *)&data, 1);
    return data;
}

uint8_t UnitRollerI2C::getMotorMode(void)
{
    uint8_t data = 0;
    uint8_t reg  = I2C_MODE_REG;
    readBytes(_addr, reg, (uint8_t *)&data, 1);
    return data;
}

uint8_t UnitRollerI2C::getMotorID(void)
{
    uint8_t data = 0;
    uint8_t reg  = I2C_ID_REG;
    readBytes(_addr, reg, (uint8_t *)&data, 1);
    return data;
}

uint8_t UnitRollerI2C::getBPS(void)
{
    uint8_t data = 0;
    uint8_t reg  = I2C_BPS_REG;
    readBytes(_addr, reg, (uint8_t *)&data, 1);
    return data;
}

uint8_t UnitRollerI2C::getRGBBrightness(void)
{
    uint8_t data = 0;
    uint8_t reg  = I2C_RGB_BRIGHTNESS_REG;
    readBytes(_addr, reg, (uint8_t *)&data, 1);
    return data;
}

uint8_t UnitRollerI2C::getRGBMode(void)
{
    uint8_t data = 0;
    uint8_t reg  = I2C_RGB_REG + 3;
    readBytes(_addr, reg, (uint8_t *)&data, 1);
    return data;
}

uint8_t UnitRollerI2C::getFirmwareVersion(void)
{
    _wire->beginTransmission(_addr);
    _wire->write(I2C_FIRMWARE_VERSION_REG);
    _wire->endTransmission(false);

    uint8_t RegValue;
    _wire->requestFrom(_addr, 1);
    RegValue = _wire->read();
    return RegValue;
}

uint8_t UnitRollerI2C::getI2CAddress(void)
{
    uint8_t data[2] = {0};

    data[0] = I2C_ADDRESS_REG;
    _wire->beginTransmission(_addr);
    _wire->write(data[0]);
    _wire->endTransmission(false);
    uint8_t RegValue;
    _wire->requestFrom(_addr, 1);
    RegValue = _wire->read();
    return RegValue;
}
