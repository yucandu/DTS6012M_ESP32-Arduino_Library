#include "DTS6012M.h"

// ---------------- Constructor ----------------
DTS6012M::DTS6012M(Interface mode, TwoWire &wirePort, HardwareSerial &serialPort) {
    _mode = mode;
    _wire = &wirePort;
    _serial = &serialPort;
    _i2cAddr = DTS6012M_I2C_ADDR;
}

// ---------------- Begin ----------------
bool DTS6012M::begin(uint32_t i2cFreq, uint32_t uartBaud) {
    if (_mode == I2C_MODE) {
        _wire->begin();
        _wire->setClock(i2cFreq);
        uint8_t test;
        if (!readTestRegister(test)) return false;
        return (test == 0x3B);
    } else {
        _serial->begin(uartBaud);
        delay(50);
        return true;
    }
}

// ---------------- I2C API ----------------
bool DTS6012M::startMeasurement() { return writeReg(0x02, 0x01); }
bool DTS6012M::stopMeasurement()  { return writeReg(0x02, 0x00); }
bool DTS6012M::readDistance(uint16_t &distance_mm) { return readReg16(0x00, distance_mm); }
bool DTS6012M::readTestRegister(uint8_t &value)   { return readReg(0x03, value); }

// I2C helpers
bool DTS6012M::writeReg(uint8_t reg, uint8_t value) {
    _wire->beginTransmission(_i2cAddr);
    _wire->write(reg);
    _wire->write(value);
    return (_wire->endTransmission() == 0);
}

bool DTS6012M::readReg(uint8_t reg, uint8_t &value) {
    _wire->beginTransmission(_i2cAddr);
    _wire->write(reg);
    if (_wire->endTransmission(false) != 0) return false;
    if (_wire->requestFrom((int)_i2cAddr, 1) != 1) return false;
    value = _wire->read();
    return true;
}

bool DTS6012M::readReg16(uint8_t regHigh, uint16_t &value) {
    _wire->beginTransmission(_i2cAddr);
    _wire->write(regHigh);
    if (_wire->endTransmission(false) != 0) return false;
    if (_wire->requestFrom((int)_i2cAddr, 2) != 2) return false;
    uint8_t high = _wire->read();
    uint8_t low  = _wire->read();
    value = (high << 8) | low;
    return true;
}

// ---------------- UART API ----------------
bool DTS6012M::sendCommand(uint8_t cmd) {
    uint8_t packet[7];
    packet[0] = 0xA5; // header
    packet[1] = 0x03; // device ID
    packet[2] = 0x20; // device type
    packet[3] = cmd;  // command
    packet[4] = 0x00; // reserved
    packet[5] = 0x00; // length high
    packet[6] = 0x00; // length low
    _serial->write(packet, sizeof(packet));
    return true;
}

bool DTS6012M::startStream() { return sendCommand(0x01); }
bool DTS6012M::stopStream()  { return sendCommand(0x02); }

bool DTS6012M::readResponse(uint8_t cmd, uint8_t *buf, size_t len, uint32_t timeout) {
    unsigned long start = millis();
    size_t idx = 0;
    while ((millis() - start) < timeout && idx < len) {
        if (_serial->available()) {
            buf[idx++] = _serial->read();
        }
    }
    return (idx == len);
}

// Parse one measurement frame (distance only)
bool DTS6012M::readFrame(uint16_t &dist) {
    // Expect at leas
