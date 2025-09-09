#include "DTS6012M.h"

DTS6012M::DTS6012M(TwoWire &wirePort, uint8_t i2cAddr) {
    _wire = &wirePort;
    _i2cAddr = i2cAddr;
}

bool DTS6012M::begin(uint32_t i2cFreq) {
    _wire->begin();
    _wire->setClock(i2cFreq);

    uint8_t test;
    if (!readTestRegister(test)) return false;
    return (test == 0x3B);  // check expected value
}

bool DTS6012M::startMeasurement() {
    return writeReg(0x02, 0x01);
}

bool DTS6012M::stopMeasurement() {
    return writeReg(0x02, 0x00);
}

bool DTS6012M::readDistance(uint16_t &distance_mm) {
    return readReg16(0x00, distance_mm);
}

bool DTS6012M::readTestRegister(uint8_t &value) {
    return readReg(0x03, value);
}

// ---- Low-level helpers ----
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
    uint8_t low = _wire->read();
    value = (high << 8) | low;
    return true;
}
