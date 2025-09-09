#ifndef DTS6012M_H
#define DTS6012M_H

#include <Arduino.h>
#include <Wire.h>

#define DTS6012M_I2C_ADDR 0x51  // 7-bit I2C address

class DTS6012M {
public:
    DTS6012M(TwoWire &wirePort = Wire, uint8_t i2cAddr = DTS6012M_I2C_ADDR);

    bool begin(uint32_t i2cFreq = 400000);
    bool startMeasurement();
    bool stopMeasurement();

    bool readDistance(uint16_t &distance_mm);
    bool readTestRegister(uint8_t &value);

private:
    TwoWire *_wire;
    uint8_t _i2cAddr;

    bool writeReg(uint8_t reg, uint8_t value);
    bool readReg(uint8_t reg, uint8_t &value);
    bool readReg16(uint8_t regHigh, uint16_t &value);
};

#endif
