#ifndef DTS6012M_H
#define DTS6012M_H

#include <Arduino.h>
#include <Wire.h>

#define DTS6012M_I2C_ADDR 0x51  // 7-bit I2C address

class DTS6012M {
public:
    enum Interface {
        I2C_MODE,
        UART_MODE
    };

    DTS6012M(Interface mode = I2C_MODE, TwoWire &wirePort = Wire, HardwareSerial &serialPort = Serial1);

    bool begin(uint32_t i2cFreq = 400000, uint32_t uartBaud = 921600);

    // ---- I2C API ----
    bool startMeasurement();
    bool stopMeasurement();
    bool readDistance(uint16_t &distance_mm);
    bool readTestRegister(uint8_t &value);

    // ---- UART API ----
    bool startStream();               // send 0x01 command
    bool stopStream();                // send 0x02 command
    bool readFrame(uint16_t &dist);   // parse one UART frame (blocking with timeout)

private:
    Interface _mode;
    TwoWire *_wire;
    HardwareSerial *_serial;
    uint8_t _i2cAddr;

    // I2C helpers
    bool writeReg(uint8_t reg, uint8_t value);
    bool readReg(uint8_t reg, uint8_t &value);
    bool readReg16(uint8_t regHigh, uint16_t &value);

    // UART helpers
    bool sendCommand(uint8_t cmd);
    bool readResponse(uint8_t cmd, uint8_t *buf, size_t len, uint32_t timeout = 50);
};

#endif
