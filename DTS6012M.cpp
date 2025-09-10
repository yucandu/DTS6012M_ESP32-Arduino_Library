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
        //_serial->begin(uartBaud);
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

bool DTS6012M::readBytes(uint8_t *buf, size_t len, uint32_t timeout) {
    unsigned long start = millis();
    size_t idx = 0;
    while ((millis() - start) < timeout && idx < len) {
        if (_serial->available()) {
            buf[idx++] = _serial->read();
        }
    }
    return (idx == len);
}

uint16_t DTS6012M::crc16(const uint8_t *data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 0x0001) {
                crc = (crc >> 1) ^ 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

bool DTS6012M::readFrame(DTS6012M_Frame &frame) {
    // Look for header
    while (_serial->available() && _serial->peek() != 0xA5) {
        _serial->read();
    }

    const size_t frameLen = 23; // header + meta + 14 data + 2 CRC
    uint8_t raw[frameLen];
    if (!readBytes(raw, frameLen, 50)) return false;

    // Verify CRC
    uint16_t crcCalc = crc16(raw, frameLen - 2);
    uint16_t crcRecv = (raw[frameLen - 2] << 8) | raw[frameLen - 1];
    if (crcCalc != crcRecv) return false;

    // Extract fields (little endian pairs)
    frame.secondaryDistance  = (raw[7] << 8) | raw[6];
    frame.secondaryCorrection= (raw[9] << 8) | raw[8];
    frame.secondaryIntensity = (raw[11] << 8) | raw[10];
    frame.primaryDistance    = (raw[13] << 8) | raw[12];
    frame.primaryCorrection  = (raw[15] << 8) | raw[14];
    frame.primaryIntensity   = (raw[17] << 8) | raw[16];
    frame.sunlightBase       = (raw[19] << 8) | raw[18];

    return true;
}


// ---------------- Non-blocking UART parser ----------------
bool DTS6012M::readFrameNonBlocking(DTS6012M_Frame &frame) {
    while (_serial->available()) {
        uint8_t b = _serial->read();

        // Reset if buffer empty and we didn’t see header yet
        if (_rxIndex == 0 && b != 0xA5) {
            continue; // wait for header
        }

        _rxBuf[_rxIndex++] = b;

        // If buffer full, process frame
        if (_rxIndex >= FRAME_LEN) {
            _rxIndex = 0;

            // CRC check
            uint16_t crcCalc = crc16(_rxBuf, FRAME_LEN - 2);
            uint16_t crcRecv = (_rxBuf[FRAME_LEN - 2] << 8) | _rxBuf[FRAME_LEN - 1];
            if (crcCalc != crcRecv) return false;

            // Decode data section
            frame.secondaryDistance  = (_rxBuf[7] << 8) | _rxBuf[6];
            frame.secondaryCorrection= (_rxBuf[9] << 8) | _rxBuf[8];
            frame.secondaryIntensity = (_rxBuf[11] << 8) | _rxBuf[10];
            frame.primaryDistance    = (_rxBuf[13] << 8) | _rxBuf[12];
            frame.primaryCorrection  = (_rxBuf[15] << 8) | _rxBuf[14];
            frame.primaryIntensity   = (_rxBuf[17] << 8) | _rxBuf[16];
            frame.sunlightBase       = (_rxBuf[19] << 8) | _rxBuf[18];

            return true; // full valid frame
        }
    }
    return false; // not complete yet
}
bool DTS6012M::readFrameI2CNonBlocking(DTS6012M_I2CFrame &frame) {
    frame.valid = false;

    if (!_i2cBusy) {
        _wire->beginTransmission(_i2cAddr);
        _wire->write(0x00);
        if (_wire->endTransmission(false) != 0) return false;

        _wire->requestFrom((int)_i2cAddr, 14, (int)true);
        _i2cIndex = 0;
        _i2cBusy = true;
    }

    while (_wire->available() && _i2cIndex < 14) {
        _i2cBuf[_i2cIndex++] = _wire->read();
    }

    if (_i2cIndex >= 14) {
        frame.primaryDistance   = ((uint16_t)_i2cBuf[0] << 8) | _i2cBuf[1];
        frame.secondaryDistance = ((uint16_t)_i2cBuf[4] << 8) | _i2cBuf[5];
        frame.temperatureCode   = ((uint16_t)_i2cBuf[6] << 8) | _i2cBuf[7];
        frame.secondaryIntensity= ((uint16_t)_i2cBuf[8] << 8) | _i2cBuf[9];
        frame.primaryCorrection = ((uint16_t)_i2cBuf[10] << 8) | _i2cBuf[11];
        frame.primaryIntensity  = ((uint16_t)_i2cBuf[12] << 8) | _i2cBuf[13];
        frame.sunlightBase      = 0; // can add if available at 0x0E–0x0F

        frame.valid = (frame.primaryDistance != 0xFFFF);
        _i2cBusy = false;
        return frame.valid;
    }

    return false; // still waiting
}
