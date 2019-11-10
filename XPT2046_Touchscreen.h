/* Touchscreen library for XPT2046 Touch Controller Chip
 * Copyright (c) 2015, Paul Stoffregen, paul@pjrc.com
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice, development funding notice, and this permission
 * notice shall be included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 * Adapted for use with the c't-Bot teensy framework and ported to C++14 by Timo Sandmann
 */

#pragma once

#include "Arduino.h"
#include "SPI.h"

#include <cstdint>


class TS_Point {
public:
    TS_Point() : x {}, y {}, z {} {}

    TS_Point(int16_t x, int16_t y, int16_t z) : x { x }, y { y }, z { z } {}

    bool operator==(TS_Point p) {
        return ((p.x == x) && (p.y == y) && (p.z == z));
    }

    bool operator!=(TS_Point p) {
        return ((p.x != x) || (p.y != y) || (p.z != z));
    }

    int16_t x, y, z;
};

class XPT2046_Touchscreen {
public:
    XPT2046_Touchscreen(uint8_t cspin, SPIClass* p_spi = &SPI, uint8_t tirq = 255);

    bool begin();
    TS_Point getPoint();
    bool tirqTouched() const;
    bool touched();
    void readData(uint16_t* x, uint16_t* y, uint8_t* z);
    bool bufferEmpty() const;

    uint8_t bufferSize() const {
        return 1;
    }

    void setRotation(uint8_t n) {
        rotation_ = n % 5;
    }

private:
    uint16_t besttwoavg(uint16_t x, uint16_t y, uint16_t z) const;
    void update();

    static constexpr uint32_t SPI_FREQUENCY { 2'500'000 };
    static constexpr uint16_t Z_THRESHOLD { 400 };
    static constexpr uint16_t Z_THRESHOLD_INT { 75 };
    static constexpr uint32_t MSEC_THRESHOLD { 3 };

    static XPT2046_Touchscreen* isrPinptr;

    SPIClass* p_spi_;
    SPISettings spisettings_;
    volatile bool isrWake;
    const uint8_t cs_;
    const uint8_t tirq_;
    uint8_t rotation_;
    uint16_t xraw_;
    uint16_t yraw_;
    uint16_t zraw_;
    uint32_t msraw_;
};
