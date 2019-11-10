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

#include "XPT2046_Touchscreen.h"


XPT2046_Touchscreen* XPT2046_Touchscreen::isrPinptr {};

XPT2046_Touchscreen::XPT2046_Touchscreen(uint8_t cspin, SPIClass* p_spi, uint8_t tirq)
    : p_spi_ { p_spi }, isrWake { true }, cs_ { cspin }, tirq_ { tirq }, rotation_ { 1 }, xraw_ {}, yraw_ {}, zraw_ {}, msraw_ {} {}

bool XPT2046_Touchscreen::begin() {
    spisettings_ = SPISettings { SPI_FREQUENCY, MSBFIRST, SPI_MODE0 };

    ::pinMode(cs_, OUTPUT);
    ::digitalWrite(cs_, 1);

    p_spi_->begin();

    if (255 != tirq_) {
        ::pinMode(tirq_, INPUT);
        ::attachInterrupt(
            digitalPinToInterrupt(tirq_),
            []() {
                auto ptr { XPT2046_Touchscreen::isrPinptr };
                if (ptr) {
                    ptr->isrWake = true;
                }
            },
            FALLING);
        isrPinptr = this;
    }

    msraw_ = ::millis();
    return true;
}

TS_Point XPT2046_Touchscreen::getPoint() {
    update();
    return TS_Point(xraw_, yraw_, zraw_);
}

bool XPT2046_Touchscreen::tirqTouched() const {
    return isrWake;
}

bool XPT2046_Touchscreen::touched() {
    update();
    return zraw_ >= Z_THRESHOLD;
}

void XPT2046_Touchscreen::readData(uint16_t* x, uint16_t* y, uint8_t* z) {
    update();
    *x = xraw_;
    *y = yraw_;
    *z = zraw_;
}

bool XPT2046_Touchscreen::bufferEmpty() const {
    return (::millis() - msraw_) < MSEC_THRESHOLD;
}

uint16_t XPT2046_Touchscreen::besttwoavg(uint16_t x, uint16_t y, uint16_t z) const {
    uint16_t da, db, dc;
    uint16_t reta { 0 };
    if (x > y) {
        da = x - y;
    } else {
        da = y - x;
    }
    if (x > z) {
        db = x - z;
    } else {
        db = z - x;
    }
    if (z > y) {
        dc = z - y;
    } else {
        dc = y - z;
    }

    if (da <= db && da <= dc) {
        reta = (x + y) >> 1;
    } else if (db <= da && db <= dc) {
        reta = (x + z) >> 1;
    } else {
        reta = (y + z) >> 1;
    }

    return reta;
}

// TODO: perhaps a future version should offer an option for more oversampling,
//       with the RANSAC algorithm https://en.wikipedia.org/wiki/RANSAC
void XPT2046_Touchscreen::update() {
    if (!isrWake) {
        return;
    }

    const uint32_t now { ::millis() };
    if (now - msraw_ < MSEC_THRESHOLD) {
        return;
    }

    p_spi_->beginTransaction(spisettings_);
    ::digitalWrite(cs_, 0);
    p_spi_->transfer(0xB1 /* Z1 */);
    const auto z1 { p_spi_->transfer16(0xC1 /* Z2 */) >> 3U };
    auto z { z1 + 4095U };
    const auto z2 { p_spi_->transfer16(0x91 /* X */) >> 3U };
    z -= z2;
    uint16_t data[6];
    if (z >= Z_THRESHOLD) {
        p_spi_->transfer16(0x91 /* X */); // dummy X measure, 1st is always noisy
        data[0] = p_spi_->transfer16(0xD1 /* Y */) >> 3;
        data[1] = p_spi_->transfer16(0x91 /* X */) >> 3; // make 3 x-y measurements
        data[2] = p_spi_->transfer16(0xD1 /* Y */) >> 3;
        data[3] = p_spi_->transfer16(0x91 /* X */) >> 3;
    }
    data[4] = p_spi_->transfer16(0xD0 /* Y */) >> 3; // Last Y touch power down
    data[5] = p_spi_->transfer16(0) >> 3;
    ::digitalWrite(cs_, 1);
    p_spi_->endTransaction();

    if (z < Z_THRESHOLD) {
        zraw_ = 0;
        if (tirq_ != 255 && z < Z_THRESHOLD_INT) {
            isrWake = false;
        }
        return;
    }

    zraw_ = z;

    /* average pair with least distance between each measured x then y */
    const auto x { besttwoavg(data[0], data[2], data[4]) };
    const auto y { besttwoavg(data[1], data[3], data[5]) };

    if (z >= Z_THRESHOLD) {
        msraw_ = now; // good read completed, set wait
        switch (rotation_) {
            case 0:
                xraw_ = 4095U - y;
                yraw_ = x;
                break;
            case 1:
                xraw_ = x;
                yraw_ = y;
                break;
            case 2:
                xraw_ = y;
                yraw_ = 4095U - x;
                break;
            case 3:
                xraw_ = 4095U - x;
                yraw_ = 4095U - y;
                break;
            case 4:
                xraw_ = 4095U - y;
                yraw_ = 4095U - x;
                break;
        }
    }
}
