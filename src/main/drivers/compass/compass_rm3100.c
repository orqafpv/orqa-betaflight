/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 */

#include <stdbool.h>
#include <stdint.h>
#include <math.h>

#include "platform.h"

#ifdef USE_MAG_RM3100

#include "common/axis.h"
#include "common/maths.h"
#include "common/utils.h"

#include "drivers/bus.h"
#include "drivers/bus_i2c.h"
#include "drivers/sensor.h"
#include "drivers/time.h"

#include "compass.h"

// RM3100 Registers
#define RM3100_REG_POLL         0x00
#define RM3100_REG_CMM          0x01
#define RM3100_REG_CCX1         0x04
#define RM3100_REG_TMRC         0x0B 
#define RM3100_REG_MX           0x24 
#define RM3100_REG_STATUS       0x34
#define RM3100_REG_REVID        0x36

#define RM3100_REVID            0x22
#define RM3100_MAG_I2C_ADDRESS  0x20

#define CC_DEFAULT              200 
#define TMRC_DEFAULT            0x94 
#define CMM_CONT_MODE           0x71 

static bool rm3100Init(magDev_t *magDev)
{
    extDevice_t *dev = &magDev->dev;
    busDeviceRegister(dev);

    bool ack = true;
    uint8_t cc[2] = { (CC_DEFAULT >> 8) & 0xFF, CC_DEFAULT & 0xFF };
    
    // Cycle Counts configuration
    ack = ack && busWriteRegister(dev, RM3100_REG_CCX1,     cc[0]); // CCX1
    ack = ack && busWriteRegister(dev, RM3100_REG_CCX1 + 1, cc[1]); // CCX0
    ack = ack && busWriteRegister(dev, RM3100_REG_CCX1 + 2, cc[0]); // CCY1
    ack = ack && busWriteRegister(dev, RM3100_REG_CCX1 + 3, cc[1]); // CCY0
    ack = ack && busWriteRegister(dev, RM3100_REG_CCX1 + 4, cc[0]); // CCZ1
    ack = ack && busWriteRegister(dev, RM3100_REG_CCX1 + 5, cc[1]); // CCZ0

    // Set Data Rate & Continuous Mode
    ack = ack && busWriteRegister(dev, RM3100_REG_TMRC, TMRC_DEFAULT);
    ack = ack && busWriteRegister(dev, RM3100_REG_CMM, CMM_CONT_MODE);

    if (!ack) {
        return false;
    }

    magDev->magOdrHz = 100; 
    return true;
}

static bool rm3100Read(magDev_t *magDev, int16_t *magData)
{
    static uint8_t buf[9]; // 3 axes * 3 bytes each
    static uint8_t status = 0;
    static enum {
        STATE_WAIT_DRDY,
        STATE_READ,
    } state = STATE_WAIT_DRDY;

    extDevice_t *dev = &magDev->dev;

    switch (state) {
        default:
        case STATE_WAIT_DRDY:
            // Check if DRDY (MSB) bit is set in the last polled status byte
            if (status & 0x80) {
                // Non-blocking asynchronous start of data matrix read
                if (busReadRegisterBufferStart(dev, RM3100_REG_MX, buf, sizeof(buf))) {
                    state = STATE_READ;
                }
            } else {
                // Non-blocking asynchronous update of status register
                busReadRegisterBufferStart(dev, RM3100_REG_STATUS, &status, sizeof(status));
            }
            return false; // Data is not ready yet to be dispatched to Betaflight core

        case STATE_READ:
        {
            // Reconstruct 24-bit signed values from big-endian buffer
            int32_t xraw = (int32_t)(((uint32_t)buf[0] << 24) | ((uint32_t)buf[1] << 16) | ((uint32_t)buf[2] << 8)) >> 8;
            int32_t yraw = (int32_t)(((uint32_t)buf[3] << 24) | ((uint32_t)buf[4] << 16) | ((uint32_t)buf[5] << 8)) >> 8;
            int32_t zraw = (int32_t)(((uint32_t)buf[6] << 24) | ((uint32_t)buf[7] << 16) | ((uint32_t)buf[8] << 8)) >> 8;

	    magData[X] = (int16_t)constrain(xraw / 10, INT16_MIN, INT16_MAX);
            magData[Y] = (int16_t)constrain(yraw / 10, INT16_MIN, INT16_MAX);
            magData[Z] = (int16_t)constrain(zraw / 10, INT16_MIN, INT16_MAX);

            // Reset state machine configurations for next read cycle
            state = STATE_WAIT_DRDY;
            status = 0;
            return true; // Successfully read fresh data
        }
    }

    return false;
}

bool rm3100Detect(magDev_t *magDev)
{
    extDevice_t *dev = &magDev->dev;

    if (dev->bus->busType == BUS_TYPE_I2C && dev->busType_u.i2c.address == 0) {
        dev->busType_u.i2c.address = RM3100_MAG_I2C_ADDRESS;
    }

    uint8_t revid = 0;
    // Blocking read is perfectly acceptable during boot up/detection phase
    bool ack = busReadRegisterBuffer(dev, RM3100_REG_REVID, &revid, 1);
    
    if (ack && revid == RM3100_REVID) {
        magDev->init = rm3100Init;
        magDev->read = rm3100Read;
        return true;
    }
    
    return false;
}
#endif
