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

//#pragma GCC optimize ("O0")

// RM3100 Registers
#define RM3100_REG_POLL        0x00
#define RM3100_REG_CMM         0x01
#define RM3100_REG_CCX1        0x04
#define RM3100_REG_CCX0        0x05
#define RM3100_REG_CCY1        0x06
#define RM3100_REG_CCY0        0x07
#define RM3100_REG_CCZ1        0x08
#define RM3100_REG_CCZ0        0x09
#define RM3100_REG_TMRC        0x0B
#define RM3100_REG_MX          0x24
#define RM3100_REG_MY          0x27
#define RM3100_REG_MZ          0x2A
#define RM3100_REG_BIST        0x33
#define RM3100_REG_STATUS      0x34
#define RM3100_REG_HSHAKE      0x35
#define RM3100_REG_REVID       0x36
#define RM3100_MAG_I2C_ADDRESS  0x20

#define CCX_DEFAULT_MSB        0x00
#define CCX_DEFAULT_LSB        0xC8
#define CCY_DEFAULT_MSB        CCX_DEFAULT_MSB
#define CCY_DEFAULT_LSB        CCX_DEFAULT_LSB
#define CCZ_DEFAULT_MSB        CCX_DEFAULT_MSB
#define CCZ_DEFAULT_LSB        CCX_DEFAULT_LSB

#define CC_DEFAULT              200 
#define TMRC_DEFAULT            0x94 
#define CMM_CONT_MODE           0x71 

#define RM3100_REVID            0x22

static bool rm3100Init(magDev_t *magDev)
{
    extDevice_t *dev = &magDev->dev;
    busDeviceRegister(dev);

    bool ack = true;

    ack = ack && busWriteRegister(dev, RM3100_REG_TMRC, TMRC_DEFAULT);

    ack = ack && busWriteRegister(dev, RM3100_REG_CCX1, CCX_DEFAULT_MSB);
    ack = ack && busWriteRegister(dev, RM3100_REG_CCX0, CCX_DEFAULT_LSB);
    
    ack = ack && busWriteRegister(dev, RM3100_REG_CCY1, CCY_DEFAULT_MSB);
    ack = ack && busWriteRegister(dev, RM3100_REG_CCY0, CCY_DEFAULT_LSB);
    
    ack = ack && busWriteRegister(dev, RM3100_REG_CCZ1, CCZ_DEFAULT_MSB);
    ack = ack && busWriteRegister(dev, RM3100_REG_CCZ0, CCZ_DEFAULT_LSB);

    ack = ack && busWriteRegister(dev, RM3100_REG_CMM, CMM_CONT_MODE);


    if (!ack) {
        return false;
    }

    magDev->magOdrHz = 100; 
    return true;
}

static bool rm3100Read(magDev_t *magDev, int16_t *magData)
{
    #pragma pack(push, 1)
    static struct {
        uint8_t x[3];
        uint8_t y[3];
        uint8_t z[3];
    } rm_report;
    #pragma pack(pop)
    
    static uint8_t status = 0;
    static enum {
        STATE_TRIGGER_STATUS,
        STATE_AWAIT_STATUS,
        STATE_AWAIT_DATA,
    } state = STATE_TRIGGER_STATUS;

    extDevice_t *dev = &magDev->dev;

    switch (state) {
        default:
        case STATE_TRIGGER_STATUS:
            status = 0;
            if(busReadRegisterBufferStart(dev, RM3100_REG_STATUS, &status, 1)){
                state = STATE_AWAIT_STATUS;
            }
            return false;
        case STATE_AWAIT_STATUS:
        if (status == 0xFF) {
                state = STATE_TRIGGER_STATUS;
                return false;
            }
            if(status & 0x80){
                if (busReadRegisterBufferStart(dev, RM3100_REG_MX, (uint8_t *)&rm_report, sizeof(rm_report))) {
                    state = STATE_AWAIT_DATA;
                }else{
                    state = STATE_TRIGGER_STATUS;
                }        
            }else {
                state = STATE_TRIGGER_STATUS;
            }
            return false;

        case STATE_AWAIT_DATA:
        {
            int32_t xraw = (int32_t)(((uint32_t)rm_report.x[0] << 24) | ((uint32_t)rm_report.x[1] << 16) | ((uint32_t)rm_report.x[2] << 8)) >> 8;
            int32_t yraw = (int32_t)(((uint32_t)rm_report.y[0] << 24) | ((uint32_t)rm_report.y[1] << 16) | ((uint32_t)rm_report.y[2] << 8)) >> 8;
            int32_t zraw = (int32_t)(((uint32_t)rm_report.z[0] << 24) | ((uint32_t)rm_report.z[1] << 16) | ((uint32_t)rm_report.z[2] << 8)) >> 8;

            magData[X] = (int16_t)constrain(xraw / 10, INT16_MIN, INT16_MAX);
            magData[Y] = (int16_t)constrain(yraw / 10, INT16_MIN, INT16_MAX);
            magData[Z] = (int16_t)constrain(zraw / 10, INT16_MIN, INT16_MAX);

            state = STATE_TRIGGER_STATUS;
            return true;
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

    bool ack = busReadRegisterBuffer(dev, RM3100_REG_REVID, &revid, 1);
    
    if (ack && revid == RM3100_REVID) {
        magDev->init = rm3100Init;
        magDev->read = rm3100Read;
        return true;
    }
    
    return false;
}
#endif
