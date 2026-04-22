#include <stdbool.h>
#include <stdint.h>
#include "platform.h"

#ifdef USE_MAG_RM3100

#include "common/axis.h"
#include "common/maths.h"
#include "drivers/bus.h"
#include "drivers/sensor.h"
#include "drivers/time.h"
#include "compass.h"

// RM3100 Registers
#define RM3100_REG_POLL        0x00
#define RM3100_REG_CMM         0x01
#define RM3100_REG_CCX1        0x04
#define RM3100_REG_TMRC        0x0B 
#define RM3100_REG_MX          0x24 
#define RM3100_REG_STATUS      0x34
#define RM3100_REG_REVID       0x36

#define RM3100_REVID           0x22
#define RM3100_MAG_I2C_ADDRESS 0x20

#define CC_DEFAULT             200 
#define TMRC_600HZ             0x92 
#define CMM_CONT_MODE          0x79 

static uint8_t rm3100_read_state = 0;

static bool rm3100Init(magDev_t *mag)
{
    extDevice_t *dev = &mag->dev;
    busDeviceRegister(dev);

    uint8_t cc[2] = { (CC_DEFAULT >> 8) & 0xFF, CC_DEFAULT & 0xFF };
    busWriteRegister(dev, RM3100_REG_CCX1, cc[0]); // CCX1
    busWriteRegister(dev, RM3100_REG_CCX1 + 1, cc[1]); // CCX0
    busWriteRegister(dev, RM3100_REG_CCX1 + 2, cc[0]); // CCY1
    busWriteRegister(dev, RM3100_REG_CCX1 + 3, cc[1]); // CCY0
    busWriteRegister(dev, RM3100_REG_CCX1 + 4, cc[0]); // CCZ1
    busWriteRegister(dev, RM3100_REG_CCX1 + 5, cc[1]); // CCZ0


    busWriteRegister(dev, RM3100_REG_TMRC, TMRC_600HZ);

    busWriteRegister(dev, RM3100_REG_CMM, CMM_CONT_MODE);

    mag->magOdrHz = 100; 
    return true;
}

static bool rm3100Read(magDev_t *mag, int16_t *magData)
{
    extDevice_t *dev = &mag->dev;

    if (rm3100_read_state == 0) {
        uint8_t status;
        
        if (busReadRegisterBuffer(dev, RM3100_REG_STATUS, &status, 1)) {
            if ((status & 0x80) != 0) {
                rm3100_read_state = 1; 
            }
        }
        
        return false; 
    } 
    
    else if (rm3100_read_state == 1) {
        uint8_t buf[9]; // 3 axes * 3 bytes each

        rm3100_read_state = 0; 
        
        if (!busReadRegisterBuffer(dev, RM3100_REG_MX, (uint8_t*)&buf, sizeof(buf))) {
            return false; 
        }

        int32_t xraw = (int32_t)(((uint32_t)buf[0] << 24) | ((uint32_t)buf[1] << 16) | ((uint32_t)buf[2] << 8)) >> 8;
        int32_t yraw = (int32_t)(((uint32_t)buf[3] << 24) | ((uint32_t)buf[4] << 16) | ((uint32_t)buf[5] << 8)) >> 8;
        int32_t zraw = (int32_t)(((uint32_t)buf[6] << 24) | ((uint32_t)buf[7] << 16) | ((uint32_t)buf[8] << 8)) >> 8;

        magData[X] = (int16_t)(xraw / 10);
        magData[Y] = (int16_t)(yraw / 10);
        magData[Z] = (int16_t)(zraw / 10);
        

        return true; 
    }
    
    return false;
}

bool rm3100Detect(magDev_t *magDev)
{
    extDevice_t *dev = &magDev->dev;
    uint8_t revid;

    if (dev->bus->busType == BUS_TYPE_I2C && dev->busType_u.i2c.address == 0) {
        dev->busType_u.i2c.address = RM3100_MAG_I2C_ADDRESS;
    }

    if (busReadRegisterBuffer(dev, RM3100_REG_REVID, &revid, 1) && revid == RM3100_REVID) {
        magDev->init = rm3100Init;
        magDev->read = rm3100Read;
        return true;
    }
    return false;
}
#endif
