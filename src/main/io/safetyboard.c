
#include <stdbool.h>
#include <stdint.h>
//#include <ctype.h>
#include <string.h>

#include "platform.h"

#include "io/safetyboard.h"

#ifdef USE_SAFETYBOARD

#include "io/serial.h"

//#include "drivers/osd.h"
#include "osd/osd.h"
#include "osd/osd_elements.h"
#include "osd/osd_warnings.h"

#include "common/printf.h"
#include "common/crc.h"

#include "drivers/time.h"

//#include "pg/pilot.h"
#include "config/config.h"

#define SB_PORT_OPTIONS               (SERIAL_STOPBITS_1 | SERIAL_PARITY_NO | SERIAL_BIDIR /*| SERIAL_BIDIR_PP*/)

#define MAX_FRAME_TIME_MS                50000 // 50ms
#define FRAME_TIMEOUT_TIME_MS           500000 // 0.5s


// RX buffer
safetyboard_frame_t rx_buffer;
safetyboard_frame_t* incomingFrame = &rx_buffer;

// Device serial port instance
static serialPort_t *safeteyBoardSerialPort = NULL;

static bool payload_detected = false;

static const char default_lines[2][17] =  {
    {" NO PAYLD       "},
    {"                "},
};

// Serial transmit and receive buffers
//static uint8_t safetyBoardReqBuffer[16];
//static uint8_t safetyBoardRespBuffer[16];


STATIC_UNIT_TESTED uint8_t SBFrameCRC(const safetyboard_frame_t *const SBframe)
{
    // CRC includes type and payload
    uint8_t crc = 0;
    for (int i = 0; i < PCKT_LEN - 1; ++i) {
        crc = crc8_dvb_s2(crc, SBframe->bytes[i]);
    }
    return crc;
}

// Receive ISR callback, called back from serial port
void SafetyBoardDataReceive(void)
{
    if (!safeteyBoardSerialPort) {
        return;
    }
    static timeUs_t SBFrameStartAtUs = 0;
    char * textVar;
    unsigned textSpace;
    static uint8_t frameIdx = 0;
    const timeUs_t currentTimeUs = microsISR();

    //static uint8_t cnt1 = 0,cnt2 = 0;


    if (cmpTimeUs(currentTimeUs, SBFrameStartAtUs) > MAX_FRAME_TIME_MS) {
        // Character received after the max. frame time, assume that this is a new frame
        frameIdx = 0;
    }
    if(cmpTimeUs(currentTimeUs, SBFrameStartAtUs) > FRAME_TIMEOUT_TIME_MS){
        payload_detected = false;
    }
    //cnt2++;
    while(serialRxBytesWaiting(safeteyBoardSerialPort))
    {
        uint8_t c = serialRead(safeteyBoardSerialPort);
        //cnt1++;
        if (frameIdx == 0) {
            // timestamp the start of the frame, to allow us to detect frame sync issues
            SBFrameStartAtUs = currentTimeUs;
        }

        if (frameIdx < PCKT_LEN) {
            incomingFrame->bytes[frameIdx++] = (uint8_t)c;
            if (frameIdx >= PCKT_LEN) {
                
                frameIdx = 0;
                // check header and crc
                uint8_t crc = SBFrameCRC(incomingFrame);
                if (incomingFrame->frame.header == PCKT_HEADER && crc == incomingFrame->frame.crc) {
                    payload_detected = true;
                    //for(int i = 0; i < 2; i++)
                    {
                        textVar = pilotConfigMutable()->pilotName;
                        textSpace = sizeof(pilotConfigMutable()->pilotName) - 1;
                        memcpy(textVar, incomingFrame->frame.lines[0], PAYLOAD_LEN);
                        textVar[textSpace] = '\0';

                        textVar = pilotConfigMutable()->craftName;
                        textSpace = sizeof(pilotConfigMutable()->craftName) - 1;
                        memcpy(textVar, incomingFrame->frame.lines[1], PAYLOAD_LEN);
                        textVar[textSpace] = '\0';
                    }
                }
            }
        }
    }

    if(!payload_detected)
    {
        //for(int i = 0; i < 2; i++)
        {
            textVar = pilotConfigMutable()->pilotName;
            textSpace = sizeof(pilotConfigMutable()->pilotName) - 1;
            memcpy(textVar, default_lines[0], PAYLOAD_LEN);
            textVar[textSpace] = '\0';

            textVar = pilotConfigMutable()->craftName;
            textSpace = sizeof(pilotConfigMutable()->craftName) - 1;
            //tfp_sprintf(textVar, "%03d %03d %03d  ",frameIdx,cnt1, cnt2  );
            memcpy(textVar, default_lines[1], PAYLOAD_LEN);
            textVar[textSpace] = '\0';
        }
    }
}


bool safetyBoardInit(void){
    const serialPortConfig_t *portConfig = findSerialPortConfig(FUNCTION_SAFETYBOARD);

    if (portConfig) {
        safeteyBoardSerialPort = openSerialPort(portConfig->identifier, FUNCTION_SAFETYBOARD, NULL, NULL, 19200, MODE_RX, SB_PORT_OPTIONS);
    }

    if (!safeteyBoardSerialPort) {
        return false;
        char * textVar;
        unsigned textSpace;
        static const char meow[] = {"MEOW            "};
        textVar = pilotConfigMutable()->pilotName;
            textSpace = sizeof(pilotConfigMutable()->pilotName) - 1;
            memcpy(textVar, default_lines[0], PAYLOAD_LEN);
            textVar[textSpace] = '\0';

            textVar = pilotConfigMutable()->craftName;
            textSpace = sizeof(pilotConfigMutable()->craftName) - 1;
            memcpy(textVar, meow, PAYLOAD_LEN);
            textVar[textSpace] = '\0';
    }
    return true;
}

/*void safetyBoardProccess(void){

    char * textVar;
    unsigned textSpace;
    uint8_t msgIdx;



    textVar = pilotConfigMutable()->message[msgIdx];
    textSpace = sizeof(pilotConfigMutable()->message[msgIdx]) - 1;
    memcpy(textVar, messageStr, textSpace);
    textVar[textSpace] = '\0';

}*/



#endif // USE_SAFETYBOARD