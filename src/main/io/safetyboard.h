
//#define USE_SAFETYBOARD
bool safetyBoardInit(void);
void SafetyBoardDataReceive(void);

// packet definition for sending the osd lines to FC
#define PCKT_LEN				0x24 // 36 bytes total, fixed
#define PAYLOAD_LEN				17	 // 16 chars + null termination
									 // PACKET contents:
#define PCKT_HEADER				0x69 // header 		1 byte
									 // payload_1	17bytes (16 user chars + null termination)
									 // payload_2	17bytes (16 user chars + null termination)
									 // crc			1 byte  (crc calculated excluding itself)
typedef struct{
    uint8_t header;
    //uint8_t lineIdx;
    char lines[2][PAYLOAD_LEN];
    uint8_t crc;
}safetyboard_frame;

typedef union{
    safetyboard_frame frame;
    uint8_t bytes[PCKT_LEN];
}safetyboard_frame_t;