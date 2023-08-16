/*
 * DESIoT_device.h
 *
 *  Created on: Jun 20, 2023
 *      Author: ADMIN
 */

#ifndef INC_DESIOT_DEVICE_H_
#define INC_DESIOT_DEVICE_H_

#include <stdint.h>
#include <stddef.h>
#include <string.h>

// DESIoT virtual storages
#define DESIOT_VS0 0x0u
#define DESIOT_VS1 0x1u
#define DESIOT_VS2 0x2u
#define DESIOT_VS3 0x3u
#define DESIOT_VS4 0x4u
#define DESIOT_VS5 0x5u
#define DESIOT_VS6 0x6u
#define DESIOT_VS7 0x7u
#define DESIOT_VS8 0x8u
#define DESIOT_VS9 0x9u
#define DESIOT_VS10 0xAu
#define DESIOT_VS11 0xBu
#define DESIOT_VS12 0xCu
#define DESIOT_VS13 0xDu
#define DESIOT_VS14 0xEu
#define DESIOT_VS15 0xFu
#define DESIOT_VS16 0x10u
#define DESIOT_VS17 0x11u
#define DESIOT_VS18 0x12u
#define DESIOT_VS19 0x13u
#define DESIOT_VS20 0x14u
#define DESIOT_VS21 0x15u
#define DESIOT_VS22 0x16u
#define DESIOT_VS23 0x17u
#define DESIOT_VS24 0x18u
#define DESIOT_VS25 0x19u
#define DESIOT_VS26 0x1Au
#define DESIOT_VS27 0x1Bu
#define DESIOT_VS28 0x1Cu
#define DESIOT_VS29 0x1Du
#define DESIOT_VS30 0x1Eu
#define DESIOT_VS31 0x1Fu

// Lengths of packet types
#define DESIOT_H1_LEN 1u
#define DESIOT_H2_LEN 1u
#define DESIOT_HEAD_LEN (DESIOT_H1_LEN + DESIOT_H2_LEN)

#define DESIOT_T1_LEN 1u
#define DESIOT_T2_LEN 1u
#define DESIOT_TRAIL_LEN (DESIOT_T1_LEN + DESIOT_T2_LEN)

#define DESIOT_HEADERS_LEN (DESIOT_HEAD_LEN + DESIOT_TRAIL_LEN)

#define DESIOT_CRC_LEN 2u

#define DESIOT_CMD_LEN 0x1u
#define DESIOT_DATALEN_LEN 0x2u

#define DESIOT_HEAD_FRAME_LEN (DESIOT_HEAD_LEN + DESIOT_CMD_LEN + DESIOT_DATALEN_LEN)

#define DESIOT_CONFIG_ID_SIZE 25u
#define DESIOT_DEVICE_ID_SIZE 25u
#define DESIOT_ADDITIONAL_DATA_SIZE (DESIOT_CONFIG_ID_SIZE + DESIOT_DEVICE_ID_SIZE)

// Length of packet assigning integer
#define DESIOT_VS_ID_LEN 0x1u
#define DESIOT_VS_INTEGER_LEN 0x4u
#define DESIOT_ASSIGN_INT_PKG_LEN (DESIOT_VS_ID_LEN + DESIOT_VS_INTEGER_LEN)

// DESIoT default values
#define DESIOT_H1_DEFAULT 0x7u
#define DESIOT_H2_DEFAULT 0x17u
#define DESIOT_T1_DEFAULT 0x70u
#define DESIOT_T2_DEFAULT 0x71u

#define DESIOT_SENDBYTES_F_NAME DESIoT_sendBytes
#define DESIOT_SENDBYTES \
		void DESIOT_SENDBYTES_F_NAME (size_t DESIOT_ATT_UNUSED size, uint8_t DESIOT_ATT_UNUSED *bytes)

// attributes
#define DESIOT_ATT_PACKED __attribute__ ((__packed__))
#define DESIOT_ATT_UNUSED __attribute__((__unused__))
#define DESIOT_ATT_WEAK __attribute__((weak))

// VS SYNC EXECs
#define DESIOT_MAX_VSSYNC_PAYLOAD_SIZE UINT8_MAX
#define DESIOT_DEF_EXEC_SYNC(VS) \
	void DESIoT_execSync ## VS (char DESIOT_ATT_UNUSED *payload, size_t DESIOT_ATT_UNUSED len)
#define DESIOT_EXEC_SYNC(VS, payload, len) \
	DESIoT_execSync ## VS (payload, len)

// CRC
#define DESIOT_CRC_GENERATOR 0x1305

// UART
#define DESIOT_CIR_BUF_SIZE 1024u

// Circular buffer
typedef struct
{
	uint16_t start:10;
	uint16_t end:10;
	uint16_t startRestore : 10;
	uint8_t buffer[DESIOT_CIR_BUF_SIZE];
} DESIoT_CBUF_t;
enum DESIOT_CBUF_STATUS
{
	DESIOT_CBUF_OK,
	DESIOT_CBUF_ERROR
};
uint8_t DESIoT_CBUF_getByte(DESIoT_CBUF_t *hCBuf, uint8_t *rx);
void DESIoT_CBUF_putByte(DESIoT_CBUF_t *hCBuf, uint8_t rx);
uint8_t DESIoT_CBUF_isEmpty(DESIoT_CBUF_t *hCBuf);

// millis
#define DESIOT_MILLIS_F_NAME DESIoT_millis
#define DESIOT_MILLIS \
		uint32_t DESIOT_MILLIS_F_NAME ()

enum DESIoTCMD {
	DESIOT_CMD_ASSIGN_VIRTUAL_STORAGE,
	DESIOT_CMD_SYNC_VIRTUAL_STORAGE,
	DESIOT_CMD_READ_VIRTUAL_STORAGE
};

typedef struct {
	uint8_t h1;
	uint8_t h2;
	uint8_t cmd;
	uint16_t dataLen : 10;
}DESIOT_ATT_PACKED DESIoT_headFrame_t;

typedef struct {
	uint8_t t1;
	uint8_t t2;
	uint16_t crc;
} DESIOT_ATT_PACKED DESIoT_trailFrame_t;

typedef struct{
	uint8_t cmd;
	union
	{
		uint8_t dataLenArr[2];
		uint16_t dataLen : 10;
	};
	uint8_t data[UINT16_MAX & 0x3FFu];
} DESIOT_ATT_PACKED DESIoT_dataPacket_t;

typedef struct
{
	uint8_t h1;
	uint8_t h2;
	DESIoT_dataPacket_t dataPacket;
	uint8_t t1;
	uint8_t t2;
	union
	{
		uint16_t crc;
		uint8_t crcArr[2];
	};
} DESIOT_ATT_PACKED DESIoT_Frame_t;

typedef struct{
	char device_id[DESIOT_DEVICE_ID_SIZE];
	char config_id[DESIOT_CONFIG_ID_SIZE];
	uint8_t status;
	uint8_t index;
	uint32_t millis;
	DESIoT_Frame_t frame;

	DESIoT_CBUF_t *curCBuf;
} DESIoT_Frame_Hander_t;

#define DESIOT_SET_FRAME_FAILED_STATUS(status) status--
#define DESIOT_SET_FRAME_SUCCESS_STATUS(status) status -= 2
#define DESIOT_IS_FRAME_ON_PROCESS_STATUS(status) ((status != DESIOT_FRAME_IDLE) && !(status % 3))
#define DESIOT_TIMEOUT_DURATION 1000u

#define DESIOT_STARTUP_TIME 3000u

enum DESIOT_FRAME_STATUSES
{
	DESIOT_FRAME_IDLE,
	//
	DESIOT_FRAME_GATEWAY_SUCCESS,
	DESIOT_FRAME_GATEWAY_FAILED,
	DESIOT_FRAME_IN_GATEWAY_PROGRESS,
};

enum DESIOT_HEAD_FRAME_INDEXES
{
	DESIOT_H1_INDEX,
	DESIOT_H2_INDEX,
	DESIOT_CMD_INDEX,
	DESIOT_DATALEN_INDEX
};

extern DESIoT_Frame_Hander_t hFrame;

void DESIoT_loop();
void DESIoT_frameArbitrating();
void DESIoT_frameProssessLoop();
void DESIOT_Rx1byte(uint8_t rxByte);
void DESIoT_assignInt(uint8_t VS, size_t integer);
void DESIoT_assignFloat(uint8_t VS, float fNumber);
void DESIoT_readVS(uint8_t VS);
void DESIoT_sendDataPacket(const size_t dataLen, uint8_t *data);
void DESIoT_CalculateTable_CRC16();
uint16_t DESIoT_Compute_CRC16(uint8_t *bytes, const int32_t BYTES_LEN);

void DESIoT_setUpStartOfParsing(DESIoT_Frame_Hander_t *hFrame, DESIoT_CBUF_t *curCBuf);
void DESIoT_FRAME_parsing(DESIoT_Frame_Hander_t *hFrame, uint8_t byte, DESIoT_CBUF_t *curCBuf);
void DESIoT_frameFailedHandler();
void DESIoT_frameSuccessHandler();
void DESIoT_restartFrameIndexes();
void DESIoT_restartCBufIndexes();
void DESIoT_frameTimeoutHandler();
void DESIoT_execSuccessfulFrame();
void DESIoT_execVSyncWF();
uint32_t DESIoT_millis();

// static funcs
static void DESIoT_begin() {
	while(DESIoT_millis() < DESIOT_STARTUP_TIME);
#if defined(DESIOT_DEVICE_ID) && defined(DESIOT_CONFIG_ID)
	strcpy(hFrame.device_id, DESIOT_DEVICE_ID);
	strcpy(hFrame.config_id, DESIOT_CONFIG_ID);
#endif
}

// weak functions
DESIOT_SENDBYTES;

DESIOT_DEF_EXEC_SYNC(DESIOT_VS0);
DESIOT_DEF_EXEC_SYNC(DESIOT_VS1);
DESIOT_DEF_EXEC_SYNC(DESIOT_VS2);
DESIOT_DEF_EXEC_SYNC(DESIOT_VS3);
DESIOT_DEF_EXEC_SYNC(DESIOT_VS4);
DESIOT_DEF_EXEC_SYNC(DESIOT_VS5);
DESIOT_DEF_EXEC_SYNC(DESIOT_VS6);
DESIOT_DEF_EXEC_SYNC(DESIOT_VS7);
DESIOT_DEF_EXEC_SYNC(DESIOT_VS8);
DESIOT_DEF_EXEC_SYNC(DESIOT_VS9);
DESIOT_DEF_EXEC_SYNC(DESIOT_VS10);
DESIOT_DEF_EXEC_SYNC(DESIOT_VS11);
DESIOT_DEF_EXEC_SYNC(DESIOT_VS12);
DESIOT_DEF_EXEC_SYNC(DESIOT_VS13);
DESIOT_DEF_EXEC_SYNC(DESIOT_VS14);
DESIOT_DEF_EXEC_SYNC(DESIOT_VS15);
DESIOT_DEF_EXEC_SYNC(DESIOT_VS16);
DESIOT_DEF_EXEC_SYNC(DESIOT_VS17);
DESIOT_DEF_EXEC_SYNC(DESIOT_VS18);
DESIOT_DEF_EXEC_SYNC(DESIOT_VS19);
DESIOT_DEF_EXEC_SYNC(DESIOT_VS20);
DESIOT_DEF_EXEC_SYNC(DESIOT_VS21);
DESIOT_DEF_EXEC_SYNC(DESIOT_VS22);
DESIOT_DEF_EXEC_SYNC(DESIOT_VS23);
DESIOT_DEF_EXEC_SYNC(DESIOT_VS24);
DESIOT_DEF_EXEC_SYNC(DESIOT_VS25);
DESIOT_DEF_EXEC_SYNC(DESIOT_VS26);
DESIOT_DEF_EXEC_SYNC(DESIOT_VS27);
DESIOT_DEF_EXEC_SYNC(DESIOT_VS28);
DESIOT_DEF_EXEC_SYNC(DESIOT_VS29);
DESIOT_DEF_EXEC_SYNC(DESIOT_VS30);
DESIOT_DEF_EXEC_SYNC(DESIOT_VS31);
#endif /* INC_DESIOT_DEVICE_H_ */
