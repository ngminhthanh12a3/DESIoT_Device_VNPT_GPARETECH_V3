/*
 * DESIoT_device.c
 *
 *  Created on: Jun 20, 2023
 *      Author: ADMIN
 */


#include "DESIoT_device.h"

DESIoT_Frame_Hander_t hFrame = {.index = 0};
DESIoT_CBUF_t hGatewayCBuffer = {.start = 0, .end = 0};

void DESIoT_loop() {
	DESIoT_frameArbitrating();
	DESIoT_frameProssessLoop();
}

void DESIoT_frameProssessLoop()
{
	while (DESIOT_IS_FRAME_ON_PROCESS_STATUS(hFrame.status))
	{
		DESIoT_frameTimeoutHandler();

		uint8_t rx;
		if (DESIoT_CBUF_getByte(hFrame.curCBuf, &rx) == DESIOT_CBUF_OK)
			DESIoT_FRAME_parsing(&hFrame, rx, hFrame.curCBuf);
	}
	DESIoT_frameFailedHandler();
	DESIoT_frameSuccessHandler();
}

void DESIoT_frameArbitrating()
{
	// arbitrating for Gateway
	if (hFrame.status == DESIOT_FRAME_IDLE)
	{
		if (!DESIoT_CBUF_isEmpty(&hGatewayCBuffer))
		{
			hFrame.status = DESIOT_FRAME_IN_GATEWAY_PROGRESS;
			DESIoT_setUpStartOfParsing(&hFrame, &hGatewayCBuffer);
		}
	}
}

void DESIOT_Rx1byte(uint8_t rxByte)
{
	DESIoT_CBUF_putByte(&hGatewayCBuffer, rxByte);
}

/**
 * Send a data packet requiring to assign an integer to the gateway.
 */
void DESIoT_assignInt(uint8_t VS, size_t integer)
{
	DESIoT_dataPacket_t dataPacket;
	dataPacket.cmd = DESIOT_CMD_ASSIGN_VIRTUAL_STORAGE;
	dataPacket.dataLen = DESIOT_ADDITIONAL_DATA_SIZE + sizeof(VS) + sizeof(integer); // add bytes of additional data

	// ignore 12-byte data for device ID
	dataPacket.data[DESIOT_ADDITIONAL_DATA_SIZE] = VS;
	memcpy(dataPacket.data + DESIOT_ADDITIONAL_DATA_SIZE + 1, &integer, sizeof(integer));

	DESIoT_sendDataPacket(DESIOT_CMD_LEN + DESIOT_DATALEN_LEN + dataPacket.dataLen,
			(uint8_t*)&dataPacket);
}

void DESIoT_assignFloat(uint8_t VS, float fNumber)
{
	DESIoT_dataPacket_t dataPacket;
	dataPacket.cmd = DESIOT_CMD_ASSIGN_VIRTUAL_STORAGE;
	dataPacket.dataLen = DESIOT_ADDITIONAL_DATA_SIZE + sizeof(VS) + sizeof(fNumber); // add bytes of additional data

	// ignore 12-byte data for device ID
	dataPacket.data[DESIOT_ADDITIONAL_DATA_SIZE] = VS;
	memcpy(dataPacket.data + DESIOT_ADDITIONAL_DATA_SIZE + 1, &fNumber, sizeof(fNumber));

	DESIoT_sendDataPacket(DESIOT_CMD_LEN + DESIOT_DATALEN_LEN + dataPacket.dataLen,
			(uint8_t*)&dataPacket);
}

void DESIoT_readVS(uint8_t VS)
{
	DESIoT_dataPacket_t dataPacket;
	dataPacket.cmd = DESIOT_CMD_READ_VIRTUAL_STORAGE;
	dataPacket.dataLen = DESIOT_ADDITIONAL_DATA_SIZE + sizeof(VS); // add bytes of additional data

	// ignore 12-byte data for device ID
	dataPacket.data[DESIOT_ADDITIONAL_DATA_SIZE] = VS;

	DESIoT_sendDataPacket(DESIOT_CMD_LEN + DESIOT_DATALEN_LEN + dataPacket.dataLen,
			(uint8_t*)&dataPacket);
}

extern uint8_t testMode;
/**
 *
 */
void DESIoT_sendDataPacket(const size_t dataLen, uint8_t *data)
{
	uint8_t frame[DESIOT_HEADERS_LEN + dataLen + DESIOT_CRC_LEN];
	memset(frame, 0, sizeof(frame));

	DESIoT_headFrame_t *headFrame = (DESIoT_headFrame_t*)frame;
	DESIoT_trailFrame_t *trailFrame = (DESIoT_trailFrame_t*)(frame + sizeof(frame) - (DESIOT_TRAIL_LEN + DESIOT_CRC_LEN));

	headFrame->h1 = DESIOT_H1_DEFAULT;
	headFrame->h2 = DESIOT_H2_DEFAULT;
	trailFrame->t1 = DESIOT_T1_DEFAULT;
	trailFrame->t2 = DESIOT_T2_DEFAULT;

	{
		size_t i = DESIOT_HEAD_LEN;
		// set data packet to frame
		memcpy(frame + i, data, dataLen);

		// set config ID
		i += (DESIOT_CMD_LEN + DESIOT_DATALEN_LEN);
		memcpy(frame + i, hFrame.config_id, DESIOT_CONFIG_ID_SIZE);
		// set device ID
		i += DESIOT_CONFIG_ID_SIZE;
		memcpy(frame + i, hFrame.device_id, DESIOT_DEVICE_ID_SIZE);
	}


	trailFrame->crc = DESIoT_Compute_CRC16(frame + DESIOT_HEAD_LEN, dataLen);

	if(testMode && !data[3 + DESIOT_ADDITIONAL_DATA_SIZE])
	{
		uint16_t randByte = rand() % (sizeof(frame) - 1);
		frame[randByte] = !frame[randByte];
	}


	DESIOT_SENDBYTES_F_NAME(sizeof(frame), frame);
}

DESIOT_ATT_WEAK DESIOT_SENDBYTES
{

}

unsigned short crctable16[256] = {0x0000, 0x1305, 0x260A, 0x350F, 0x4C14, 0x5F11, 0x6A1E, 0x791B, 0x9828, 0x8B2D, 0xBE22, 0xAD27, 0xD43C, 0xC739, 0xF236, 0xE133, 0x2355, 0x3050, 0x055F, 0x165A, 0x6F41, 0x7C44, 0x494B, 0x5A4E, 0xBB7D, 0xA878, 0x9D77, 0x8E72, 0xF769, 0xE46C, 0xD163, 0xC266, 0x46AA, 0x55AF, 0x60A0, 0x73A5, 0x0ABE, 0x19BB, 0x2CB4, 0x3FB1, 0xDE82, 0xCD87, 0xF888, 0xEB8D, 0x9296, 0x8193, 0xB49C, 0xA799, 0x65FF, 0x76FA, 0x43F5, 0x50F0, 0x29EB, 0x3AEE, 0x0FE1, 0x1CE4, 0xFDD7, 0xEED2, 0xDBDD, 0xC8D8, 0xB1C3, 0xA2C6, 0x97C9, 0x84CC, 0x8D54, 0x9E51, 0xAB5E, 0xB85B, 0xC140, 0xD245, 0xE74A, 0xF44F, 0x157C, 0x0679, 0x3376, 0x2073, 0x5968, 0x4A6D, 0x7F62, 0x6C67, 0xAE01, 0xBD04, 0x880B, 0x9B0E, 0xE215, 0xF110, 0xC41F, 0xD71A, 0x3629, 0x252C, 0x1023, 0x0326, 0x7A3D, 0x6938, 0x5C37, 0x4F32, 0xCBFE, 0xD8FB, 0xEDF4, 0xFEF1, 0x87EA, 0x94EF, 0xA1E0, 0xB2E5, 0x53D6, 0x40D3, 0x75DC, 0x66D9, 0x1FC2, 0x0CC7, 0x39C8, 0x2ACD, 0xE8AB, 0xFBAE, 0xCEA1, 0xDDA4, 0xA4BF, 0xB7BA, 0x82B5, 0x91B0, 0x7083, 0x6386, 0x5689, 0x458C, 0x3C97, 0x2F92, 0x1A9D, 0x0998, 0x09AD, 0x1AA8, 0x2FA7, 0x3CA2, 0x45B9, 0x56BC, 0x63B3, 0x70B6, 0x9185, 0x8280, 0xB78F, 0xA48A, 0xDD91, 0xCE94, 0xFB9B, 0xE89E, 0x2AF8, 0x39FD, 0x0CF2, 0x1FF7, 0x66EC, 0x75E9, 0x40E6, 0x53E3, 0xB2D0, 0xA1D5, 0x94DA, 0x87DF, 0xFEC4, 0xEDC1, 0xD8CE, 0xCBCB, 0x4F07, 0x5C02, 0x690D, 0x7A08, 0x0313, 0x1016, 0x2519, 0x361C, 0xD72F, 0xC42A, 0xF125, 0xE220, 0x9B3B, 0x883E, 0xBD31, 0xAE34, 0x6C52, 0x7F57, 0x4A58, 0x595D, 0x2046, 0x3343, 0x064C, 0x1549, 0xF47A, 0xE77F, 0xD270, 0xC175, 0xB86E, 0xAB6B, 0x9E64, 0x8D61, 0x84F9, 0x97FC, 0xA2F3, 0xB1F6, 0xC8ED, 0xDBE8, 0xEEE7, 0xFDE2, 0x1CD1, 0x0FD4, 0x3ADB, 0x29DE, 0x50C5, 0x43C0, 0x76CF, 0x65CA, 0xA7AC, 0xB4A9, 0x81A6, 0x92A3, 0xEBB8, 0xF8BD, 0xCDB2, 0xDEB7, 0x3F84, 0x2C81, 0x198E, 0x0A8B, 0x7390, 0x6095, 0x559A, 0x469F, 0xC253, 0xD156, 0xE459, 0xF75C, 0x8E47, 0x9D42, 0xA84D, 0xBB48, 0x5A7B, 0x497E, 0x7C71, 0x6F74, 0x166F, 0x056A, 0x3065, 0x2360, 0xE106, 0xF203, 0xC70C, 0xD409, 0xAD12, 0xBE17, 0x8B18, 0x981D, 0x792E, 0x6A2B, 0x5F24, 0x4C21, 0x353A, 0x263F, 0x1330, 0x0035};

void DESIoT_CalculateTable_CRC16()
{
	for (int divident = 0; divident < 256; divident++) /* iterate over all possible input byte values 0 - 255 */
	{
		unsigned short curByte = (unsigned short)(divident << 8); /* move divident byte into MSB of 16Bit CRC */
		for (unsigned char bit = 0; bit < 8; bit++)
		{
			if ((curByte & 0x8000) != 0)
			{
				curByte <<= 1;
				curByte ^= DESIOT_CRC_GENERATOR;
			}
			else
				curByte <<= 1;
		}

		crctable16[divident] = curByte;
	}
}

uint16_t DESIoT_Compute_CRC16(uint8_t *bytes, const int32_t BYTES_LEN)
{
	uint16_t crc = 0;

	for (int32_t i = 0; i < BYTES_LEN; i++)
	{
		uint8_t b = bytes[i];

		/* XOR-in next input byte into MSB of crc, that's our new intermediate divident */
		uint8_t pos = (uint8_t)((crc >> 8) ^ b);
		/* Shift out the MSB used for division per lookuptable and XOR with the remainder */
		crc = (uint16_t)((crc << 8) ^ (uint16_t)(crctable16[pos]));
	}
	//	printf("\nHERE 3");

	return crc;
}

void DESIoT_CBUF_putByte(DESIoT_CBUF_t *hCBuf, uint8_t rx)
{
	hCBuf->buffer[hCBuf->end++] = rx;
}

uint8_t DESIoT_CBUF_isEmpty(DESIoT_CBUF_t *hCBuf) {
	return hCBuf->end == hCBuf->start;
}

uint8_t DESIoT_CBUF_getByte(DESIoT_CBUF_t *hCBuf, uint8_t *rx)
{
	if (hCBuf->end != hCBuf->start)
	{
		*rx = hCBuf->buffer[hCBuf->start++];
		return DESIOT_CBUF_OK;
	}

	return DESIOT_CBUF_ERROR;
}

void DESIoT_setUpStartOfParsing(DESIoT_Frame_Hander_t *hFrame, DESIoT_CBUF_t *curCBuf)
{
	hFrame->millis = DESIoT_millis();
	hFrame->curCBuf = curCBuf;
	hFrame->curCBuf->startRestore = hFrame->curCBuf->start;
}

void DESIoT_FRAME_parsing(DESIoT_Frame_Hander_t *hFrame, uint8_t byte, DESIoT_CBUF_t *curCBuf)
{
	switch (hFrame->index)
	{
	case DESIOT_H1_INDEX:
		hFrame->curCBuf->startRestore = hFrame->curCBuf->start;
		if (byte == DESIOT_H1_DEFAULT)
			hFrame->frame.h1 = byte;

		else
			DESIOT_SET_FRAME_FAILED_STATUS(hFrame->status);
		break;
	case DESIOT_H2_INDEX:
		if (byte == DESIOT_H2_DEFAULT)
			hFrame->frame.h2 = byte;
		else
			DESIOT_SET_FRAME_FAILED_STATUS(hFrame->status);
		break;
	case DESIOT_CMD_INDEX:
		hFrame->frame.dataPacket.cmd = byte;
		break;
	case DESIOT_DATALEN_INDEX:
		hFrame->frame.dataPacket.dataLenArr[0] = byte;
		break;
	case DESIOT_DATALEN_INDEX + 1:
	hFrame->frame.dataPacket.dataLenArr[1] = byte;
	break;
	default:
		if (hFrame->index == (DESIOT_HEAD_FRAME_LEN + hFrame->frame.dataPacket.dataLen)) // t1
		{
			if (byte == DESIOT_T1_DEFAULT)
				hFrame->frame.t1 = byte;
			else
				DESIOT_SET_FRAME_FAILED_STATUS(hFrame->status);
		}
		else if (hFrame->index == (DESIOT_HEAD_FRAME_LEN + hFrame->frame.dataPacket.dataLen + 1)) // t2
		{
			if (byte == DESIOT_T2_DEFAULT)
				hFrame->frame.t2 = byte;
			else
				DESIOT_SET_FRAME_FAILED_STATUS(hFrame->status);
		}
		else if (hFrame->index == (DESIOT_HEAD_FRAME_LEN + hFrame->frame.dataPacket.dataLen + 2)) // crc1
		{
			hFrame->frame.crcArr[0] = byte;
		}
		else if (hFrame->index == (DESIOT_HEAD_FRAME_LEN + hFrame->frame.dataPacket.dataLen + 3)) // crc2
		{
			hFrame->frame.crcArr[1] = byte;
			uint16_t crcCalculate = DESIoT_Compute_CRC16((uint8_t *)&hFrame->frame.dataPacket, DESIOT_CMD_LEN + DESIOT_DATALEN_LEN + hFrame->frame.dataPacket.dataLen);
			if (crcCalculate == hFrame->frame.crc)
				DESIOT_SET_FRAME_SUCCESS_STATUS(hFrame->status);
			else
			{
				DESIOT_SET_FRAME_FAILED_STATUS(hFrame->status);
			}
		}
		else
		{
			hFrame->frame.dataPacket.data[hFrame->index - (DESIOT_HEAD_LEN + DESIOT_CMD_LEN + DESIOT_DATALEN_LEN)] = byte;
		}
		break;
	}
	hFrame->index++;
}


void DESIoT_frameFailedHandler()
{
	switch (hFrame.status)
	{
	case DESIOT_FRAME_GATEWAY_FAILED:
		DESIoT_restartFrameIndexes();
		DESIoT_restartCBufIndexes();
		break;
	}
}

void DESIoT_frameSuccessHandler()
{
	switch (hFrame.status)
	{
	case DESIOT_FRAME_GATEWAY_SUCCESS:

		DESIoT_execSuccessfulFrame();
		DESIoT_restartFrameIndexes();

		break;

	default:
		break;
	}
}

void DESIoT_restartFrameIndexes()
{
	hFrame.status = DESIOT_FRAME_IDLE;
	hFrame.index = 0;
}

void DESIoT_restartCBufIndexes()
{
	hFrame.curCBuf->start = hFrame.curCBuf->startRestore;

	// Flush to next headers
	uint16_t currentCBufEnd = hFrame.curCBuf->end;
	for (; hFrame.curCBuf->start != currentCBufEnd; hFrame.curCBuf->start++)
	{
		// check for H1 and H2 mathch
		if (hFrame.curCBuf->buffer[hFrame.curCBuf->start] == DESIOT_H1_DEFAULT && hFrame.curCBuf->buffer[hFrame.curCBuf->start + 1] == DESIOT_H2_DEFAULT)
			break;
	}
}

void DESIoT_frameTimeoutHandler()
{
	if (DESIOT_IS_FRAME_ON_PROCESS_STATUS(hFrame.status))
		if (DESIOT_MILLIS_F_NAME() - hFrame.millis > DESIOT_TIMEOUT_DURATION)
		{
			DESIoT_restartFrameIndexes();
			DESIoT_restartCBufIndexes();
		}
}

void DESIoT_execSuccessfulFrame() {
	switch (hFrame.frame.dataPacket.cmd) {
	case DESIOT_CMD_SYNC_VIRTUAL_STORAGE:
		DESIoT_execVSyncWF();
		break;
	default:
		break;
	}
}

void DESIoT_execVSyncWF() {
	char payload[DESIOT_MAX_VSSYNC_PAYLOAD_SIZE];
	uint8_t VSID = hFrame.frame.dataPacket.data[0];
	size_t len = hFrame.frame.dataPacket.dataLen - sizeof(VSID);

	memcpy(payload,hFrame.frame.dataPacket.data + 1, len);

	switch (VSID) {
	case DESIOT_VS0:
		DESIOT_EXEC_SYNC(DESIOT_VS0, payload, len);
		break;
	case DESIOT_VS1:
		DESIOT_EXEC_SYNC(DESIOT_VS1, payload, len);
		break;
	case DESIOT_VS2:
		DESIOT_EXEC_SYNC(DESIOT_VS2, payload, len);
		break;
	case DESIOT_VS3:
		DESIOT_EXEC_SYNC(DESIOT_VS3, payload, len);
		break;
	case DESIOT_VS4:
		DESIOT_EXEC_SYNC(DESIOT_VS4, payload, len);
		break;
	case DESIOT_VS5:
		DESIOT_EXEC_SYNC(DESIOT_VS5, payload, len);
		break;
	case DESIOT_VS6:
		DESIOT_EXEC_SYNC(DESIOT_VS6, payload, len);
		break;
	case DESIOT_VS7:
		DESIOT_EXEC_SYNC(DESIOT_VS7, payload, len);
		break;
	case DESIOT_VS8:
		DESIOT_EXEC_SYNC(DESIOT_VS8, payload, len);
		break;
	case DESIOT_VS9:
		DESIOT_EXEC_SYNC(DESIOT_VS9, payload, len);
		break;
	case DESIOT_VS10:
		DESIOT_EXEC_SYNC(DESIOT_VS10, payload, len);
		break;
	case DESIOT_VS11:
		DESIOT_EXEC_SYNC(DESIOT_VS11, payload, len);
		break;
	case DESIOT_VS12:
		DESIOT_EXEC_SYNC(DESIOT_VS12, payload, len);
		break;
	case DESIOT_VS13:
		DESIOT_EXEC_SYNC(DESIOT_VS13, payload, len);
		break;
	case DESIOT_VS14:
		DESIOT_EXEC_SYNC(DESIOT_VS14, payload, len);
		break;
	case DESIOT_VS15:
		DESIOT_EXEC_SYNC(DESIOT_VS15, payload, len);
		break;
	case DESIOT_VS16:
		DESIOT_EXEC_SYNC(DESIOT_VS16, payload, len);
		break;
	case DESIOT_VS17:
		DESIOT_EXEC_SYNC(DESIOT_VS17, payload, len);
		break;
	case DESIOT_VS18:
		DESIOT_EXEC_SYNC(DESIOT_VS18, payload, len);
		break;
	case DESIOT_VS19:
		DESIOT_EXEC_SYNC(DESIOT_VS19, payload, len);
		break;
	case DESIOT_VS20:
		DESIOT_EXEC_SYNC(DESIOT_VS20, payload, len);
		break;
	case DESIOT_VS21:
		DESIOT_EXEC_SYNC(DESIOT_VS21, payload, len);
		break;
	case DESIOT_VS22:
		DESIOT_EXEC_SYNC(DESIOT_VS22, payload, len);
		break;
	case DESIOT_VS23:
		DESIOT_EXEC_SYNC(DESIOT_VS23, payload, len);
		break;
	case DESIOT_VS24:
		DESIOT_EXEC_SYNC(DESIOT_VS24, payload, len);
		break;
	case DESIOT_VS25:
		DESIOT_EXEC_SYNC(DESIOT_VS25, payload, len);
		break;
	case DESIOT_VS26:
		DESIOT_EXEC_SYNC(DESIOT_VS26, payload, len);
		break;
	case DESIOT_VS27:
		DESIOT_EXEC_SYNC(DESIOT_VS27, payload, len);
		break;
	case DESIOT_VS28:
		DESIOT_EXEC_SYNC(DESIOT_VS28, payload, len);
		break;
	case DESIOT_VS29:
		DESIOT_EXEC_SYNC(DESIOT_VS29, payload, len);
		break;
	case DESIOT_VS30:
		DESIOT_EXEC_SYNC(DESIOT_VS30, payload, len);
		break;
	case DESIOT_VS31:
		DESIOT_EXEC_SYNC(DESIOT_VS31, payload, len);
		break;
	}

}

DESIOT_ATT_WEAK DESIOT_DEF_EXEC_SYNC(DESIOT_VS0) {}
DESIOT_ATT_WEAK DESIOT_DEF_EXEC_SYNC(DESIOT_VS1) {}
DESIOT_ATT_WEAK DESIOT_DEF_EXEC_SYNC(DESIOT_VS2) {}
DESIOT_ATT_WEAK DESIOT_DEF_EXEC_SYNC(DESIOT_VS3) {}
DESIOT_ATT_WEAK DESIOT_DEF_EXEC_SYNC(DESIOT_VS4) {}
DESIOT_ATT_WEAK DESIOT_DEF_EXEC_SYNC(DESIOT_VS5) {}
DESIOT_ATT_WEAK DESIOT_DEF_EXEC_SYNC(DESIOT_VS6) {}
DESIOT_ATT_WEAK DESIOT_DEF_EXEC_SYNC(DESIOT_VS7) {}
DESIOT_ATT_WEAK DESIOT_DEF_EXEC_SYNC(DESIOT_VS8) {}
DESIOT_ATT_WEAK DESIOT_DEF_EXEC_SYNC(DESIOT_VS9) {}
DESIOT_ATT_WEAK DESIOT_DEF_EXEC_SYNC(DESIOT_VS10) {}
DESIOT_ATT_WEAK DESIOT_DEF_EXEC_SYNC(DESIOT_VS11) {}
DESIOT_ATT_WEAK DESIOT_DEF_EXEC_SYNC(DESIOT_VS12) {}
DESIOT_ATT_WEAK DESIOT_DEF_EXEC_SYNC(DESIOT_VS13) {}
DESIOT_ATT_WEAK DESIOT_DEF_EXEC_SYNC(DESIOT_VS14) {}
DESIOT_ATT_WEAK DESIOT_DEF_EXEC_SYNC(DESIOT_VS15) {}
DESIOT_ATT_WEAK DESIOT_DEF_EXEC_SYNC(DESIOT_VS16) {}
DESIOT_ATT_WEAK DESIOT_DEF_EXEC_SYNC(DESIOT_VS17) {}
DESIOT_ATT_WEAK DESIOT_DEF_EXEC_SYNC(DESIOT_VS18) {}
DESIOT_ATT_WEAK DESIOT_DEF_EXEC_SYNC(DESIOT_VS19) {}
DESIOT_ATT_WEAK DESIOT_DEF_EXEC_SYNC(DESIOT_VS20) {}
DESIOT_ATT_WEAK DESIOT_DEF_EXEC_SYNC(DESIOT_VS21) {}
DESIOT_ATT_WEAK DESIOT_DEF_EXEC_SYNC(DESIOT_VS22) {}
DESIOT_ATT_WEAK DESIOT_DEF_EXEC_SYNC(DESIOT_VS23) {}
DESIOT_ATT_WEAK DESIOT_DEF_EXEC_SYNC(DESIOT_VS24) {}
DESIOT_ATT_WEAK DESIOT_DEF_EXEC_SYNC(DESIOT_VS25) {}
DESIOT_ATT_WEAK DESIOT_DEF_EXEC_SYNC(DESIOT_VS26) {}
DESIOT_ATT_WEAK DESIOT_DEF_EXEC_SYNC(DESIOT_VS27) {}
DESIOT_ATT_WEAK DESIOT_DEF_EXEC_SYNC(DESIOT_VS28) {}
DESIOT_ATT_WEAK DESIOT_DEF_EXEC_SYNC(DESIOT_VS29) {}
DESIOT_ATT_WEAK DESIOT_DEF_EXEC_SYNC(DESIOT_VS30) {}
DESIOT_ATT_WEAK DESIOT_DEF_EXEC_SYNC(DESIOT_VS31) {}
