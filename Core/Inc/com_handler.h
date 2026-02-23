/*
 * com_handler.h
 *
 *  Created on: Feb 23, 2026
 *      Author: erenegdemir
 */

#ifndef INC_COM_HANDLER_H_
#define INC_COM_HANDLER_H_


#include <stdbool.h>
#include <stdio.h>
#include <stdint.h>
#include "eit_types.h"

#ifdef __cplusplus
extern "C" {
#endif

#define RXRB_SIZE	2048
#define MAX_PAYLOAD	1024


typedef enum {
	M_CMD_REQ	= 1,
	M_CMD_RSP	= 2,
	M_DATA 		= 3,
	M_EVENT		= 4
}msg_type_t;

typedef enum {
	CMD_PING		= 1,
	CMD_HELLO		= 2,
	CMD_SET_PARAM	= 3,
	CMD_START		= 4,
	CMD_STOP		= 5,
	CMD_STATUS_GET	= 6
}cmd_id_t;

typedef enum {
	ST_OK		= 0,
	ST_BAD_LEN	= 1,
	ST_BAD_CMD	= 2,
	ST_BAD_TYP	= 3
}status_t;

void rb_push(const uint8_t* p, uint16_t n);

void CDC_TxRetry_Poll(void);
void EIT_FrameParser_Poll(void);
void EIT_SendData(const float* A, uint16_t N);

bool EIT_TakeStartReq(void);
bool EIT_TakeStopReq(void);
void EIT_GetCfg(eit_cfg_t* out);

#ifdef __cplusplus
}
#endif


#endif /* INC_COM_HANDLER_H_ */
