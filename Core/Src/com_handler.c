/*
 * com_handler.c
 *
 *  Created on: Feb 23, 2026
 *      Author: erenegdemir
 */

#ifndef INC_COM_HANDLER_C_
#define INC_COM_HANDLER_C_

#include "com_handler.h"
#include "usbd_cdc_if.h"
#include <string.h>

static uint8_t rxrb[RXRB_SIZE];
static volatile uint16_t rx_head = 0;
static volatile uint16_t rx_tail = 0;

static uint8_t g_tx_buf[4 + MAX_PAYLOAD + 2];
static uint16_t g_tx_len = 0;
static volatile bool g_tx_pending = false;

typedef enum {PS_M0, PS_M1, PS_L0, PS_L1, PS_PAY, PS_CRC0, PS_CRC1} ps_t;
static ps_t ps = PS_M0;
static uint16_t want_len = 0;
static uint16_t got_len = 0;
static uint8_t pl[MAX_PAYLOAD];
static uint16_t crc_rx = 0;

static volatile uint8_t g_req_start = 0;
static volatile uint8_t g_req_stop = 0;

static uint32_t g_meas_seq = 0;

static eit_cfg_t g_cfg = {
		.map_id = 1,
		.fs_hz = 800000,
		.f0_hz = 25000,
		.blank_periods = 10,
		.int_periods = 16
};

static uint16_t crc16_ccitt(const uint8_t* data, uint16_t len);

static inline bool rb_pop(uint8_t* out) {
	if (rx_tail == rx_head) return false;
	*out = rxrb[rx_tail];
	rx_tail = (uint16_t)(rx_tail + 1) & (RXRB_SIZE - 1);
	return true;
}

static inline uint16_t rd_u16le(const uint8_t* p)
{
	return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

static inline uint32_t rd_u32le(const uint8_t* p)
{
	return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
			((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static inline void wr_u16le(uint8_t* p, uint16_t v)
{
	p[0] = (uint8_t)v & 0xFF;
	p[1] = (uint8_t)((v >> 8) & 0xFF);
}

static inline void wr_u32le(uint8_t* p, uint32_t v)
{
	p[0] = (uint8_t)v & 0xFF;
	p[1] = (uint8_t)((v >> 8) & 0xFF);
	p[2] = (uint8_t)((v >> 16) & 0xFF);
	p[3] = (uint8_t)((v >> 24) & 0xFF);

}

void rb_push(const uint8_t* p, uint16_t n)
{
	for (uint16_t i = 0; i < n; i++) {
		rxrb[rx_head] = p[i];
		rx_head = (uint16_t)((rx_head + 1) & (RXRB_SIZE - 1));
		if (rx_head == rx_tail) {
			rx_tail = (uint16_t)((rx_tail + 1) & (RXRB_SIZE - 1));
		}
	}
}

static bool usb_try_send(const uint8_t* data, uint16_t len)
{
	if(CDC_Transmit_FS((uint8_t*)data, len) == USBD_OK) return true;
	return false;
}

static void send_frame(const uint8_t* payload, uint16_t len)
{
	if(len > MAX_PAYLOAD) return;

	g_tx_buf[0] = 0xA5;
	g_tx_buf[1] = 0x5A;
	g_tx_buf[2] = (uint8_t)len & 0xFF;
	g_tx_buf[3] = (uint8_t)((len >> 8) & 0xFF);
	if(len && payload) memcpy(&g_tx_buf[4], payload, len);

	uint16_t crc = crc16_ccitt(payload, len);
	g_tx_buf[4 + len] = (uint8_t)(crc & 0xFF);
	g_tx_buf[5 + len] = (uint8_t)((crc >> 8) & 0xFF);

	g_tx_len = (uint16_t)(4 + len + 2);
	if(!usb_try_send(g_tx_buf, g_tx_len)) g_tx_pending = true;

}

static void send_cmd_rsp(uint16_t req_id, uint16_t cmd_id, uint16_t status, const uint8_t* opt, uint16_t opt_len)
{
	//payload: [mtype=2][cmd_id u16][status u16][opt...]
	uint8_t pl[1 + 2 + 2 + 2 + 64]; //opt şimdikik 64 ile sınırladık
	if(opt_len > 64) opt_len = 64;

	pl[0] = (uint8_t)M_CMD_RSP;
	wr_u16le(&pl[1], req_id);
	wr_u16le(&pl[3], cmd_id);
	wr_u16le(&pl[5], status);
	if(opt && opt_len) memcpy(&pl[7], opt, opt_len);

	send_frame(pl, (uint16_t)(7 + opt_len));
}

static void send_data_frame(uint32_t meas_seq, uint16_t map_id, const float* A, uint16_t N)
{
	uint16_t need = (uint16_t)(1 + 4 + 2 + 2 + (uint16_t)(N * 4));
	if(need > MAX_PAYLOAD) return;

	static uint8_t pl[MAX_PAYLOAD];
	pl[0] = (uint8_t)M_DATA;

	wr_u32le(&pl[1], meas_seq);
	wr_u16le(&pl[5], map_id);
	wr_u16le(&pl[7], N);

	memcpy(&pl[9], A, (size_t)N * sizeof(float));

	send_frame(pl, need);
}

static void handle_payload(const uint8_t* pl, uint16_t len)
{
	if(len < 1) return;

	uint8_t mtype = pl[0];

	if(mtype != M_CMD_REQ) {
		send_cmd_rsp(0, 0, ST_BAD_TYP, NULL, 0);
		return;
	}

	if(len < 1 + 2) {
		send_cmd_rsp(0, 0, ST_BAD_LEN, NULL, 0);
		return;
	}

	uint16_t req_id = rd_u16le(&pl[1]);
	uint16_t cmd = rd_u16le(&pl[3]);
	const uint8_t* args = &pl[5];
	uint16_t arglen = (uint16_t)(len - 5);

	switch((cmd_id_t)cmd){
	case CMD_PING:
		send_cmd_rsp(req_id, cmd, ST_OK, NULL, 0);
		break;
	case CMD_HELLO: {
		// opt: map_id(u16), fs(u32), f0(u32)
		uint8_t opt[2 + 4 + 4];
		wr_u16le(&opt[0], g_cfg.map_id);
		wr_u32le(&opt[2], g_cfg.fs_hz);
		wr_u32le(&opt[6], g_cfg.f0_hz);
		send_cmd_rsp(req_id, cmd, ST_OK, opt, sizeof(opt));
	}	break;
	case CMD_SET_PARAM: {
		// args: map_id(u16) + fs(u32) + f0(u32) + blank (u16) + integ(u16) => 14 byte
		if(arglen != 14) {
			send_cmd_rsp(req_id, cmd, ST_BAD_LEN, NULL, 0);
			break;
		}
		g_cfg.map_id = rd_u16le(&args[0]);
		g_cfg.fs_hz = rd_u32le(&args[2]);
		g_cfg.f0_hz = rd_u32le(&args[6]);
		g_cfg.blank_periods = rd_u16le(&args[10]);
		g_cfg.int_periods = rd_u16le(&args[12]);

		send_cmd_rsp(req_id, cmd, ST_OK, NULL, 0);
	}	break;
	case CMD_STATUS_GET: {
		//opt: start_req(u8) , stop_req(u8), map_id(u16), blank(u16), integ(u16)
		uint8_t opt[1 + 1 + 2 + 2 + 2 + 4 + 4];
		opt[0] = g_req_start;
		opt[1] = g_req_stop;
		wr_u16le(&opt[2], g_cfg.map_id);
		wr_u16le(&opt[4], g_cfg.blank_periods);
		wr_u16le(&opt[6], g_cfg.int_periods);
		wr_u32le(&opt[8], g_cfg.fs_hz);
		wr_u32le(&opt[12], g_cfg.f0_hz);

		send_cmd_rsp(req_id, cmd, ST_OK, opt, sizeof(opt));
	} break;
	case CMD_START:
		g_req_start = 1;
		g_req_stop = 0;

		send_cmd_rsp(req_id, cmd, ST_OK, NULL, 0);
		break;
	case CMD_STOP:
		g_req_start = 0;
		g_req_stop = 1;

		send_cmd_rsp(req_id, cmd, ST_OK, NULL, 0);
		break;

	default:
		send_cmd_rsp(req_id, cmd, ST_BAD_CMD, NULL, 0);
		break;

	}
}

static uint16_t crc16_ccitt(const uint8_t* data, uint16_t len)
{
	uint16_t crc = 0xFFFF;

	for (uint16_t i = 0; i < len; i++) {
		crc ^= (uint16_t)data[i] << 8;
		for (int b = 0; b < 8; b++) {
			if(crc & 0x8000) crc = (uint16_t)((crc << 1) ^ 0x1021) & 0xFFFF;
			else 			crc = (uint16_t)(crc << 1) & 0xFFFF;
		}
	}
	return crc;
}

void CDC_TxRetry_Poll(void)
{
	if(!g_tx_pending) return;

	if(CDC_Transmit_FS(g_tx_buf, g_tx_len) == USBD_OK) {
		g_tx_pending = false;
	}
}

void EIT_FrameParser_Poll(void)
{
	uint8_t b;
	while(rb_pop(&b)) {
		switch (ps) {
		case PS_M0:
			if(b == 0xA5) ps = PS_M1;
			break;
		case PS_M1:
			if(b == 0x5A) ps = PS_L0;
			else ps = PS_M0;
			break;
		case PS_L0:
			want_len = b;
			ps = PS_L1;
			break;
		case PS_L1:
			want_len |= (uint16_t)(b << 8);
			if(want_len > MAX_PAYLOAD) {
				ps = PS_M0;
			} else if (want_len == 0){
				ps = PS_CRC0;
			}else {
				got_len = 0;
				ps = PS_PAY;
			}
			break;
		case PS_PAY:
			pl[got_len++] = b;
			if(got_len == want_len) {
//				handle_payload(pl, want_len);
				ps = PS_CRC0;
			}
			break;
		case PS_CRC0:
			crc_rx = b;
			ps = PS_CRC1;
			break;
		case PS_CRC1:
			crc_rx |= ((uint16_t)b << 8);
			{
				uint16_t crc_calc = crc16_ccitt(pl, want_len);
				if(crc_calc == crc_rx) {
					handle_payload(pl, want_len);
				}
				ps = PS_M0;
			}
			break;

		}
	}
}

void EIT_SendData(const float* A, uint16_t N){
    if(g_tx_pending) return;
    g_meas_seq++;
    send_data_frame(g_meas_seq, g_cfg.map_id, A, N);
}

bool EIT_TakeStartReq(void){
    if(!g_req_start) return false;
    g_req_start = 0;
    return true;
}
bool EIT_TakeStopReq(void){
    if(!g_req_stop) return false;
    g_req_stop = 0;
    return true;
}
void EIT_GetCfg(eit_cfg_t* out){
    if(out) *out = g_cfg; // snapshot
}


#endif /* INC_COM_HANDLER_H_ */


