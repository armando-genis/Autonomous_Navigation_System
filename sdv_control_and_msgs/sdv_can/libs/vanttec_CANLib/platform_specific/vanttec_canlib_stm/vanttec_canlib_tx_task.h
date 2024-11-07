#pragma once

#include <stdint.h>

void init_canlib_tx();

int8_t canlib_send_debug_string(const char *string);

void canlib_queue_tx(const uint8_t *data, uint8_t len);
void canlib_send_byte(uint8_t id, uint8_t data);
void canlib_send_short(uint8_t id, uint16_t data);
void canlib_send_long(uint8_t id, uint32_t data);
void canlib_send_float(uint8_t id, float data);


typedef struct {
	uint8_t buf[8];
	uint8_t msg_size;
} CAN_TX_QUEUE_OBJ;

