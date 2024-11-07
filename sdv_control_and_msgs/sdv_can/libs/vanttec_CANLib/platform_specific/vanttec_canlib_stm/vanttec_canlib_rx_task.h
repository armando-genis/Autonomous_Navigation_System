#pragma once
#include <stdint.h>
#include "cmsis_os.h"

#define MAX_CANLIB_RX_ENTRIES 32
#define CANLIB_RX_QUEUE_SIZE 64

typedef enum {
    VANTTEC_CANLIB_BYTE, //uint8_t
    VANTTEC_CANLIB_SHORT, //uint16_t
    VANTTEC_CANLIB_LONG, //uint32_t
    VANTTEC_CANLIB_FLOAT, //float
} VANTTEC_CANLIB_MSGTYPE;

void init_canlib_rx();

// WARNING -> data must be static, otherwise memory address may be invalid if it is a variable in stack
// 0xff on device id works as a wildcard
void register_canlib_rx(uint8_t device_id, uint8_t message_id, VANTTEC_CANLIB_MSGTYPE message_type, void * data, uint8_t len);

// Same functionality as register_canlib_rx, where message queue will be given
void register_canlib_rx_queue(uint8_t device_id, uint8_t message_id, VANTTEC_CANLIB_MSGTYPE message_type, osMessageQueueId_t messageQueue, uint8_t len);
