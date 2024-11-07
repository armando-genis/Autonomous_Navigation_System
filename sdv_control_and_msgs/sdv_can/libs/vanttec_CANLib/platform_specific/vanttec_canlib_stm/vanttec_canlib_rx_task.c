#include "vanttec_canlib_rx_task.h"
#include "vanttec_canlib_tx_task.h"
#include "utils.h"
#include <string.h>
#include <cmsis_os.h>
#include <stdlib.h>
#include <stdbool.h>
#include "CANMessage.h"

typedef struct {
    uint8_t device_id;
    uint8_t message_id;
    void* data;
    uint8_t len;
    uint8_t is_message_queue;
    VANTTEC_CANLIB_MSGTYPE type;
} canlib_rx_entry;

canlib_rx_entry rx_table[MAX_CANLIB_RX_ENTRIES];
uint32_t last_entry = 0;
extern osMutexId_t g_can_lock;

typedef struct {
    uint8_t device_id;
    uint8_t message_id;
    uint8_t priority;
	uint8_t buf[8];
	uint8_t len;
} CAN_RX_QUEUE_OBJ;

osMessageQueueId_t rxMessageQueue;

#include <stm32l4xx_hal.h>
extern CAN_HandleTypeDef g_vanttec_hcan;

osThreadId_t canRxTaskHandle;
const osThreadAttr_t canRxTask_attributes = {
		.name = "canlib_rx",
		.stack_size = 128 * 2
};

osThreadId_t canRxQueueTaskHandle;
const osThreadAttr_t canRxQueueTask_attributes = {
		.name = "canlib_queue_rx",
		.stack_size = 128 * 2
};

void can_rx_task();
void can_rx_queue_task();

void init_canlib_rx(){
    // Initialize all tables to 0, easier to debug
    memset(rx_table, 0, sizeof(rx_table));
    last_entry = 0;

    rxMessageQueue = osMessageQueueNew(
        CANLIB_RX_QUEUE_SIZE,
        sizeof(CAN_RX_QUEUE_OBJ),
        NULL
    );

    canRxTaskHandle = osThreadNew(can_rx_task, NULL, &canRxTask_attributes);
    canRxQueueTaskHandle = osThreadNew(can_rx_queue_task, NULL, &canRxQueueTask_attributes);
}

bool is_valid_length(VANTTEC_CANLIB_MSGTYPE message_type, uint8_t len){
    uint8_t type_len;
    switch(message_type){
        case VANTTEC_CANLIB_BYTE:
            return len == 1;
        case VANTTEC_CANLIB_SHORT:
            return len == 2;
        case VANTTEC_CANLIB_LONG:
            return len == 4;
        case VANTTEC_CANLIB_FLOAT:
            return len == 4;
    }

    return false;
}

void parse_data(VANTTEC_CANLIB_MSGTYPE message_type, void *dst, const void *src, uint8_t len){
    switch(message_type){
        case VANTTEC_CANLIB_BYTE:
            *((uint8_t*) dst) = can_parse_byte(src, len);
            break;
        case VANTTEC_CANLIB_SHORT:
            *((uint16_t*) dst) = can_parse_short(src, len);
            break;
        case VANTTEC_CANLIB_LONG:
            *((uint32_t*) dst) = can_parse_long(src, len);
            uint32_t tmp = can_parse_long(src, len);
            break;
        case VANTTEC_CANLIB_FLOAT:
            *((float*) dst) = can_parse_float(src, len);
            break;
        }
}

void update_table(uint8_t device_id, uint8_t message_id, const void *rxData, uint32_t dlc){
    void *buf[8];
    for(uint32_t i = 0; i < last_entry; i++){
        if(rx_table[i].device_id == 0xff || device_id == rx_table[i].device_id){
            if(rx_table[i].message_id == message_id){
                // Write to table
				if(rx_table[i].len + 1 != dlc){ // msg length + 1 byte of msg id
					// Data length must be the same!
					vanttec_canlib_error_handler();
				}
                // Parse data into buf
                parse_data(rx_table[i].type, buf, rxData, dlc);

                if(rx_table[i].is_message_queue){
                    // TODO handle priority
                    osMessageQueuePut(*( (osMessageQueueId_t*) rx_table[i].data), buf, 1, 1);
                } else {
                    memcpy(rx_table[i].data, buf, rx_table[i].len);
                }
            }
        }
    }
}

void register_canlib_rx(uint8_t device_id, uint8_t message_id, VANTTEC_CANLIB_MSGTYPE message_type, void * data, uint8_t len){
    // Check if entries are not full
    if(last_entry == MAX_CANLIB_RX_ENTRIES){
        vanttec_canlib_error_handler();
    }

    if(!is_valid_length(message_type, len)){
        vanttec_canlib_error_handler();
    }

    // Add entry to table
    rx_table[last_entry].device_id = device_id;
    rx_table[last_entry].message_id = message_id;
    rx_table[last_entry].data = data;
    rx_table[last_entry].len = len;
    rx_table[last_entry].is_message_queue = 0;
    rx_table[last_entry].type = message_type;

    last_entry++;
}

void register_canlib_rx_queue(uint8_t device_id, uint8_t message_id, VANTTEC_CANLIB_MSGTYPE message_type, osMessageQueueId_t messageQueue, uint8_t len){
    // Check if entries are not full
    if(last_entry == MAX_CANLIB_RX_ENTRIES || messageQueue == 0){
        vanttec_canlib_error_handler();
    }

    if(!is_valid_length(message_type, len)){
        vanttec_canlib_error_handler();
    }

    rx_table[last_entry].device_id = device_id;
    rx_table[last_entry].message_id = message_id;

    // Allocates memory and writes message queue id
    rx_table[last_entry].data = malloc(sizeof(osMessageQueueId_t));		// use free somewhere?
    *((osMessageQueueId_t*) rx_table[last_entry].data) = messageQueue;
    rx_table[last_entry].len = len;
    rx_table[last_entry].is_message_queue = 1;
    rx_table[last_entry].type = message_type;

    last_entry++;
}

/**
 * Two tasks, one handles reading from canbus and placing into msg queue
 * The other task handles parsing msg queue, prevents can bus msgs from being lost
*/

void can_rx_task(){
    CAN_RX_QUEUE_OBJ msg;
    CAN_RxHeaderTypeDef header;
    uint8_t buf[8];
    uint8_t buf_temp[4];
    for(;;){
        osMutexAcquire(g_can_lock, 10);
        while(HAL_CAN_GetRxFifoFillLevel(&g_vanttec_hcan, CAN_RX_FIFO0)){
            HAL_StatusTypeDef ret = HAL_CAN_GetRxMessage(&g_vanttec_hcan, CAN_RX_FIFO0, &header, buf);
            if(ret != HAL_OK || header.DLC < 1){
                continue;
            }

            msg.message_id = buf[0];
            msg.device_id = header.StdId & 0x3F;
            msg.priority = (header.StdId & 0xC0) >> 6;
            memcpy(msg.buf, buf, header.DLC);
            msg.len = header.DLC;

            osMessageQueuePut(rxMessageQueue, &msg, msg.priority, 10);            
        }
        osMutexRelease(g_can_lock);
        osDelay(10);
    }
}

void can_rx_queue_task(){
    CAN_RX_QUEUE_OBJ msg;
    uint8_t prio;
    for(;;){
        if(osMessageQueueGet(rxMessageQueue, &msg, &prio, 10) == osOK){
            update_table(msg.device_id, msg.message_id, msg.buf, msg.len);
        }
        osDelay(10);
    }
}
