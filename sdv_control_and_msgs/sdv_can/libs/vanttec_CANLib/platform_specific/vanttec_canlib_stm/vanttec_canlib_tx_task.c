#include "vanttec_canlib_tx_task.h"

#include "cmsis_os.h"
#include "utils.h"
#include "stm32l4xx_hal.h"
#include <stddef.h>
#include <string.h>
#include "CANMessage.h"
#include "vanttec_canlib_generic_ids.h"

// Globals
osMessageQueueId_t txMessageQueue;
osMessageQueueId_t debugMessageQueue;

extern uint32_t g_vanttec_deviceId;
extern CAN_HandleTypeDef g_vanttec_hcan;
extern osMutexId_t g_can_lock;

osThreadId_t canTxTaskHandle;
const osThreadAttr_t canTxTask_attributes = {
		.name = "canlib_tx",
		.stack_size = 128 * 3
};

void canlib_tx_task();

void init_canlib_tx(){
	txMessageQueue = osMessageQueueNew(256, sizeof(CAN_TX_QUEUE_OBJ), NULL);
	if(txMessageQueue == NULL){
		vanttec_canlib_error_handler();
	}

	canTxTaskHandle = osThreadNew(canlib_tx_task, NULL, &canTxTask_attributes);
}

void canlib_tx_update(){
	CAN_TxHeaderTypeDef txHeader;
	uint32_t txMailbox;
	txHeader.IDE = CAN_ID_STD;
	txHeader.StdId = g_vanttec_deviceId;
	txHeader.RTR = CAN_RTR_DATA;

	CAN_TX_QUEUE_OBJ txOut;
	while(osMessageQueueGet(txMessageQueue, &txOut, NULL, 100) == osOK){
		txHeader.DLC = txOut.msg_size;
		osMutexAcquire(g_can_lock, 10);
		uint32_t freeLevel = HAL_CAN_GetTxMailboxesFreeLevel(&g_vanttec_hcan);
		HAL_StatusTypeDef ret = HAL_CAN_AddTxMessage(&g_vanttec_hcan, &txHeader, txOut.buf, &txMailbox);
		if(ret != HAL_OK){
			txHeader.DLC++;
			//TODO do something on error
		}
		osMutexRelease(g_can_lock);
		osDelay(10);
	}
}

void canlib_tx_task(void *data){
	uint8_t buf[8];
	for(;;){
		canlib_tx_update();
		osDelay(5);
	}
}

void canlib_queue_tx(const uint8_t *data, uint8_t len){
	CAN_TX_QUEUE_OBJ obj;
	if(len < 0 || len > 8){
		vanttec_canlib_error_handler();
	}

	obj.msg_size = len;
	memcpy(obj.buf, data, len);

	osStatus_t ret = osMessageQueuePut(txMessageQueue, &obj, 0, 10);
	if(ret != osOK){
		len++;
	}
}

uint32_t lastDebugSent = 0;

int8_t canlib_send_debug_string(const char *string){
	uint32_t len = strlen(string);
	// Rate limit debug messages
	if(HAL_GetTick() - lastDebugSent < 10){
		return -1;
	}

	uint32_t i = 0;
	uint32_t msg_len = 0;
	char buf[8];
	while(i != len){
		// Queue max number of bytes, decrease len
		if(len - i >= 7){
			msg_len = 7;
		} else {
			msg_len = len - i;
		}

		buf[0] = VANTTEC_CAN_ID_DEBUG;
		strncpy(buf + 1, string + i, msg_len);
		canlib_queue_tx(buf, msg_len + 1);
		i += msg_len;
	}


	lastDebugSent = HAL_GetTick();
	return 0;
}

void canlib_send_byte(uint8_t id, const uint8_t data){
	uint8_t buf[2];
	can_pack_byte(id, data, buf, 2);
	canlib_queue_tx(buf, 2);
}

void canlib_send_short(uint8_t id, uint16_t data){
	uint8_t buf[3];
	can_pack_short(id, data, buf, 3);
	canlib_queue_tx(buf, 3);
}

void canlib_send_long(uint8_t id, uint32_t data){
	uint8_t buf[5];
	can_pack_long(id, data, buf, 5);
	canlib_queue_tx(buf, 5);
}

void canlib_send_float(uint8_t id, float data){
	uint8_t buf[5];
	can_pack_float(id, data, buf, 5);
	canlib_queue_tx(buf, 5);
}
