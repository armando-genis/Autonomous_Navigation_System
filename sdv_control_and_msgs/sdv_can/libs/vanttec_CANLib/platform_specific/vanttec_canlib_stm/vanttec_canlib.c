#include "vanttec_canlib.h"
#include "utils.h"
#include <cmsis_os.h>

CAN_HandleTypeDef g_vanttec_hcan;
uint16_t g_vanttec_deviceId;
osMutexId_t g_can_lock;

void init_canlib(CAN_HandleTypeDef hcan, uint8_t deviceId, int filters[], int filters_size){
	g_vanttec_hcan = hcan;
	g_vanttec_deviceId = 0x400 | deviceId;

	//// Initialize CAN filters ////

	// go through the list, and set up filters
	// each filter will have 2 IDs
	// but also add a 0 if we have an odd number of them
	// since we need 2 IDs for each filter

	for (int i = 0; i < filters_size; i+=2) { 
		CAN_FilterTypeDef filter_FIFO0;
		filter_FIFO0.FilterBank = i + 1;
		filter_FIFO0.FilterMode = CAN_FILTERMODE_IDLIST;
		filter_FIFO0.FilterScale = CAN_FILTERSCALE_16BIT;
		filter_FIFO0.FilterFIFOAssignment = CAN_RX_FIFO0;
		filter_FIFO0.FilterActivation = CAN_FILTER_ENABLE;
		filter_FIFO0.FilterIdHigh = filters[i] << 5u;
		
		if ( i == (filters_size-1) ) {
			filter_FIFO0.FilterIdLow = 0x0000;
		} else {
			filter_FIFO0.FilterIdLow = filters[i + 1] << 5u;
		}
		
		filter_FIFO0.FilterMaskIdHigh = 0x0000;
		filter_FIFO0.FilterMaskIdLow = 0x0000;
		filter_FIFO0.SlaveStartFilterBank = 14;

		HAL_StatusTypeDef ret = HAL_CAN_ConfigFilter(&hcan, &filter_FIFO0);
		if(ret != HAL_OK) vanttec_canlib_error_handler();

		CAN_FilterTypeDef filter_FIFO1;
		filter_FIFO1.FilterBank = i + 2;
		filter_FIFO1.FilterMode = CAN_FILTERMODE_IDLIST;
		filter_FIFO1.FilterScale = CAN_FILTERSCALE_16BIT;
		filter_FIFO1.FilterFIFOAssignment = CAN_RX_FIFO1;
		filter_FIFO1.FilterActivation = CAN_FILTER_ENABLE;
		filter_FIFO1.FilterIdHigh = filters[i] << 5u;
		if ( i == (filters_size-1) ) {
			filter_FIFO1.FilterIdLow = 0x0000;
		} else {
			filter_FIFO1.FilterIdLow = filters[i + 1] << 5u;
		}
		filter_FIFO1.FilterMaskIdHigh = 0x0000;
		filter_FIFO1.FilterMaskIdLow = 0x0000;
		filter_FIFO1.SlaveStartFilterBank = 14;

		ret = HAL_CAN_ConfigFilter(&hcan, &filter_FIFO1);
		if(ret != HAL_OK) vanttec_canlib_error_handler();
	}

	HAL_StatusTypeDef ret = HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
	if(ret != HAL_OK) vanttec_canlib_error_handler();

	ret = HAL_CAN_Start(&hcan);
	if(ret != HAL_OK) vanttec_canlib_error_handler();

	g_can_lock = osMutexNew(NULL);

}
