#include "vanttec_canlib.h"
#include "utils.h"
#include <cmsis_os.h>

CAN_HandleTypeDef g_vanttec_hcan;
uint16_t g_vanttec_deviceId;
osMutexId_t g_can_lock;

void init_canlib(CAN_HandleTypeDef hcan, uint8_t deviceId){
	g_vanttec_hcan = hcan;
	g_vanttec_deviceId = 0x400 | deviceId;

	// Initialize can filters
	// TODO We should only filter for vanttec device ID: 0x4FF
	CAN_FilterTypeDef filter;
	filter.FilterBank = 1;
	filter.FilterMode = CAN_FILTERMODE_IDLIST;
	filter.FilterScale = CAN_FILTERSCALE_16BIT;
	filter.FilterFIFOAssignment = CAN_RX_FIFO0;
	filter.FilterActivation = CAN_FILTER_ENABLE;
	filter.FilterIdHigh = 0;
	filter.FilterIdLow = 0x0410 << 5u; // TODO Stepper id for now
	filter.FilterMaskIdHigh = 0x0000;
	filter.FilterMaskIdLow = 0x0000;
	filter.SlaveStartFilterBank = 14;

	// TODO set vanttec can ID
	HAL_StatusTypeDef ret = HAL_CAN_ConfigFilter(&hcan, &filter);
	if(ret != HAL_OK) vanttec_canlib_error_handler();

	ret = HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
	if(ret != HAL_OK) vanttec_canlib_error_handler();

	ret = HAL_CAN_Start(&hcan);
	if(ret != HAL_OK) vanttec_canlib_error_handler();

	g_can_lock = osMutexNew(NULL);

}
