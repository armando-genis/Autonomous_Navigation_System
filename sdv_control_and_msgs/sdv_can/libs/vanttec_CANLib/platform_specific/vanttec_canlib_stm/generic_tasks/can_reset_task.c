#include "vanttec_canlib_tx_task.h"
#include "generic_tasks/can_reset_task.h"
#include "cmsis_os.h"
#include <stdbool.h>
#include "stm32l4xx_hal.h"


extern CAN_HandleTypeDef g_vanttec_hcan;

void can_reset_task(){
	bool initializingBus = false;
	for(;;){
		SET_BIT(g_vanttec_hcan.Instance->MCR, CAN_MCR_ABOM);
		// Check for bus off bit
		// if(READ_BIT(g_vanttec_hcan.Instance->ESR, CAN_ESR_BOFF)){
		//   // Bus is off!!!! Reset bus
		//   SET_BIT(g_vanttec_hcan.Instance->MCR, CAN_MCR_INRQ);
		//   initializingBus = true;
		// }
		// if(initializingBus && READ_BIT(g_vanttec_hcan.Instance->MSR,
		// CAN_MSR_INAK)){
		//   // Bus is initialized, reset init bit
		//   CLEAR_BIT(g_vanttec_hcan.Instance->MCR, CAN_MCR_INRQ);
		//   initializingBus = false;
		// }
		osDelay(10);
	}
}

osThreadId_t canResetTaskHandle;
const osThreadAttr_t canResetAttributes = {
		.name = "vanttec_canlib_canreset",
		.stack_size = 128 * 1
};

void init_can_reset_task(){
	canResetTaskHandle = osThreadNew(can_reset_task, NULL, &canResetAttributes);
}
