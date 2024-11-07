#include "generic_tasks/hb_task.h"
#include "vanttec_canlib_generic_ids.h"
#include "vanttec_canlib_tx_task.h"
#include "cmsis_os.h"
#include <stddef.h>
#include "main.h"

void hb_task(void *args){
	uint8_t data = 0;
	for(;;){
		canlib_send_debug_string("Hello");
		canlib_send_byte(VANTTEC_CAN_ID_HB, data);
		data++;
		HAL_GPIO_TogglePin(DEBUG_1_GPIO_Port, DEBUG_1_Pin);
		osDelay(1000);
	}
}

osThreadId_t hbTaskHandle;
const osThreadAttr_t hbTaskAttributes = {
		.name = "canlib_hb",
		.stack_size = 128 * 1
};

void init_hb_task(){
	hbTaskHandle = osThreadNew(hb_task, NULL, &hbTaskAttributes);
}
