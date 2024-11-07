#include "ping_task.h"
#include "vanttec_canlib_tx_task.h"
#include "vanttec_canlib_rx_task.h"
#include "cmsis_os.h"
#include <stddef.h>
#include "stm32l4xx_hal.h"
#include "main.h"

void ping_task(void *args){
    uint8_t rxData = 0;
    uint8_t lastRxData = 0;
    register_canlib_rx(0x2, 0x00, VANTTEC_CANLIB_BYTE, &rxData, 1);
    for(;;){
        canlib_send_byte(0xBE, rxData);
        lastRxData = rxData;
        osDelay(1000);
    }
}

void toggle_led(uint8_t i){
    switch(i){
        case 0:
            HAL_GPIO_TogglePin(DEBUG_1_GPIO_Port, DEBUG_1_Pin);
            break;
        case 1:
            HAL_GPIO_TogglePin(DEBUG_2_GPIO_Port, DEBUG_2_Pin);
            break;
        case 2:
            HAL_GPIO_TogglePin(DEBUG_3_GPIO_Port, DEBUG_3_Pin);
            break;
        case 3:
            HAL_GPIO_TogglePin(DEBUG_4_GPIO_Port, DEBUG_4_Pin);
            break;
        default:
            break;
    }
}

void hb_rx_task(void *args){
    uint8_t hb_data[4];
    uint8_t last_hb_data[4];
    register_canlib_rx(0x1, 0x00, VANTTEC_CANLIB_BYTE, &(hb_data[0]), 1);
    register_canlib_rx(0x2, 0x00, VANTTEC_CANLIB_BYTE, &(hb_data[1]), 1);
    register_canlib_rx(0x3, 0x00, VANTTEC_CANLIB_BYTE, &(hb_data[2]), 1);
    register_canlib_rx(0x4, 0x00, VANTTEC_CANLIB_BYTE, &(hb_data[3]), 1);

    for(;;){
        for(uint8_t i = 0; i < 4; i++){
            if(hb_data[i] != last_hb_data[i]){
                // Blink led corresponding to board
                toggle_led(i);
            }

            last_hb_data[i] = hb_data[i];
        }
        osDelay(1000);
    }
}

osThreadId_t pingTaskHandle;
const osThreadAttr_t pingTaskAttributes = {
		.name = "ping",
		.stack_size = 128 * 1
};


void init_ping_task(){
	pingTaskHandle = osThreadNew(hb_rx_task, NULL, &pingTaskAttributes);
}
