#pragma once

#include "stm32l4xx_hal.h"

void init_canlib(CAN_HandleTypeDef hcan, uint8_t deviceId);
