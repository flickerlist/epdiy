/**
 * @file "epd_board_common.h"
 * @brief Common board functions shared between boards.
 */
#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

void epd_board_temperature_init_v2();
float epd_board_ambient_temperature_v2();

void epd_init_i2c_semaphore();
SemaphoreHandle_t epd_get_i2c_semaphore();
