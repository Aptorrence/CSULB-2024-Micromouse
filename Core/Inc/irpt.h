

#pragma once

#include "stm32f4xx.h"
#include <stdbool.h>

typedef struct
{
	GPIO_TypeDef* PT_port;
	uint16_t PT_pin;
	uint8_t ir_index;
}IR_PT;

void init_pts(IR_PT *pins, GPIO_TypeDef* pt_port, uint16_t pt_pin, uint8_t ir_index) {
    pins->PT_port = pt_port;
    pins->PT_pin = pt_pin;
    pins->ir_index = ir_index;
}

bool Dif_PT (IR_PT pins, uint16_t * ir_data, uint16_t thresh) {
	// Read all Photo Transistors

	uint32_t ir_off = ir_data[pins.ir_index];

	// Turn on PT.
	HAL_GPIO_WritePin(pins.PT_port, pins.PT_pin, GPIO_PIN_SET);

	HAL_Delay(1);

	// Take difference.
	uint32_t diff = ir_data[pins.ir_index] - ir_off;

	// Turn off PT
	HAL_GPIO_WritePin(pins.PT_port, pins.PT_pin, GPIO_PIN_RESET);

	HAL_Delay(1);

	return diff > thresh;
}
