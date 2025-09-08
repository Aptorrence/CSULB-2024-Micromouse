/*
 * mouse_position.h
 *
 *  Created on: May 22, 2024
 *      Author: guineverebergdorf
 */

#pragma once

#include "main.h"
#include <stdbool.h>

// Read IR sensor data to locate current wall position relative to mouse

bool* irDataGreaterThanFlags(uint16_t array[], int size, uint16_t irThreshold);

// Start condition:
// If PT readings are all > 2000 for more than 3000 ms, return true.

bool* checkStartCondition(uint16_t array[], int size, uint16_t threshold, uint16_t duration_ms);
