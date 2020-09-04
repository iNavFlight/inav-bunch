/** 
 * This file is part of the INAV Bunch https://github.com/iNavFlight/inav-bunch 
 * Copyright (c) 2020 Pawel Spychalski pspychalski@gmail.com
 * 
 * This program is free software: you can redistribute it and/or modify  
 * it under the terms of the GNU General Public License as published by  
 * the Free Software Foundation, version 3.
 *
 * This program is distributed in the hope that it will be useful, but 
 * WITHOUT ANY WARRANTY; without even the implied warranty of 
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU 
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License 
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#ifndef RUNTIME_STATE_H
#define RUNTIME_STATE_H

#include <Arduino.h>

typedef enum {
    RUNTIME_STATE_RADIO_LISTEN      = (1 << 0),
    RUNTIME_STATE_MSP_POLL_DATA     = (1 << 1),
    RUNTIME_STATE_BEACON_LOCKED     = (1 << 3),
} DeviceState_e;

#define DISABLE_STATE(mask) (runtimeState &= ~(mask))
#define ENABLE_STATE(mask) (runtimeState |= (mask))
#define STATE(mask) (runtimeState & (mask))

extern uint32_t runtimeState;

#endif