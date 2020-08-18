/* 
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

#ifndef UAV_NODE_H
#define UAV_NODE_H

#include "Arduino.h"

class UAVNode {
    public:
        UAVNode(void);
        bool isValid(void);
        double lat;
        double lon;
        double alt;
        uint8_t fixType;
        uint8_t hdop;
        uint8_t sats;
        int16_t groundSpeed;
        int16_t groundCourse;
        uint32_t lastContact = 0;
};

#endif