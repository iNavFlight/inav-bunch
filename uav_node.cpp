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

#include "uav_node.h"

UAVNode::UAVNode(void) {

}

bool UAVNode::isValid(void) {
    return (millis() - lastContact < 1000) && lastContact > 0;
}

bool UAVNode::isArmed(void) {
    return _armingState(ARMED);
}

String UAVNode::gpsFix(void) {
    if (fixType == GPS_FIX_2D) {
        return "2D";
    } else if (fixType == GPS_FIX_3D) {
        return "3D";
    } else {
        return "NO";
    }
}

bool UAVNode::_armingState(uint32_t mask) {
    return (armingFlags & (mask));
}