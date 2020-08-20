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

typedef enum {
    ARMED                                           = (1 << 2),
    WAS_EVER_ARMED                                  = (1 << 3),

    ARMING_DISABLED_FAILSAFE_SYSTEM                 = (1 << 7),

    ARMING_DISABLED_NOT_LEVEL                       = (1 << 8),
    ARMING_DISABLED_SENSORS_CALIBRATING             = (1 << 9),
    ARMING_DISABLED_SYSTEM_OVERLOADED               = (1 << 10),
    ARMING_DISABLED_NAVIGATION_UNSAFE               = (1 << 11),
    ARMING_DISABLED_COMPASS_NOT_CALIBRATED          = (1 << 12),
    ARMING_DISABLED_ACCELEROMETER_NOT_CALIBRATED    = (1 << 13),
    ARMING_DISABLED_ARM_SWITCH                      = (1 << 14),
    ARMING_DISABLED_HARDWARE_FAILURE                = (1 << 15),
    ARMING_DISABLED_BOXFAILSAFE                     = (1 << 16),
    ARMING_DISABLED_BOXKILLSWITCH                   = (1 << 17),
    ARMING_DISABLED_RC_LINK                         = (1 << 18),
    ARMING_DISABLED_THROTTLE                        = (1 << 19),
    ARMING_DISABLED_CLI                             = (1 << 20),
    ARMING_DISABLED_CMS_MENU                        = (1 << 21),
    ARMING_DISABLED_OSD_MENU                        = (1 << 22),
    ARMING_DISABLED_ROLLPITCH_NOT_CENTERED          = (1 << 23),
    ARMING_DISABLED_SERVO_AUTOTRIM                  = (1 << 24),
    ARMING_DISABLED_OOM                             = (1 << 25),
    ARMING_DISABLED_INVALID_SETTING                 = (1 << 26),
    ARMING_DISABLED_PWM_OUTPUT_ERROR                = (1 << 27),

    ARMING_DISABLED_ALL_FLAGS                       = (ARMING_DISABLED_FAILSAFE_SYSTEM | ARMING_DISABLED_NOT_LEVEL | ARMING_DISABLED_SENSORS_CALIBRATING | 
                                                       ARMING_DISABLED_SYSTEM_OVERLOADED | ARMING_DISABLED_NAVIGATION_UNSAFE |
                                                       ARMING_DISABLED_COMPASS_NOT_CALIBRATED | ARMING_DISABLED_ACCELEROMETER_NOT_CALIBRATED |
                                                       ARMING_DISABLED_ARM_SWITCH | ARMING_DISABLED_HARDWARE_FAILURE | ARMING_DISABLED_BOXFAILSAFE |
                                                       ARMING_DISABLED_BOXKILLSWITCH | ARMING_DISABLED_RC_LINK | ARMING_DISABLED_THROTTLE | ARMING_DISABLED_CLI |
                                                       ARMING_DISABLED_CMS_MENU | ARMING_DISABLED_OSD_MENU | ARMING_DISABLED_ROLLPITCH_NOT_CENTERED |
                                                       ARMING_DISABLED_SERVO_AUTOTRIM | ARMING_DISABLED_OOM | ARMING_DISABLED_INVALID_SETTING |
                                                       ARMING_DISABLED_PWM_OUTPUT_ERROR),
} armingFlag_e;

typedef enum {
    GPS_NO_FIX = 0,
    GPS_FIX_2D,
    GPS_FIX_3D
} gpsFixType_e;

class UAVNode {
    public:
        UAVNode(void);
        bool isValid(void);
        bool isArmed(void);
        String gpsFix(void);
        double lat;                 // GPS Latitude
        double lon;                 // GPS Longitude
        double alt;                 // GPS Altitude
        double positionAltitude;    // Position estimator Altitude in meters
        uint8_t fixType;
        uint8_t hdop;
        uint8_t sats;
        int16_t groundSpeed;
        int16_t groundCourse;
        uint16_t sensorStatus;
        uint32_t flightModes;
        uint16_t armingFlags;
        uint32_t lastContact = 0;
    private:
        bool _armingState(uint32_t mask);
};

#endif