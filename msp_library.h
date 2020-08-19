/*
  MSP.cpp

  Copyright (c) 2017, Fabrizio Di Vittorio (fdivitto2013@gmail.com)

  This file originally was distributed under LGPL 2.1 and was transferred to 
  GNU GPL under the section 3 of LGPL 2.1 License 

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

#ifndef MSP_LIBRARY_H
#define MSP_LIBRARY_H

#include <Arduino.h>
#include <Stream.h>

#define MSP_RAW_GPS     106
#define MSP_SET_WP      0xD1
#define MSP_STATUS_EX   150

// MSP_COMP_GPS reply
struct msp_comp_gps_t
{
    int16_t distanceToHome;  // distance to home in meters
    int16_t directionToHome; // direction to home in degrees
    uint8_t heartbeat;       // toggles 0 and 1 for each change
} __attribute__((packed));

struct MSP_RAW_GPS_t
{
    uint8_t fixType; // MSP_GPS_NO_FIX, MSP_GPS_FIX_2D, MSP_GPS_FIX_3D
    uint8_t numSat;
    int32_t lat;          // 1 / 10000000 deg
    int32_t lon;          // 1 / 10000000 deg
    int16_t alt;          // meters
    int16_t groundSpeed;  // cm/s
    int16_t groundCourse; // cm/s
    int16_t hdop;         // cm/s
} __attribute__((packed));

struct MSP_STATUS_EX_t
{
    uint16_t cycleTime;
    uint16_t i2cErrorCounter;
    uint16_t sensorStatus;
    uint32_t flightModes;
    uint8_t configProfile;
    uint16_t systemLoad;
    uint16_t armingFlags;
    uint8_t accCalibrationAxis;
} __attribute__((packed));

// MSP_SET_WP command
// Special waypoints are 0 and 255. 0 is the RTH position, 255 is the POSHOLD position (lat, lon, alt).
struct msp_set_wp_t
{
    uint8_t waypointNumber;
    uint8_t action; // one of MSP_NAV_STATUS_WAYPOINT_ACTION_XXX
    int32_t lat;    // decimal degrees latitude * 10000000
    int32_t lon;    // decimal degrees longitude * 10000000
    int32_t alt;    // altitude (cm)
    int16_t p1;     // speed (cm/s) when action is MSP_NAV_STATUS_WAYPOINT_ACTION_WAYPOINT, or "land" (value 1) when action is MSP_NAV_STATUS_WAYPOINT_ACTION_RTH
    int16_t p2;     // not used
    int16_t p3;     // not used
    uint8_t flag;   // 0xa5 = last, otherwise set to 0
} __attribute__((packed));

class MSPLibrary
{
public:
    void begin(Stream &stream, uint32_t timeout);
    void send(uint8_t messageID, void *payload, uint8_t size);
    bool recv(uint8_t *messageID, void *payload, uint8_t maxSize, uint8_t *recvSize);
    bool waitFor(uint8_t messageID, void *payload, uint8_t maxSize, uint8_t *recvSize = NULL);
    bool request(uint8_t messageID, void *payload, uint8_t maxSize, uint8_t *recvSize = NULL);
    bool command(uint8_t messageID, void *payload, uint8_t size, bool waitACK = true);
    void reset();

private:
    Stream *_stream;
    uint32_t _timeout;
};

#endif