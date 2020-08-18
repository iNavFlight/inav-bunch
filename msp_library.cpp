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

#include <Arduino.h>
#include "msp_library.h"

void MSPLibrary::begin(Stream &stream, uint32_t timeout)
{
  _stream = &stream;
  _timeout = timeout;
}

void MSPLibrary::reset()
{
  _stream->flush();
  while (_stream->available() > 0)
    _stream->read();
}

void MSPLibrary::send(uint8_t messageID, void *payload, uint8_t size)
{
  _stream->write('$');
  _stream->write('M');
  _stream->write('<');
  _stream->write(size);
  _stream->write(messageID);

  uint8_t checksum = size ^ messageID;
  uint8_t *payloadPtr = (uint8_t *)payload;
  for (uint8_t i = 0; i < size; ++i)
  {
    uint8_t b = *(payloadPtr++);
    checksum ^= b;

    _stream->write(b);
  }

  _stream->write(checksum);
}

// timeout in milliseconds
bool MSPLibrary::recv(uint8_t *messageID, void *payload, uint8_t maxSize, uint8_t *recvSize)
{
  uint32_t t0 = millis();

  while (1)
  {

    // read header
    while (_stream->available() < 6)
      if (millis() - t0 >= _timeout)
        return false;
    char header[3];
    _stream->readBytes((char *)header, 3);

    // check header
    if (header[0] == '$' && header[1] == 'M' && header[2] == '>')
    {
      // header ok, read payload size
      *recvSize = _stream->read();

      // read message ID (type)
      *messageID = _stream->read();

      uint8_t checksumCalc = *recvSize ^ *messageID;

      // read payload
      uint8_t *payloadPtr = (uint8_t *)payload;
      uint8_t idx = 0;
      while (idx < *recvSize)
      {
        if (millis() - t0 >= _timeout)
          return false;
        if (_stream->available() > 0)
        {
          uint8_t b = _stream->read();
          checksumCalc ^= b;
          if (idx < maxSize)
            *(payloadPtr++) = b;
          ++idx;
        }
      }
      // zero remaining bytes if *size < maxSize
      for (; idx < maxSize; ++idx)
        *(payloadPtr++) = 0;

      // read and check checksum
      while (_stream->available() == 0)
        if (millis() - t0 >= _timeout)
          return false;
      uint8_t checksum = _stream->read();
      if (checksumCalc == checksum)
      {
        return true;
      }
    }
  }
}

// wait for messageID
// recvSize can be NULL
bool MSPLibrary::waitFor(uint8_t messageID, void *payload, uint8_t maxSize, uint8_t *recvSize)
{
  uint8_t recvMessageID;
  uint8_t recvSizeValue;
  uint32_t t0 = millis();
  while (millis() - t0 < _timeout)
    if (recv(&recvMessageID, payload, maxSize, (recvSize ? recvSize : &recvSizeValue)) && messageID == recvMessageID)
      return true;

  // timeout
  return false;
}

// send a message and wait for the reply
// recvSize can be NULL
bool MSPLibrary::request(uint8_t messageID, void *payload, uint8_t maxSize, uint8_t *recvSize)
{
  send(messageID, NULL, 0);
  return waitFor(messageID, payload, maxSize, recvSize);
}

// send message and wait for ack
bool MSPLibrary::command(uint8_t messageID, void *payload, uint8_t size, bool waitACK)
{
  send(messageID, payload, size);

  // ack required
  if (waitACK)
    return waitFor(messageID, NULL, 0);

  return true;
}