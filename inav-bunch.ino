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
#include <HardwareSerial.h>
#include <stdint.h>
#include "SSD1306.h"
#include <QmuTactile.h>
#include "qsp.h"
#include "radio_node.h"
#include "beacons.h"
#include "oled_display.h"
#include "lora.h"

#ifdef ARDUINO_ESP32_DEV
    #define LORA_SS_PIN     18
    #define LORA_RST_PIN    14
    #define LORA_DI0_PIN    26

    #define SPI_SCK_PIN     5
    #define SPI_MISO_PIN    19
    #define SPI_MOSI_PIN    27
#else
    #error please select hardware
#endif

#define PIN_BUTTON 0
#define TASK_LORA_READ 2 // We check for new packets only from time to time, no need to do it more often

SSD1306 display(0x3c, 4, 15);
OledDisplay oledDisplay(&display);

RadioNode radioNode;
QspConfiguration_t qsp = {};
uint8_t bindKey[4] = {0x13, 0x27, 0x42, 0x07};

Beacons beacons;

QmuTactile button(PIN_BUTTON);

uint32_t nextLoRaReadTaskTs = 0;
uint32_t currentBeaconId = 0;
int8_t currentBeaconIndex = -1;

HardwareSerial MspSerial(1);

void onQspSuccess(uint8_t receivedChannel) {
    //If recide received a valid frame, that means it can start to talk
    radioNode.lastReceivedChannel = receivedChannel;
    
    radioNode.readRssi();
    radioNode.readSnr();

    uint32_t beaconId = qsp.payload[3] << 24;
    beaconId += qsp.payload[2] << 16;
    beaconId += qsp.payload[1] << 8;
    beaconId += qsp.payload[0];
    
    Beacon *beacon = beacons.getBeacon(beaconId);

    /*
     * Set common beacon attributes
     */
    beacon->setRssi(radioNode.rssi);
    beacon->setSnr(radioNode.snr);
    beacon->setLastContactMillis(millis());

    if (qsp.frameId == QSP_FRAME_COORDS) {
        long tmp;

        tmp = qsp.payload[4];
        tmp += qsp.payload[5] << 8;
        tmp += qsp.payload[6] << 16;
        tmp += qsp.payload[7] << 24;

        beacon->setLat(tmp / 10000000.0d);

        tmp = qsp.payload[8];
        tmp += qsp.payload[9] << 8;
        tmp += qsp.payload[10] << 16;
        tmp += qsp.payload[11] << 24;

        beacon->setLon(tmp / 10000000.0d);
    }
}

void onQspFailure() {

}

void setup()
{
    MspSerial.begin(115200, SERIAL_8N1, 32, 33);
    Serial.begin(115200);
    /*
     * Setup OLED display
     */
    pinMode(16, OUTPUT);
    digitalWrite(16, LOW); // set GPIO16 low to reset OLED
    delay(50);
    digitalWrite(16, HIGH); // while OLED is running, must set GPIO16 to high
    Wire.begin(4, 15);
    display.init();
    display.flipScreenVertically();
    display.setFont(ArialMT_Plain_10);

    qsp.onSuccessCallback = onQspSuccess;
    qsp.onFailureCallback = onQspFailure;

    /*
     * Radio setup
     */
    SPI.begin(SPI_SCK_PIN, SPI_MISO_PIN, SPI_MOSI_PIN, LORA_SS_PIN);
    LoRa.setSPIFrequency(4E6);
    radioNode.init(LORA_SS_PIN, LORA_RST_PIN, LORA_DI0_PIN, NULL);
    radioNode.reset();
    radioNode.canTransmit = true;
    LoRa.receive();

    oledDisplay.init();
    oledDisplay.setPage(OLED_PAGE_BEACON_LIST);

    button.start();
}

void loop()
{
    button.loop();

    radioNode.handleTxDoneState(false);

    if (radioNode.radioState != RADIO_STATE_TX && nextLoRaReadTaskTs < millis()) {
        int packetSize = LoRa.parsePacket();
        if (packetSize) {
            Serial.println("incoming");
            radioNode.bytesToRead = packetSize;
            radioNode.readAndDecode(&qsp, bindKey);
        }

        nextLoRaReadTaskTs = millis() + TASK_LORA_READ;
    }

    /*
     * Watchdog for frame decoding stuck somewhere in the middle of a process
     */
    if (
        qsp.protocolState != QSP_STATE_IDLE &&
        qsp.frameDecodingStartedAt + QSP_MAX_FRAME_DECODE_TIME < millis()
    ) {
        qsp.protocolState = QSP_STATE_IDLE;
    }

    if (currentBeaconIndex == -1 && beacons.count() > 0) {
        currentBeaconIndex = 0;
        currentBeaconId = beacons.get(currentBeaconIndex)->getId();
    }

    oledDisplay.loop();
}
