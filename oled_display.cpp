#include "oled_display.h"
#include "Arduino.h"
#include "beacons.h"
#include "runtime_state.h"

OledDisplay::OledDisplay(SSD1306 *display) {
    _display = display;
}

void OledDisplay::init() {
    _display->init();
    _display->setFont(ArialMT_Plain_10);
}

void OledDisplay::loop() {
    page();
}

void OledDisplay::setPage(uint8_t page) {
    _page = page;
}

void OledDisplay::page() {

    static uint32_t lastUpdate = 0;

    //Do not allow for OLED to be updated too often
    if (lastUpdate > 0 && millis() - lastUpdate < 200 && _forceDisplay == false) {
        return;
    }

    _forceDisplay = false;

    switch (_page) {
        
        case OLED_PAGE_BEACON_LIST:
            renderPageBeaconList();
            break;
        case OLED_PAGE_I_AM_A_BEACON:
            renderPageIamBeacon();
            break;
    }

    lastUpdate = millis();
}

void OledDisplay::renderHeader(String title) {
    _display->setFont(ArialMT_Plain_10);
    _display->drawString(0, 0, title);

    // _display->drawString(90, 0, String(gps.satellites.value()) + " sats");
}

void OledDisplay::renderPageBeaconList() {
    _display->clear();

    renderHeader("Beacon " + String(currentBeaconIndex + 1) + "/" + String(beacons.count()));

    if (beacons.count() > 0) {
        _display->setFont(ArialMT_Plain_10);

        Beacon *beacon = beacons.getBeacon(currentBeaconId);
        _display->drawString(0, 10, "ID: " + String(beacon->getId(), HEX));
        _display->drawString(70, 10, "RSSI: " + String(beacon->getRssi()));

        int lastContact = (millis() - beacon->getLastContactMillis()) / 1000;
        String contactString = "";

        if (lastContact > 60) {
            contactString = String((int)(lastContact / 60)) + "min";
        } else {
            contactString = String(lastContact) + "s";
        }

        _display->drawString(70, 20, "T: -" + contactString);

        if (STATE(RUNTIME_STATE_BEACON_LOCKED)) {   
            _display->drawString(0, 20, "Locked");
        } else {
            _display->drawString(0, 20, "Select beacon");
        }

        if (STATE(RUNTIME_STATE_SET_WAYPOINT)) {   
            _display->drawString(0, 34, "MSP command SET_WP");
        } else {
            _display->drawString(0, 34, "MSP command IDLE");
        }

        // if (beacon->hasPos() && gps.satellites.value() > 5) {
        //     _display->setFont(ArialMT_Plain_16);

        //     double dst = TinyGPSPlus::distanceBetween(gps.location.lat(), gps.location.lng(), beacon->getLat(), beacon->getLon());
        //     _display->drawString(0, 34, "Dst: " + String(dst, 0) + "m");

        //     double courseTo =
        //         TinyGPSPlus::courseTo(
        //             gps.location.lat(),
        //             gps.location.lng(),
        //             beacon->getLat(),
        //             beacon->getLon());

        //     _display->setFont(ArialMT_Plain_10);
        //     _display->drawString(00, 20, "Course: " + String(TinyGPSPlus::cardinal(courseTo)));

        // } else {
        //     _display->setFont(ArialMT_Plain_16);
        //     _display->drawString(0, 34, "No distance");
        // }

        // _display->setFont(ArialMT_Plain_10);
        // _display->drawString(0, 54, String(beacon->getLat(), 5));
        // _display->drawString(64, 54, String(beacon->getLon(), 5));


    } else {
        _display->setFont(ArialMT_Plain_16);

        _display->drawString(0, 12, "No beacons");
    }

    if (uavNode.isValid()) {
        _display->setFont(ArialMT_Plain_10);
        _display->drawString(0, 48, "UAV: OK");
    } else {
        _display->setFont(ArialMT_Plain_10);
        _display->drawString(0, 48, "UAV: -");
    }

    _display->display();
}

void OledDisplay::renderPageIamBeacon() {
    _display->clear();

    renderHeader("I'm a beacon");

    _display->drawString(0, 30, "TX mode");
    
    // _display->drawString(0, 54, String(gps.location.lat(), 5));
    // _display->drawString(64, 54, String(gps.location.lng(), 5));
    
    _display->display();
}