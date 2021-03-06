#ifndef OLED_DISPLAY
#define OLED_DISPLAY

#include "SSD1306.h"
#include <TinyGPS++.h>
#include "beacons.h"
#include "uav_node.h"

enum txOledPages {
    OLED_PAGE_NONE,
    OLED_PAGE_BEACON_LIST,
    OLED_PAGE_I_AM_A_BEACON
};

#define OLED_COL_COUNT 64
#define OLED_DISPLAY_PAGE_COUNT 2

extern Beacons beacons;
extern UAVNode uavNode;

class OledDisplay {
    public:
        OledDisplay(SSD1306 *display);
        void init();
        void loop();
        void setPage(uint8_t page);
    private:
        SSD1306 *_display;
        void renderPageBeaconList();
        void renderPageIamBeacon();
        void renderHeader(String title);
        void page();
        uint8_t _page = OLED_PAGE_NONE;
        bool _forceDisplay = false;
};


#endif