#include "Display.h"
#include "settings.h"

// 8mm converted to pixels: this panel doesn't have a published active-area
// mm figure, so this is derived from the stated 4.2" diagonal at 300x400
// (diag = sqrt(300^2+400^2) = 500px = 106.68mm -> ~4.687 px/mm -> 8mm ~= 37.5px).
// Close enough for layout purposes, but if the exact 8mm matters, verify
// against the physical panel with calipers rather than trust this alone.
U8G2_ST7305_300X400_1_4W_HW_SPI u8g2(U8G2_R0, /*cs=*/RLCD_CS, /*dc=*/RLCD_DC, /*reset=*/RLCD_RST);

// Set whenever page/menu state changes; drives an immediate redraw instead
// of waiting for the next 1Hz display tick, so menu navigation feels
// responsive rather than laggy.
bool displayDirty = true;

unsigned long lastDisplayUpdate = 0;

// =====================================================
// SCREEN ORIENTATION (Config > Screen)
// Called once from setup() right after u8g2.begin(), and again
// immediately from menu.cpp any time the pilot changes it. U8G2_R0 is
// the app's original (GPS Bottom) orientation; U8G2_R2 is the same
// panel rotated 180 degrees (GPS Top).
// =====================================================
void applyScreenOrientation() {
  u8g2.setDisplayRotation(screenOrientation == SCREEN_ORIENTATION_GPS_TOP
                             ? U8G2_R2
                             : U8G2_R0);
  displayDirty = true;
}
