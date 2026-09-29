#include "Display.h"

U8G2_ST7305_300X400_1_4W_HW_SPI u8g2(U8G2_R0, /*cs=*/RLCD_CS, /*dc=*/RLCD_DC, /*reset=*/RLCD_RST);

bool displayDirty = true;
unsigned long lastDisplayUpdate = 0;

void applyScreenOrientation() {
  u8g2.setDisplayRotation(screenOrientation == SCREEN_ORIENTATION_GPS_TOP
                             ? U8G2_R2
                             : U8G2_R0);
  displayDirty = true;
}
