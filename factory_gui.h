#pragma once

#define UI_BG_COLOR    lv_color_black()
#define UI_FRAME_COLOR lv_color_hex(0x282828)
#define UI_FONT_COLOR  lv_color_white()
#define UI_PAGE_COUNT  3

#define MSG_NEW_BUS_VOLT   1
#define MSG_NEW_CURRENT    2
#define MSG_NEW_VOLT   3
#define MSG_NEW_POWER   4
#define MSG_NEW_BAT_VOLT   5

#define LV_DELAY(x)                                                                                                                                  \
  do {                                                                                                                                               \
    uint32_t t = x;                                                                                                                                  \
    while (t--) {                                                                                                                                    \
      lv_timer_handler();                                                                                                                            \
      delay(1);                                                                                                                                      \
    }                                                                                                                                                \
  } while (0);

void display_charge();
void ui_switch_page(void);