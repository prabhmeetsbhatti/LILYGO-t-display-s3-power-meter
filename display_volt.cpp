#include "factory_gui.h"
#include "lvgl.h"
#include "Arduino.h"

LV_FONT_DECLARE(arial_24);

static void update_text_subscriber_cb(lv_event_t *e);

void display_charge() {
  lv_obj_t *bat_label = lv_label_create(lv_scr_act());
  lv_obj_align(bat_label, LV_ALIGN_TOP_LEFT, 0, 0);
  lv_obj_set_style_text_font(bat_label, &arial_24, 0);
  lv_obj_add_event_cb(bat_label, update_text_subscriber_cb, LV_EVENT_MSG_RECEIVED, NULL);
  lv_msg_subsribe_obj(MSG_NEW_CURRENT, bat_label, (void *)"Current : %.2f A");

  lv_obj_t *shunt_label = lv_label_create(lv_scr_act());
  lv_obj_align_to(shunt_label, bat_label, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 0);
  lv_obj_set_style_text_font(shunt_label, &arial_24, 0);
  lv_obj_add_event_cb(shunt_label, update_text_subscriber_cb, LV_EVENT_MSG_RECEIVED, NULL);
  lv_msg_subsribe_obj(MSG_NEW_VOLT, shunt_label, (void *)"Volts : %.2f V");
  
  lv_obj_t *power_label = lv_label_create(lv_scr_act());
  lv_obj_align_to(power_label, shunt_label, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 5);
  lv_obj_set_style_text_font(power_label, &arial_24, 0);
  lv_obj_add_event_cb(power_label, update_text_subscriber_cb, LV_EVENT_MSG_RECEIVED, NULL);
  lv_msg_subsribe_obj(MSG_NEW_POWER, power_label, (void *)"Power : %.2f W");
  
  lv_obj_t *bat_esp_label = lv_label_create(lv_scr_act());
  lv_obj_align_to(bat_esp_label, power_label, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 5);
  lv_obj_set_style_text_font(bat_esp_label, &arial_24, 0);
  lv_obj_add_event_cb(bat_esp_label, update_text_subscriber_cb, LV_EVENT_MSG_RECEIVED, NULL);
  lv_msg_subsribe_obj(MSG_NEW_BAT_VOLT, bat_esp_label, (void *)"Small Bat : %.2f V");
}

static void update_text_subscriber_cb(lv_event_t *e) {
  lv_obj_t *label = lv_event_get_target(e);
  lv_msg_t *m = lv_event_get_msg(e);

  const char *fmt = (const char *)lv_msg_get_user_data(m);
  const float *v = (const float *)lv_msg_get_payload(m);
  // Serial.print("Shunt VOLT , ");
  lv_label_set_text_fmt(label, fmt, *v);
}
