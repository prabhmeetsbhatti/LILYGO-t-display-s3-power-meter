// Import header files
#include "Arduino.h"
#include "pin_config.h"
#include "OneButton.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_sntp.h"
#include "factory_gui.h"
#include <lvgl.h>
#include <Adafruit_INA219.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>



//initialize classes variable
Adafruit_INA219 ina219;


OneButton button1(PIN_BUTTON_1, true);
OneButton button2(PIN_BUTTON_2, true);

esp_lcd_panel_io_handle_t io_handle = NULL;
static lv_disp_draw_buf_t disp_buf;  // contains internal graphic buffer(s) called draw buffer(s)
static lv_disp_drv_t disp_drv;       // contains callback functions
static lv_color_t *lv_disp_buf;
static bool is_initialized_lvgl = false;
static int brightness_level = 8;
// OneButton button1(PIN_BUTTON_1, true);
// OneButton button2(PIN_BUTTON_2, true);
typedef struct {
  uint8_t cmd;
  uint8_t data[14];
  uint8_t len;
} lcd_cmd_t;

lcd_cmd_t lcd_st7789v[] = {
  { 0x11, { 0 }, 0 | 0x80 },
  { 0x3A, { 0X05 }, 1 },
  { 0xB2, { 0X0B, 0X0B, 0X00, 0X33, 0X33 }, 5 },
  { 0xB7, { 0X75 }, 1 },
  { 0xBB, { 0X28 }, 1 },
  { 0xC0, { 0X2C }, 1 },
  { 0xC2, { 0X01 }, 1 },
  { 0xC3, { 0X1F }, 1 },
  { 0xC6, { 0X13 }, 1 },
  { 0xD0, { 0XA7 }, 1 },
  { 0xD0, { 0XA4, 0XA1 }, 2 },
  { 0xD6, { 0XA1 }, 1 },
  { 0xE0, { 0XF0, 0X05, 0X0A, 0X06, 0X06, 0X03, 0X2B, 0X32, 0X43, 0X36, 0X11, 0X10, 0X2B, 0X32 }, 14 },
  { 0xE1, { 0XF0, 0X08, 0X0C, 0X0B, 0X09, 0X24, 0X2B, 0X22, 0X43, 0X38, 0X15, 0X16, 0X2F, 0X37 }, 14 },

};

bool inited_touch = false;
bool inited_sd = false;

static bool example_notify_lvgl_flush_ready(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_io_event_data_t *edata, void *user_ctx) {
  if (is_initialized_lvgl) {
    lv_disp_drv_t *disp_driver = (lv_disp_drv_t *)user_ctx;
    lv_disp_flush_ready(disp_driver);
  }
  return false;
}

static void example_lvgl_flush_cb(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_map) {
  esp_lcd_panel_handle_t panel_handle = (esp_lcd_panel_handle_t)drv->user_data;
  int offsetx1 = area->x1;
  int offsetx2 = area->x2;
  int offsety1 = area->y1;
  int offsety2 = area->y2;
  // copy a buffer's content to a specific area of the display
  esp_lcd_panel_draw_bitmap(panel_handle, offsetx1, offsety1, offsetx2 + 1, offsety2 + 1, color_map);
}

void ota_update() {
  Serial.println("Booting");
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("Connection Failed! Rebooting...");
    delay(5000);
    ESP.restart();
  }

  // Port defaults to 3232
  // ArduinoOTA.setPort(3232);

  // Hostname defaults to esp3232-[MAC]
  // ArduinoOTA.setHostname("myesp32");

  // No authentication by default
  // ArduinoOTA.setPassword("admin");

  // Password can be set with it's md5 value as well
  // MD5(admin) = 21232f297a57a5a743894a0e4a801fc3
  // ArduinoOTA.setPasswordHash("21232f297a57a5a743894a0e4a801fc3");

  ArduinoOTA
    .onStart([]() {
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH)
        type = "sketch";
      else  // U_SPIFFS
        type = "filesystem";

      // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
      Serial.println("Start updating " + type);
    })
    .onEnd([]() {
      Serial.println("\nEnd");
    })
    .onProgress([](unsigned int progress, unsigned int total) {
      Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    })
    .onError([](ota_error_t error) {
      Serial.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
      else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
      else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
      else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
      else if (error == OTA_END_ERROR) Serial.println("End Failed");
    });

  ArduinoOTA.begin();

  Serial.println("Ready");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}



void setup() {
  // put your setup code here, to run once:
  Serial.begin(921600);
  // Serial.begin(115200);
  ota_update();
  ina219.begin();
  setup_display();
  display_charge();
  button1.attachClick([]() {
    // Sleep display
    if (digitalRead(PIN_LCD_BL) == LOW) increase_brightness(8);
    else increase_brightness(0);
  });
  button2.attachClick([]() {
    // Sleep display
    pinMode(PIN_LCD_BL, OUTPUT);
    if (brightness_level < 16) brightness_level++;
    else brightness_level = 0;
    increase_brightness(brightness_level);
  });
}

void increase_brightness(uint8_t level) {
  for (int i = 0; i <= level; ++i) {
    setBrightness(i);
    lv_timer_handler();
    delay(50);
  }
}


void loop() {
  // put your main code here, to run repeatedly:
  ArduinoOTA.handle();
  lv_timer_handler();
  button1.tick();
  button2.tick();
  update_power_display();
  check_bat_charge();
}

void update_power_display() {
  float shuntvoltage = ina219.getShuntVoltage_mV();
  float busvoltage = ina219.getBusVoltage_V();
  float current_mA = ina219.getCurrent_mA() / 1000;
  float power_mW = ina219.getPower_mW() / 1000;
  static uint32_t last_tick;
  if (millis() - last_tick > 100) {
    float volt = ((analogRead(PIN_BAT_VOLT) * 2 * 3.3 * 1000) / 4096) / 1000;
    float loadvoltage = busvoltage + (shuntvoltage / 1000);
    // Serial.print("Load VOLT , ");
    // Serial.println(volt);
    lv_msg_send(MSG_NEW_VOLT, &busvoltage);
    lv_msg_send(MSG_NEW_CURRENT, &current_mA);
    lv_msg_send(MSG_NEW_POWER, &power_mW);
    lv_msg_send(MSG_NEW_BAT_VOLT, &volt);

    last_tick = millis();
  }
  delay(100);
}

void check_bat_charge() {
  int volt = (analogRead(PIN_BAT_VOLT) * 2 * 3.3 * 1000) / 4096;
  if (volt < 3300) {
    esp_lcd_panel_io_tx_param(io_handle, 0x10, NULL, 0);
    pinMode(PIN_POWER_ON, OUTPUT);
    pinMode(PIN_LCD_BL, OUTPUT);
    digitalWrite(PIN_POWER_ON, LOW);
    digitalWrite(PIN_LCD_BL, LOW);
    esp_sleep_enable_ext0_wakeup((gpio_num_t)PIN_BUTTON_2, 0);  // 1 = High, 0 = Low
    esp_deep_sleep_start();
  }
}

void setup_display() {

  pinMode(PIN_POWER_ON, OUTPUT);
  digitalWrite(PIN_POWER_ON, HIGH);

  pinMode(PIN_LCD_RD, OUTPUT);
  digitalWrite(PIN_LCD_RD, HIGH);
  esp_lcd_i80_bus_handle_t i80_bus = NULL;
  esp_lcd_i80_bus_config_t bus_config = {
    .dc_gpio_num = PIN_LCD_DC,
    .wr_gpio_num = PIN_LCD_WR,
    .clk_src = LCD_CLK_SRC_PLL160M,
    .data_gpio_nums = {
      PIN_LCD_D0,
      PIN_LCD_D1,
      PIN_LCD_D2,
      PIN_LCD_D3,
      PIN_LCD_D4,
      PIN_LCD_D5,
      PIN_LCD_D6,
      PIN_LCD_D7,
    },
    .bus_width = 8,
    .max_transfer_bytes = LVGL_LCD_BUF_SIZE * sizeof(uint16_t),
    .psram_trans_align = 0,
    .sram_trans_align = 0
  };
  esp_lcd_new_i80_bus(&bus_config, &i80_bus);

  esp_lcd_panel_io_i80_config_t io_config = {
    .cs_gpio_num = PIN_LCD_CS,
    .pclk_hz = EXAMPLE_LCD_PIXEL_CLOCK_HZ,
    .trans_queue_depth = 20,
    .on_color_trans_done = example_notify_lvgl_flush_ready,
    .user_ctx = &disp_drv,
    .lcd_cmd_bits = 8,
    .lcd_param_bits = 8,
    .dc_levels = {
      .dc_idle_level = 0,
      .dc_cmd_level = 0,
      .dc_dummy_level = 0,
      .dc_data_level = 1,
    },
  };
  ESP_ERROR_CHECK(esp_lcd_new_panel_io_i80(i80_bus, &io_config, &io_handle));
  esp_lcd_panel_handle_t panel_handle = NULL;
  esp_lcd_panel_dev_config_t panel_config = {
    .reset_gpio_num = PIN_LCD_RES,
    .color_space = ESP_LCD_COLOR_SPACE_RGB,
    .bits_per_pixel = 16,
    .vendor_config = NULL
  };
  esp_lcd_new_panel_st7789(io_handle, &panel_config, &panel_handle);
  esp_lcd_panel_reset(panel_handle);
  esp_lcd_panel_init(panel_handle);

  esp_lcd_panel_invert_color(panel_handle, true);

  esp_lcd_panel_swap_xy(panel_handle, true);

  //The screen faces you, and the USB is on the left
  esp_lcd_panel_mirror(panel_handle, false, true);

  //The screen faces you, the USB is to the right
  // esp_lcd_panel_mirror(panel_handle, true, false);

  // the gap is LCD panel specific, even panels with the same driver IC, can
  // have different gap value
  esp_lcd_panel_set_gap(panel_handle, 0, 35);

  for (uint8_t i = 0; i < (sizeof(lcd_st7789v) / sizeof(lcd_cmd_t)); i++) {
    esp_lcd_panel_io_tx_param(io_handle, lcd_st7789v[i].cmd, lcd_st7789v[i].data, lcd_st7789v[i].len & 0x7f);
    if (lcd_st7789v[i].len & 0x80)
      delay(120);
  }

  lv_init();
  lv_disp_buf = (lv_color_t *)heap_caps_malloc(LVGL_LCD_BUF_SIZE * sizeof(lv_color_t), MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);

  lv_disp_draw_buf_init(&disp_buf, lv_disp_buf, NULL, LVGL_LCD_BUF_SIZE);
  /*Initialize the display*/
  lv_disp_drv_init(&disp_drv);
  /*Change the following line to your display resolution*/
  disp_drv.hor_res = EXAMPLE_LCD_H_RES;
  disp_drv.ver_res = EXAMPLE_LCD_V_RES;
  disp_drv.flush_cb = example_lvgl_flush_cb;
  disp_drv.draw_buf = &disp_buf;
  disp_drv.user_data = panel_handle;
  lv_disp_drv_register(&disp_drv);

  is_initialized_lvgl = true;

  // Adjust brightness
  pinMode(PIN_LCD_BL, OUTPUT);
  // Brightness range : 0 ~ 16 level
  increase_brightness(brightness_level);
}

// LilyGo  T-Display-S3  control backlight chip has 16 levels of adjustment range
// The adjustable range is 0~16, 0 is the minimum brightness, 16 is the maximum brightness
void setBrightness(uint8_t value) {
  static uint8_t level = 0;
  static uint8_t steps = 16;
  if (value == 0) {
    digitalWrite(PIN_LCD_BL, LOW);
    delay(3);
    level = 0;
    return;
  }
  if (level == 0) {
    digitalWrite(PIN_LCD_BL, HIGH);
    level = steps;
    delayMicroseconds(30);
  }
  int from = steps - level;
  int to = steps - value;
  int num = (steps + to - from) % steps;
  // Serial.print(num);
  // Serial.print("\n");
  for (int i = 0; i < num; i++) {
    digitalWrite(PIN_LCD_BL, LOW);
    digitalWrite(PIN_LCD_BL, HIGH);
  }
  level = value;
}
