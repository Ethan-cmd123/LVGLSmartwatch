#include <lvgl.h>
#include <Wire.h>
#include <WiFi.h>
#include <Ticker.h>
#include <SPI.h>
#include <TFT_eSPI.h>
#include <math.h>
#include "QMI8658.h"

#define TFT_HOR_RES 240
#define TFT_VER_RES 240

#define GYRO_SENSITIVITY_DIVISOR 10.0f
#define GYRO_MAX_CIRCLE_SIZE 90.0f

TFT_eSPI tft = TFT_eSPI();

#define DRAW_BUF_SIZE (TFT_HOR_RES * TFT_VER_RES / 10 * (LV_COLOR_DEPTH / 8))
uint32_t draw_buf[DRAW_BUF_SIZE / 4];

void my_disp_flush(lv_display_t *disp, const lv_area_t *area, uint8_t *px_map)
{
    uint32_t w = lv_area_get_width(area);
    uint32_t h = lv_area_get_height(area);
    tft.startWrite();
    tft.setAddrWindow(area->x1, area->y1, w, h);
    tft.pushColors((uint16_t *)px_map, w * h, true);
    tft.endWrite();
    lv_display_flush_ready(disp);
}

static uint32_t my_tick(void) { return millis(); }

#define TCA6408Int 45
#define TCA6408I2CAddr 0x20
#define TCA6408ConfigurationReg 0x03
#define TCA6408ConfigurationData 0xFF
#define TCA6408InputPortReg 0x00

Ticker TCA6408InterruptTicker;
volatile bool TCA6408EventFlag = false;
volatile bool TouchEventFlag = false;

#define TouchRST 0
#define TouchI2CAddr 0x15
#define ChipIdRegister 0xA7
#define CST716ChipId 0X20
#define CST816SChipId 0XB4
#define CST816TChipId 0XB5
#define CST816DChipId 0XB6
#define CST826ChipId 0X11
#define CST830ChipId 0X12
#define CST836UChipId 0X13

unsigned char ChipID = 0x00;

lv_coord_t last_x = 0;
lv_coord_t last_y = 0;
bool touch_down = false;

lv_obj_t *screens[4];
int current_screen = 0;

static uint32_t fake_hours = 10;
static uint32_t fake_minutes = 9;
static uint32_t fake_seconds = 0;
static uint32_t last_tick_ms = 0;

#define CX 120
#define CY 120

lv_obj_t *hour_line = NULL;
lv_obj_t *min_line = NULL;
lv_obj_t *sec_line = NULL;

static lv_point_precise_t hour_pts[2];
static lv_point_precise_t min_pts[2];
static lv_point_precise_t sec_pts[2];

#define GYRO_RING_COUNT 3
static lv_obj_t *gyro_rings[GYRO_RING_COUNT];
static float gyro_ring_sizes[GYRO_RING_COUNT];
static float gyro_motion_smooth = 0.0f;
static float gyro_phase = 0.0f;
static uint32_t last_gyro_visual_ms = 0;

static uint32_t step_count = 0;
static float acc[3];
static float gyro[3];
static bool step_above = false;
static uint32_t last_step_ms = 0;
#define STEP_THRESHOLD 12.0f
#define STEP_DEBOUNCE_MS 300
lv_obj_t *step_label = NULL;

static lv_color_t mix_blue(uint8_t r1, uint8_t g1, uint8_t b1, uint8_t r2, uint8_t g2, uint8_t b2, float t)
{
    uint8_t r = (uint8_t)(r1 + (r2 - r1) * t);
    uint8_t g = (uint8_t)(g1 + (g2 - g1) * t);
    uint8_t b = (uint8_t)(b1 + (b2 - b1) * t);
    return lv_color_hex((r << 16) | (g << 8) | b);
}

void calc_hand(lv_point_precise_t *pts, float angle_deg, int length)
{
    float rad = (angle_deg - 90.0f) * M_PI / 180.0f;
    pts[0].x = CX;
    pts[0].y = CY;
    pts[1].x = CX + (int)(cos(rad) * length);
    pts[1].y = CY + (int)(sin(rad) * length);
}

void update_hands()
{
    float sec_angle = fake_seconds * 6.0f;
    float min_angle = fake_minutes * 6.0f + fake_seconds * 0.1f;
    float hour_angle = (fake_hours % 12) * 30.0f + fake_minutes * 0.5f;

    calc_hand(hour_pts, hour_angle, 55);
    calc_hand(min_pts, min_angle, 75);
    calc_hand(sec_pts, sec_angle, 85);

    lv_line_set_points(hour_line, hour_pts, 2);
    lv_line_set_points(min_line, min_pts, 2);
    lv_line_set_points(sec_line, sec_pts, 2);
}

void update_gyro_fancy()
{
    uint32_t now = millis();
    float dt = last_gyro_visual_ms == 0 ? 0.005f : (now - last_gyro_visual_ms) / 1000.0f;
    last_gyro_visual_ms = now;

    float gx = gyro[0];
    float gy = gyro[1];
    float gz = gyro[2];

    float mag = sqrtf(gx * gx + gy * gy + gz * gz);
    float motion = mag / GYRO_SENSITIVITY_DIVISOR;
    gyro_motion_smooth += (motion - gyro_motion_smooth) * 0.08f;

    gyro_phase += dt * (1.8f + gyro_motion_smooth * 0.08f);

    for(int i = 0; i < GYRO_RING_COUNT; i++)
    {
        float t = (GYRO_RING_COUNT == 1) ? 0.0f : (float)i / (float)(GYRO_RING_COUNT - 1);
        float base_size = 18.0f + i * 7.0f;
        float ripple = 0.5f + 0.5f * sinf(gyro_phase - i * 0.34f);
        float cascade = 1.0f - t * 0.35f;
        float target_size = base_size + gyro_motion_smooth * 18.0f * ripple * cascade;
        if(target_size > GYRO_MAX_CIRCLE_SIZE) target_size = GYRO_MAX_CIRCLE_SIZE;
        if(target_size < 8.0f) target_size = 8.0f;
        gyro_ring_sizes[i] += (target_size - gyro_ring_sizes[i]) * 0.12f;

        int size = (int)gyro_ring_sizes[i];
        int pos = CX - size / 2;
        int py = CY - size / 2;

        lv_obj_set_size(gyro_rings[i], size, size);
        lv_obj_set_pos(gyro_rings[i], pos, py);
    }
}

void load_screen_animated(int index, bool go_down)
{
    lv_scr_load_anim(screens[index],
        go_down ? LV_SCR_LOAD_ANIM_MOVE_BOTTOM : LV_SCR_LOAD_ANIM_MOVE_TOP,
        250, 0, false);
    current_screen = index;
}

void TouchInit()
{
    pinMode(TouchRST, OUTPUT);
    digitalWrite(TouchRST, LOW);
    delay(10);
    digitalWrite(TouchRST, HIGH);
    delay(50);

    Wire.beginTransmission(TouchI2CAddr);
    Wire.write(ChipIdRegister);
    Wire.endTransmission(false);
    Wire.requestFrom(TouchI2CAddr, 1, true);
    ChipID = Wire.read();

    Serial.printf("\r\nTouchChipID: 0x%02X", ChipID);
    if(ChipID == CST716ChipId) Serial.println(",Touch chip model :CST716");
    else if(ChipID == CST816SChipId) Serial.println(",Touch chip model :CST816S");
    else if(ChipID == CST816TChipId) Serial.println(",Touch chip model :CST816T");
    else if(ChipID == CST816DChipId) Serial.println(",Touch chip model :CST816D");
    else if(ChipID == CST826ChipId) Serial.println(",Touch chip model :CST826");
    else if(ChipID == CST830ChipId) Serial.println(",Touch chip model :CST830");
    else if(ChipID == CST836UChipId) Serial.println(",Touch chip model :CST836U");
    else Serial.println(",error!");
}

void TCA6408HandleInterrupt(void)
{
    TCA6408EventFlag = true;
}

void my_AllInt()
{
    if(TCA6408EventFlag)
    {
        int TCA6408IntValue = 0;
        Wire.beginTransmission(TCA6408I2CAddr);
        Wire.write(TCA6408InputPortReg);
        Wire.endTransmission(false);
        Wire.requestFrom(TCA6408I2CAddr, 1, true);
        TCA6408IntValue = Wire.read();

        if((TCA6408IntValue & 0x01) == 0x00) { TouchEventFlag = true; Serial.print("\r\nTouch Int"); }
        if((TCA6408IntValue & 0x02) == 0x00) { Serial.print("\r\nIMU Int1"); }
        if((TCA6408IntValue & 0x04) == 0x00) { Serial.print("\r\nIMU Int2"); }
        if((TCA6408IntValue & 0x08) == 0x00) { Serial.print("\r\nSW UP Int"); if(current_screen > 0) load_screen_animated(current_screen - 1, true); }
        if((TCA6408IntValue & 0x10) == 0x00) { Serial.print("\r\nSW PW Int"); }
        if((TCA6408IntValue & 0x20) == 0x00) { Serial.print("\r\nSW Down Int"); if(current_screen < 3) load_screen_animated(current_screen + 1, false); }
        if((TCA6408IntValue & 0x80) == 0x00) { Serial.print("\r\nRTC Int"); }

        TCA6408EventFlag = false;
    }
}

void IICInit()
{
    Wire.begin();
    Wire.setClock(400000);
    delay(10);
}

void TCA6408Init()
{
    int TCA6408TempData = 0;

    Wire.beginTransmission(TCA6408I2CAddr);
    Wire.write(TCA6408ConfigurationReg);
    Wire.write(0x55);
    Wire.endTransmission(true);
    delay(10);
    Wire.beginTransmission(TCA6408I2CAddr);
    Wire.write(TCA6408ConfigurationReg);
    Wire.endTransmission(false);
    Wire.requestFrom(TCA6408I2CAddr, 1, true);
    TCA6408TempData = Wire.read();

    if(0x55 == TCA6408TempData) Serial.print("\r\nTCA6408 pass!");
    else { Serial.print("\r\nTCA6408 fail!"); Serial.print(TCA6408TempData, HEX); }

    delay(10);
    Wire.beginTransmission(TCA6408I2CAddr);
    Wire.write(TCA6408ConfigurationReg);
    Wire.write(TCA6408ConfigurationData);
    Wire.endTransmission(true);

    delay(10);
    Wire.beginTransmission(TCA6408I2CAddr);
    Wire.write(TCA6408InputPortReg);
    Wire.endTransmission(false);
    Wire.requestFrom(TCA6408I2CAddr, 1, true);
    TCA6408TempData = Wire.read();

    Serial.print("\r\nTCA6408Statu:");
    Serial.print(TCA6408TempData, HEX);
    delay(10);
}

void touch_read_cb(lv_indev_t *indev, lv_indev_data_t *data)
{
    if(touch_down)
    {
        data->point.x = last_x;
        data->point.y = last_y;
        data->state = LV_INDEV_STATE_PRESSED;
    }
    else
    {
        data->state = LV_INDEV_STATE_RELEASED;
    }
}

void build_watch_face(lv_obj_t *scr)
{
    lv_obj_set_style_bg_color(scr, lv_color_hex(0x000000), 0);

    lv_obj_t *ring = lv_obj_create(scr);
    lv_obj_set_size(ring, 220, 220);
    lv_obj_center(ring);
    lv_obj_set_style_bg_color(ring, lv_color_hex(0x050505), 0);
    lv_obj_set_style_border_color(ring, lv_color_hex(0x666666), 0);
    lv_obj_set_style_border_width(ring, 3, 0);
    lv_obj_set_style_radius(ring, LV_RADIUS_CIRCLE, 0);
    lv_obj_clear_flag(ring, LV_OBJ_FLAG_SCROLLABLE);

    for(int i = 0; i < GYRO_RING_COUNT; i++)
    {
        float t = (GYRO_RING_COUNT == 1) ? 0.0f : (float)i / (float)(GYRO_RING_COUNT - 1);
        gyro_rings[i] = lv_obj_create(scr);
        lv_obj_set_style_bg_opa(gyro_rings[i], LV_OPA_TRANSP, 0);
        lv_obj_set_style_border_opa(gyro_rings[i], LV_OPA_COVER, 0);
        lv_obj_set_style_border_width(gyro_rings[i], 2, 0);
        lv_obj_set_style_radius(gyro_rings[i], LV_RADIUS_CIRCLE, 0);
        lv_obj_set_style_bg_color(gyro_rings[i], lv_color_hex(0x000000), 0);
        lv_obj_set_style_border_color(gyro_rings[i], mix_blue(8, 18, 70, 90, 205, 255, t), 0);
        lv_obj_clear_flag(gyro_rings[i], LV_OBJ_FLAG_SCROLLABLE);
        gyro_ring_sizes[i] = 18.0f + i * 7.0f;
        int size = (int)gyro_ring_sizes[i];
        lv_obj_set_size(gyro_rings[i], size, size);
        lv_obj_set_pos(gyro_rings[i], CX - size / 2, CY - size / 2);
    }

    for(int i = 0; i < 12; i++)
    {
        float rad = (i * 30.0f - 90.0f) * M_PI / 180.0f;
        lv_obj_t *tick = lv_obj_create(scr);
        lv_obj_set_size(tick, 4, 4);
        lv_obj_set_style_bg_color(tick, lv_color_hex(0x808080), 0);
        lv_obj_set_style_radius(tick, LV_RADIUS_CIRCLE, 0);
        lv_obj_set_style_border_width(tick, 0, 0);
        lv_obj_set_pos(tick, (int)(CX + cos(rad) * 98) - 2, (int)(CY + sin(rad) * 98) - 2);
    }

    hour_line = lv_line_create(scr);
    lv_obj_set_style_line_color(hour_line, lv_color_hex(0xE0E0E0), 0);
    lv_obj_set_style_line_width(hour_line, 4, 0);
    lv_obj_set_style_line_rounded(hour_line, true, 0);

    min_line = lv_line_create(scr);
    lv_obj_set_style_line_color(min_line, lv_color_hex(0xC0C0C0), 0);
    lv_obj_set_style_line_width(min_line, 3, 0);
    lv_obj_set_style_line_rounded(min_line, true, 0);

    sec_line = lv_line_create(scr);
    lv_obj_set_style_line_color(sec_line, lv_color_hex(0x909090), 0);
    lv_obj_set_style_line_width(sec_line, 2, 0);
    lv_obj_set_style_line_rounded(sec_line, true, 0);
}

void build_step_screen(lv_obj_t *scr)
{
    lv_obj_set_style_bg_color(scr, lv_color_hex(0x000000), 0);

    lv_obj_t *ring = lv_obj_create(scr);
    lv_obj_set_size(ring, 220, 220);
    lv_obj_center(ring);
    lv_obj_set_style_bg_color(ring, lv_color_hex(0x111111), 0);
    lv_obj_set_style_border_color(ring, lv_color_hex(0xFFAA00), 0);
    lv_obj_set_style_border_width(ring, 3, 0);
    lv_obj_set_style_radius(ring, LV_RADIUS_CIRCLE, 0);
    lv_obj_clear_flag(ring, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *title = lv_label_create(scr);
    lv_label_set_text(title, "Steps");
    lv_obj_set_style_text_color(title, lv_color_hex(0xFFAA00), 0);
    lv_obj_set_style_text_font(title, &lv_font_montserrat_14, 0);
    lv_obj_align(title, LV_ALIGN_CENTER, 0, -30);

    step_label = lv_label_create(scr);
    lv_label_set_text(step_label, "0");
    lv_obj_set_style_text_color(step_label, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_font(step_label, &lv_font_montserrat_14, 0);
    lv_obj_align(step_label, LV_ALIGN_CENTER, 0, 10);
}

static bool ph1_toggled = false;
static bool ph2_toggled = false;

void placeholder_btn_cb(lv_event_t *e)
{
    lv_obj_t *btn = (lv_obj_t *)lv_event_get_target(e);
    bool *state = (bool *)lv_event_get_user_data(e);
    *state = !(*state);
    lv_obj_set_style_bg_color(btn, *state ? lv_color_hex(0x555555) : lv_color_hex(0x222222), 0);
}

void build_placeholder_screen(lv_obj_t *scr, uint32_t accent, const char *icon_text, const char *label_text, bool *toggle_state)
{
    lv_obj_set_style_bg_color(scr, lv_color_hex(0x000000), 0);

    lv_obj_t *ring = lv_obj_create(scr);
    lv_obj_set_size(ring, 220, 220);
    lv_obj_center(ring);
    lv_obj_set_style_bg_color(ring, lv_color_hex(0x111111), 0);
    lv_obj_set_style_border_color(ring, lv_color_hex(accent), 0);
    lv_obj_set_style_border_width(ring, 3, 0);
    lv_obj_set_style_radius(ring, LV_RADIUS_CIRCLE, 0);
    lv_obj_clear_flag(ring, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *icon = lv_label_create(scr);
    lv_label_set_text(icon, icon_text);
    lv_obj_set_style_text_color(icon, lv_color_hex(accent), 0);
    lv_obj_set_style_text_font(icon, &lv_font_montserrat_14, 0);
    lv_obj_align(icon, LV_ALIGN_CENTER, 0, -30);

    lv_obj_t *lbl = lv_label_create(scr);
    lv_label_set_text(lbl, label_text);
    lv_obj_set_style_text_color(lbl, lv_color_hex(0xAAAAAA), 0);
    lv_obj_set_style_text_font(lbl, &lv_font_montserrat_14, 0);
    lv_obj_align(lbl, LV_ALIGN_CENTER, 0, -10);

    lv_obj_t *touch_btn = lv_btn_create(scr);
    lv_obj_set_size(touch_btn, 90, 32);
    lv_obj_align(touch_btn, LV_ALIGN_CENTER, 0, 25);
    lv_obj_set_style_bg_color(touch_btn, lv_color_hex(0x222222), 0);
    lv_obj_set_style_border_color(touch_btn, lv_color_hex(accent), 0);
    lv_obj_set_style_border_width(touch_btn, 1, 0);
    lv_obj_set_style_radius(touch_btn, 8, 0);
    lv_obj_add_event_cb(touch_btn, placeholder_btn_cb, LV_EVENT_CLICKED, (void *)toggle_state);
    lv_obj_t *touch_lbl = lv_label_create(touch_btn);
    lv_label_set_text(touch_lbl, "Press me");
    lv_obj_set_style_text_color(touch_lbl, lv_color_hex(0xCCCCCC), 0);
    lv_obj_set_style_text_font(touch_lbl, &lv_font_montserrat_14, 0);
    lv_obj_center(touch_lbl);
}

void setup()
{
    Serial.begin(115200);

    IICInit();
    TCA6408Init();

    pinMode(TCA6408Int, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(TCA6408Int), TCA6408HandleInterrupt, FALLING);

    TouchInit();

    QMI8658_init();
    QMI8658_enableSensors(QMI8658_CONFIG_ACCGYR_ENABLE);

    Serial.begin(115200);
    tft.begin();
    tft.setRotation(0);

    lv_init();
    lv_tick_set_cb(my_tick);

    lv_display_t *disp = lv_display_create(TFT_HOR_RES, TFT_VER_RES);
    lv_display_set_flush_cb(disp, my_disp_flush);
    lv_display_set_buffers(disp, draw_buf, NULL, sizeof(draw_buf), LV_DISPLAY_RENDER_MODE_PARTIAL);

    lv_indev_t *indev = lv_indev_create();
    lv_indev_set_type(indev, LV_INDEV_TYPE_POINTER);
    lv_indev_set_read_cb(indev, touch_read_cb);

    screens[0] = lv_obj_create(NULL);
    build_watch_face(screens[0]);

    screens[1] = lv_obj_create(NULL);
    build_step_screen(screens[1]);

    screens[2] = lv_obj_create(NULL);
    build_placeholder_screen(screens[2], 0x00BFFF, "~ Activity ~", "Placeholder", &ph1_toggled);

    screens[3] = lv_obj_create(NULL);
    build_placeholder_screen(screens[3], 0x32CD32, "+ Health +", "Placeholder", &ph2_toggled);

    current_screen = 0;
    update_hands();
    update_gyro_fancy();
    lv_scr_load(screens[0]);

    last_tick_ms = millis();
}

void loop()
{
    if(TCA6408EventFlag == true)
    {
        my_AllInt();
    }

    uint32_t now = millis();
    if(now - last_tick_ms >= 1000)
    {
        last_tick_ms = now;
        fake_seconds++;
        if(fake_seconds >= 60) { fake_seconds = 0; fake_minutes++; }
        if(fake_minutes >= 60) { fake_minutes = 0; fake_hours++; }
        if(fake_hours >= 24)   { fake_hours = 0; }
        update_hands();
    }

    QMI8658_read_acc_xyz(acc);
    QMI8658_read_gyro_xyz(gyro);
    update_gyro_fancy();

    float mag = sqrtf(acc[0] * acc[0] + acc[1] * acc[1] + acc[2] * acc[2]);
    if(mag > STEP_THRESHOLD && !step_above && (now - last_step_ms > STEP_DEBOUNCE_MS))
    {
        step_count++;
        last_step_ms = now;
        if(step_label)
        {
            char buf[16];
            snprintf(buf, sizeof(buf), "%lu", step_count);
            lv_label_set_text(step_label, buf);
        }
    }
    step_above = (mag > STEP_THRESHOLD);

    unsigned int X_H4 = 0;
    unsigned int X_L8 = 0;
    unsigned int Y_H4 = 0;
    unsigned int Y_L8 = 0;
    unsigned int finger_count = 0;

    Wire.beginTransmission(TouchI2CAddr);
    Wire.write(0x02);
    Wire.endTransmission(false);
    Wire.requestFrom(TouchI2CAddr, 1, true);
    finger_count = Wire.read() & 0x0F;

    Wire.beginTransmission(TouchI2CAddr);
    Wire.write(0x03);
    Wire.endTransmission(false);
    Wire.requestFrom(TouchI2CAddr, 1, true);
    X_H4 = Wire.read();

    Wire.beginTransmission(TouchI2CAddr);
    Wire.write(0x04);
    Wire.endTransmission(false);
    Wire.requestFrom(TouchI2CAddr, 1, true);
    X_L8 = Wire.read();

    Wire.beginTransmission(TouchI2CAddr);
    Wire.write(0x05);
    Wire.endTransmission(false);
    Wire.requestFrom(TouchI2CAddr, 1, true);
    Y_H4 = Wire.read();

    Wire.beginTransmission(TouchI2CAddr);
    Wire.write(0x06);
    Wire.endTransmission(false);
    Wire.requestFrom(TouchI2CAddr, 1, true);
    Y_L8 = Wire.read();

    last_x = 0xFF - ((X_H4 << 8 | X_L8) & 0X0FFF);
    last_y = ((Y_H4 << 8 | Y_L8) & 0X0FFF);

    touch_down = (finger_count > 0) || TouchEventFlag;
    TouchEventFlag = false;

    Serial.printf("Touch Point:%02X , %02X \r\n", last_x, last_y);
    lv_timer_handler();
    delay(5);
}