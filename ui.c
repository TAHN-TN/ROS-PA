#include "ui.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "board.h"
#include "wifi_log.h"
#include "buttons.h"
#include "common.h"
#include "esp_console.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lcd.h"
#include "leds.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "sdkconfig.h"
#include "sx1262.h"

static const char *TAG = "ui";

/* ------------------------- du lieu menu ------------------------- */
#define SET_VER 1
typedef struct { uint8_t ver, mod, role, cs, sf, pw, fq; } settings_t;
enum { IT_MOD, IT_ROLE, IT_CASE, IT_SF, IT_PW, IT_FQ, IT_START, IT_COUNT };

static const uint8_t SF_LIST[6]  = {12, 11, 10, 9, 8, 7};
static const int8_t  PW_LIST[4]  = {22, 19, 16, 6};              /* muc chip; dau ra danh dinh 30, 27, 24, 14 dBm (muc 14 dBm khong co o T30D) */
static const float   FQ_LIST[3]  = {920.5f, 921.5f, 922.5f};
static const char *const MOD_SHORT[SX_MOD_COUNT] = {"T30D", "M30S", "RA01SHP"};
static const char *const ROLE_NAMES[3] = {"GATEWAY", "NODE 1", "NODE 2"};
static const char *const CASE_NAMES[6] = {"EXP-01 R1", "EXP-02 R2", "EXP-03 D", "EXP-04 VACHAM", "EXP-05 RELAY", "EXP-06 ASYM"};

static settings_t S = {.ver = SET_VER, .mod = SX_MOD_M30S, .role = 0, .cs = 0, .sf = 2, .pw = 0, .fq = 1};
static int s_init_mod = -1;      /* module da khoi tao trong lan chay nay */

static void settings_load(void)
{
    nvs_handle_t h;
    if (nvs_open("bt", NVS_READONLY, &h) != ESP_OK) return;
    settings_t t;
    size_t n = sizeof t;
    if (nvs_get_blob(h, "set", &t, &n) == ESP_OK && n == sizeof t && t.ver == SET_VER && t.mod < SX_MOD_COUNT) S = t;
    nvs_close(h);
}

static void settings_save(void)
{
    nvs_handle_t h;
    if (nvs_open("bt", NVS_READWRITE, &h) != ESP_OK) return;
    nvs_set_blob(h, "set", &S, sizeof S);
    nvs_commit(h);
    nvs_close(h);
}

static void sanitize(void)
{
    if (S.mod >= SX_MOD_COUNT) S.mod = SX_MOD_M30S;
    if (S.role > 2) S.role = 0;
    if (S.cs > 5) S.cs = 0;
    if (S.sf > 5) S.sf = 2;
    if (S.pw > 3) S.pw = 3;
    if (S.mod == SX_MOD_T30D && S.pw > 2) S.pw = 2;
    if (S.fq > 2) S.fq = 1;
}

/* ------------------------- nhan hien thi ------------------------- */
static void speed_label(int sfi, char *o, size_t n)
{
    int sf = SF_LIST[sfi];
    if (S.mod == SX_MOD_T30D) {
        static const char *const a[6] = {"2.4k", "4.8k", "9.6k", "19.2k", "38.4k", "62.5k"};
        snprintf(o, n, "SF%d(%s)", sf, a[sfi]);
    } else {
        snprintf(o, n, "SF%d/125k", sf);
    }
}

static int pw_levels(void) { return S.mod == SX_MOD_T30D ? 3 : 4; }   /* T30D (UART) toi thieu 21 dBm */

static int8_t pw_chip(int pwi)
{
    if (pwi == 3) return S.mod == SX_MOD_RA01SHP ? 7 : 6;   /* dau ra 14 dBm = chip + 7 (Ra-01SH-P) hoac + 8 (M30S) */
    return PW_LIST[pwi];
}

static int nominal_dbm(int pwi)
{
    return sx_output_power_dbm(pw_chip(pwi));
}

static float   eff_freq(float f)    { return S.mod == SX_MOD_T30D ? 850.125f + lroundf(f - 850.125f) : f; }

/* ------------------------- nhap lieu bang nut ------------------------- */
static int edit_float(const char *title, float *v, float step, float lo, float hi)
{
    float x = *v;
    while (1) {
        led_tick();
        lcd_printf(0, "%s", title);
        lcd_printf(1, "<%.1f> U/D OK/BK", x);
        int e = btn_get(200);
        if (e == BTN_UP)        { x += step; if (x > hi) x = hi; }
        else if (e == BTN_DOWN) { x -= step; if (x < lo) x = lo; }
        else if (e == BTN_OK)   { *v = x; return BTN_OK; }
        else if (e == BTN_BACK) return BTN_BACK;
    }
}

static int pick(const char *title, const char *const *opts, int n, int *idx)
{
    int i = *idx;
    while (1) {
        led_tick();
        lcd_printf(0, "%s", title);
        lcd_printf(1, "UP/DN <%.8s>", opts[i]);
        int e = btn_get(200);
        if (e == BTN_UP)        i = (i + 1) % n;
        else if (e == BTN_DOWN) i = (i + n - 1) % n;
        else if (e == BTN_OK)   { *idx = i; return BTN_OK; }
        else if (e == BTN_BACK) return BTN_BACK;
    }
}

static void msg(const char *a, const char *b)
{
    lcd_printf(0, "%s", a ? a : "");
    lcd_printf(1, "%s", b ? b : "");
}

static const char *init_error_label(esp_err_t err)
{
    switch (err) {
    case ESP_ERR_INVALID_ARG:      return "ERR INVALID_ARG";
    case ESP_ERR_INVALID_RESPONSE: return "ERR INVALID_RESP";
    case ESP_ERR_TIMEOUT:          return "ERR TIMEOUT";
    default:                       return "ERR OTHER";
    }
}

static bool show_log_page(uint32_t *cursor)
{
    char top[17], bottom[17];
    if (!wifi_log_read_lcd(cursor, top, bottom)) return false;
    lcd_printf(0, "%s", top);
    lcd_printf(1, "%s", bottom);
    return true;
}

static uint32_t s_result_start;
static uint32_t s_result_end;

static void draw_result_page(uint32_t cursor, uint32_t end)
{
    char top[17], bottom[17];
    if (wifi_log_read_lcd_until(&cursor, end, top, bottom)) {
        lcd_printf(0, "%s", top);
        lcd_printf(1, "%s", bottom);
    } else {
        lcd_printf(0, "Khong co log");
        lcd_printf(1, "UP/DN BK=MENU");
    }
}

static int show_result_log(int first_event)
{
    uint32_t page = wifi_log_lcd_previous_page(s_result_end, s_result_start);
    draw_result_page(page, s_result_end);
    int e = first_event;
    while (1) {
        led_tick();
        if (e == BTN_NONE) e = btn_get(200);
        if (e == BTN_UP) {
            uint32_t previous = wifi_log_lcd_previous_page(page, s_result_start);
            if (previous < page) {
                page = previous;
                draw_result_page(page, s_result_end);
            }
        } else if (e == BTN_DOWN) {
            char top[17], bottom[17];
            uint32_t next = page;
            if (wifi_log_read_lcd_until(&next, s_result_end, top, bottom) && next < s_result_end) {
                page = next;
                draw_result_page(page, s_result_end);
            }
        } else if (e == BTN_OK || e == BTN_BACK) {
            led_set(LED_WAIT);
            return e;
        }
        e = BTN_NONE;
    }
}

static int wait_ok_back(void)
{
    while (1) {
        led_tick();
        int e = btn_get(200);
        if (e == BTN_OK || e == BTN_BACK) return e;
    }
}

/* ------------------------- chay 1 lenh console cu ------------------------- */
static volatile bool s_busy;
static char          s_cmd[96];

static void worker(void *arg)
{
    int ret = 0;
    esp_err_t err = esp_console_run(s_cmd, &ret);
    if (err != ESP_OK) printf("Lenh '%s' that bai: %s\n", s_cmd, esp_err_to_name(err));
    s_busy = false;
    vTaskDelete(NULL);
}

static bool confirm_stop(void)
{
    msg("DUNG BAI TEST?", "OK=dung BK=tiep");
    int e;
    do { e = btn_get(5000); } while (e == BTN_UP || e == BTN_DOWN);
    return e == BTN_OK;
}

/* Tra ve true neu bi nguoi dung huy. */
static bool run_cmd(const char *cmd, const char *verdict_prefix, const char *title)
{
    strncpy(s_cmd, cmd, sizeof s_cmd - 1);
    s_cmd[sizeof s_cmd - 1] = '\0';
    wifi_log_set_verdict_prefix(verdict_prefix);
    uint32_t log_cursor = wifi_log_cursor();
    s_result_start = log_cursor;
    printf("\n> %s\n", s_cmd);                    /* dong tieu de lenh trong log */
    g_abort = false;
    s_busy = true;
    if (xTaskCreate(worker, "cmd", 12288, NULL, 5, NULL) != pdPASS) {
        s_busy = false;
        printf("Khong du bo nho de tao task chay bai test\n");
        led_set(LED_ERROR);
        return false;
    }
    led_set(LED_RUN);

    char spd[16];
    speed_label(S.sf, spd, sizeof spd);
    int64_t last = 0;
    bool aborted = false;
    bool shown_logs = false;
    while (s_busy) {
        led_tick();
        int e = btn_get(120);
        if (e == BTN_BACK && !aborted && confirm_stop()) {
            g_abort = true;                      /* code cu thoat o lan kiem tra ke tiep */
            aborted = true;
            msg("Dang dung...", "");
            int64_t dl = esp_timer_get_time() + 6000000;
            while (s_busy && esp_timer_get_time() < dl) { led_tick(); vTaskDelay(pdMS_TO_TICKS(50)); }
            if (s_busy) { settings_save(); esp_restart(); }    /* ham cu bi ket: khoi dong lai nhu "reset board" truoc day */
            break;
        }
        if (esp_timer_get_time() - last > 800000) {
            last = esp_timer_get_time();
            if (show_log_page(&log_cursor)) {
                shown_logs = true;
            } else if (!shown_logs) {
                lcd_printf(0, "%s CHAY", title);
                lcd_printf(1, "OUT%d TX%d", nominal_dbm(S.pw), pw_chip(S.pw));
            }
        }
    }
    s_result_end = wifi_log_cursor();
    led_set(aborted ? LED_WAIT : LED_DONE);
    return aborted;
}

/* Man hinh ket thuc; UP/DOWN xem log, OK tiep tuc, BACK ve menu chinh. */
static int show_done(const char *title, bool aborted)
{
    int v = wifi_log_verdict();
    const char *result = aborted ? "HUY" : (v == 1 ? "PASS" : (v == 2 ? "FAIL" : "LOG"));
    printf("Ket qua %s: %s\n", title, result);
    s_result_end = wifi_log_cursor();
    while (1) {
        lcd_printf(0, "%s %s", title, result);
        lcd_printf(1, "OK=TIEP U/D=LOG");
        led_tick();
        int e = btn_get(200);
        if (e == BTN_OK || e == BTN_BACK) {
            led_set(LED_WAIT);
            return e;
        }
        if (e == BTN_UP || e == BTN_DOWN) {
            e = show_result_log(e);
            if (e == BTN_BACK) return BTN_BACK;
        }
    }
}

static void run_quick(const char *cmd)           /* lenh ngan, khong can theo doi */
{
    int r = 0;
    uint32_t log_cursor = wifi_log_cursor();
    printf("\n> %s\n", cmd);
    esp_err_t err = esp_console_run(cmd, &r);
    if (err != ESP_OK) printf("Lenh '%s' that bai: %s\n", cmd, esp_err_to_name(err));
    while (show_log_page(&log_cursor)) vTaskDelay(pdMS_TO_TICKS(800));
}

/* ------------------------- cac bai test ------------------------- */
static void flow_poll(int node)
{
    static float km = 1.0f;
    char title[24], cmd[64];
    snprintf(title, sizeof title, "EXP-0%d N%d", node, node);
    while (1) {
        led_set(LED_WAIT);
        if (edit_float("Khoang cach (km)", &km, 0.1f, 0.0f, 30.0f) != BTN_OK) break;
        snprintf(cmd, sizeof cmd, "gw_poll %d %.1f 200", node, km);
        bool ab = run_cmd(cmd, "Dat (PDR", title);
        if (show_done(title, ab) == BTN_BACK) return;
    }
    run_quick("gw_points");                      /* bang tong hop R_max */
}

static void flow_tdma(int exp)
{
    float r1 = 0, r2 = 0;
    char cmd[80], title[24], pre[12];
    snprintf(title, sizeof title, "EXP-0%d", exp);
    snprintf(pre, sizeof pre, "EXP-0%d:", exp);
    if (edit_float(exp == 3 ? "R1 (km)" : "R_LOS (km)", &r1, 0.1f, 0, 30) != BTN_OK) return;
    if (edit_float(exp == 3 ? "R2 (km)" : "R_NLOS (km)", &r2, 0.1f, 0, 30) != BTN_OK) return;
    if (exp == 6) {
        char sl[6][16];
        char pl[4][12];
        const char *so[6];
        const char *po[4];
        for (int i = 0; i < 6; i++) { speed_label(i, sl[i], sizeof sl[i]); so[i] = sl[i]; }
        for (int i = 0; i < pw_levels(); i++) {
            snprintf(pl[i], sizeof pl[i], "%d dBm", nominal_dbm(i));
            po[i] = pl[i];
        }
        int s1 = S.sf, p1 = S.pw, s2 = S.sf, p2 = S.pw;
        if (pick("Node 1 toc do", so, 6, &s1) != BTN_OK) return;
        if (pick("Node 1 cong suat", po, pw_levels(), &p1) != BTN_OK) return;
        if (pick("Node 2 toc do", so, 6, &s2) != BTN_OK) return;
        if (pick("Node 2 cong suat", po, pw_levels(), &p2) != BTN_OK) return;
        snprintf(cmd, sizeof cmd, "gw_nodecfg 1 %d %d", SF_LIST[s1], nominal_dbm(p1));
        run_quick(cmd);
        snprintf(cmd, sizeof cmd, "gw_nodecfg 2 %d %d", SF_LIST[s2], nominal_dbm(p2));
        run_quick(cmd);
        msg("Dat dung SF Node", "OK=chay BK=huy");
        int e;
        do { e = btn_get(200); } while (e != BTN_OK && e != BTN_BACK);
        if (e == BTN_BACK) return;
        snprintf(cmd, sizeof cmd, "gw_tdma 720 3600 %.1f %.1f", r1, r2);
    } else {
        snprintf(cmd, sizeof cmd, "gw_tdma 500 1800 %.1f %.1f", r1, r2);
    }
    bool ab = run_cmd(cmd, pre, title);
    show_done(title, ab);
}

static void flow_collision(void)
{
    static const char *const modes[2] = {"ALOHA", "CAD/LBT"};
    int cad = 0;
    char cmd[64];
    if (pick("EXP-04 kich ban", modes, 2, &cad) != BTN_OK) return;
    snprintf(cmd, sizeof cmd, "gw_listen 300 %d", cad);
    bool ab = run_cmd(cmd, "Mat goi TB", "EXP-04 GW");
    show_done("EXP-04 GW", ab);
}

static void flow_node(int id)
{
    char cmd[64], title[24];
    const char *pre = NULL;
    snprintf(title, sizeof title, "NODE %d %s", id, CASE_NAMES[S.cs]);
    if (S.cs == 3 || (S.cs == 4 && id == 1)) {               /* bai co thu tu khoi dong */
        msg(S.cs == 3 ? "GW nghe truoc" : "GW + N2 truoc", "OK=chay BK=huy");
        if (wait_ok_back() == BTN_BACK) return;
    }
    if (S.cs == 3) {                                         /* EXP-04: node gui uplink */
        static const char *const modes[2] = {"ALOHA", "CAD/LBT"};
        int cad = 0;
        if (pick("EXP-04 kich ban", modes, 2, &cad) != BTN_OK) return;
        snprintf(cmd, sizeof cmd, "node_up %d 2000 300 %d", id, cad);
    } else if (S.cs == 4 && id == 1) {                       /* EXP-05: Node 1 la nguon relay */
        snprintf(cmd, sizeof cmd, "n_relay 1 2 100 5");
        pre = "Chuyen tiep thanh cong";
    } else {                                                 /* cac bai con lai: node cho */
        snprintf(cmd, sizeof cmd, "node %d", id);
    }
    bool ab = run_cmd(cmd, pre, title);
    show_done(title, ab);
}

/* ------------------------- menu ------------------------- */
static void change(int item, int dir)
{
    switch (item) {
    case IT_MOD:  S.mod = (uint8_t)((S.mod + SX_MOD_COUNT + dir) % SX_MOD_COUNT);
                  sx_select_module((sx_module_t)S.mod);                    /* de bang tinh dBm dau ra dung theo module dang chon */
                  if (S.pw >= pw_levels()) S.pw = (uint8_t)(pw_levels() - 1);
                  break;
    case IT_ROLE: S.role = (uint8_t)((S.role + 3 + dir) % 3); break;
    case IT_CASE: S.cs = (uint8_t)((S.cs + 6 + dir) % 6); break;
    case IT_SF:   S.sf = (uint8_t)((S.sf + 6 + dir) % 6); break;
    case IT_PW:   S.pw = (uint8_t)((S.pw + pw_levels() + dir) % pw_levels()); break;
    case IT_FQ:   S.fq = (uint8_t)((S.fq + 3 + dir) % 3); break;
    default: break;
    }
}

static void draw_menu(int cur, int top)
{
    for (int row = 0; row < LCD_ROWS; row++) {
        int it = top + row;
        char txt[28] = "", sp[16];
        switch (it) {
        case IT_MOD:   snprintf(txt, sizeof txt, "Mod:%s", MOD_SHORT[S.mod]); break;
        case IT_ROLE:  snprintf(txt, sizeof txt, "Vai tro:%s", ROLE_NAMES[S.role]); break;
        case IT_CASE:  snprintf(txt, sizeof txt, "T:%s", CASE_NAMES[S.cs]); break;
        case IT_SF:    speed_label(S.sf, sp, sizeof sp); snprintf(txt, sizeof txt, "TD:%s", sp); break;
        case IT_PW:    snprintf(txt, sizeof txt, "OUT:%d dBm", nominal_dbm(S.pw)); break;
        case IT_FQ:    snprintf(txt, sizeof txt, "F:%.3f MHz", eff_freq(FQ_LIST[S.fq])); break;
        case IT_START: snprintf(txt, sizeof txt, ">> BAT DAU <<"); break;
        default: break;
        }
        lcd_printf(row, "%c%s", it == cur ? '>' : ' ', txt);
    }
}

static void start_test(void)
{
    /* doi module: driver cu dang giu chan -> khoi dong lai de giai phong sach (cai dat da duoc luu) */
    if (s_init_mod >= 0 && s_init_mod != S.mod) {
        settings_save();
        msg("Doi module...", "Khoi dong lai");
        vTaskDelay(pdMS_TO_TICKS(1200));
        esp_restart();
    }
    sx_select_module((sx_module_t)S.mod);

    if (s_init_mod < 0) {
        g_cfg.freq_mhz = FQ_LIST[S.fq];
        g_cfg.sf = SF_LIST[S.sf];
        g_cfg.power = pw_chip(S.pw);
        msg("Khoi tao module", MOD_SHORT[S.mod]);
        led_set(LED_RUN);
        esp_err_t e = sx_init(&g_cfg);
        if (e != ESP_OK) {
            ESP_LOGE(TAG, "sx_init loi: %s", esp_err_to_name(e));
            msg("LOI KHOI TAO 1", init_error_label(e));
            led_set(LED_ERROR);
            wait_ok_back();
            led_set(LED_WAIT);
            return;
        }
        s_init_mod = S.mod;
    }
    settings_save();

    char cmd[64];
    snprintf(cmd, sizeof cmd, "cfg %.3f %d 125 %d", FQ_LIST[S.fq], SF_LIST[S.sf], nominal_dbm(S.pw));
    run_quick(cmd);                                          /* ap dung toc do / cong suat / tan so da chon */

    if (S.role == 0) {
        switch (S.cs) {
        case 0: flow_poll(1); break;
        case 1: flow_poll(2); break;
        case 2: flow_tdma(3); break;
        case 3: flow_collision(); break;
        case 4: { bool ab = run_cmd("gw_relay", NULL, "EXP-05 GW"); show_done("EXP-05 GW", ab); break; }
        default: flow_tdma(6); break;
        }
    } else {
        flow_node(S.role);
    }
    led_set(LED_WAIT);
}

void ui_run(void)
{
    btn_init();
    led_init();
    if (lcd_init() != ESP_OK) ESP_LOGE(TAG, "LCD loi - van chay, xem log qua Wi-Fi");
    settings_load();
    sanitize();

    lcd_clear();
    lcd_printf(0, "BROAD TEST LORA");
    lcd_printf(1, "WiFi: LoRaBroadTest");
    led_set(LED_ERROR);                                      /* thu 3 den luc khoi dong */
    for (int i = 0; i < 8; i++) { led_tick(); vTaskDelay(pdMS_TO_TICKS(150)); }
    led_set(LED_WAIT);

    int cur = IT_START, top = IT_COUNT - LCD_ROWS;
    while (1) {
        if (cur < top) top = cur;
        if (cur > top + LCD_ROWS - 1) top = cur - (LCD_ROWS - 1);
        draw_menu(cur, top);
        int ev = btn_get(300);
        led_tick();
        switch (ev) {
        case BTN_UP:   cur = (cur + IT_COUNT - 1) % IT_COUNT; break;
        case BTN_DOWN: cur = (cur + 1) % IT_COUNT; break;
        case BTN_OK:
            if (cur == IT_START) { start_test(); lcd_clear(); }
            else                 { change(cur, +1); sanitize(); }
            break;
        case BTN_BACK:
            if (cur != IT_START) { change(cur, -1); sanitize(); }
            break;
        default: break;
        }
    }
}
