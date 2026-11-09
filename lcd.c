#include "lcd.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "board.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "esp_timer.h"

static const char *TAG = "lcd";

static i2c_master_bus_handle_t s_bus;
static i2c_master_dev_handle_t s_dev;
static bool    s_ok;
/* Anh xa chan PCF8574 -> LCD. MAP 0 (pho bien): RS=P0 RW=P1 EN=P2 BL=P3 D4..D7=P4..P7.
   MAP 1 (mot so mach khac): D4..D7=P0..P3 EN=P4 RW=P5 RS=P6 BL=P7. */
#if LCD_PCF_MAP == 0
#define M_RS 0x01
#define M_EN 0x04
#define M_BL 0x08
#define D_SHIFT 4
#else
#define M_RS 0x40
#define M_EN 0x10
#define M_BL 0x80
#define D_SHIFT 0
#endif
static uint8_t s_bl = M_BL;
static char    s_cache[LCD_ROWS][LCD_COLS + 1];
#define LCD_REFRESH_US 15000000LL                /* 15 s: du de tu chua nhieu, khong gay nhap nhay */
static int64_t s_last[LCD_ROWS];                  /* lan ghi that gan nhat (us) de dinh ky ghi lai */
static SemaphoreHandle_t s_mx;

/* Tick FreeRTOS thuong 10 ms: pdMS_TO_TICKS(1..9) = 0 -> KHONG cho gi. Cho ngan dung busy-wait cua ROM. */
static void dly_ms(int ms) { if (ms >= 20) vTaskDelay(pdMS_TO_TICKS(ms)); else esp_rom_delay_us((uint32_t)ms * 1000); }

/* PCF8574 (mach thuong gap): P0=RS P1=RW P2=EN P3=BL P4..P7=D4..D7 */
static void pcf(uint8_t v)     { if (s_ok) i2c_master_transmit(s_dev, &v, 1, 50); }
static void pulse(uint8_t d)   { pcf(d | M_EN); esp_rom_delay_us(2); pcf(d & (uint8_t)~M_EN); esp_rom_delay_us(50); }
static void write4(uint8_t n, uint8_t rs) { pulse((uint8_t)((n << D_SHIFT) | s_bl | (rs ? M_RS : 0))); }
static void send(uint8_t b, uint8_t rs)   { write4(b >> 4, rs); write4(b & 0x0F, rs); }

esp_err_t lcd_init(void)
{
    i2c_master_bus_config_t bc = {.i2c_port = I2C_NUM_0, .sda_io_num = PIN_SDA, .scl_io_num = PIN_SCL,
                                  .clk_source = I2C_CLK_SRC_DEFAULT, .glitch_ignore_cnt = 7,
                                  .flags.enable_internal_pullup = LCD_I2C_INTERNAL_PULLUP};
    esp_err_t e = i2c_new_master_bus(&bc, &s_bus);
    if (e != ESP_OK) return e;

    uint8_t addr = 0;
    if (i2c_master_probe(s_bus, 0x27, 100) == ESP_OK)      addr = 0x27;
    else if (i2c_master_probe(s_bus, 0x3F, 100) == ESP_OK) addr = 0x3F;
    if (!addr) { ESP_LOGE(TAG, "Khong thay LCD tren I2C (0x27/0x3F)"); return ESP_ERR_NOT_FOUND; }

    i2c_device_config_t dc = {.dev_addr_length = I2C_ADDR_BIT_LEN_7, .device_address = addr, .scl_speed_hz = LCD_I2C_HZ};
    e = i2c_master_bus_add_device(s_bus, &dc, &s_dev);
    if (e != ESP_OK) return e;
    s_ok = true;

    s_mx = xSemaphoreCreateMutex();
    pcf(s_bl);
    vTaskDelay(pdMS_TO_TICKS(250));                /* LCD can >= 40 ms sau khi nguon 5 V on dinh; cho du lau de khoi ket qua nhieu luc cap nguon */
    write4(0x3, 0); dly_ms(5);                     /* chuoi reset bat buoc cua HD44780 (che do 4 bit) */
    write4(0x3, 0); dly_ms(5);
    write4(0x3, 0); dly_ms(2);
    write4(0x2, 0); dly_ms(2);                     /* chuyen sang 4 bit */
    send(0x28, 0); dly_ms(1);                      /* 4 bit, 2 dong logic, font 5x8 */
    send(0x08, 0); dly_ms(1);                      /* tat hien thi */
    send(0x01, 0); dly_ms(5);                      /* xoa (can ~1.6 ms) */
    send(0x06, 0); dly_ms(1);                      /* tang con tro */
    send(0x0C, 0); dly_ms(1);                      /* bat hien thi, an con tro */
    memset(s_cache, 0, sizeof s_cache);
    return ESP_OK;
}

void lcd_clear(void)
{
    if (s_mx) xSemaphoreTake(s_mx, portMAX_DELAY);
    send(0x01, 0);
    dly_ms(5);
    memset(s_cache, 0, sizeof s_cache);
    memset(s_last, 0, sizeof s_last);
    if (s_mx) xSemaphoreGive(s_mx);
}

void lcd_printf(int row, const char *fmt, ...)
{
    if (row < 0 || row >= LCD_ROWS) return;
    char tmp[64];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(tmp, sizeof tmp, fmt, ap);
    va_end(ap);

    char line[LCD_COLS + 1];
    for (int i = 0; i < LCD_COLS; i++) {
        char c = i < (int)strlen(tmp) ? tmp[i] : ' ';
        line[i] = (c >= 32 && c < 127) ? c : '?';
    }
    line[LCD_COLS] = '\0';
    int64_t now = esp_timer_get_time();
    /* Khong doi -> khong ghi I2C, TRU khi da qua LCD_REFRESH_US: ghi lai chi phan du lieu de tu chua man hinh bi nhieu */
    if (!strcmp(line, s_cache[row]) && now - s_last[row] < LCD_REFRESH_US) return;
    strcpy(s_cache[row], line);
    s_last[row] = now;

    if (s_mx) xSemaphoreTake(s_mx, portMAX_DELAY);
    static const uint8_t base[LCD_ROWS] = {0x00, 0x40};
    send(0x80 | base[row], 0);
    for (int i = 0; i < LCD_COLS; i++) send((uint8_t)line[i], 1);
    if (s_mx) xSemaphoreGive(s_mx);
}
