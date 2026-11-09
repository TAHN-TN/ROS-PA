#include "drv_e22uart.h"
#include "board.h"

#include <math.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "e22uart";

#define UART_PORT  UART_NUM_2
#define PIN_TXD    PIN_IO17
#define PIN_RXD    PIN_IO16
#define PIN_M0     PIN_IO25
#define PIN_M1     PIN_IO26
#define PIN_AUX    PIN_IO4
#define BAUD_CFG   9600        /* che do cau hinh luon 9600 8N1 */
#define BAUD_RUN   115200      /* toc do UART khi chay (REG0 bit7:5 = 111) */
#define GAP_MS     12          /* im lang > 12 ms tren UART = het 1 goi vo tuyen */

static const uint32_t air_bps[6]  = {2400, 4800, 9600, 19200, 38400, 62500};
static const uint8_t  air_code[6] = {0x02, 0x03, 0x04, 0x05, 0x06, 0x07};   /* REG0 bit2:0 */
static const char    *air_name[6] = {"2.4k", "4.8k", "9.6k", "19.2k", "38.4k", "62.5k"};

static e22u_cfg_t s_cfg;
static bool       s_inited;

float       e22u_freq_mhz(const e22u_cfg_t *c)  { return 850.125f + c->channel; }
int         e22u_power_dbm(const e22u_cfg_t *c) { return E22U_MAX_DBM - 3 * c->power_idx; }
const char *e22u_air_name(uint8_t i)            { return i < 6 ? air_name[i] : "?"; }
bool e22u_cfg_equal(const e22u_cfg_t *a, const e22u_cfg_t *b)
{
    return a->channel == b->channel && a->air_idx == b->air_idx && a->power_idx == b->power_idx && a->lbt == b->lbt;
}

/* AUX: cao = module ranh; thap = dang tu kiem tra / nhan UART / phat vo tuyen / xuat du lieu */
static bool wait_aux(int level, uint32_t timeout_ms)
{
    int64_t dl = esp_timer_get_time() + (int64_t)timeout_ms * 1000;
    while (gpio_get_level(PIN_AUX) != level) {
        if (esp_timer_get_time() > dl) return false;
        vTaskDelay(1);
    }
    return true;
}

static void set_mode(int m0, int m1)
{
    gpio_set_level(PIN_M0, m0);
    gpio_set_level(PIN_M1, m1);
    vTaskDelay(pdMS_TO_TICKS(15));
    if (!wait_aux(1, 2000)) {
        ESP_LOGW(TAG, "AUX timeout khi doi mode M0=%d M1=%d", m0, m1);
    }
    vTaskDelay(pdMS_TO_TICKS(30));
}

static void build_cmd(const e22u_cfg_t *c, uint8_t r[12])
{
    r[0] = 0xC2;                                  /* C0 = luu khi mat dien, C2 = khong luu */
    r[1] = 0x00; r[2] = 0x09;                     /* dia chi bat dau 0, do dai 9 */
    r[3] = 0x00; r[4] = 0x00; r[5] = 0x00;        /* ADDH, ADDL, NETID (dung chung de phat quang ba) */
    r[6] = (7 << 5) | (0 << 3) | air_code[c->air_idx];   /* REG0: UART 115200, 8N1, toc do khong khi */
    r[7] = (0 << 6) | (0 << 5) | (c->power_idx & 3);     /* REG1: goi con 240 B, khong do nhieu moi truong, cong suat */
    r[8] = c->channel;                                   /* REG2: kenh */
    r[9] = 0x80 | (c->lbt ? 0x10 : 0x00) | 0x03;         /* REG3: bat RSSI byte, truyen trong suot, LBT, WOR 2000 ms */
    r[10] = 0x00; r[11] = 0x00;                          /* khoa ma hoa */
}

esp_err_t e22u_configure(const e22u_cfg_t *c)
{
    if (c->channel > 80 || c->air_idx > 5 || c->power_idx > 3) return ESP_ERR_INVALID_ARG;

    set_mode(E22U_CFG_M0, E22U_CFG_M1);
    uart_set_baudrate(UART_PORT, BAUD_CFG);
    uart_flush_input(UART_PORT);

    uint8_t cmd[12], resp[12] = {0};
    build_cmd(c, cmd);
    uart_write_bytes(UART_PORT, cmd, sizeof cmd);
    uart_wait_tx_done(UART_PORT, pdMS_TO_TICKS(200));
    int n = uart_read_bytes(UART_PORT, resp, sizeof resp, pdMS_TO_TICKS(1000));

    esp_err_t ret = ESP_OK;
    if (n != 12 || resp[0] != 0xC1 || memcmp(resp + 3, cmd + 3, 7) != 0) {
        ESP_LOGE(TAG, "Module khong xac nhan cau hinh (nhan %d byte). Kiem tra day, M0/M1, AUX, nguon.", n);
        ret = ESP_FAIL;
    }

    set_mode(0, 0);                               /* ve che do truyen/nhan binh thuong */
    uart_set_baudrate(UART_PORT, BAUD_RUN);
    uart_flush_input(UART_PORT);
    if (ret == ESP_OK) s_cfg = *c;
    return ret;
}

esp_err_t e22u_set_lbt(bool on)
{
    e22u_cfg_t c = s_cfg;
    c.lbt = on;
    return e22u_configure(&c);
}

esp_err_t e22u_read_config(uint8_t out[12])
{
    set_mode(E22U_CFG_M0, E22U_CFG_M1);
    uart_set_baudrate(UART_PORT, BAUD_CFG);
    uart_flush_input(UART_PORT);
    const uint8_t rd[3] = {0xC1, 0xC1, 0xC1};
    uart_write_bytes(UART_PORT, rd, 3);
    uart_wait_tx_done(UART_PORT, pdMS_TO_TICKS(100));
    int n = uart_read_bytes(UART_PORT, out, 12, pdMS_TO_TICKS(1000));
    set_mode(0, 0);
    uart_set_baudrate(UART_PORT, BAUD_RUN);
    uart_flush_input(UART_PORT);
    return (n == 12 && out[0] == 0xC1) ? ESP_OK : ESP_FAIL;
}

esp_err_t e22u_init(const e22u_cfg_t *cfg)
{
    if (!s_inited) {
        gpio_config_t o = {.pin_bit_mask = (1ULL << PIN_M0) | (1ULL << PIN_M1), .mode = GPIO_MODE_OUTPUT};
        gpio_config(&o);
        gpio_config_t a = {.pin_bit_mask = 1ULL << PIN_AUX, .mode = GPIO_MODE_INPUT};
        gpio_config(&a);

        const uart_config_t uc = {.baud_rate = BAUD_CFG, .data_bits = UART_DATA_8_BITS, .parity = UART_PARITY_DISABLE,
                                  .stop_bits = UART_STOP_BITS_1, .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
                                  .source_clk = UART_SCLK_DEFAULT};
        esp_err_t e = uart_driver_install(UART_PORT, 2048, 0, 0, NULL, 0);
        if (e != ESP_OK) return e;
        uart_param_config(UART_PORT, &uc);
        uart_set_pin(UART_PORT, PIN_TXD, PIN_RXD, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
        s_inited = true;
    }
    gpio_set_level(PIN_M0, 0);
    gpio_set_level(PIN_M1, 0);
    if (!wait_aux(1, 3000)) { ESP_LOGE(TAG, "AUX khong len cao sau khi cap nguon - kiem tra day AUX/nguon"); return ESP_ERR_TIMEOUT; }

    esp_err_t r = e22u_configure(cfg);
    if (r != ESP_OK) return r;

    uint8_t reg[12] = {0};
    if (e22u_read_config(reg) == ESP_OK)
        ESP_LOGI(TAG, "Cau hinh module: %02X %02X %02X | ADDH %02X ADDL %02X NETID %02X | REG0 %02X REG1 %02X REG2 %02X REG3 %02X",
                 reg[0], reg[1], reg[2], reg[3], reg[4], reg[5], reg[6], reg[7], reg[8], reg[9]);
    return ESP_OK;
}

/* Phat: cho AUX cao (ranh) -> ghi UART -> AUX xuong (dang xu ly) -> AUX len cao (phat xong) */
esp_err_t e22u_transmit(const uint8_t *d, uint8_t len, uint32_t timeout_ms)
{
    if (!wait_aux(1, timeout_ms)) return ESP_ERR_TIMEOUT;
    uart_write_bytes(UART_PORT, d, len);
    uart_wait_tx_done(UART_PORT, pdMS_TO_TICKS(200));
    wait_aux(0, 100);                              /* co the bo lo neu qua nhanh, khong bat buoc */
    return wait_aux(1, timeout_ms) ? ESP_OK : ESP_ERR_TIMEOUT;
}

/* Nhan: gom byte cho den khi UART im lang GAP_MS. Byte cuoi la RSSI (dBm = RSSI - 256). */
radio_rx_status_t e22u_receive(uint8_t *buf, uint8_t maxlen, uint32_t timeout_ms, radio_rx_info_t *info)
{
    uint8_t tmp[260];
    int n = uart_read_bytes(UART_PORT, tmp, 1, pdMS_TO_TICKS(timeout_ms));
    if (n <= 0) return RADIO_RX_TIMEOUT;
    while (n < (int)sizeof tmp) {
        int k = uart_read_bytes(UART_PORT, tmp + n, sizeof tmp - n, pdMS_TO_TICKS(GAP_MS));
        if (k <= 0) break;
        n += k;
    }
    if (n < 2) return RADIO_RX_ERR;
    int dl = n - 1;
    if (dl > maxlen) dl = maxlen;
    memcpy(buf, tmp, dl);
    if (info) {
        info->len      = (uint8_t)dl;
        info->rssi     = (float)tmp[n - 1] - 256.0f;
        info->snr      = NAN;
        info->sig_rssi = NAN;
    }
    return RADIO_RX_OK;
}

void      e22u_ensure_rx(void) {}
esp_err_t e22u_cad(bool *detected) { if (detected) *detected = false; return ESP_OK; }

uint32_t e22u_airtime_us(const e22u_cfg_t *c, uint8_t len)
{
    double t = (len + 12) * 8.0 / air_bps[c->air_idx];                 /* thoi gian bit thuan + ~12 byte overhead */
    return (uint32_t)((t * E22U_AIR_SCALE_PCT / 100.0 + 0.15) * 1e6);   /* + 150 ms xu ly module/UART */
}
