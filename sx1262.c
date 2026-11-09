/*
 * Lop radio hop nhat cho Broad Test: giu nguyen giao dien sx_* cua ban SPI cu,
 * chon driver theo loai module dang cam (T30D UART / M30S SPI / Ra-01SH-P SPI).
 */
#include "sx1262.h"

#include <math.h>

#include "board.h"
#include "drv_e22uart.h"
#include "drv_sx126x.h"
#include "esp_log.h"

static const char *TAG = "radio";

static sx_module_t s_mod = SX_MOD_M30S;

/* M30S: RXEN/TXEN do ESP32 dieu khien, TCXO 1.8 V (da kiem chung bang log: tcxo=1800 mV + DC-DC chay duoc) */
static const sx126x_profile_t PROF_M30S = {.tcxo_mv = 1800, .dio2_rfsw = false,
                                           .pin_rxen = PIN_RXEN, .pin_txen = PIN_TXEN, .pin_rfen = -1};
/* Ra-01SH-P: RF_EN bat bang IO25, DIO2 dieu khien cong tac RF. CAN doi chieu datasheet Ra-01SH-P. */
static const sx126x_profile_t PROF_RA = {.tcxo_mv = 0, .dio2_rfsw = true,
                                         .pin_rxen = -1, .pin_txen = -1, .pin_rfen = PIN_IO25};

void sx_select_module(sx_module_t m) { if (m >= SX_MOD_T30D && m < SX_MOD_COUNT) s_mod = m; }
sx_module_t sx_get_module(void)             { return s_mod; }

static bool is_uart(void) { return s_mod == SX_MOD_T30D; }

static const sx126x_profile_t *prof(void) { return s_mod == SX_MOD_RA01SHP ? &PROF_RA : &PROF_M30S; }

int sx_output_power_dbm(int8_t chip_power_dbm)
{
    return chip_power_dbm + (s_mod == SX_MOD_RA01SHP ? 7 : 8);
}

bool sx_chip_power_from_output(int output_power_dbm, int8_t *chip_power_dbm)
{
    if (!chip_power_dbm) return false;
    int8_t chip = (int8_t)(output_power_dbm - (s_mod == SX_MOD_RA01SHP ? 7 : 8));
    if (chip < -9 || chip > 22) return false;

    bool supported = s_mod == SX_MOD_RA01SHP
        ? output_power_dbm == 14 || output_power_dbm == 23 || output_power_dbm == 26 || output_power_dbm == 29
        : output_power_dbm == 14 || output_power_dbm == 24 || output_power_dbm == 27 || output_power_dbm == 30;
    if (!supported) return false;
    *chip_power_dbm = chip;
    return true;
}

/* ---- anh xa cau hinh chung -> tung driver ---- */
static sx126x_cfg_t to_spi(const lora_cfg_t *c)
{
    sx126x_cfg_t s = {.freq_mhz = c->freq_mhz, .sf = c->sf, .bw_khz = c->bw_khz, .cr = c->cr, .power = c->power,
                      .preamble = c->preamble};
    return s;
}

static e22u_cfg_t to_uart(const lora_cfg_t *c)
{
    int ch = (int)lroundf(c->freq_mhz - 850.125f);
    if (ch < 0) ch = 0;
    if (ch > 80) ch = 80;
    int air = c->sf >= 12 ? 0 : (c->sf == 11 ? 1 : (c->sf == 10 ? 2 : (c->sf == 9 ? 3 : (c->sf == 8 ? 4 : 5))));
    int pw = (22 - c->power + 1) / 3;                      /* 22,19,16,13 dBm -> muc 0..3 */
    if (pw < 0) pw = 0;
    if (pw > 3) pw = 3;
    e22u_cfg_t u = {.channel = (uint8_t)ch, .air_idx = (uint8_t)air, .power_idx = (uint8_t)pw, .lbt = false};
    return u;
}

static void to_info(const radio_rx_info_t *r, rx_info_t *o)
{
    if (!o) return;
    o->len = r->len; o->rssi = r->rssi; o->snr = r->snr; o->sig_rssi = r->sig_rssi;
}

/* ---- API ---- */
esp_err_t sx_init(const lora_cfg_t *cfg)
{
    if (!cfg) return ESP_ERR_INVALID_ARG;

    if (is_uart()) {
        e22u_cfg_t u = to_uart(cfg);
        ESP_LOGI(TAG, "Module=E22-900T30D (UART), channel=%u, air_idx=%u, power_idx=%u",
                 (unsigned)u.channel, (unsigned)u.air_idx, (unsigned)u.power_idx);
        return e22u_init(&u);
    }

    sx126x_cfg_t c = to_spi(cfg);
    sx126x_profile_t p = *prof();
    ESP_LOGI(TAG, "Module=%s (SPI), TCXO=%d mV, DIO2_RF=%s, RXEN=%d, TXEN=%d, RF_EN=%d",
             s_mod == SX_MOD_RA01SHP ? "Ra-01SH-P" : "E22-900M30S",
             p.tcxo_mv, p.dio2_rfsw ? "on" : "off", p.pin_rxen, p.pin_txen, p.pin_rfen);

    /* Do not silently retry with a different oscillator profile: that can hide a
       hardware/profile mismatch. Keep the real initialization error for diagnosis. */
    return sx126x_init(&c, &p);
}

esp_err_t sx_configure(const lora_cfg_t *cfg)
{
    if (!cfg) return ESP_ERR_INVALID_ARG;
    if (is_uart()) { e22u_cfg_t u = to_uart(cfg); return e22u_configure(&u); }
    sx126x_cfg_t c = to_spi(cfg);
    return sx126x_configure(&c);
}

esp_err_t sx_set_lbt(bool on) { return is_uart() ? e22u_set_lbt(on) : ESP_OK; }

esp_err_t sx_transmit(const uint8_t *d, uint8_t len, uint32_t timeout_ms)
{
    return is_uart() ? e22u_transmit(d, len, timeout_ms) : sx126x_transmit(d, len, timeout_ms);
}

sx_rx_status_t sx_receive(uint8_t *buf, uint8_t maxlen, uint32_t timeout_ms, rx_info_t *info)
{
    radio_rx_info_t ri = {0};
    radio_rx_status_t s = is_uart() ? e22u_receive(buf, maxlen, timeout_ms, &ri) : sx126x_receive(buf, maxlen, timeout_ms, &ri);
    switch (s) {
    case RADIO_RX_OK:      to_info(&ri, info); return SX_RX_OK;
    case RADIO_RX_TIMEOUT: return SX_RX_TIMEOUT;
    case RADIO_RX_CRC_ERR: return SX_RX_CRC_ERR;
    default:               return SX_RX_ERR;
    }
}

void sx_ensure_rx(void)
{
    if (is_uart()) e22u_ensure_rx(); else sx126x_ensure_rx();
}

float sx_rssi_inst(void) { return is_uart() ? NAN : sx126x_rssi_inst(); }

uint32_t sx_airtime_us(const lora_cfg_t *cfg, uint8_t len)
{
    if (is_uart()) { e22u_cfg_t u = to_uart(cfg); return e22u_airtime_us(&u, len); }
    sx126x_cfg_t c = to_spi(cfg);
    return sx126x_airtime_us(&c, len);
}

esp_err_t sx_cad(bool *detected)
{
    if (is_uart()) { if (detected) *detected = false; return ESP_OK; }
    return sx126x_cad(detected);
}
