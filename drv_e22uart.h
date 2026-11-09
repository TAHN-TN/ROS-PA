#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "radio_types.h"

typedef struct {
    uint8_t channel;     /* 0..80   f = 850.125 + channel MHz */
    uint8_t air_idx;     /* 0..5    2.4k 4.8k 9.6k 19.2k 38.4k 62.5k */
    uint8_t power_idx;   /* 0..3    toi da, -3, -6, -9 dB */
    bool    lbt;
} e22u_cfg_t;

esp_err_t         e22u_init(const e22u_cfg_t *cfg);
esp_err_t         e22u_configure(const e22u_cfg_t *cfg);
esp_err_t         e22u_set_lbt(bool on);
esp_err_t         e22u_transmit(const uint8_t *data, uint8_t len, uint32_t timeout_ms);
radio_rx_status_t e22u_receive(uint8_t *buf, uint8_t maxlen, uint32_t timeout_ms, radio_rx_info_t *info);
void              e22u_ensure_rx(void);
esp_err_t         e22u_cad(bool *detected);
uint32_t          e22u_airtime_us(const e22u_cfg_t *cfg, uint8_t payload_len);
esp_err_t         e22u_read_config(uint8_t out[12]);
float             e22u_freq_mhz(const e22u_cfg_t *c);
int               e22u_power_dbm(const e22u_cfg_t *c);
const char       *e22u_air_name(uint8_t idx);
bool              e22u_cfg_equal(const e22u_cfg_t *a, const e22u_cfg_t *b);
