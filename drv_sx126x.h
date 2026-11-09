#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "radio_types.h"

typedef struct {
    float    freq_mhz;
    uint8_t  sf;        /* 5..12 */
    float    bw_khz;
    uint8_t  cr;        /* 1..4 = 4/5..4/8 */
    int8_t   power;     /* -9..22 dBm (chip) */
    uint16_t preamble;
} sx126x_cfg_t;

/* Khac biet giua cac module dung SX1262 */
typedef struct {
    int  tcxo_mv;       /* DIO3 cap nguon TCXO (mV), 0 = thach anh */
    bool dio2_rfsw;     /* DIO2 dieu khien cong tac RF */
    int  pin_rxen;      /* GPIO RXEN, -1 = khong co */
    int  pin_txen;      /* GPIO TXEN, -1 = khong co */
    int  pin_rfen;      /* GPIO RF_EN (bat FEM), -1 = khong co */
} sx126x_profile_t;

/* DIO1 is not used: IRQ status is polled over SPI. ESP_ERR_INVALID_RESPONSE indicates calibration/XOSC/PLL errors. */
esp_err_t         sx126x_init(const sx126x_cfg_t *cfg, const sx126x_profile_t *prof);
esp_err_t         sx126x_configure(const sx126x_cfg_t *cfg);
esp_err_t         sx126x_transmit(const uint8_t *data, uint8_t len, uint32_t timeout_ms);
radio_rx_status_t sx126x_receive(uint8_t *buf, uint8_t maxlen, uint32_t timeout_ms, radio_rx_info_t *info);
void              sx126x_ensure_rx(void);
float             sx126x_rssi_inst(void);
esp_err_t         sx126x_cad(bool *detected);
uint32_t          sx126x_airtime_us(const sx126x_cfg_t *cfg, uint8_t payload_len);
