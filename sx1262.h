#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

typedef struct {
    float    freq_mhz;
    uint8_t  sf;        // 5..12
    float    bw_khz;    // 7.8 .. 500
    uint8_t  cr;        // 1..4  (4/5 .. 4/8)
    int8_t   power;     // -9..22 dBm (SX1262)
    uint16_t preamble;  // so symbol preamble
} lora_cfg_t;

typedef struct {
    uint8_t len;
    float   rssi;       // RSSI goi (dBm)
    float   snr;        // SNR goi (dB)
    float   sig_rssi;   // RSSI tin hieu LoRa sau despread (dBm)
} rx_info_t;

typedef enum { SX_RX_OK, SX_RX_TIMEOUT, SX_RX_CRC_ERR, SX_RX_HDR_ERR, SX_RX_ERR } sx_rx_status_t;

/*
 * Giao dien radio cho code Broad Test (giu nguyen ten ham/kieu). Ben duoi la 3 loai module:
 *   SX_MOD_T30D     UART (SX1262 trong module): freq -> kenh 850.125+n MHz, sf -> toc do khong khi
 *                   (SF12=2.4k, SF11=4.8k, SF10=9.6k, SF9=19.2k, SF8=38.4k, SF7=62.5k), power -> 4 muc
 *   SX_MOD_M30S     SPI, RXEN/TXEN do ESP32 dieu khien, TCXO 1.8 V
 *   SX_MOD_RA01SHP  SPI, RF_EN + DIO2 dieu khien cong tac RF
 * Voi T30D: snr = NAN (module khong cung cap SNR).
 */
typedef enum { SX_MOD_T30D = 0, SX_MOD_M30S, SX_MOD_RA01SHP, SX_MOD_COUNT } sx_module_t;
void           sx_select_module(sx_module_t m);      // goi truoc sx_init
sx_module_t    sx_get_module(void);
int            sx_output_power_dbm(int8_t chip_power_dbm);
bool           sx_chip_power_from_output(int output_power_dbm, int8_t *chip_power_dbm);
esp_err_t      sx_set_lbt(bool on);                  // LBT cua module (T30D); module SPI: khong lam gi
esp_err_t      sx_init(const lora_cfg_t *cfg);       // reset + khoi tao + cau hinh
esp_err_t      sx_cad(bool *detected);               // CAD: true neu kenh dang ban
esp_err_t      sx_configure(const lora_cfg_t *cfg);  // doi tham so RF luc dang chay
esp_err_t      sx_transmit(const uint8_t *data, uint8_t len, uint32_t timeout_ms);
sx_rx_status_t sx_receive(uint8_t *buf, uint8_t maxlen, uint32_t timeout_ms, rx_info_t *info);
void           sx_ensure_rx(void);                   // dua chip vao RX lien tuc
float          sx_rssi_inst(void);                   // RSSI tuc thoi (phai dang RX)
uint32_t       sx_airtime_us(const lora_cfg_t *cfg, uint8_t payload_len);
