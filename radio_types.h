#pragma once
#include <stdint.h>

typedef struct { uint8_t len; float rssi, snr, sig_rssi; } radio_rx_info_t;   /* snr = NAN neu module khong co */
typedef enum { RADIO_RX_OK, RADIO_RX_TIMEOUT, RADIO_RX_CRC_ERR, RADIO_RX_ERR } radio_rx_status_t;
