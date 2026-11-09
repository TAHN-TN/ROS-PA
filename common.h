#pragma once
/*
 * Dinh nghia chung cho Gateway va Node.
 * Giao thuc: header 16 byte + padding den g_plen byte (mac dinh 32).
 *
 *   POLL      GW -> Node        (EXP-01/02/03/06)  seq = so lan poll
 *   DATA      Node -> GW        tra loi POLL, mang RSSI/SNR node do duoc cua goi POLL
 *   UP        Node -> GW        uplink tu do (EXP-04)
 *   RLY_DATA  Node -> GW        yeu cau chuyen tiep, dst = node dich (EXP-05)
 *   RLY_FWD   GW -> Node dich   GW chuyen tiep
 *   RLY_REPLY Node dich -> GW   phan hoi
 *   RLY_ACK   GW -> Node nguon  GW chuyen tiep phan hoi
 */
#include <inttypes.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "esp_console.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sx1262.h"
#include "wifi_log.h"      /* printf() -> Wi-Fi (xem wifi_log.h) */

#define PKT_MAGIC      0xA5
#define HDR_LEN        16
#define MAX_PLEN       64
#define ID_GW          0
#define ANY_ID         0xFF
#define ANY_SEQ        0xFFFFFFFFu
#define TURNAROUND_MS  30      /* node cho 30 ms truoc khi tra loi de ben kia kip chuyen sang RX */

enum { PT_POLL = 1, PT_DATA, PT_UP, PT_RLY_DATA, PT_RLY_FWD, PT_RLY_REPLY, PT_RLY_ACK };

typedef struct __attribute__((packed)) {
    uint8_t  magic;
    uint8_t  type;
    uint8_t  src;
    uint8_t  dst;
    uint32_t seq;
    int16_t  rssi_x10;   /* RSSI (dBm x10) ben gui do duoc o goi truoc do */
    int16_t  snr_x10;    /* SNR  (dB  x10) tuong tu */
    uint32_t aux;
} hdr_t;
_Static_assert(sizeof(hdr_t) == HDR_LEN, "header phai 16 byte");

extern lora_cfg_t g_cfg;
extern uint8_t    g_plen;
extern uint32_t   g_crc_err, g_hdr_err;

typedef struct { uint32_t n; double sum, min, max; } stat_t;
static inline void   st_add(stat_t *s, double v) { if (isnan(v)) return; if (!s->n || v < s->min) s->min = v; if (!s->n || v > s->max) s->max = v; s->sum += v; s->n++; }
static inline double st_avg(const stat_t *s)     { return s->n ? s->sum / s->n : NAN; }   /* NAN = khong co du lieu */

static inline int64_t now_us(void)    { return esp_timer_get_time(); }
static inline int16_t x10(float v)    { return isnan(v) ? INT16_MIN : (int16_t)lroundf(v * 10.0f); }
static inline double  from_x10(int16_t v) { return v == INT16_MIN ? NAN : v / 10.0; }   /* INT16_MIN = khong co (vd SNR cua T30D) */

extern volatile bool g_abort;   /* giao dien dat true de cac lenh dai tu thoat (thay cho "reset board") */

uint32_t air_ms_cfg(const lora_cfg_t *c);
static inline uint32_t air_ms(void)   { return air_ms_cfg(&g_cfg); }
void     sleep_until(int64_t t_us);
float    demod_snr_limit(uint8_t sf);
void     print_rf(const lora_cfg_t *c);

bool pkt_send(uint8_t type, uint8_t src, uint8_t dst, uint32_t seq, int16_t rssi_x10, int16_t snr_x10, uint32_t aux);
bool pkt_recv(hdr_t *h, rx_info_t *ri, uint32_t timeout_ms);
bool wait_pkt(uint8_t type, uint8_t src, uint8_t dst, uint32_t seq, uint32_t timeout_ms, hdr_t *h, rx_info_t *ri);

void rf_invalidate(void);     /* gateway: bat buoc cau hinh RF lai o lan giao tiep ke tiep */
void reg_cmd(const char *name, const char *help, esp_console_cmd_func_t f);
void gateway_register(void);
void node_register(void);
