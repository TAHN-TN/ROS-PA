#include "common.h"

uint32_t g_crc_err, g_hdr_err;
volatile bool g_abort;
uint8_t  g_plen = 32;

uint32_t air_ms_cfg(const lora_cfg_t *c) { return sx_airtime_us(c, g_plen) / 1000; }

void sleep_until(int64_t t_us)
{
    if (g_abort) return;
    int64_t ms = (t_us - now_us()) / 1000;
    while (ms > 0 && !g_abort) {                       /* ngu theo tung doan de thoat nhanh khi bi huy */
        int64_t step = ms > 200 ? 200 : ms;
        vTaskDelay(pdMS_TO_TICKS(step));
        ms = (t_us - now_us()) / 1000;
    }
}

/* Nguong SNR giai dieu che LoRa ly thuyet, BW bat ky (Semtech) */
float demod_snr_limit(uint8_t sf)
{
    switch (sf) {
    case 5: return -2.5f;  case 6: return -5.0f;  case 7: return -7.5f;  case 8: return -10.0f;
    case 9: return -12.5f; case 10: return -15.0f; case 11: return -17.5f; default: return -20.0f;
    }
}

void print_rf(const lora_cfg_t *c)
{
    printf("RF: %.3f MHz | SF%u | BW %.1f kHz | CR 4/%u | TX output %d dBm | payload %u B | airtime %u ms\n",
           c->freq_mhz, c->sf, c->bw_khz, c->cr + 4, sx_output_power_dbm(c->power), g_plen,
           (unsigned)air_ms_cfg(c));
}

bool pkt_send(uint8_t type, uint8_t src, uint8_t dst, uint32_t seq, int16_t rssi_x10, int16_t snr_x10, uint32_t aux)
{
    uint8_t buf[MAX_PLEN];
    memset(buf, 0x55, sizeof buf);
    hdr_t h = {.magic = PKT_MAGIC, .type = type, .src = src, .dst = dst, .seq = seq,
               .rssi_x10 = rssi_x10, .snr_x10 = snr_x10, .aux = aux};
    memcpy(buf, &h, sizeof h);
    esp_err_t err = sx_transmit(buf, g_plen, 10000);
    if (err != ESP_OK) {
        printf("Radio TX loi: %s\n", esp_err_to_name(err));
        return false;
    }
    return true;
}

bool pkt_recv(hdr_t *h, rx_info_t *ri, uint32_t timeout_ms)
{
    uint8_t buf[255];
    sx_rx_status_t s = sx_receive(buf, sizeof buf, timeout_ms, ri);
    if (s == SX_RX_CRC_ERR) { g_crc_err++; return false; }
    if (s == SX_RX_HDR_ERR) { g_hdr_err++; return false; }
    if (s != SX_RX_OK || ri->len < HDR_LEN) return false;
    memcpy(h, buf, sizeof *h);
    return h->magic == PKT_MAGIC;
}

/* Cho 1 goi dung type/src/dst/seq trong timeout_ms (src = ANY_ID, seq = ANY_SEQ: bo qua dieu kien) */
bool wait_pkt(uint8_t type, uint8_t src, uint8_t dst, uint32_t seq, uint32_t timeout_ms, hdr_t *h, rx_info_t *ri)
{
    int64_t dl = now_us() + (int64_t)timeout_ms * 1000;
    while (1) {
        if (g_abort) return false;
        int64_t rem = (dl - now_us()) / 1000;
        if (rem <= 0) return false;
        hdr_t t; rx_info_t r;
        if (!pkt_recv(&t, &r, (uint32_t)rem)) continue;
        if (t.type != type || t.dst != dst) continue;
        if (src != ANY_ID && t.src != src) continue;
        if (seq != ANY_SEQ && t.seq != seq) continue;
        *h = t; *ri = r;
        return true;
    }
}
