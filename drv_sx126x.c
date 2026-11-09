/*
 * Driver SX1262 toi gian cho E22-900M30S (LoRa, explicit header, CRC bat).
 * Tham khao: Semtech SX1261/2 datasheet (muc 13: Commands Interface).
 */
#include "drv_sx126x.h"
#include "board.h"

#include <math.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "sx1262";

#define PIN_SCLK PIN_SCK

/* ESP32: 18/19/23/5 la chan IOMUX truc tiep cua VSPI (SPI3_HOST); HSPI (SPI2_HOST) phai qua GPIO matrix. */
#ifdef CONFIG_IDF_TARGET_ESP32
#define SX_SPI_HOST SPI3_HOST
#else
#define SX_SPI_HOST SPI2_HOST
#endif

#define IRQ_TX_DONE 0x0001
#define IRQ_RX_DONE 0x0002
#define IRQ_HDR_ERR 0x0020
#define IRQ_CRC_ERR 0x0040
#define IRQ_CAD_DONE 0x0080
#define IRQ_CAD_DET  0x0100
#define IRQ_TIMEOUT 0x0200
#define IRQ_USED    (IRQ_TX_DONE | IRQ_RX_DONE | IRQ_HDR_ERR | IRQ_CRC_ERR | IRQ_CAD_DONE | IRQ_CAD_DET | IRQ_TIMEOUT)

#define CHK(x) do { esp_err_t _e = (x); if (_e != ESP_OK) return _e; } while (0)

static spi_device_handle_t s_spi;
static sx126x_cfg_t        s_cfg;
static sx126x_profile_t    s_prof = {.tcxo_mv = 1800, .dio2_rfsw = false, .pin_rxen = -1, .pin_txen = -1, .pin_rfen = -1};
static bool                s_rx_active;
static bool                s_inited;

typedef enum { SW_OFF, SW_RX, SW_TX } sw_t;
static void rf_switch(sw_t m)
{
    if (s_prof.pin_rxen >= 0) gpio_set_level(s_prof.pin_rxen, m == SW_RX);
    if (s_prof.pin_txen >= 0) gpio_set_level(s_prof.pin_txen, m == SW_TX);
}

static bool wait_busy(uint32_t timeout_ms)
{
    int64_t dl = esp_timer_get_time() + (int64_t)timeout_ms * 1000;
    while (gpio_get_level(PIN_BUSY)) {
        if (esp_timer_get_time() > dl) return false;
        esp_rom_delay_us(10);
    }
    return true;
}

static esp_err_t xfer(const uint8_t *tx, uint8_t *rx, size_t n)
{
    if (!s_spi || !tx || n == 0) return ESP_ERR_INVALID_STATE;
    if (!wait_busy(1000)) return ESP_ERR_TIMEOUT;
    spi_transaction_t t = {0};
    t.length    = n * 8;
    t.tx_buffer = tx;
    t.rx_buffer = rx;
    return spi_device_polling_transmit(s_spi, &t);
}

static esp_err_t reset_and_check_status(void)
{
    esp_err_t last_err = ESP_ERR_INVALID_RESPONSE;
    s_rx_active = false;

    for (int attempt = 1; attempt <= 3; attempt++) {
        gpio_set_level(PIN_NRST, 0);
        vTaskDelay(pdMS_TO_TICKS(10));
        gpio_set_level(PIN_NRST, 1);
        vTaskDelay(pdMS_TO_TICKS(50));

        if (!wait_busy(1000)) {
            last_err = ESP_ERR_TIMEOUT;
            ESP_LOGW(TAG, "BUSY stuck high after reset attempt %d (BUSY=%d, NRST=%d)",
                     attempt, gpio_get_level(PIN_BUSY), gpio_get_level(PIN_NRST));
            continue;
        }

        uint8_t tx[2] = {0xC0, 0x00}, rx[2] = {0};
        esp_err_t err = xfer(tx, rx, sizeof(tx));
        if (err != ESP_OK) {
            last_err = err;
            ESP_LOGW(TAG, "GetStatus failed after reset attempt %d: %s", attempt, esp_err_to_name(err));
            continue;
        }

        uint8_t mode = (rx[1] >> 4) & 0x07;
        if (rx[1] != 0x00 && rx[1] != 0xFF && (mode == 2 || mode == 3)) return ESP_OK;
        last_err = ESP_ERR_INVALID_RESPONSE;
        ESP_LOGW(TAG, "Invalid SX126x status 0x%02X after reset attempt %d", rx[1], attempt);
    }
    return last_err;
}

/* Tu kiem tra day NRST/BUSY: khi NRST=0 chip giu BUSY=1. Neu BUSY van = 0 (co pull-down) thi day BUSY bi ho/sai chan,
   wait_busy() se khong bao gio cho -> lenh gui khi chip dang ban bi bo qua. */
static void pin_selftest(void)
{
    gpio_set_level(PIN_NRST, 0);
    vTaskDelay(pdMS_TO_TICKS(5));
    int busy_in_reset = gpio_get_level(PIN_BUSY);
    gpio_set_level(PIN_NRST, 1);
    vTaskDelay(pdMS_TO_TICKS(30));
    ESP_LOGI(TAG, "Pin check: BUSY(IO%d) khi NRST(IO%d)=0 -> %d, sau reset -> %d | RXEN(IO%d)=%d TXEN(IO%d)=%d",
             PIN_BUSY, PIN_NRST, busy_in_reset, gpio_get_level(PIN_BUSY),
             s_prof.pin_rxen, s_prof.pin_rxen >= 0 ? gpio_get_level(s_prof.pin_rxen) : -1,
             s_prof.pin_txen, s_prof.pin_txen >= 0 ? gpio_get_level(s_prof.pin_txen) : -1);
    if (!busy_in_reset)
        ESP_LOGW(TAG, "BUSY khong len cao khi reset -> kiem tra day BUSY (IO%d) toi chan BUSY cua module", PIN_BUSY);
}

static esp_err_t cmd_w(uint8_t op, const uint8_t *d, size_t n)
{
    uint8_t b[16];
    if (n > sizeof(b) - 1 || (n > 0 && !d)) return ESP_ERR_INVALID_ARG;
    b[0] = op;
    if (n) memcpy(b + 1, d, n);
    return xfer(b, NULL, n + 1);
}

// Lenh doc: MOSI = op, NOP...; MISO = RFU, Status, data...
static esp_err_t cmd_r(uint8_t op, uint8_t *out, size_t n)
{
    uint8_t tx[16] = {0}, rx[16] = {0};
    if (!out || n > sizeof(rx) - 2) return ESP_ERR_INVALID_ARG;
    tx[0] = op;
    esp_err_t e = xfer(tx, rx, n + 2);
    if (e == ESP_OK) memcpy(out, rx + 2, n);
    return e;
}

static esp_err_t reg_w(uint16_t addr, uint8_t v)
{
    uint8_t b[4] = {0x0D, (uint8_t)(addr >> 8), (uint8_t)addr, v};
    return xfer(b, NULL, 4);
}

static esp_err_t reg_r(uint16_t addr, uint8_t *value)
{
    uint8_t tx[5] = {0x1D, (uint8_t)(addr >> 8), (uint8_t)addr, 0, 0};
    uint8_t rx[5] = {0};
    if (!value) return ESP_ERR_INVALID_ARG;
    esp_err_t err = xfer(tx, rx, sizeof(tx));
    if (err == ESP_OK) *value = rx[4];
    return err;
}

static esp_err_t standby(void)
{
    uint8_t d = 0;                       // STDBY_RC
    s_rx_active = false;
    return cmd_w(0x80, &d, 1);
}

static esp_err_t clear_irq(uint16_t m)
{
    uint8_t d[2] = {(uint8_t)(m >> 8), (uint8_t)m};
    return cmd_w(0x02, d, 2);
}

static esp_err_t get_irq(uint16_t *irq)
{
    uint8_t d[2] = {0};
    if (!irq) return ESP_ERR_INVALID_ARG;
    esp_err_t err = cmd_r(0x12, d, sizeof(d));
    if (err == ESP_OK) *irq = ((uint16_t)d[0] << 8) | d[1];
    return err;
}

static esp_err_t wait_irq(uint16_t mask, uint32_t timeout_ms, uint16_t *out)
{
    int64_t dl = esp_timer_get_time() + (int64_t)timeout_ms * 1000;
    while (1) {
        uint16_t irq = 0;
        esp_err_t err = get_irq(&irq);
        if (err != ESP_OK) return err;
        if (irq & mask) {
            if (out) *out = irq;
            return ESP_OK;
        }
        int64_t rem = (dl - esp_timer_get_time()) / 1000;
        if (rem <= 0) return ESP_ERR_TIMEOUT;
        vTaskDelay(1);
    }
}

static int bw_code(float bw)
{
    static const struct { float k; uint8_t c; } t[] = {
        {7.8f, 0x00}, {10.4f, 0x08}, {15.6f, 0x01}, {20.8f, 0x09}, {31.25f, 0x02},
        {41.7f, 0x0A}, {62.5f, 0x03}, {125.0f, 0x04}, {250.0f, 0x05}, {500.0f, 0x06}};
    for (unsigned i = 0; i < sizeof t / sizeof t[0]; i++)
        if (fabsf(bw - t[i].k) < 0.6f) return t[i].c;   // chap nhan 7/10/15/20/31/41/62 lam tron
    return -1;
}

static esp_err_t set_pkt_params(uint8_t len)
{
    uint8_t d[6] = {(uint8_t)(s_cfg.preamble >> 8), (uint8_t)s_cfg.preamble,
                    0x00 /*explicit hdr*/, len, 0x01 /*CRC on*/, 0x00 /*IQ std*/};
    return cmd_w(0x8C, d, 6);
}

static esp_err_t calib_image(float f)
{
    uint8_t d[2];
    if (f >= 902.f)      { d[0] = 0xE1; d[1] = 0xE9; }
    else if (f >= 863.f) { d[0] = 0xD7; d[1] = 0xDB; }
    else if (f >= 779.f) { d[0] = 0xC1; d[1] = 0xC5; }
    else if (f >= 470.f) { d[0] = 0x75; d[1] = 0x81; }
    else                 { d[0] = 0x6B; d[1] = 0x6F; }
    return cmd_w(0x98, d, 2);
}

static int s_dcdc = 1;   /* 1 = DC-DC, 0 = LDO; init tu dong chon */

static uint8_t tcxo_code(int mv)
{
    switch (mv) {
    case 1600: return 0; case 1700: return 1; case 1800: return 2; case 2200: return 3;
    case 2400: return 4; case 2700: return 5; case 3000: return 6; case 3300: return 7;
    default:   return 2;
    }
}

esp_err_t sx126x_configure(const sx126x_cfg_t *c)
{
    if (!c || !isfinite(c->freq_mhz) || c->freq_mhz < 150.0f || c->freq_mhz > 960.0f ||
        !isfinite(c->bw_khz) || c->power < -9 || c->power > 22 || c->preamble == 0 ||
        c->sf < 5 || c->sf > 12 || c->cr < 1 || c->cr > 4) {
        return ESP_ERR_INVALID_ARG;
    }
    int bwc = bw_code(c->bw_khz);
    if (bwc < 0) return ESP_ERR_INVALID_ARG;
    s_cfg = *c;

    CHK(standby());
    { uint8_t d = 0x01; CHK(cmd_w(0x8A, &d, 1)); }               // packet type = LoRa
    CHK(calib_image(c->freq_mhz));

    uint32_t frf = (uint32_t)(((uint64_t)(c->freq_mhz * 1e6) << 25) / 32000000ULL);
    { uint8_t d[4] = {frf >> 24, frf >> 16, frf >> 8, frf}; CHK(cmd_w(0x86, d, 4)); }

    { uint8_t d[4] = {0x04, 0x07, 0x00, 0x01}; CHK(cmd_w(0x95, d, 4)); }   // PA SX1262 +22 dBm
    { int8_t p = c->power > 22 ? 22 : c->power;
      uint8_t d[2] = {(uint8_t)p, 0x04}; CHK(cmd_w(0x8E, d, 2)); }         // power, ramp 200us
    { uint8_t d[2] = {0, 0}; CHK(cmd_w(0x8F, d, 2)); }                     // buffer base

    float tsym_ms = (float)(1u << c->sf) / c->bw_khz;
    uint8_t ldro  = tsym_ms > 16.38f ? 1 : 0;
    { uint8_t d[4] = {c->sf, (uint8_t)bwc, c->cr, ldro}; CHK(cmd_w(0x8B, d, 4)); }
    CHK(set_pkt_params(255));

    CHK(reg_w(0x0740, 0x14));                                    // sync word private 0x1424
    CHK(reg_w(0x0741, 0x24));
    {
        uint8_t reg_value = 0;
        CHK(reg_r(0x0736, &reg_value));
        CHK(reg_w(0x0736, reg_value | 0x04));                    // LoRa IQ errata setting
        CHK(reg_w(0x08E7, 0x38));                                // PA OCP setting
        CHK(reg_r(0x08D8, &reg_value));
        CHK(reg_w(0x08D8, reg_value | 0x1E));                    // PA clamping errata setting
    }

    /* IRQ status van duoc doc bang GetIrqStatus (polling); khong route IRQ ra DIO1/DIO2/DIO3. */
    { uint8_t d[8] = {
          IRQ_USED >> 8, IRQ_USED & 0xFF,  /* IRQ mask to enable inside chip */
          0x00, 0x00,                      /* DIO1 mask: disabled */
          0x00, 0x00,                      /* DIO2 mask: disabled */
          0x00, 0x00                       /* DIO3 mask: disabled */
      };
      CHK(cmd_w(0x08, d, 8)); }
    CHK(clear_irq(0xFFFF));
    rf_switch(SW_OFF);

    uint8_t tx[2] = {0xC0, 0}, rx[2] = {0};              // GetStatus
    CHK(xfer(tx, rx, 2));
    uint8_t mode = (rx[1] >> 4) & 7;
    if (rx[1] == 0x00 || rx[1] == 0xFF || (mode != 2 && mode != 3)) {
        ESP_LOGE(TAG, "Chip khong phan hoi SPI hop le (status=0x%02X) - kiem tra module/SPI/BUSY/nguon", rx[1]);
        return ESP_FAIL;
    }
    return ESP_OK;
}

esp_err_t sx126x_init(const sx126x_cfg_t *cfg, const sx126x_profile_t *prof)
{
    if (!cfg || !prof) return ESP_ERR_INVALID_ARG;
    if (prof->tcxo_mv != 0 && prof->tcxo_mv != 1600 && prof->tcxo_mv != 1700 &&
        prof->tcxo_mv != 1800 && prof->tcxo_mv != 2200 && prof->tcxo_mv != 2400 &&
        prof->tcxo_mv != 2700 && prof->tcxo_mv != 3000 && prof->tcxo_mv != 3300) {
        ESP_LOGE(TAG, "TCXO voltage unsupported: %d mV", prof->tcxo_mv);
        return ESP_ERR_INVALID_ARG;
    }
    s_prof = *prof;

    /* Reclaim the control pins on every init/retry; BUSY must not be left
       configured as another peripheral (for example UART TX on GPIO17). */
    uint64_t output_mask = 1ULL << PIN_NRST;
    if (prof->pin_rxen >= 0) output_mask |= 1ULL << prof->pin_rxen;
    if (prof->pin_txen >= 0) output_mask |= 1ULL << prof->pin_txen;
    if (prof->pin_rfen >= 0) output_mask |= 1ULL << prof->pin_rfen;
    gpio_config_t output_cfg = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = output_mask,
    };
    esp_err_t gpio_err = gpio_config(&output_cfg);
    if (gpio_err != ESP_OK) return gpio_err;
    gpio_set_level(PIN_NRST, 1);
    if (prof->pin_rfen >= 0) gpio_set_level(prof->pin_rfen, 1);

    gpio_config_t busy_cfg = {
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = 1ULL << PIN_BUSY,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,   /* datasheet 8.4: BUSY do chip keo (PU 20k khi reset/sleep); pull-down cua ESP lam BUSY doc sai */
        .pull_up_en = GPIO_PULLUP_DISABLE,
    };
    gpio_err = gpio_config(&busy_cfg);
    if (gpio_err != ESP_OK) return gpio_err;

    if (!s_inited) {
        rf_switch(SW_OFF);

        spi_bus_config_t bus = {.mosi_io_num = PIN_MOSI, .miso_io_num = PIN_MISO,
                                .sclk_io_num = PIN_SCLK, .quadwp_io_num = -1, .quadhd_io_num = -1,
                                .max_transfer_sz = 300};
        esp_err_t spi_err = spi_bus_initialize(SX_SPI_HOST, &bus, SPI_DMA_CH_AUTO);
        if (spi_err != ESP_OK) {
            ESP_LOGE(TAG, "SPI bus init failed: %s", esp_err_to_name(spi_err));
            return spi_err;
        }
        spi_device_interface_config_t dev = {.clock_speed_hz = 4 * 1000 * 1000, .mode = 0,
                                             .spics_io_num = PIN_NSS, .queue_size = 1};
        spi_err = spi_bus_add_device(SX_SPI_HOST, &dev, &s_spi);
        if (spi_err != ESP_OK) {
            spi_bus_free(SX_SPI_HOST);
            ESP_LOGE(TAG, "SPI device add failed: %s", esp_err_to_name(spi_err));
            return spi_err;
        }
        s_inited = true;
    }

    pin_selftest();
    vTaskDelay(pdMS_TO_TICKS(100)); /* allow the radio supply and oscillator to settle */
    esp_err_t reset_err = reset_and_check_status();
    if (reset_err != ESP_OK) {
        ESP_LOGE(TAG, "SX126x did not respond after 3 reset attempts; check VCC/GND, NRST, BUSY and SPI wiring");
        return reset_err;
    }

    /* Tu dong tim cau hinh nguon xung/regulator ma module that su chay duoc.
       0x0020 (XOSC_START_ERR) = chip khong thay xung 32 MHz: sai che do TCXO/thach anh, hoac DC-DC khong co cuon cam. */
    typedef struct { int tcxo_mv; int dcdc; } clk_try_t;
    const clk_try_t tries[] = {
        {s_prof.tcxo_mv, s_dcdc}, {s_prof.tcxo_mv, !s_dcdc},
        {1800, 1}, {1800, 0}, {0, 1}, {0, 0}, {1700, 0}, {1600, 0}, {3300, 0}, {2700, 0}, {3000, 0}, {2400, 0}, {2200, 0},
    };
    bool clk_ok = false;
    for (size_t i = 0; i < sizeof tries / sizeof tries[0] && !clk_ok; i++) {
        if (i > 0) {
            esp_err_t r = reset_and_check_status();           /* reset cung truoc moi lan thu lai */
            if (r != ESP_OK) return r;
        }
        CHK(standby());
        { uint8_t d = tries[i].dcdc ? 0x01 : 0x00; CHK(cmd_w(0x96, &d, 1)); }      /* regulator: 1 = DC-DC, 0 = LDO */
        if (tries[i].tcxo_mv > 0) {                                                   /* DIO3 cap nguon TCXO */
            uint8_t d[4] = {tcxo_code(tries[i].tcxo_mv), 0x00, 0x04, 0x00};           /* timeout 0x000400 = 16 ms */
            CHK(cmd_w(0x97, d, 4));
        }
        { uint8_t d = s_prof.dio2_rfsw ? 0x01 : 0x00; CHK(cmd_w(0x9D, &d, 1)); }   /* DIO2 dieu khien RF switch (neu module can) */
        { uint8_t z[2] = {0, 0}; CHK(cmd_w(0x07, z, 2)); }                         /* xoa loi tu luc reset truoc khi calibrate */
        { uint8_t d = 0x7F; CHK(cmd_w(0x89, &d, 1)); }                             /* calibrate tat ca */
        vTaskDelay(pdMS_TO_TICKS(10));
        if (!wait_busy(1000)) return ESP_ERR_TIMEOUT;
        uint8_t e0[2] = {0};
        CHK(cmd_r(0x17, e0, 2));
        uint16_t err0 = ((uint16_t)e0[0] << 8) | e0[1];
        ESP_LOGI(TAG, "Thu #%u: tcxo=%d mV, %s -> GetDeviceErrors 0x%04X", (unsigned)(i + 1), tries[i].tcxo_mv,
                 tries[i].dcdc ? "DC-DC" : "LDO", err0);
        { uint8_t z[2] = {0, 0}; CHK(cmd_w(0x07, z, 2)); }
        if (!(err0 & 0x0064)) {
            clk_ok = true;
            s_prof.tcxo_mv = tries[i].tcxo_mv;
            s_dcdc = tries[i].dcdc;
            ESP_LOGI(TAG, "Cau hinh chay duoc: tcxo=%d mV, regulator=%s  (dat vao PROF_M30S neu muon co dinh)",
                     s_prof.tcxo_mv, s_dcdc ? "DC-DC" : "LDO");
        }
    }
    if (!clk_ok) {
        ESP_LOGE(TAG, "Moi cau hinh xung/regulator deu loi XOSC -> kiem tra nguon 3V3/5V cua module (sut ap?), thach anh/TCXO, day GND");
        return ESP_ERR_INVALID_RESPONSE;
    }

    CHK(sx126x_configure(cfg));

    uint8_t e[2] = {0};
    CHK(cmd_r(0x17, e, 2));
    uint16_t errs = ((uint16_t)e[0] << 8) | e[1];
    if (errs) ESP_LOGW(TAG, "Device errors = 0x%04X", errs);
    else      ESP_LOGI(TAG, "SX1262 OK");
    return ESP_OK;
}

/* Phat hien chip bi reset ngoai y muon (rut/cam day NRST, sut ap): sau reset chip quay ve mac dinh (packet type GFSK,
   mat tan so/SF/BW/TCXO) trong khi driver van nghi la da cau hinh -> goi di khong toi node. Kiem tra packet type = LoRa (0x01). */
static void ensure_alive(void)
{
    uint8_t pt = 0xFF;
    if (cmd_r(0x11, &pt, 1) == ESP_OK && pt == 0x01) return;
    ESP_LOGW(TAG, "Chip mat cau hinh (packet type=0x%02X) - co the NRST/nguon bi nhieu -> khoi tao lai", pt);
    sx126x_cfg_t c = s_cfg;
    sx126x_profile_t p = s_prof;
    esp_err_t e = sx126x_init(&c, &p);
    if (e != ESP_OK) ESP_LOGE(TAG, "Khoi tao lai that bai: %s", esp_err_to_name(e));
}

static esp_err_t tx_once(const uint8_t *data, uint8_t len, uint32_t timeout_ms)
{
    if (!data || len == 0 || timeout_ms == 0) return ESP_ERR_INVALID_ARG;
    ensure_alive();

    const char *stage = "standby";
    esp_err_t err = standby();
    if (err != ESP_OK) return err;

    {   /* STDBY_XOSC: cho TCXO/PLL on dinh truoc khi bat PA */
        uint8_t d = 0x01;
        err = cmd_w(0x80, &d, 1);
        if (err != ESP_OK) return err;
        vTaskDelay(pdMS_TO_TICKS(3));
    }
    rf_switch(SW_TX);
    esp_rom_delay_us(500);                       /* cho FEM / cong tac RF chuyen sang TX */
    stage = "packet parameters";
    err = set_pkt_params(len);
    if (err != ESP_OK) goto cleanup;

    uint8_t b[258];
    b[0] = 0x0E;
    b[1] = 0x00;
    memcpy(b + 2, data, len);
    stage = "write TX buffer";
    err = xfer(b, NULL, (size_t)len + 2);
    if (err != ESP_OK) goto cleanup;

    stage = "clear IRQ";
    err = clear_irq(0xFFFF);
    if (err != ESP_OK) goto cleanup;

    {
        uint64_t timeout_ticks = (uint64_t)timeout_ms * 64ULL;
        if (timeout_ticks > 0xFFFFFFULL) timeout_ticks = 0xFFFFFFULL;
        uint8_t tx_timeout[3] = {
            (uint8_t)(timeout_ticks >> 16),
            (uint8_t)(timeout_ticks >> 8),
            (uint8_t)timeout_ticks,
        };
        stage = "SetTx command";
        err = cmd_w(0x83, tx_timeout, sizeof(tx_timeout));
    }
    if (err != ESP_OK) goto cleanup;

    uint16_t irq = 0;
    stage = "TX_DONE IRQ";
    uint32_t irq_wait_ms = timeout_ms <= UINT32_MAX - 100 ? timeout_ms + 100 : UINT32_MAX;
    err = wait_irq(IRQ_TX_DONE | IRQ_TIMEOUT, irq_wait_ms, &irq);
    if (err == ESP_OK && (irq & IRQ_TIMEOUT)) {
        ESP_LOGE(TAG, "Radio reported hardware TX timeout (IRQ=0x%04X, timeout_ms=%u)",
                 irq, (unsigned)timeout_ms);
        err = ESP_ERR_TIMEOUT;
    }
    if (err == ESP_OK && !(irq & IRQ_TX_DONE)) err = ESP_ERR_TIMEOUT;
    if (err != ESP_OK && strcmp(stage, "TX_DONE IRQ") == 0) {
        uint16_t current_irq = 0;
        uint8_t status_tx[2] = {0xC0, 0};
        uint8_t status_rx[2] = {0};
        esp_err_t irq_err = get_irq(&current_irq);
        esp_err_t status_err = xfer(status_tx, status_rx, sizeof(status_tx));
        ESP_LOGE(TAG,
                 "TX timeout diagnosis: wait=%s IRQ=0x%04X (%s), status=0x%02X (%s), BUSY=%d",
                 esp_err_to_name(err), current_irq, esp_err_to_name(irq_err),
                 status_rx[1], esp_err_to_name(status_err), gpio_get_level(PIN_BUSY));
        uint8_t de[2] = {0};
        if (cmd_r(0x17, de, 2) == ESP_OK)
            ESP_LOGE(TAG, "DeviceErrors=0x%02X%02X (0x0040 PLL_LOCK, 0x0020 XOSC_START, 0x0004 PLL_CALIB) "
                          "-> thuong do sut ap nguon khi phat cong suat lon", de[0], de[1]);
    }

cleanup:
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "TX failed at %s: %s", stage, esp_err_to_name(err));
    }
    rf_switch(SW_OFF);
    (void)clear_irq(0xFFFF);
    (void)standby();
    return err;
}

esp_err_t sx126x_transmit(const uint8_t *data, uint8_t len, uint32_t timeout_ms)
{
    esp_err_t err = tx_once(data, len, timeout_ms);
    if (err == ESP_ERR_TIMEOUT) {
        /* Chip ket o TX: reset cung + nap lai toan bo cau hinh, roi thu lai 1 lan */
        ESP_LOGW(TAG, "TX timeout -> reset module va thu lai");
        sx126x_cfg_t c = s_cfg;
        sx126x_profile_t p = s_prof;
        if (sx126x_init(&c, &p) == ESP_OK) err = tx_once(data, len, timeout_ms);
    }
    return err;
}

static esp_err_t start_rx(void)
{
    esp_err_t err = standby();
    if (err != ESP_OK) return err;
    err = set_pkt_params(255);
    if (err != ESP_OK) return err;

    rf_switch(SW_RX);
    err = clear_irq(0xFFFF);
    if (err == ESP_OK) {
        uint8_t timeout[3] = {0xFF, 0xFF, 0xFF};
        err = cmd_w(0x82, timeout, sizeof(timeout));
    }
    if (err == ESP_OK) s_rx_active = true;
    else {
        rf_switch(SW_OFF);
        (void)standby();
    }
    return err;
}

void sx126x_ensure_rx(void)
{
    if (!s_rx_active) (void)start_rx();
}

radio_rx_status_t sx126x_receive(uint8_t *buf, uint8_t maxlen, uint32_t timeout_ms, radio_rx_info_t *info)
{
    if (!buf || maxlen == 0 || timeout_ms == 0) return RADIO_RX_ERR;
    ensure_alive();
    if (!s_rx_active && start_rx() != ESP_OK) return RADIO_RX_ERR;
    uint16_t irq = 0;
    esp_err_t wait_err = wait_irq(IRQ_RX_DONE | IRQ_CRC_ERR | IRQ_HDR_ERR, timeout_ms, &irq);
    if (wait_err == ESP_ERR_TIMEOUT) return RADIO_RX_TIMEOUT;
    if (wait_err != ESP_OK) return RADIO_RX_ERR;
    if (clear_irq(0xFFFF) != ESP_OK) return RADIO_RX_ERR;
    if (irq & IRQ_CRC_ERR) return RADIO_RX_CRC_ERR;     // RX_DONE cung duoc set -> kiem CRC truoc
    if (irq & IRQ_HDR_ERR) return RADIO_RX_ERR;

    uint8_t bs[2] = {0};
    if (cmd_r(0x13, bs, 2) != ESP_OK) return RADIO_RX_ERR;   // payloadLen, startPtr
    uint8_t len = bs[0] > maxlen ? maxlen : bs[0];

    uint8_t ps[3] = {0};
    if (cmd_r(0x14, ps, 3) != ESP_OK) return RADIO_RX_ERR; // PacketStatus

    uint8_t tx[258] = {0}, rx[258] = {0};
    tx[0] = 0x1E; tx[1] = bs[1];
    if (xfer(tx, rx, len + 3) != ESP_OK) return RADIO_RX_ERR;
    memcpy(buf, rx + 3, len);

    if (info) {
        info->len      = len;
        info->rssi     = -(float)ps[0] / 2.0f;
        info->snr      = (float)(int8_t)ps[1] / 4.0f;
        info->sig_rssi = -(float)ps[2] / 2.0f;
    }
    return RADIO_RX_OK;
}

float sx126x_rssi_inst(void)
{
    uint8_t v = 0;
    if (cmd_r(0x15, &v, 1) != ESP_OK) return NAN;
    return -(float)v / 2.0f;
}

uint32_t sx126x_airtime_us(const sx126x_cfg_t *c, uint8_t pl)
{
    double tsym = (double)(1u << c->sf) / (c->bw_khz * 1000.0);
    int de = tsym > 0.01638 ? 1 : 0;
    double num = 8.0 * pl - 4.0 * c->sf + 28 + 16;       // CRC on, header explicit
    double den = 4.0 * (c->sf - 2 * de);
    double n = ceil(num / den);
    if (n < 0) n = 0;
    n = n * (c->cr + 4) + 8;
    return (uint32_t)(((c->preamble + 4.25) * tsym + n * tsym) * 1e6);
}

/*
 * CAD (Channel Activity Detection) - dung cho Listen-Before-Talk.
 * Tham so theo khuyen nghi Semtech AN1200.48 (125 kHz) - co the can tinh chinh thuc te.
 */
esp_err_t sx126x_cad(bool *detected)
{
    if (!detected) return ESP_ERR_INVALID_ARG;
    *detected = false;

    esp_err_t err = standby();
    if (err != ESP_OK) return err;

    uint8_t sym  = s_cfg.sf >= 10 ? 0x02 : 0x01;
    uint8_t peak = s_cfg.sf <= 8 ? 22 : (s_cfg.sf == 9 ? 23 : (s_cfg.sf == 10 ? 24 : (s_cfg.sf == 11 ? 25 : 26)));
    uint8_t params[7] = {sym, peak, 10, 0x00, 0, 0, 0};
    err = cmd_w(0x88, params, sizeof(params));
    if (err != ESP_OK) goto cleanup;

    rf_switch(SW_RX);
    err = clear_irq(0xFFFF);
    if (err != ESP_OK) goto cleanup;
    err = cmd_w(0xC5, NULL, 0);
    if (err != ESP_OK) goto cleanup;

    {
        uint16_t irq = 0;
        err = wait_irq(IRQ_CAD_DONE, 500, &irq);
        if (err == ESP_OK) *detected = (irq & IRQ_CAD_DET) != 0;
    }

cleanup:
    rf_switch(SW_OFF);
    (void)clear_irq(0xFFFF);
    (void)standby();
    return err;
}
