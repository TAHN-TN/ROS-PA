/*
 * E22-900M30S + ESP32 (ESP-IDF 5.5.x) - Mang 1 Gateway + 2 Node, test ban kinh / duong kinh phu song.
 * Cung 1 firmware cho GW va Node. Dieu khien bang MENU LCD + 4 nut (xem ui.c); log qua Wi-Fi (wifi_log.c).
 * Cac lenh duoi day van la ham xu ly that: menu goi chung qua esp_console_run().
 *
 *  EXP-01 / EXP-02  Ban kinh doc lap (Node 1 / Node 2)
 *       GW  : gw_poll <node 1|2> <khoang_cach_km> [so_goi=200] [timeout_ms]
 *       Node: node <1|2>              (node con lai TAT)
 *       Xem tong hop: gw_points
 *  EXP-03  Duong kinh D = R1 + R2, TDMA/polling tuan tu
 *       GW  : gw_tdma [chu_ky=500] [thoi_gian_s=1800] [R1_km] [R2_km]
 *       Node: node 1  /  node 2
 *  EXP-04  2 node dong thoi gui ve GW
 *       GW  : gw_listen [so_goi_moi_node=300] [cad 0|1] [idle_s=60]
 *       Node: node_up <1|2> [chu_ky_ms=2000] [so_goi=300] [cad 0|1] [jitter_ms=200]
 *  EXP-05  Chuyen tiep Node1 -> GW -> Node2
 *       GW  : gw_relay [idle_s=60]
 *       Node2: node 2        Node1: n_relay [1] [2] [so_goi=100] [chu_ky_s=5]
 *  EXP-06  Bat doi xung LOS/NLOS (cau hinh SF/cong suat rieng tung node)
 *       GW  : gw_nodecfg <id> <sf> <cong_suat_dau_ra_module>  roi  gw_tdma ...
 *       Node: cfg <freq> <sf> <bw> <cong_suat_dau_ra_module>  roi  node <id>
 *  Chung: cfg [freq_MHz sf bw_kHz cong_suat_dau_ra_module [cr]]   plen [16..64]   info
 */
#include "common.h"

#include "driver/gpio.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "sdkconfig.h"
#include "ui.h"

lora_cfg_t g_cfg = {
    .freq_mhz = CONFIG_E22_FREQ_KHZ / 1000.0f,
    .sf       = CONFIG_E22_SF,
    .bw_khz   = (float)CONFIG_E22_BW_KHZ,
    .cr       = CONFIG_E22_CR,
    .power    = CONFIG_E22_POWER_DBM,
    .preamble = 8,
};

static int cmd_cfg(int argc, char **argv)
{
    if (argc != 1 && argc < 5) {
        printf("Dung: cfg <freq_MHz> <sf 5..12> <bw_kHz> <cong_suat_dau_ra_module> [cr 1..4]\n");
        return 1;
    }
    if (argc >= 5) {
        lora_cfg_t c = g_cfg;
        int output_power = atoi(argv[4]);
        c.freq_mhz = atof(argv[1]);
        c.sf       = (uint8_t)atoi(argv[2]);
        c.bw_khz   = atof(argv[3]);
        if (argc >= 6) c.cr = (uint8_t)atoi(argv[5]);
        if (c.freq_mhz < 150.0f || c.freq_mhz > 960.0f || c.sf < 5 || c.sf > 12 ||
            c.cr < 1 || c.cr > 4 || !sx_chip_power_from_output(output_power, &c.power)) {
            printf("Cong suat dau ra khong ho tro cho module da chon\n");
            printf("Tham so ngoai pham vi cho phep\n");
            return 1;
        }
        esp_err_t err = sx_configure(&c);
        if (err != ESP_OK) {
            printf("Cau hinh radio that bai: %s\n", esp_err_to_name(err));
            return 1;
        }
        g_cfg = c;
        rf_invalidate();
    }
    print_rf(&g_cfg);
    return 0;
}

static int cmd_plen(int argc, char **argv)
{
    if (argc >= 2) {
        int n = atoi(argv[1]);
        if (n < HDR_LEN || n > MAX_PLEN) { printf("plen %d..%d\n", HDR_LEN, MAX_PLEN); return 1; }
        g_plen = (uint8_t)n;
    }
    print_rf(&g_cfg);
    return 0;
}

static int cmd_info(int argc, char **argv)
{
    print_rf(&g_cfg);
    double air = air_ms();
    printf("Tai kenh neu 1 node gui moi 2 s: %.0f %%\n", 100.0 * air / 2000.0);
    return 0;
}

void reg_cmd(const char *name, const char *help, esp_console_cmd_func_t f)
{
    const esp_console_cmd_t c = {.command = name, .help = help, .hint = NULL, .func = f};
    ESP_ERROR_CHECK(esp_console_cmd_register(&c));
}

void app_main(void)
{
    esp_err_t e = nvs_flash_init();
    if (e == ESP_ERR_NVS_NO_FREE_PAGES || e == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        e = nvs_flash_init();
    }
    ESP_ERROR_CHECK(e);

    wifi_log_init();                                             /* khoi tao bo dem log; Wi-Fi AP dang tam tat */

    /* Cac lenh console CU giu nguyen; giao dien LCD goi chung qua esp_console_run(). Khong con REPL tren UART. */
    esp_console_config_t cc = ESP_CONSOLE_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_console_init(&cc));
    esp_console_register_help_command();
    reg_cmd("cfg",  "cfg [freq_MHz sf bw_kHz cong_suat_dau_ra_module [cr]]  xem/doi tham so RF", cmd_cfg);
    reg_cmd("plen", "plen [16..64]  chieu dai payload (mac dinh 32)", cmd_plen);
    reg_cmd("info", "hien tham so RF + airtime", cmd_info);
    gateway_register();
    node_register();

    ui_run();                                                  /* menu LCD + nut + den; khong tra ve */
}
