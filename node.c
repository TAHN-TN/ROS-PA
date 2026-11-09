/*
 * NODE: node <id> (tra loi POLL va chuyen tiep), node_up (EXP-04), n_relay (EXP-05 phia nguon)
 */
#include "common.h"
#include "esp_random.h"

#define LBT_MAX_TRY   6
#define RELAY_OK_TARGET 95.0

void rf_invalidate(void);

/* ------------------ node <id>: tra loi POLL + node dich cua relay ------------------ */
static int cmd_node(int argc, char **argv)
{
    uint8_t id = argc > 1 ? atoi(argv[1]) : 1;
    if (id < 1 || id > 2) { printf("Dung: node <1|2>\n"); return 1; }
    print_rf(&g_cfg);
    printf("[NODE %u] Cho POLL / RLY_FWD (reset board de thoat)...\n", id);
    uint32_t polls = 0, fwds = 0;
    while (1) {
        if (g_abort) { printf("[HUY] node dung\n"); break; }
        hdr_t h; rx_info_t ri;
        if (!pkt_recv(&h, &ri, 1000) || h.dst != id) continue;
        int64_t t0 = now_us();
        if (h.type == PT_POLL) {
            vTaskDelay(pdMS_TO_TICKS(TURNAROUND_MS));
            /* tra loi kem RSSI/SNR node do duoc cua goi POLL, aux = thoi gian xu ly (us) */
            if (pkt_send(PT_DATA, id, ID_GW, h.seq, x10(ri.rssi), x10(ri.snr), (uint32_t)(now_us() - t0))) {
                polls++;
                printf("POLL #%u: RSSI %.0f dBm SNR %.1f dB -> DATA TX OK (tong %u)\n",
                       (unsigned)h.seq, ri.rssi, ri.snr, (unsigned)polls);
            } else {
                printf("POLL #%u: nhan duoc, nhung DATA TX LOI\n", (unsigned)h.seq);
            }
        } else if (h.type == PT_RLY_FWD) {
            vTaskDelay(pdMS_TO_TICKS(TURNAROUND_MS));
            pkt_send(PT_RLY_REPLY, id, h.src, h.seq, x10(ri.rssi), x10(ri.snr), 0);   /* tra loi ve node nguon (qua GW) */
            fwds++;
            printf("RLY_FWD #%u tu Node %u -> REPLY (tong %u)\n", (unsigned)h.seq, h.src, (unsigned)fwds);
        }
    }
    return 0;
}

/* ------------------ node_up: uplink tu do, ALOHA hoac CAD/LBT (EXP-04) ------------------ */
static uint32_t cad_busy, cad_giveup;
static uint64_t bo_total_ms;

static void lbt(void)
{
    uint32_t base = air_ms();                       /* cua so backoff bat dau bang 1 airtime */
    for (int a = 0; a < LBT_MAX_TRY; a++) {
        bool det = false;
        if (sx_cad(&det) != ESP_OK || !det) return;  /* kenh ranh -> phat */
        cad_busy++;
        uint32_t win = base << (a < 3 ? a : 3);      /* backoff mu, toi da 8 x airtime */
        uint32_t bo  = 20 + esp_random() % (win + 1);
        bo_total_ms += bo;
        vTaskDelay(pdMS_TO_TICKS(bo));
    }
    cad_giveup++;                                    /* het so lan thu -> van phat de khong bi treo */
}

static int cmd_node_up(int argc, char **argv)
{
    uint8_t  id     = argc > 1 ? atoi(argv[1]) : 1;
    uint32_t period = argc > 2 ? atoi(argv[2]) : 2000;
    uint32_t count  = argc > 3 ? atoi(argv[3]) : 300;
    int      cad    = argc > 4 ? atoi(argv[4]) : 0;
    uint32_t jitter = argc > 5 ? atoi(argv[5]) : 200;
    if (id < 1 || id > 2 || period < 100 || count < 1 || count > 1000) { printf("Tham so sai\n"); return 1; }

    print_rf(&g_cfg);
    double air = air_ms();
    printf("[NODE %u UP] %u goi, chu ky %u ms (+ngau nhien 0..%u ms), %s\n", id, (unsigned)count, (unsigned)period,
           (unsigned)jitter, cad ? "CAD/LBT + backoff" : "ALOHA");
    printf("Tai kenh moi node: %.0f %% (2 node: %.0f %%)%s\n", 100.0 * air / period, 200.0 * air / period,
           (2.0 * air / period > 0.3) ? "  <-- CAO: va cham se nhieu du khong co nhieu" : "");
    cad_busy = cad_giveup = 0; bo_total_ms = 0;
    sx_set_lbt(cad != 0);                                                 /* T30D: LBT trong module; module SPI: khong lam gi */

    int64_t t0 = now_us() + (int64_t)(esp_random() % period) * 1000;      /* pha ban dau ngau nhien */
    uint32_t fail = 0;
    for (uint32_t n = 0; n < count; n++) {
        if (g_abort) { printf("[HUY] dung som\n"); break; }
        sleep_until(t0 + (int64_t)n * period * 1000 + (int64_t)(esp_random() % (jitter + 1)) * 1000);
        if (cad) lbt();
        if (!pkt_send(PT_UP, id, ID_GW, n, 0, 0, 0)) fail++;
        if ((n + 1) % 50 == 0 || n + 1 == count) printf("  da gui %u/%u\n", (unsigned)(n + 1), (unsigned)count);
    }
    sx_set_lbt(false);
    printf("[NODE %u UP] xong. Loi TX %u", id, (unsigned)fail);
    if (cad) printf(" | CAD thay kenh ban %u lan, bo cuoc %u lan, backoff tong %u ms", (unsigned)cad_busy, (unsigned)cad_giveup, (unsigned)bo_total_ms);
    printf("\n");
    return 0;
}

/* ------------------ n_relay: Node nguon gui toi Node dich qua GW (EXP-05) ------------------ */
static int cmd_n_relay(int argc, char **argv)
{
    uint8_t  me    = argc > 1 ? atoi(argv[1]) : 1;
    uint8_t  dst   = argc > 2 ? atoi(argv[2]) : 2;
    uint32_t count = argc > 3 ? atoi(argv[3]) : 100;
    uint32_t every = argc > 4 ? atoi(argv[4]) : 5;
    if (me < 1 || me > 2 || dst < 1 || dst > 2 || me == dst || count < 1) { printf("Dung: n_relay [nguon=1] [dich=2] [so_goi=100] [chu_ky_s=5]\n"); return 1; }

    print_rf(&g_cfg);
    uint32_t air = air_ms();
    uint32_t to  = 4 * air + 2500;
    printf("[EXP-05] Node %u -> GW -> Node %u: %u goi, timeout %u ms\n", me, dst, (unsigned)count, (unsigned)to);
    printf("Ly thuyet: 4 chang x %u ms airtime = %u ms (chua tinh turnaround ~%u ms)\n", (unsigned)air, (unsigned)(4 * air), 4 * TURNAROUND_MS);

    stat_t lat = {0};
    uint32_t ok = 0;
    for (uint32_t i = 0; i < count; i++) {
        if (g_abort) { printf("[HUY] dung som\n"); break; }
        int64_t t0 = now_us();
        pkt_send(PT_RLY_DATA, me, dst, i, 0, 0, 0);
        hdr_t h; rx_info_t ri;
        if (wait_pkt(PT_RLY_ACK, dst, me, i, to, &h, &ri)) {
            double l = (now_us() - t0) / 1000.0;
            ok++; st_add(&lat, l);
            printf("#%u OK  End-to-End %.0f ms\n", (unsigned)i, l);
        } else {
            printf("#%u MAT\n", (unsigned)i);
        }
        sleep_until(t0 + (int64_t)every * 1000000);
    }
    double pdr = 100.0 * ok / count;
    double lim = g_cfg.sf <= 8 ? 600 : 1500;
    printf("\n===== EXP-05 Node %u ket qua =====\n", me);
    printf("Chuyen tiep thanh cong %u/%u = %.1f %% (>= %.0f %%: %s)\n", (unsigned)ok, (unsigned)count, pdr, RELAY_OK_TARGET,
           pdr >= RELAY_OK_TARGET ? "PASS" : "FAIL");
    if (ok) printf("End-to-End: tb %.0f ms, min %.0f, max %.0f ms (nguong %.0f ms @ SF%u: %s)\n", st_avg(&lat), lat.min, lat.max, lim,
                   g_cfg.sf, st_avg(&lat) < lim ? "PASS" : "FAIL");
    printf("CSV,EXP05,SF%u,%u,%u,%.1f,%.0f,%.0f,%.0f\n", g_cfg.sf, (unsigned)ok, (unsigned)count, pdr, st_avg(&lat), lat.min, lat.max);
    return 0;
}

void node_register(void)
{
    reg_cmd("node",    "node <1|2>  tra loi POLL / node dich cua relay (EXP-01..03,05,06)", cmd_node);
    reg_cmd("node_up", "node_up <id> [chu_ky_ms=2000] [so_goi=300] [cad 0|1] [jitter_ms=200]  EXP-04", cmd_node_up);
    reg_cmd("n_relay", "n_relay [nguon=1] [dich=2] [so_goi=100] [chu_ky_s=5]  EXP-05", cmd_n_relay);
}
