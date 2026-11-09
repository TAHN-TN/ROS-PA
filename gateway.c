/*
 * GATEWAY: gw_poll (EXP-01/02), gw_points, gw_tdma (EXP-03/06), gw_listen (EXP-04),
 *          gw_relay (EXP-05), gw_nodecfg (EXP-06)
 */
#include "common.h"
#include "esp_system.h"

#define POLL_PDR_TARGET   95.0
#define POLL_SNR_TARGET  -15.0
#define TDMA_PDR_TARGET   95.0
#define ASYM_PDR_TARGET   92.0
#define CAD_LOSS_MAX       5.0
#define ALOHA_LOSS_MAX    15.0
#define MAX_GAP_OK        20     /* so goi mat lien tiep toi da coi la "khong bi treo" */
#define RELAY_OK_TARGET   95.0
#define MAXN              1000

/* ------------------ cau hinh RF rieng tung node (EXP-06) ------------------ */
static struct { uint8_t sf; int8_t pwr; } node_rf[3];
static lora_cfg_t applied;
static bool       applied_ok;

static lora_cfg_t cfg_for_node(uint8_t id)
{
    lora_cfg_t c = g_cfg;
    if (id >= 1 && id <= 2 && node_rf[id].sf) { c.sf = node_rf[id].sf; c.power = node_rf[id].pwr; }
    return c;
}

void rf_invalidate(void) { applied_ok = false; }

static bool rf_for_node(uint8_t id)
{
    lora_cfg_t c = cfg_for_node(id);
    if (applied_ok && c.sf == applied.sf && c.power == applied.power && c.freq_mhz == applied.freq_mhz &&
        c.bw_khz == applied.bw_khz && c.cr == applied.cr && c.preamble == applied.preamble) return true;
    esp_err_t err = sx_configure(&c);
    if (err == ESP_OK) {
        applied = c;
        applied_ok = true;
        return true;
    }
    applied_ok = false;
    printf("Loi cau hinh RF cho node %u: %s\n", id, esp_err_to_name(err));
    return false;
}

static uint32_t node_air_ms(uint8_t id) { lora_cfg_t c = cfg_for_node(id); return air_ms_cfg(&c); }

static int cmd_gw_nodecfg(int argc, char **argv)
{
    if (argc < 4) { printf("Dung: gw_nodecfg <id 1|2> <sf> <cong_suat_dau_ra_module>   (sf=0: dung cau hinh chung)\n"); return 1; }
    int id = atoi(argv[1]);
    if (id < 1 || id > 2) return 1;
    int sf = atoi(argv[2]);
    int8_t chip_power;
    int output_power = atoi(argv[3]);
    if (!((sf == 0) || (sf >= 5 && sf <= 12)) ||
        !sx_chip_power_from_output(output_power, &chip_power)) {
        printf("SF phai la 0 hoac 5..12; cong suat dau ra khong ho tro cho module\n");
        return 1;
    }
    node_rf[id].sf  = (uint8_t)sf;
    node_rf[id].pwr = chip_power;
    rf_invalidate();
    lora_cfg_t c = cfg_for_node(id);
    printf("Node %d: ", id); print_rf(&c);
    printf("(Nho dat cfg tuong ung tren chinh Node %d)\n", id);
    return 0;
}

/* ============================ EXP-01 / EXP-02 ============================ */
typedef struct {
    uint8_t node; float dist; uint32_t sent, ok;
    double pdr, rssi_gw, snr_gw, rssi_nd, snr_nd, rtt;
    bool pass;
} point_t;
static point_t pts[32];
static int     npts;

static void print_rmax(uint8_t node)
{
    float rmax = 0; bool any = false;
    for (int i = 0; i < npts; i++)
        if (pts[i].node == node && pts[i].pass && (!any || pts[i].dist > rmax)) { rmax = pts[i].dist; any = true; }
    if (any) printf("R%u_max hien tai = %.1f km\n", node, rmax);
    else     printf("R%u_max: chua co diem nao dat\n", node);
}

static int cmd_gw_poll(int argc, char **argv)
{
    if (argc < 3) { printf("Dung: gw_poll <node 1|2> <khoang_cach_km> [so_goi=200] [timeout_ms]\n"); return 1; }
    uint8_t  node  = atoi(argv[1]);
    float    dist  = atof(argv[2]);
    uint32_t count = argc > 3 ? atoi(argv[3]) : 200;
    if (node < 1 || node > 2 || count < 1 || count > MAXN || dist < 0.0f || dist > 100.0f) {
        printf("node phai la 1|2; count 1..%d; khoang cach 0..100 km\n", MAXN);
        return 1;
    }
    uint32_t timeout = argc > 4 ? (uint32_t)atoi(argv[4]) : node_air_ms(node) + 400;

    if (!rf_for_node(node)) return 1;
    lora_cfg_t c = cfg_for_node(node);
    printf("[EXP-0%u] Node %u @ %.1f km, %u goi POLL\n", node, node, dist, (unsigned)count);
    print_rf(&c);

    stat_t rg = {0}, sg = {0}, rn = {0}, sn = {0}, rtt = {0};
    uint32_t ok = 0, sent = 0, tx_failed = 0;
    for (uint32_t i = 0; i < count; i++) {
        if (g_abort) { printf("[HUY] dung som\n"); break; }
        int64_t t0 = now_us();
        if (!pkt_send(PT_POLL, ID_GW, node, i, 0, 0, 0)) {
            tx_failed++;
            printf("#%u LOI TX (khong gui duoc POLL)\n", (unsigned)i);
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }
        sent++;
        hdr_t h; rx_info_t ri;
        if (wait_pkt(PT_DATA, node, ID_GW, i, timeout, &h, &ri)) {
            double r = (now_us() - t0) / 1000.0;
            ok++;
            st_add(&rg, ri.rssi); st_add(&sg, ri.snr);
            st_add(&rn, from_x10(h.rssi_x10)); st_add(&sn, from_x10(h.snr_x10)); st_add(&rtt, r);
            printf("#%u OK  RTT %.0f ms | GW: %.0f dBm %.1f dB | Node: %.1f dBm %.1f dB\n",
                   (unsigned)i, r, ri.rssi, ri.snr, from_x10(h.rssi_x10), from_x10(h.snr_x10));
        } else {
            printf("#%u MAT\n", (unsigned)i);
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    point_t p = {.node = node, .dist = dist, .sent = sent, .ok = ok,
                 .pdr = sent ? 100.0 * ok / sent : 0.0, .rssi_gw = st_avg(&rg), .snr_gw = st_avg(&sg),
                 .rssi_nd = st_avg(&rn), .snr_nd = st_avg(&sn), .rtt = st_avg(&rtt)};
    /* T30D khong co SNR (NAN): chi xet PDR */
    p.pass = ok && p.pdr >= POLL_PDR_TARGET && (isnan(p.snr_gw) || p.snr_gw >= POLL_SNR_TARGET) && (isnan(p.snr_nd) || p.snr_nd >= POLL_SNR_TARGET);
    if (npts < 32) pts[npts++] = p;

    printf("\n===== EXP-0%u | Node %u | %.1f km =====\n", node, node, dist);
    printf("Nhan %u/%u POLL da gui  | TX loi %u/%u  | PDR %.1f %%  PER %.1f %%\n",
           (unsigned)ok, (unsigned)sent, (unsigned)tx_failed, (unsigned)count, p.pdr, 100.0 - p.pdr);
    if (ok) {
        printf("GW  : RSSI tb %.1f dBm (min %.0f) | SNR tb %.1f dB (min %.1f)\n", p.rssi_gw, rg.min, p.snr_gw, sg.min);
        printf("Node: RSSI tb %.1f dBm (min %.0f) | SNR tb %.1f dB (min %.1f)\n", p.rssi_nd, rn.min, p.snr_nd, sn.min);
        printf("RTT tb %.0f ms (max %.0f)\n", p.rtt, rtt.max);
    }
    printf("Dat (PDR >= %.0f %% va SNR >= %.0f dB): %s\n", POLL_PDR_TARGET, POLL_SNR_TARGET, p.pass ? "PASS" : "FAIL");
    print_rmax(node);
    printf("CSV,EXP0%u,%u,%.1f,%u,%u,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.0f,%s\n", node, node, dist, (unsigned)sent, (unsigned)ok,
           p.pdr, 100.0 - p.pdr, p.rssi_gw, p.snr_gw, p.rssi_nd, p.snr_nd, p.rtt, p.pass ? "PASS" : "FAIL");
    return 0;
}

static int cmd_gw_points(int argc, char **argv)
{
    printf("Node | km   | PDR%%  | RSSI_GW | SNR_GW | RSSI_Nd | SNR_Nd | Ket qua\n");
    for (int i = 0; i < npts; i++)
        printf("  %u  | %4.1f | %5.1f | %7.1f | %6.1f | %7.1f | %6.1f | %s\n", pts[i].node, pts[i].dist, pts[i].pdr,
               pts[i].rssi_gw, pts[i].snr_gw, pts[i].rssi_nd, pts[i].snr_nd, pts[i].pass ? "PASS" : "FAIL");
    float r[3] = {0, 0, 0};
    for (int n = 1; n <= 2; n++) { for (int i = 0; i < npts; i++) if (pts[i].node == n && pts[i].pass && pts[i].dist > r[n]) r[n] = pts[i].dist; }
    printf("R1_max = %.1f km | R2_max = %.1f km | D_max = R1 + R2 = %.1f km\n", r[1], r[2], r[1] + r[2]);
    printf("Tieu chi E22-900M30S (LOS): R_max >= 5-8 km, D_max >= 10-15 km\n");
    return 0;
}

/* ============================ EXP-03 / EXP-06: TDMA ============================ */
typedef struct { uint32_t sent, ok; stat_t rg, sg, rn, sn, rtt; } ns_t;
static double ns_pdr(const ns_t *n) { return n->sent ? 100.0 * n->ok / n->sent : 0; }

static void win_print(int idx, const ns_t *w)
{
    printf("[Cua so %d] ", idx);
    for (int n = 1; n <= 2; n++)
        printf("Node%d PDR %.0f%% (%.0f dBm, %.1f dB)%s", n, ns_pdr(&w[n]), st_avg(&w[n].rg), st_avg(&w[n].sg), n == 1 ? " | " : "\n");
    printf("CSV,TDMAWIN,%d,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f\n", idx, ns_pdr(&w[1]), st_avg(&w[1].rg), st_avg(&w[1].sg),
           ns_pdr(&w[2]), st_avg(&w[2].rg), st_avg(&w[2].sg));
}

static void suggest_sf(int node, const ns_t *n, uint8_t sf)
{
    if (!n->ok) { printf("Node %d: khong co du lieu de khuyen nghi\n", node); return; }
    double margin = st_avg(&n->sg) - demod_snr_limit(sf);
    if (isnan(margin)) { printf("Node %d: SF%u, khong co SNR de danh gia\n", node, sf); return; }
    printf("Node %d: SF%u, SNR tb %.1f dB, du phong so voi nguong giai dieu che %.1f dB -> ", node, sf, st_avg(&n->sg), margin);
    if (margin > 10 && sf > 7)       printf("du thua tin hieu: co the giam SF xuong %u (hoac giam cong suat)\n", sf - 1);
    else if (margin < 3 && sf < 12)  printf("du phong thap: nen tang SF len %u hoac tang cong suat/anten\n", sf + 1);
    else                             printf("cau hinh hop ly\n");
}

static int cmd_gw_tdma(int argc, char **argv)
{
    uint32_t cycles = argc > 1 ? atoi(argv[1]) : 500;
    uint32_t dur_s  = argc > 2 ? atoi(argv[2]) : 1800;
    float    r1 = argc > 3 ? atof(argv[3]) : 0, r2 = argc > 4 ? atof(argv[4]) : 0;
    if (cycles < 1 || dur_s < 1) return 1;
    int64_t period_us = (int64_t)dur_s * 1000000 / cycles;
    uint32_t win_cycles = cycles >= 100 ? 50 : (cycles / 2 ? cycles / 2 : 1);

    printf("[TDMA] %u chu ky / %u s (1 chu ky moi %.1f s): POLL N1 -> DATA 1 -> POLL N2 -> DATA 2\n",
           (unsigned)cycles, (unsigned)dur_s, period_us / 1e6);
    for (int n = 1; n <= 2; n++) { lora_cfg_t c = cfg_for_node(n); printf("Node %d: ", n); print_rf(&c); }

    ns_t tot[3] = {0}, win[3] = {0};
    stat_t cyc = {0};
    int widx = 1;
    int64_t t0 = now_us();
    for (uint32_t c = 0; c < cycles; c++) {
        if (g_abort) { printf("[HUY] dung som\n"); break; }
        sleep_until(t0 + (int64_t)c * period_us);
        if (g_abort) { printf("[HUY] dung som\n"); break; }
        int64_t cs = now_us();
        for (uint8_t n = 1; n <= 2; n++) {
            if (!rf_for_node(n)) {
                printf("[TDMA] Dung vi cau hinh RF cho Node %u that bai\n", n);
                return 1;
            }
            uint32_t to = node_air_ms(n) + 400;
            int64_t ts = now_us();
            pkt_send(PT_POLL, ID_GW, n, c, 0, 0, 0);
            hdr_t h; rx_info_t ri;
            bool ok = wait_pkt(PT_DATA, n, ID_GW, c, to, &h, &ri);
            tot[n].sent++; win[n].sent++;
            if (ok) {
                double rtt = (now_us() - ts) / 1000.0;
                ns_t *a[2] = {&tot[n], &win[n]};
                for (int k = 0; k < 2; k++) {
                    a[k]->ok++;
                    st_add(&a[k]->rg, ri.rssi); st_add(&a[k]->sg, ri.snr);
                    st_add(&a[k]->rn, from_x10(h.rssi_x10)); st_add(&a[k]->sn, from_x10(h.snr_x10)); st_add(&a[k]->rtt, rtt);
                }
            } else {
                printf("  !! chu ky %u: MAT Node %u\n", (unsigned)c, n);
            }
        }
        st_add(&cyc, (now_us() - cs) / 1000.0);
        if ((c + 1) % win_cycles == 0 || c + 1 == cycles) { win_print(widx++, win); memset(win, 0, sizeof win); }
    }

    uint32_t s = tot[1].sent + tot[2].sent, o = tot[1].ok + tot[2].ok;
    double pdr = s ? 100.0 * o / s : 0;
    printf("\n===== TDMA tong ket =====\n");
    for (int n = 1; n <= 2; n++)
        printf("Node %d: PDR %.1f %% (%u/%u) | GW %.1f dBm %.1f dB | Node %.1f dBm %.1f dB | RTT tb %.0f ms (max %.0f)\n", n,
               ns_pdr(&tot[n]), (unsigned)tot[n].ok, (unsigned)tot[n].sent, st_avg(&tot[n].rg), st_avg(&tot[n].sg),
               st_avg(&tot[n].rn), st_avg(&tot[n].sn), st_avg(&tot[n].rtt), tot[n].rtt.max);
    printf("PDR toan mang: %.1f %% | Thoi gian 1 chu ky mang: tb %.0f ms (max %.0f ms)\n", pdr, st_avg(&cyc), cyc.max);
    if (r1 > 0 || r2 > 0) printf("D_max = R1 + R2 = %.1f + %.1f = %.1f km\n", r1, r2, r1 + r2);
    printf("EXP-03: PDR toan mang >= %.0f %%: %s\n", TDMA_PDR_TARGET, pdr >= TDMA_PDR_TARGET ? "PASS" : "FAIL");
    printf("EXP-06: moi node PDR >= %.0f %%: %s\n", ASYM_PDR_TARGET,
           (ns_pdr(&tot[1]) >= ASYM_PDR_TARGET && ns_pdr(&tot[2]) >= ASYM_PDR_TARGET) ? "PASS" : "FAIL");
    printf("Chenh lech RSSI giua 2 node tai GW: %.1f dB\n", fabs(st_avg(&tot[1].rg) - st_avg(&tot[2].rg)));
    for (int n = 1; n <= 2; n++) suggest_sf(n, &tot[n], cfg_for_node(n).sf);
    printf("CSV,TDMA,%u,%u,%.1f,%.1f,%.1f,%.0f,%.0f,%.1f\n", (unsigned)cycles, (unsigned)dur_s, pdr, ns_pdr(&tot[1]), ns_pdr(&tot[2]),
           st_avg(&tot[1].rtt), st_avg(&tot[2].rtt), r1 + r2);
    return 0;
}

/* ============================ EXP-04: 2 node dong thoi ============================ */
static uint8_t seenN[3][MAXN];

static int cmd_gw_listen(int argc, char **argv)
{
    uint32_t expected = argc > 1 ? atoi(argv[1]) : 300;
    int      cad      = argc > 2 ? atoi(argv[2]) : 0;
    uint32_t idle_s   = argc > 3 ? atoi(argv[3]) : 60;
    if (expected < 1 || expected > MAXN) { printf("so_goi 1..%d\n", MAXN); return 1; }
    if (cad != 0 && cad != 1) { printf("cad phai la 0 hoac 1\n"); return 1; }
    if (idle_s < 1 || idle_s > 3600) { printf("idle_s phai trong 1..3600\n"); return 1; }
    memset(seenN, 0, sizeof seenN);
    g_crc_err = g_hdr_err = 0;
    if (!rf_for_node(0)) return 1;
    print_rf(&g_cfg);
    printf("[EXP-04] Kich ban %s | cho %u goi/node (ket thuc khi im lang %u s)\n",
           cad ? "CAD/LBT + backoff" : "ALOHA", (unsigned)expected, (unsigned)idle_s);

    uint32_t ok[3] = {0}, dup[3] = {0};
    stat_t rs[3] = {{0}}, sn[3] = {{0}};
    int64_t last = 0;
    while (1) {
        if (g_abort) { printf("[HUY] dung som\n"); break; }
        if (ok[1] + ok[2] > 0 && (now_us() - last) > (int64_t)idle_s * 1000000) break;
        hdr_t h; rx_info_t ri;
        if (!pkt_recv(&h, &ri, 300) || h.type != PT_UP || h.dst != ID_GW) continue;
        if (h.src < 1 || h.src > 2 || h.seq >= expected) continue;
        last = now_us();
        if (seenN[h.src][h.seq]) { dup[h.src]++; continue; }
        seenN[h.src][h.seq] = 1; ok[h.src]++;
        st_add(&rs[h.src], ri.rssi); st_add(&sn[h.src], ri.snr);
        if ((ok[1] + ok[2]) % 100 == 0) printf("  nhan N1 %u | N2 %u\n", (unsigned)ok[1], (unsigned)ok[2]);
    }

    uint32_t gap[3] = {0};
    for (int n = 1; n <= 2; n++) {
        uint32_t run = 0;
        for (uint32_t i = 0; i < expected; i++) { if (!seenN[n][i]) { run++; if (run > gap[n]) gap[n] = run; } else run = 0; }
    }
    double pdr[3], loss[3];
    printf("\n===== EXP-04 ket qua (%s) =====\n", cad ? "CAD/LBT" : "ALOHA");
    for (int n = 1; n <= 2; n++) {
        pdr[n] = 100.0 * ok[n] / expected; loss[n] = 100.0 - pdr[n];
        printf("Node %d: nhan %u/%u | PDR %.1f %% | mat %.1f %% | RSSI tb %.1f dBm (min %.0f max %.0f) | SNR tb %.1f dB | mat lien tiep toi da %u\n",
               n, (unsigned)ok[n], (unsigned)expected, pdr[n], loss[n], st_avg(&rs[n]), rs[n].min, rs[n].max, st_avg(&sn[n]),
               (unsigned)gap[n]);
    }
    double avg_loss = (loss[1] + loss[2]) / 2.0, thr = cad ? CAD_LOSS_MAX : ALOHA_LOSS_MAX;
    double rdiff = fabs(st_avg(&rs[1]) - st_avg(&rs[2]));
    bool alive = ok[1] > 0 && ok[2] > 0 && gap[1] <= MAX_GAP_OK && gap[2] <= MAX_GAP_OK;
    printf("Khung loi CRC %u, loi header %u (dau hieu va cham)\n", (unsigned)g_crc_err, (unsigned)g_hdr_err);
    printf("Chenh lech RSSI N1/N2: %.1f dB -> %s\n", rdiff,
           (rdiff >= 6 && fabs(pdr[1] - pdr[2]) >= 5) ? "co dau hieu Capture Effect (node manh hon it mat goi hon)" : "khong ro Capture Effect");
    printf("Mat goi TB %.1f %% (nguong < %.0f %%): %s | Khong node nao bi treo/drop: %s\n", avg_loss, thr,
           avg_loss < thr ? "PASS" : "FAIL", alive ? "PASS" : "FAIL");
    printf("Luu y: mat goi gom ca suy hao duong truyen; tru di PER nen cua tung node do bang gw_poll tai cung vi tri.\n");
    printf("CSV,EXP04,%s,%u,%u,%.1f,%.1f,%.1f,%.1f,%.1f,%u,%u\n", cad ? "CAD" : "ALOHA", (unsigned)ok[1], (unsigned)ok[2], pdr[1], pdr[2],
           st_avg(&rs[1]), st_avg(&rs[2]), avg_loss, (unsigned)g_crc_err, (unsigned)g_hdr_err);
    return 0;
}

/* ============================ EXP-05: relay ============================ */
static int cmd_gw_relay(int argc, char **argv)
{
    uint32_t idle_s = argc > 1 ? atoi(argv[1]) : 60;
    if (idle_s < 1 || idle_s > 3600) { printf("idle_s phai trong 1..3600\n"); return 1; }
    if (!rf_for_node(0)) return 1;
    print_rf(&g_cfg);
    printf("[EXP-05 GW] Dang cho RLY_DATA de chuyen tiep (ket thuc khi im lang %u s)\n", (unsigned)idle_s);

    uint32_t got = 0, fwd_ok = 0, fwd_fail = 0, bad = 0;
    stat_t proc = {0};
    int64_t first = 0, last = 0, busy_us = 0;
    uint32_t reply_to = air_ms() + 500;
    while (1) {
        if (g_abort) { printf("[HUY] dung som\n"); break; }
        if (got > 0 && (now_us() - last) > (int64_t)idle_s * 1000000) break;
        hdr_t h; rx_info_t ri;
        if (!pkt_recv(&h, &ri, 300) || h.type != PT_RLY_DATA) continue;
        int64_t t_in = now_us();
        if (!first) first = t_in;
        got++;
        if ((h.dst != 1 && h.dst != 2) || h.dst == h.src || (h.src != 1 && h.src != 2)) { bad++; last = now_us(); continue; }

        vTaskDelay(pdMS_TO_TICKS(TURNAROUND_MS));
        pkt_send(PT_RLY_FWD, h.src, h.dst, h.seq, x10(ri.rssi), x10(ri.snr), 0);          /* chuyen tiep toi node dich */
        hdr_t r; rx_info_t rri;
        if (wait_pkt(PT_RLY_REPLY, h.dst, h.src, h.seq, reply_to, &r, &rri)) {
            vTaskDelay(pdMS_TO_TICKS(TURNAROUND_MS));
            pkt_send(PT_RLY_ACK, h.dst, h.src, h.seq, x10(rri.rssi), x10(rri.snr), 0);    /* chuyen phan hoi ve node nguon */
            fwd_ok++;
            printf("#%u: N%u -> N%u OK\n", (unsigned)h.seq, h.src, h.dst);
        } else {
            fwd_fail++;
            printf("#%u: N%u -> N%u: node dich khong tra loi\n", (unsigned)h.seq, h.src, h.dst);
        }
        int64_t t_out = now_us();
        busy_us += t_out - t_in; st_add(&proc, (t_out - t_in) / 1000.0);
        last = now_us();
    }
    double span = first ? (last - first) / 1e6 : 0;
    printf("\n===== EXP-05 GW tong ket =====\n");
    printf("Nhan yeu cau: %u | chuyen tiep thanh cong: %u | node dich khong tra loi: %u | sai dia chi: %u\n",
           (unsigned)got, (unsigned)fwd_ok, (unsigned)fwd_fail, (unsigned)bad);
    printf("Thoi gian xu ly 1 luot tai GW: tb %.0f ms (max %.0f ms)\n", st_avg(&proc), proc.max);
    printf("GW ban %.1f %% thoi gian | heap trong %u B (thap nhat %u B)\n", span > 0 ? 100.0 * busy_us / 1e6 / span : 0,
           (unsigned)esp_get_free_heap_size(), (unsigned)esp_get_minimum_free_heap_size());
    printf("GW xu ly tuan tu (khong co hang doi RF): goi den khi GW dang ban se bi mat -> tinh vao PDR phia Node 1.\n");
    return 0;
}

void gateway_register(void)
{
    reg_cmd("gw_nodecfg", "gw_nodecfg <id> <sf> <cong_suat_dau_ra_module>  RF rieng tung node (EXP-06)", cmd_gw_nodecfg);
    reg_cmd("gw_poll",    "gw_poll <node> <km> [so_goi=200] [timeout_ms]  EXP-01/02", cmd_gw_poll);
    reg_cmd("gw_points",  "bang tong hop cac diem + R1_max/R2_max/D_max", cmd_gw_points);
    reg_cmd("gw_tdma",    "gw_tdma [chu_ky=500] [thoi_gian_s=1800] [R1_km] [R2_km]  EXP-03/06", cmd_gw_tdma);
    reg_cmd("gw_listen",  "gw_listen [so_goi=300] [cad 0|1] [idle_s=60]  EXP-04", cmd_gw_listen);
    reg_cmd("gw_relay",   "gw_relay [idle_s=60]  EXP-05", cmd_gw_relay);
}
