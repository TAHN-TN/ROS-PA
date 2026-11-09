#pragma once
/*
 * Giao dien dieu khien Broad Test: menu tren LCD 16x2 + 4 nut + 3 den trang thai.
 * Moi bai test chay lai cac lenh console CU (gw_poll, gw_tdma, gw_listen, gw_relay, node, node_up, n_relay...)
 * qua esp_console_run(), nen noi dung va quy trinh xu ly giu nguyen; log di qua Wi-Fi (wifi_log).
 */
void ui_run(void) __attribute__((noreturn));
