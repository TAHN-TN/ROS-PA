#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
void wifi_log_init(void);
int wifi_log_printf(const char *fmt, ...) __attribute__((format(printf, 1, 2)));
bool wifi_log_connected(void);
const char *wifi_log_last_line(int back);
uint32_t wifi_log_cursor(void);
bool wifi_log_read_lcd(uint32_t *cursor, char top[17], char bottom[17]);
bool wifi_log_read_lcd_until(uint32_t *cursor, uint32_t end, char top[17], char bottom[17]);
uint32_t wifi_log_lcd_previous_page(uint32_t cursor, uint32_t floor);
void wifi_log_set_verdict_prefix(const char *p);
int wifi_log_verdict(void);
#ifndef WIFI_LOG_NO_PRINTF_MACRO
#define printf(...) wifi_log_printf(__VA_ARGS__)
#endif
