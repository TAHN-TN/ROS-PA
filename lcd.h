#pragma once
#include "esp_err.h"
esp_err_t lcd_init(void);                       /* LCD 16x2 I2C (PCF8574), dia chi 0x27 hoac 0x3F */
void      lcd_clear(void);
void      lcd_printf(int row, const char *fmt, ...) __attribute__((format(printf, 2, 3)));   /* ghi 1 dong, tu dem khoang trang */
