#pragma once
/*
 * So do chan bo Broad Test (ESP32 DevKit 38 chan). Chi gom: ESP32 + LCD 1602 I2C + 4 nut + 3 den + module LoRa.
 *
 * GIAC MODULE: SPI 18/19/23/5 + cac chan IO tuy module
 *   IO16: NRST | ESP RX (T30D)     IO17: BUSY | ESP TX (T30D)     IO4: AUX (T30D), khong can noi DIO1
 *   IO25: RXEN (M30S) / RF_EN (Ra-01SH-P) | M0 (T30D)     IO26: TXEN (M30S) | M1 (T30D)
 */
#define PIN_SCK   18
#define PIN_MISO  19
#define PIN_MOSI  23
#define PIN_NSS   5
#define PIN_IO16  16
#define PIN_IO17  17
#define PIN_IO4   4
#define PIN_IO25  25
#define PIN_IO26  26
#define PIN_NRST  PIN_IO16
#define PIN_BUSY  PIN_IO17
#define PIN_RXEN  PIN_IO25
#define PIN_TXEN  PIN_IO26

/* LCD 16x2 I2C (PCF8574 tren mach LCD) */
#define PIN_SDA   21
#define PIN_SCL   22
#define LCD_COLS  16
#define LCD_ROWS  2
#define LCD_PCF_MAP             0       /* 0 = mach pho bien (D4..D7 = P4..P7); thu 1 neu LCD hien ky tu rac */
#define LCD_I2C_HZ              100000
#define LCD_I2C_INTERNAL_PULLUP 0       /* 0 = chi dung tro keo len tren mach LCD (khuyen nghi); 1 = bat them pull-up trong ESP32 */

/* 4 nut bam noi chan -> GND (pull-up trong). BACK = GPIO0 (nut BOOT tren DevKit hoac nut ngoai song song) */
#define PIN_BTN_UP    32
#define PIN_BTN_DOWN  33
#define PIN_BTN_OK    27
#define PIN_BTN_BACK  0

/* 3 den: chan GPIO -> dien tro 330 ohm -> LED -> GND (sang khi muc cao) */
#define PIN_LED_WAIT  13      /* vang: cho lenh */
#define PIN_LED_RUN   14      /* xanh la: dang chay (nhay) */
#define PIN_LED_DONE  2       /* xanh duong: hoan thanh */

/* E22 UART */
#define E22U_CFG_M0         1     /* M0/M1 vao che do cau hinh */
#define E22U_CFG_M1         1
#define E22U_MAX_DBM        30
#define E22U_AIR_SCALE_PCT  250
