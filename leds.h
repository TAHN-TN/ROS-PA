#pragma once

typedef enum {
    LED_WAIT = 0,
    LED_RUN,
    LED_DONE,
    LED_ERROR
} led_state_t;

void led_init(void);
void led_set(led_state_t state);
void led_tick(void);
