#include "leds.h"

#include "board.h"
#include "driver/gpio.h"
#include "esp_timer.h"

static led_state_t s_state = LED_WAIT;

static void apply(int wait, int run, int done)
{
    gpio_set_level(PIN_LED_WAIT, wait);
    gpio_set_level(PIN_LED_RUN, run);
    gpio_set_level(PIN_LED_DONE, done);
}

void led_set(led_state_t s)
{
    s_state = s;
    switch (s) {
    case LED_WAIT:  apply(1, 0, 0); break;
    case LED_RUN:   apply(0, 1, 0); break;
    case LED_DONE:  apply(0, 0, 1); break;
    default:        apply(0, 0, 0); break;
    }
}

void led_init(void)
{
    gpio_config_t c = {.pin_bit_mask = (1ULL << PIN_LED_WAIT) | (1ULL << PIN_LED_RUN) | (1ULL << PIN_LED_DONE),
                       .mode = GPIO_MODE_OUTPUT};
    gpio_config(&c);
    led_set(LED_WAIT);
}

void led_tick(void)
{
    int phase = (int)((esp_timer_get_time() / 300000) & 1);      /* doi trang thai moi 300 ms */
    if (s_state == LED_RUN)        gpio_set_level(PIN_LED_RUN, phase);
    else if (s_state == LED_ERROR) apply(phase, phase, phase);
}
