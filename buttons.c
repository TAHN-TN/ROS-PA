#include "buttons.h"

#include <stdbool.h>
#include <stddef.h>

#include "board.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

static QueueHandle_t s_q;

/* Quet 10 ms, chong doi 5 mau; UP/DOWN tu lap sau 500 ms (moi 100 ms) */
static void btn_task(void *arg)
{
    const int pins[4] = {PIN_BTN_UP, PIN_BTN_DOWN, PIN_BTN_OK, PIN_BTN_BACK};
    const int ids[4]  = {BTN_UP, BTN_DOWN, BTN_OK, BTN_BACK};
    uint8_t  cnt[4]   = {0};
    bool     down[4]  = {false};
    uint32_t held[4]  = {0};
    while (1) {
        for (int i = 0; i < 4; i++) {
            bool pressed = gpio_get_level(pins[i]) == 0;
            if (pressed) {
                if (cnt[i] < 5) cnt[i]++;
                if (cnt[i] == 5 && !down[i]) {
                    down[i] = true; held[i] = 0;
                    int e = ids[i];
                    xQueueSend(s_q, &e, 0);
                } else if (down[i] && i < 2) {
                    held[i] += 10;
                    if (held[i] >= 500 && ((held[i] - 500) % 100) == 0) {
                        int e = ids[i];
                        xQueueSend(s_q, &e, 0);
                    }
                }
            } else {
                cnt[i] = 0; down[i] = false;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void btn_init(void)
{
    gpio_config_t c = {.pin_bit_mask = (1ULL << PIN_BTN_UP) | (1ULL << PIN_BTN_DOWN) | (1ULL << PIN_BTN_OK) | (1ULL << PIN_BTN_BACK),
                       .mode = GPIO_MODE_INPUT, .pull_up_en = GPIO_PULLUP_ENABLE};
    gpio_config(&c);
    s_q = xQueueCreate(8, sizeof(int));
    xTaskCreate(btn_task, "btn", 2048, NULL, 5, NULL);
}

int btn_get(int32_t timeout_ms)
{
    int e = BTN_NONE;
    TickType_t t = timeout_ms < 0 ? portMAX_DELAY : pdMS_TO_TICKS((uint32_t)timeout_ms);
    if (xQueueReceive(s_q, &e, t) == pdTRUE) return e;
    return BTN_NONE;
}
