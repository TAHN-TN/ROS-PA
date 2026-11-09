#pragma once
#include <stdint.h>
enum { BTN_NONE = 0, BTN_UP, BTN_DOWN, BTN_OK, BTN_BACK };
void btn_init(void);
int  btn_get(int32_t timeout_ms);     /* BTN_* hoac BTN_NONE khi het gio; timeout < 0 = cho mai */
