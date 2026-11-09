#define WIFI_LOG_NO_PRINTF_MACRO
#include "wifi_log.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <inttypes.h>
#include "esp_event.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include "lwip/sockets.h"
#include "lwip/inet.h"
#include "lwip/netdb.h"
#include <errno.h>

#define RING_SZ (16 * 1024)
#define WIFI_LOG_LINE_MAX 64
#define NLINES 4
#define AP_SSID "LoRaBroadTest"
#define AP_PASS "lora1234"
#define WIFI_AP_ENABLED 1
static const char *TAG = "wifi_log";
static char s_ring[RING_SZ];
static uint32_t s_wr;
static SemaphoreHandle_t s_mx;
static volatile bool s_conn;
static char s_lines[NLINES][WIFI_LOG_LINE_MAX];
static int s_nl;
static char s_cur[WIFI_LOG_LINE_MAX];
static int s_cur_len;
static char s_prefix[24];
static int s_verdict;

static void line_done(void) {
    s_cur[s_cur_len] = 0;
    strcpy(s_lines[s_nl % NLINES], s_cur); s_nl++;
    if (s_prefix[0] && !strncmp(s_cur, s_prefix, strlen(s_prefix))) {
        if (strstr(s_cur, "FAIL")) s_verdict = 2;
        else if (strstr(s_cur, "PASS") && s_verdict != 2) s_verdict = 1;
    }
    s_cur_len = 0;
}
static void log_write(const char *s, size_t n) {
    if (!s_mx || !n) return;
    xSemaphoreTake(s_mx, portMAX_DELAY);
    for (size_t i=0;i<n;i++) {
        s_ring[s_wr % RING_SZ] = s[i]; s_wr++;
        if (s[i]=='\n') line_done();
        else if (s[i]!='\r' && s_cur_len<WIFI_LOG_LINE_MAX-1) s_cur[s_cur_len++]=s[i];
    }
    xSemaphoreGive(s_mx);
#if CONFIG_FT_LOG_UART
    fwrite(s, 1, n, stdout);
#endif
}
int wifi_log_printf(const char *fmt, ...) {
    char buf[256]; va_list ap; va_start(ap,fmt); int n=vsnprintf(buf,sizeof buf,fmt,ap); va_end(ap);
    if (n < 0) return n;
    log_write(buf, (size_t)n >= sizeof buf ? sizeof buf - 1 : (size_t)n);
    return n;
}
static int log_vprintf(const char *fmt, va_list ap) {
    char buf[256]; int n=vsnprintf(buf,sizeof buf,fmt,ap); if(n<0)return n;
    if (strstr(buf, " DEBUG: ")) return n;
    log_write(buf,(size_t)n>=sizeof buf?sizeof buf-1:(size_t)n); return n;
}
bool wifi_log_connected(void) { return s_conn; }
uint32_t wifi_log_cursor(void) {
    if (!s_mx) return 0;
    xSemaphoreTake(s_mx, portMAX_DELAY);
    uint32_t cursor = s_wr;
    xSemaphoreGive(s_mx);
    return cursor;
}

bool wifi_log_read_lcd_until(uint32_t *cursor, uint32_t end, char top[17], char bottom[17]) {
    if (!s_mx || !cursor || !top || !bottom) return false;

    memset(top, ' ', 16);
    memset(bottom, ' ', 16);
    top[16] = '\0';
    bottom[16] = '\0';

    xSemaphoreTake(s_mx, portMAX_DELAY);
    uint32_t oldest = s_wr > RING_SZ ? s_wr - RING_SZ : 0;
    if (end > s_wr) end = s_wr;
    if (*cursor < oldest) {
        *cursor = oldest;
        memcpy(top, "LOG LOST", sizeof("LOG LOST") - 1);
        xSemaphoreGive(s_mx);
        return true;
    }

    uint32_t start = *cursor;
    char *rows[2] = {top, bottom};
    int row = 0, col = 0;
    bool wrapped = false;
    while (*cursor < end && row < 2) {
        char c = s_ring[(*cursor)++ % RING_SZ];
        if (c == '\r') continue;
        if (c == '\n') {
            if (wrapped) wrapped = false;
            else { row++; col = 0; }
            continue;
        }
        wrapped = false;
        if ((unsigned char)c < 32 || (unsigned char)c >= 127) c = '?';
        rows[row][col++] = c;
        if (col == 16) {
            row++;
            col = 0;
            wrapped = true;
        }
    }
    bool read = *cursor != start;
    xSemaphoreGive(s_mx);
    return read;
}

bool wifi_log_read_lcd(uint32_t *cursor, char top[17], char bottom[17]) {
    return wifi_log_read_lcd_until(cursor, wifi_log_cursor(), top, bottom);
}

uint32_t wifi_log_lcd_previous_page(uint32_t cursor, uint32_t floor) {
    if (!s_mx) return floor;
    xSemaphoreTake(s_mx, portMAX_DELAY);
    uint32_t oldest = s_wr > RING_SZ ? s_wr - RING_SZ : 0;
    if (floor < oldest) floor = oldest;
    if (cursor > s_wr) cursor = s_wr;
    if (cursor <= floor) {
        xSemaphoreGive(s_mx);
        return floor;
    }

    uint32_t starts[3] = {floor, floor, floor};
    int count = 1, col = 0;
    bool wrapped = false;
    for (uint32_t pos = floor; pos < cursor; pos++) {
        char c = s_ring[pos % RING_SZ];
        if (c == '\r') continue;
        if (c == '\n') {
            if (wrapped) {
                wrapped = false;
            } else {
                col = 0;
                uint32_t next = pos + 1;
                if (count < 3) starts[count++] = next;
                else { starts[0] = starts[1]; starts[1] = starts[2]; starts[2] = next; }
            }
            continue;
        }

        wrapped = false;
        if (++col == 16) {
            col = 0;
            uint32_t next = pos + 1;
            if (count < 3) starts[count++] = next;
            else { starts[0] = starts[1]; starts[1] = starts[2]; starts[2] = next; }
            wrapped = true;
        }
    }
    uint32_t previous = count == 3 ? starts[0] : floor;
    xSemaphoreGive(s_mx);
    return previous;
}

const char *wifi_log_last_line(int back) {
    static char out[WIFI_LOG_LINE_MAX]; if(!s_mx)return ""; xSemaphoreTake(s_mx,portMAX_DELAY);
    int idx=s_nl-1-back; if(idx<0||back>=NLINES)out[0]=0;else strcpy(out,s_lines[idx%NLINES]);
    xSemaphoreGive(s_mx); return out;
}
void wifi_log_set_verdict_prefix(const char *p) { xSemaphoreTake(s_mx,portMAX_DELAY); strncpy(s_prefix,p?p:"",sizeof s_prefix-1); s_prefix[sizeof s_prefix-1]=0; s_verdict=0; xSemaphoreGive(s_mx); }
int wifi_log_verdict(void) { return s_verdict; }

static const char page[] = "<!doctype html><html><head><meta charset='utf-8'><meta name='viewport' content='width=device-width,initial-scale=1'><title>LoRa Broad Test</title><style>body{font:16px system-ui;background:#101820;color:#e8f0f5;margin:0;padding:16px}h2{margin:0 0 8px}p{color:#a9bdc9}pre{background:#05090c;border:1px solid #31434f;border-radius:8px;padding:12px;height:72vh;overflow:auto;white-space:pre-wrap;word-break:break-word}button{padding:8px 12px;margin:4px 4px 10px 0}</style></head><body><h2>LoRa Broad Test</h2><p>Log trực tiếp từ ESP32 qua Wi-Fi · tự động cập nhật</p><button onclick='log.textContent=\"\"'>Xóa màn hình</button><button onclick='paused=!paused;this.textContent=paused?\"Tiếp tục\":\"Tạm dừng\"'>Tạm dừng</button><pre id='log'>Đang kết nối...</pre><script>let pos=0,paused=false;const el=document.getElementById('log');async function poll(){try{let r=await fetch('/logs?from='+pos,{cache:'no-store'});let t=await r.text();pos=Number(r.headers.get('X-Next-Offset')||pos);if(!paused&&t){if(el.textContent==='Đang kết nối...')el.textContent='';el.textContent+=t;el.scrollTop=el.scrollHeight}}catch(e){}setTimeout(poll,350)}poll();</script></body></html>";
static esp_err_t root_get(httpd_req_t *req) { httpd_resp_set_type(req,"text/html; charset=utf-8"); return httpd_resp_send(req,page,HTTPD_RESP_USE_STRLEN); }
static esp_err_t captive_redirect(httpd_req_t *req) {
    httpd_resp_set_status(req, "302 Found");
    httpd_resp_set_hdr(req, "Location", "http://192.168.4.1/");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return httpd_resp_send(req, "", 0);
}

/* Resolve every DNS name to the ESP32 captive portal address. */
static void dns_server_task(void *arg) {
    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock < 0) { ESP_LOGE(TAG, "DNS socket failed: %d", errno); vTaskDelete(NULL); return; }
    struct sockaddr_in addr = {0};
    addr.sin_family = AF_INET; addr.sin_port = htons(53); addr.sin_addr.s_addr = htonl(INADDR_ANY);
    if (bind(sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        ESP_LOGE(TAG, "DNS bind failed: %d", errno); close(sock); vTaskDelete(NULL); return;
    }
    uint8_t pkt[512];
    while (1) {
        struct sockaddr_in client; socklen_t clen = sizeof(client);
        int n = recvfrom(sock, pkt, sizeof(pkt), 0, (struct sockaddr *)&client, &clen);
        if (n < 17) continue;
        size_t qend = 12;
        while (qend < (size_t)n && pkt[qend] != 0) {
            uint8_t len = pkt[qend];
            if ((len & 0xC0) || qend + 1 + len >= (size_t)n) break;
            qend += 1 + len;
        }
        if (qend + 5 > (size_t)n || pkt[qend] != 0) continue;
        qend += 1;
        uint16_t qtype = ((uint16_t)pkt[qend] << 8) | pkt[qend+1];
        uint16_t qclass = ((uint16_t)pkt[qend+2] << 8) | pkt[qend+3];
        size_t question_end = qend + 4;
        if (question_end > (size_t)n) continue;
        pkt[2] = 0x81; pkt[3] = 0x80; /* response, recursion available */
        pkt[6] = 0; pkt[7] = (qtype == 1 && qclass == 1) ? 1 : 0;
        pkt[8] = pkt[9] = pkt[10] = pkt[11] = 0;
        size_t out = question_end;
        if (qtype == 1 && qclass == 1 && out + 16 <= sizeof(pkt)) {
            pkt[out++] = 0xC0; pkt[out++] = 0x0C;
            pkt[out++] = 0; pkt[out++] = 1; pkt[out++] = 0; pkt[out++] = 1;
            pkt[out++] = 0; pkt[out++] = 0; pkt[out++] = 0; pkt[out++] = 30;
            pkt[out++] = 0; pkt[out++] = 4;
            pkt[out++] = 192; pkt[out++] = 168; pkt[out++] = 4; pkt[out++] = 1;
        }
        sendto(sock, pkt, out, 0, (struct sockaddr *)&client, clen);
    }
}
static esp_err_t logs_get(httpd_req_t *req) {
    s_conn = true;
    char query[48]; uint32_t from=0;
    if(httpd_req_get_url_query_str(req,query,sizeof query)==ESP_OK){char v[20];if(httpd_query_key_value(query,"from",v,sizeof v)==ESP_OK)from=(uint32_t)strtoul(v,NULL,10);}
    char out[1200]; size_t n=0; uint32_t end;
    xSemaphoreTake(s_mx,portMAX_DELAY); end=s_wr;
    if(end-from>RING_SZ)from=end-RING_SZ;
    uint32_t avail=end-from; if(avail>sizeof(out)-1)avail=sizeof(out)-1;
    for(uint32_t i=0;i<avail;i++)out[n++]=s_ring[(from+i)%RING_SZ];
    xSemaphoreGive(s_mx); out[n]=0;
    char off[24]; snprintf(off,sizeof off,"%" PRIu32,from+(uint32_t)n);
    httpd_resp_set_type(req,"text/plain; charset=utf-8"); httpd_resp_set_hdr(req,"Cache-Control","no-store"); httpd_resp_set_hdr(req,"X-Next-Offset",off);
    return httpd_resp_send(req,out,n);
}
static void wifi_start(void) {
    ESP_ERROR_CHECK(esp_netif_init()); ESP_ERROR_CHECK(esp_event_loop_create_default()); esp_netif_create_default_wifi_ap();
    wifi_init_config_t cfg=WIFI_INIT_CONFIG_DEFAULT(); ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    wifi_config_t ap={.ap={.ssid=AP_SSID,.ssid_len=sizeof(AP_SSID)-1,.channel=6,.password=AP_PASS,.max_connection=4,.authmode=WIFI_AUTH_WPA2_PSK}};
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP)); ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP,&ap)); ESP_ERROR_CHECK(esp_wifi_start());
    httpd_config_t hc=HTTPD_DEFAULT_CONFIG(); hc.max_uri_handlers=12; hc.stack_size=6144; hc.uri_match_fn=httpd_uri_match_wildcard;
    httpd_handle_t server=NULL; ESP_ERROR_CHECK(httpd_start(&server,&hc));
    httpd_uri_t root={.uri="/",.method=HTTP_GET,.handler=root_get};
    httpd_uri_t logs={.uri="/logs",.method=HTTP_GET,.handler=logs_get};
    httpd_uri_t captive={.uri="/*",.method=HTTP_GET,.handler=captive_redirect};
    ESP_ERROR_CHECK(httpd_register_uri_handler(server,&root));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server,&logs));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server,&captive));
    xTaskCreate(dns_server_task, "captive_dns", 3072, NULL, 5, NULL);
    ESP_LOGI(TAG,"Wi-Fi AP: SSID=%s password=%s; captive portal http://192.168.4.1",AP_SSID,AP_PASS);
}
void wifi_log_init(void) {
    s_mx=xSemaphoreCreateMutex(); esp_log_set_vprintf(log_vprintf);
    if (WIFI_AP_ENABLED) wifi_start();
}
