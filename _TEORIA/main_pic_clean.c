/*
 * Balancín ESP32 - versión limpia basada en PICCODE.c
 *
 * - Mantiene el mapeo de pines actual del proyecto
 * - Mantiene el envío de telemetría por WiFi/UDP
 * - Usa la lógica de control del PICCODE
 * - Mantiene el lazo de seguimiento de línea separado para poder tunearlo
 */

#include <math.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "driver/i2c_master.h"
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "nvs_flash.h"
#include "lwip/sockets.h"
#include "lwip/inet.h"

/* ═══════════════════════════════════════════════════════════════
 *  CONFIG - PINES
 * ═══════════════════════════════════════════════════════════════ */

#define PIN_SDA         GPIO_NUM_21
#define PIN_SCL         GPIO_NUM_22

#define ADC_CH_IZQ      ADC_CHANNEL_7
#define ADC_CH_DER      ADC_CHANNEL_6

#define PIN_ENC_RA      GPIO_NUM_18
#define PIN_ENC_RB      GPIO_NUM_19
#define PIN_ENC_LA      GPIO_NUM_33
#define PIN_ENC_LB      GPIO_NUM_32

#define PIN_PWM_L       GPIO_NUM_14
#define PIN_MOT_L_IN1   GPIO_NUM_5
#define PIN_MOT_L_IN2   GPIO_NUM_4
#define PIN_MOT_R_IN1   GPIO_NUM_2
#define PIN_MOT_R_IN2   GPIO_NUM_15
#define PIN_PWM_R       GPIO_NUM_27

#define PIN_DEBUG       GPIO_NUM_26

#define PIN_DIR_L1      PIN_MOT_L_IN1
#define PIN_DIR_L2      PIN_MOT_L_IN2
#define PIN_DIR_R1      PIN_MOT_R_IN1
#define PIN_DIR_R2      PIN_MOT_R_IN2
#define PIN_SENSOR_L    ADC_CH_IZQ
#define PIN_SENSOR_R    ADC_CH_DER

/* ═══════════════════════════════════════════════════════════════
 *  CONFIG - PARÁMETROS FÍSICOS (igual que el balancín actual)
 * ═══════════════════════════════════════════════════════════════ */

#define R_RUEDA     0.035f
#define DIST_EJES   0.09f

#define RA          3.0f
#define KM          0.0008f
#define NR          34.014f
#define TAU_MAX     0.1654f
#define OMEGA_MAX   14.7f
#define U_MAX       11.0f

#define PPR         12.0f
#define ESC_ENC     0.003848f
#define ESC_PWM     23.18f

#define RASNKM      110.29f
#define NKM         0.0272f
#define V2TAUM      0.3308f

#define CALPHA      0.145f
#define ACCEL_F     0.0000610352f
#define GYRO_F      0.0076336f
#define DEG2RAD     0.0174533f

#define C1          0.995f
#define C_DERF      0.7f

/* ═══════════════════════════════════════════════════════════════
 *  CONFIG - GANANCIAS
 * ═══════════════════════════════════════════════════════════════ */

#define ALPHAD      0.0f
#define THETAD      0.0f

/* Ganancias ajustables en tiempo real via UDP (puerto CMD_UDP_PORT).
 * El monitor las envia como: 0xBB + id(1B) + float32_LE(4B) */
volatile float g_kpi  = 2.20f;
volatile float g_kdi  = 0.16f;
volatile float g_kpv  = 1.0f;
volatile float g_kiv  = 2.0f;
volatile float g_kpo  = 0.09f;
volatile float g_kdo  = 0.01f;
volatile float g_vd   = 0.1f;
volatile float g_ramp = 0.005f;  /* m/s por ciclo de 10ms → ~0.5 m/s² */

#define TS          0.01f
#define ITS         100.0f

/* ═══════════════════════════════════════════════════════════════
 *  CONFIG - TELEMETRÍA UART/WIFI
 * ═══════════════════════════════════════════════════════════════ */

#define CMD_UDP_PORT 5006
#define CMD_HEADER   0xBB
#define CMD_PKT_SIZE 6

#define UART_BAUD   115200
#define KT_V        637.5f
#define KT_TH       426.6f
#define KT_AL       91.07f
#define KT_OM       8.67f
#define KT_U        12.08f

#define WIFI_SSID        "WifiAlex"
#define WIFI_PASS        "Acetilena1702@@//"
#define UDP_DEST_IP      "192.168.43.71"
#define UDP_PORT         5006
#define WIFI_TIMEOUT_MS  15000

#define I2C_PORT        I2C_NUM_0
#define I2C_FREQ_HZ     100000

#define LEDC_TIMER_N    LEDC_TIMER_0
#define LEDC_MODE_N     LEDC_LOW_SPEED_MODE
#define LEDC_CH_L       LEDC_CHANNEL_0
#define LEDC_CH_R       LEDC_CHANNEL_1
#define LEDC_FREQ       5000

#define MPU_ADDR        0x68
#define REG_PWR_MGMT    0x6B
#define REG_CONFIG      0x1A
#define REG_GYRO_CFG    0x1B
#define REG_ACCEL_XH    0x3B
#define REG_GYRO_YH     0x45

static const char *TAG = "BALANCIN_PIC";

static EventGroupHandle_t wifi_eg;
#define WIFI_CONNECTED_BIT  BIT0
static volatile int udp_sock = -1;
static struct sockaddr_in udp_dest;

static i2c_master_dev_handle_t mpu_dev;
static adc_oneshot_unit_handle_t adc1_handle;

static portMUX_TYPE enc_mux = portMUX_INITIALIZER_UNLOCKED;
static volatile int32_t enc_r = 0;
static volatile int32_t enc_l = 0;
static volatile uint8_t ra_p = 0, rb_p = 0;
static volatile uint8_t la_p = 0, lb_p = 0;

static TaskHandle_t ctrl_task_handle = NULL;

static void IRAM_ATTR enc_isr(void *arg)
{
    uint8_t ra = (uint8_t)gpio_get_level(PIN_ENC_RA);
    uint8_t rb = (uint8_t)gpio_get_level(PIN_ENC_RB);
    uint8_t la = (uint8_t)gpio_get_level(PIN_ENC_LA);
    uint8_t lb = (uint8_t)gpio_get_level(PIN_ENC_LB);

    portENTER_CRITICAL_ISR(&enc_mux);

    uint8_t ab = (ra << 1) | rb;
    uint8_t ab_1 = (ra_p << 1) | rb_p;
    uint8_t aux = ab ^ ab_1;
    if (aux != 0 && aux != 3) {
        if (((ab_1 << 1) ^ ab) & 0x02) enc_r--;
        else enc_r++;
    }
    ra_p = ra;
    rb_p = rb;

    uint8_t cd = (la << 1) | lb;
    uint8_t cd_1 = (la_p << 1) | lb_p;
    aux = cd ^ cd_1;
    if (aux != 0 && aux != 3) {
        if (((cd_1 << 1) ^ cd) & 0x02) enc_l--;
        else enc_l++;
    }
    la_p = la;
    lb_p = lb;

    portEXIT_CRITICAL_ISR(&enc_mux);
}

static void timer_cb(void *arg)
{
    xTaskNotifyGive(ctrl_task_handle);
}

static void mpu_write(uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = { reg, val };
    i2c_master_transmit(mpu_dev, buf, 2, 100);
}

static void mpu_read(uint8_t reg, uint8_t *out, size_t n)
{
    i2c_master_transmit_receive(mpu_dev, &reg, 1, out, n, 100);
}

static void mpu_init(void)
{
    mpu_write(REG_PWR_MGMT, 0x80);
    vTaskDelay(pdMS_TO_TICKS(100));
    mpu_write(REG_PWR_MGMT, 0x00);
    vTaskDelay(pdMS_TO_TICKS(100));
    mpu_write(REG_CONFIG, 0x01);
    vTaskDelay(pdMS_TO_TICKS(10));
    mpu_write(REG_GYRO_CFG, 0x00);
}

static void motor_set(gpio_num_t in1, gpio_num_t in2,
                      ledc_channel_t ch, float u)
{
    uint32_t duty = (uint32_t)(ESC_PWM * fabsf(u));
    if (duty > 255U) duty = 255U;

    gpio_set_level(in1, (u < 0.0f) ? 1 : 0);
    gpio_set_level(in2, (u < 0.0f) ? 0 : 1);

    ledc_set_duty(LEDC_MODE_N, ch, duty);
    ledc_update_duty(LEDC_MODE_N, ch);
}

static float line_follow_theta(float sl, float sr)
{
    float line_err = sr - sl;
    if (line_err > 220.0f) line_err = 220.0f;
    if (line_err < -220.0f) line_err = -220.0f;

    return -0.18f * line_err / 220.0f;
}

static void wifi_event_cb(void *arg, esp_event_base_t base,
                          int32_t id, void *data)
{
    if (base == WIFI_EVENT) {
        if (id == WIFI_EVENT_STA_START) {
            esp_wifi_connect();
        } else if (id == WIFI_EVENT_STA_DISCONNECTED) {
            esp_wifi_connect();
        }
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *ev = (ip_event_got_ip_t *)data;
        ESP_LOGI(TAG, "WiFi OK - IP: " IPSTR, IP2STR(&ev->ip_info.ip));
        xEventGroupSetBits(wifi_eg, WIFI_CONNECTED_BIT);
    }
}

static void wifi_init(void)
{
    wifi_eg = xEventGroupCreate();

    nvs_flash_init();
    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);

    esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_event_cb, NULL);
    esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, wifi_event_cb, NULL);

    wifi_config_t wc = {};
    strncpy((char *)wc.sta.ssid, WIFI_SSID, sizeof(wc.sta.ssid) - 1);
    strncpy((char *)wc.sta.password, WIFI_PASS, sizeof(wc.sta.password) - 1);

    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_set_config(WIFI_IF_STA, &wc);
    esp_wifi_start();

    xEventGroupWaitBits(wifi_eg, WIFI_CONNECTED_BIT,
                        false, true, pdMS_TO_TICKS(WIFI_TIMEOUT_MS));
}

static void udp_init(void)
{
    if (!(xEventGroupGetBits(wifi_eg) & WIFI_CONNECTED_BIT)) {
        ESP_LOGW(TAG, "WiFi sin conexion - UDP desactivado");
        return;
    }

    int s = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (s < 0) {
        ESP_LOGE(TAG, "No se pudo crear socket UDP");
        return;
    }

    memset(&udp_dest, 0, sizeof(udp_dest));
    udp_dest.sin_family = AF_INET;
    udp_dest.sin_port = htons(UDP_PORT);
    inet_aton(UDP_DEST_IP, &udp_dest.sin_addr);

    udp_sock = s;
    ESP_LOGI(TAG, "UDP listo -> %s:%d", UDP_DEST_IP, UDP_PORT);
}

static void hw_init(void)
{
    i2c_master_bus_config_t i2c_bus_cfg = {
        .i2c_port = I2C_PORT,
        .sda_io_num = PIN_SDA,
        .scl_io_num = PIN_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    i2c_master_bus_handle_t i2c_bus;
    i2c_new_master_bus(&i2c_bus_cfg, &i2c_bus);

    i2c_device_config_t mpu_dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = MPU_ADDR,
        .scl_speed_hz = I2C_FREQ_HZ,
    };
    i2c_master_bus_add_device(i2c_bus, &mpu_dev_cfg, &mpu_dev);

    ledc_timer_config_t lt = {
        .speed_mode = LEDC_MODE_N,
        .timer_num = LEDC_TIMER_N,
        .duty_resolution = LEDC_TIMER_8_BIT,
        .freq_hz = LEDC_FREQ,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ledc_timer_config(&lt);

    ledc_channel_config_t lc = {
        .speed_mode = LEDC_MODE_N,
        .timer_sel = LEDC_TIMER_N,
        .duty = 0,
        .hpoint = 0,
    };
    lc.gpio_num = PIN_PWM_L;
    lc.channel = LEDC_CH_L;
    ledc_channel_config(&lc);
    lc.gpio_num = PIN_PWM_R;
    lc.channel = LEDC_CH_R;
    ledc_channel_config(&lc);

    gpio_config_t go = {
        .pin_bit_mask = (1ULL << PIN_MOT_L_IN1) | (1ULL << PIN_MOT_L_IN2) |
                        (1ULL << PIN_MOT_R_IN1) | (1ULL << PIN_MOT_R_IN2) |
                        (1ULL << PIN_DEBUG),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&go);
    gpio_set_level(PIN_DEBUG, 0);

    gpio_config_t gi = {
        .pin_bit_mask = (1ULL << PIN_ENC_RA) | (1ULL << PIN_ENC_RB) |
                        (1ULL << PIN_ENC_LA) | (1ULL << PIN_ENC_LB),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_ANYEDGE,
    };
    gpio_config(&gi);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(PIN_ENC_RA, enc_isr, NULL);
    gpio_isr_handler_add(PIN_ENC_RB, enc_isr, NULL);
    gpio_isr_handler_add(PIN_ENC_LA, enc_isr, NULL);
    gpio_isr_handler_add(PIN_ENC_LB, enc_isr, NULL);
    ra_p = (uint8_t)gpio_get_level(PIN_ENC_RA);
    rb_p = (uint8_t)gpio_get_level(PIN_ENC_RB);
    la_p = (uint8_t)gpio_get_level(PIN_ENC_LA);
    lb_p = (uint8_t)gpio_get_level(PIN_ENC_LB);

    adc_oneshot_unit_init_cfg_t adc_unit_cfg = { .unit_id = ADC_UNIT_1 };
    adc_oneshot_new_unit(&adc_unit_cfg, &adc1_handle);

    adc_oneshot_chan_cfg_t adc_ch_cfg = {
        .bitwidth = ADC_BITWIDTH_12,
        .atten = ADC_ATTEN_DB_12,
    };
    adc_oneshot_config_channel(adc1_handle, ADC_CH_IZQ, &adc_ch_cfg);
    adc_oneshot_config_channel(adc1_handle, ADC_CH_DER, &adc_ch_cfg);
}

static void control_task(void *arg)
{
    float angulox_1 = 0.0f;
    float c2 = 1.0f - C1;
    float ei_l = 0.0f;
    float eip_f = 0.0f;
    float intev = 0.0f;
    float vd_r = 0.0f;
    float theta_f = 0.0f;
    uint8_t raw[2];
    uint32_t log_cnt = 0;

    while (1) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        gpio_set_level(PIN_DEBUG, 1);

        int raw_izq = 0;
        int raw_der = 0;
        adc_oneshot_read(adc1_handle, ADC_CH_IZQ, &raw_izq);
        adc_oneshot_read(adc1_handle, ADC_CH_DER, &raw_der);
        float Sl = raw_izq * (255.0f / 4095.0f);
        float Sr = raw_der * (255.0f / 4095.0f);

        mpu_read(REG_ACCEL_XH, raw, 2);
        int16_t Ax = (int16_t)((raw[0] << 8) | raw[1]);

        mpu_read(REG_GYRO_YH, raw, 2);
        int16_t Gy = (int16_t)((raw[0] << 8) | raw[1]);

        portENTER_CRITICAL(&enc_mux);
        int32_t cnt_r = enc_r;
        enc_r = 0;
        int32_t cnt_l = enc_l;
        enc_l = 0;
        portEXIT_CRITICAL(&enc_mux);

        /* filtro pasa-bajas en theta: suaviza los saltos bruscos al cruzar el borde
         * de la línea para no sacudir el lazo de balance (τ ≈ 35 ms a 100 Hz) */
        theta_f = 0.75f * theta_f + 0.25f * line_follow_theta(Sl, Sr);
        float theta = theta_f;

        /* enc_l físicamente en rueda derecha (+fwd), enc_r en rueda izq (invertido) */
        float omegar = (float)cnt_l * ESC_ENC * ITS;
        float omegal = -(float)cnt_r * ESC_ENC * ITS;

        float Xa = (float)Ax * ACCEL_F;
        float Yg = (float)Gy * GYRO_F * DEG2RAD;
        float accelx = Xa * 1.570796f;
        float angulox = C1 * (angulox_1 + Yg * TS) + c2 * accelx;
        angulox_1 = angulox;
        float alpha = angulox - CALPHA;

        float eop = 0.0f;
        float eo = 0.0f;
        float v = (omegar + omegal) * R_RUEDA / 2.0f;
        float thetap = (omegar - omegal) * R_RUEDA / (2.0f * DIST_EJES);
        eop = -thetap;
        eo = THETAD - theta;

        /* rampa de velocidad: evita arrancones bruscos */
        {
            float vd_target = g_vd;
            float rate = g_ramp;
            if (vd_r < vd_target) {
                vd_r += rate;
                if (vd_r > vd_target) vd_r = vd_target;
            } else if (vd_r > vd_target) {
                vd_r -= rate;
                if (vd_r < vd_target) vd_r = vd_target;
            }
        }
        float ev = vd_r - v;

        float ei = ALPHAD - alpha;
        float eip_raw = (ei - ei_l) * ITS;
        eip_f = C_DERF * eip_f + (1.0f - C_DERF) * eip_raw;
        ei_l = ei;

        if (intev < V2TAUM && intev > -V2TAUM) {
            intev += TS * ev;
        }

        float taua = (g_kdo * eop + g_kpo * eo) * 2.0f * DIST_EJES / R_RUEDA;
        float u = -g_kpi * ei - g_kdi * eip_f - g_kpv * ev - g_kiv * intev;
        if (u > V2TAUM) u = V2TAUM;
        if (u < -V2TAUM) u = -V2TAUM;

        float taur = (taua + u) / 2.0f;
        float taul = (-taua + u) / 2.0f;

        float ul = taul * RASNKM + NKM * omegal;
        if (ul > U_MAX) ul = U_MAX;
        if (ul < -U_MAX) ul = -U_MAX;

        float ur = taur * RASNKM + NKM * omegar;
        if (ur > U_MAX) ur = U_MAX;
        if (ur < -U_MAX) ur = -U_MAX;

        motor_set(PIN_MOT_L_IN1, PIN_MOT_L_IN2, LEDC_CH_L, -ul);
        motor_set(PIN_MOT_R_IN1, PIN_MOT_R_IN2, LEDC_CH_R, ur);

        uint8_t sl_byte = (Sl > 255.0f) ? 255u : (Sl < 0.0f) ? 0u : (uint8_t)Sl;
        uint8_t sr_byte = (Sr > 255.0f) ? 255u : (Sr < 0.0f) ? 0u : (uint8_t)Sr;
        uint8_t pkt[12] = {
            0xAA,
            (uint8_t)(KT_V * vd_r + 127.0f),
            (uint8_t)(KT_V * v + 127.0f),
            (uint8_t)(KT_TH * theta + 128.0f),
            (uint8_t)(KT_AL * alpha + 127.0f),
            (uint8_t)(KT_OM * omegal + 127.0f),
            (uint8_t)(KT_OM * omegar + 127.0f),
            (uint8_t)(KT_U * ul + 127.0f),
            (uint8_t)(KT_U * ur + 127.0f),
            sl_byte,
            sr_byte,
            0x00u,
        };

        int s = udp_sock;
        if (s >= 0) {
            sendto(s, pkt, sizeof(pkt), 0,
                   (struct sockaddr *)&udp_dest, sizeof(udp_dest));
        }

        if (++log_cnt >= 50) {
            log_cnt = 0;
            ESP_LOGI(TAG, "al=%.3f v=%.3f wl=%.2f wr=%.2f ul=%.2f ur=%.2f Sl=%.0f Sr=%.0f",
                     alpha, v, omegal, omegar, ul, ur, Sl, Sr);
        }

        gpio_set_level(PIN_DEBUG, 0);
    }
}

static void cmd_task(void *arg)
{
    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock < 0) { vTaskDelete(NULL); return; }

    struct sockaddr_in addr = {
        .sin_family      = AF_INET,
        .sin_port        = htons(CMD_UDP_PORT),
        .sin_addr.s_addr = INADDR_ANY,
    };
    bind(sock, (struct sockaddr *)&addr, sizeof(addr));

    uint8_t buf[CMD_PKT_SIZE];
    while (1) {
        int n = recv(sock, buf, sizeof(buf), 0);
        if (n != CMD_PKT_SIZE || buf[0] != CMD_HEADER) continue;
        float val;
        memcpy(&val, &buf[2], 4);
        switch (buf[1]) {
            case 0: g_kpi = val; break;
            case 1: g_kdi = val; break;
            case 2: g_kpv = val; break;
            case 3: g_kiv = val; break;
            case 4: g_kpo = val; break;
            case 5: g_kdo = val; break;
            case 6: g_vd   = val; break;
            case 7: g_ramp = val; break;
        }
        ESP_LOGI(TAG, "CMD gain[%d]=%.4f", buf[1], val);
    }
    close(sock);
    vTaskDelete(NULL);
}

void app_main(void)
{
    hw_init();
    mpu_init();

    xTaskCreatePinnedToCore(control_task, "control", 4096, NULL,
                            configMAX_PRIORITIES - 1, &ctrl_task_handle, 1);

    esp_timer_handle_t tmr;
    const esp_timer_create_args_t ta = {
        .callback = timer_cb,
        .name = "ctrl_10ms",
    };
    esp_timer_create(&ta, &tmr);
    esp_timer_start_periodic(tmr, 10000);

    wifi_init();
    udp_init();
    xTaskCreate(cmd_task, "cmd", 3072, NULL, 5, NULL);
}
