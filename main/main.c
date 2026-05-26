/*
 * PISDRSL — Péndulo Invertido Seguidor de Línea
 * ESP32 + FreeRTOS
 * Universidad Autónoma de Querétaro — Ing. Automatización
 *
 * FUENTES:
 *   Control  → embebidoRVCSclas.c (PIC18F4550)  — variables físicas, filtro, ganancias
 *   Pines    → main_pic_clean.c                  — verificados en el hardware real
 *   Telemetría → UDP binario v2 0xAB + float32 + ADC crudo 12-bit
 *
 * ARQUITECTURA FreeRTOS:
 *   Core 1 — control_task  (prioridad máxima, 100 Hz via esp_timer)
 *   Core 0 — cmd_task      (recibe ganancias por UDP en tiempo real, puerto 5006)
 *   Timer  — esp_timer ISR (10 000 µs → vTaskNotifyGiveFromISR → control_task)
 *
 * TELEMETRÍA (cada ciclo de 10 ms):
 *   Paquete binario v2: 0xAB + float32 + ADC12 → UDP broadcast 255.255.255.255:5005
 *   Abrir monitor: python monitor/monitor.py
 *
 * AJUSTE EN VIVO (sin recompilar):
 *   Enviar UDP a ESP32:5006 → 0xBB + id(1B) + float32_LE(4B)
 *   id: 0=kpi 1=kdi 2=kpv 3=kiv 4=kpo 5=kdo 6=vd 7=ramp 8=alphad
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

static const char *TAG = "PISDRSL";

/* ============================================================================
   SECCIÓN 1 — MAPEO DE PINES (verificado físicamente en el robot)
   ============================================================================ */
#define PIN_SDA         GPIO_NUM_21
#define PIN_SCL         GPIO_NUM_22

/* Sensores de línea TCRT5000 (ADC1) */
#define ADC_CH_IZQ      ADC_CHANNEL_6   /* GPIO34 */
#define ADC_CH_DER      ADC_CHANNEL_7   /* GPIO35 */

/* Encoders de cuadratura — ISR en flanco cualquiera */
#define PIN_ENC_RA      GPIO_NUM_18
#define PIN_ENC_RB      GPIO_NUM_19
#define PIN_ENC_LA      GPIO_NUM_33
#define PIN_ENC_LB      GPIO_NUM_32

/* L298N — motor izquierdo: PWM=GPIO14, IN1=GPIO5, IN2=GPIO4
 *        — motor derecho:  PWM=GPIO27, IN1=GPIO2, IN2=GPIO15
 * NOTA: enc_l (pines LA/LB) es físicamente la rueda DERECHA.
 *       enc_r (pines RA/RB) es físicamente la rueda IZQUIERDA (negada). */
#define PIN_PWM_L       GPIO_NUM_14
#define PIN_MOT_L_IN1   GPIO_NUM_5
#define PIN_MOT_L_IN2   GPIO_NUM_4
#define PIN_PWM_R       GPIO_NUM_27
#define PIN_MOT_R_IN1   GPIO_NUM_2
#define PIN_MOT_R_IN2   GPIO_NUM_15

/* Pin de debug — medir tiempo de ejecución del lazo con osciloscopio */
#define PIN_DEBUG       GPIO_NUM_26

/* ============================================================================
   SECCIÓN 2 — PARÁMETROS FÍSICOS (del código PIC embebidoRVCSclas.c)
   ============================================================================ */
#define PI_         3.141593f
#define PI_S2       1.570796f       /* pi/2 */
#define RAD2DEG     57.29578f
#define DEG2RAD     0.0174533f
#define TS          0.01f           /* Periodo de muestreo: 10 ms */
#define ITS         100.0f          /* 1/Ts */

/* Motor y transmisión */
#define PPR         12.0f           /* Pulsos por revolución del encoder */
#define NR          34.014f           /* Relación de transmisión */
#define R_W         0.035f          /* Radio de rueda [m] */
#define RA_MOT      3.0f            /* Resistencia del motor [Ohm] */
#define KM_MOT      0.0008f         /* Constante de par [Nm/A] */
#define B_H         0.09f         /* Semidistancia entre ruedas [m] */

/* IMU y ángulo */
#define CALPHA      0.145f           /* Compensación de alineación del MPU [rad] */
#define C1          0.70f          /* Constante filtro complementario ajustable */
#define C2          (1.0f - C1)
#define ACCEL_F     0.0000610352f   /* Escala acelerómetro (LSB→rango ±1) */
#define GYRO_F      0.0076336f       /* Escala giroscopio (LSB→°/s) — ±250°/s: 1/131 */

/* Sensores de línea: usar ADC12 nativo, no escala heredada de 8 bits */
#define ADC12_MAX        4095.0f
#define LINE_THETA_MAX   0.3f
#define LINE_ERR_MAX_ADC (160.0f * ADC12_MAX / 255.0f)

/* Límites */
#define TAU_MAX     0.1654f            /* Par máximo en rueda [Nm] */
#define ALPHA_MAX   1.4f            /* Ángulo máximo antes de declarar caída [rad] */
#define OMEGA_MAX   44.0f           /* Velocidad angular máxima de rueda [rad/s] */
#define U_M         11.0f          /* Voltaje máximo para escala PWM [V] */
#define U_NM        11.0f           /* Voltaje máximo a los motores [V] */

/* Constantes derivadas (mismas fórmulas que PIC) */
#define ESC_ENC     (PI_ / (2.0f * PPR * NR))  /* rad por pulso encoder */
#define RASNKM      (RA_MOT / (KM_MOT * NR))   /* Ra/(km·NR) para back-EMF */
#define NKM         (KM_MOT * NR)              /* km·NR para back-EMF */
#define PWM_RES_BITS     13U
#define PWM_DUTY_MAX     ((1U << PWM_RES_BITS) - 1U)
#define ESC_PWM          ((float)PWM_DUTY_MAX / U_M)  /* Escala voltaje→duty */
#define V2TAUM      (2.0f * TAU_MAX)           /* Límite de saturación de u */

/* ============================================================================
   SECCIÓN 3 — REFERENCIAS Y GANANCIAS (del código PIC)
   Ajustables en tiempo real via UDP sin recompilar (cmd_task, puerto 5006).
   ============================================================================ */
#define ALPHAD  0.0f    /* Referencia de inclinación [rad] */
#define THETAD  0.0f    /* Referencia de orientación [rad] */

volatile float g_kpi   = 0.0f;    /* Ganancia proporcional balance */
volatile float g_kdi   = 0.0f;    /* Ganancia derivativa balance */
volatile float g_kpv   = 0.0f;    /* Ganancia proporcional velocidad */
volatile float g_kiv   = 0.0f;    /* Ganancia integral velocidad */
volatile float g_kpo   = 0.0f;    /* Ganancia proporcional orientación */
volatile float g_kdo   = 0.0f;    /* Ganancia derivativa orientación */
volatile float g_vd    = 0.0f;    /* Velocidad deseada [m/s] — empezar en 0 para calibrar */
volatile float g_ramp  = 0.0f;    /* Rampa de velocidad [m/s por ciclo de 10ms] */
volatile float g_alphad = 0.0f;   /* Referencia de inclinación [rad] — ajustar para calibrar balance */

/* ============================================================================
   SECCIÓN 4 — TELEMETRÍA
   Paquete v2: 0xAB + version + seq + 8 float32 + 2 ADC12 + flags.
   Monitor: python monitor/monitor.py → "Escuchar"
   ============================================================================ */
#define TELEM_HEADER    0xABu
#define TELEM_VERSION   2u
#define TELEM_FLAG_SOL  0x01u

typedef struct __attribute__((packed)) {
    uint8_t  header;
    uint8_t  version;
    uint16_t seq;
    float    vd;
    float    v;
    float    theta;
    float    alpha;
    float    omegal;
    float    omegar;
    float    ul;
    float    ur;
    uint16_t sl_adc;
    uint16_t sr_adc;
    uint8_t  flags;
} telemetry_pkt_t;

_Static_assert(sizeof(telemetry_pkt_t) == 41, "telemetry_pkt_t debe medir 41 bytes");

/* ============================================================================
   SECCIÓN 5 — CONFIGURACIÓN WIFI Y UDP
   ============================================================================ */
#define WIFI_SSID        "WifiAlex"
#define WIFI_PASS        "Acetilena1702@@//"
#define UDP_DEST_IP      "192.168.43.101"  /* Broadcast: no necesita IP de la PC */
#define UDP_PORT         5005               /* Puerto que escucha monitor.py */
#define CMD_UDP_PORT     5006               /* Puerto para recibir ganancias desde PC */
#define WIFI_TIMEOUT_MS  15000 

/* ============================================================================
   SECCIÓN 6 — CONFIGURACIÓN DE PERIFÉRICOS
   ============================================================================ */
#define I2C_PORT        I2C_NUM_0
#define I2C_FREQ_HZ     400000          /* 400 kHz: lectura MPU más rápida */

#define LEDC_TIMER_N    LEDC_TIMER_0
#define LEDC_MODE_N     LEDC_LOW_SPEED_MODE
#define LEDC_CH_L       LEDC_CHANNEL_0
#define LEDC_CH_R       LEDC_CHANNEL_1
#define LEDC_FREQ_HZ    5000
#define LEDC_RES        LEDC_TIMER_13_BIT  /* 13-bit: 0–8191 a 5 kHz */

/* Registros MPU6050 */
#define MPU_ADDR        0x68
#define REG_PWR_MGMT    0x6B
#define REG_CONFIG_R    0x1A
#define REG_GYRO_CFG    0x1B
#define REG_ACCEL_XH    0x3B    /* Acelerómetro X (High byte) */
#define MPU_FRAME_LEN   14      /* ACCEL(6) + TEMP(2) + GYRO(6) desde 0x3B */
#define MPU_GYRO_Y_IDX  10      /* Índice de GYRO_YH dentro del frame */

/* ============================================================================
   SECCIÓN 7 — VARIABLES GLOBALES
   ============================================================================ */
static EventGroupHandle_t   wifi_eg;
#define WIFI_CONNECTED_BIT  BIT0

static volatile int         udp_sock = -1;
static struct sockaddr_in   udp_dest;

static i2c_master_dev_handle_t   mpu_dev;
static adc_oneshot_unit_handle_t adc1_handle;

/* Encoders: acceso desde ISR y tarea de control → sección crítica */
static portMUX_TYPE     enc_mux = portMUX_INITIALIZER_UNLOCKED;
static volatile int32_t enc_r = 0;     /* Cuentas encoder RA/RB (rueda física izquierda) */
static volatile int32_t enc_l = 0;     /* Cuentas encoder LA/LB (rueda física derecha) */
static volatile uint8_t ra_p = 0, rb_p = 0;
static volatile uint8_t la_p = 0, lb_p = 0;

static TaskHandle_t ctrl_task_handle = NULL;

/* ============================================================================
   SECCIÓN 8 — ISR DE ENCODERS (lógica idéntica al PIC: int_rb)
   Decodificación cuadratura 4× en todos los flancos.
   ============================================================================ */
static void IRAM_ATTR enc_isr(void *arg)
{
    uint8_t ra = (uint8_t)gpio_get_level(PIN_ENC_RA);
    uint8_t rb = (uint8_t)gpio_get_level(PIN_ENC_RB);
    uint8_t la = (uint8_t)gpio_get_level(PIN_ENC_LA);
    uint8_t lb = (uint8_t)gpio_get_level(PIN_ENC_LB);

    portENTER_CRITICAL_ISR(&enc_mux);

    /* Encoder derecho (pines RA/RB) — físicamente rueda izquierda */
    uint8_t ab   = (ra << 1) | rb;
    uint8_t ab_1 = (ra_p << 1) | rb_p;
    uint8_t aux  = ab ^ ab_1;
    if (aux != 0 && aux != 3) {
        if (((ab_1 << 1) ^ ab) & 0x02) enc_r--;
        else                            enc_r++;
    }
    ra_p = ra; rb_p = rb;

    /* Encoder izquierdo (pines LA/LB) — físicamente rueda derecha */
    uint8_t cd   = (la << 1) | lb;
    uint8_t cd_1 = (la_p << 1) | lb_p;
    aux = cd ^ cd_1;
    if (aux != 0 && aux != 3) {
        if (((cd_1 << 1) ^ cd) & 0x02) enc_l--;
        else                            enc_l++;
    }
    la_p = la; lb_p = lb;

    portEXIT_CRITICAL_ISR(&enc_mux);
}

/* ============================================================================
   SECCIÓN 9 — TIMER: dispara control_task a exactamente 100 Hz
   ============================================================================ */
static void timer_cb(void *arg)
{
    BaseType_t hp_task_woken = pdFALSE;
    vTaskNotifyGiveFromISR(ctrl_task_handle, &hp_task_woken);
    if (hp_task_woken == pdTRUE) {
        esp_timer_isr_dispatch_need_yield();
    }
}

/* ============================================================================
   SECCIÓN 10 — I2C: helpers para MPU6050
   ============================================================================ */
static void mpu_write(uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = {reg, val};
    i2c_master_transmit(mpu_dev, buf, 2, 100);
}

static void mpu_read_regs(uint8_t reg, uint8_t *out, size_t n)
{
    i2c_master_transmit_receive(mpu_dev, &reg, 1, out, n, 100);
}

/* ============================================================================
   SECCIÓN 11 — ACTUADORES: motor con PWM de alta resolución y H-bridge
   ============================================================================ */
static void motor_set(gpio_num_t in1, gpio_num_t in2,
                      ledc_channel_t ch, float u)
{
    uint32_t duty = (uint32_t)(ESC_PWM * fabsf(u));
    if (duty > PWM_DUTY_MAX) duty = PWM_DUTY_MAX;

    /* Dirección según signo de u */
    gpio_set_level(in1, (u < 0.0f) ? 1 : 0);
    gpio_set_level(in2, (u < 0.0f) ? 0 : 1);

    ledc_set_duty(LEDC_MODE_N, ch, duty);
    ledc_update_duty(LEDC_MODE_N, ch);
}

static void motors_off(void)
{
    motor_set(PIN_MOT_L_IN1, PIN_MOT_L_IN2, LEDC_CH_L, 0.0f);
    motor_set(PIN_MOT_R_IN1, PIN_MOT_R_IN2, LEDC_CH_R, 0.0f);
}

/* ============================================================================
   SECCIÓN 12 — INICIALIZACIÓN DE HARDWARE
   ============================================================================ */
static void hw_init(void)
{
    /* ── I2C ──────────────────────────────────────────────────────────────── */
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port            = I2C_PORT,
        .sda_io_num          = PIN_SDA,
        .scl_io_num          = PIN_SCL,
        .clk_source          = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt   = 7,
        .flags.enable_internal_pullup = true,
    };
    i2c_master_bus_handle_t i2c_bus;
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &i2c_bus));

    i2c_device_config_t mpu_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = MPU_ADDR,
        .scl_speed_hz    = I2C_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(i2c_bus, &mpu_cfg, &mpu_dev));

    /* ── LEDC PWM de alta resolución ──────────────────────────────────────── */
    ledc_timer_config_t lt = {
        .speed_mode      = LEDC_MODE_N,
        .timer_num       = LEDC_TIMER_N,
        .duty_resolution = LEDC_RES,
        .freq_hz         = LEDC_FREQ_HZ,
        .clk_cfg         = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&lt));

    ledc_channel_config_t lc = {
        .speed_mode = LEDC_MODE_N,
        .timer_sel  = LEDC_TIMER_N,
        .duty       = 0,
        .hpoint     = 0,
    };
    lc.gpio_num = PIN_PWM_L; lc.channel = LEDC_CH_L;
    ESP_ERROR_CHECK(ledc_channel_config(&lc));
    lc.gpio_num = PIN_PWM_R; lc.channel = LEDC_CH_R;
    ESP_ERROR_CHECK(ledc_channel_config(&lc));

    /* ── GPIO salidas (dirección motores + debug) ─────────────────────────── */
    gpio_config_t go = {
        .pin_bit_mask = (1ULL << PIN_MOT_L_IN1) | (1ULL << PIN_MOT_L_IN2) |
                        (1ULL << PIN_MOT_R_IN1) | (1ULL << PIN_MOT_R_IN2) |
                        (1ULL << PIN_DEBUG),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&go));
    gpio_set_level(PIN_DEBUG, 0);

    /* ── GPIO entradas encoders con interrupción en cualquier flanco ─────── */
    gpio_config_t gi = {
        .pin_bit_mask = (1ULL << PIN_ENC_RA) | (1ULL << PIN_ENC_RB) |
                        (1ULL << PIN_ENC_LA) | (1ULL << PIN_ENC_LB),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_ANYEDGE,
    };
    ESP_ERROR_CHECK(gpio_config(&gi));

    gpio_install_isr_service(0);
    gpio_isr_handler_add(PIN_ENC_RA, enc_isr, NULL);
    gpio_isr_handler_add(PIN_ENC_RB, enc_isr, NULL);
    gpio_isr_handler_add(PIN_ENC_LA, enc_isr, NULL);
    gpio_isr_handler_add(PIN_ENC_LB, enc_isr, NULL);

    /* Leer estado inicial para no contar el primer flanco como movimiento */
    ra_p = (uint8_t)gpio_get_level(PIN_ENC_RA);
    rb_p = (uint8_t)gpio_get_level(PIN_ENC_RB);
    la_p = (uint8_t)gpio_get_level(PIN_ENC_LA);
    lb_p = (uint8_t)gpio_get_level(PIN_ENC_LB);

    /* ── ADC (sensores de línea TCRT5000) ────────────────────────────────── */
    adc_oneshot_unit_init_cfg_t adc_ucfg = { .unit_id = ADC_UNIT_1 };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&adc_ucfg, &adc1_handle));

    adc_oneshot_chan_cfg_t adc_ccfg = {
        .bitwidth = ADC_BITWIDTH_12,
        .atten    = ADC_ATTEN_DB_12,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC_CH_IZQ, &adc_ccfg));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC_CH_DER, &adc_ccfg));

    ESP_LOGI(TAG, "Hardware OK");
}

static void mpu_init(void)
{
    /* Secuencia de inicio igual a MPU6050_init() del PIC */
    mpu_write(REG_PWR_MGMT, 0x80);       /* Reset total */
    vTaskDelay(pdMS_TO_TICKS(100));
    mpu_write(REG_PWR_MGMT, 0x00);       /* Wake-up */
    vTaskDelay(pdMS_TO_TICKS(100));
    mpu_write(REG_CONFIG_R, 0x01);       /* DLPF 188 Hz */
    vTaskDelay(pdMS_TO_TICKS(10));
    mpu_write(REG_GYRO_CFG, 0x00);       /* Giroscopio ±250°/s — coincide con GYRO_F y el PIC */
    vTaskDelay(pdMS_TO_TICKS(10));
    ESP_LOGI(TAG, "MPU6050 OK");
}

/* ============================================================================
   SECCIÓN 13 — WIFI
   ============================================================================ */
static void wifi_event_cb(void *arg, esp_event_base_t base,
                          int32_t id, void *data)
{
    if (base == WIFI_EVENT) {
        if (id == WIFI_EVENT_STA_START)          esp_wifi_connect();
        else if (id == WIFI_EVENT_STA_DISCONNECTED) esp_wifi_connect();
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *ev = (ip_event_got_ip_t *)data;
        ESP_LOGI(TAG, "WiFi OK — IP: " IPSTR, IP2STR(&ev->ip_info.ip));
        xEventGroupSetBits(wifi_eg, WIFI_CONNECTED_BIT);
    }
}

static void wifi_init(void)
{
    wifi_eg = xEventGroupCreate();
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                wifi_event_cb, NULL);
    esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                wifi_event_cb, NULL);

    wifi_config_t wc = {};
    strncpy((char *)wc.sta.ssid,     WIFI_SSID, sizeof(wc.sta.ssid) - 1);
    strncpy((char *)wc.sta.password, WIFI_PASS,  sizeof(wc.sta.password) - 1);
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wc));
    ESP_ERROR_CHECK(esp_wifi_start());

    xEventGroupWaitBits(wifi_eg, WIFI_CONNECTED_BIT, false, true,
                        pdMS_TO_TICKS(WIFI_TIMEOUT_MS));
}

static void udp_init(void)
{
    if (!(xEventGroupGetBits(wifi_eg) & WIFI_CONNECTED_BIT)) {
        ESP_LOGW(TAG, "WiFi sin conexión — UDP desactivado");
        return;
    }

    int s = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (s < 0) { ESP_LOGE(TAG, "Error creando socket UDP"); return; }

    /* Habilitar broadcast (necesario para 255.255.255.255) */
    int bcast = 1;
    setsockopt(s, SOL_SOCKET, SO_BROADCAST, &bcast, sizeof(bcast));

    memset(&udp_dest, 0, sizeof(udp_dest));
    udp_dest.sin_family      = AF_INET;
    udp_dest.sin_port        = htons(UDP_PORT);
    udp_dest.sin_addr.s_addr = inet_addr(UDP_DEST_IP);

    udp_sock = s;
    ESP_LOGI(TAG, "UDP listo → %s:%d", UDP_DEST_IP, UDP_PORT);
}

/* ============================================================================
   SECCIÓN 14 — TAREA DE RECEPCIÓN DE GANANCIAS (Core 0, prio baja)
   Protocolo: 0xBB + id(1B) + float32_LE(4B) = 6 bytes
   Permite tuning en tiempo real desde el monitor o cualquier tool UDP.
   ============================================================================ */
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

    uint8_t buf[6];
    while (1) {
        int n = recv(sock, buf, sizeof(buf), 0);
        if (n != 6 || buf[0] != 0xBB) continue;

        float val;
        memcpy(&val, &buf[2], 4);

        switch (buf[1]) {
            case 0: g_kpi  = val; break;
            case 1: g_kdi  = val; break;
            case 2: g_kpv  = val; break;
            case 3: g_kiv  = val; break;
            case 4: g_kpo  = val; break;
            case 5: g_kdo   = val; break;
            case 6: g_vd    = val; break;
            case 7: g_ramp  = val; break;
            case 8: g_alphad = val; break;
        }
        ESP_LOGI(TAG, "Ganancia[%d] = %.4f", buf[1], val);
    }
    close(sock);
    vTaskDelete(NULL);
}

static uint16_t clamp_adc12(int raw)
{
    if (raw < 0) return 0u;
    if (raw > 4095) return 4095u;
    return (uint16_t)raw;
}

static void telemetry_send(uint16_t seq,
                           float vd, float v,
                           float theta, float alpha,
                           float omegal, float omegar,
                           float ul, float ur,
                           int raw_izq, int raw_der,
                           uint8_t flags)
{
    int s = udp_sock;
    if (s < 0) return;

    const telemetry_pkt_t pkt = {
        .header = TELEM_HEADER,
        .version = TELEM_VERSION,
        .seq = seq,
        .vd = vd,
        .v = v,
        .theta = theta,
        .alpha = alpha,
        .omegal = omegal,
        .omegar = omegar,
        .ul = ul,
        .ur = ur,
        .sl_adc = clamp_adc12(raw_izq),
        .sr_adc = clamp_adc12(raw_der),
        .flags = flags,
    };
    sendto(s, &pkt, sizeof(pkt), 0,
           (struct sockaddr *)&udp_dest, sizeof(udp_dest));
}

/* ============================================================================
   SECCIÓN 15 — TAREA DE CONTROL (Core 1, prioridad máxima, 100 Hz)

   Lógica idéntica al while(true) del PIC (embebidoRVCSclas.c):
     1. Leer sensores de línea (ADC)
     2. Leer IMU: Ax y Gy
     3. Leer encoders y calcular velocidades
     4. Calcular theta (error seguimiento línea)
     5. Filtro complementario → alpha
     6. Detección de caída → motores off
     7. Cinemática: v, thetap, eop
     8. Errores: ei, eip, ev + integrador con anti-windup
     9. Leyes de control: taua, u
    10. Torques por rueda: taur, taul
    11. Voltajes con back-EMF: ur, ul
    12. Actuación + telemetría UDP
   ============================================================================ */
static void control_task(void *arg)
{
    /* Estado del filtro complementario */
    float angulox_1 = 0.0f;

    /* Estado de los controladores */
    float ei_1  = 0.0f;
    float intev = 0.0f;
    float vd_r  = 0.0f;   /* Velocidad deseada con rampa */

    uint8_t imu_raw[MPU_FRAME_LEN];
    uint32_t log_cnt = 0;
    uint16_t telem_seq = 0;

    /* Esperar convergencia del filtro antes de activar control */
    vTaskDelay(pdMS_TO_TICKS(500));
    ESP_LOGI(TAG, "Control activo — 100 Hz");

    while (1) {
        /* Esperar tick del timer (10 ms exactos) */
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        gpio_set_level(PIN_DEBUG, 1);   /* ← para medir con osciloscopio */

        /* ── PASO 1: Sensores de línea ───────────────────────────────────── */
        int raw_izq = 0, raw_der = 0;
        adc_oneshot_read(adc1_handle, ADC_CH_IZQ, &raw_izq);
        adc_oneshot_read(adc1_handle, ADC_CH_DER, &raw_der);
        float Sl = (float)raw_izq;   /* ADC12 crudo: 0–4095 */
        float Sr = (float)raw_der;

        /* ── PASO 2: IMU — lectura burst ACCEL+GYRO para menor jitter ───── */
        mpu_read_regs(REG_ACCEL_XH, imu_raw, MPU_FRAME_LEN);
        int16_t Ax = (int16_t)((imu_raw[0] << 8) | imu_raw[1]);
        int16_t Gy = (int16_t)((imu_raw[MPU_GYRO_Y_IDX] << 8) |
                               imu_raw[MPU_GYRO_Y_IDX + 1]);

        /* ── PASO 3: Encoders — delta de cuentas en 10 ms ───────────────── */
        portENTER_CRITICAL(&enc_mux);
        int32_t cnt_r = enc_r; enc_r = 0;
        int32_t cnt_l = enc_l; enc_l = 0;
        portEXIT_CRITICAL(&enc_mux);

        /* ── PASO 4: Error de seguimiento de línea en escala ADC12 ─────── */
        float theta = Sr - Sl;
        if (theta >  LINE_ERR_MAX_ADC) theta =  LINE_ERR_MAX_ADC;
        if (theta < -LINE_ERR_MAX_ADC) theta = -LINE_ERR_MAX_ADC;
        theta = -LINE_THETA_MAX * theta / LINE_ERR_MAX_ADC;

        /* ── PASO 5: Velocidades angulares de rueda ──────────────────────
         * Cruce físico verificado: enc_l (LA/LB) → rueda derecha
         *                          enc_r (RA/RB) → rueda izquierda (negada)
         * esc = pi/(2·ppr·NR)  — igual que PIC                           */
        float omegar = (float) cnt_l * ESC_ENC * ITS;
        float omegal = -(float)cnt_r * ESC_ENC * ITS;
        if (omegar >  OMEGA_MAX) omegar =  OMEGA_MAX;
        if (omegar < -OMEGA_MAX) omegar = -OMEGA_MAX;
        if (omegal >  OMEGA_MAX) omegal =  OMEGA_MAX;
        if (omegal < -OMEGA_MAX) omegal = -OMEGA_MAX;

        /* ── PASO 6: Filtro complementario → alpha ───────────────────────
         * Igual al PIC:
         *   Xa     = -Ax * accel_factor          (rango -1 a 1)
         *   Yg     = Gy * gyro_factor * deg2rad  (rad/s)
         *   accelx = Xa * pi/2                   (rango -pi/2 a pi/2)
         *   angulox= c1*(angulox_1 + Yg*Ts) + c2*accelx
         *   alpha  = -angulox - calpha            */
        float Xa      = -(float)Ax * ACCEL_F;
        float Yg_rads = (float)Gy  * GYRO_F * DEG2RAD;
        float accelx  = Xa * PI_S2;
        float angulox = C1 * (angulox_1 + Yg_rads * TS) + C2 * accelx;
        angulox_1 = angulox;
        float alpha = -angulox - CALPHA;
        if (alpha >  ALPHA_MAX) alpha =  ALPHA_MAX;
        if (alpha < -ALPHA_MAX) alpha = -ALPHA_MAX;

        /* ── PASO 7: Detección de caída ──────────────────────────────────
         * Si alpha >= alphaM el robot está en el suelo — apagar motores
         * y resetear integradores (igual que PIC)                        */
        if (fabsf(alpha) >= ALPHA_MAX) {
            motors_off();
            ei_1  = 0.0f;
            intev = 0.0f;
            vd_r  = 0.0f;
            telemetry_send(telem_seq++, 0.0f, 0.0f, theta, alpha,
                           0.0f, 0.0f, 0.0f, 0.0f,
                           raw_izq, raw_der, 0x00u);
            gpio_set_level(PIN_DEBUG, 0);
            continue;
        }

        /* ── PASO 8: Cinemática (PIC) ────────────────────────────────────
         *   v      = (omegar + omegal)*R/2
         *   thetap = (omegar - omegal)*R/(2b)
         *   eop    = -thetap                                              */
        float v      = (omegar + omegal) * R_W / 2.0f;
        float thetap = (omegar - omegal) * R_W / (2.0f * B_H);
        float eop    = -thetap;
        float eo     = THETAD - theta;

        /* ── PASO 9: Errores y controladores (PIC) ───────────────────────
         * Rampa de velocidad para arranque suave (g_ramp=0 → escalón)    */
        float ei  = g_alphad - alpha;
        float eip = (ei - ei_1) * ITS;
        ei_1 = ei;

        if (g_ramp > 0.0f) {
            float vd_t = g_vd;
            if      (vd_r < vd_t) { vd_r += g_ramp; if (vd_r > vd_t) vd_r = vd_t; }
            else if (vd_r > vd_t) { vd_r -= g_ramp; if (vd_r < vd_t) vd_r = vd_t; }
        } else {
            vd_r = g_vd;
        }
        float ev = vd_r - v;

        /* Integral de ev con anti-windup (igual que PIC) */
        if (intev < V2TAUM && intev > -V2TAUM) {
            intev += TS * ev;
        } else {
            intev = (intev >= V2TAUM) ? 0.95f * V2TAUM : -0.95f * V2TAUM;
        }

        /* ── PASO 10: Leyes de control (PIC) ────────────────────────────
         *   taua = (kdo*eop + kpo*eo)*2b/R    — par diferencial (giro)
         *   u    = -kpi*ei - kdi*eip - kpv*ev - kiv*intev — par común   */
        float taua = (g_kdo * eop + g_kpo * eo) * 2.0f * B_H / R_W;
        float u    = -g_kpi * ei - g_kdi * eip - g_kpv * ev - g_kiv * intev;
        if (u >  V2TAUM) u =  V2TAUM;
        if (u < -V2TAUM) u = -V2TAUM;

        /* ── PASO 11: Torques por rueda (PIC) ───────────────────────────
         *   taur = (taua + u)/2     taul = (-taua + u)/2                 */
        float taur = (taua + u) / 2.0f;
        float taul = (-taua + u) / 2.0f;

        /* ── PASO 12: Voltaje con compensación back-EMF (PIC) ───────────
         *   V = tau*Ra/(km*NR) + km*NR*omega                             */
        float ul = taul * RASNKM + NKM * omegal;
        if (ul >  U_NM) ul =  U_NM;
        if (ul < -U_NM) ul = -U_NM;

        float ur = taur * RASNKM + NKM * omegar;
        if (ur >  U_NM) ur =  U_NM;
        if (ur < -U_NM) ur = -U_NM;

        /* ── PASO 13: Actuación ──────────────────────────────────────────
         * Motor izquierdo recibe -ul (inversión física verificada).
         * Motor derecho recibe ur directamente.                           */
        motor_set(PIN_MOT_L_IN1, PIN_MOT_L_IN2, LEDC_CH_L, -ul);
        motor_set(PIN_MOT_R_IN1, PIN_MOT_R_IN2, LEDC_CH_R,  ur);

        /* ── PASO 14: Telemetría UDP v2, float32 + ADC crudo 12-bit ───── */
        telemetry_send(telem_seq++, vd_r, v, theta, alpha,
                       omegal, omegar, ul, ur,
                       raw_izq, raw_der, 0x00u);

        /* ── Log serial cada 500 ms (50 ciclos) ──────────────────────── */
        if (++log_cnt >= 50) {
            log_cnt = 0;
            ESP_LOGI(TAG, "a=%.1f° v=%.3f wl=%.2f wr=%.2f ul=%.2f ur=%.2f",
                     alpha * RAD2DEG, v, omegal, omegar, ul, ur);
        }

        gpio_set_level(PIN_DEBUG, 0);
    }
}

/* ============================================================================
   SECCIÓN 16 — PUNTO DE ENTRADA
   ============================================================================ */
void app_main(void)
{
    ESP_LOGI(TAG, "=== PISDRSL — ESP32 + FreeRTOS ===");
    ESP_LOGI(TAG, "NR=%.0f  calpha=%.3f  tauM=%.2f", NR, CALPHA, TAU_MAX);
    ESP_LOGI(TAG, "kpi=%.2f  kdi=%.2f  kpv=%.2f  kiv=%.2f",
             g_kpi, g_kdi, g_kpv, g_kiv);

    hw_init();
    mpu_init();

    /* Control: Core 1, prioridad máxima */
    xTaskCreatePinnedToCore(control_task, "ctrl", 4096, NULL,
                            configMAX_PRIORITIES - 1, &ctrl_task_handle, 1);

    /* Timer a 100 Hz (10 000 µs) — dispatch desde ISR para menor jitter */
    esp_timer_handle_t tmr;
    const esp_timer_create_args_t ta = {
        .callback = timer_cb,
        .dispatch_method = ESP_TIMER_ISR,
        .name = "ctrl_tmr",
    };
    ESP_ERROR_CHECK(esp_timer_create(&ta, &tmr));
    ESP_ERROR_CHECK(esp_timer_start_periodic(tmr, 10000));

    /* WiFi + UDP (bloquea hasta conectar o timeout de 15 s) */
    wifi_init();
    udp_init();

    /* Recepción de ganancias: Core 0, prioridad baja */
    xTaskCreate(cmd_task, "cmd", 3072, NULL, 5, NULL);

    ESP_LOGI(TAG, "Sistema listo. Monitor: python monitor/monitor.py");
}
