#include "esp_heap_caps.h"
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/i2c_master.h"
#include "icm42670.h"
#include "led_strip.h"

static const char *TAG = "IMU_ORIENTACION";

/* Pines de la placa ESP-RUST-BOARD (ESP32-C3) */
#define I2C_MASTER_SCL_IO   8
#define I2C_MASTER_SDA_IO   10
#define LED_RGB_GPIO        2

static i2c_master_bus_handle_t i2c_handle = NULL;
static icm42670_handle_t icm42670 = NULL;
static led_strip_handle_t led_strip = NULL;

/* 1. Inicialización del bus I2C con el nuevo driver */
static void i2c_bus_init(void)
{
    i2c_master_bus_config_t bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_NUM_0,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &i2c_handle));
}

/* 2. Inicialización del LED RGB (WS2812 en GPIO 2) */
static void rgb_led_init(void)
{
    led_strip_config_t strip_config = {
        .strip_gpio_num = LED_RGB_GPIO,
        .max_leds = 1,
    };
    led_strip_rmt_config_t rmt_config = {
        .resolution_hz = 10 * 1000 * 1000, // 10 MHz
    };
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));
    led_strip_clear(led_strip);
}

/* Helper para cambiar el color del LED RGB */
static void set_led_color(uint8_t r, uint8_t g, uint8_t b)
{
    led_strip_set_pixel(led_strip, 0, r, g, b);
    led_strip_refresh(led_strip);
}

void app_main(void)
{
    /* Inicializar periféricos */
    i2c_bus_init();
    rgb_led_init();

    /* 3. Crear el handle del dispositivo ICM-42670-P (dirección I2C 0x68) */
    ESP_ERROR_CHECK(icm42670_create(i2c_handle, ICM42670_I2C_ADDRESS, &icm42670));

    /* 4. Configurar acelerómetro y giroscopio */
    const icm42670_cfg_t imu_cfg = {
        .acce_fs = ACCE_FS_2G,
        .acce_odr = ACCE_ODR_400HZ,
        .gyro_fs = GYRO_FS_2000DPS,
        .gyro_odr = GYRO_ODR_400HZ,
    };
    ESP_ERROR_CHECK(icm42670_config(icm42670, &imu_cfg));

    /* Activar el acelerómetro en modo bajo ruido (Low Noise) */
    ESP_ERROR_CHECK(icm42670_acce_set_pwr(icm42670, ACCE_PWR_LOWNOISE));

    icm42670_value_t acce_val;
    int estado_anterior = -1; // 0: Boca arriba, 1: Boca abajo, 2: De lado

    while (1) {
        /* Leer valores calibrados en unidades g (X, Y, Z) */
        if (icm42670_get_acce_value(icm42670, &acce_val) == ESP_OK) {
            if (acce_val.z > 0.5f) {
                if (estado_anterior != 0) {
                    ESP_LOGI(TAG, "Estado: BOCA ARRIBA (Z = %.2f g) -> LED VERDE", acce_val.z);
                    set_led_color(0, 40, 0); // Verde
                    estado_anterior = 0;
                }
            } else if (acce_val.z < -0.5f) {
                if (estado_anterior != 1) {
                    ESP_LOGI(TAG, "Estado: BOCA ABAJO  (Z = %.2f g) -> LED ROJO", acce_val.z);
                    set_led_color(40, 0, 0); // Rojo
                    estado_anterior = 1;
                }
            } else {
                if (estado_anterior != 2) {
                    ESP_LOGI(TAG, "Estado: DE LADO / INCLINADA (Z = %.2f g) -> LED AZUL", acce_val.z);
                    set_led_color(0, 0, 40); // Azul
                    estado_anterior = 2;
                }
            }
        } else {
            ESP_LOGE(TAG, "Error al leer el acelerómetro");
        }

        vTaskDelay(pdMS_TO_TICKS(200));
    }
}