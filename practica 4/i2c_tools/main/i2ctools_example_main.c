#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "esp_err.h"

// Pines I2C del ESP32 DevKitC V4
#define I2C_SDA_PIN     21
#define I2C_SCL_PIN     22
#define I2C_FREQ_HZ     100000
#define I2C_DEV_ADDR    0x40

static const char *TAG = "si7021";

// Lee dos bytes del sensor usando el comando indicado
static esp_err_t si7021_read_raw(i2c_master_dev_handle_t dev,
                                 uint8_t command,
                                 uint16_t *raw)
{
    uint8_t data[2];

    // Enviar comando de medición
    esp_err_t ret = i2c_master_transmit(dev, &command, 1, 1000);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error enviando comando: %s", esp_err_to_name(ret));
        return ret;
    }

    // Esperar a que el sensor termine la medición
    vTaskDelay(pdMS_TO_TICKS(20));

    // Recibir los dos bytes del resultado
    ret = i2c_master_receive(dev, data, 2, 1000);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error leyendo sensor: %s", esp_err_to_name(ret));
        return ret;
    }

    // Combinar los dos bytes y eliminar los bits de estado
    *raw = ((uint16_t)data[0] << 8) | data[1];
    *raw &= 0xFFFC;

    return ESP_OK;
}

// Leer temperatura
static float si7021_read_temperature(i2c_master_dev_handle_t dev)
{
    uint16_t raw;

    if (si7021_read_raw(dev, 0xF3, &raw) != ESP_OK) {
        return -999.0f;
    }

    float temp_c = ((175.72f * raw) / 65536.0f) - 46.85f;

    return temp_c;
}

// Leer humedad
static float si7021_read_humidity(i2c_master_dev_handle_t dev)
{
    uint16_t raw;

    if (si7021_read_raw(dev, 0xF5, &raw) != ESP_OK) {
        return -1.0f;
    }

    float rh = ((125.0f * raw) / 65536.0f) - 6.0f;

    if (rh > 100.0f) rh = 100.0f;
    if (rh < 0.0f) rh = 0.0f;

    return rh;
}

void app_main(void)
{
    // Configurar el bus I2C
    i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = I2C_SDA_PIN,
        .scl_io_num = I2C_SCL_PIN,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    i2c_master_bus_handle_t bus_handle;

    ESP_ERROR_CHECK(
        i2c_new_master_bus(&bus_config, &bus_handle)
    );

    // Configurar el dispositivo Si7021
    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = I2C_DEV_ADDR,
        .scl_speed_hz = I2C_FREQ_HZ,
    };

    i2c_master_dev_handle_t si7021;

    ESP_ERROR_CHECK(
        i2c_master_bus_add_device(bus_handle, &dev_config, &si7021)
    );

    ESP_LOGI(TAG, "Sensor Si7021 inicializado");

    // Bucle principal
    while (1) {

        float temp = si7021_read_temperature(si7021);

        if (temp > -100.0f) {
            ESP_LOGI(TAG, "Temperatura: %.2f °C", temp);
        }

        vTaskDelay(pdMS_TO_TICKS(1000));

        float hum = si7021_read_humidity(si7021);

        if (hum >= 0.0f) {
            ESP_LOGI(TAG, "Humedad: %.2f %%", hum);
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    // Limpieza (no se alcanza mientras el bucle sea infinito)
    ESP_ERROR_CHECK(i2c_master_bus_rm_device(si7021));
    ESP_ERROR_CHECK(i2c_del_master_bus(bus_handle));
}