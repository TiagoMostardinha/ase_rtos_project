#include <stdio.h>
#include "bme280.h"
#include <esp_log.h>
#include "freertos/FreeRTOS.h"

void app_main(void)
{
    struct bme280_settings settings = {
        .osrs_t = BME280_OVERSAMPLING_4X,
        .osrs_p = BME280_OVERSAMPLING_2X,
        .osrs_h = BME280_OVERSAMPLING_16X,
        .standby_time = BME280_STANDBY_TIME_500_MS,
        .filter = BME280_FILTER_COEFF_2};

    struct bme280_dev dev = {
        .chip_id = BME280_CHIP_ID,
    };

    esp_err_t err = bme280_init(&dev, BME280_I2C_ADDRESS, 3, 2, 400000);
    if (err != ESP_OK)
    {
        ESP_LOGE("BME280", "BME280 init failed! (%d)", err);
        return;
    }

    err = bme280_set_sensor_settings(settings, &dev);
    if (err != ESP_OK)
    {
        ESP_LOGE("BME280", "BME280 set sensor settings failed. %x", err);
        return;
    }
    else
    {
        ESP_LOGI("BME280", "BME280 set sensor settings success");
    }

    err = get_calib_data(&dev);
    if (err != ESP_OK)
    {
        return;
    }

    vTaskDelay(1000 / portTICK_PERIOD_MS);

    uint8_t i = 5;
    bool mode = true;

    uint8_t sensor_id;
    uint8_t sensor_mode;
    struct bme280_data data;
    while (1)
    {

        if (i > 5)
        {
            if (mode)
            {
                err = bme280_set_sensor_mode(BME280_SLEEP_MODE, &dev);
                mode = false;
            }
            else
            {
                err = bme280_set_sensor_mode(BME280_NORMAL_MODE, &dev);
                mode = true;
            }

            if (err != ESP_OK)
            {
                ESP_LOGE("BME280", "BME280 set sensor mode failed. %x", err);
                return;
            }
            else
            {
                ESP_LOGI("BME280", "BME280 set sensor mode success");
            }
            i = 0;
        }

        err = bme280_get_sensor_id(&sensor_id, &dev);
        if (err != ESP_OK)
        {
            ESP_LOGE("BME280", "BME280 get sensor id failed. %x", err);
            return;
        }

        err = bme280_get_sensor_mode(&sensor_mode, &dev);
        if (err != ESP_OK)
        {
            ESP_LOGE("BME280", "BME280 get sensor mode failed. %x", err);
            return;
        }
        err = bme280_get_sensor_settings(&dev);
        if (err != ESP_OK)
        {
            ESP_LOGE("BME280", "BME280 get sensor settings failed. %x", err);
            return;
        }

        err = bme280_get_sensor_data(7, &data, &dev);
        if (err != ESP_OK)
        {
            ESP_LOGE("BME280", "BME280 get sensor data failed. %x", err);
            return;
        }

        i += 5;

        printf("Sensor ID: %x\n", sensor_id);
        printf("Sensor Mode: %x\n", sensor_mode);
        printf("osrs_t: %x\nosrs_p: %x\nosrs_h: %x\nstandby_time: %x\nfilter: %x\n\n", dev.settings.osrs_t, dev.settings.osrs_p, dev.settings.osrs_h, dev.settings.standby_time, dev.settings.filter);
        printf("Pressure: %f\nTemperature: %f\nHumidity: %f\n\n", data.pressure, data.temperature, data.humidity);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
