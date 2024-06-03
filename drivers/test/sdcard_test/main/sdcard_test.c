#include <stdio.h>
#include "sdspi.h"

static const char *TAG = "sdcard_test";

void app_main(void)
{
    esp_err_t err;

    const char mount_point[] = MOUNT_POINT;
    struct sdcard_dev reader = {
        .mount_point = mount_point,
    };

    err = sdcard_init(PIN_NUM_MOSI, PIN_NUM_MISO, PIN_NUM_CLK, &reader);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to initialize SD card");
        return;
    }
    else
    {
        ESP_LOGI(TAG, "SD card initialized");
    }

    err = sdcard_mount(PIN_NUM_CS, &reader);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to mount SD card");
        return;
    }
    else
    {
        ESP_LOGI(TAG, "SD card mounted");
    }

    const char *file = MOUNT_POINT "/TESTING.txt";
    char data[MAX_CHAR_SIZE];
    snprintf(data, MAX_CHAR_SIZE, "temperature: %f pressure: %f humidity: %f \n", 30.4, 2.5, 802.2);
    err = sdcard_write(file, data);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to write to file");
        return;
    }
    else
    {
        ESP_LOGI(TAG, "Data written to file");
    }

    // Read from file
    FILE *f = fopen(file, "r");
    if (f == NULL)
    {
        return ;
    }

    char line[MAX_CHAR_SIZE];
    while (fgets(line, MAX_CHAR_SIZE, f) != NULL)
    {
        char *pos = strchr(line, '\n');
        if (pos)
        {
            *pos = '\0';
        }
        ESP_LOGI(TAG, "Read from file: '%s'", line);
    }
    fclose(f);

    err = sdcard_unmount(&reader);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to unmount SD card");
        return;
    }
    else
    {
        ESP_LOGI(TAG, "SD card unmounted");
    }

    err = sdcard_free(&reader);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to unmount SD card");
        return;
    }
    else
    {
        ESP_LOGI(TAG, "SD card freed");
    }
}
