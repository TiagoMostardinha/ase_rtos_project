#include "sdspi.h"

esp_err_t sdcard_init(uint8_t mosi, uint8_t miso, uint8_t clk, struct sdcard_dev *reader)
{
    esp_err_t err;
    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    reader->host = host;

    spi_bus_config_t bus_cfg = {
        .mosi_io_num = mosi,
        .miso_io_num = miso,
        .sclk_io_num = clk,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4000,
    };

    err = spi_bus_initialize(reader->host.slot, &bus_cfg, SDSPI_DEFAULT_DMA);
    if (err != ESP_OK)
    {
        return err;
    }

    return ESP_OK;
}

esp_err_t sdcard_free(struct sdcard_dev *reader)
{
    esp_err_t err;

    err = spi_bus_free(reader->host.slot);
    if (err != ESP_OK)
    {
        return err;
    }
    return ESP_OK;
}

esp_err_t sdcard_mount(uint8_t cs, struct sdcard_dev *reader)
{
    esp_err_t err;

    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = cs;
    slot_config.host_id = reader->host.slot;

    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = true,
        .max_files = 5,
        .allocation_unit_size = 16 * 1024};

    err = esp_vfs_fat_sdspi_mount(reader->mount_point, &reader->host, &slot_config, &mount_config, &reader->card);

    if (err != ESP_OK)
    {
        return err;
    }

    sdmmc_card_print_info(stdout, reader->card);

    return ESP_OK;
}

esp_err_t sdcard_unmount(struct sdcard_dev *reader)
{
    esp_err_t err;

    err = esp_vfs_fat_sdcard_unmount(reader->mount_point, reader->card);
    if (err != ESP_OK)
    {
        return err;
    }
    return ESP_OK;
}

esp_err_t sdcard_write(const char *file, const char *data)
{
    FILE *f = fopen(file, "a");
    if (f == NULL)
    {
        return ESP_FAIL;
    }
    fprintf(f, data);
    fclose(f);
    return ESP_OK;
}