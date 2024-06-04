#include "bme280.h"

int32_t compensate_temperature(const struct bme280_uncomp_data *uncomp_data, struct bme280_calib_data *calib_data);
uint32_t compensate_pressure(const struct bme280_uncomp_data *uncomp_data,
                             const struct bme280_calib_data *calib_data);
uint32_t compensate_humidity(const struct bme280_uncomp_data *uncomp_data,
                             const struct bme280_calib_data *calib_data);

esp_err_t bme280_free(struct bme280_dev *dev)
{
    esp_err_t err;
    err = ESP_ERROR_CHECK_WITHOUT_ABORT(i2c_del_master_bus(dev->i2c_bus_handle));
    if (err != ESP_OK)
    {
        return err;
    }

    err = ESP_ERROR_CHECK_WITHOUT_ABORT(i2c_master_bus_rm_device(dev->i2c_dev_handle));
    if (err != ESP_OK)
    {
        return err;
    }

    return ESP_OK;
}

esp_err_t bme280_get_regs(uint8_t reg_addr, uint8_t *reg_data, uint16_t len, const struct bme280_dev *dev)
{
    esp_err_t err;
    uint16_t read_size;

    if (len > 1)
    {
        read_size = len * sizeof(reg_data[0]);
    }
    else
    {
        read_size = sizeof(reg_data);
    }

    err = ESP_ERROR_CHECK_WITHOUT_ABORT(i2c_master_transmit_receive(dev->i2c_dev_handle, &reg_addr, sizeof(reg_addr), reg_data, read_size, -1));
    if (err != ESP_OK)
    {
        return err;
    }
    return ESP_OK;
}

esp_err_t bme280_set_regs(uint8_t *reg_addr, const uint8_t *reg_data, uint8_t len, const struct bme280_dev *dev)
{
    esp_err_t err;
    uint8_t txBuf[2 * len];

    if (len > 1)
    {
        for (int i = 0; i < len; i++)
        {
            txBuf[2 * i] = reg_addr[i];
            txBuf[2 * i + 1] = reg_data[i];
        }
    }
    else
    {
        txBuf[0] = reg_addr[0];
        txBuf[1] = reg_data[0];
    }

    err = ESP_ERROR_CHECK_WITHOUT_ABORT(i2c_master_transmit(dev->i2c_dev_handle, txBuf, sizeof(txBuf), -1));
    if (err != ESP_OK)
    {
        return err;
    }
    return ESP_OK;
}

esp_err_t bme280_get_sensor_id(uint8_t *sensor_id, const struct bme280_dev *dev)
{
    esp_err_t err;
    err = bme280_get_regs(BME280_CHIP_ID_ADDR, sensor_id, sizeof(sensor_id), dev);
    if (err != ESP_OK)
    {
        return err;
    }
    return ESP_OK;
}

esp_err_t bme280_soft_reset(const struct bme280_dev *dev)
{
    esp_err_t err;
    uint8_t soft_reset_command = 0xB6;
    uint8_t soft_reset_address = BME280_RESET_ADDR;

    err = bme280_set_regs(&soft_reset_address, &soft_reset_command, 1, dev);
    if (err != ESP_OK)
    {
        return err;
    }
    return ESP_OK;
}

esp_err_t bme280_set_sensor_mode(uint8_t sensor_mode, const struct bme280_dev *dev)
{
    esp_err_t err;
    uint8_t reg_data;
    uint8_t pwr_ctrl_addr = BME280_PWR_CTRL_ADDR;

    if (!(sensor_mode == BME280_SLEEP_MODE || sensor_mode == BME280_NORMAL_MODE))
    {
        return ESP_ERR_INVALID_ARG;
    }

    err = bme280_get_regs(pwr_ctrl_addr, &reg_data, 1, dev);
    if (err != ESP_OK)
    {
        return err;
    }
    reg_data = (reg_data & 0xFC) | sensor_mode;
    err = bme280_set_regs(&pwr_ctrl_addr, &reg_data, 1, dev);
    if (err != ESP_OK)
    {
        return err;
    }

    return ESP_OK;
}

esp_err_t bme280_get_sensor_mode(uint8_t *sensor_mode, const struct bme280_dev *dev)
{
    esp_err_t err;
    uint8_t reg_data;
    uint8_t pwr_ctrl_addr = BME280_PWR_CTRL_ADDR;

    err = bme280_get_regs(pwr_ctrl_addr, &reg_data, 1, dev);
    if (err != ESP_OK)
    {
        return err;
    }

    *sensor_mode = reg_data & 0x03;
    return ESP_OK;
}

esp_err_t bme280_set_sensor_settings(const struct bme280_settings new_settings, const struct bme280_dev *dev)
{
    esp_err_t err;
    uint8_t ctrl_hum_meas_config_addr = BME280_CTRL_HUM_ADDR;
    uint8_t current_settings[4];

    if (new_settings.osrs_t > BME280_OVERSAMPLING_16X ||
        new_settings.osrs_p > BME280_OVERSAMPLING_16X ||
        new_settings.osrs_h > BME280_OVERSAMPLING_16X ||
        new_settings.filter > BME280_FILTER_COEFF_16 ||
        new_settings.standby_time > BME280_STANDBY_TIME_20_MS)
    {
        return ESP_ERR_INVALID_ARG;
    }

    err = bme280_get_regs(ctrl_hum_meas_config_addr, current_settings, 4, dev);
    if (err != ESP_OK)
    {
        return err;
    }

    if (new_settings.osrs_h)
    {
        current_settings[0] = (current_settings[0] & 0xF8) | (new_settings.osrs_h & 0x07);
    }

    if (new_settings.osrs_t)
    {
        current_settings[2] = (current_settings[2] & 0x1F) | (new_settings.osrs_t << 5);
    }

    if (new_settings.osrs_p)
    {
        current_settings[2] = (current_settings[2] & 0xE3) | (new_settings.osrs_p << 2);
    }

    if (new_settings.standby_time)
    {
        current_settings[3] = (current_settings[3] & 0x1F) | (new_settings.standby_time << 5);
    }

    if (new_settings.filter)
    {
        current_settings[3] = (current_settings[3] & 0xE3) | (new_settings.filter << 2);
    }

    uint8_t reg_addr[3] = {
        BME280_CTRL_HUM_ADDR,
        BME280_CTRL_MEAS_ADDR,
        BME280_CONFIG_ADDR,
    };

    uint8_t reg_data[3] = {
        current_settings[0],
        current_settings[2],
        current_settings[3],
    };

    err = bme280_set_regs(reg_addr, reg_data, 3, dev);
    if (err != ESP_OK)
    {
        return err;
    }
    return ESP_OK;
}

esp_err_t bme280_get_sensor_settings(struct bme280_dev *dev)
{
    esp_err_t err;
    uint8_t ctrl_hum_meas_config_addr = BME280_CTRL_HUM_ADDR;
    uint8_t current_settings[4];

    err = bme280_get_regs(ctrl_hum_meas_config_addr, current_settings, 4, dev);
    if (err != ESP_OK)
    {
        return err;
    }

    dev->settings.osrs_h = current_settings[0] & 0x07;
    dev->settings.osrs_t = (current_settings[2] & 0xE3) >> 5;
    dev->settings.osrs_p = (current_settings[2] & 0x1C) >> 2;
    dev->settings.standby_time = (current_settings[3] & 0xE3) >> 5;
    dev->settings.filter = (current_settings[3] & 0x1C) >> 2;

    return ESP_OK;
}

esp_err_t get_calib_data(struct bme280_dev *dev)
{
    esp_err_t err;

    /* Read the calibration data for  from the sensor */
    uint8_t reg_addr = BME280_TEMP_PRESS_CALIB_DATA_ADDR;
    uint8_t calib_data[BME280_TEMP_PRESS_CALIB_DATA_LEN];

    err = bme280_get_regs(reg_addr, calib_data, BME280_TEMP_PRESS_CALIB_DATA_LEN, dev);
    if (err != ESP_OK)
    {
        return err;
    }

    dev->calib_data.dig_T1 = BME280_CONCAT_BYTES(calib_data[1], calib_data[0]);
    dev->calib_data.dig_T2 = (int16_t)BME280_CONCAT_BYTES(calib_data[3], calib_data[2]);
    dev->calib_data.dig_T3 = (int16_t)BME280_CONCAT_BYTES(calib_data[5], calib_data[4]);
    dev->calib_data.dig_P1 = BME280_CONCAT_BYTES(calib_data[7], calib_data[6]);
    dev->calib_data.dig_P2 = (int16_t)BME280_CONCAT_BYTES(calib_data[9], calib_data[8]);
    dev->calib_data.dig_P3 = (int16_t)BME280_CONCAT_BYTES(calib_data[11], calib_data[10]);
    dev->calib_data.dig_P4 = (int16_t)BME280_CONCAT_BYTES(calib_data[13], calib_data[12]);
    dev->calib_data.dig_P5 = (int16_t)BME280_CONCAT_BYTES(calib_data[15], calib_data[14]);
    dev->calib_data.dig_P6 = (int16_t)BME280_CONCAT_BYTES(calib_data[17], calib_data[16]);
    dev->calib_data.dig_P7 = (int16_t)BME280_CONCAT_BYTES(calib_data[19], calib_data[18]);
    dev->calib_data.dig_P8 = (int16_t)BME280_CONCAT_BYTES(calib_data[21], calib_data[20]);
    dev->calib_data.dig_P9 = (int16_t)BME280_CONCAT_BYTES(calib_data[23], calib_data[22]);
    dev->calib_data.dig_H1 = calib_data[25];

    reg_addr = BME280_HUMIDITY_CALIB_DATA_ADDR;
    err = bme280_get_regs(reg_addr, calib_data, BME280_HUMIDITY_CALIB_DATA_LEN, dev);
    if (err != ESP_OK)
    {
        return err;
    }

    dev->calib_data.dig_H2 = (int16_t)BME280_CONCAT_BYTES(calib_data[1], calib_data[0]);
    dev->calib_data.dig_H3 = calib_data[2];
    dev->calib_data.dig_H4 = (int16_t)(((int16_t)(int8_t)calib_data[3] * 16) | ((int16_t)(calib_data[4] & 0x0F)));
    dev->calib_data.dig_H5 = (int16_t)(((int16_t)(int8_t)calib_data[5] * 16) | ((int16_t)(calib_data[4] >> 4)));
    dev->calib_data.dig_H6 = (int8_t)calib_data[6];

    return ESP_OK;
}

void bme280_parse_sensor_data(const uint8_t *reg_data, struct bme280_uncomp_data *uncomp_data)
{
    uncomp_data->pressure = (uint32_t)reg_data[0] << 12 | (uint32_t)reg_data[1] << 4 | (uint32_t)reg_data[2] >> 4;
    uncomp_data->temperature = (uint32_t)reg_data[3] << 12 | (uint32_t)reg_data[4] << 4 | (uint32_t)reg_data[5] >> 4;
    uncomp_data->humidity = (uint32_t)reg_data[6] << 8 | (uint32_t)reg_data[7];
}

esp_err_t bme280_compensate_data(uint8_t sensor_comp, const struct bme280_uncomp_data *uncomp_data,
                                 struct bme280_comp_data *comp_data, struct bme280_calib_data *calib_data)
{
    if ((uncomp_data == NULL) || (comp_data == NULL) || (calib_data == NULL))
    {
        return ESP_ERR_INVALID_ARG;
    }

    comp_data->temperature = 0;
    comp_data->pressure = 0;
    comp_data->humidity = 0;

    if (sensor_comp & 1 || sensor_comp & 7)
    {
        comp_data->pressure = compensate_pressure(uncomp_data, calib_data);
    }

    if (sensor_comp & 2 || sensor_comp & 7)
    {
        comp_data->temperature = compensate_temperature(uncomp_data, calib_data);
    }

    if (sensor_comp & 4 || sensor_comp & 7)
    {
        comp_data->humidity = compensate_humidity(uncomp_data, calib_data);
    }

    return ESP_OK;
}

int32_t compensate_temperature(const struct bme280_uncomp_data *uncomp_data, struct bme280_calib_data *calib_data)
{
    int32_t var1;
    int32_t var2;
    int32_t temperature;
    int32_t temperature_min = -4000;
    int32_t temperature_max = 8500;

    var1 = (int32_t)((uncomp_data->temperature / 8) - ((int32_t)calib_data->dig_T1 * 2));
    var1 = (var1 * ((int32_t)calib_data->dig_T2)) / 2048;
    var2 = (int32_t)((uncomp_data->temperature / 16) - ((int32_t)calib_data->dig_T1));
    var2 = (((var2 * var2) / 4096) * ((int32_t)calib_data->dig_T3)) / 16384;
    calib_data->t_fine = var1 + var2;
    temperature = (calib_data->t_fine * 5 + 128) / 256;

    if (temperature < temperature_min)
        temperature = temperature_min;
    else if (temperature > temperature_max)
        temperature = temperature_max;

    return temperature;
}

uint32_t compensate_pressure(const struct bme280_uncomp_data *uncomp_data,
                             const struct bme280_calib_data *calib_data)
{
    int32_t var1;
    int32_t var2;
    int32_t var3;
    int32_t var4;
    uint32_t var5;
    uint32_t pressure;
    uint32_t pressure_min = 30000;
    uint32_t pressure_max = 110000;

    var1 = (((int32_t)calib_data->t_fine) / 2) - (int32_t)64000;
    var2 = (((var1 / 4) * (var1 / 4)) / 2048) * ((int32_t)calib_data->dig_P6);
    var2 = var2 + ((var1 * ((int32_t)calib_data->dig_P5)) * 2);
    var2 = (var2 / 4) + (((int32_t)calib_data->dig_P4) * 65536);
    var3 = (calib_data->dig_P3 * (((var1 / 4) * (var1 / 4)) / 8192)) / 8;
    var4 = (((int32_t)calib_data->dig_P2) * var1) / 2;
    var1 = (var3 + var4) / 262144;
    var1 = (((32768 + var1)) * ((int32_t)calib_data->dig_P1)) / 32768;

    if (var1)
    {
        var5 = (uint32_t)((uint32_t)1048576) - uncomp_data->pressure;
        pressure = ((uint32_t)(var5 - (uint32_t)(var2 / 4096))) * 3125;
        if (pressure < 0x80000000)
            pressure = (pressure << 1) / ((uint32_t)var1);
        else
            pressure = (pressure / (uint32_t)var1) * 2;

        var1 = (((int32_t)calib_data->dig_P9) * ((int32_t)(((pressure / 8) * (pressure / 8)) / 8192))) / 4096;
        var2 = (((int32_t)(pressure / 4)) * ((int32_t)calib_data->dig_P8)) / 8192;
        pressure = (uint32_t)((int32_t)pressure + ((var1 + var2 + calib_data->dig_P7) / 16));

        if (pressure < pressure_min)
            pressure = pressure_min;
        else if (pressure > pressure_max)
            pressure = pressure_max;
    }
    else
    {
        pressure = pressure_min;
    }

    return pressure;
}

uint32_t compensate_humidity(const struct bme280_uncomp_data *uncomp_data,
                             const struct bme280_calib_data *calib_data)
{
    int32_t var1;
    int32_t var2;
    int32_t var3;
    int32_t var4;
    int32_t var5;
    uint32_t humidity;
    uint32_t humidity_max = 102400;

    var1 = calib_data->t_fine - ((int32_t)76800);
    var2 = (int32_t)(uncomp_data->humidity * 16384);
    var3 = (int32_t)(((int32_t)calib_data->dig_H4) * 1048576);
    var4 = ((int32_t)calib_data->dig_H5) * var1;
    var5 = (((var2 - var3) - var4) + (int32_t)16384) / 32768;
    var2 = (var1 * ((int32_t)calib_data->dig_H6)) / 1024;
    var3 = (var1 * ((int32_t)calib_data->dig_H3)) / 2048;
    var4 = ((var2 * (var3 + (int32_t)32768)) / 1024) + (int32_t)2097152;
    var2 = ((var4 * ((int32_t)calib_data->dig_H2)) + 8192) / 16384;
    var3 = var5 * var2;
    var4 = ((var3 / 32768) * (var3 / 32768)) / 128;
    var5 = var3 - ((var4 * ((int32_t)calib_data->dig_H1)) / 16);
    var5 = (var5 < 0 ? 0 : var5);
    var5 = (var5 > 419430400 ? 419430400 : var5);
    humidity = (uint32_t)(var5 / 4096);

    if (humidity > humidity_max)
        humidity = humidity_max;

    return humidity;
}

esp_err_t bme280_get_sensor_data(uint8_t sensor_comp_mode, struct bme280_data *data, struct bme280_dev *dev)
{
    esp_err_t err;
    uint8_t reg_data[BME280_P_T_H_DATA_LEN] = {0};
    struct bme280_uncomp_data uncomp_data = {0};

    err = bme280_get_regs(BME280_DATA_ADDR, reg_data, BME280_P_T_H_DATA_LEN, dev);
    if (err != ESP_OK)
    {
        return err;
    }

    bme280_parse_sensor_data(reg_data, &uncomp_data);

    struct bme280_comp_data comp_data = {0};

    err = bme280_compensate_data(sensor_comp_mode, &uncomp_data, &comp_data, &dev->calib_data);
    if (err != ESP_OK)
    {
        return err;
    }

    data->temperature = comp_data.temperature / 100.0;
    data->pressure = comp_data.pressure / 256.0 / 100.0;
    data->humidity = comp_data.humidity / 1024.0;

    return ESP_OK;
}

esp_err_t bme280_init(struct bme280_dev *dev, uint32_t slave_addr, uint8_t sda, uint8_t scl, uint32_t clk_freq)
{
    i2c_master_bus_handle_t bus_handle;
    i2c_master_dev_handle_t dev_handle;
    esp_err_t err;

    i2c_master_bus_config_t i2c_mst_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .sda_io_num = sda,
        .scl_io_num = scl,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    err = ESP_ERROR_CHECK_WITHOUT_ABORT(i2c_new_master_bus(&i2c_mst_config, &bus_handle));
    if (err != ESP_OK)
    {
        return err;
    }

    i2c_device_config_t i2c_dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_7,
        .device_address = slave_addr,
        .scl_speed_hz = clk_freq,
    };

    err = ESP_ERROR_CHECK_WITHOUT_ABORT(i2c_master_bus_add_device(bus_handle, &i2c_dev_config, &dev_handle));
    if (err != ESP_OK)
    {
        return err;
    }

    dev->i2c_bus_handle = bus_handle;
    dev->i2c_dev_handle = dev_handle;

    uint8_t sensor_id;
    err = bme280_get_sensor_id(&sensor_id, dev);
    if (err != ESP_OK)
    {
        return err;
    }

    if (sensor_id != BME280_CHIP_ID)
    {
        return ESP_ERR_INVALID_RESPONSE;
    }

    dev->chip_id = sensor_id;

    err = bme280_soft_reset(dev);
    if (err != ESP_OK)
    {
        return err;
    }

    return ESP_OK;
}