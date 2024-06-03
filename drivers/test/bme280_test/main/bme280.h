#ifndef __BME280_SENSOR_H__INCLUDED__
#define __BME280_SENSOR_H__INCLUDED__

#include "esp_err.h"
#include "bme280_defs.h"
#include "driver/i2c_master.h"

// TODO: 0xF2 e 0xF5 temos que fazer primeiro uma leitura para depois escrever

esp_err_t bme280_init(struct bme280_dev *dev, uint32_t slave_addr, uint8_t sda, uint8_t scl, uint32_t clk_freq);

esp_err_t bme280_free(struct bme280_dev *dev);

esp_err_t bme280_soft_reset(const struct bme280_dev *dev);

// TODO: bme280 set registers
esp_err_t bme280_set_regs(uint8_t *reg_addr, const uint8_t *reg_data, uint8_t len, const struct bme280_dev *dev);

esp_err_t bme280_get_regs(uint8_t reg_addr, uint8_t *reg_data, uint16_t len, const struct bme280_dev *dev);

esp_err_t bme280_get_sensor_id(uint8_t *sensor_id, const struct bme280_dev *dev);

// TODO: bme280 set sensor settings
esp_err_t bme280_set_sensor_settings(const struct bme280_settings new_settings, const struct bme280_dev *dev);

// TODO: bme280 get sensor settings
esp_err_t bme280_get_sensor_settings(struct bme280_dev *dev);

// TODO: set bme280 sensor power mode
/*
 * sensor_mode |
 * ------------|-------------------
 *     1       | normal
 *     2       | sleep
 */
esp_err_t bme280_set_sensor_mode(uint8_t sensor_mode, const struct bme280_dev *dev);

// TODO: get bme280 sensor power mode
esp_err_t bme280_get_sensor_mode(uint8_t *sensor_mode, const struct bme280_dev *dev);

// TODO: get bme280 calibration data
esp_err_t get_calib_data(struct bme280_dev *dev);

// TODO: bme280 parse sensor data
void bme280_parse_sensor_data(const uint8_t *reg_data, struct bme280_uncomp_data *uncomp_data);

// TODO: bme280 compensate sensor data
esp_err_t bme280_compensate_data(uint8_t sensor_comp, const struct bme280_uncomp_data *uncomp_data,
                                 struct bme280_comp_data *comp_data, struct bme280_calib_data *calib_data);
// TODO: get bme280 sensor data
/*
 * sensor_comp_mode |
 * -----------------|-------------------
 *     1            | pressure
 *     2            | temperature
 *     4            | humidity
 *     7            | all
 */
esp_err_t bme280_get_sensor_data(uint8_t sensor_comp_mode, struct bme280_data *data, struct bme280_dev *dev);
#endif // __BME280_SENSOR_H__INCLUDED__