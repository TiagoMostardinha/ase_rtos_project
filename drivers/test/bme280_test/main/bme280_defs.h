#include <stdint.h>
#include <stddef.h>
#include "driver/i2c_types.h"

#ifndef BME280_SENSOR_DEFS_H__INCLUDED__
#define BME280_SENSOR_DEFS_H__INCLUDED__

/*BME280 chip id*/
#define BME280_CHIP_ID 0x60

/*BME280 I2C address*/
#define BME280_I2C_ADDRESS 0x77

/*BME280 sensor power modes*/
#define BME280_SLEEP_MODE UINT8_C(0x00)
#define BME280_FORCED_MODE UINT8_C(0x01)
#define BME280_NORMAL_MODE UINT8_C(0x03)

/*BME280 register address from memory map*/
#define BME280_CHIP_ID_ADDR UINT8_C(0xD0)
#define BME280_RESET_ADDR UINT8_C(0xE0)
#define BME280_TEMP_PRESS_CALIB_DATA_ADDR UINT8_C(0x88)
#define BME280_HUMIDITY_CALIB_DATA_ADDR UINT8_C(0xE1)
#define BME280_PWR_CTRL_ADDR UINT8_C(0xF4)
#define BME280_CTRL_HUM_ADDR UINT8_C(0xF2)
#define BME280_CTRL_MEAS_ADDR UINT8_C(0xF4)
#define BME280_CONFIG_ADDR UINT8_C(0xF5)
#define BME280_DATA_ADDR UINT8_C(0xF7)

/*BME280 calibration data length*/
#define BME280_TEMP_PRESS_CALIB_DATA_LEN UINT8_C(26)
#define BME280_HUMIDITY_CALIB_DATA_LEN UINT8_C(7)
#define BME280_P_T_H_DATA_LEN UINT8_C(8)

/*BME280 oversampling macros*/
#define BME280_NO_OVERSAMPLING UINT8_C(0x00)
#define BME280_OVERSAMPLING_1X UINT8_C(0x01)
#define BME280_OVERSAMPLING_2X UINT8_C(0x02)
#define BME280_OVERSAMPLING_4X UINT8_C(0x03)
#define BME280_OVERSAMPLING_8X UINT8_C(0x04)
#define BME280_OVERSAMPLING_16X UINT8_C(0x05)

/*Standby duration selection macros */
#define BME280_STANDBY_TIME_1_MS (0x00)
#define BME280_STANDBY_TIME_62_5_MS (0x01)
#define BME280_STANDBY_TIME_125_MS (0x02)
#define BME280_STANDBY_TIME_250_MS (0x03)
#define BME280_STANDBY_TIME_500_MS (0x04)
#define BME280_STANDBY_TIME_1000_MS (0x05)
#define BME280_STANDBY_TIME_10_MS (0x06)
#define BME280_STANDBY_TIME_20_MS (0x07)

/*Filter coefficient selection macros */
#define BME280_FILTER_COEFF_OFF (0x00)
#define BME280_FILTER_COEFF_2 (0x01)
#define BME280_FILTER_COEFF_4 (0x02)
#define BME280_FILTER_COEFF_8 (0x03)
#define BME280_FILTER_COEFF_16 (0x04)

/*BME280 delay function pointer*/
typedef void (*bme280_delay_fptr_t)(uint32_t period);

/*BME280 Macros*/
#define BME280_CONCAT_BYTES(msb, lsb) (((uint16_t)msb << 8) | (uint16_t)lsb)

/*BME280 sensor settings*/
struct bme280_settings
{
    uint8_t osrs_t;
    uint8_t osrs_p;
    uint8_t osrs_h;
    uint8_t standby_time;
    uint8_t filter;
};

/*BME280 uncompensated data*/
struct bme280_uncomp_data
{
    uint32_t pressure;
    uint32_t temperature;
    uint32_t humidity;
};

/*BME280 compensated data*/
struct bme280_comp_data
{
    uint32_t pressure;
    int32_t temperature;
    uint32_t humidity;
};

struct bme280_data
{
    float pressure;
    float temperature;
    float humidity;
};


/*BME280 calibration data*/
struct bme280_calib_data
{
    uint16_t dig_T1;
    int16_t dig_T2;
    int16_t dig_T3;
    uint16_t dig_P1;
    int16_t dig_P2;
    int16_t dig_P3;
    int16_t dig_P4;
    int16_t dig_P5;
    int16_t dig_P6;
    int16_t dig_P7;
    int16_t dig_P8;
    int16_t dig_P9;
    uint8_t dig_H1;
    int16_t dig_H2;
    uint8_t dig_H3;
    int16_t dig_H4;
    int16_t dig_H5;
    int8_t dig_H6;
    int32_t t_fine;
};

/*BME280 device structure*/
struct bme280_dev
{
    uint8_t chip_id;
    uint8_t dev_id;
    i2c_master_bus_handle_t i2c_bus_handle;
    i2c_master_dev_handle_t i2c_dev_handle;
    struct bme280_calib_data calib_data;
    bme280_delay_fptr_t delay_ms;
    struct bme280_settings settings;
};

#endif // BME280_DEFS_H__INCLUDED__