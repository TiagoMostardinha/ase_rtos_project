#include "bme280.h"
#include "sdspi.h"
#include "wifi.h"
#include "mqtt.h"
#include "servo.c"

#define MOTOR_GPIO 10

struct boat
{
    bool realtime;
    bool send_file;
    bool turn_motor;
    uint8_t servo_direction;
};

void app_main(void)
{
    struct boat boat = {
        .realtime = true,
        .send_file = false,
        .turn_motor = false,
        .servo_direction = 0};

    ESP_LOGI("BME280", "Starting BME280...");
    struct bme280_settings bme280_settings = {
        .osrs_t = BME280_OVERSAMPLING_2X,
        .osrs_p = BME280_OVERSAMPLING_1X,
        .osrs_h = BME280_OVERSAMPLING_1X,
        .standby_time = BME280_STANDBY_TIME_10_MS,
        .filter = BME280_FILTER_COEFF_2};

    struct bme280_dev dev = {
        .chip_id = BME280_CHIP_ID,
    };

    esp_err_t err;

    /*WIFI Configuration*/
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }

    wifi_init_sta();

    /*MQTT Configuration*/
    mqtt_init();

    /*Servo Configuration*/
    config_servo_init();

    /*Motor Configuration*/
    gpio_reset_pin(MOTOR_GPIO);
    gpio_set_direction(MOTOR_GPIO, GPIO_MODE_OUTPUT);

    /*BME280 Configuration*/

    ESP_LOGI("BME280", "Initializing BME280...");
    err = bme280_init(&dev, BME280_I2C_ADDRESS, 3, 2, 400000);
    if (err != ESP_OK)
    {
        ESP_LOGE("BME280", "BME280 init failed! (%x)", err);
        return;
    }

    ESP_LOGI("BME280", "Setting BME280 sensor...");
    err = bme280_set_sensor_settings(bme280_settings, &dev);
    if (err != ESP_OK)
    {
        ESP_LOGE("BME280", "BME280 set sensor settings failed. (%x)", err);
        return;
    }

    ESP_LOGI("BME280", "Getting BME280 calibration data...");
    err = get_calib_data(&dev);
    if (err != ESP_OK)
    {
        return;
    }

    vTaskDelay(100 / portTICK_PERIOD_MS);

    ESP_LOGI("BME280", "Setting BME280 in normal mode...");
    err = bme280_set_sensor_mode(BME280_NORMAL_MODE, &dev);
    if (err != ESP_OK)
    {
        ESP_LOGE("BME280", "BME280 set sensor mode failed. (%x)", err);
        return;
    }

    /*SDcard Configuration*/
    ESP_LOGI("SDSPI", "Initializing SD card...");
    const char mount_point[] = MOUNT_POINT;
    struct sdcard_dev reader = {
        .mount_point = mount_point,
    };
    err = sdcard_init(PIN_NUM_MOSI, PIN_NUM_MISO, PIN_NUM_CLK, &reader);
    if (err != ESP_OK)
    {
        ESP_LOGE("SDSPI", "Failed to initialize SD card. (%x)", err);
        return;
    }

    ESP_LOGI("SDSPI", "Mounting SD card...");
    err = sdcard_mount(PIN_NUM_CS, &reader);
    if (err != ESP_OK)
    {
        ESP_LOGE("SDSPI", "Failed to mount SD card. (%x)", err);
        return;
    }

    const char *file = MOUNT_POINT "/sensor.txt";

    vTaskDelay(1000 / portTICK_PERIOD_MS);

    /*Variables*/
    struct bme280_data data;

    while (1)
    {
        char *topic_boat = get_topic_data();

        // check if topic_boat is "realtime","sendfile","motor","servo0" or "servo2"
        if (strcmp(topic_boat, "realtime") == 0)
        {
            boat.realtime = true;
        }
        else if (strcmp(topic_boat, "realtime_off") == 0)
        {
            boat.realtime = false;
        }
        else if (strcmp(topic_boat, "sendfile") == 0)
        {
            boat.send_file = true;
        }
        else if (strcmp(topic_boat, "motor") == 0)
        {
            boat.turn_motor = true;
        }
        else if (strcmp(topic_boat, "motoroff") == 0)
        {
            boat.turn_motor = false;
        }
        else if (strcmp(topic_boat, "servo_middle") == 0)
        {
            boat.servo_direction = 0;
        }
        else if (strcmp(topic_boat, "servo_left") == 0)
        {
            boat.servo_direction = 1;
        }
        else if (strcmp(topic_boat, "servo_right") == 0)
        {
            boat.servo_direction = 2;
        }
        else if (strcmp(topic_boat, "boatoff") == 0){
            sdcard_unmount(&reader);
            sdcard_free(&reader);
            return;
        }

        /*Motor control*/
        if (boat.turn_motor)
        {
            ESP_LOGI("MOTOR", "Turning motor...");
            gpio_set_level(MOTOR_GPIO, 1);
        }else{
            gpio_set_level(MOTOR_GPIO, 0);}

        /*Servo control*/
        if (boat.servo_direction == 1 || boat.servo_direction == 2 || boat.servo_direction == 0)
        {
            ESP_LOGI("SERVO", "Turning servo...");
            if (boat.servo_direction == 1)
            {
                set_servo_angle(90);
            }
            else if (boat.servo_direction == 2)
            {
                set_servo_angle(-90);
            }
            else{
                set_servo_angle(0);
            }
        }

        /*Get sensor data*/

        err = bme280_get_sensor_data(0x07, &data, &dev);
        if (err != ESP_OK)
        {
            ESP_LOGE("BME280", "Could not get sensor data. (%x)", err);
            return;
        }

        ESP_LOGI("BME280", "Temperature: %.2f C, Pressure: %.2f Pa, Humidity: %.2f %%", data.temperature, data.pressure, data.humidity);

        if (boat.realtime)
        {
            ESP_LOGI("MQTT", "Sending data to MQTT broker...");
            // TODO: Send data to MQTT broker
            char temp[MAX_CHAR_SIZE];
            char pres[MAX_CHAR_SIZE];
            char hum[MAX_CHAR_SIZE];

            snprintf(temp, MAX_CHAR_SIZE, "%.2f",data.temperature);
            snprintf(pres, MAX_CHAR_SIZE, "%.2f",data.pressure);
            snprintf(hum, MAX_CHAR_SIZE, "%.2f",data.humidity);


            mqtt_publish(BOATTEMP, temp);
            mqtt_publish(BOATHUM, hum);
            mqtt_publish(BOATPRES, pres);
        }
        else
        {
            ESP_LOGI("SDSPI", "Writing data to file...");
            char buffer[MAX_CHAR_SIZE];
            snprintf(buffer, MAX_CHAR_SIZE, "temperature: %.2f pressure: %.2f humidity: %.2f \n", data.temperature, data.pressure, data.humidity);
            err = sdcard_write(file, buffer);
            if (err != ESP_OK)
            {
                ESP_LOGE("SDSPI", "Failed to write to file. (%x)", err);
                return;
            }
        }

        if (boat.send_file)
        {
            err = bme280_set_sensor_mode(BME280_SLEEP_MODE, &dev);

            FILE *f = fopen(file, "r");
            if (f == NULL)
            {
                return;
            }

            mqtt_publish(BOATFILE, "START");

            char line[MAX_CHAR_SIZE];
            while (fgets(line, MAX_CHAR_SIZE, f) != NULL)
            {
                char *pos = strchr(line, '\n');
                if (pos)
                {
                    *pos = '\0';
                }

                mqtt_publish(BOATFILE, line);

                vTaskDelay(10 / portTICK_PERIOD_MS);
            }
            fclose(f);

            mqtt_publish(BOATFILE, "END");

            boat.send_file = false;
            err = bme280_set_sensor_mode(BME280_NORMAL_MODE, &dev);
            vTaskDelay(10 / portTICK_PERIOD_MS);
        }

        /*Wait 100 ms*/
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
