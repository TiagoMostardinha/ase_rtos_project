#include <string.h>
#include <sys/unistd.h>
#include <sys/stat.h>
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "esp_err.h"

#define MOUNT_POINT "/sdcard"
#define PIN_NUM_MISO UINT8_C(0x06)
#define PIN_NUM_MOSI UINT8_C(0x04)
#define PIN_NUM_CLK UINT8_C(0x05)
#define PIN_NUM_CS UINT8_C(0x01)
#define MAX_CHAR_SIZE UINT16_C(256)

struct sdcard_dev
{
    const char *mount_point;
    sdmmc_card_t *card;
    sdmmc_host_t host;
};

esp_err_t sdcard_init(uint8_t mosi, uint8_t miso, uint8_t clk, struct sdcard_dev *reader);

esp_err_t sdcard_free(struct sdcard_dev *reader);

esp_err_t sdcard_mount(uint8_t cs, struct sdcard_dev *reader);

esp_err_t sdcard_unmount(struct sdcard_dev *reader);

esp_err_t sdcard_write(const char *file, const char *data);