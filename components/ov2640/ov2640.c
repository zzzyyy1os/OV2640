/*
 * ov2640.c - OV2640 camera sensor driver (SCCB / register configuration)
 *
 * ESP-IDF port of the OV2640 driver from the STM32 reference project
 * (STM32-OV2640-master, SimpleMethod, MIT):
 *   - HAL_I2C_Master_Transmit/Receive -> esp_driver_i2c (i2c_master)
 *   - HAL_GPIO reset pulse            -> esp_driver_gpio
 *   - HAL_Delay                       -> vTaskDelay
 * Capture (formerly DCMI) is implemented by the "camera" component using the
 * official esp_driver_cam (DVP) controller.
 */

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_check.h"
#include "ov2640.h"
#include "ov2640_regs.h"

static const char *TAG = "ov2640";

#define SCCB_TIMEOUT_MS     100
#define SCCB_WRITE_RETRIES  3

struct ov2640_t {
    i2c_master_bus_handle_t bus;
    i2c_master_dev_handle_t dev;
    gpio_num_t pwdn_io;
    gpio_num_t reset_io;
};

/* ------------------------------------------------------------------ */
/* SCCB primitives (SCCB_Write / SCCB_Read ports)                      */
/* ------------------------------------------------------------------ */

esp_err_t ov2640_write_reg(ov2640_handle_t dev, uint8_t reg, uint8_t data)
{
    ESP_RETURN_ON_FALSE(dev, ESP_ERR_INVALID_ARG, TAG, "null handle");
    uint8_t buf[2] = { reg, data };
    /* Retry: long register tables on a marginal bus can see the occasional
     * aborted transfer (NACK / bus glitch) - give it another go */
    esp_err_t err = ESP_FAIL;
    for (int i = 0; i < SCCB_WRITE_RETRIES; i++) {
        err = i2c_master_transmit(dev->dev, buf, sizeof(buf), pdMS_TO_TICKS(SCCB_TIMEOUT_MS));
        if (err == ESP_OK) {
            return ESP_OK;
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    return err;
}

esp_err_t ov2640_read_reg(ov2640_handle_t dev, uint8_t reg, uint8_t *pdata)
{
    ESP_RETURN_ON_FALSE(dev && pdata, ESP_ERR_INVALID_ARG, TAG, "null pointer");
    return i2c_master_transmit_receive(dev->dev, &reg, 1, pdata, 1, pdMS_TO_TICKS(SCCB_TIMEOUT_MS));
}

esp_err_t ov2640_write_reg_list(ov2640_handle_t dev, const uint8_t arr[][2])
{
    ESP_RETURN_ON_FALSE(dev && arr, ESP_ERR_INVALID_ARG, TAG, "null pointer");

    size_t i;
    for (i = 0;; i++) {
        uint8_t reg = arr[i][0];
        uint8_t val = arr[i][1];
        if (reg == 0xff && val == 0xff) {           /* end-of-table sentinel */
            break;
        }
        esp_err_t err = ov2640_write_reg(dev, reg, val);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "SCCB write failed: reg 0x%02x (err %s)", reg, esp_err_to_name(err));
            return err;
        }
        vTaskDelay(pdMS_TO_TICKS(1));               /* short settle, ref used 10 ms */

        /* Read-back verification (OV2640_Configuration port) */
        uint8_t check = 0;
        if (ov2640_read_reg(dev, reg, &check) == ESP_OK && check != val) {
            ESP_LOGW(TAG, "verify mismatch: reg 0x%02x wrote 0x%02x read 0x%02x", reg, val, check);
        }
    }
    ESP_LOGI(TAG, "register table applied: %u entries", (unsigned)i);
    return ESP_OK;
}

/* ------------------------------------------------------------------ */
/* Lifecycle                                                           */
/* ------------------------------------------------------------------ */

esp_err_t ov2640_new(const ov2640_config_t *config, ov2640_handle_t *ret_handle)
{
    ESP_RETURN_ON_FALSE(config && ret_handle, ESP_ERR_INVALID_ARG, TAG, "null pointer");

    struct ov2640_t *dev = calloc(1, sizeof(struct ov2640_t));
    ESP_RETURN_ON_FALSE(dev, ESP_ERR_NO_MEM, TAG, "no mem");
    dev->pwdn_io = config->pwdn_io;
    dev->reset_io = config->reset_io;

    /* PWDN: keep the sensor out of power-down (active high on OV2640) */
    if (dev->pwdn_io != GPIO_NUM_NC) {
        gpio_config_t io = {
            .pin_bit_mask = 1ULL << dev->pwdn_io,
            .mode = GPIO_MODE_OUTPUT,
        };
        ESP_ERROR_CHECK(gpio_config(&io));
        gpio_set_level(dev->pwdn_io, 0);
    }

    /* Hardware reset pulse (reference: low 100 ms, high 100 ms) */
    if (dev->reset_io != GPIO_NUM_NC) {
        gpio_config_t io = {
            .pin_bit_mask = 1ULL << dev->reset_io,
            .mode = GPIO_MODE_OUTPUT,
        };
        ESP_ERROR_CHECK(gpio_config(&io));
        gpio_set_level(dev->reset_io, 0);
        vTaskDelay(pdMS_TO_TICKS(100));
        gpio_set_level(dev->reset_io, 1);
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    /* SCCB bus (official esp_driver_i2c) */
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = config->i2c_port,
        .sda_io_num = config->sda_io_num,
        .scl_io_num = config->scl_io_num,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    esp_err_t ret = i2c_new_master_bus(&bus_cfg, &dev->bus);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "i2c bus init failed: %s", esp_err_to_name(ret));
        goto fail;
    }

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = OV2640_SCCB_ADDR,
        .scl_speed_hz = config->sccb_freq_hz ? config->sccb_freq_hz : 100000,
    };
    ret = i2c_master_bus_add_device(dev->bus, &dev_cfg, &dev->dev);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "i2c add device failed: %s", esp_err_to_name(ret));
        goto fail;
    }

    /* Software reset: select sensor bank, reset all registers to default */
    ESP_GOTO_ON_ERROR(ov2640_write_reg(dev, 0xff, 0x01), fail, TAG, "sccb write failed");
    ESP_GOTO_ON_ERROR(ov2640_write_reg(dev, 0x12, 0x80), fail, TAG, "sccb write failed");
    vTaskDelay(pdMS_TO_TICKS(100));

    /* Identification (reference DEBUG block: PID 0x26, VER 0x42) */
    uint8_t pid = 0, ver = 0;
    ret = ov2640_read_id(dev, &pid, &ver);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "sensor does not respond on SCCB (%s)", esp_err_to_name(ret));
        goto fail;
    }
    if (pid != OV2640_PID_VALUE || ver != OV2640_VER_VALUE) {
        ESP_LOGW(TAG, "unexpected ID: PID 0x%02x VER 0x%02x (expected 0x26/0x42)", pid, ver);
    } else {
        ESP_LOGI(TAG, "OV2640 detected: PID 0x%02x, VER 0x%02x", pid, ver);
    }

    *ret_handle = dev;
    return ESP_OK;

fail:
    ov2640_del(dev);
    return ret;
}

esp_err_t ov2640_del(ov2640_handle_t dev)
{
    ESP_RETURN_ON_FALSE(dev, ESP_ERR_INVALID_ARG, TAG, "null handle");
    if (dev->dev) {
        i2c_master_bus_rm_device(dev->dev);
    }
    if (dev->bus) {
        i2c_del_master_bus(dev->bus);
    }
    free(dev);
    return ESP_OK;
}

esp_err_t ov2640_read_id(ov2640_handle_t dev, uint8_t *pid, uint8_t *ver)
{
    ESP_RETURN_ON_FALSE(dev, ESP_ERR_INVALID_ARG, TAG, "null handle");
    ESP_RETURN_ON_ERROR(ov2640_write_reg(dev, 0xff, 0x01), TAG, "select bank failed");
    if (pid) {
        ESP_RETURN_ON_ERROR(ov2640_read_reg(dev, 0x0a, pid), TAG, "read pid failed");
    }
    if (ver) {
        ESP_RETURN_ON_ERROR(ov2640_read_reg(dev, 0x0b, ver), TAG, "read ver failed");
    }
    return ESP_OK;
}

/* ------------------------------------------------------------------ */
/* JPEG pipeline and resolution (OV2640_ResolutionConfiguration port)  */
/* ------------------------------------------------------------------ */

static esp_err_t set_resolution(ov2640_handle_t dev, ov2640_res_t res, bool jpeg)
{
    ESP_RETURN_ON_FALSE(dev, ESP_ERR_INVALID_ARG, TAG, "null handle");

    ESP_RETURN_ON_ERROR(ov2640_write_reg_list(dev, OV2640_JPEG_INIT), TAG, "JPEG_INIT failed");
    if (jpeg) {
        /* Reference chain: YUV422 + JPEG switch the DSP output to a JPEG
         * stream (IMAGE_MODE 0xDA = 0x10 - "YUV422" here only describes the
         * sensor array feeding the DSP). Needed for JPEG output, harmful for
         * raw frames, so the raw path below skips both tables: JPEG_INIT's
         * tail already leaves the sensor in the raw-ready state. */
        ESP_RETURN_ON_ERROR(ov2640_write_reg_list(dev, OV2640_YUV422), TAG, "YUV422 failed");
        ESP_RETURN_ON_ERROR(ov2640_write_reg_list(dev, OV2640_JPEG), TAG, "JPEG failed");
    }

    vTaskDelay(pdMS_TO_TICKS(10));
    ESP_RETURN_ON_ERROR(ov2640_write_reg(dev, 0xff, 0x01), TAG, "select bank failed");
    vTaskDelay(pdMS_TO_TICKS(10));
    ESP_RETURN_ON_ERROR(ov2640_write_reg(dev, 0x15, 0x00), TAG, "set 0x15 failed");

    const uint8_t (*table)[2];
    switch (res) {
    case OV2640_RES_160X120:
        table = OV2640_160x120_JPEG;
        break;
    case OV2640_RES_320X240:
        table = OV2640_320x240_JPEG;
        break;
    case OV2640_RES_640X480:
        table = OV2640_640x480_JPEG;
        break;
    case OV2640_RES_800X600:
        table = OV2640_800x600_JPEG;
        break;
    case OV2640_RES_1024X768:
        table = OV2640_1024x768_JPEG;
        break;
    case OV2640_RES_1280X960:
        table = OV2640_1280x960_JPEG;
        break;
    default:
        return ESP_ERR_INVALID_ARG;
    }
    ESP_RETURN_ON_ERROR(ov2640_write_reg_list(dev, table), TAG, "resolution table failed");

    if (!jpeg) {
        /* Raw YUV422 frames on the DVP bus. The JPEG resolution tables size
         * the output for the JPEG encoder (176x144 for "160x120"), so force
         * the exact raw frame size - the official esp_cam_sensor driver does
         * the same in ov2640_set_outsize(). OV2640_YUV422_RAW flips
         * IMAGE_MODE to raw YUV422 (0x00) like the official
         * ov2640_settings_yuv422. */
        uint32_t w = 0, h = 0;
        ESP_RETURN_ON_ERROR(ov2640_get_jpeg_resolution_size(res, &w, &h), TAG, "bad resolution");
        ESP_RETURN_ON_ERROR(ov2640_write_reg_list(dev, OV2640_YUV422_RAW), TAG, "raw output mode failed");

        uint16_t outw = w / 4, outh = h / 4;    /* output size unit is 4 pixels */
        uint8_t msb = ((outw >> 8) & 0x03) | ((outh >> 6) & 0x04);
        ESP_RETURN_ON_ERROR(ov2640_write_reg(dev, 0xe0, 0x04), TAG, "raw outsize failed");  /* RESET_DVP */
        ESP_RETURN_ON_ERROR(ov2640_write_reg(dev, 0x5a, (uint8_t)(outw & 0xff)), TAG, "raw outsize failed");
        ESP_RETURN_ON_ERROR(ov2640_write_reg(dev, 0x5b, (uint8_t)(outh & 0xff)), TAG, "raw outsize failed");
        ESP_RETURN_ON_ERROR(ov2640_write_reg(dev, 0x5c, msb), TAG, "raw outsize failed");
        ESP_RETURN_ON_ERROR(ov2640_write_reg(dev, 0xe0, 0x00), TAG, "raw outsize failed");  /* release */
        ESP_LOGI(TAG, "raw YUV422 output enabled: %ux%u (IMAGE_MODE=0x00)", (unsigned)w, (unsigned)h);
    }
    return ESP_OK;
}

esp_err_t ov2640_set_jpeg_resolution(ov2640_handle_t dev, ov2640_res_t res)
{
    return set_resolution(dev, res, true);
}

esp_err_t ov2640_set_yuv422_resolution(ov2640_handle_t dev, ov2640_res_t res)
{
    /* Base init + resolution table without the JPEG encoder stage, then
     * switch the DVP output to raw YUV422 at the exact frame size (see the
     * raw patch at the end of set_resolution). */
    return set_resolution(dev, res, false);
}

esp_err_t ov2640_get_jpeg_resolution_size(ov2640_res_t res, uint32_t *width, uint32_t *height)
{
    static const uint16_t dims[][2] = {
        [OV2640_RES_160X120]  = { 160, 120 },
        [OV2640_RES_320X240]  = { 320, 240 },
        [OV2640_RES_640X480]  = { 640, 480 },
        [OV2640_RES_800X600]  = { 800, 600 },
        [OV2640_RES_1024X768] = { 1024, 768 },
        [OV2640_RES_1280X960] = { 1280, 960 },
    };
    if ((unsigned)res >= sizeof(dims) / sizeof(dims[0])) {
        return ESP_ERR_INVALID_ARG;
    }
    if (width) {
        *width = dims[res][0];
    }
    if (height) {
        *height = dims[res][1];
    }
    return ESP_OK;
}

/* ------------------------------------------------------------------ */
/* Image controls                                                      */
/* ------------------------------------------------------------------ */

static const uint8_t (*const s_contrast_tables[5])[2] = {
    /* index = level + 2 : -2, -1, 0, +1, +2 */
    OV2640_CONTRAST_2, OV2640_CONTRAST_1, OV2640_CONTRAST0, OV2640_CONTRAST1, OV2640_CONTRAST2,
};

static const uint8_t (*const s_saturation_tables[5])[2] = {
    OV2640_SATURATION_2, OV2640_SATURATION_1, OV2640_SATURATION0, OV2640_SATURATION1, OV2640_SATURATION2,
};

static const uint8_t (*const s_brightness_tables[5])[2] = {
    OV2640_BRIGHTNESS_2, OV2640_BRIGHTNESS_1, OV2640_BRIGHTNESS0, OV2640_BRIGHTNESS1, OV2640_BRIGHTNESS2,
};

static esp_err_t apply_level_table(ov2640_handle_t dev, const uint8_t (*const *tables)[2], int level, const char *what)
{
    if (level < -2 || level > 2) {
        ESP_LOGE(TAG, "%s level %d out of range (-2..+2)", what, level);
        return ESP_ERR_INVALID_ARG;
    }
    return ov2640_write_reg_list(dev, tables[level + 2]);
}

esp_err_t ov2640_set_contrast(ov2640_handle_t dev, int level)
{
    return apply_level_table(dev, s_contrast_tables, level, "contrast");
}

esp_err_t ov2640_set_saturation(ov2640_handle_t dev, int level)
{
    return apply_level_table(dev, s_saturation_tables, level, "saturation");
}

esp_err_t ov2640_set_brightness(ov2640_handle_t dev, int level)
{
    return apply_level_table(dev, s_brightness_tables, level, "brightness");
}

esp_err_t ov2640_set_effect(ov2640_handle_t dev, ov2640_effect_t effect)
{
    const uint8_t (*table)[2];
    switch (effect) {
    case OV2640_EFFECT_ANTIQUE:
        table = OV2640_SPECIAL_EFFECTS_ANTIQUE;
        break;
    case OV2640_EFFECT_BLUISH:
        table = OV2640_SPECIAL_EFFECTS_BLUISH;
        break;
    case OV2640_EFFECT_GREENISH:
        table = OV2640_SPECIAL_EFFECTS_GREENISH;
        break;
    case OV2640_EFFECT_REDDISH:
        table = OV2640_SPECIAL_EFFECTS_REDDISH;
        break;
    case OV2640_EFFECT_BLACKWHITE:
        table = OV2640_SPECIAL_EFFECTS_BLACK;
        break;
    case OV2640_EFFECT_NEGATIVE:
        table = OV2640_SPECIAL_EFFECTS_NEGATIVE;
        break;
    case OV2640_EFFECT_NEGATIVE_BLACKWHITE:
        table = OV2640_SPECIAL_EFFECTS_BLACK_NEGATIVE;
        break;
    case OV2640_EFFECT_NORMAL:
        table = OV2640_SPECIAL_EFFECTS_NORMAL;
        break;
    default:
        return ESP_ERR_INVALID_ARG;
    }
    return ov2640_write_reg_list(dev, table);
}

esp_err_t ov2640_set_light_mode(ov2640_handle_t dev, ov2640_light_mode_t mode)
{
    const uint8_t (*table)[2];
    switch (mode) {
    case OV2640_LIGHT_AUTO:
        /* Advanced white balance (OV2640_LightMode Auto port) */
        return ov2640_set_white_balance(dev, false);
    case OV2640_LIGHT_SUNNY:
        table = OV2640_LIGHT_MODE_SUNNY;
        break;
    case OV2640_LIGHT_CLOUDY:
        table = OV2640_LIGHT_MODE_CLOUDY;
        break;
    case OV2640_LIGHT_OFFICE:
        table = OV2640_LIGHT_MODE_OFFICE;
        break;
    case OV2640_LIGHT_HOME:
        table = OV2640_LIGHT_MODE_HOME;
        break;
    default:
        return ESP_ERR_INVALID_ARG;
    }
    return ov2640_write_reg_list(dev, table);
}

esp_err_t ov2640_set_white_balance(ov2640_handle_t dev, bool simple)
{
    ESP_RETURN_ON_FALSE(dev, ESP_ERR_INVALID_ARG, TAG, "null handle");
    ESP_RETURN_ON_ERROR(ov2640_write_reg(dev, 0xff, 0x00), TAG, "select bank failed");
    vTaskDelay(pdMS_TO_TICKS(1));
    return ov2640_write_reg(dev, 0xc7, simple ? 0x10 : 0x00);
}
