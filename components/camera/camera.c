/*
 * camera.c - OV2640 DVP capture module for ESP32-S3
 *
 * Capture engine: official esp_driver_cam (esp_cam_ctlr DVP, LCD_CAM based).
 * JPEG handling: the driver reports the real JPEG length via received_size
 * (SOI 0xFFD8 / EOI 0xFFD9 scan). A fallback marker scan - ported from the
 * STM32 reference main.c - covers the case where dummy bytes precede the
 * JPEG SOI at frame start.
 */

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_heap_caps.h"
#include "esp_cache.h"
#include "esp_memory_utils.h"
#include "esp_cam_ctlr.h"
#include "esp_cam_ctlr_dvp.h"
#include "camera.h"
#include "camera_pins.h"

static const char *TAG = "camera";

/* ---- esp_cache_msync shim (linked via -Wl,--wrap=esp_cache_msync) ---- *
 *
 * esp_cam_ctlr_dvp calls esp_cache_msync() on the frame buffer right
 * before starting DMA. In IDF 5.5 esp_cache_msync() only accepts
 * cache-mapped vaddrs (flash/PSRAM: the 0x3C/0x42 ranges on the S3), so
 * a frame buffer in internal SRAM - the only DMA-capable RAM on a board
 * without PSRAM - is rejected with "invalid addr or null pointer" and
 * esp_cam_ctlr_start() fails. The official dvp_spi_lcd example dodges
 * this by requiring PSRAM (its sdkconfig.defaults.esp32s3 sets
 * CONFIG_SPIRAM=y).
 *
 * Internal SRAM is not accessed through the cache on the S3 - CPU and
 * DMA are always coherent there - so a cache sync for those addresses is
 * a no-op. Skip it and forward everything else to the real function. */
esp_err_t __real_esp_cache_msync(void *addr, size_t size, int flags);

esp_err_t __wrap_esp_cache_msync(void *addr, size_t size, int flags)
{
    if (addr && (esp_ptr_in_dram(addr) || esp_ptr_in_iram(addr))) {
        return ESP_OK;
    }
    return __real_esp_cache_msync(addr, size, flags);
}

/* How far into the frame the JPEG SOI may start (fallback scan, ref scans unbounded) */
#define JPEG_SOI_SEARCH_BYTES   256

typedef struct {
    const uint8_t *buf;
    size_t len;
    uint32_t seq;
} cam_frame_msg_t;

static struct {
    esp_cam_ctlr_handle_t ctlr;
    ov2640_handle_t sensor;
    camera_output_t output;
    void **fb;                  /* frame buffer array */
    size_t fb_count;
    size_t fb_size;
    volatile uint32_t next_fb;  /* round-robin index for on_get_new_trans */
    QueueHandle_t frame_q;
    uint32_t seq;
    volatile uint32_t drop_count;
    uint32_t last_seen_drop;
    bool inited;
} s_cam;

/* ------------------------------------------------------------------ */
/* DVP event callbacks (run in ISR context)                            */
/* ------------------------------------------------------------------ */

/* Fallback JPEG size detection: forward scan for SOI, then EOI.
 * Port of the FF D8 / FF D9 search from the STM32 reference main.c. */
static size_t scan_jpeg_size(const uint8_t *buf, size_t cap, size_t *soi_off)
{
    size_t start = SIZE_MAX;
    for (size_t i = 0; i + 1 < cap && i < JPEG_SOI_SEARCH_BYTES; i++) {
        if (buf[i] == 0xff && buf[i + 1] == 0xd8) {
            start = i;
            break;
        }
    }
    if (start == SIZE_MAX) {
        return 0;
    }
    for (size_t i = start + 2; i + 1 < cap; i++) {
        if (buf[i] == 0xff && buf[i + 1] == 0xd9) {
            *soi_off = start;
            return i + 2 - start;
        }
    }
    return 0;
}

static bool IRAM_ATTR on_get_new_trans(esp_cam_ctlr_handle_t handle, esp_cam_ctlr_trans_t *trans, void *user_data)
{
    trans->buffer = s_cam.fb[s_cam.next_fb];
    trans->buflen = s_cam.fb_size;
    s_cam.next_fb = (s_cam.next_fb + 1) % s_cam.fb_count;
    return false;
}

static bool IRAM_ATTR on_trans_finished(esp_cam_ctlr_handle_t handle, esp_cam_ctlr_trans_t *trans, void *user_data)
{
    const uint8_t *frame = trans->buffer;
    size_t len = trans->received_size;      /* raw mode: full frame; JPEG: SOI..EOI */

    if (len == 0 && s_cam.output == CAMERA_OUTPUT_JPEG) {
        /* Driver found no markers at byte 0 - retry with the reference's
         * forward scan in case the SOI is not at the very first byte. */
        size_t soi_off = 0;
        len = scan_jpeg_size(trans->buffer, trans->buflen, &soi_off);
        frame += soi_off;
    }

    if (len == 0) {
        s_cam.drop_count++;                 /* no valid data in this frame */
        return false;
    }

    cam_frame_msg_t msg = { .buf = frame, .len = len, .seq = ++s_cam.seq };
    BaseType_t woken = pdFALSE;
    if (xQueueSendFromISR(s_cam.frame_q, &msg, &woken) != pdTRUE) {
        s_cam.drop_count++;                 /* consumer too slow, drop the frame */
    }
    return woken == pdTRUE;
}

/* ------------------------------------------------------------------ */
/* Public API                                                          */
/* ------------------------------------------------------------------ */

esp_err_t camera_default_config(camera_config_t *cfg)
{
    ESP_RETURN_ON_FALSE(cfg, ESP_ERR_INVALID_ARG, TAG, "null pointer");
    memset(cfg, 0, sizeof(*cfg));
    cfg->i2c_port = CAM_I2C_PORT;
    cfg->sda_io_num = CAM_PIN_SIOD;
    cfg->scl_io_num = CAM_PIN_SIOC;
    cfg->pwdn_io = CAM_PIN_PWDN;
    cfg->reset_io = CAM_PIN_RESET;
    cfg->xclk_io = CAM_PIN_XCLK;
    cfg->pclk_io = CAM_PIN_PCLK;
    cfg->vsync_io = CAM_PIN_VSYNC;
    cfg->href_io = CAM_PIN_HREF;
    cfg->data_io[0] = CAM_PIN_D0;
    cfg->data_io[1] = CAM_PIN_D1;
    cfg->data_io[2] = CAM_PIN_D2;
    cfg->data_io[3] = CAM_PIN_D3;
    cfg->data_io[4] = CAM_PIN_D4;
    cfg->data_io[5] = CAM_PIN_D5;
    cfg->data_io[6] = CAM_PIN_D6;
    cfg->data_io[7] = CAM_PIN_D7;
    cfg->xclk_freq_hz = CAM_XCLK_FREQ_HZ;
    cfg->output = CAMERA_OUTPUT_JPEG;
    cfg->resolution = OV2640_RES_320X240;
    cfg->frame_buffer_count = 2;
    return ESP_OK;
}

esp_err_t camera_init(const camera_config_t *cfg)
{
    ESP_RETURN_ON_FALSE(cfg && !s_cam.inited, ESP_ERR_INVALID_STATE, TAG, "bad state");
    ESP_LOGI(TAG, "camera clock: %s", cfg->xclk_io == GPIO_NUM_NC ?
             "external crystal (XCLK not driven)" : "XCLK generated on gpio");
    memset(&s_cam, 0, sizeof(s_cam));

    uint32_t w = 0, h = 0;
    ESP_RETURN_ON_ERROR(ov2640_get_jpeg_resolution_size(cfg->resolution, &w, &h), TAG, "bad resolution");
    s_cam.output = cfg->output;
    /* JPEG budget: 1 byte per "pixel"; raw YUV422: 2 bytes per pixel */
    s_cam.fb_size = (size_t)w * h * (cfg->output == CAMERA_OUTPUT_JPEG ? 1 : 2);
    s_cam.fb_count = cfg->frame_buffer_count ? cfg->frame_buffer_count : 2;

    /* ---- DVP controller (starts XCLK) ---- */
    esp_cam_ctlr_dvp_pin_config_t pin_cfg = {
        .data_width = CAM_CTLR_DATA_WIDTH_8,
        .data_io = {
            cfg->data_io[0], cfg->data_io[1], cfg->data_io[2], cfg->data_io[3],
            cfg->data_io[4], cfg->data_io[5], cfg->data_io[6], cfg->data_io[7],
        },
        .vsync_io = cfg->vsync_io,
        .de_io = cfg->href_io,
        .pclk_io = cfg->pclk_io,
        .xclk_io = cfg->xclk_io,
    };

    esp_cam_ctlr_dvp_config_t dvp_cfg = {
        .ctlr_id = 0,
        .clk_src = CAM_CLK_SRC_DEFAULT,
        .h_res = w,
        .v_res = h,
        .input_data_color_type = CAM_CTLR_COLOR_YUV422,   /* bit depth in raw mode */
        .cam_data_width = 8,
        .bk_buffer_dis = 1,                 /* we manage the frame buffers */
        .pic_format_jpeg = (cfg->output == CAMERA_OUTPUT_JPEG),
        .external_xtal = (cfg->xclk_io == GPIO_NUM_NC),  /* module with own oscillator */
        .dma_burst_size = 64,
        .xclk_freq = cfg->xclk_freq_hz ? cfg->xclk_freq_hz : CAM_XCLK_FREQ_HZ,
        .pin = &pin_cfg,
    };

    esp_err_t ret = esp_cam_new_dvp_ctlr(&dvp_cfg, &s_cam.ctlr);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "DVP controller init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    /* ---- Frame buffers ---- */
#if CONFIG_SPIRAM
    uint32_t caps = MALLOC_CAP_SPIRAM | MALLOC_CAP_DMA;
#else
    uint32_t caps = MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA;
#endif
    s_cam.fb = calloc(s_cam.fb_count, sizeof(void *));
    ESP_GOTO_ON_FALSE(s_cam.fb, ESP_ERR_NO_MEM, fail, TAG, "no mem");
    for (size_t i = 0; i < s_cam.fb_count; i++) {
        s_cam.fb[i] = esp_cam_ctlr_alloc_buffer(s_cam.ctlr, s_cam.fb_size, caps);
        ESP_GOTO_ON_FALSE(s_cam.fb[i], ESP_ERR_NO_MEM, fail, TAG, "fb alloc failed (%u bytes)", (unsigned)s_cam.fb_size);
    }

    /* ---- Sensor (SCCB) - XCLK is running now ---- */
    ov2640_config_t sensor_cfg = {
        .i2c_port = cfg->i2c_port,
        .sda_io_num = cfg->sda_io_num,
        .scl_io_num = cfg->scl_io_num,
        .pwdn_io = cfg->pwdn_io,
        .reset_io = cfg->reset_io,
    };
    ESP_GOTO_ON_ERROR(ov2640_new(&sensor_cfg, &s_cam.sensor), fail, TAG, "ov2640 init failed");
    if (cfg->output == CAMERA_OUTPUT_JPEG) {
        ESP_GOTO_ON_ERROR(ov2640_set_jpeg_resolution(s_cam.sensor, cfg->resolution), fail,
                          TAG, "ov2640 config failed");
    } else {
        ESP_GOTO_ON_ERROR(ov2640_set_yuv422_resolution(s_cam.sensor, cfg->resolution), fail,
                          TAG, "ov2640 config failed");
    }

    /* ---- Wire up capture callbacks ---- */
    s_cam.frame_q = xQueueCreate(s_cam.fb_count, sizeof(cam_frame_msg_t));
    ESP_GOTO_ON_FALSE(s_cam.frame_q, ESP_ERR_NO_MEM, fail, TAG, "no mem");

    esp_cam_ctlr_evt_cbs_t cbs = {
        .on_get_new_trans = on_get_new_trans,
        .on_trans_finished = on_trans_finished,
    };
    ESP_GOTO_ON_ERROR(esp_cam_ctlr_register_event_callbacks(s_cam.ctlr, &cbs, NULL), fail,
                      TAG, "callback register failed");

    /* Anchor log: frame buffer addresses (internal SRAM = 0x3FC8xxxx,
     * PSRAM = 0x3Cxxxxxx) - useful when start_trans fails in msync */
    for (size_t i = 0; i < s_cam.fb_count; i++) {
        ESP_LOGI(TAG, "fb[%u] @ %p (%u bytes)", (unsigned)i, s_cam.fb[i], (unsigned)s_cam.fb_size);
    }
    ESP_GOTO_ON_ERROR(esp_cam_ctlr_enable(s_cam.ctlr), fail, TAG, "enable failed");
    ESP_GOTO_ON_ERROR(esp_cam_ctlr_start(s_cam.ctlr), fail, TAG, "start failed");

    s_cam.inited = true;
    ESP_LOGI(TAG, "capture started: %s %ux%u, %u frame buffer(s) of %u bytes",
             cfg->output == CAMERA_OUTPUT_JPEG ? "JPEG" : "YUV422",
             (unsigned)w, (unsigned)h, (unsigned)s_cam.fb_count, (unsigned)s_cam.fb_size);
    return ESP_OK;

fail:
    camera_deinit();
    return ret;
}

esp_err_t camera_grab_frame(const uint8_t **out_buf, size_t *out_len, uint32_t timeout_ms)
{
    ESP_RETURN_ON_FALSE(s_cam.inited && out_buf && out_len, ESP_ERR_INVALID_STATE, TAG, "not inited");

    if (s_cam.drop_count != s_cam.last_seen_drop) {
        ESP_LOGW(TAG, "%u frame(s) dropped so far (no valid data or consumer too slow)",
                 (unsigned)s_cam.drop_count);
        s_cam.last_seen_drop = s_cam.drop_count;
    }

    cam_frame_msg_t msg;
    BaseType_t ok = xQueueReceive(s_cam.frame_q, &msg, pdMS_TO_TICKS(timeout_ms));
    if (ok != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    *out_buf = msg.buf;
    *out_len = msg.len;
    return ESP_OK;
}

ov2640_handle_t camera_get_sensor(void)
{
    return s_cam.sensor;
}

esp_err_t camera_deinit(void)
{
    if (s_cam.ctlr) {
        esp_cam_ctlr_stop(s_cam.ctlr);
        esp_cam_ctlr_disable(s_cam.ctlr);
        esp_cam_ctlr_del(s_cam.ctlr);
        s_cam.ctlr = NULL;
    }
    if (s_cam.sensor) {
        ov2640_del(s_cam.sensor);
        s_cam.sensor = NULL;
    }
    if (s_cam.fb) {
        for (size_t i = 0; i < s_cam.fb_count; i++) {
            free(s_cam.fb[i]);
        }
        free(s_cam.fb);
        s_cam.fb = NULL;
    }
    if (s_cam.frame_q) {
        vQueueDelete(s_cam.frame_q);
        s_cam.frame_q = NULL;
    }
    s_cam.inited = false;
    return ESP_OK;
}
