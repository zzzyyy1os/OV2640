/*
 * OV2640 + ST7735S test - FreeRTOS multi-thread demo: camera on the LCD
 *
 * Threads (producer/consumer pipeline over two queues):
 *   camera_task  - grabs raw YUV422 frames (160x120) from the OV2640 over
 *                  DVP, converts + scales them to 128x96 RGB565 and hands
 *                  the buffer to the display thread.
 *   display_task - blits the RGB565 frames to the ST7735S TFT and shows the
 *                  achieved frame rate. Owns the LCD exclusively.
 *   stats_task   - prints real-time CPU usage of all tasks (kept from the
 *                  original FreeRTOS real-time stats example).
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "camera.h"
#include "ov2640.h"
#include "st7735s.h"

/* ==================================================================== */
/* Pin assignment - edit the numbers here to match your wiring           */
/* (camera_pins.h / st7735s_pins.h stay as component defaults)          */
/* ==================================================================== */

/* ---- ST7735S TFT on SPI2 (4-wire SPI) ---- */
#define PIN_LCD_SCLK        41      /* SCK      */
#define PIN_LCD_MOSI        42      /* SDA      */
#define PIN_LCD_CS          40      /* CS       */
#define PIN_LCD_DC          39      /* A0/D-CX  */
#define PIN_LCD_RST         38      /* RES      */
#define PIN_LCD_BACKLIGHT   GPIO_NUM_NC   /* module has a fixed backlight */

/* ---- OV2640 camera (DVP 8-bit + SCCB) ---- */
#define PIN_CAM_D0          4
#define PIN_CAM_D1          5
#define PIN_CAM_D2          6
#define PIN_CAM_D3          7
#define PIN_CAM_D4          8
#define PIN_CAM_D5          9
#define PIN_CAM_D6          10
#define PIN_CAM_D7          11
#define PIN_CAM_PCLK        12
#define PIN_CAM_VSYNC       13
#define PIN_CAM_HREF        14
#define PIN_CAM_XCLK        GPIO_NUM_NC   /* module has its own 12 MHz crystal;
                                             use 15 only for crystal-less modules */
#define PIN_CAM_SIOD        16
#define PIN_CAM_SIOC        17
#define PIN_CAM_PWDN        18
#define PIN_CAM_RESET       21

/* ==================================================================== */
/* Pipeline geometry                                                     */
/* ==================================================================== */

#define CAM_RES             OV2640_RES_160X120  /* sensor output resolution */
#define CAM_W               160
#define CAM_H               120
#define VIEW_X              0       /* where the image lands on the LCD  */
#define VIEW_Y              32      /* (4:3 scaled to 128x96, centered)  */
#define VIEW_W              128
#define VIEW_H              96
#define CAM_GRAB_TIMEOUT_MS 2000

/* If the picture comes out with red/blue swapped, flip this (the sensor's
 * YUV422 byte order is Y0 U Y1 V; the alternative is U Y0 V Y1). */
#define CAM_YUV_YUYV        1

static const char *TAG = "app";

/* Frame hand-off: camera_task converts into a free view buffer, then sends
 * it to display_task, which blits it and returns it to the free pool. */
static QueueHandle_t s_free_q;
static QueueHandle_t s_frame_q;
static uint8_t *s_view_buf[2];

/* ------------------------------------------------------------------ */
/* camera_task: grab + YUV422 -> RGB565 convert + scale                */
/* ------------------------------------------------------------------ */

#define CAM_TASK_PRIO       4       /* capture is the real-time part */
#define CAM_TASK_STACK      4096

/* Nearest-neighbour scale CAM_W x CAM_H YUV422 -> VIEW_W x VIEW_H RGB565,
 * written big-endian for the panel */
static void yuv422_to_view(const uint8_t *yuv, uint8_t *out_be)
{
    for (int y = 0; y < VIEW_H; y++) {
        const uint8_t *row = yuv + (size_t)(y * CAM_H / VIEW_H) * CAM_W * 2;
        for (int x = 0; x < VIEW_W; x++) {
            int sx = x * CAM_W / VIEW_W;
            const uint8_t *pair = row + (sx & ~1) * 2;
#if CAM_YUV_YUYV
            int luma = (sx & 1) ? pair[2] : pair[0];
            int u = pair[1], v = pair[3];
#else
            int luma = (sx & 1) ? pair[3] : pair[1];
            int u = pair[0], v = pair[2];
#endif
            /* BT.601 YCbCr -> RGB (8.8 fixed point) */
            int c = luma - 16, d = u - 128, e = v - 128;
            int r = (298 * c + 409 * e + 128) >> 8;
            int g = (298 * c - 100 * d - 208 * e + 128) >> 8;
            int b = (298 * c + 516 * d + 128) >> 8;
            r = r < 0 ? 0 : (r > 255 ? 255 : r);
            g = g < 0 ? 0 : (g > 255 ? 255 : g);
            b = b < 0 ? 0 : (b > 255 ? 255 : b);

            uint16_t rgb = ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
            *out_be++ = rgb >> 8;
            *out_be++ = rgb & 0xFF;
        }
    }
}

static void camera_task(void *arg)
{
    camera_config_t cfg;
    ESP_ERROR_CHECK(camera_default_config(&cfg));
    cfg.output = CAMERA_OUTPUT_YUV422;
    cfg.resolution = CAM_RES;
    /* pin wiring - from the pin block at the top of this file */
    cfg.sda_io_num = PIN_CAM_SIOD;
    cfg.scl_io_num = PIN_CAM_SIOC;
    cfg.pwdn_io = PIN_CAM_PWDN;
    cfg.reset_io = PIN_CAM_RESET;
    cfg.xclk_io = PIN_CAM_XCLK;
    cfg.pclk_io = PIN_CAM_PCLK;
    cfg.vsync_io = PIN_CAM_VSYNC;
    cfg.href_io = PIN_CAM_HREF;
    cfg.data_io[0] = PIN_CAM_D0;
    cfg.data_io[1] = PIN_CAM_D1;
    cfg.data_io[2] = PIN_CAM_D2;
    cfg.data_io[3] = PIN_CAM_D3;
    cfg.data_io[4] = PIN_CAM_D4;
    cfg.data_io[5] = PIN_CAM_D5;
    cfg.data_io[6] = PIN_CAM_D6;
    cfg.data_io[7] = PIN_CAM_D7;

    /* Retry init forever: the error reprints every 5 s so it can be read
     * from the log without catching the boot moment */
    esp_err_t err;
    while ((err = camera_init(&cfg)) != ESP_OK) {
        ESP_LOGE(TAG, "camera init failed: %s - check the PIN_CAM_* wiring, retrying in 5 s",
                 esp_err_to_name(err));
        vTaskDelay(pdMS_TO_TICKS(5000));
    }

    uint32_t grabbed = 0;
    while (1) {
        const uint8_t *frame = NULL;
        size_t len = 0;
        err = camera_grab_frame(&frame, &len, CAM_GRAB_TIMEOUT_MS);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "grab failed: %s", esp_err_to_name(err));
            continue;
        }
        if (len != (size_t)CAM_W * CAM_H * 2) {
            ESP_LOGW(TAG, "unexpected frame size %u (want %u)", (unsigned)len, (unsigned)(CAM_W * CAM_H * 2));
            continue;
        }
        if (++grabbed % 30 == 0) {
            ESP_LOGI(TAG, "captured %" PRIu32 " frames", grabbed);
        }

        uint8_t *view;
        if (xQueueReceive(s_free_q, &view, portMAX_DELAY) != pdTRUE) {
            continue;
        }
        yuv422_to_view(frame, view);
        if (xQueueSend(s_frame_q, &view, 0) != pdTRUE) {
            xQueueSend(s_free_q, &view, 0);     /* display too slow - recycle */
        }
    }
}

/* ------------------------------------------------------------------ */
/* display_task: blit frames to the ST7735S and show the frame rate    */
/* ------------------------------------------------------------------ */

#define DISPLAY_TASK_PRIO   3
#define DISPLAY_TASK_STACK  5120

static void display_task(void *arg)
{
    st7735s_config_t cfg;
    ESP_ERROR_CHECK(st7735s_default_config(&cfg));
    /* pin wiring - from the pin block at the top of this file */
    cfg.sclk_io = PIN_LCD_SCLK;
    cfg.mosi_io = PIN_LCD_MOSI;
    cfg.cs_io = PIN_LCD_CS;
    cfg.dc_io = PIN_LCD_DC;
    cfg.rst_io = PIN_LCD_RST;
    cfg.backlight_io = PIN_LCD_BACKLIGHT;

    st7735s_handle_t lcd = NULL;
    esp_err_t err = st7735s_new(&cfg, &lcd);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "display init failed: %s - check the PIN_LCD_* wiring", esp_err_to_name(err));
        vTaskDelete(NULL);
        return;
    }

    st7735s_clear(lcd, ST7735S_BLACK);
    st7735s_show_string(lcd, 8, 8, 112, 16, 16, "CAM WAIT...", ST7735S_GRAY, ST7735S_BLACK);

    uint32_t frames = 0;
    int64_t t_last = esp_timer_get_time();
    while (1) {
        uint8_t *view;
        if (xQueueReceive(s_frame_q, &view, pdMS_TO_TICKS(CAM_GRAB_TIMEOUT_MS)) != pdTRUE) {
            ESP_LOGW(TAG, "no camera frame yet - is the camera connected?");
            continue;
        }

        st7735s_draw_bitmap(lcd, VIEW_X, VIEW_Y, VIEW_W, VIEW_H, view);
        xQueueSend(s_free_q, &view, portMAX_DELAY);

        frames++;
        if (frames == 1) {
            ESP_LOGI(TAG, "first frame shown on LCD");
        }
        int64_t t_now = esp_timer_get_time();
        float fps = (t_now > t_last) ? 1000000.0f / (t_now - t_last) : 0.0f;
        t_last = t_now;

        char line[32];
        snprintf(line, sizeof(line), "CAM %5.1f fps", fps);
        st7735s_show_string(lcd, 8, 8, 112, 16, 16, line, ST7735S_GREEN, ST7735S_BLACK);

        if (frames % 30 == 0) {
            ESP_LOGI(TAG, "%" PRIu32 " frames shown, %.2f fps", frames, fps);
        }
    }
}

/* ------------------------------------------------------------------ */
/* stats_task: real-time CPU usage (from the FreeRTOS stats example)   */
/* ------------------------------------------------------------------ */

#define STATS_TASK_PRIO     2
#define STATS_TASK_STACK    4096
#define STATS_TICKS         pdMS_TO_TICKS(5000)
#define ARRAY_SIZE_OFFSET   5   //Increase this if print_real_time_stats returns ESP_ERR_INVALID_SIZE

/**
 * @brief   Function to print the CPU usage of tasks over a given duration.
 *
 * This function will measure and print the CPU usage of tasks over a specified
 * number of ticks (i.e. real time stats). This is implemented by simply calling
 * uxTaskGetSystemState() twice separated by a delay, then calculating the
 * differences of task run times before and after the delay.
 *
 * @note    If any tasks are added or removed during the delay, the stats of
 *          those tasks will not be printed.
 * @note    This function should be called from a high priority task to minimize
 *          inaccuracies with delays.
 * @note    When running in dual core mode, each core will correspond to 50% of
 *          the run time.
 *
 * @param   xTicksToWait    Period of stats measurement
 *
 * @return
 *  - ESP_OK                Success
 *  - ESP_ERR_NO_MEM        Insufficient memory to allocated internal arrays
 *  - ESP_ERR_INVALID_SIZE  Insufficient array size for uxTaskGetSystemState. Trying increasing ARRAY_SIZE_OFFSET
 *  - ESP_ERR_INVALID_STATE Delay duration too short
 */
static esp_err_t print_real_time_stats(TickType_t xTicksToWait)
{
    TaskStatus_t *start_array = NULL, *end_array = NULL;
    UBaseType_t start_array_size, end_array_size;
    configRUN_TIME_COUNTER_TYPE start_run_time, end_run_time;
    esp_err_t ret;

    //Allocate array to store current task states
    start_array_size = uxTaskGetNumberOfTasks() + ARRAY_SIZE_OFFSET;
    start_array = malloc(sizeof(TaskStatus_t) * start_array_size);
    if (start_array == NULL) {
        ret = ESP_ERR_NO_MEM;
        goto exit;
    }
    //Get current task states
    start_array_size = uxTaskGetSystemState(start_array, start_array_size, &start_run_time);
    if (start_array_size == 0) {
        ret = ESP_ERR_INVALID_SIZE;
        goto exit;
    }

    vTaskDelay(xTicksToWait);

    //Allocate array to store tasks states post delay
    end_array_size = uxTaskGetNumberOfTasks() + ARRAY_SIZE_OFFSET;
    end_array = malloc(sizeof(TaskStatus_t) * end_array_size);
    if (end_array == NULL) {
        ret = ESP_ERR_NO_MEM;
        goto exit;
    }
    //Get post delay task states
    end_array_size = uxTaskGetSystemState(end_array, end_array_size, &end_run_time);
    if (end_array_size == 0) {
        ret = ESP_ERR_INVALID_SIZE;
        goto exit;
    }

    //Calculate total_elapsed_time in units of run time stats clock period.
    uint32_t total_elapsed_time = (end_run_time - start_run_time);
    if (total_elapsed_time == 0) {
        ret = ESP_ERR_INVALID_STATE;
        goto exit;
    }

    printf("| Task | Run Time | Percentage\n");
    //Match each task in start_array to those in the end_array
    for (int i = 0; i < start_array_size; i++) {
        int k = -1;
        for (int j = 0; j < end_array_size; j++) {
            if (start_array[i].xHandle == end_array[j].xHandle) {
                k = j;
                //Mark that task have been matched by overwriting their handles
                start_array[i].xHandle = NULL;
                end_array[j].xHandle = NULL;
                break;
            }
        }
        //Check if matching task found
        if (k >= 0) {
            uint32_t task_elapsed_time = end_array[k].ulRunTimeCounter - start_array[i].ulRunTimeCounter;
            uint32_t percentage_time = (task_elapsed_time * 100UL) / (total_elapsed_time * CONFIG_FREERTOS_NUMBER_OF_CORES);
            printf("| %s | %"PRIu32" | %"PRIu32"%%\n", start_array[i].pcTaskName, task_elapsed_time, percentage_time);
        }
    }

    //Print unmatched tasks
    for (int i = 0; i < start_array_size; i++) {
        if (start_array[i].xHandle != NULL) {
            printf("| %s | Deleted\n", start_array[i].pcTaskName);
        }
    }
    for (int i = 0; i < end_array_size; i++) {
        if (end_array[i].xHandle != NULL) {
            printf("| %s | Created\n", end_array[i].pcTaskName);
        }
    }
    ret = ESP_OK;

exit:    //Common return path
    free(start_array);
    free(end_array);
    return ret;
}

static void stats_task(void *arg)
{
    while (1) {
        printf("\nReal time stats over %"PRIu32" ticks:\n", (uint32_t)STATS_TICKS);
        if (print_real_time_stats(STATS_TICKS) == ESP_OK) {
            printf("Real time stats obtained\n");
        } else {
            printf("Error getting real time stats\n");
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

/* ------------------------------------------------------------------ */
/* app_main                                                            */
/* ------------------------------------------------------------------ */

void app_main(void)
{
    ESP_LOGI(TAG, "OV2640 -> ST7735S demo: starting camera + display + stats threads");

    /* View buffers for the camera -> display pipeline (DMA-capable so the
     * SPI driver can stream them straight to the panel) */
    for (int i = 0; i < 2; i++) {
        s_view_buf[i] = heap_caps_malloc(VIEW_W * VIEW_H * 2, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
        if (s_view_buf[i] == NULL) {
            ESP_LOGE(TAG, "no memory for view buffers");
            return;
        }
    }
    s_free_q = xQueueCreate(2, sizeof(uint8_t *));
    s_frame_q = xQueueCreate(2, sizeof(uint8_t *));
    for (int i = 0; i < 2; i++) {
        xQueueSend(s_free_q, &s_view_buf[i], 0);
    }

    xTaskCreatePinnedToCore(camera_task, "cam_task", CAM_TASK_STACK, NULL, CAM_TASK_PRIO, NULL, tskNO_AFFINITY);
    xTaskCreatePinnedToCore(display_task, "disp_task", DISPLAY_TASK_STACK, NULL, DISPLAY_TASK_PRIO, NULL, tskNO_AFFINITY);
    xTaskCreatePinnedToCore(stats_task, "stats", STATS_TASK_STACK, NULL, STATS_TASK_PRIO, NULL, tskNO_AFFINITY);
}
