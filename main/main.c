/*
 * USB webcam demo (Logitech C270) for the Guition JC8012P4A1C_I_W_Y (ESP32-P4).
 *
 * Pipeline: UVC MJPEG (USB-HS, "High speed USB" Type-C) -> hardware JPEG decoder
 * -> PPA scale + rotate into the portrait 800x1280 DPI framebuffer -> JD9365.
 * The picture is shown in landscape (1280x800), same orientation as espVT.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/jpeg_decode.h"
#include "driver/ppa.h"
#include "esp_check.h"
#include "esp_heap_caps.h"
#include "esp_lcd_jd9365.h"
#include "esp_lcd_mipi_dsi.h"
#include "esp_lcd_panel_ops.h"
#include "esp_ldo_regulator.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "usb/usb_host.h"
#include "usb/uvc_host.h"

extern const jd9365_lcd_init_cmd_t jc8012_lcd_init_cmds[];
extern const size_t jc8012_lcd_init_cmds_count;

static const char *TAG = "uvc_demo";

/* Panel (native portrait) and landscape view */
#define PANEL_W 800
#define PANEL_H 1280
#define VIEW_W 1280
#define VIEW_H 800
#define PANEL_FB_BYTES (PANEL_W * PANEL_H * 2)
#define LCD_BACKLIGHT GPIO_NUM_23
#define LCD_RESET GPIO_NUM_27

/* Largest frame we accept from the camera */
#define MAX_FRAME_W 1280
#define MAX_FRAME_H 960
#define NUMBER_OF_FRAME_BUFFERS 5

/* Preferred picture size: the largest MJPEG format the camera offers up to this size */
#define PREFERRED_MAX_W 1280
#define PREFERRED_MAX_H 720
/* Upper limit for the requested frame rate: the display shows about 20 fps, and analog
 * sources (PAL 25, AHD 25/30) have no more anyway. More only loads USB and CPU. */
#define MAX_FPS 30

/* MJPEG formats offered by the connected camera, best first (filled on connection) */
#define MAX_CANDIDATES 32
static uvc_host_stream_format_t s_cand[MAX_CANDIDATES];
static int s_cand_n;
static SemaphoreHandle_t s_cam_ready;

static esp_lcd_panel_handle_t s_panel;
static void *s_fb[2];
static int s_back = 1;
static SemaphoreHandle_t s_vsync;
static ppa_client_handle_t s_ppa;
static jpeg_decoder_handle_t s_jpeg;
static uint8_t *s_jpeg_in;
static size_t s_jpeg_in_size;
static uint8_t *s_jpeg_out;
static size_t s_jpeg_out_size;

static QueueHandle_t s_frame_q;
static SemaphoreHandle_t s_disconnected;
static uvc_host_stream_hdl_t s_stream;
static volatile uint32_t s_underflows;
static volatile uint32_t s_dropped;

/* ---------------------------------------------------------------- display */

static bool IRAM_ATTR on_frame_done(esp_lcd_panel_handle_t panel, esp_lcd_dpi_panel_event_data_t *edata, void *ctx)
{
    BaseType_t woken = pdFALSE;
    xSemaphoreGiveFromISR(s_vsync, &woken);
    return woken == pdTRUE;
}

static esp_err_t display_init(void)
{
    esp_ldo_channel_handle_t ldo = NULL;
    esp_ldo_channel_config_t ldo_cfg = {.chan_id = 3, .voltage_mv = 2500};
    ESP_RETURN_ON_ERROR(esp_ldo_acquire_channel(&ldo_cfg, &ldo), TAG, "DSI PHY LDO");

    esp_lcd_dsi_bus_handle_t bus = NULL;
    esp_lcd_dsi_bus_config_t bus_cfg = {.bus_id = 0, .num_data_lanes = 2, .lane_bit_rate_mbps = 1500};
    ESP_RETURN_ON_ERROR(esp_lcd_new_dsi_bus(&bus_cfg, &bus), TAG, "DSI bus");

    esp_lcd_panel_io_handle_t io = NULL;
    esp_lcd_dbi_io_config_t io_cfg = {.virtual_channel = 0, .lcd_cmd_bits = 8, .lcd_param_bits = 8};
    ESP_RETURN_ON_ERROR(esp_lcd_new_panel_io_dbi(bus, &io_cfg, &io), TAG, "DBI IO");

    esp_lcd_dpi_panel_config_t dpi_cfg = {
        .virtual_channel = 0,
        .dpi_clk_src = MIPI_DSI_DPI_CLK_SRC_DEFAULT,
        .dpi_clock_freq_mhz = 70,
        .in_color_format = LCD_COLOR_FMT_RGB565,
        .out_color_format = LCD_COLOR_FMT_RGB565,
        .num_fbs = 2,
        .video_timing = {
            .h_size = PANEL_W, .v_size = PANEL_H,
            .hsync_pulse_width = 20, .hsync_back_porch = 20, .hsync_front_porch = 40,
            .vsync_pulse_width = 4, .vsync_back_porch = 10, .vsync_front_porch = 30,
        },
    };
    jd9365_vendor_config_t vendor_cfg = {
        .init_cmds = jc8012_lcd_init_cmds,
        .init_cmds_size = (uint16_t)jc8012_lcd_init_cmds_count,
        .mipi_config = {.dsi_bus = bus, .dpi_config = &dpi_cfg, .lane_num = 2},
    };
    esp_lcd_panel_dev_config_t panel_cfg = {
        .reset_gpio_num = LCD_RESET,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB,
        .bits_per_pixel = 16,
        .vendor_config = &vendor_cfg,
    };
    ESP_RETURN_ON_ERROR(esp_lcd_new_panel_jd9365(io, &panel_cfg, &s_panel), TAG, "JD9365");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_reset(s_panel), TAG, "reset");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_init(s_panel), TAG, "init");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_disp_on_off(s_panel, true), TAG, "disp on");

    ESP_RETURN_ON_ERROR(esp_lcd_dpi_panel_get_frame_buffer(s_panel, 2, &s_fb[0], &s_fb[1]), TAG, "fb");
    for (int i = 0; i < 2; i++) {
        memset(s_fb[i], 0, PANEL_FB_BYTES);
        esp_lcd_panel_draw_bitmap(s_panel, 0, 0, PANEL_W, PANEL_H, s_fb[i]);
    }
    s_back = 0;

    s_vsync = xSemaphoreCreateBinary();
    esp_lcd_dpi_panel_event_callbacks_t cbs = {.on_frame_buf_complete = on_frame_done};
    ESP_RETURN_ON_ERROR(esp_lcd_dpi_panel_register_event_callbacks(s_panel, &cbs, NULL), TAG, "cb");

    gpio_config_t bl = {.pin_bit_mask = 1ULL << LCD_BACKLIGHT, .mode = GPIO_MODE_OUTPUT};
    gpio_config(&bl);
    gpio_set_level(LCD_BACKLIGHT, 1);

    ppa_client_config_t ppa_cfg = {.oper_type = PPA_OPERATION_SRM, .max_pending_trans_num = 1};
    ESP_RETURN_ON_ERROR(ppa_register_client(&ppa_cfg, &s_ppa), TAG, "PPA");
    return ESP_OK;
}

static void display_clear(void)
{
    for (int i = 0; i < 2; i++) {
        memset(s_fb[i], 0, PANEL_FB_BYTES);
        esp_lcd_panel_draw_bitmap(s_panel, 0, 0, PANEL_W, PANEL_H, s_fb[i]);
    }
    s_back = 0;
}

/* Scale the decoded landscape picture to fit 1280x800, rotate it into the
 * portrait panel buffer (centered, black borders) and flip buffers. */
static esp_err_t display_show(const void *rgb565, uint32_t w, uint32_t h)
{
    float s = (float)VIEW_W / w;
    if ((float)VIEW_H / h < s) {
        s = (float)VIEW_H / h;
    }
    s = (float)(int)(s * 16.0f) / 16.0f; /* PPA uses 1/16 steps */
    uint32_t out_w = (uint32_t)(w * s); /* landscape width -> panel height */
    uint32_t out_h = (uint32_t)(h * s); /* landscape height -> panel width */

    ppa_srm_oper_config_t op = {
        .in = {
            .buffer = rgb565, .pic_w = w, .pic_h = h,
            .block_w = w, .block_h = h, .srm_cm = PPA_SRM_COLOR_MODE_RGB565,
        },
        .out = {
            .buffer = s_fb[s_back], .buffer_size = PANEL_FB_BYTES,
            .pic_w = PANEL_W, .pic_h = PANEL_H,
            .block_offset_x = (PANEL_W - out_h) / 2,
            .block_offset_y = (PANEL_H - out_w) / 2,
            .srm_cm = PPA_SRM_COLOR_MODE_RGB565,
        },
        .rotation_angle = PPA_SRM_ROTATION_ANGLE_270,
        .scale_x = s,
        .scale_y = s,
        .mode = PPA_TRANS_MODE_BLOCKING,
    };
    ESP_RETURN_ON_ERROR(ppa_do_scale_rotate_mirror(s_ppa, &op), TAG, "PPA SRM");

    xSemaphoreTake(s_vsync, 0);
    ESP_RETURN_ON_ERROR(esp_lcd_panel_draw_bitmap(s_panel, 0, 0, PANEL_W, PANEL_H, s_fb[s_back]), TAG, "flip");
    xSemaphoreTake(s_vsync, pdMS_TO_TICKS(100));
    s_back ^= 1;
    return ESP_OK;
}

/* -------------------------------------------------------------------- UVC */

static void stream_callback(const uvc_host_stream_event_data_t *event, void *user_ctx)
{
    switch (event->type) {
    case UVC_HOST_TRANSFER_ERROR:
        ESP_LOGE(TAG, "USB transfer error %d", event->transfer_error.error);
        break;
    case UVC_HOST_DEVICE_DISCONNECTED:
        ESP_LOGW(TAG, "Camera disconnected");
        uvc_host_stream_close(event->device_disconnected.stream_hdl);
        xSemaphoreGive(s_disconnected);
        break;
    case UVC_HOST_FRAME_BUFFER_OVERFLOW:
        ESP_LOGW(TAG, "Frame buffer overflow");
        break;
    case UVC_HOST_FRAME_BUFFER_UNDERFLOW:
        s_underflows++; /* reported in the 5 s statistics */
        break;
    default:
        break;
    }
}

static bool frame_callback(const uvc_host_frame_t *frame, void *user_ctx)
{
    if (frame->vs_format.format != UVC_VS_FORMAT_MJPEG) {
        return true; /* return the buffer immediately */
    }
    if (xQueueSendToBack(s_frame_q, &frame, 0) != pdPASS) {
        return true; /* queue full: drop */
    }
    return false; /* processing task returns it */
}

/* Frame interval (100 ns units) to request: the highest frame rate the camera offers for
 * this format that does not exceed MAX_FPS; if every offered rate is higher, the lowest one.
 * Only offered rates may be requested, the driver rejects anything else. */
static uint32_t choose_interval(const uvc_host_frame_info_t *f)
{
    const uint32_t target = 10000000 / MAX_FPS;
    if (f->interval_type == 0) {
        /* continuous range: clamp to [min, max] and align to the step */
        uint32_t iv = target;
        if (iv < f->interval_min) {
            iv = f->interval_min;
        }
        if (f->interval_step > 0 && iv > f->interval_min) {
            iv = f->interval_min + (iv - f->interval_min + f->interval_step - 1) / f->interval_step * f->interval_step;
        }
        if (f->interval_max && iv > f->interval_max) {
            iv = f->interval_max;
        }
        return iv ? iv : f->default_interval;
    }
    int count = f->interval_type < CONFIG_UVC_INTERVAL_ARRAY_SIZE ? f->interval_type : CONFIG_UVC_INTERVAL_ARRAY_SIZE;
    uint32_t best_ok = 0;  /* smallest interval >= target, i.e. highest fps <= MAX_FPS */
    uint32_t slowest = 0;  /* largest interval, used if all rates are above MAX_FPS */
    for (int i = 0; i < count; i++) {
        uint32_t iv = f->interval[i];
        if (iv == 0) {
            continue;
        }
        if (iv >= target && (best_ok == 0 || iv < best_ok)) {
            best_ok = iv;
        }
        if (iv > slowest) {
            slowest = iv;
        }
    }
    if (best_ok) {
        return best_ok;
    }
    return slowest ? slowest : f->default_interval;
}

static void frame_rates_str(const uvc_host_frame_info_t *f, char *out, size_t len)
{
    if (f->interval_type == 0) {
        snprintf(out, len, "%.1f-%.1f fps", f->interval_max ? 1e7f / f->interval_max : 0.0f,
                 f->interval_min ? 1e7f / f->interval_min : 0.0f);
        return;
    }
    int count = f->interval_type < CONFIG_UVC_INTERVAL_ARRAY_SIZE ? f->interval_type : CONFIG_UVC_INTERVAL_ARRAY_SIZE;
    size_t pos = 0;
    out[0] = 0;
    for (int i = 0; i < count && pos < len; i++) {
        if (f->interval[i]) {
            pos += snprintf(out + pos, len - pos, "%s%.1f", pos ? " " : "", 1e7f / f->interval[i]);
        }
    }
    if (pos < len) {
        snprintf(out + pos, len - pos, " fps");
    }
}

/* Formats up to PREFERRED_MAX_* first (largest first), then larger ones (smallest first). */
static int candidate_order(const void *a, const void *b)
{
    const uvc_host_stream_format_t *fa = a, *fb = b;
    bool pa = fa->h_res <= PREFERRED_MAX_W && fa->v_res <= PREFERRED_MAX_H;
    bool pb = fb->h_res <= PREFERRED_MAX_W && fb->v_res <= PREFERRED_MAX_H;
    if (pa != pb) {
        return pa ? -1 : 1;
    }
    long area_a = (long)fa->h_res * fa->v_res;
    long area_b = (long)fb->h_res * fb->v_res;
    if (area_a == area_b) {
        return 0;
    }
    return pa ? (area_a > area_b ? -1 : 1) : (area_a < area_b ? -1 : 1);
}

static void driver_event_callback(const uvc_host_driver_event_data_t *event, void *user_ctx)
{
    if (event->type != UVC_HOST_DRIVER_EVENT_DEVICE_CONNECTED) {
        return;
    }
    size_t n = event->device_connected.frame_info_num;
    ESP_LOGI(TAG, "UVC camera at address %d, %u frame formats:", event->device_connected.dev_addr, (unsigned)n);
    uvc_host_frame_info_t *list = calloc(n, sizeof(*list));
    if (list && uvc_host_get_frame_list(event->device_connected.dev_addr, event->device_connected.uvc_stream_index,
                                        (uvc_host_frame_info_t (*)[])list, &n) == ESP_OK) {
        for (size_t i = 0; i < n; i++) {
            char rates[96];
            frame_rates_str(&list[i], rates, sizeof(rates));
            ESP_LOGI(TAG, "  %-5s %4ux%-4u default %.1f fps, offers %s",
                     list[i].format == UVC_VS_FORMAT_MJPEG ? "MJPEG" :
                     list[i].format == UVC_VS_FORMAT_YUY2 ? "YUY2" : "other",
                     list[i].h_res, list[i].v_res,
                     list[i].default_interval ? 1e7f / list[i].default_interval : 0.0f, rates);
        }
        /* Candidates: MJPEG up to the preferred size, largest first, at the highest offered
         * frame rate up to MAX_FPS; larger formats (up to MAX_FRAME_*) only as a last resort. */
        int c = 0;
        for (size_t i = 0; i < n && c < MAX_CANDIDATES; i++) {
            const uvc_host_frame_info_t *f = &list[i];
            if (f->format != UVC_VS_FORMAT_MJPEG || f->h_res > MAX_FRAME_W || f->v_res > MAX_FRAME_H) {
                continue;
            }
            s_cand[c++] = (uvc_host_stream_format_t) {
                .h_res = f->h_res,
                .v_res = f->v_res,
                .fps = 1e7f / choose_interval(f),
                .format = UVC_VS_FORMAT_MJPEG,
            };
        }
        qsort(s_cand, c, sizeof(s_cand[0]), candidate_order);
        s_cand_n = c;
        if (c == 0) {
            ESP_LOGW(TAG, "Camera offers no MJPEG format up to %dx%d", MAX_FRAME_W, MAX_FRAME_H);
        }
    }
    free(list);
    xSemaphoreGive(s_cam_ready);
}

static void processing_task(void *arg)
{
    jpeg_decode_cfg_t dec_cfg = {
        .output_format = JPEG_DECODE_OUT_FORMAT_RGB565,
        .rgb_order = JPEG_DEC_RGB_ELEMENT_ORDER_BGR,
        .conv_std = JPEG_YUV_RGB_CONV_STD_BT601,
    };
    int64_t t0 = esp_timer_get_time();
    int shown = 0;
    uint32_t last_w = 0, last_h = 0;

    while (1) {
        uvc_host_frame_t *frame;
        xQueueReceive(s_frame_q, &frame, portMAX_DELAY);
        /* Only show the newest frame: hand older ones straight back to the driver. */
        uvc_host_frame_t *newer;
        while (xQueueReceive(s_frame_q, &newer, 0) == pdPASS) {
            uvc_host_frame_return(s_stream, frame);
            frame = newer;
            s_dropped++;
        }

        jpeg_decode_picture_info_t info;
        bool ok = frame->data_len <= s_jpeg_in_size &&
                  jpeg_decoder_get_info(frame->data, frame->data_len, &info) == ESP_OK &&
                  info.width <= MAX_FRAME_W && info.height <= MAX_FRAME_H;
        if (ok) {
            /* HW decoder needs a 16-byte aligned DMA buffer; UVC buffers are not guaranteed to be. */
            memcpy(s_jpeg_in, frame->data, frame->data_len);
            size_t len = frame->data_len;
            uvc_host_frame_return(s_stream, frame);
            frame = NULL;

            uint32_t out_size = 0;
            esp_err_t err = jpeg_decoder_process(s_jpeg, &dec_cfg, s_jpeg_in, len, s_jpeg_out, s_jpeg_out_size, &out_size);
            if (err == ESP_OK) {
                if (info.width != last_w || info.height != last_h) {
                    display_clear();
                    last_w = info.width;
                    last_h = info.height;
                    ESP_LOGI(TAG, "Showing %" PRIu32 "x%" PRIu32 " MJPEG", last_w, last_h);
                }
                display_show(s_jpeg_out, info.width, info.height);
                shown++;
            } else {
                ESP_LOGW(TAG, "JPEG decode failed: %s", esp_err_to_name(err));
            }
        } else {
            ESP_LOGW(TAG, "Dropping frame (%u bytes)", (unsigned)frame->data_len);
        }
        if (frame) {
            uvc_host_frame_return(s_stream, frame);
        }

        int64_t now = esp_timer_get_time();
        if (now - t0 >= 5000000) {
            ESP_LOGI(TAG, "%.1f fps displayed, %" PRIu32 " skipped (late), %" PRIu32 " underflows",
                     shown * 1e6f / (now - t0), s_dropped, s_underflows);
            shown = 0;
            s_dropped = 0;
            s_underflows = 0;
            t0 = now;
        }
    }
}

static void usb_lib_task(void *arg)
{
    while (1) {
        uint32_t flags;
        usb_host_lib_handle_events(portMAX_DELAY, &flags);
        if (flags & USB_HOST_LIB_EVENT_FLAGS_NO_CLIENTS) {
            usb_host_device_free_all();
        }
    }
}

void app_main(void)
{
    ESP_ERROR_CHECK(display_init());
    ESP_LOGI(TAG, "Display ready");

    jpeg_decode_engine_cfg_t eng_cfg = {.intr_priority = 0, .timeout_ms = 100};
    ESP_ERROR_CHECK(jpeg_new_decoder_engine(&eng_cfg, &s_jpeg));
    jpeg_decode_memory_alloc_cfg_t in_cfg = {.buffer_direction = JPEG_DEC_ALLOC_INPUT_BUFFER};
    jpeg_decode_memory_alloc_cfg_t out_cfg = {.buffer_direction = JPEG_DEC_ALLOC_OUTPUT_BUFFER};
    s_jpeg_in = jpeg_alloc_decoder_mem(512 * 1024, &in_cfg, &s_jpeg_in_size);
    s_jpeg_out = jpeg_alloc_decoder_mem(MAX_FRAME_W * MAX_FRAME_H * 2, &out_cfg, &s_jpeg_out_size);
    assert(s_jpeg_in && s_jpeg_out);

    s_frame_q = xQueueCreate(NUMBER_OF_FRAME_BUFFERS, sizeof(uvc_host_frame_t *));
    s_disconnected = xSemaphoreCreateBinary();
    s_cam_ready = xSemaphoreCreateBinary();
    assert(s_frame_q && s_disconnected);

    const usb_host_config_t host_cfg = {.skip_phy_setup = false, .intr_flags = ESP_INTR_FLAG_LOWMED};
    ESP_ERROR_CHECK(usb_host_install(&host_cfg));
    xTaskCreate(usb_lib_task, "usb_lib", 4096, NULL, 10, NULL);

    const uvc_host_driver_config_t uvc_cfg = {
        .driver_task_stack_size = 6 * 1024,
        .driver_task_priority = 6,
        .xCoreID = tskNO_AFFINITY,
        .create_background_task = true,
        .event_cb = driver_event_callback,
    };
    ESP_ERROR_CHECK(uvc_host_install(&uvc_cfg));
    xTaskCreatePinnedToCore(processing_task, "frame_proc", 6 * 1024, NULL, 5, NULL, 1);

    ESP_LOGI(TAG, "Waiting for a UVC camera on the High-Speed USB port...");
    while (true) {
        /* Wait until a camera is connected and its formats are known */
        xSemaphoreTake(s_cam_ready, portMAX_DELAY);
        bool streaming = false;
        for (int i = 0; i < s_cand_n && !streaming; i++) {
            const uvc_host_stream_format_t *f = &s_cand[i];
            uvc_host_stream_config_t stream_cfg = {
                .event_cb = stream_callback,
                .frame_cb = frame_callback,
                .usb = {.vid = UVC_HOST_ANY_VID, .pid = UVC_HOST_ANY_PID, .uvc_stream_index = 0},
                .vs_format = *f,
                .advanced = {
                    .number_of_frame_buffers = NUMBER_OF_FRAME_BUFFERS,
                    .frame_size = 512 * 1024,
                    .frame_heap_caps = MALLOC_CAP_SPIRAM,
                    .number_of_urbs = 4,
                    .urb_size = 0,
                },
            };
            esp_err_t err = uvc_host_stream_open(&stream_cfg, pdMS_TO_TICKS(5000), &s_stream);
            if (err != ESP_OK) {
                ESP_LOGW(TAG, "%ux%u@%.1f MJPEG rejected (%s), trying the next format", f->h_res, f->v_res, f->fps,
                         esp_err_to_name(err));
                continue;
            }
            err = uvc_host_stream_start(s_stream);
            if (err != ESP_OK) {
                ESP_LOGW(TAG, "%ux%u@%.1f MJPEG did not start (%s), trying the next format", f->h_res, f->v_res, f->fps,
                         esp_err_to_name(err));
                uvc_host_stream_close(s_stream);
                continue;
            }
            ESP_LOGI(TAG, "Stream opened: %ux%u@%.1f MJPEG", f->h_res, f->v_res, f->fps);
            streaming = true;
        }
        if (!streaming) {
            ESP_LOGE(TAG, "No usable MJPEG format - reconnect the camera");
            continue;
        }
        xSemaphoreTake(s_disconnected, portMAX_DELAY);
    }
}
