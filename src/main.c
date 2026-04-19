#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/display.h>
#include <zephyr/drivers/video.h>
#include <zephyr/drivers/video-controls.h>
#include <zephyr/logging/log.h>

#include <lvgl.h>
#include <stdio.h>

#include "vl53l5cx_api.h"
#include "vl53l5cx_plugin_motion_indicator.h"
#include "platform.h"

LOG_MODULE_REGISTER(smart_presence, LOG_LEVEL_INF);


#if !DT_HAS_CHOSEN(zephyr_camera)
#error "No camera chosen. Use: --shield st_b_cams_imx_mb1854"
#endif

#if !DT_HAS_CHOSEN(zephyr_display)
#error "No display chosen in devicetree."
#endif

#define VL53L5CX_NODE     DT_NODELABEL(vl53l5cx)
#define VL53L5CX_I2C_ADDR DT_REG_ADDR(VL53L5CX_NODE)
#define I2C_DEV           DEVICE_DT_GET(DT_BUS(VL53L5CX_NODE))

#define SCREEN_W       800
#define SCREEN_H       480
#define NUM_BUFFERS    2

#define MAX_PRESENCE_DISTANCE_MM  1500

#define MIN_ZONES_PRESENT         2

#define RANGING_FREQ_HZ           10

#define STANDING_FRAMES           15

#define LEAVE_TIMEOUT_MS          5000

#define TARGET_STATUS_VALID_5     5
#define TARGET_STATUS_VALID_9     9

#define MOTION_THRESHOLD          30

#define PRESENCE_DECAY            2

enum app_state {
	STATE_SLEEP, 
	STATE_ACTIVE,
};

static atomic_t g_presence_count   = ATOMIC_INIT(0);
static atomic_t g_last_presence_ms = ATOMIC_INIT(0);

static lv_obj_t *camera_canvas;
static lv_obj_t *overlay_panel;
static lv_obj_t *status_label;
static lv_obj_t *presence_label;

static lv_style_t style_overlay;
static lv_style_t style_text;

static void init_styles(void)
{
	lv_style_init(&style_overlay);
	lv_style_set_bg_color(&style_overlay, lv_color_make(0x00, 0x00, 0x00));
	lv_style_set_bg_opa(&style_overlay, LV_OPA_60);
	lv_style_set_radius(&style_overlay, 0);
	lv_style_set_pad_all(&style_overlay, 8);

	lv_style_init(&style_text);
	lv_style_set_text_font(&style_text, &lv_font_montserrat_16);
	lv_style_set_text_color(&style_text, lv_color_white());
}

static void build_ui(void)
{
	lv_obj_t *scr = lv_screen_active();

	camera_canvas = lv_canvas_create(scr);
	lv_obj_set_size(camera_canvas, SCREEN_W, SCREEN_H);
	lv_obj_set_pos(camera_canvas, 0, 0);

	overlay_panel = lv_obj_create(scr);
	lv_obj_remove_style_all(overlay_panel);
	lv_obj_add_style(overlay_panel, &style_overlay, 0);
	lv_obj_set_size(overlay_panel, SCREEN_W, 48);
	lv_obj_align(overlay_panel, LV_ALIGN_BOTTOM_MID, 0, 0);
	lv_obj_set_flex_flow(overlay_panel, LV_FLEX_FLOW_ROW);
	lv_obj_set_flex_align(overlay_panel, LV_FLEX_ALIGN_SPACE_BETWEEN,
			      LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

	status_label = lv_label_create(overlay_panel);
	lv_obj_add_style(status_label, &style_text, 0);
	lv_obj_set_style_text_font(status_label, &lv_font_montserrat_20, 0);
	lv_label_set_text(status_label, "SLEEP");
	lv_obj_set_style_text_color(status_label,
				    lv_color_make(0xFF, 0x99, 0x00), 0);

	presence_label = lv_label_create(overlay_panel);
	lv_obj_add_style(presence_label, &style_text, 0);
	lv_label_set_text(presence_label, "Presence: 0/15");
}

#define TOF_STACK_SIZE  8192
#define TOF_PRIORITY    5

K_THREAD_STACK_DEFINE(tof_stack, TOF_STACK_SIZE);
static struct k_thread tof_thread_data;

static VL53L5CX_Configuration tof_dev;
static VL53L5CX_Motion_Configuration motion_config;
static volatile bool tof_running;

static void tof_thread_entry(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	VL53L5CX_ResultsData results;
	uint8_t ready;
	int status;

	while (1) {
		if (!tof_running) {
			k_msleep(100);
			continue;
		}

		status = vl53l5cx_check_data_ready(&tof_dev, &ready);
		if (status != 0 || !ready) {
			k_msleep(5);
			continue;
		}

		status = vl53l5cx_get_ranging_data(&tof_dev, &results);
		if (status != 0) {
			k_msleep(5);
			continue;
		}

		int zones_present = 0;

		for (int z = 0; z < 16; z++) {
			int idx = z * VL53L5CX_NB_TARGET_PER_ZONE;
			uint8_t st = results.target_status[idx];
			uint32_t mot = results.motion_indicator
				       .motion[motion_config.map_id[z]];

			if (results.nb_target_detected[z] > 0 &&
			    results.distance_mm[idx] > 0 &&
			    results.distance_mm[idx] < MAX_PRESENCE_DISTANCE_MM &&
			    (st == TARGET_STATUS_VALID_5 ||
			     st == TARGET_STATUS_VALID_9) &&
			    mot >= MOTION_THRESHOLD) {
				zones_present++;
			}
		}

		if (zones_present >= MIN_ZONES_PRESENT) {
			atomic_val_t cur = atomic_get(&g_presence_count);

			if (cur < STANDING_FRAMES * 2) {
				atomic_inc(&g_presence_count);
			}
			atomic_set(&g_last_presence_ms,
				   (atomic_val_t)k_uptime_get());
		} else {
			atomic_val_t cur = atomic_get(&g_presence_count);

			if (cur > PRESENCE_DECAY) {
				atomic_sub(&g_presence_count, PRESENCE_DECAY);
			} else if (cur > 0) {
				atomic_set(&g_presence_count, 0);
			}
		}

		k_yield();
	}
}

int main(void)
{
	const struct device *display_dev;
	const struct device *video_dev;
	struct video_buffer *buffers[NUM_BUFFERS];
	struct video_buffer *vbuf = &(struct video_buffer){};
	struct video_format fmt;
	struct video_caps caps;
	struct video_selection sel;
	enum video_buf_type type = VIDEO_BUF_TYPE_OUTPUT;
	int err;

	LOG_INF("Smart Presence Wakeup");

	const struct device *i2c_dev = I2C_DEV;

	if (!device_is_ready(i2c_dev)) {
		LOG_ERR("I2C device not ready");
		return -ENODEV;
	}

	tof_dev.platform.address = VL53L5CX_I2C_ADDR << 1;
	tof_dev.platform.i2c_dev = i2c_dev;

	uint8_t alive = 0;

	err = vl53l5cx_is_alive(&tof_dev, &alive);
	if (err != 0 || !alive) {
		LOG_ERR("VL53L5CX not detected (status=%d alive=%u)", err, alive);
		return -ENODEV;
	}
	LOG_INF("VL53L5CX detected on I2C bus");

	LOG_INF("Loading VL53L5CX firmware (~95 KB, please wait)...");
	err = vl53l5cx_init(&tof_dev);
	if (err != 0) {
		LOG_ERR("VL53L5CX init failed: %d", err);
		return -EIO;
	}
	LOG_INF("VL53L5CX firmware loaded successfully");

	vl53l5cx_set_resolution(&tof_dev, VL53L5CX_RESOLUTION_4X4);
	vl53l5cx_set_ranging_frequency_hz(&tof_dev, RANGING_FREQ_HZ);

	err = vl53l5cx_motion_indicator_init(&tof_dev, &motion_config,
						 VL53L5CX_RESOLUTION_4X4);
	if (err != 0) {
		LOG_ERR("Motion indicator init failed: %d", err);
		return -EIO;
	}
	err = vl53l5cx_motion_indicator_set_distance_motion(
		&tof_dev, &motion_config, 400, MAX_PRESENCE_DISTANCE_MM);
	if (err != 0) {
		LOG_ERR("Motion indicator set distance failed: %d", err);
		return -EIO;
	}
	LOG_INF("Motion indicator configured (400-%d mm)",
		MAX_PRESENCE_DISTANCE_MM);

	vl53l5cx_start_ranging(&tof_dev);
	tof_running = true;
	LOG_INF("VL53L5CX ranging started — 4x4 @ %d Hz", RANGING_FREQ_HZ);

	k_thread_create(&tof_thread_data, tof_stack,
			K_THREAD_STACK_SIZEOF(tof_stack),
			tof_thread_entry, NULL, NULL, NULL,
			TOF_PRIORITY, 0, K_NO_WAIT);
	k_thread_name_set(&tof_thread_data, "tof_ranging");

	display_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_display));
	if (!device_is_ready(display_dev)) {
		LOG_ERR("Display device not ready");
		return -ENODEV;
	}

	video_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_camera));
	if (!device_is_ready(video_dev)) {
		LOG_ERR("Camera device not ready: %s", video_dev->name);
		return -ENODEV;
	}

	LOG_INF("Camera device: %s", video_dev->name);

	caps.type = type;
	err = video_get_caps(video_dev, &caps);
	if (err) {
		LOG_ERR("Unable to retrieve video capabilities");
		return -EIO;
	}

	for (int i = 0; caps.format_caps[i].pixelformat; i++) {
		const struct video_format_cap *fcap = &caps.format_caps[i];

		LOG_INF("  Cap: %c%c%c%c  w[%u-%u] h[%u-%u]",
			(char)fcap->pixelformat,
			(char)(fcap->pixelformat >> 8),
			(char)(fcap->pixelformat >> 16),
			(char)(fcap->pixelformat >> 24),
			fcap->width_min, fcap->width_max,
			fcap->height_min, fcap->height_max);
	}

	fmt.type = type;
	err = video_get_format(video_dev, &fmt);
	if (err) {
		LOG_ERR("Unable to retrieve video format");
		return -EIO;
	}

	fmt.width  = CONFIG_VIDEO_WIDTH;
	fmt.height = CONFIG_VIDEO_HEIGHT;
	fmt.pixelformat = VIDEO_PIX_FMT_RGB565;

	sel.type = type;
	sel.target = VIDEO_SEL_TGT_CROP;
	err = video_get_selection(video_dev, &sel);
	if (err == 0 &&
	    (sel.rect.width != fmt.width || sel.rect.height != fmt.height)) {
		sel.target = VIDEO_SEL_TGT_COMPOSE;
		sel.rect.left   = 0;
		sel.rect.top    = 0;
		sel.rect.width  = fmt.width;
		sel.rect.height = fmt.height;
		err = video_set_selection(video_dev, &sel);
		if (err && err != -ENOSYS) {
			LOG_WRN("Unable to set compose selection (%d)", err);
		}
	}

	err = video_set_format(video_dev, &fmt);
	if (err) {
		LOG_ERR("Unable to set video format");
		return -EIO;
	}

	LOG_INF("Video format: %c%c%c%c %ux%u pitch=%u",
		(char)fmt.pixelformat,
		(char)(fmt.pixelformat >> 8),
		(char)(fmt.pixelformat >> 16),
		(char)(fmt.pixelformat >> 24),
		fmt.width, fmt.height, fmt.pitch);

	for (int i = 0; i < NUM_BUFFERS; i++) {
		buffers[i] = video_buffer_aligned_alloc(fmt.size,
							CONFIG_VIDEO_BUFFER_POOL_ALIGN,
							K_NO_WAIT);
		if (buffers[i] == NULL) {
			LOG_ERR("Unable to allocate video buffer %d", i);
			return -ENOMEM;
		}
		buffers[i]->type = type;
		video_enqueue(video_dev, buffers[i]);
	}

	err = video_stream_start(video_dev, type);
	if (err) {
		LOG_ERR("Unable to start video capture (%d)", err);
		return -EIO;
	}

	LOG_INF("Video capture started (ISP processing active)");

	init_styles();
	build_ui();

	display_blanking_on(display_dev);
	LOG_INF("Display blanked — entering sleep mode");

	enum app_state state = STATE_SLEEP;

	vbuf->type = type;

	while (1) {
		int32_t count = atomic_get(&g_presence_count);
		int64_t now   = k_uptime_get();
		int64_t last  = (int64_t)atomic_get(&g_last_presence_ms);

		err = video_dequeue(video_dev, &vbuf, K_MSEC(200));

		if (err == 0 && vbuf != NULL) {
			if (state == STATE_ACTIVE) {
				lv_canvas_set_buffer(camera_canvas,
						     vbuf->buffer,
						     fmt.width, fmt.height,
						     LV_COLOR_FORMAT_RGB565);
			}

			lv_timer_handler();

			video_enqueue(video_dev, vbuf);
		} else {
			lv_timer_handler();
			k_sleep(K_MSEC(10));
		}

		char buf[48];

		snprintf(buf, sizeof(buf), "Presence: %d/%d", count,
			 STANDING_FRAMES);
		lv_label_set_text(presence_label, buf);

		switch (state) {

		case STATE_SLEEP:
			if (count >= STANDING_FRAMES) {
				LOG_INF(">> Person STANDING detected "
					"(count=%d) — waking display", count);
				display_blanking_off(display_dev);
				lv_label_set_text(status_label, "LIVE");
				lv_obj_set_style_text_color(
					status_label,
					lv_color_make(0x00, 0xE6, 0x76), 0);
				state = STATE_ACTIVE;
			} else if (count > 0 && (count % 5) == 0) {
				LOG_INF("   detecting... %d/%d frames",
					count, STANDING_FRAMES);
			}
			break;

		case STATE_ACTIVE:
			if (count == 0 && (now - last) > LEAVE_TIMEOUT_MS) {
				LOG_INF("<< Person LEFT (absent %lld ms) "
					"— blanking display",
					now - last);
				display_blanking_on(display_dev);
				lv_label_set_text(status_label, "SLEEP");
				lv_obj_set_style_text_color(
					status_label,
					lv_color_make(0xFF, 0x99, 0x00), 0);
				state = STATE_SLEEP;
			}
			break;
		}
	}
}
