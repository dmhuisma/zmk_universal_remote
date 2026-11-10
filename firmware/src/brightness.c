#include <stdio.h>
#include <stdlib.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/init.h>
#include <zephyr/logging/log.h>
#include <zephyr/settings/settings.h>

#include <zmk/backlight.h>

#include <dt_lvgl.h>

LOG_MODULE_REGISTER(brightness, CONFIG_ZMK_LOG_LEVEL);

// The More screen's brightness keys step the screen backlight 5% at a time,
// from 5% to 100%. ZMK's backlight saves the new level to flash and restores
// it at boot, CONFIG_ZMK_SETTINGS_SAVE_DEBOUNCE after the last change.
//
// The percentage shown is how bright the screen looks, not the PWM duty: the
// eye's response is roughly a power law, so equal duty steps look huge at the
// dark end and barely change anything at the bright end. Each step's duty is
// 100 * (step / 20)^2.5, except at the bottom, where that falls under 1% and
// the backlight only takes whole percents, so it rises 1% a step instead.
static const uint8_t step_duty[] = {1,  2,  3,  4,  5,  6,  7,  10, 14, 18,
                                    22, 28, 34, 41, 49, 57, 67, 77, 88, 100};

#define BRIGHTNESS_STEP_PERCENT (100 / ARRAY_SIZE(step_duty))

static const struct device *const down_key = DEVICE_DT_GET(DT_NODELABEL(brightness_down));
static const struct device *const up_key = DEVICE_DT_GET(DT_NODELABEL(brightness_up));
static const struct device *const value_label = DEVICE_DT_GET(DT_NODELABEL(brightness_value));

// The step whose duty is nearest the backlight's. The backlight need not be
// on a step: CONFIG_ZMK_BACKLIGHT_BRT_START, for one, can be any duty.
static size_t current_step(void)
{
    int duty = zmk_backlight_get_brt();
    size_t nearest = 0;

    for (size_t i = 1; i < ARRAY_SIZE(step_duty); i++) {
        if (abs(step_duty[i] - duty) < abs(step_duty[nearest] - duty)) {
            nearest = i;
        }
    }

    return nearest;
}

static void format_step(char *text, size_t len, size_t step)
{
    snprintf(text, len, "%u%%", (unsigned int)((step + 1) * BRIGHTNESS_STEP_PERCENT));
}

// Runs on the dt-lvgl thread.
static void brightness_ui_event(const struct dt_lvgl_event *event, void *user_data)
{
    ARG_UNUSED(user_data);

    if (event->event != DT_LVGL_EVENT_CLICKED) {
        return;
    }

    size_t step = current_step();

    if (event->widget == down_key) {
        step = step > 0 ? step - 1 : 0;
    } else if (event->widget == up_key) {
        step = MIN(step + 1, ARRAY_SIZE(step_duty) - 1);
    } else {
        return;
    }

    int err = zmk_backlight_set_brt(step_duty[step]);
    if (err < 0) {
        LOG_ERR("Could not set the backlight: %d", err);
        return;
    }

    char text[8];
    format_step(text, sizeof(text), step);
    err = dt_lvgl_widget_set_text(value_label, text);
    if (err < 0) {
        LOG_WRN("Could not update the brightness label: %d", err);
    }
}

// ZMK loads the saved backlight level in main(), after SYS_INIT, and settings
// commit handlers run once everything is loaded.
static int brightness_settings_commit(void)
{
    char text[8];

    format_step(text, sizeof(text), current_step());
    int err = dt_lvgl_widget_set_text_async(value_label, text);
    if (err < 0) {
        LOG_WRN("Could not update the brightness label: %d", err);
    }

    return 0;
}

SETTINGS_STATIC_HANDLER_DEFINE(brightness, "brightness", NULL, NULL, brightness_settings_commit,
                               NULL);

static struct dt_lvgl_event_callback brightness_event_callback;

static int brightness_init(void)
{
    dt_lvgl_event_callback_init(&brightness_event_callback, brightness_ui_event, NULL);
    return dt_lvgl_event_callback_register(&brightness_event_callback);
}

SYS_INIT(brightness_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
