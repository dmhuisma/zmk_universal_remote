#define DT_DRV_COMPAT universal_remote_touchpad

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/init.h>
#include <zephyr/input/input.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/spinlock.h>

#include <lvgl.h>

#include <dt_lvgl.h>

LOG_MODULE_REGISTER(touchpad, CONFIG_ZMK_LOG_LEVEL);

#if DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT)

BUILD_ASSERT(DT_NUM_INST_STATUS_OKAY(DT_DRV_COMPAT) == 1, "Only one touchpad is supported");

// The Touchpad screen's pad moves the host's mouse pointer. Touches reach
// this file straight from the touch controller, on the input thread, rather
// than through LVGL, which polls the touch input only at its refresh rate and
// would make the pointer stutter. LVGL still gets every touch, but the pad is
// an empty container, so it does nothing with them.
//
// A touch is the pad's only if it starts on the pad while the screen is
// shown, so a touch that starts on Back, or on any other screen, never moves
// the pointer. Then:
//   - Moving further than tap-slop-px moves the pointer, by INPUT_REL_X and
//     INPUT_REL_Y from this device, which a zmk,input-listener scales and
//     sends to the host as mouse movement.
//   - Lifting within tap-timeout-ms, without moving, clicks the left button.
//   - Holding still for long-press-ms clicks the right button, and the rest
//     of the touch is ignored.

#define TOUCHPAD_NODE DT_DRV_INST(0)

#define TAP_TIMEOUT_MS DT_PROP(TOUCHPAD_NODE, tap_timeout_ms)
#define TAP_SLOP_PX DT_PROP(TOUCHPAD_NODE, tap_slop_px)
#define LONG_PRESS_MS DT_PROP(TOUCHPAD_NODE, long_press_ms)

static const struct device *const touchpad = DEVICE_DT_GET(TOUCHPAD_NODE);
static const struct device *const screen_dev = DEVICE_DT_GET(DT_PHANDLE(TOUCHPAD_NODE, screen));
static const struct device *const area_dev = DEVICE_DT_GET(DT_PHANDLE(TOUCHPAD_NODE, area));

// Shared by the input thread (touches), the LVGL thread (the screen being
// shown or left) and the system work queue (the long press).
static struct k_spinlock lock;

// While the screen is shown, the pad's bounds in screen coordinates, which
// the touch controller's match.
static bool shown;
static lv_area_t area;

// The touch in progress.
static bool touching;
// It started on the pad while the screen was shown.
static bool owned;
// It has gone past the slop, so it moves the pointer.
static bool moving;
// It was a long press, which has clicked; the rest of it is ignored.
static bool consumed;
static int64_t start_ms;
static int32_t start_x, start_y;
static int32_t last_x, last_y;

// The latest position from the touch controller, which reports it ahead of
// each INPUT_BTN_TOUCH. Only the input thread uses these.
static int32_t touch_x, touch_y;

static void report_key(uint16_t code, int32_t value)
{
    // K_NO_WAIT: touches arrive on the input thread, which is what empties
    // the queue, so waiting for room would never end.
    int err = input_report_key(touchpad, code, value, true, K_NO_WAIT);
    if (err < 0) {
        LOG_WRN("Dropped a button report: %d", err);
    }
}

static void click(uint16_t code)
{
    report_key(code, 1);
    report_key(code, 0);
}

static void move(int32_t dx, int32_t dy)
{
    int err = input_report_rel(touchpad, INPUT_REL_X, dx, false, K_NO_WAIT);
    if (err == 0) {
        err = input_report_rel(touchpad, INPUT_REL_Y, dy, true, K_NO_WAIT);
    }
    if (err < 0) {
        LOG_WRN("Dropped a movement report: %d", err);
    }
}

static void long_press_handler(struct k_work *work)
{
    ARG_UNUSED(work);

    k_spinlock_key_t key = k_spin_lock(&lock);
    // A late run, left over from a touch that has already ended, finds the
    // next touch too young.
    bool fire = touching && owned && !moving && !consumed &&
                k_uptime_get() - start_ms >= LONG_PRESS_MS;
    if (fire) {
        consumed = true;
    }
    k_spin_unlock(&lock, key);

    if (fire) {
        click(INPUT_BTN_1);
    }
}

static K_WORK_DELAYABLE_DEFINE(long_press_work, long_press_handler);

static void touch_down_or_move(int32_t x, int32_t y)
{
    bool start_long_press = false;
    bool cancel_long_press = false;
    int32_t dx = 0;
    int32_t dy = 0;

    k_spinlock_key_t key = k_spin_lock(&lock);

    if (!touching) {
        touching = true;
        owned = shown && x >= area.x1 && x <= area.x2 && y >= area.y1 && y <= area.y2;
        moving = false;
        consumed = false;
        start_ms = k_uptime_get();
        start_x = last_x = x;
        start_y = last_y = y;
        start_long_press = owned;
    } else if (owned && !consumed) {
        if (!moving) {
            int32_t sx = x - start_x;
            int32_t sy = y - start_y;

            if (sx * sx + sy * sy > TAP_SLOP_PX * TAP_SLOP_PX) {
                // Past the slop the pointer catches up with the finger, from
                // where the touch started.
                moving = true;
                cancel_long_press = true;
            }
        }

        if (moving) {
            dx = x - last_x;
            dy = y - last_y;
            last_x = x;
            last_y = y;
        }
    }

    k_spin_unlock(&lock, key);

    if (start_long_press) {
        k_work_reschedule(&long_press_work, K_MSEC(LONG_PRESS_MS));
    }
    if (cancel_long_press) {
        k_work_cancel_delayable(&long_press_work);
    }
    if (dx != 0 || dy != 0) {
        move(dx, dy);
    }
}

static void touch_up(void)
{
    k_spinlock_key_t key = k_spin_lock(&lock);
    bool tap = touching && owned && !moving && !consumed &&
               k_uptime_get() - start_ms <= TAP_TIMEOUT_MS;
    touching = false;
    owned = false;
    k_spin_unlock(&lock, key);

    k_work_cancel_delayable(&long_press_work);

    if (tap) {
        click(INPUT_BTN_0);
    }
}

// On the input thread. The touch controller reports INPUT_ABS_X and
// INPUT_ABS_Y, then INPUT_BTN_TOUCH with sync set, for every sample while a
// finger is down, and INPUT_BTN_TOUCH 0 once it lifts.
static void touch_input_cb(struct input_event *evt, void *user_data)
{
    ARG_UNUSED(user_data);

    switch (evt->type) {
    case INPUT_EV_ABS:
        if (evt->code == INPUT_ABS_X) {
            touch_x = evt->value;
        } else if (evt->code == INPUT_ABS_Y) {
            touch_y = evt->value;
        }
        break;

    case INPUT_EV_KEY:
        if (evt->code != INPUT_BTN_TOUCH) {
            break;
        }
        if (evt->value) {
            touch_down_or_move(touch_x, touch_y);
        } else {
            touch_up();
        }
        break;

    default:
        break;
    }
}

INPUT_CALLBACK_DEFINE(DEVICE_DT_GET(DT_PHANDLE(TOUCHPAD_NODE, touch_device)), touch_input_cb,
                      NULL);

// On the LVGL thread.
static void set_shown(bool show)
{
    lv_area_t bounds = {0};

    if (show) {
        lv_obj_t *obj = dt_lvgl_widget_obj_get(area_dev);
        if (obj == NULL) {
            LOG_ERR("No touchpad area widget");
            return;
        }
        lv_obj_update_layout(obj);
        lv_obj_get_coords(obj, &bounds);
    }

    k_spinlock_key_t key = k_spin_lock(&lock);
    shown = show;
    area = bounds;
    if (!show) {
        // A touch still on the pad stops moving the pointer.
        owned = false;
    }
    k_spin_unlock(&lock, key);

    if (!show) {
        k_work_cancel_delayable(&long_press_work);
    }
}

static void screen_event_cb(lv_event_t *e)
{
    switch (lv_event_get_code(e)) {
    case LV_EVENT_SCREEN_LOADED:
        set_shown(true);
        break;
    case LV_EVENT_SCREEN_UNLOAD_START:
        set_shown(false);
        break;
    default:
        break;
    }
}

static void touchpad_attach(void *user_data)
{
    ARG_UNUSED(user_data);

    lv_obj_t *screen = dt_lvgl_screen_get(screen_dev);
    if (screen == NULL) {
        LOG_ERR("No touchpad screen");
        return;
    }

    lv_obj_add_event_cb(screen, screen_event_cb, LV_EVENT_SCREEN_LOADED, NULL);
    lv_obj_add_event_cb(screen, screen_event_cb, LV_EVENT_SCREEN_UNLOAD_START, NULL);

    // src/sleep.c may already have brought the screen back on a wake.
    if (lv_screen_active() == screen) {
        set_shown(true);
    }
}

static struct dt_lvgl_async attach_work;

// LVGL calls have to run on the dt-lvgl thread, which creates the screens
// before it handles queued work.
static int touchpad_init(void)
{
    dt_lvgl_async_init(&attach_work, touchpad_attach, NULL);
    return dt_lvgl_async_submit(&attach_work);
}

SYS_INIT(touchpad_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);

// The device only gives the zmk,input-listener something to listen to; it
// has no hardware of its own. It refers to the dt-lvgl screen and widget, so
// it has to come after them, and dt-lvgl gives each nesting level a later
// priority. The build checks the order.
#define TOUCHPAD_INIT_PRIORITY 99

DEVICE_DT_INST_DEFINE(0, NULL, NULL, NULL, NULL, POST_KERNEL, TOUCHPAD_INIT_PRIORITY, NULL);

#endif
