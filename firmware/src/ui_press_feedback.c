#include <zephyr/init.h>
#include <zephyr/logging/log.h>

#include <lvgl.h>

#include <dt_lvgl.h>

LOG_MODULE_REGISTER(ui_press_feedback, CONFIG_ZMK_LOG_LEVEL);

// LVGL draws a button pressed as soon as it is touched, and only clears that
// once the finger has moved far enough to start a scroll, so every scroll
// flashed the button it started on. Instead, a button is drawn pressed only
// once it has been held this long without a scroll starting.
#define UI_PRESS_SHOW_DELAY_MS 100
// A tap released before the delay is up is still drawn pressed for this long.
#define UI_PRESS_TAP_FLASH_MS 100

static lv_indev_t *pointer;
static lv_timer_t *show_timer;
static lv_timer_t *flash_timer;

// The touched button, until it is drawn pressed or the touch ends.
static lv_obj_t *delayed_obj;
// The tapped button drawn pressed by flash_timer.
static lv_obj_t *flash_obj;

static void restart_timer(lv_timer_t *timer)
{
    lv_timer_reset(timer);
    lv_timer_resume(timer);
}

static void end_flash(void)
{
    lv_timer_pause(flash_timer);
    if (flash_obj != NULL && lv_obj_is_valid(flash_obj)) {
        lv_obj_remove_state(flash_obj, LV_STATE_PRESSED);
    }
    flash_obj = NULL;
}

static void flash_timer_cb(lv_timer_t *timer)
{
    ARG_UNUSED(timer);
    end_flash();
}

static void show_timer_cb(lv_timer_t *timer)
{
    lv_timer_pause(timer);

    // Buttons are press-locked, so while the touch lasts and has not become a
    // scroll it is still on delayed_obj.
    if (delayed_obj != NULL && lv_indev_get_state(pointer) == LV_INDEV_STATE_PRESSED &&
        lv_indev_get_scroll_obj(pointer) == NULL) {
        lv_obj_add_state(delayed_obj, LV_STATE_PRESSED);
    }
    delayed_obj = NULL;
}

// Indev events are sent before the matching event reaches the widget.
static void pointer_event_cb(lv_event_t *e)
{
    lv_obj_t *obj = lv_event_get_param(e);

    switch (lv_event_get_code(e)) {
    case LV_EVENT_PRESSED:
        end_flash();
        delayed_obj = NULL;

        if (obj != NULL && lv_obj_check_type(obj, &lv_button_class)) {
            // The widget's own PRESSED handler is what sets the pressed state,
            // so the button does not get this event.
            lv_indev_stop_processing(pointer);
            delayed_obj = obj;
            restart_timer(show_timer);
        }
        break;

    case LV_EVENT_CLICKED:
        // LVGL only clicks when the touch was not a scroll. By now the
        // widget's RELEASED handler has cleared the pressed state.
        if (obj != NULL && obj == delayed_obj) {
            lv_timer_pause(show_timer);
            lv_obj_add_state(obj, LV_STATE_PRESSED);
            flash_obj = obj;
            restart_timer(flash_timer);
        }
        delayed_obj = NULL;
        break;

    case LV_EVENT_INDEV_RESET:
        delayed_obj = NULL;
        break;

    default:
        break;
    }
}

static void ui_press_feedback_attach(void *user_data)
{
    ARG_UNUSED(user_data);

    for (lv_indev_t *indev = lv_indev_get_next(NULL); indev != NULL;
         indev = lv_indev_get_next(indev)) {
        if (lv_indev_get_type(indev) == LV_INDEV_TYPE_POINTER) {
            pointer = indev;
            break;
        }
    }

    if (pointer == NULL) {
        LOG_WRN("No touch input found");
        return;
    }

    show_timer = lv_timer_create(show_timer_cb, UI_PRESS_SHOW_DELAY_MS, NULL);
    flash_timer = lv_timer_create(flash_timer_cb, UI_PRESS_TAP_FLASH_MS, NULL);
    if (show_timer == NULL || flash_timer == NULL) {
        LOG_ERR("Could not create press feedback timers");
        return;
    }
    lv_timer_pause(show_timer);
    lv_timer_pause(flash_timer);

    lv_indev_add_event_cb(pointer, pointer_event_cb, LV_EVENT_ALL, NULL);
}

static struct dt_lvgl_async attach_work;

// LVGL calls have to run on the dt-lvgl thread, which creates the touch input
// before it handles queued work.
static int ui_press_feedback_init(void)
{
    dt_lvgl_async_init(&attach_work, ui_press_feedback_attach, NULL);
    return dt_lvgl_async_submit(&attach_work);
}

SYS_INIT(ui_press_feedback_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
