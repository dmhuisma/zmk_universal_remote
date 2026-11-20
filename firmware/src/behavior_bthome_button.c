#define DT_DRV_COMPAT zmk_behavior_bthome_button

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/util.h>

#include <drivers/behavior.h>

#include <zmk/behavior.h>

#include "bthome.h"

// A key bound to &bthome_button sends a press over BTHome like a touchscreen
// button: released before LVGL's long press time it is a tap, held that long
// it is a long press, sent as soon as the time is up.

#if DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT)

// LVGL's default, LV_INDEV_DEF_LONG_PRESS_TIME, so keys and tiles agree.
#define BTHOME_BUTTON_LONG_PRESS_MS 400

enum bthome_button_state {
    BTHOME_BUTTON_IDLE,
    // Held, nothing sent yet.
    BTHOME_BUTTON_HELD,
    // Held, and the long press has been sent.
    BTHOME_BUTTON_LONG_PRESSED,
};

struct behavior_bthome_button_config {
    uint8_t number;
};

struct behavior_bthome_button_data {
    const struct device *dev;
    struct k_work_delayable long_press_work;
    atomic_t state;
};

static void bthome_button_long_press(struct k_work *work)
{
    struct k_work_delayable *dwork = k_work_delayable_from_work(work);
    struct behavior_bthome_button_data *data =
        CONTAINER_OF(dwork, struct behavior_bthome_button_data, long_press_work);
    const struct behavior_bthome_button_config *config = data->dev->config;

    // Only one of this and the release sends, whichever gets there first.
    if (atomic_cas(&data->state, BTHOME_BUTTON_HELD, BTHOME_BUTTON_LONG_PRESSED)) {
        bthome_send_button(config->number, true);
    }
}

static int on_keymap_binding_pressed(struct zmk_behavior_binding *binding,
                                     struct zmk_behavior_binding_event event)
{
    ARG_UNUSED(event);

    const struct device *dev = zmk_behavior_get_binding(binding->behavior_dev);
    struct behavior_bthome_button_data *data = dev->data;

    atomic_set(&data->state, BTHOME_BUTTON_HELD);
    k_work_reschedule(&data->long_press_work, K_MSEC(BTHOME_BUTTON_LONG_PRESS_MS));

    return ZMK_BEHAVIOR_OPAQUE;
}

static int on_keymap_binding_released(struct zmk_behavior_binding *binding,
                                      struct zmk_behavior_binding_event event)
{
    ARG_UNUSED(event);

    const struct device *dev = zmk_behavior_get_binding(binding->behavior_dev);
    const struct behavior_bthome_button_config *config = dev->config;
    struct behavior_bthome_button_data *data = dev->data;

    k_work_cancel_delayable(&data->long_press_work);

    if (atomic_cas(&data->state, BTHOME_BUTTON_HELD, BTHOME_BUTTON_IDLE)) {
        bthome_send_button(config->number, false);
    } else {
        atomic_set(&data->state, BTHOME_BUTTON_IDLE);
    }

    return ZMK_BEHAVIOR_OPAQUE;
}

static const struct behavior_driver_api behavior_bthome_button_driver_api = {
    .binding_pressed = on_keymap_binding_pressed,
    .binding_released = on_keymap_binding_released,
    // BTHome is broadcast by the remote, so on a split build the key runs
    // there rather than on the dongle.
    .locality = BEHAVIOR_LOCALITY_EVENT_SOURCE,
#if IS_ENABLED(CONFIG_ZMK_BEHAVIOR_METADATA)
    .get_parameter_metadata = zmk_behavior_get_empty_param_metadata,
#endif
};

static int behavior_bthome_button_init(const struct device *dev)
{
    struct behavior_bthome_button_data *data = dev->data;

    data->dev = dev;
    k_work_init_delayable(&data->long_press_work, bthome_button_long_press);

    return 0;
}

#define BTHOME_BUTTON_INST(n)                                                                      \
    BUILD_ASSERT(DT_INST_PROP(n, button_number) >= 1 && DT_INST_PROP(n, button_number) <= 255,    \
                 "A BTHome button number is from 1 to 255");                                       \
    static const struct behavior_bthome_button_config behavior_bthome_button_config_##n = {        \
        .number = DT_INST_PROP(n, button_number),                                                  \
    };                                                                                             \
    static struct behavior_bthome_button_data behavior_bthome_button_data_##n;                     \
    BEHAVIOR_DT_INST_DEFINE(n, behavior_bthome_button_init, NULL,                                  \
                            &behavior_bthome_button_data_##n,                                      \
                            &behavior_bthome_button_config_##n, POST_KERNEL,                       \
                            CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,                                   \
                            &behavior_bthome_button_driver_api);

DT_INST_FOREACH_STATUS_OKAY(BTHOME_BUTTON_INST)

#endif // DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT)
