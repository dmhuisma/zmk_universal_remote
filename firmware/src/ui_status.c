#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/init.h>
#include <zephyr/logging/log.h>

#include <zmk/event_manager.h>

// A split peripheral's wireless link is its connection to the dongle. A
// central or unibody build links straight to a host over its active BLE
// profile.
#define UI_STATUS_SPLIT_PERIPHERAL                                                                 \
    (IS_ENABLED(CONFIG_ZMK_SPLIT) && !IS_ENABLED(CONFIG_ZMK_SPLIT_ROLE_CENTRAL))
#define UI_STATUS_BLE_HOST (IS_ENABLED(CONFIG_ZMK_BLE) && !UI_STATUS_SPLIT_PERIPHERAL)

#if UI_STATUS_SPLIT_PERIPHERAL
#include <zmk/events/split_peripheral_status_changed.h>
#elif UI_STATUS_BLE_HOST
#include <zmk/ble.h>
#include <zmk/events/ble_active_profile_changed.h>
#endif

#include <dt_lvgl.h>

LOG_MODULE_REGISTER(ui_status, CONFIG_ZMK_LOG_LEVEL);

#if UI_STATUS_SPLIT_PERIPHERAL || UI_STATUS_BLE_HOST

#define UI_WIRELESS_ICON(node_id, prop, idx) DEVICE_DT_GET(DT_PHANDLE_BY_IDX(node_id, prop, idx)),

// Every screen has its own copy of the status bar, listed in ui-wireless-icons.
static const struct device *const wireless_icons[] = {
    DT_FOREACH_PROP_ELEM(DT_PATH(zephyr_user), ui_wireless_icons, UI_WIRELESS_ICON)};

static void set_wireless_connected(bool connected)
{
    for (size_t i = 0; i < ARRAY_SIZE(wireless_icons); i++) {
        int err = dt_lvgl_widget_set_disabled_async(wireless_icons[i], !connected);
        if (err < 0) {
            LOG_WRN("Could not update wireless icon: %d", err);
        }
    }
}

// The icon is dimmed (disabled) until the wireless link is up.
static int ui_status_listener(const zmk_event_t *eh)
{
#if UI_STATUS_SPLIT_PERIPHERAL
    const struct zmk_split_peripheral_status_changed *ev =
        as_zmk_split_peripheral_status_changed(eh);

    if (ev != NULL) {
        set_wireless_connected(ev->connected);
    }
#else
    // ZMK raises this when the active profile connects or disconnects, as
    // well as when another profile is selected.
    if (as_zmk_ble_active_profile_changed(eh) != NULL) {
        set_wireless_connected(zmk_ble_active_profile_is_connected());
    }
#endif

    return ZMK_EV_EVENT_BUBBLE;
}

ZMK_LISTENER(ui_status, ui_status_listener);
#if UI_STATUS_SPLIT_PERIPHERAL
ZMK_SUBSCRIPTION(ui_status, zmk_split_peripheral_status_changed);
#else
ZMK_SUBSCRIPTION(ui_status, zmk_ble_active_profile_changed);
#endif

#endif

static void ui_event_handler(const struct dt_lvgl_event *event, void *user_data)
{
    ARG_UNUSED(user_data);

    if (event->event == DT_LVGL_EVENT_CLICKED && event->widget_id != NULL) {
        LOG_INF("%s pressed", event->widget_id);
    }
}

static struct dt_lvgl_event_callback ui_event_callback;

static int ui_status_init(void)
{
    dt_lvgl_event_callback_init(&ui_event_callback, ui_event_handler, NULL);
    return dt_lvgl_event_callback_register(&ui_event_callback);
}

SYS_INIT(ui_status_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
