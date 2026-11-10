#include <stddef.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gap.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/byteorder.h>

#include <zmk/battery.h>
#include <zmk/event_manager.h>
#include <zmk/events/battery_state_changed.h>

#include <dt_lvgl.h>

LOG_MODULE_REGISTER(bthome, CONFIG_ZMK_LOG_LEVEL);

// Sends the UI buttons to Home Assistant as BTHome v2 broadcasts
// (https://bthome.io/format/). BTHome tells buttons apart only by their
// position in the packet, and a button object per UI button does not fit in
// an advertisement, so each press is one dimmer event with the button number
// as its step count: a tap rotates right, a long press rotates left. Home
// Assistant fires these as bthome_ble_event, with the button number in
// event_properties.steps. Every packet also carries the battery level and
// voltage, and a battery change on its own sends a packet without an event.

#define BTHOME_UUID 0xfcd2
// BTHome v2, unencrypted, sent on events rather than at a regular interval.
#define BTHOME_DEVICE_INFO 0x44

#define BTHOME_ID_PACKET_ID 0x00
#define BTHOME_ID_BATTERY 0x01
#define BTHOME_ID_VOLTAGE 0x0C
#define BTHOME_ID_DIMMER 0x3C

#define BTHOME_DIMMER_ROTATE_LEFT 0x01
#define BTHOME_DIMMER_ROTATE_RIGHT 0x02

// Each packet is advertised 20 times, 30 ms apart plus the controller's random
// delay of up to 10 ms: for about 0.7 s. That spans several scan windows of a
// receiver that only scans part of the time, such as an ESPHome Bluetooth
// proxy. Home Assistant drops the repeats, as they share a packet ID.
#define BTHOME_ADV_INTERVAL BT_GAP_MS_TO_ADV_INTERVAL(30)
#define BTHOME_ADV_PACKETS 20

// Presses wait here while an earlier one is still being advertised.
#define BTHOME_QUEUE_SIZE 8

struct bthome_payload {
    uint8_t uuid[2];
    uint8_t device_info;
    uint8_t packet_id_type;
    uint8_t packet_id;
    uint8_t battery_type;
    uint8_t battery;
    uint8_t voltage_type;
    uint16_t voltage;
    // Left off a battery-only packet.
    uint8_t dimmer_type;
    uint8_t dimmer_direction;
    uint8_t dimmer_steps;
} __packed;

struct bthome_press {
    uint8_t direction;
    uint8_t button;
};

static struct bthome_payload payload = {
    .uuid = {BT_UUID_16_ENCODE(BTHOME_UUID)},
    .device_info = BTHOME_DEVICE_INFO,
    .packet_id_type = BTHOME_ID_PACKET_ID,
    .battery_type = BTHOME_ID_BATTERY,
    .voltage_type = BTHOME_ID_VOLTAGE,
    .dimmer_type = BTHOME_ID_DIMMER,
};

// The service data length is set for each packet.
static struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR),
    BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_ZMK_KEYBOARD_NAME, sizeof(CONFIG_ZMK_KEYBOARD_NAME) - 1),
    BT_DATA(BT_DATA_SVC_DATA16, &payload, sizeof(payload)),
};

// Each AD structure has a length and a type byte ahead of its data.
BUILD_ASSERT((2 + 1) + (2 + sizeof(CONFIG_ZMK_KEYBOARD_NAME) - 1) +
                     (2 + sizeof(struct bthome_payload)) <=
                 BT_GAP_ADV_MAX_ADV_DATA_LEN,
             "BTHome packet does not fit in an advertisement; shorten ZMK_KEYBOARD_NAME");

// The identity address keeps the remote the same device in Home Assistant.
static const struct bt_le_adv_param adv_param = BT_LE_ADV_PARAM_INIT(
    BT_LE_ADV_OPT_USE_IDENTITY, BTHOME_ADV_INTERVAL, BTHOME_ADV_INTERVAL, NULL);

#define BTHOME_BUTTON(node_id, prop, idx) DEVICE_DT_GET(DT_PHANDLE_BY_IDX(node_id, prop, idx)),

// Each entry of bthome-buttons is sent as the number at the same position in
// bthome-button-numbers. Widgets not listed, such as navigation buttons, are
// not sent.
static const struct device *const buttons[] = {
    DT_FOREACH_PROP_ELEM(DT_PATH(zephyr_user), bthome_buttons, BTHOME_BUTTON)};

static const uint8_t button_numbers[] = DT_PROP(DT_PATH(zephyr_user), bthome_button_numbers);

BUILD_ASSERT(ARRAY_SIZE(buttons) == ARRAY_SIZE(button_numbers),
             "bthome-buttons and bthome-button-numbers differ in length");

#define BTHOME_BUTTON_NUMBER_VALID(node_id, prop, idx)                                             \
    BUILD_ASSERT(DT_PROP_BY_IDX(node_id, prop, idx) >= 1 &&                                        \
                     DT_PROP_BY_IDX(node_id, prop, idx) <= UINT8_MAX,                              \
                 "A dimmer step count holds a button number from 1 to 255");

DT_FOREACH_PROP_ELEM(DT_PATH(zephyr_user), bthome_button_numbers, BTHOME_BUTTON_NUMBER_VALID)

static const struct device *const battery = DEVICE_DT_GET(DT_CHOSEN(zmk_battery));

K_MSGQ_DEFINE(bthome_presses, sizeof(struct bthome_press), BTHOME_QUEUE_SIZE, 1);

static void bthome_send(struct k_work *work);
static K_WORK_DELAYABLE_DEFINE(bthome_send_work, bthome_send);

static void bthome_adv_sent(struct bt_le_ext_adv *instance, struct bt_le_ext_adv_sent_info *info);

static const struct bt_le_ext_adv_cb adv_callbacks = {
    .sent = bthome_adv_sent,
};

static struct bt_le_ext_adv *adv;
// Set while a packet is advertised, until bthome_adv_sent() runs.
static atomic_t advertising;
static bool advertising_press;
static atomic_t battery_changed;

static uint16_t battery_millivolts(void)
{
    struct sensor_value voltage;

    // ZMK's battery polling takes the reading; this only reads its result.
    if (sensor_channel_get(battery, SENSOR_CHAN_GAUGE_VOLTAGE, &voltage) < 0) {
        return 0;
    }

    return voltage.val1 * 1000 + voltage.val2 / 1000;
}

static void bthome_send(struct k_work *work)
{
    ARG_UNUSED(work);

    struct bthome_press press;
    bool has_press = k_msgq_peek(&bthome_presses, &press) == 0;

    if (atomic_get(&advertising)) {
        // A press cuts a battery-only packet short, as its own packet carries
        // the battery too. Anything else waits for bthome_adv_sent().
        if (!has_press || advertising_press) {
            return;
        }

        int err = bt_le_ext_adv_stop(adv);
        if (err < 0) {
            LOG_WRN("Could not stop the BTHome advertisement: %d", err);
            return;
        }

        atomic_clear(&advertising);
    }

    if (adv == NULL) {
        // Bluetooth is ready once ZMK has loaded its settings during boot.
        if (!bt_is_ready()) {
            k_work_reschedule(&bthome_send_work, K_MSEC(100));
            return;
        }

        int err = bt_le_ext_adv_create(&adv_param, &adv_callbacks, &adv);
        if (err < 0) {
            LOG_ERR("Could not create the BTHome advertising set: %d", err);
            return;
        }
    }

    // A press's packet carries the battery as well.
    bool battery_update = atomic_clear(&battery_changed);
    if (!has_press && !battery_update) {
        return;
    }

    size_t len = offsetof(struct bthome_payload, dimmer_type);
    if (has_press) {
        k_msgq_get(&bthome_presses, &press, K_NO_WAIT);
        payload.dimmer_direction = press.direction;
        payload.dimmer_steps = press.button;
        len = sizeof(payload);
    }

    payload.packet_id++;
    payload.battery = zmk_battery_state_of_charge();
    payload.voltage = sys_cpu_to_le16(battery_millivolts());
    ad[ARRAY_SIZE(ad) - 1].data_len = len;

    int err = bt_le_ext_adv_set_data(adv, ad, ARRAY_SIZE(ad), NULL, 0);
    if (err < 0) {
        LOG_ERR("Could not set the BTHome advertising data: %d", err);
        return;
    }

    advertising_press = has_press;
    atomic_set(&advertising, true);

    err = bt_le_ext_adv_start(adv, BT_LE_EXT_ADV_START_PARAM(0, BTHOME_ADV_PACKETS));
    if (err < 0) {
        atomic_clear(&advertising);
        LOG_ERR("Could not start the BTHome advertisement: %d", err);
    }
}

// Runs once a packet has been advertised BTHOME_ADV_PACKETS times, but not
// when bthome_send() stops it early.
static void bthome_adv_sent(struct bt_le_ext_adv *instance, struct bt_le_ext_adv_sent_info *info)
{
    ARG_UNUSED(instance);
    ARG_UNUSED(info);

    atomic_clear(&advertising);
    k_work_reschedule(&bthome_send_work, K_NO_WAIT);
}

static void bthome_queue_press(uint8_t direction, uint8_t button)
{
    struct bthome_press press = {
        .direction = direction,
        .button = button,
    };

    if (k_msgq_put(&bthome_presses, &press, K_NO_WAIT) < 0) {
        LOG_WRN("BTHome queue full; dropped button %u", button);
        return;
    }

    k_work_reschedule(&bthome_send_work, K_NO_WAIT);
}

static uint8_t button_number(const struct device *widget)
{
    for (size_t i = 0; i < ARRAY_SIZE(buttons); i++) {
        if (buttons[i] == widget) {
            return button_numbers[i];
        }
    }

    return 0;
}

// LVGL also reports a click when a long press is released, so the click that
// ends a long press is skipped.
static const struct device *long_pressed;

static void bthome_ui_event(const struct dt_lvgl_event *event, void *user_data)
{
    ARG_UNUSED(user_data);

    uint8_t direction;

    switch (event->event) {
    case DT_LVGL_EVENT_PRESSED:
        long_pressed = NULL;
        return;
    case DT_LVGL_EVENT_LONG_PRESSED:
        long_pressed = event->widget;
        direction = BTHOME_DIMMER_ROTATE_LEFT;
        break;
    case DT_LVGL_EVENT_CLICKED:
        if (event->widget == long_pressed) {
            return;
        }
        direction = BTHOME_DIMMER_ROTATE_RIGHT;
        break;
    default:
        return;
    }

    uint8_t button = button_number(event->widget);
    if (button != 0) {
        bthome_queue_press(direction, button);
    }
}

static int bthome_battery_listener(const zmk_event_t *eh)
{
    ARG_UNUSED(eh);

    atomic_set(&battery_changed, true);
    k_work_reschedule(&bthome_send_work, K_NO_WAIT);

    return ZMK_EV_EVENT_BUBBLE;
}

ZMK_LISTENER(bthome, bthome_battery_listener);
ZMK_SUBSCRIPTION(bthome, zmk_battery_state_changed);

static struct dt_lvgl_event_callback bthome_event_callback;

static int bthome_init(void)
{
    dt_lvgl_event_callback_init(&bthome_event_callback, bthome_ui_event, NULL);
    return dt_lvgl_event_callback_register(&bthome_event_callback);
}

SYS_INIT(bthome_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
