#define DT_DRV_COMPAT universal_remote_motion_pointer

#include <stdlib.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/init.h>
#include <zephyr/input/input.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/settings/settings.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/byteorder.h>

#include <lvgl.h>

#include <dt_lvgl.h>

#include <zmk/activity.h>
#include <zmk/event_manager.h>
#include <zmk/events/activity_state_changed.h>

#include "motion_pointer.h"

LOG_MODULE_REGISTER(motion_pointer, CONFIG_ZMK_LOG_LEVEL);

#if DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT)

BUILD_ASSERT(DT_NUM_INST_STATUS_OKAY(DT_DRV_COMPAT) == 1, "Only one motion pointer is supported");

// While the Touchpad screen is shown with Tilt on, the MPU6050's gyroscope
// is read every sample-period-ms and its turn rates move the pointer: yaw
// across and pitch up and down. Otherwise the MPU6050 sleeps, which saves the
// few milliamps its gyroscope draws.
//
// A gyroscope reads slightly off zero even when still, by up to 20 deg/s on
// the MPU6050, which would make the pointer drift. That offset is measured
// each time the gyroscope starts, before the pointer moves, and then follows
// the gyroscope while the remote is held still.
//
// All MPU6050 access is on the system work queue, which is also where ZMK
// decides to sleep: src/sleep.c then arms the MPU6050 to wake the remote, and
// nothing here touches it again.

#define MOTION_NODE DT_DRV_INST(0)

#define I2C_ADDR DT_PROP(MOTION_NODE, address)
#define X_AXIS DT_ENUM_IDX(MOTION_NODE, x_axis)
#define Y_AXIS DT_ENUM_IDX(MOTION_NODE, y_axis)
#define X_SIGN (DT_PROP(MOTION_NODE, invert_x) ? -1 : 1)
#define Y_SIGN (DT_PROP(MOTION_NODE, invert_y) ? -1 : 1)
#define PIXELS_PER_DEGREE DT_PROP(MOTION_NODE, pixels_per_degree)
#define DEADZONE_MDPS DT_PROP(MOTION_NODE, deadzone_mdps)
#define SAMPLE_PERIOD_MS DT_PROP(MOTION_NODE, sample_period_ms)
#define TOUCH_HOLDOFF_MS DT_PROP(MOTION_NODE, touch_holdoff_ms)

BUILD_ASSERT(X_AXIS != Y_AXIS, "x-axis and y-axis must be different gyroscope axes");

#define MPU6050_CONFIG 0x1A
#define MPU6050_GYRO_CONFIG 0x1B
#define MPU6050_GYRO_XOUT_H 0x43
#define MPU6050_PWR_MGMT_1 0x6B
#define MPU6050_PWR_MGMT_2 0x6C
#define MPU6050_WHO_AM_I 0x75

// Gyroscope X's PLL as the clock, temperature sensor off.
#define PWR_MGMT_1_RUN 0x09
// The power-on value: asleep.
#define PWR_MGMT_1_SLEEP 0x40
// The accelerometer in standby, the gyroscope on.
#define PWR_MGMT_2_GYRO_ONLY 0x38
// A 44 Hz low-pass filter, which takes out most of the noise without lag
// a hand would notice.
#define CONFIG_DLPF_44HZ 0x03
// +/-500 deg/s full scale, 65.5 counts per deg/s: a quick flick of the wrist
// goes past 250 deg/s.
#define GYRO_CONFIG_500DPS 0x08
#define COUNTS_PER_DPS_X10 655

// The gyroscope takes about 30 ms to start, and the filter a little longer.
#define START_DELAY_MS 100
// The offset is the mean of this many samples, taken while the remote is
// still. Readings further apart than CALIBRATE_SPREAD mean it moved.
#define CALIBRATE_SAMPLES 32
#define CALIBRATE_SPREAD (3 * COUNTS_PER_DPS_X10 / 10)
// After this many tries the offset is taken anyway, and tracking corrects it.
#define CALIBRATE_TRIES 3

// While every axis stays within STILL_THRESHOLD of the offset for
// STILL_SAMPLES samples, the remote is still, and the offset eases towards
// the readings by 1/TRACK_DIVISOR a sample.
#define STILL_THRESHOLD (3 * COUNTS_PER_DPS_X10 / 20)
#define STILL_SAMPLES 25
#define TRACK_DIVISOR 64

// The offset is kept in sixteenths of a count.
#define BIAS_SCALE 16

#define DEADZONE_COUNTS (DEADZONE_MDPS * COUNTS_PER_DPS_X10 / 10000)

// A late sample, after the system work queue was busy, is counted as at
// most this long.
#define MAX_SAMPLE_MS 50

static const struct device *const motion = DEVICE_DT_GET(MOTION_NODE);
static const struct device *const i2c = DEVICE_DT_GET(DT_PHANDLE(MOTION_NODE, i2c_bus));
static const struct device *const toggle = DEVICE_DT_GET(DT_PHANDLE(MOTION_NODE, toggle));
static const struct device *const toggle_icon =
    DEVICE_DT_GET(DT_PHANDLE(MOTION_NODE, toggle_icon));

// Set from any thread.
static atomic_t shown;
static atomic_t enabled;
static atomic_t touching;
static atomic_t touch_lifted_ms;
static atomic_t asleep;

// Only the system work queue uses these.
static bool running;
static bool parked;
static bool calibrated;
static int calibrate_tries;
static int calibrate_count;
static int32_t calibrate_sum[3];
static int16_t calibrate_min[3];
static int16_t calibrate_max[3];
static int32_t bias[3];
static int still_count;
static int64_t last_sample_ms;
// Pointer movement still to send, in 1/(COUNTS_PER_DPS_X10 * 100) pixels.
static int64_t remainder_x;
static int64_t remainder_y;

static int mpu6050_write(uint8_t reg, uint8_t value)
{
    return i2c_reg_write_byte(i2c, I2C_ADDR, reg, value);
}

#define TRY(expr)                                                                                  \
    do {                                                                                           \
        int err_ = (expr);                                                                         \
        if (err_ < 0) {                                                                            \
            return err_;                                                                           \
        }                                                                                          \
    } while (0)

static int mpu6050_start(void)
{
    uint8_t who_am_i;

    TRY(mpu6050_write(MPU6050_PWR_MGMT_1, PWR_MGMT_1_RUN));
    TRY(mpu6050_write(MPU6050_PWR_MGMT_2, PWR_MGMT_2_GYRO_ONLY));
    TRY(mpu6050_write(MPU6050_CONFIG, CONFIG_DLPF_44HZ));
    TRY(mpu6050_write(MPU6050_GYRO_CONFIG, GYRO_CONFIG_500DPS));

    // Clones answer with other IDs but the same registers, so this only
    // helps make sense of a log.
    if (i2c_reg_read_byte(i2c, I2C_ADDR, MPU6050_WHO_AM_I, &who_am_i) == 0 && who_am_i != 0x68) {
        LOG_INF("MPU6050 WHO_AM_I is 0x%02x", who_am_i);
    }

    return 0;
}

// Back to the power-on settings, asleep. src/sleep.c starts from those when it
// arms wake-on-motion.
static int mpu6050_park(void)
{
    TRY(mpu6050_write(MPU6050_CONFIG, 0x00));
    TRY(mpu6050_write(MPU6050_GYRO_CONFIG, 0x00));
    TRY(mpu6050_write(MPU6050_PWR_MGMT_2, 0x00));
    TRY(mpu6050_write(MPU6050_PWR_MGMT_1, PWR_MGMT_1_SLEEP));

    return 0;
}

static int mpu6050_read_gyro(int16_t gyro[3])
{
    uint8_t buf[6];

    TRY(i2c_burst_read(i2c, I2C_ADDR, MPU6050_GYRO_XOUT_H, buf, sizeof(buf)));

    for (int i = 0; i < 3; i++) {
        gyro[i] = (int16_t)sys_get_be16(&buf[2 * i]);
    }

    return 0;
}

static void calibrate_reset(void)
{
    calibrate_count = 0;
    memset(calibrate_sum, 0, sizeof(calibrate_sum));
}

// True once the offset is known.
static bool calibrate(const int16_t gyro[3])
{
    for (int i = 0; i < 3; i++) {
        calibrate_sum[i] += gyro[i];
        if (calibrate_count == 0 || gyro[i] < calibrate_min[i]) {
            calibrate_min[i] = gyro[i];
        }
        if (calibrate_count == 0 || gyro[i] > calibrate_max[i]) {
            calibrate_max[i] = gyro[i];
        }
    }

    if (++calibrate_count < CALIBRATE_SAMPLES) {
        return false;
    }

    bool moved = false;
    for (int i = 0; i < 3; i++) {
        if (calibrate_max[i] - calibrate_min[i] > CALIBRATE_SPREAD) {
            moved = true;
        }
    }

    if (moved && ++calibrate_tries < CALIBRATE_TRIES) {
        calibrate_reset();
        return false;
    }

    for (int i = 0; i < 3; i++) {
        bias[i] = calibrate_sum[i] * BIAS_SCALE / CALIBRATE_SAMPLES;
    }
    LOG_DBG("Gyroscope offset %d %d %d (/16)%s", bias[0], bias[1], bias[2],
            moved ? ", taken while moving" : "");

    return true;
}

static void track_bias(const int16_t gyro[3])
{
    for (int i = 0; i < 3; i++) {
        if (abs(gyro[i] * BIAS_SCALE - bias[i]) > STILL_THRESHOLD * BIAS_SCALE) {
            still_count = 0;
            return;
        }
    }

    if (still_count < STILL_SAMPLES) {
        still_count++;
        return;
    }

    for (int i = 0; i < 3; i++) {
        bias[i] += (gyro[i] * BIAS_SCALE - bias[i]) / TRACK_DIVISOR;
    }
}

// A turn rate in counts, offset and dead zone taken off.
static int32_t rate(const int16_t gyro[3], int axis)
{
    int32_t value = (gyro[axis] * BIAS_SCALE - bias[axis]) / BIAS_SCALE;

    if (value > DEADZONE_COUNTS) {
        return value - DEADZONE_COUNTS;
    }
    if (value < -DEADZONE_COUNTS) {
        return value + DEADZONE_COUNTS;
    }
    return 0;
}

// counts * ms * px/deg, over (counts per deg/s * 1000 ms/s), is pixels.
#define PIXEL_UNITS ((int64_t)COUNTS_PER_DPS_X10 * 100)

static int32_t take_pixels(int64_t *remainder)
{
    int32_t pixels = (int32_t)(*remainder / PIXEL_UNITS);

    *remainder -= (int64_t)pixels * PIXEL_UNITS;
    return pixels;
}

static bool held_by_touch(void)
{
    if (atomic_get(&touching)) {
        return true;
    }

    uint32_t since = k_uptime_get_32() - (uint32_t)atomic_get(&touch_lifted_ms);

    return since < TOUCH_HOLDOFF_MS;
}

static void move_pointer(const int16_t gyro[3], int64_t elapsed_ms)
{
    if (held_by_touch()) {
        remainder_x = 0;
        remainder_y = 0;
        return;
    }

    remainder_x += (int64_t)X_SIGN * rate(gyro, X_AXIS) * elapsed_ms * PIXELS_PER_DEGREE;
    remainder_y += (int64_t)Y_SIGN * rate(gyro, Y_AXIS) * elapsed_ms * PIXELS_PER_DEGREE;

    int32_t dx = take_pixels(&remainder_x);
    int32_t dy = take_pixels(&remainder_y);

    if (dx == 0 && dy == 0) {
        return;
    }

    // K_NO_WAIT: a full input queue drops a sample rather than holding up the
    // system work queue.
    int err = input_report_rel(motion, INPUT_REL_X, dx, false, K_NO_WAIT);
    if (err == 0) {
        err = input_report_rel(motion, INPUT_REL_Y, dy, true, K_NO_WAIT);
    }
    if (err < 0) {
        LOG_WRN("Dropped a movement report: %d", err);
    }
}

static void sample_handler(struct k_work *work);
static K_WORK_DELAYABLE_DEFINE(sample_work, sample_handler);

static void sample_handler(struct k_work *work)
{
    ARG_UNUSED(work);

    if (!running || atomic_get(&asleep)) {
        return;
    }

    k_work_schedule(&sample_work, K_MSEC(SAMPLE_PERIOD_MS));

    int16_t gyro[3];
    int err = mpu6050_read_gyro(gyro);
    if (err < 0) {
        LOG_WRN("Could not read the gyroscope: %d", err);
        return;
    }

    int64_t now = k_uptime_get();
    int64_t elapsed_ms = MIN(now - last_sample_ms, MAX_SAMPLE_MS);
    last_sample_ms = now;

    if (!calibrated) {
        calibrated = calibrate(gyro);
        return;
    }

    track_bias(gyro);
    move_pointer(gyro, elapsed_ms);
}

// Starts or stops the gyroscope to match the screen and the toggle.
static void update_handler(struct k_work *work)
{
    ARG_UNUSED(work);

    if (atomic_get(&asleep)) {
        return;
    }

    bool want = atomic_get(&shown) && atomic_get(&enabled);
    int err;

    if (want && !running) {
        err = mpu6050_start();
        if (err < 0) {
            LOG_ERR("Could not start the gyroscope: %d", err);
            return;
        }
        running = true;
        parked = false;
        calibrated = false;
        calibrate_tries = 0;
        calibrate_reset();
        still_count = 0;
        remainder_x = 0;
        remainder_y = 0;
        last_sample_ms = k_uptime_get();
        k_work_schedule(&sample_work, K_MSEC(START_DELAY_MS));
    } else if (!want && (running || !parked)) {
        running = false;
        k_work_cancel_delayable(&sample_work);
        err = mpu6050_park();
        if (err < 0) {
            LOG_WRN("Could not put the MPU6050 to sleep: %d", err);
            return;
        }
        parked = true;
    }
}

static K_WORK_DEFINE(update_work, update_handler);

void motion_pointer_set_shown(bool show)
{
    atomic_set(&shown, show);
    if (!show) {
        atomic_set(&touching, false);
    }
    k_work_submit(&update_work);
}

void motion_pointer_set_touching(bool touch)
{
    if (!touch) {
        atomic_set(&touch_lifted_ms, (atomic_val_t)k_uptime_get_32());
    }
    atomic_set(&touching, touch);
}

static void save_handler(struct k_work *work)
{
    ARG_UNUSED(work);

    uint8_t value = atomic_get(&enabled) ? 1 : 0;
    int err = settings_save_one("motion/enabled", &value, sizeof(value));
    if (err < 0) {
        LOG_ERR("Could not save the motion setting: %d", err);
    }
}

static K_WORK_DEFINE(save_work, save_handler);

static const char *toggle_icon_text(bool on)
{
    return on ? DT_PROP(MOTION_NODE, icon_on) : DT_PROP(MOTION_NODE, icon_off);
}

// On the dt-lvgl thread. LVGL checks or unchecks the toggle on a tap.
static void toggle_event(const struct dt_lvgl_event *event, void *user_data)
{
    ARG_UNUSED(user_data);

    if (event->widget != toggle || event->event != DT_LVGL_EVENT_VALUE_CHANGED) {
        return;
    }

    bool on = lv_obj_has_state(event->obj, LV_STATE_CHECKED);

    dt_lvgl_widget_set_text(toggle_icon, toggle_icon_text(on));

    if (atomic_set(&enabled, on) != on) {
        k_work_submit(&update_work);
        k_work_submit(&save_work);
    }
}

static struct dt_lvgl_event_callback toggle_callback;

static int motion_settings_set(const char *key, size_t len, settings_read_cb read_cb, void *cb_arg)
{
    if (strcmp(key, "enabled") != 0) {
        return -ENOENT;
    }

    uint8_t value;

    if (len != sizeof(value)) {
        return -EINVAL;
    }

    int err = read_cb(cb_arg, &value, sizeof(value));
    if (err < 0) {
        return err;
    }

    atomic_set(&enabled, value != 0);
    return 0;
}

// On the dt-lvgl thread: the toggle and its icon show the loaded setting.
static void show_setting(void *user_data)
{
    ARG_UNUSED(user_data);

    bool on = atomic_get(&enabled);
    int err = dt_lvgl_widget_set_checked(toggle, on);

    if (err == 0) {
        err = dt_lvgl_widget_set_text(toggle_icon, toggle_icon_text(on));
    }
    if (err < 0) {
        LOG_WRN("Could not show the motion setting: %d", err);
    }
}

// Its own work item rather than dt-lvgl's shared update queue
// (dt_lvgl_widget_set_*_async), which holds only
// CONFIG_DT_LVGL_ASYNC_UPDATE_QUEUE_SIZE updates. At boot every copy of the
// status bar's battery label queues one at about the same time, and an update
// that finds the queue full is dropped.
static struct dt_lvgl_async show_setting_work;

// After ZMK loads settings in main(), so after src/sleep.c's SYS_INIT has
// finished with the MPU6050. The first update also parks it if motion is not
// in use.
static int motion_settings_commit(void)
{
    int err = dt_lvgl_async_submit(&show_setting_work);
    if (err < 0) {
        LOG_WRN("Could not show the motion setting: %d", err);
    }

    k_work_submit(&update_work);
    return 0;
}

SETTINGS_STATIC_HANDLER_DEFINE(motion_pointer, "motion", NULL, motion_settings_set,
                               motion_settings_commit, NULL);

// src/sleep.c arms the MPU6050 to wake the remote from the same event, on the
// same work queue; from then on nothing here touches it.
static int motion_activity_listener(const zmk_event_t *eh)
{
    const struct zmk_activity_state_changed *ev = as_zmk_activity_state_changed(eh);

    if (ev != NULL && ev->state == ZMK_ACTIVITY_SLEEP) {
        atomic_set(&asleep, true);
        running = false;
        k_work_cancel_delayable(&sample_work);
    }

    return ZMK_EV_EVENT_BUBBLE;
}

ZMK_LISTENER(motion_pointer, motion_activity_listener);
ZMK_SUBSCRIPTION(motion_pointer, zmk_activity_state_changed);

static int motion_pointer_init(void)
{
    dt_lvgl_async_init(&show_setting_work, show_setting, NULL);
    dt_lvgl_event_callback_init(&toggle_callback, toggle_event, NULL);
    return dt_lvgl_event_callback_register(&toggle_callback);
}

SYS_INIT(motion_pointer_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);

// The device only gives the zmk,input-listener something to listen to. It
// refers to dt-lvgl widgets, so it comes after them, like src/touchpad.c's.
#define MOTION_POINTER_INIT_PRIORITY 99

DEVICE_DT_INST_DEFINE(0, NULL, NULL, NULL, NULL, POST_KERNEL, MOTION_POINTER_INIT_PRIORITY, NULL);

#endif
