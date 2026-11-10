#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/display.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/pm/device.h>
#include <zephyr/retention/retention.h>

#include <hal/nrf_gpio.h>
#include <hal/nrf_power.h>
#include <soc.h>

#include <dt_lvgl.h>

#include <zmk/activity.h>
#include <zmk/event_manager.h>
#include <zmk/events/activity_state_changed.h>

#include "ir_rx_enable.h"

LOG_MODULE_REGISTER(remote_sleep, CONFIG_ZMK_LOG_LEVEL);

// ZMK's idle sleep is System OFF, so waking is a reboot. Before sleep, the
// MPU6050 is armed for wake-on-motion and its INT pin becomes the wake source;
// on boot, wake-on-motion is undone. The register sequences follow the ESPHome
// project's accelerometer.yaml.
//
// The MPU6050 runs from the battery. ZMK suspends EXT_POWER on the way to
// sleep, cutting VCC to the display, the touch controller and the I2C
// pull-ups, so every pin to them is released before it does.

// AD0 is low.
#define MPU6050_ADDR 0x68

#define MPU6050_ACCEL_CONFIG 0x1C
#define MPU6050_MOT_THR 0x1F
#define MPU6050_MOT_DUR 0x20
#define MPU6050_INT_PIN_CFG 0x37
#define MPU6050_INT_ENABLE 0x38
#define MPU6050_INT_STATUS 0x3A
#define MPU6050_MOT_DETECT_CTRL 0x69
#define MPU6050_PWR_MGMT_1 0x6B
#define MPU6050_PWR_MGMT_2 0x6C

// Wake-on-motion threshold, ~2 mg per count (1-255). Lower is more sensitive.
#define MPU6050_MOTION_THRESHOLD 10

// Generous: the datasheet gives the accelerometer tens of milliseconds to start.
#define MPU6050_ACCEL_STARTUP_MS 50

// The MPU6050 INT pin, active high.
#define IMU_INT_PIN NRF_DT_GPIOS_TO_PSEL(DT_PATH(zephyr_user), imu_int_gpios)

// After EXT_POWER (81 in ZMK's ext_power_generic.c, which waits for VCC to
// settle) and before the CST816 driver first uses the bus.
#define I2C_RECOVER_INIT_PRIORITY 82
BUILD_ASSERT(I2C_RECOVER_INIT_PRIORITY < CONFIG_INPUT_INIT_PRIORITY,
             "The I2C bus must be recovered before the touch driver starts");

// Adafruit nRF52 bootloader (0.5.0 and later) GPREGRET value that skips its DFU
// checks, and so its 500 ms double-reset window, on the next boot only. The
// bootloader clears it, so a double reset still enters the bootloader later.
#define BOOTLOADER_MAGIC_SKIP 0x6D

// GPREGRET2, which nothing else uses. Like GPREGRET it is kept through System
// OFF and cleared at power-on, so it carries the screen shown at sleep over to
// the wake. It holds that screen's index plus one; 0 leaves the default screen.
#define SCREEN_GPREGRET 1

#define DISPLAY_DBI DT_PARENT(DT_CHOSEN(zephyr_display))
#define TOUCH_NODE DT_NODELABEL(cst816)

#define UI_NODE DT_NODELABEL(ui)
#define UI_SCREEN(node_id) DEVICE_DT_GET(node_id),
#define UI_SCREEN_ID(node_id) DT_PROP(node_id, id),

#define TRY(expr)                                                                                  \
    do {                                                                                           \
        err = (expr);                                                                              \
        if (err < 0) {                                                                             \
            return err;                                                                            \
        }                                                                                          \
    } while (0)

static const struct device *const i2c = DEVICE_DT_GET(DT_NODELABEL(i2c0));
static const struct device *const display = DEVICE_DT_GET(DT_CHOSEN(zephyr_display));
static const struct gpio_dt_spec display_cs =
    GPIO_DT_SPEC_GET_BY_IDX(DT_PHANDLE(DISPLAY_DBI, spi_dev), cs_gpios, 0);
static const struct gpio_dt_spec display_dc = GPIO_DT_SPEC_GET(DISPLAY_DBI, dc_gpios);
static const struct gpio_dt_spec display_reset = GPIO_DT_SPEC_GET(DISPLAY_DBI, reset_gpios);
static const struct gpio_dt_spec touch_irq = GPIO_DT_SPEC_GET(TOUCH_NODE, irq_gpios);
static const struct gpio_dt_spec touch_reset = GPIO_DT_SPEC_GET(TOUCH_NODE, rst_gpios);
static const struct device *const backlight = DEVICE_DT_GET(DT_CHOSEN(zmk_backlight));
// GPREGRET, where ZMK also writes the bootloader's magic values.
static const struct device *const boot_magic = DEVICE_DT_GET(DT_CHOSEN(zmk_magic_boot_mode));
static const struct device *const ui = DEVICE_DT_GET(UI_NODE);
static const struct device *const ui_screens[] = {
    DT_FOREACH_CHILD_STATUS_OKAY(UI_NODE, UI_SCREEN)};
static const char *const ui_screen_ids[] = {DT_FOREACH_CHILD_STATUS_OKAY(UI_NODE, UI_SCREEN_ID)};

static int mpu6050_write(uint8_t reg, uint8_t value)
{
    return i2c_reg_write_byte(i2c, MPU6050_ADDR, reg, value);
}

static int mpu6050_read(uint8_t reg, uint8_t *value)
{
    return i2c_reg_read_byte(i2c, MPU6050_ADDR, reg, value);
}

static int mpu6050_arm_wake_on_motion(void)
{
    uint8_t accel_config;
    uint8_t int_status;
    int err;

    // Disable interrupts while reconfiguring. Accel only, on the internal
    // oscillator (required once the gyros are in standby), temp disabled.
    TRY(mpu6050_write(MPU6050_INT_ENABLE, 0x00));
    TRY(mpu6050_write(MPU6050_PWR_MGMT_1, 0x08));
    TRY(mpu6050_write(MPU6050_PWR_MGMT_2, 0x07));

    // After power-up the MPU6050 sits in sleep mode until the write above.
    // Give the accelerometer time to start, or the reference held below is
    // taken from unsettled samples and reads as motion straight away.
    k_msleep(MPU6050_ACCEL_STARTUP_MS);

    // Keep the ±2 g range; reset the motion high-pass filter.
    TRY(mpu6050_read(MPU6050_ACCEL_CONFIG, &accel_config));
    accel_config &= ~0x07;
    TRY(mpu6050_write(MPU6050_ACCEL_CONFIG, accel_config));

    // Threshold held for one sample; 1 ms accel on-delay and a motion counter
    // decrement of 1.
    TRY(mpu6050_write(MPU6050_MOT_THR, MPU6050_MOTION_THRESHOLD));
    TRY(mpu6050_write(MPU6050_MOT_DUR, 1));
    TRY(mpu6050_write(MPU6050_MOT_DETECT_CTRL, 0x15));

    // INT active-high, push-pull, latched until INT_STATUS is read.
    TRY(mpu6050_write(MPU6050_INT_PIN_CFG, 0x20));
    TRY(mpu6050_read(MPU6050_INT_STATUS, &int_status));
    TRY(mpu6050_write(MPU6050_INT_ENABLE, 0x40));

    // Let the filter settle, then hold the current orientation as the
    // reference so any change from it counts as motion.
    TRY(mpu6050_write(MPU6050_ACCEL_CONFIG, accel_config | 0x01));
    k_msleep(10);
    TRY(mpu6050_write(MPU6050_ACCEL_CONFIG, accel_config | 0x07));

    // Low-power accel cycling: wake at 5 Hz (LP_WAKE_CTRL = 1), gyros off.
    TRY(mpu6050_write(MPU6050_PWR_MGMT_2, 0x47));
    TRY(mpu6050_write(MPU6050_PWR_MGMT_1, 0x28));

    // Release anything latched while the reference was being set up.
    TRY(mpu6050_read(MPU6050_INT_STATUS, &int_status));

    return 0;
}

// The MPU6050 keeps its registers across a nRF52 reset, so this undoes
// mpu6050_arm_wake_on_motion() and releases the latched interrupt.
static int mpu6050_disarm_wake_on_motion(void)
{
    uint8_t value;
    int err;

    TRY(mpu6050_write(MPU6050_INT_ENABLE, 0x00));
    TRY(mpu6050_read(MPU6050_INT_STATUS, &value));

    // Leave cycle mode and bring the gyros out of standby.
    TRY(mpu6050_read(MPU6050_PWR_MGMT_1, &value));
    TRY(mpu6050_write(MPU6050_PWR_MGMT_1, value & ~0x20));
    TRY(mpu6050_write(MPU6050_PWR_MGMT_2, 0x00));

    // Clear the motion high-pass filter "hold" setting.
    TRY(mpu6050_read(MPU6050_ACCEL_CONFIG, &value));
    TRY(mpu6050_write(MPU6050_ACCEL_CONFIG, value & ~0x07));

    return 0;
}

static int enable_motion_wake(void)
{
    int err;

    TRY(mpu6050_arm_wake_on_motion());

    // Only SENSE is set, with no interrupt: SENSE alone wakes the nRF52 from
    // System OFF, while a Zephyr level interrupt keeps firing for as long as
    // the latched INT is high, starving the sleep path. If INT is already high
    // at power-off, the nRF52 just wakes straight away.
    nrf_gpio_cfg_sense_input(IMU_INT_PIN, NRF_GPIO_PIN_PULLDOWN, NRF_GPIO_PIN_SENSE_HIGH);

    return 0;
}

// Display Off, then Sleep In, so the panel is idle when VCC goes. ZMK then
// skips the display, as it is already suspended, and sends it nothing more.
static int display_sleep(void)
{
    int err;

    TRY(display_blanking_on(display));
    TRY(pm_device_action_run(display, PM_DEVICE_ACTION_SUSPEND));

    return 0;
}

// The GPIOs to the display header (J1). Pinctrl sleep states release the SPI,
// I2C and PWM pins when ZMK suspends those devices.
static const struct gpio_dt_spec *const released_pins[] = {
    &display_cs, &display_dc, &display_reset, &touch_reset, &touch_irq,
};

// A pin left high would power the display and touch controller through their
// input protection once VCC is cut. The pins are disconnected rather than
// driven low: with VCC off, nothing pulls them up. TP_RST is on P0.10, and
// the matrix holds its NFC pad partner P0.09 high through sleep, which leaks
// current between the two if P0.10 is driven low.
//
// The display, SPI and touch drivers set these pins' direction only at init, so
// they stay released even if the LVGL thread draws again before power-off.
static int release_pins(void)
{
    int err;

    // The touch IRQ goes low with VCC.
    TRY(gpio_pin_interrupt_configure_dt(&touch_irq, GPIO_INT_DISABLE));

    for (size_t i = 0; i < ARRAY_SIZE(released_pins); i++) {
        TRY(gpio_pin_configure_dt(released_pins[i], GPIO_DISCONNECTED));
    }

    // In case sleep came in the middle of learning a code.
    TRY(ir_rx_set_enabled(false));

    // The backlight's pwm-leds device starts after EXT_POWER, so ZMK would
    // suspend it, and with it PWM0 and the backlight pin, while VCC is still
    // on. Skipped, PWM0 starts before EXT_POWER and is suspended after it.
    pm_device_busy_set(backlight);

    return 0;
}

// This only reads the active screen pointer, so it does not need the LVGL
// thread.
static void save_screen(void)
{
    lv_obj_t *active = lv_screen_active();
    uint32_t value = 0;

    for (size_t i = 0; i < ARRAY_SIZE(ui_screens); i++) {
        if (dt_lvgl_screen_get(ui_screens[i]) == active) {
            value = i + 1;
            break;
        }
    }

    nrf_power_gpregret_set(NRF_POWER, SCREEN_GPREGRET, value);
}

// Static threads start after the APPLICATION level, so this load is queued
// before the LVGL thread draws its first frame, and the default screen is
// never drawn first.
static void restore_screen(void)
{
    uint32_t value = nrf_power_gpregret_get(NRF_POWER, SCREEN_GPREGRET);

    nrf_power_gpregret_set(NRF_POWER, SCREEN_GPREGRET, 0);

    if (value == 0 || value > ARRAY_SIZE(ui_screens)) {
        return;
    }

    int err = dt_lvgl_screen_load_by_id_async(ui, ui_screen_ids[value - 1]);
    if (err < 0) {
        LOG_WRN("Could not restore the screen shown at sleep (%d)", err);
    }
}

// Runs before ZMK suspends the devices and powers off.
static void prepare_for_sleep(void)
{
    save_screen();

    // The I2C pull-ups are on VCC, so this goes before the pins are released.
    int err = enable_motion_wake();
    if (err < 0) {
        LOG_ERR("Could not arm wake-on-motion (%d); only a key or reset will wake the remote",
                err);
    }

    err = display_sleep();
    if (err < 0) {
        LOG_WRN("Could not put the display to sleep (%d)", err);
    }

    err = release_pins();
    if (err < 0) {
        LOG_ERR("Could not release the display pins (%d)", err);
    }

    // GPREGRET is kept through System OFF, so waking goes straight to the app.
    uint8_t magic = BOOTLOADER_MAGIC_SKIP;
    err = retention_write(boot_magic, 0, &magic, sizeof(magic));
    if (err < 0) {
        LOG_WRN("Could not set the bootloader skip flag (%d)", err);
    }
}

static int sleep_listener(const zmk_event_t *eh)
{
    const struct zmk_activity_state_changed *ev = as_zmk_activity_state_changed(eh);

    if (ev != NULL && ev->state == ZMK_ACTIVITY_SLEEP) {
        prepare_for_sleep();
    }

    return ZMK_EV_EVENT_BUBBLE;
}

ZMK_LISTENER(remote_sleep, sleep_listener);
ZMK_SUBSCRIPTION(remote_sleep, zmk_activity_state_changed);

static int sleep_init(void)
{
    restore_screen();

    if (!device_is_ready(i2c)) {
        LOG_ERR("I2C bus is not ready");
        return 0;
    }

    int err = mpu6050_disarm_wake_on_motion();
    if (err < 0) {
        LOG_WRN("Could not clear MPU6050 wake-on-motion (%d)", err);
    }

    return 0;
}

SYS_INIT(sleep_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);

// VCC, and the I2C pull-ups with it, was off through sleep while the MPU6050
// stayed powered. The bus falling and rising again can look like part of a
// transfer to it, leaving it holding SDA low, so clock the bus free first.
static int i2c_bus_recover(void)
{
    if (!device_is_ready(i2c)) {
        return 0;
    }

    int err = i2c_recover_bus(i2c);
    if (err < 0) {
        LOG_WRN("Could not recover the I2C bus (%d)", err);
    }

    return 0;
}

SYS_INIT(i2c_bus_recover, POST_KERNEL, I2C_RECOVER_INIT_PRIORITY);
