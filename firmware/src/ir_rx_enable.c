#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "ir_rx_enable.h"

LOG_MODULE_REGISTER(remote_ir_rx, CONFIG_ZMK_LOG_LEVEL);

// The IR receiver is only powered while learning codes. zmk-ir's driver sets
// its pin up once, as an input with a pull-up and an edge interrupt, and the
// pull-up would feed the unpowered receiver through its output. So the pin is
// released along with the power, and set up again as the driver had it. The
// driver's edge callback stays registered throughout.

// C1 charging through R6 (about 0.5 ms), then the TSOP38238 settling.
#define IR_RX_POWER_UP_MS 10

#define TRY(expr)                                                                                  \
    do {                                                                                           \
        err = (expr);                                                                              \
        if (err < 0) {                                                                             \
            return err;                                                                            \
        }                                                                                          \
    } while (0)

static const struct gpio_dt_spec ir_power =
    GPIO_DT_SPEC_GET(DT_PATH(zephyr_user), ir_power_gpios);
static const struct gpio_dt_spec ir_rx = GPIO_DT_SPEC_GET(DT_NODELABEL(ir_rx), in_gpios);

static K_MUTEX_DEFINE(ir_rx_lock);
static bool ir_rx_enabled;

static int ir_rx_power_up(void)
{
    int err;

    TRY(gpio_pin_configure_dt(&ir_power, GPIO_OUTPUT_ACTIVE));
    k_msleep(IR_RX_POWER_UP_MS);

    TRY(gpio_pin_configure_dt(&ir_rx, GPIO_INPUT | GPIO_PULL_UP));
    TRY(gpio_pin_interrupt_configure_dt(&ir_rx, GPIO_INT_EDGE_BOTH));

    return 0;
}

static int ir_rx_power_down(void)
{
    int err;

    TRY(gpio_pin_interrupt_configure_dt(&ir_rx, GPIO_INT_DISABLE));
    TRY(gpio_pin_configure_dt(&ir_rx, GPIO_DISCONNECTED));
    TRY(gpio_pin_configure_dt(&ir_power, GPIO_DISCONNECTED));

    return 0;
}

int ir_rx_set_enabled(bool enabled)
{
    k_mutex_lock(&ir_rx_lock, K_FOREVER);

    int err = enabled ? ir_rx_power_up() : ir_rx_power_down();
    if (err < 0) {
        LOG_ERR("Could not turn the IR receiver %s (%d)", enabled ? "on" : "off", err);
        // Leave it off rather than half set up.
        (void)ir_rx_power_down();
        enabled = false;
    }
    ir_rx_enabled = enabled;

    k_mutex_unlock(&ir_rx_lock);

    return err;
}

bool ir_rx_is_enabled(void)
{
    return ir_rx_enabled;
}

// Undoes the driver's setup, which runs at the POST_KERNEL level.
static int ir_rx_enable_init(void)
{
    (void)ir_rx_set_enabled(false);

    return 0;
}

SYS_INIT(ir_rx_enable_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
