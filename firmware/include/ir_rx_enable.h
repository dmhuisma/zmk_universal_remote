#pragma once

#include <stdbool.h>

// Powers the IR receiver and lets zmk-ir's driver capture from it, or powers it
// down and releases its pin. Off at boot and through sleep. Turning it on
// blocks for the receiver's power-up time.
int ir_rx_set_enabled(bool enabled);

bool ir_rx_is_enabled(void);
