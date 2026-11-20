#pragma once

#include <stdbool.h>
#include <stdint.h>

// Sends a press of button number 1-255 to Home Assistant, as a touchscreen
// button press is sent: a tap rotates the dimmer right, a long press left.
void bthome_send_button(uint8_t button, bool long_press);
