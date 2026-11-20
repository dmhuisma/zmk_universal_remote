#pragma once

#include <stdbool.h>

// src/motion_pointer.c moves the host's pointer as the remote turns, while the
// Touchpad screen is shown and its Motion toggle is on. src/touchpad.c drives
// it with these; both are safe to call from any thread.

// The Touchpad screen was shown, or left.
void motion_pointer_set_shown(bool shown);

// A finger is on the pad, or has just lifted. The pointer ignores the remote
// turning until shortly after, so tapping the pad does not jolt it.
void motion_pointer_set_touching(bool touching);
