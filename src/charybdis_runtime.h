/*
 * Copyright (c) 2026 The ZMK Contributors
 * SPDX-License-Identifier: MIT
 *
 * Shared live runtime configuration between src/input_processor_accel.c
 * and src/helper_bridge.c (persisted in NVS flash under "charybdis/rt").
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>

#define CHARYBDIS_RT_CFG_VERSION 1

struct charybdis_runtime_cfg {
    uint8_t  version;           /* CHARYBDIS_RT_CFG_VERSION */
    uint8_t  enabled;           /* 1 = Trackball active, 0 = Hardware Mute */
    uint8_t  manual_mode;       /* 0 = Auto-Mouse (Layer 6), 1 = Manual (Nav + ~) */
    uint8_t  scroll_invert;     /* 0 = Standard wheel, 1 = Natural Scroll */
    uint8_t  snipe_ratio;       /* 2, 3, or 4 (1:2, 1:3, 1:4) */
    uint8_t  activation_px;     /* Min cursor delta (px) before Auto-Mouse activates (default 5) */
    uint16_t sens_x100;         /* Base sensitivity x100: 10..500 (default 150 = 1.50x @ 3200 CPI) */
    uint16_t min_factor;        /* Min accel factor x1000: 100..2000 (default 450 = 0.45x) */
    uint16_t max_factor;        /* Max accel factor x1000: 1000..10000 (default 4000 = 4.00x) */
    uint16_t vibration_idle_ms; /* Anti-jitter typing guard ms: 0..1500 (default 350) */
    uint16_t scroll_speed_x100; /* 2D Scroll speed x100: 25..400 (default 100 = 1.00x) */
};

extern struct charybdis_runtime_cfg g_charybdis_cfg;

void charybdis_runtime_on_config_updated(void);
void charybdis_helper_notify_state_changed(void);
