/*
 * Copyright (c) 2026 The ZMK Contributors
 * SPDX-License-Identifier: MIT
 *
 * Unified Live Runtime Trackball Processor for Charybdis Mini MOD:
 *   - Base Sensitivity (0.10x .. 5.00x) + Quadratic Dynamic Acceleration (min_factor .. max_factor)
 *   - Hardware Trackball Mute (Layer 10 / g_charybdis_cfg.enabled)
 *   - Auto-Mouse (Layer 6) vs Manual Mode (Layer 9 / Nav + ~)
 *   - Typing Anti-Jitter Guard (vibration_idle_ms) + 5px Movement Activation Threshold + Drag Hold Lock
 *   - 2D Smooth Scroll (Layer 8: adjustable speed + Natural Scroll invert)
 *   - Precision Snipe Mode (Layer 7: 1:2 / 1:3 / 1:4 subpixel scaling)
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/sys/util.h>
#include <drivers/input_processor.h>
#include <zephyr/dt-bindings/input/input-event-codes.h>
#include <stdlib.h>

#include "charybdis_runtime.h"

#if !IS_ENABLED(CONFIG_ZMK_SPLIT) || IS_ENABLED(CONFIG_ZMK_SPLIT_ROLE_CENTRAL)
#include <zmk/keymap.h>
#include <zmk/event_manager.h>
#include <zmk/events/position_state_changed.h>
#include <zmk/events/layer_state_changed.h>
#endif

#define DT_DRV_COMPAT zmk_input_processor_acceleration

#ifndef CONFIG_ZMK_INPUT_PROCESSOR_INIT_PRIORITY
#define CONFIG_ZMK_INPUT_PROCESSOR_INIT_PRIORITY 50
#endif

#define ACCEL_MAX_CODES 2
#define SCALE           1000

#define LAYER_MOUSE        6
#define LAYER_SNIPE        7
#define LAYER_SCROLL       8
#define LAYER_MANUAL_MOUSE 9
#define LAYER_MUTE_TB      10
#define AUTO_MOUSE_TIMEOUT_MS 800

/* Global live runtime configuration (modified on-the-fly via helper_bridge.c + saved in NVS) */
struct charybdis_runtime_cfg g_charybdis_cfg = {
    .version           = CHARYBDIS_RT_CFG_VERSION,
    .enabled           = 1,
    .manual_mode       = 0,
    .scroll_invert     = 0,
    .snipe_ratio       = 3,
    .activation_px     = 5,
    .sens_x100         = 150,  /* 1.50x base sensitivity on 3200 CPI sensor */
    .min_factor        = 450,  /* 0.45x slow precision */
    .max_factor        = 4000, /* 4.00x fast flick */
    .vibration_idle_ms = 350,
    .scroll_speed_x100 = 100,  /* 1.00x smooth 2D scroll */
};

__attribute__((weak)) void charybdis_helper_notify_state_changed(void) {}

static inline uint32_t clamp_u32(uint32_t v, uint32_t lo, uint32_t hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

#if !IS_ENABLED(CONFIG_ZMK_SPLIT) || IS_ENABLED(CONFIG_ZMK_SPLIT_ROLE_CENTRAL)

static bool auto_mouse_active = false;
static bool syncing_layers = false;
static int64_t last_typing_ms = 0;
static int64_t last_accum_ms = 0;
static uint32_t auto_mouse_px_accum = 0;
static uint32_t held_mouse_keys_mask = 0;
static uint64_t phys_held_positions = 0;

/* Positions on Layer 6 that act as Mouse Buttons / Scroll / Snipe / Thumbs */
static int mouse_hold_bit_for_pos(uint32_t pos) {
    switch (pos) {
    case 3:  return 0; /* E = Scroll */
    case 4:  return 1; /* R = Snipe */
    case 8:  return 8; /* I = Scroll */
    case 14: return 2; /* S = RCLK */
    case 15: return 3; /* D = MCLK */
    case 16: return 4; /* F = LCLK */
    case 19: return 5; /* J = LCLK */
    case 20: return 6; /* K = MCLK */
    case 21: return 7; /* L = RCLK */
    default: return -1;
    }
}

static bool is_excluded_when_mouse_active(uint32_t pos) {
    if (mouse_hold_bit_for_pos(pos) >= 0) {
        return true;
    }
    return (pos >= 36 && pos <= 41); /* Thumbs */
}

static void auto_mouse_enable_work_cb(struct k_work *work) {
    ARG_UNUSED(work);
    if (!g_charybdis_cfg.enabled || g_charybdis_cfg.manual_mode ||
        zmk_keymap_layer_active(LAYER_MUTE_TB) || zmk_keymap_layer_active(LAYER_MANUAL_MOUSE)) {
        return;
    }
    if (!auto_mouse_active) {
        auto_mouse_active = true;
        zmk_keymap_layer_activate(LAYER_MOUSE, false);
    }
}

static K_WORK_DEFINE(auto_mouse_enable_work, auto_mouse_enable_work_cb);

static void auto_mouse_disable_work_cb(struct k_work *work);
static K_WORK_DELAYABLE_DEFINE(auto_mouse_disable_work, auto_mouse_disable_work_cb);

static void auto_mouse_disable_work_cb(struct k_work *work) {
    ARG_UNUSED(work);
    if (!auto_mouse_active) {
        return;
    }
    /* Do not drop Mouse Layer mid-drag while LCLK/RCLK/MCLK/Scroll/Snipe is physically held! */
    if (held_mouse_keys_mask != 0 ||
        zmk_keymap_layer_active(LAYER_SCROLL) ||
        zmk_keymap_layer_active(LAYER_SNIPE)) {
        k_work_reschedule(&auto_mouse_disable_work, K_MSEC(400));
        return;
    }
    auto_mouse_active = false;
    if (zmk_keymap_layer_active(LAYER_MOUSE)) {
        zmk_keymap_layer_deactivate(LAYER_MOUSE, false);
    }
}

static void force_deactivate_auto_mouse_work_cb(struct k_work *work) {
    ARG_UNUSED(work);
    k_work_cancel_delayable(&auto_mouse_disable_work);
    held_mouse_keys_mask = 0;
    auto_mouse_px_accum = 0;
    if (auto_mouse_active) {
        auto_mouse_active = false;
        if (zmk_keymap_layer_active(LAYER_MOUSE)) {
            zmk_keymap_layer_deactivate(LAYER_MOUSE, false);
        }
    }
}

static K_WORK_DEFINE(force_deactivate_auto_mouse_work, force_deactivate_auto_mouse_work_cb);

static void panic_layer_cleanup_work_cb(struct k_work *work) {
    ARG_UNUSED(work);
    k_work_cancel_delayable(&auto_mouse_disable_work);
    held_mouse_keys_mask = 0;
    auto_mouse_px_accum = 0;
    auto_mouse_active = false;

    /* Deactivate all transient / toggled layers 1..8 (while preserving 9=Manual and 10=MuteTB).
     * Keep layer 3 (Fn) active only if thumb 36 is still physically held down.
     */
    static const uint8_t transient_layers[] = { 1, 2, 4, 5, 6, 7, 8 };
    for (size_t i = 0; i < ARRAY_SIZE(transient_layers); i++) {
        uint8_t l = transient_layers[i];
        if (zmk_keymap_layer_active(l)) {
            zmk_keymap_layer_deactivate(l, false);
        }
    }
    if ((phys_held_positions & BIT64(36)) == 0 && zmk_keymap_layer_active(3)) {
        zmk_keymap_layer_deactivate(3, false);
    }
}

static K_WORK_DELAYABLE_DEFINE(panic_layer_cleanup_work, panic_layer_cleanup_work_cb);

static void sync_layers_work_cb(struct k_work *work) {
    ARG_UNUSED(work);
    syncing_layers = true;

    bool want_mute = (g_charybdis_cfg.enabled == 0);
    if (want_mute != zmk_keymap_layer_active(LAYER_MUTE_TB)) {
        if (want_mute) {
            zmk_keymap_layer_activate(LAYER_MUTE_TB, false);
        } else {
            zmk_keymap_layer_deactivate(LAYER_MUTE_TB, false);
        }
    }

    bool want_manual = (g_charybdis_cfg.manual_mode != 0);
    if (want_manual != zmk_keymap_layer_active(LAYER_MANUAL_MOUSE)) {
        if (want_manual) {
            zmk_keymap_layer_activate(LAYER_MANUAL_MOUSE, false);
        } else {
            zmk_keymap_layer_deactivate(LAYER_MANUAL_MOUSE, false);
        }
    }

    if ((want_mute || want_manual) && auto_mouse_active) {
        k_work_cancel_delayable(&auto_mouse_disable_work);
        auto_mouse_active = false;
        auto_mouse_px_accum = 0;
        if (zmk_keymap_layer_active(LAYER_MOUSE)) {
            zmk_keymap_layer_deactivate(LAYER_MOUSE, false);
        }
    }

    syncing_layers = false;
}

static K_WORK_DEFINE(sync_layers_work, sync_layers_work_cb);

void charybdis_runtime_on_config_updated(void) {
    k_work_submit(&sync_layers_work);
}

static int charybdis_pointer_event_listener(const zmk_event_t *eh) {
    const struct zmk_position_state_changed *pos_ev = as_zmk_position_state_changed(eh);
    if (pos_ev != NULL) {
        if (pos_ev->position < 64) {
            if (pos_ev->state) {
                phys_held_positions |= BIT64(pos_ev->position);
            } else {
                phys_held_positions &= ~BIT64(pos_ev->position);
            }
        }

        /* Panic Key on Fn layer (layer 3, position 12): reset all transient/toggled layers & mouse state */
        if (pos_ev->state && pos_ev->position == 12 && zmk_keymap_layer_active(3)) {
            k_work_reschedule(&panic_layer_cleanup_work, K_MSEC(25));
        }

        int hold_bit = mouse_hold_bit_for_pos(pos_ev->position);
        if (!pos_ev->state) {
            /* Key released */
            if (hold_bit >= 0) {
                held_mouse_keys_mask &= ~BIT(hold_bit);
                if (auto_mouse_active) {
                    k_work_reschedule(&auto_mouse_disable_work, K_MSEC(AUTO_MOUSE_TIMEOUT_MS));
                }
            }
            return ZMK_EV_EVENT_BUBBLE;
        }

        /* Key pressed */
        if (auto_mouse_active) {
            if (is_excluded_when_mouse_active(pos_ev->position)) {
                if (hold_bit >= 0) {
                    held_mouse_keys_mask |= BIT(hold_bit);
                }
                k_work_reschedule(&auto_mouse_disable_work, K_MSEC(AUTO_MOUSE_TIMEOUT_MS));
            } else {
                /* Typing key pressed -> exit Auto-Mouse immediately so letters type cleanly */
                last_typing_ms = k_uptime_get();
                k_work_submit(&force_deactivate_auto_mouse_work);
            }
        } else {
            /* Normal typing while Auto-Mouse is inactive -> arm anti-jitter guard */
            last_typing_ms = k_uptime_get();
            auto_mouse_px_accum = 0;
        }
        return ZMK_EV_EVENT_BUBBLE;
    }

    const struct zmk_layer_state_changed *lay_ev = as_zmk_layer_state_changed(eh);
    if (lay_ev != NULL) {
        if (lay_ev->layer == LAYER_MOUSE && !lay_ev->state) {
            auto_mouse_active = false;
            held_mouse_keys_mask = 0;
        } else if (!syncing_layers) {
            /* Keep runtime state synced if user presses &tog 9 or &tog 10 on physical keyboard */
            if (lay_ev->layer == LAYER_MANUAL_MOUSE) {
                uint8_t new_man = lay_ev->state ? 1 : 0;
                if (g_charybdis_cfg.manual_mode != new_man) {
                    g_charybdis_cfg.manual_mode = new_man;
                    if (new_man && auto_mouse_active) {
                        k_work_submit(&force_deactivate_auto_mouse_work);
                    }
                    charybdis_helper_notify_state_changed();
                }
            } else if (lay_ev->layer == LAYER_MUTE_TB) {
                uint8_t new_en = lay_ev->state ? 0 : 1;
                if (g_charybdis_cfg.enabled != new_en) {
                    g_charybdis_cfg.enabled = new_en;
                    if (!new_en && auto_mouse_active) {
                        k_work_submit(&force_deactivate_auto_mouse_work);
                    }
                    charybdis_helper_notify_state_changed();
                }
            }
        }
        return ZMK_EV_EVENT_BUBBLE;
    }

    return ZMK_EV_EVENT_BUBBLE;
}

ZMK_LISTENER(charybdis_pointer_rt, charybdis_pointer_event_listener);
ZMK_SUBSCRIPTION(charybdis_pointer_rt, zmk_position_state_changed);
ZMK_SUBSCRIPTION(charybdis_pointer_rt, zmk_layer_state_changed);

#else /* Split Peripheral stub */

void charybdis_runtime_on_config_updated(void) {}

#endif /* Central check */

struct accel_config {
    uint8_t  input_type;
    const uint16_t *codes;
    uint32_t codes_count;
    bool     track_remainders;
    uint16_t min_factor;
    uint16_t max_factor;
    uint32_t speed_threshold;
    uint32_t speed_max;
    uint8_t  acceleration_exponent;
};

struct accel_data {
    int64_t  last_time_ms[ACCEL_MAX_CODES];
    int32_t  last_phys[ACCEL_MAX_CODES];
    uint32_t last_cps[ACCEL_MAX_CODES];
    int16_t  remainders[ACCEL_MAX_CODES];
    int16_t  scroll_rem[ACCEL_MAX_CODES];
    int16_t  snipe_rem[ACCEL_MAX_CODES];
};

static inline bool code_to_index(const struct accel_config *cfg, uint16_t code, uint32_t *out_idx) {
    for (uint32_t i = 0; i < cfg->codes_count; ++i) {
        if (cfg->codes[i] == code) {
            if (out_idx) *out_idx = i;
            return true;
        }
    }
    return false;
}

static uint32_t pow_scaled(uint32_t t, uint8_t exp) {
    t = clamp_u32(t, 0, SCALE);
    if (exp <= 1) return t;
    uint64_t acc = t;
    for (uint8_t i = 1; i < exp; ++i) {
        acc = (acc * t) / SCALE;
    }
    if (acc > UINT32_MAX) acc = UINT32_MAX;
    return (uint32_t)acc;
}

static uint32_t compute_factor_scaled(const struct accel_config *cfg, uint32_t cps,
                                      uint16_t rt_min, uint16_t rt_max) {
    const uint32_t f_min = clamp_u32(rt_min, 100, 20000);
    const uint32_t f_max = clamp_u32(rt_max, f_min, 20000);
    const uint32_t v1 = cfg->speed_threshold;
    const uint32_t v2 = (cfg->speed_max > v1) ? cfg->speed_max : (v1 + 1);
    const uint8_t  e  = cfg->acceleration_exponent ? cfg->acceleration_exponent : 2;

    const uint32_t base = (f_min > 1000) ? f_min : 1000;

    if (cps <= v1) {
        uint32_t t = (v1 == 0) ? SCALE : (uint32_t)((uint64_t)cps * SCALE / v1);
        uint32_t shaped = pow_scaled(t, e);
        int32_t span = (int32_t)base - (int32_t)f_min;
        int32_t f = (int32_t)f_min + (int32_t)((int64_t)span * shaped / SCALE);
        return clamp_u32((uint32_t)f, f_min, base);
    } else if (cps >= v2) {
        return f_max;
    } else {
        uint32_t t = (uint32_t)((uint64_t)(cps - v1) * SCALE / (v2 - v1));
        uint32_t shaped = pow_scaled(t, e);
        int32_t span = (int32_t)f_max - (int32_t)base;
        int32_t f = (int32_t)base + (int32_t)((int64_t)span * shaped / SCALE);
        return clamp_u32((uint32_t)f, base, f_max);
    }
}

static int accel_handle_event(const struct device *dev, struct input_event *event,
                              uint32_t param1, uint32_t param2,
                              struct zmk_input_processor_state *state) {
    ARG_UNUSED(param1);
    ARG_UNUSED(param2);
    ARG_UNUSED(state);

    const struct accel_config *cfg = dev->config;
    struct accel_data *data = dev->data;

    if (event->type != cfg->input_type) {
        return ZMK_INPUT_PROC_CONTINUE;
    }

    uint32_t idx = 0;
    if (!code_to_index(cfg, event->code, &idx)) {
        return ZMK_INPUT_PROC_CONTINUE;
    }

#if !IS_ENABLED(CONFIG_ZMK_SPLIT) || IS_ENABLED(CONFIG_ZMK_SPLIT_ROLE_CENTRAL)
    /* 1. Hardware Trackball Mute */
    if (!g_charybdis_cfg.enabled || zmk_keymap_layer_active(LAYER_MUTE_TB)) {
        data->remainders[0] = data->remainders[1] = 0;
        data->scroll_rem[0] = data->scroll_rem[1] = 0;
        data->snipe_rem[0] = data->snipe_rem[1] = 0;
        event->value = 0;
        return ZMK_INPUT_PROC_STOP;
    }
#endif

    const int32_t raw = event->value;
    const int64_t now = k_uptime_get();

    if (raw == 0) {
        data->last_time_ms[idx] = now;
        return ZMK_INPUT_PROC_CONTINUE;
    }

#if !IS_ENABLED(CONFIG_ZMK_SPLIT) || IS_ENABLED(CONFIG_ZMK_SPLIT_ROLE_CENTRAL)
    /* 2. 2D Smooth Scroll Mode (Layer 8 — hold E) */
    if (zmk_keymap_layer_active(LAYER_SCROLL)) {
        uint32_t spd = clamp_u32(g_charybdis_cfg.scroll_speed_x100, 25, 400);
        int64_t total = (int64_t)raw * (int64_t)spd + (int64_t)data->scroll_rem[idx];
        int32_t out = (int32_t)(total / 1600);
        data->scroll_rem[idx] = (int16_t)(total - (int64_t)out * 1600);

        if (event->code == INPUT_REL_Y) {
            event->code = INPUT_REL_WHEEL;
            event->value = g_charybdis_cfg.scroll_invert ? out : -out;
        } else {
            event->code = INPUT_REL_HWHEEL;
            event->value = g_charybdis_cfg.scroll_invert ? -out : out;
        }

        if (auto_mouse_active) {
            k_work_reschedule(&auto_mouse_disable_work, K_MSEC(AUTO_MOUSE_TIMEOUT_MS));
        }
        data->last_time_ms[idx] = now;
        return ZMK_INPUT_PROC_CONTINUE;
    }

    /* 3. Precision Snipe Mode (Layer 7 — hold R) */
    if (zmk_keymap_layer_active(LAYER_SNIPE)) {
        int32_t div = (int32_t)clamp_u32(g_charybdis_cfg.snipe_ratio, 2, 6) * 2;
        int32_t total = raw + (int32_t)data->snipe_rem[idx];
        int32_t out = total / div;
        data->snipe_rem[idx] = (int16_t)(total - out * div);
        event->value = out;

        if (auto_mouse_active) {
            k_work_reschedule(&auto_mouse_disable_work, K_MSEC(AUTO_MOUSE_TIMEOUT_MS));
        }
        data->last_time_ms[idx] = now;
        return ZMK_INPUT_PROC_CONTINUE;
    }
#endif

    /* 4. Base Sensitivity + Dynamic V^2 Acceleration */
    uint32_t dt_ms = 12;
    if (data->last_time_ms[idx] > 0 && now > data->last_time_ms[idx]) {
        int64_t diff = now - data->last_time_ms[idx];
        if (diff > 100) diff = 100;
        dt_ms = (uint32_t)diff;
    }

    uint32_t axis_cps = (uint32_t)(((uint64_t)abs(raw) * 1000ULL) / dt_ms);
    data->last_cps[idx] = axis_cps;

    uint32_t other_idx = (idx == 0) ? 1 : 0;
    uint32_t eff_cps = axis_cps;
    if (data->last_time_ms[other_idx] > 0 && (now - data->last_time_ms[other_idx]) <= 18) {
        if (data->last_cps[other_idx] > eff_cps) {
            eff_cps = (eff_cps + data->last_cps[other_idx]) / 2;
        }
    }

    uint32_t accel_f = compute_factor_scaled(cfg, eff_cps,
                                             g_charybdis_cfg.min_factor,
                                             g_charybdis_cfg.max_factor);

    if ((int64_t)data->last_phys[idx] * (int64_t)raw < 0 && accel_f > 1000) {
        accel_f = 1000;
    }

    uint32_t sens = clamp_u32(g_charybdis_cfg.sens_x100, 10, 500);
    uint64_t combined_f = ((uint64_t)accel_f * (uint64_t)sens) / 100ULL;

    int32_t out_val;
    if (cfg->track_remainders) {
        int64_t total = (int64_t)raw * (int64_t)combined_f + (int64_t)data->remainders[idx];
        out_val = (int32_t)(total / SCALE);
        int32_t rem = (int32_t)(total - (int64_t)out_val * SCALE);
        if (out_val > 32767) out_val = 32767;
        if (out_val < -32768) out_val = -32768;
        event->value = out_val;
        data->remainders[idx] = (int16_t)rem;
    } else {
        out_val = (int32_t)(((int64_t)raw * (int64_t)combined_f) / SCALE);
        event->value = out_val;
    }

    data->last_phys[idx] = raw;
    data->last_time_ms[idx] = now;

#if !IS_ENABLED(CONFIG_ZMK_SPLIT) || IS_ENABLED(CONFIG_ZMK_SPLIT_ROLE_CENTRAL)
    /* 5. Auto-Mouse Layer 6 Management (skipped when Manual Mode is active) */
    if (!g_charybdis_cfg.manual_mode && !zmk_keymap_layer_active(LAYER_MANUAL_MOUSE)) {
        if (auto_mouse_active) {
            k_work_reschedule(&auto_mouse_disable_work, K_MSEC(AUTO_MOUSE_TIMEOUT_MS));
        } else if (out_val != 0) {
            bool typing_guard = (last_typing_ms > 0) &&
                                ((now - last_typing_ms) < (int64_t)g_charybdis_cfg.vibration_idle_ms);
            if (!typing_guard) {
                if ((now - last_accum_ms) > 350) {
                    auto_mouse_px_accum = 0;
                }
                last_accum_ms = now;
                auto_mouse_px_accum += (uint32_t)abs(out_val);

                uint32_t req_px = clamp_u32(g_charybdis_cfg.activation_px, 1, 40);
                if (auto_mouse_px_accum >= req_px) {
                    auto_mouse_px_accum = 0;
                    k_work_submit(&auto_mouse_enable_work);
                    k_work_reschedule(&auto_mouse_disable_work, K_MSEC(AUTO_MOUSE_TIMEOUT_MS));
                }
            }
        }
    }
#endif

    return ZMK_INPUT_PROC_CONTINUE;
}

static const struct zmk_input_processor_driver_api accel_api = {
    .handle_event = accel_handle_event,
};

#define ACCEL_INST_INIT(inst)                                                      \
    static const uint16_t accel_codes_##inst[] = { INPUT_REL_X, INPUT_REL_Y };     \
    static const struct accel_config accel_config_##inst = {                       \
        .input_type = DT_INST_PROP_OR(inst, input_type, INPUT_EV_REL),             \
        .codes = accel_codes_##inst,                                               \
        .codes_count = 2,                                                          \
        .track_remainders = DT_INST_NODE_HAS_PROP(inst, track_remainders),         \
        .min_factor = DT_INST_PROP_OR(inst, min_factor, 450),                      \
        .max_factor = DT_INST_PROP_OR(inst, max_factor, 4000),                     \
        .speed_threshold = DT_INST_PROP_OR(inst, speed_threshold, 600),            \
        .speed_max = DT_INST_PROP_OR(inst, speed_max, 3200),                       \
        .acceleration_exponent = DT_INST_PROP_OR(inst, acceleration_exponent, 2),  \
    };                                                                             \
    static struct accel_data accel_data_##inst = {0};                              \
    DEVICE_DT_INST_DEFINE(inst,                                                    \
                          NULL,                                                    \
                          NULL,                                                    \
                          &accel_data_##inst,                                      \
                          &accel_config_##inst,                                    \
                          POST_KERNEL,                                             \
                          CONFIG_ZMK_INPUT_PROCESSOR_INIT_PRIORITY,                \
                          &accel_api);

DT_INST_FOREACH_STATUS_OKAY(ACCEL_INST_INIT)
