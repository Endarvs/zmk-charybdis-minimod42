/*
 * Copyright (c) 2026 The ZMK Contributors
 * SPDX-License-Identifier: MIT
 *
 * Dedicated Second USB CDC-ACM Bridge (helper_cdc_acm_uart -> MI_02) for Charybdis Helper:
 *   - 100% isolated from ZMK Studio's RPC port (snippet_studio_rpc_usb_uart -> MI_00 / COM8)
 *   - Live runtime updates of Trackball Speed, Dynamic Acceleration, Mute, Auto-Mouse/Manual Mode,
 *     Anti-Jitter, 2D Scroll, and Snipe Ratio
 *   - Non-volatile Flash persistence via Zephyr Settings ("charybdis/rt")
 *   - Safe post-connect BLE GATT battery polling (BT_UUID_BAS_BATTERY_LEVEL) for Left & Right halves
 *     without touching ZMK's internal sub_discover_params
 *   - Automatic discovery of active BLE central connections via bt_conn_foreach
 *   - Deterministic Left vs Right peripheral resolution via position state events
 */

#include <zephyr/devicetree.h>

#if DT_NODE_HAS_STATUS(DT_NODELABEL(helper_cdc_acm_uart), okay)

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/init.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/sys/ring_buffer.h>
#include <zephyr/sys/printk.h>
#include <zephyr/settings/settings.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/uuid.h>
#include <string.h>

#include <zmk/battery.h>
#include <zmk/event_manager.h>
#include <zmk/events/position_state_changed.h>
#include "charybdis_runtime.h"

#define HELPER_UART_NODE DT_NODELABEL(helper_cdc_acm_uart)
static const struct device *const helper_uart = DEVICE_DT_GET(HELPER_UART_NODE);

RING_BUF_DECLARE(helper_tx_ring, 512);
RING_BUF_DECLARE(helper_rx_ring, 256);

static bool host_listening = false;
static char cmd_line_buf[192];
static size_t cmd_line_len = 0;

/* ── Safe Post-Connect BLE Peripheral Battery Reader ── */
#define MAX_BATT_SLOTS 2

struct half_batt_slot {
    struct bt_conn *conn;
    int64_t connected_ms;
    int64_t read_start_ms;
    int8_t  side;       /* -1 = unknown, 0 = Left, 1 = Right */
    int16_t level;      /* -1 = unknown, 0..100 = BAS % */
    bool    read_busy;
    struct bt_gatt_read_params bas_params;
};

static struct half_batt_slot batt_slots[MAX_BATT_SLOTS];
static struct k_work_delayable batt_poll_work;
static struct k_work send_state_work;
static struct k_work rx_process_work;

/* File-scope static storage for GATT UUID (must outlive async bt_gatt_read) */
static const struct bt_uuid_16 uuid_bas_lvl = BT_UUID_INIT_16(BT_UUID_BAS_BATTERY_LEVEL_VAL);

static void enqueue_tx_str(const char *s) {
    if (!device_is_ready(helper_uart) || s == NULL) {
        return;
    }
    size_t len = strlen(s);
    if (len == 0) {
        return;
    }
    uint32_t put = ring_buf_put(&helper_tx_ring, (const uint8_t *)s, (uint32_t)len);
    if (put > 0) {
        uart_irq_tx_enable(helper_uart);
    }
}

void charybdis_helper_notify_state_changed(void) {
    if (host_listening) {
        k_work_submit(&send_state_work);
    }
}

static void add_active_conn(struct bt_conn *conn, void *data) {
    ARG_UNUSED(data);
    struct bt_conn_info info;
    if (bt_conn_get_info(conn, &info) != 0) {
        return;
    }
    if (info.state != BT_CONN_STATE_CONNECTED || info.role != BT_CONN_ROLE_CENTRAL) {
        return;
    }

    for (int i = 0; i < MAX_BATT_SLOTS; i++) {
        if (batt_slots[i].conn == conn) {
            return; /* already tracked */
        }
    }

    for (int i = 0; i < MAX_BATT_SLOTS; i++) {
        if (batt_slots[i].conn == NULL) {
            batt_slots[i].conn = bt_conn_ref(conn);
            batt_slots[i].connected_ms = k_uptime_get();
            batt_slots[i].read_start_ms = 0;
            batt_slots[i].level = -1;
            batt_slots[i].read_busy = false;
#if defined(CONFIG_ZMK_SPLIT_BLE_CENTRAL_PERIPHERALS) && (CONFIG_ZMK_SPLIT_BLE_CENTRAL_PERIPHERALS == 1)
            batt_slots[i].side = 0; /* Mode A: Only 1 peripheral = Left half */
#else
            batt_slots[i].side = -1;
#endif
            charybdis_helper_notify_state_changed();
            return;
        }
    }
}

static void sync_connections(void) {
    /* 1. Purge disconnected or dead connections */
    for (int i = 0; i < MAX_BATT_SLOTS; i++) {
        if (batt_slots[i].conn != NULL) {
            struct bt_conn_info info;
            if (bt_conn_get_info(batt_slots[i].conn, &info) != 0 ||
                info.state != BT_CONN_STATE_CONNECTED) {
                bt_conn_unref(batt_slots[i].conn);
                batt_slots[i].conn = NULL;
                batt_slots[i].side = -1;
                batt_slots[i].level = -1;
                batt_slots[i].read_busy = false;
                batt_slots[i].read_start_ms = 0;
            }
        }
    }

    /* 2. Enumerate active LE central connections (catches boot & reconnection) */
    bt_conn_foreach(BT_CONN_TYPE_LE, add_active_conn, NULL);
}

static void get_split_battery_state(int *l_conn, int *l_batt, int *r_conn, int *r_batt) {
    *l_conn = 0;
    *l_batt = -1;
    *r_conn = 0;
    *r_batt = -1;

#if defined(CONFIG_ZMK_SPLIT_BLE_CENTRAL_PERIPHERALS) && (CONFIG_ZMK_SPLIT_BLE_CENTRAL_PERIPHERALS == 1)
    /* Mode A: charybdis_right is Central (local ADC = Right half, 1 BLE peripheral = Left half) */
    *r_conn = 1;
    *r_batt = (int)zmk_battery_state_of_charge();

    for (int i = 0; i < MAX_BATT_SLOTS; i++) {
        if (batt_slots[i].conn != NULL) {
            *l_conn = 1;
            *l_batt = batt_slots[i].level;
            break;
        }
    }
    return;
#endif

    /* Mode B: Central Dongle (2 BLE peripherals: Left + Right) */
    for (int i = 0; i < MAX_BATT_SLOTS; i++) {
        if (batt_slots[i].conn == NULL) {
            continue;
        }
        if (batt_slots[i].side == 0) {
            *l_conn = 1;
            *l_batt = batt_slots[i].level;
        } else if (batt_slots[i].side == 1) {
            *r_conn = 1;
            *r_batt = batt_slots[i].level;
        }
    }

    /* Fallback if side is not yet determined by key events:
       If both are connected, map slot 0 -> Left, slot 1 -> Right.
       If only one is connected, map to Left if r_conn==0 else Right.
     */
    if (!*l_conn && !*r_conn) {
        if (batt_slots[0].conn != NULL) {
            *l_conn = 1;
            *l_batt = batt_slots[0].level;
        }
        if (batt_slots[1].conn != NULL) {
            *r_conn = 1;
            *r_batt = batt_slots[1].level;
        }
    } else if (*r_conn && !*l_conn) {
        for (int i = 0; i < MAX_BATT_SLOTS; i++) {
            if (batt_slots[i].conn != NULL && batt_slots[i].side != 1) {
                *l_conn = 1;
                *l_batt = batt_slots[i].level;
                break;
            }
        }
    } else if (*l_conn && !*r_conn) {
        for (int i = 0; i < MAX_BATT_SLOTS; i++) {
            if (batt_slots[i].conn != NULL && batt_slots[i].side != 0) {
                *r_conn = 1;
                *r_batt = batt_slots[i].level;
                break;
            }
        }
    }
}

static void send_state_work_handler(struct k_work *work) {
    ARG_UNUSED(work);

    int l_conn = 0, l_batt = -1, r_conn = 0, r_batt = -1;
    get_split_battery_state(&l_conn, &l_batt, &r_conn, &r_batt);

    char msg[256];
    int n = snprintk(
        msg, sizeof(msg),
        "{\"type\":\"state\",\"fw\":\"2.0-live\","
        "\"left_conn\":%d,\"left_batt\":%d,\"right_conn\":%d,\"right_batt\":%d,"
        "\"enabled\":%u,\"manual\":%u,\"sens\":%u,\"min_f\":%u,\"max_f\":%u,"
        "\"vib_ms\":%u,\"scroll\":%u,\"scroll_inv\":%u,\"snipe\":%u,\"act_px\":%u}\n",
        l_conn, l_batt, r_conn, r_batt,
        (unsigned)g_charybdis_cfg.enabled,
        (unsigned)g_charybdis_cfg.manual_mode,
        (unsigned)g_charybdis_cfg.sens_x100,
        (unsigned)g_charybdis_cfg.min_factor,
        (unsigned)g_charybdis_cfg.max_factor,
        (unsigned)g_charybdis_cfg.vibration_idle_ms,
        (unsigned)g_charybdis_cfg.scroll_speed_x100,
        (unsigned)g_charybdis_cfg.scroll_invert,
        (unsigned)g_charybdis_cfg.snipe_ratio,
        (unsigned)g_charybdis_cfg.activation_px
    );
    if (n > 0 && n < (int)sizeof(msg)) {
        enqueue_tx_str(msg);
    }
}

/* ── BLE GATT Callback (BAS Battery Level 0x2A19) ── */

static struct half_batt_slot *slot_for_bas_params(struct bt_gatt_read_params *p) {
    for (int i = 0; i < MAX_BATT_SLOTS; i++) {
        if (&batt_slots[i].bas_params == p) {
            return &batt_slots[i];
        }
    }
    return NULL;
}

static uint8_t bas_read_cb(struct bt_conn *conn, uint8_t err,
                           struct bt_gatt_read_params *params,
                           const void *data, uint16_t length) {
    ARG_UNUSED(conn);
    struct half_batt_slot *slot = slot_for_bas_params(params);
    if (slot == NULL || slot->conn == NULL) {
        if (slot) slot->read_busy = false;
        return BT_GATT_ITER_STOP;
    }

    if (err == 0 && data != NULL && length >= 1) {
        uint8_t lvl = ((const uint8_t *)data)[0];
        if (lvl <= 100) {
            slot->level = (int16_t)lvl;
        }
        slot->read_busy = false;
        charybdis_helper_notify_state_changed();
        return BT_GATT_ITER_STOP;
    }

    slot->read_busy = false;
    return BT_GATT_ITER_STOP;
}

static void batt_poll_work_handler(struct k_work *work) {
    ARG_UNUSED(work);
    const int64_t now = k_uptime_get();
    bool need_soon = false;

    sync_connections();

    for (int i = 0; i < MAX_BATT_SLOTS; i++) {
        struct half_batt_slot *slot = &batt_slots[i];
        if (slot->conn == NULL) {
            continue;
        }

        /* Timeout guard: clear read_busy if stuck > 5 seconds */
        if (slot->read_busy) {
            if ((now - slot->read_start_ms) > 5000) {
                slot->read_busy = false;
            } else {
                continue;
            }
        }

        /* Give 3 seconds after connection before issuing GATT read */
        if ((now - slot->connected_ms) < 3000) {
            need_soon = true;
            continue;
        }

        slot->read_busy = true;
        slot->read_start_ms = now;
        memset(&slot->bas_params, 0, sizeof(slot->bas_params));
        slot->bas_params.func = bas_read_cb;
        slot->bas_params.handle_count = 0;
        slot->bas_params.by_uuid.start_handle = 0x0001;
        slot->bas_params.by_uuid.end_handle = 0xffff;
        slot->bas_params.by_uuid.uuid = &uuid_bas_lvl.uuid;

        if (bt_gatt_read(slot->conn, &slot->bas_params) != 0) {
            slot->read_busy = false;
        }
    }

    k_work_reschedule(&batt_poll_work, need_soon ? K_MSEC(2000) : K_SECONDS(30));
}

static void helper_bt_connected(struct bt_conn *conn, uint8_t conn_err) {
    if (conn_err) {
        return;
    }
    add_active_conn(conn, NULL);
    k_work_reschedule(&batt_poll_work, K_MSEC(3200));
}

static void helper_bt_disconnected(struct bt_conn *conn, uint8_t reason) {
    ARG_UNUSED(reason);
    for (int i = 0; i < MAX_BATT_SLOTS; i++) {
        if (batt_slots[i].conn == conn) {
            bt_conn_unref(batt_slots[i].conn);
            batt_slots[i].conn = NULL;
            batt_slots[i].side = -1;
            batt_slots[i].level = -1;
            batt_slots[i].read_busy = false;
            batt_slots[i].read_start_ms = 0;
            charybdis_helper_notify_state_changed();
            return;
        }
    }
}

static struct bt_conn_cb helper_conn_callbacks = {
    .connected = helper_bt_connected,
    .disconnected = helper_bt_disconnected,
};

/* ── Key Position Listener (Mode B: Dongle Left vs Right Identification) ── */

static inline bool is_left_key_position(uint32_t pos) {
    if (pos < 6) return true;
    if (pos >= 12 && pos < 18) return true;
    if (pos >= 24 && pos < 30) return true;
    if (pos >= 36 && pos < 39) return true;
    return false;
}

static int helper_pos_listener(const zmk_event_t *eh) {
    const struct zmk_position_state_changed *ev = as_zmk_position_state_changed(eh);
    if (ev == NULL || !ev->state) {
        return ZMK_EV_EVENT_BUBBLE;
    }

#if !defined(CONFIG_ZMK_SPLIT_BLE_CENTRAL_PERIPHERALS) || (CONFIG_ZMK_SPLIT_BLE_CENTRAL_PERIPHERALS != 1)
    uint8_t src = ev->source;
    if (src < MAX_BATT_SLOTS && batt_slots[src].conn != NULL) {
        bool is_left = is_left_key_position(ev->position);
        int8_t new_side = is_left ? 0 : 1;
        if (batt_slots[src].side != new_side) {
            batt_slots[src].side = new_side;
            int other = 1 - src;
            if (other >= 0 && other < MAX_BATT_SLOTS && batt_slots[other].conn != NULL) {
                batt_slots[other].side = is_left ? 1 : 0;
            }
            charybdis_helper_notify_state_changed();
        }
    }
#endif

    return ZMK_EV_EVENT_BUBBLE;
}

ZMK_LISTENER(helper_pos_tracker, helper_pos_listener);
ZMK_SUBSCRIPTION(helper_pos_tracker, zmk_position_state_changed);

/* ── NVS Flash Persistence ("charybdis/rt") ── */
#if IS_ENABLED(CONFIG_SETTINGS)
static int rt_settings_set(const char *name, size_t len, settings_read_cb read_cb, void *cb_arg) {
    if (strcmp(name, "rt") == 0) {
        struct charybdis_runtime_cfg loaded;
        if (len != sizeof(loaded)) {
            return -EINVAL;
        }
        int rc = read_cb(cb_arg, &loaded, sizeof(loaded));
        if (rc >= 0 && loaded.version == CHARYBDIS_RT_CFG_VERSION) {
            g_charybdis_cfg = loaded;
            return 0;
        }
    }
    return -ENOENT;
}

static int rt_settings_commit(void) {
    charybdis_runtime_on_config_updated();
    return 0;
}

SETTINGS_STATIC_HANDLER_DEFINE(charybdis_rt, "charybdis", NULL, rt_settings_set, rt_settings_commit, NULL);
#endif

/* ── Lightweight Command Parser (Zero scanf/libc bloat) ── */

static int parse_u32_tokens(const char *s, uint32_t *out, int max_tokens) {
    int count = 0;
    while (*s != '\0' && count < max_tokens) {
        while (*s == ' ' || *s == '\t') {
            s++;
        }
        if (*s < '0' || *s > '9') {
            break;
        }
        uint32_t val = 0;
        while (*s >= '0' && *s <= '9') {
            val = val * 10U + (uint32_t)(*s - '0');
            s++;
        }
        out[count++] = val;
    }
    return count;
}

static inline uint16_t clamp_u16(uint32_t v, uint16_t lo, uint16_t hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return (uint16_t)v;
}

static void handle_command_line(const char *line) {
    host_listening = true;

    if (strncmp(line, "GET", 3) == 0 || strncmp(line, "PING", 4) == 0) {
        k_work_submit(&send_state_work);
        return;
    }

    if (strncmp(line, "REFRESH", 7) == 0) {
        sync_connections();
        for (int i = 0; i < MAX_BATT_SLOTS; i++) {
            batt_slots[i].read_busy = false;
        }
        k_work_reschedule(&batt_poll_work, K_NO_WAIT);
        k_work_submit(&send_state_work);
        return;
    }

    if (strncmp(line, "SET ", 4) == 0) {
        /* SET <enabled> <manual> <sens_x100> <min_f> <max_f> <vib_ms> <scroll_x100> <scroll_inv> <snipe> <act_px> */
        uint32_t args[10] = {0};
        int n = parse_u32_tokens(line + 4, args, 10);
        if (n >= 9) {
            g_charybdis_cfg.version           = CHARYBDIS_RT_CFG_VERSION;
            g_charybdis_cfg.enabled           = args[0] ? 1 : 0;
            g_charybdis_cfg.manual_mode       = args[1] ? 1 : 0;
            g_charybdis_cfg.sens_x100         = clamp_u16(args[2], 10, 500);
            g_charybdis_cfg.min_factor        = clamp_u16(args[3], 100, 2000);
            g_charybdis_cfg.max_factor        = clamp_u16(args[4], g_charybdis_cfg.min_factor, 10000);
            g_charybdis_cfg.vibration_idle_ms = clamp_u16(args[5], 0, 1500);
            g_charybdis_cfg.scroll_speed_x100 = clamp_u16(args[6], 25, 400);
            g_charybdis_cfg.scroll_invert     = args[7] ? 1 : 0;
            g_charybdis_cfg.snipe_ratio       = (uint8_t)clamp_u16(args[8], 2, 6);
            if (n >= 10) {
                g_charybdis_cfg.activation_px = (uint8_t)clamp_u16(args[9], 1, 40);
            }

            charybdis_runtime_on_config_updated();
            k_work_submit(&send_state_work);
        }
        return;
    }

    if (strncmp(line, "SAVE", 4) == 0) {
        int rc = 0;
#if IS_ENABLED(CONFIG_SETTINGS)
        rc = settings_save_one("charybdis/rt", &g_charybdis_cfg, sizeof(g_charybdis_cfg));
#endif
        if (rc == 0) {
            enqueue_tx_str("{\"type\":\"ack\",\"cmd\":\"save\",\"ok\":1}\n");
        } else {
            enqueue_tx_str("{\"type\":\"ack\",\"cmd\":\"save\",\"ok\":0}\n");
        }
        k_work_submit(&send_state_work);
        return;
    }
}

static void rx_process_work_handler(struct k_work *work) {
    ARG_UNUSED(work);
    uint8_t ch;
    while (ring_buf_get(&helper_rx_ring, &ch, 1) == 1) {
        if (ch == '\r') {
            continue;
        }
        if (ch == '\n') {
            cmd_line_buf[cmd_line_len] = '\0';
            if (cmd_line_len > 0) {
                handle_command_line(cmd_line_buf);
            }
            cmd_line_len = 0;
        } else if (cmd_line_len < (sizeof(cmd_line_buf) - 1)) {
            cmd_line_buf[cmd_line_len++] = (char)ch;
        } else {
            cmd_line_len = 0; /* Overflow guard */
        }
    }
}

static void helper_uart_isr(const struct device *dev, void *user_data) {
    ARG_UNUSED(user_data);

    if (!uart_irq_update(dev)) {
        return;
    }

    if (uart_irq_rx_ready(dev)) {
        uint8_t tmp[64];
        int rd;
        bool got_any = false;
        while ((rd = uart_fifo_read(dev, tmp, sizeof(tmp))) > 0) {
            ring_buf_put(&helper_rx_ring, tmp, (uint32_t)rd);
            got_any = true;
        }
        if (got_any) {
            k_work_submit(&rx_process_work);
        }
    }

    if (uart_irq_tx_ready(dev)) {
        uint8_t *buf;
        uint32_t claim = ring_buf_get_claim(&helper_tx_ring, &buf, 64);
        if (claim == 0) {
            uart_irq_tx_disable(dev);
        } else {
            int sent = uart_fifo_fill(dev, buf, claim);
            if (sent <= 0) {
                /* Host closed port / USB endpoint full: stop TX IRQ cleanly without spinning */
                ring_buf_get_finish(&helper_tx_ring, 0);
                ring_buf_reset(&helper_tx_ring);
                host_listening = false;
                uart_irq_tx_disable(dev);
            } else {
                ring_buf_get_finish(&helper_tx_ring, (uint32_t)sent);
                if (ring_buf_is_empty(&helper_tx_ring)) {
                    uart_irq_tx_disable(dev);
                }
            }
        }
    }
}

static int helper_bridge_init(void) {
    for (int i = 0; i < MAX_BATT_SLOTS; i++) {
        batt_slots[i].conn = NULL;
        batt_slots[i].side = -1;
        batt_slots[i].level = -1;
        batt_slots[i].read_busy = false;
        batt_slots[i].read_start_ms = 0;
    }

    k_work_init(&send_state_work, send_state_work_handler);
    k_work_init(&rx_process_work, rx_process_work_handler);
    k_work_init_delayable(&batt_poll_work, batt_poll_work_handler);

    bt_conn_cb_register(&helper_conn_callbacks);

    sync_connections();
    k_work_reschedule(&batt_poll_work, K_MSEC(3500));

    if (!device_is_ready(helper_uart)) {
        return 0;
    }

    if (uart_irq_callback_user_data_set(helper_uart, helper_uart_isr, NULL) == 0) {
        uart_irq_rx_enable(helper_uart);
    }

    return 0;
}

SYS_INIT(helper_bridge_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);

#endif /* DT_NODE_HAS_STATUS(DT_NODELABEL(helper_cdc_acm_uart), okay) */
