/*
 * Copyright (c) 2026 The ZMK Contributors
 * SPDX-License-Identifier: MIT
 *
 * Неблокирующая индикация включения питания и BLE-связи штатным синим светодиодом
 * (P0.15 blue_led на nice!nano v2 / led2 на Seeed XIAO BLE):
 *   - Включение тумблера питания (Boot): 2 чётких коротких мигания (120 мс) -> ВЫКЛ.
 *   - Подключение половинки к донглу / правой части: 1 плавная вспышка (550 мс) -> ВЫКЛ.
 *   - Потеря связи с центральной платой: 3 быстрых коротких импульса (70 мс) -> ВЫКЛ.
 *   - Всё остальное время светодиод полностью выключен (нулевой расход аккумулятора).
 */

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>

#if !IS_ENABLED(CONFIG_ZMK_SPLIT_ROLE_CENTRAL)
#include <zmk/event_manager.h>
#include <zmk/events/split_peripheral_status_changed.h>
#endif

#if DT_NODE_HAS_PROP(DT_NODELABEL(blue_led), gpios)
#define STATUS_LED_NODE DT_NODELABEL(blue_led)
#elif DT_NODE_HAS_PROP(DT_ALIAS(led2), gpios)
#define STATUS_LED_NODE DT_ALIAS(led2)
#elif DT_NODE_HAS_PROP(DT_ALIAS(led0), gpios)
#define STATUS_LED_NODE DT_ALIAS(led0)
#endif

#ifdef STATUS_LED_NODE

static const struct gpio_dt_spec status_led = GPIO_DT_SPEC_GET(STATUS_LED_NODE, gpios);

struct blink_step {
    bool on;
    uint16_t duration_ms;
};

/* 2 коротких мигания при подаче питания */
static const struct blink_step pattern_boot[] = {
    {true, 120},
    {false, 100},
    {true, 120},
    {false, 0},
};

#if !IS_ENABLED(CONFIG_ZMK_SPLIT_ROLE_CENTRAL)
/* Пауза + 1 уверенная вспышка (550 мс) при успешном BLE-соединении с Central / донглом */
static const struct blink_step pattern_connected[] = {
    {false, 220},
    {true, 550},
    {false, 0},
};

/* 3 быстрых коротких мигания при разрыве связи */
static const struct blink_step pattern_disconnected[] = {
    {true, 70},
    {false, 70},
    {true, 70},
    {false, 70},
    {true, 70},
    {false, 0},
};
#endif

static const struct blink_step *current_pattern = NULL;
static const struct blink_step *pending_pattern = NULL;
static size_t current_idx = 0;
static struct k_work_delayable blink_work;
static bool led_ready = false;

static void blink_work_handler(struct k_work *work) {
    ARG_UNUSED(work);

    if (!led_ready || current_pattern == NULL) {
        return;
    }

    const struct blink_step step = current_pattern[current_idx];
    gpio_pin_set_dt(&status_led, step.on ? 1 : 0);

    if (step.duration_ms == 0) {
        if (pending_pattern != NULL) {
            current_pattern = pending_pattern;
            pending_pattern = NULL;
            current_idx = 0;
            k_work_reschedule(&blink_work, K_NO_WAIT);
        } else {
            current_pattern = NULL;
            current_idx = 0;
        }
        return;
    }

    current_idx++;
    k_work_reschedule(&blink_work, K_MSEC(step.duration_ms));
}

static void trigger_pattern(const struct blink_step *pattern) {
    if (!led_ready) {
        return;
    }

    /* Не обрываем стартовое двойное мигание при мгновенном коннекте — ставим вспышку следом */
    if (current_pattern == pattern_boot && pattern != pattern_boot) {
        pending_pattern = pattern;
        return;
    }

    pending_pattern = NULL;
    current_pattern = pattern;
    current_idx = 0;
    k_work_reschedule(&blink_work, K_NO_WAIT);
}

static int status_led_init(void) {
    if (!gpio_is_ready_dt(&status_led)) {
        return 0;
    }
    if (gpio_pin_configure_dt(&status_led, GPIO_OUTPUT_INACTIVE) < 0) {
        return 0;
    }

    k_work_init_delayable(&blink_work, blink_work_handler);
    led_ready = true;
    trigger_pattern(pattern_boot);
    return 0;
}

SYS_INIT(status_led_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);

#if !IS_ENABLED(CONFIG_ZMK_SPLIT_ROLE_CENTRAL)
static int status_led_listener(const zmk_event_t *eh) {
    const struct zmk_split_peripheral_status_changed *ev =
        as_zmk_split_peripheral_status_changed(eh);
    if (ev != NULL) {
        trigger_pattern(ev->connected ? pattern_connected : pattern_disconnected);
    }
    return ZMK_EV_EVENT_BUBBLE;
}

ZMK_LISTENER(charybdis_status_led, status_led_listener);
ZMK_SUBSCRIPTION(charybdis_status_led, zmk_split_peripheral_status_changed);
#endif

#endif /* STATUS_LED_NODE */
