/*
 * fine!40 — full-screen keystroke ripple widget
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

#include <lvgl.h>
#include <zephyr/kernel.h>

/* Matches the SSD1306 in fine40.overlay. */
#define FINE40_RIPPLE_W 128
#define FINE40_RIPPLE_H 64

struct fine40_ripple {
    int16_t cx;
    int16_t cy;
    uint16_t age; /* frames since spawn */
    uint8_t active;
};

struct zmk_widget_ripple {
    lv_obj_t *obj;
    lv_timer_t *timer;

    /*
     * One lv_color_t per pixel. At LV_COLOR_DEPTH 1 (set by Kconfig.defconfig)
     * lv_color_t is a single byte, so this is 8 KB of .bss.
     */
    lv_color_t cbuf[FINE40_RIPPLE_W * FINE40_RIPPLE_H];

    struct fine40_ripple ripples[CONFIG_FINE40_RIPPLE_MAX_ACTIVE];
    bool caps;
    bool dirty;
};

int zmk_widget_ripple_init(struct zmk_widget_ripple *widget, lv_obj_t *parent);
lv_obj_t *zmk_widget_ripple_obj(struct zmk_widget_ripple *widget);
