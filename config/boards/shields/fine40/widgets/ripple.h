/*
 * fine!40 — full-screen keystroke ripple widget
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

#include <lvgl.h>
#include <zephyr/devicetree.h>
#include <zephyr/kernel.h>

/*
 * Panel geometry comes from the devicetree, not from constants here, so the
 * widget follows whatever `zephyr,display` is set to. Correct width/height in
 * fine40.overlay and everything below adapts on the next build.
 */
#define FINE40_DISPLAY_NODE DT_CHOSEN(zephyr_display)
#define FINE40_RIPPLE_W DT_PROP(FINE40_DISPLAY_NODE, width)
#define FINE40_RIPPLE_H DT_PROP(FINE40_DISPLAY_NODE, height)

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
     * lv_color_t is a single byte, so the 128x32 panel costs 4 KB of .bss.
     */
    lv_color_t cbuf[FINE40_RIPPLE_W * FINE40_RIPPLE_H];

    struct fine40_ripple ripples[CONFIG_FINE40_RIPPLE_MAX_ACTIVE];
    bool caps;
    bool dirty;
    bool test_pattern;
};

int zmk_widget_ripple_init(struct zmk_widget_ripple *widget, lv_obj_t *parent);
lv_obj_t *zmk_widget_ripple_obj(struct zmk_widget_ripple *widget);
