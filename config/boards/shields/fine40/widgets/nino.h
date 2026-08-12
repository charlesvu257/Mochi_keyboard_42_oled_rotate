/*
 * NINO — Notetaking Input, Networkless Output
 *
 * OLED status screen: boot splash, typewriter carriage, margin bell, and a
 * wireframe globe driven by the encoder.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

#include <lvgl.h>
#include <zephyr/devicetree.h>
#include <zephyr/kernel.h>

/*
 * Panel geometry comes from the devicetree, so the widget follows whatever
 * `zephyr,display` is set to. Correct width/height in fine40.overlay and
 * everything below adapts on the next build.
 */
#define NINO_DISPLAY_NODE DT_CHOSEN(zephyr_display)
#define NINO_FB_W DT_PROP(NINO_DISPLAY_NODE, width)
#define NINO_FB_H DT_PROP(NINO_DISPLAY_NODE, height)

/* What arrived from the input threads, queued for the display thread. */
enum nino_input_kind {
    NINO_INPUT_CHAR = 0,
    NINO_INPUT_BACKSPACE,
    NINO_INPUT_RETURN,
    NINO_INPUT_KNOB_TURN, /* value carries +1 / -1 */
    NINO_INPUT_KNOB_CLICK,
};

struct zmk_widget_nino {
    lv_obj_t *obj;
    lv_timer_t *timer;

    /*
     * One lv_color_t per pixel. At LV_COLOR_DEPTH 1 (set by Kconfig.defconfig)
     * lv_color_t is a single byte, so the 128x32 panel costs 4 KB of .bss.
     */
    lv_color_t cbuf[NINO_FB_W * NINO_FB_H];

    /* Counts down. splash_total is kept so the phase can be worked out from
     * how much has elapsed rather than tracked separately. */
    uint16_t splash_frames;
    uint16_t splash_total;

    /* Carriage. column is the logical count; carriage_pos is what is drawn,
     * which lags behind during a slam. */
    uint16_t column;
    int16_t carriage_pos;
    bool slamming;
    bool bell_rung;
    uint8_t shake_frames;
    uint8_t shake_mag; /* pixels of offset: small for a ruler tick, big for a slam */

    /*
     * Globe orientation, in sixteenths of a sine-table step (1024 to the turn).
     * Current eases toward target so a detent glides rather than snaps; both
     * are free-running rather than wrapped, so the difference between them is
     * always unambiguous and the easing never takes the long way round.
     */
    int32_t yaw;
    int32_t yaw_target;
    int32_t pitch;
    int32_t pitch_target;

    /* Whole-screen invert. Used only by the carriage-return slam. */
    uint8_t strike_timer;

    /*
     * The globe's local flash: one pulse for a knob click, two for the margin
     * bell. Runs on its own clock so typing cannot cut it short.
     */
    uint8_t globe_flash;
    uint8_t globe_flash_pending;

    /* The globe kicks upward when the carriage slams, then settles. */
    int8_t ball_recoil;

    bool caps;
    bool dirty;
    bool test_pattern;
};

int zmk_widget_nino_init(struct zmk_widget_nino *widget, lv_obj_t *parent);
lv_obj_t *zmk_widget_nino_obj(struct zmk_widget_nino *widget);
