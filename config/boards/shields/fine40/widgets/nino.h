/*
 * NINO — Notetaking Input, Networkless Output
 *
 * OLED status screen: boot splash, typewriter carriage, and a wireframe globe
 * driven by the encoder.
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

/*
 * Every screen but the boot splash is the same machine: a cartridge at the top
 * and the globe at the bottom, both always present. Only the sheet inside the
 * cartridge changes, and a view is which sheet that is. Changing one wipes
 * through the track and leaves the machine around it alone.
 */
enum nino_view {
    NINO_VIEW_MAIN = 0,
    NINO_VIEW_DESTOW_MENU,
    NINO_VIEW_DESTOW_CONFIRM,
    NINO_VIEW_DESTOW_SENT,
    NINO_VIEW_DESTOW_EMPTY,
    NINO_VIEW_XFER_IN,  /* the host is filling the store */
    NINO_VIEW_XFER_OUT, /* the host is emptying it */
};

/*
 * A sheet change is a carriage return. The bar throws itself home, which empties
 * the cartridge, and then travels back down uncovering the new sheet as it goes.
 * The knob click is the selector everywhere, so the machine's answer to being
 * clicked is the same gesture everywhere too.
 */
enum nino_shift_phase {
    NINO_SHIFT_NONE = 0,
    NINO_SHIFT_SLAM,   /* bar flying home, old sheet withdrawing behind it */
    NINO_SHIFT_REVEAL, /* bar descending, new sheet appearing above it */
};

/* What arrived from the input threads, queued for the display thread. */
enum nino_input_kind {
    NINO_INPUT_CHAR = 0,
    NINO_INPUT_BACKSPACE,
    NINO_INPUT_RETURN,
    NINO_INPUT_KNOB_TURN, /* value carries +1 / -1 */
    NINO_INPUT_KNOB_CLICK,
    NINO_INPUT_DESTOW, /* enter or leave the note-reading screen */
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

    /* Carriage. column is the character count within the current cartridge
     * pass; carriage_pos is where the bar is drawn on the track, in pixels,
     * which lags behind during a slam. The two are no longer one-to-one: a
     * cartridge holds more characters than the track has pixels. */
    uint16_t column;
    int16_t carriage_pos;

    /*
     * Where the bar is actually drawn, in sixteenths of a pixel, easing toward
     * carriage_pos. A cartridge is more characters than the track is pixels, so
     * stepping straight to the target made the bar stand still for a keystroke
     * or two and then jump — a tick rather than travel. Interpolating spreads
     * each step over a few frames so the motion reads as continuous.
     */
    int32_t carriage_shown;

    /*
     * Which pass over the track this is. A full cartridge sends the bar back
     * to the top and starts a new pass in a heavier texture; only a slam
     * resets it to zero.
     */
    uint8_t fill_layer;

    bool slamming;

    /* The slam overshoots the stop and springs back, so the return lands as an
     * impact rather than stopping dead at zero. */
    bool rebounding;

    uint8_t shake_frames;
    uint8_t shake_mag;  /* pixels of offset: small for a ruler tick, big for a slam */
    bool shake_lateral; /* slams throw the screen sideways, ticks jolt it down */

    /* Frames since the last input. Past the idle threshold the globe turns on
     * its own; any input stops it where it stands. */
    uint16_t idle_frames;

    /*
     * Passing a ruler graduation strikes it: that mark reaches further into the
     * track for a few frames. tick_mark is which graduation, by index, so the
     * emphasis stays on the mark itself even as the carriage moves past it.
     */
    uint8_t tick_flash;
    int8_t tick_mark;

    /*
     * A note transfer running on the host link. While this is set the track is
     * a progress bar rather than a cartridge; the accumulated passes are left
     * untouched underneath and come back when it finishes.
     */
    bool transferring;
    uint8_t transfer_progress; /* 0..255 */

    /*
     * Globe orientation, in sixteenths of a sine-table step (1024 to the turn).
     * Current springs toward target so a detent lands with some weight rather
     * than snapping; both are free-running rather than wrapped, so the
     * difference between them is always unambiguous and the spring never takes
     * the long way round. The velocities are what give it mass.
     */
    int32_t yaw;
    int32_t yaw_target;
    int32_t yaw_vel;
    int32_t pitch;
    int32_t pitch_target;
    int32_t pitch_vel;

    /* Whole-screen invert. Used only by the carriage-return slam. */
    uint8_t strike_timer;

    /*
     * The globe's local flash, fired by a knob click. Runs on its own clock so
     * typing cannot cut it short.
     */
    uint8_t globe_flash;

    /* The globe kicks upward when the carriage slams, then settles. */
    int8_t ball_recoil;

    /*
     * Which sheet is in the machine, which one it was, and where the bar has
     * got to between them. Both sheets are kept because each is drawn on its
     * own side of the travelling bar.
     */
    uint8_t view;
    uint8_t view_prev;
    uint8_t shift_phase;
    int16_t shift_pos; /* the bar, in pixels down the track */

    bool caps;
    bool dirty;
    bool test_pattern;
};

int zmk_widget_nino_init(struct zmk_widget_nino *widget, lv_obj_t *parent);
lv_obj_t *zmk_widget_nino_obj(struct zmk_widget_nino *widget);
