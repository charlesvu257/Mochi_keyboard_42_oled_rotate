/*
 * fine!40 — full-screen keystroke ripple widget
 *
 * Every key press drops a "stone" on the OLED at the spot matching the physical
 * key, sending out expanding concentric wavefronts. Caps Lock flips the
 * polarity of the whole panel: normally white ripples on black, with Caps Lock
 * held it becomes black ripples on white.
 *
 * The panel is 1 bit per pixel, so there is no such thing as a dim pixel. The
 * fade of a wavefront is done with an ordered (Bayer) dither instead: as a ring
 * loses energy it is drawn with progressively fewer of its pixels set, which
 * reads as a fade at arm's length.
 *
 * SPDX-License-Identifier: MIT
 */

#include <string.h>

#include <lvgl.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>

#include <zmk/display.h>
#include <zmk/event_manager.h>
#include <zmk/events/hid_indicators_changed.h>
#include <zmk/events/position_state_changed.h>

#include "ripple.h"

#define W FINE40_RIPPLE_W
#define H FINE40_RIPPLE_H

/* Physical key grid, matching the matrix transform in fine40.overlay. */
#define KEY_COLS 12
#define KEY_ROWS 4

/* Wavefronts drawn per impact. Index 0 is the leading edge. */
#define RING_COUNT 3

/* HID keyboard LED report: bit 0 Num Lock, bit 1 Caps Lock, bit 2 Scroll Lock. */
#define HID_INDICATOR_CAPS_LOCK BIT(1)

/*
 * 4x4 ordered dither. A pixel is set when its threshold is below the ring's
 * amplitude (0-16), so amplitude 16 is solid and 0 is empty.
 */
static const uint8_t bayer4[4][4] = {
    {0, 8, 2, 10},
    {12, 4, 14, 6},
    {3, 11, 1, 9},
    {15, 7, 13, 5},
};

/* Trailing wavefronts carry less energy than the leading one. */
static const uint8_t ring_weight[RING_COUNT] = {16, 10, 5};

/*
 * Key presses arrive on the ZMK event thread but the canvas is only ever
 * touched from the display work queue, so impacts are handed over through a
 * queue rather than written straight into the widget. Caps Lock is a level, not
 * an event, so a plain atomic is enough for it.
 */
struct ripple_impact {
    int16_t x;
    int16_t y;
};

K_MSGQ_DEFINE(ripple_impact_q, sizeof(struct ripple_impact), 16, 4);

static atomic_t caps_lock_on = ATOMIC_INIT(0);

static void plot(lv_color_t *buf, int x, int y, uint8_t amp, lv_color_t ink) {
    if (x < 0 || x >= W || y < 0 || y >= H) {
        return;
    }
    if (bayer4[y & 3][x & 3] >= amp) {
        return;
    }
    buf[y * W + x] = ink;
}

/* Midpoint circle, eight-way symmetry, dithered by amplitude. */
static void draw_ring(lv_color_t *buf, int cx, int cy, int r, uint8_t amp, lv_color_t ink) {
    if (r <= 0 || amp == 0) {
        return;
    }

    int x = r;
    int y = 0;
    int err = 1 - r;

    while (x >= y) {
        plot(buf, cx + x, cy + y, amp, ink);
        plot(buf, cx + y, cy + x, amp, ink);
        plot(buf, cx - y, cy + x, amp, ink);
        plot(buf, cx - x, cy + y, amp, ink);
        plot(buf, cx - x, cy - y, amp, ink);
        plot(buf, cx - y, cy - x, amp, ink);
        plot(buf, cx + y, cy - x, amp, ink);
        plot(buf, cx + x, cy - y, amp, ink);

        y++;
        if (err < 0) {
            err += 2 * y + 1;
        } else {
            x--;
            err += 2 * (y - x) + 1;
        }
    }
}

static void ripple_spawn(struct zmk_widget_ripple *widget, int16_t x, int16_t y) {
    struct fine40_ripple *slot = &widget->ripples[0];

    /* Prefer a free slot; otherwise recycle the one nearest the end of its life. */
    for (int i = 0; i < CONFIG_FINE40_RIPPLE_MAX_ACTIVE; i++) {
        if (!widget->ripples[i].active) {
            slot = &widget->ripples[i];
            break;
        }
        if (widget->ripples[i].age > slot->age || !slot->active) {
            slot = &widget->ripples[i];
        }
    }

    slot->cx = x;
    slot->cy = y;
    slot->age = 0;
    slot->active = 1;
    widget->dirty = true;
}

static void ripple_render(lv_timer_t *timer) {
    struct zmk_widget_ripple *widget = (struct zmk_widget_ripple *)timer->user_data;

    struct ripple_impact impact;
    while (k_msgq_get(&ripple_impact_q, &impact, K_NO_WAIT) == 0) {
        ripple_spawn(widget, impact.x, impact.y);
    }

    bool caps = atomic_get(&caps_lock_on) != 0;
    if (caps != widget->caps) {
        widget->caps = caps;
        widget->dirty = true;
#if IS_ENABLED(CONFIG_FINE40_RIPPLE_CAPS_SHOCKWAVE)
        ripple_spawn(widget, W / 2, H / 2);
#endif
    }

    bool any = widget->dirty;
    for (int i = 0; !any && i < CONFIG_FINE40_RIPPLE_MAX_ACTIVE; i++) {
        any = widget->ripples[i].active;
    }
    if (!any) {
        /* Nothing moving and nothing stale on screen — leave the panel alone. */
        return;
    }

    /*
     * Caps Lock inverts the field. FINE40_RIPPLE_INVERT flips the whole
     * convention if the panel reads the other way round than expected.
     */
    bool ink_is_white = !caps;
#if IS_ENABLED(CONFIG_FINE40_RIPPLE_INVERT)
    ink_is_white = !ink_is_white;
#endif

    lv_color_t ink = ink_is_white ? lv_color_white() : lv_color_black();

    /* One byte per pixel at LV_COLOR_DEPTH 1; any non-zero byte reads as set. */
    memset(widget->cbuf, ink_is_white ? 0x00 : 0xFF, sizeof(widget->cbuf));

    int still_active = 0;

    for (int i = 0; i < CONFIG_FINE40_RIPPLE_MAX_ACTIVE; i++) {
        struct fine40_ripple *r = &widget->ripples[i];
        if (!r->active) {
            continue;
        }

        int lead = (r->age * CONFIG_FINE40_RIPPLE_SPEED_X10) / 10;

        /* Energy bleeds off as the wave spreads; at max radius it is gone. */
        int env = ((CONFIG_FINE40_RIPPLE_MAX_RADIUS - lead) * 16) /
                  CONFIG_FINE40_RIPPLE_MAX_RADIUS;
        if (env <= 0) {
            r->active = 0;
            continue;
        }

        for (int k = 0; k < RING_COUNT; k++) {
            int radius = lead - (k * CONFIG_FINE40_RIPPLE_WAVELENGTH);
            if (radius <= 0) {
                break;
            }

            uint8_t amp = (uint8_t)((env * ring_weight[k]) / 16);
            if (amp == 0) {
                continue;
            }

            draw_ring(widget->cbuf, r->cx, r->cy, radius, amp, ink);

            /* Thicken the leading edge while it still has real energy. */
            if (k == 0 && amp > 10) {
                draw_ring(widget->cbuf, r->cx, r->cy, radius - 1, amp, ink);
            }
        }

        r->age++;
        still_active++;
    }

    lv_obj_invalidate(widget->obj);

    /* One more frame is owed after the last ripple dies, to clear the panel. */
    widget->dirty = (still_active > 0);
}

static int ripple_position_listener(const zmk_event_t *eh) {
    const struct zmk_position_state_changed *ev = as_zmk_position_state_changed(eh);
    if (ev == NULL || !ev->state) {
        return ZMK_EV_EVENT_BUBBLE;
    }

    uint32_t col = ev->position % KEY_COLS;
    uint32_t row = ev->position / KEY_COLS;
    if (row >= KEY_ROWS) {
        row = KEY_ROWS - 1;
    }

    /* Drop the stone at the centre of the cell belonging to that physical key. */
    struct ripple_impact impact = {
        .x = (int16_t)((col * W) / KEY_COLS + (W / (KEY_COLS * 2))),
        .y = (int16_t)((row * H) / KEY_ROWS + (H / (KEY_ROWS * 2))),
    };

    /* Dropping an impact is better than blocking the event thread. */
    k_msgq_put(&ripple_impact_q, &impact, K_NO_WAIT);

    return ZMK_EV_EVENT_BUBBLE;
}

ZMK_LISTENER(fine40_ripple_position, ripple_position_listener);
ZMK_SUBSCRIPTION(fine40_ripple_position, zmk_position_state_changed);

static int ripple_indicators_listener(const zmk_event_t *eh) {
    const struct zmk_hid_indicators_changed *ev = as_zmk_hid_indicators_changed(eh);
    if (ev != NULL) {
        atomic_set(&caps_lock_on, (ev->indicators & HID_INDICATOR_CAPS_LOCK) ? 1 : 0);
    }
    return ZMK_EV_EVENT_BUBBLE;
}

ZMK_LISTENER(fine40_ripple_indicators, ripple_indicators_listener);
ZMK_SUBSCRIPTION(fine40_ripple_indicators, zmk_hid_indicators_changed);

int zmk_widget_ripple_init(struct zmk_widget_ripple *widget, lv_obj_t *parent) {
    widget->obj = lv_canvas_create(parent);
    lv_canvas_set_buffer(widget->obj, widget->cbuf, W, H, LV_IMG_CF_TRUE_COLOR);

    memset(widget->ripples, 0, sizeof(widget->ripples));
    widget->caps = false;
    widget->dirty = true;

    widget->timer = lv_timer_create(ripple_render, CONFIG_FINE40_RIPPLE_FRAME_MS, widget);

    return 0;
}

lv_obj_t *zmk_widget_ripple_obj(struct zmk_widget_ripple *widget) { return widget->obj; }
