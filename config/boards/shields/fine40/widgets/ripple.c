/*
 * fine!40 — full-screen keystroke ripple widget
 *
 * Every key press drops a "stone" on the OLED at the spot matching the physical
 * key, sending out expanding concentric wavefronts. Caps Lock flips the
 * polarity of the whole panel: normally white ripples on black, with Caps Lock
 * on it becomes black ripples on white.
 *
 * Waves reflect off the four edges and interfere with each other. Reflection is
 * done with image sources: a wavefront crossing a wall is exactly the wavefront
 * of an identical source mirrored through that wall, so a bounce costs one more
 * ring rather than any boundary tracking. Interference is XOR — where two
 * wavefronts meet, the pixel cancels, which is what makes crossing waves read
 * as water rather than as overlapping stencils.
 *
 * The panel is 1 bit per pixel, so there is no such thing as a dim pixel. A
 * wavefront losing energy is drawn through an ordered (Bayer) dither instead:
 * fewer and fewer of its pixels get set, which reads as a fade.
 *
 * SPDX-License-Identifier: MIT
 */

#include <string.h>

#include <lvgl.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/util.h>

#include <zmk/display.h>
#include <zmk/event_manager.h>
#include <zmk/events/hid_indicators_changed.h>
#include <zmk/events/position_state_changed.h>
#include <zmk/events/sensor_event.h>
#include <zmk/keymap.h>

#include "ripple.h"

#define W FINE40_RIPPLE_W
#define H FINE40_RIPPLE_H

/* Physical key grid, matching the matrix transform in fine40.overlay. */
#define KEY_COLS 12
#define KEY_ROWS 4

/* Wavefronts drawn per impact. Index 0 is the leading edge. */
#define RING_COUNT 3

/* A converging wave is a tighter, quieter thing than an impact. */
#define IMPLODE_RING_COUNT 2

/* Trailing wavefronts behind an encoder swipe. */
#define SWIPE_RING_COUNT 3

/*
 * Panel mounting. The framebuffer is always 128 wide by 32 tall because that is
 * how the SSD1306 addresses the glass, but this module is mounted a quarter
 * turn over, so on screen the 128-pixel axis runs top to bottom and the
 * 32-pixel axis runs left to right. Swipes are named for what the user sees, so
 * they need to know which framebuffer axis is physically horizontal.
 */
#if IS_ENABLED(CONFIG_FINE40_RIPPLE_PANEL_ROTATED)
#define HORIZ_ALONG_FB_X 0
#else
#define HORIZ_ALONG_FB_X 1
#endif

/* HID keyboard LED report: bit 0 Num Lock, bit 1 Caps Lock, bit 2 Scroll Lock. */
#define HID_INDICATOR_CAPS_LOCK BIT(1)

/*
 * 4x4 ordered dither. A pixel is drawn when its threshold is below the ring's
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
 * Repeated presses of one key would otherwise spawn perfectly concentric
 * ripples. As soon as two of them drift exactly one wavelength apart their
 * rings coincide pixel for pixel, and XOR interference annihilates both at
 * once — the wave looks like it cancels itself and restarts. Nudging each
 * spawn by a couple of pixels means rings from the same key are never
 * perfectly in phase, which turns total annihilation back into ordinary
 * interference where they happen to cross.
 */
static const int8_t spawn_jitter[8][2] = {
    {0, 0}, {2, 1}, {-1, 2}, {1, -2}, {-2, -1}, {2, -1}, {-2, 1}, {1, 2},
};

static uint8_t spawn_seq;

/*
 * Key presses arrive on the ZMK event thread but the canvas is only ever
 * touched from the display work queue, so impacts are handed over through a
 * queue rather than written straight into the widget. Caps Lock is a level, not
 * an event, so a plain atomic is enough for it.
 */
struct ripple_impact {
    int16_t x;
    int16_t y;
    uint8_t mode;
};

/* Deep enough that a fast roll of press-and-release pairs cannot overrun it. */
K_MSGQ_DEFINE(ripple_impact_q, sizeof(struct ripple_impact), 24, 4);

static atomic_t caps_lock_on = ATOMIC_INIT(0);

/* Set per pass: XOR for the wave field, plain writes for the test pattern. */
static bool draw_xor;
static uint8_t draw_ink;

static inline void put(uint8_t *buf, int px, int py, uint8_t amp) {
    if (px < 0 || px >= W || py < 0 || py >= H) {
        return;
    }
    if (bayer4[py & 3][px & 3] >= amp) {
        return;
    }
    if (draw_xor) {
        buf[py * W + px] ^= 0xFF;
    } else {
        buf[py * W + px] = draw_ink;
    }
}

/*
 * Midpoint circle, eight-way symmetry. aspect_pct squeezes the horizontal axis
 * for panels whose pixels are not square.
 *
 * The octant boundaries (y == 0 and x == y) are special-cased because the plain
 * eight-way form emits those points twice, and under XOR a double-plot cancels
 * itself and punches holes at the poles and diagonals.
 */
static void draw_ring(uint8_t *buf, int cx, int cy, int r, uint8_t amp, int aspect_pct) {
    if (r <= 0 || amp == 0) {
        return;
    }

    int rx = (r * aspect_pct) / 100;
    if (cx + rx < 0 || cx - rx >= W || cy + r < 0 || cy - r >= H) {
        return; /* Nothing of this ring can land on the panel. */
    }

#define PUT(dx, dy) put(buf, cx + (((dx) * aspect_pct) / 100), cy + (dy), amp)

    int x = r;
    int y = 0;
    int err = 1 - r;

    while (x >= y) {
        if (y == 0) {
            PUT(x, 0);
            PUT(-x, 0);
            PUT(0, x);
            PUT(0, -x);
        } else if (x == y) {
            PUT(x, y);
            PUT(-x, y);
            PUT(x, -y);
            PUT(-x, -y);
        } else {
            PUT(x, y);
            PUT(y, x);
            PUT(-y, x);
            PUT(-x, y);
            PUT(-x, -y);
            PUT(-y, -x);
            PUT(y, -x);
            PUT(x, -y);
        }

        y++;
        if (err < 0) {
            err += 2 * y + 1;
        } else {
            x--;
            err += 2 * (y - x) + 1;
        }
    }

#undef PUT
}

/* One wavefront plus its reflections in the four walls. */
static void emit_ring(uint8_t *buf, int cx, int cy, int r, uint8_t amp) {
    const int aspect = CONFIG_FINE40_RIPPLE_ASPECT_PCT;

    draw_ring(buf, cx, cy, r, amp, aspect);

#if CONFIG_FINE40_RIPPLE_REFLECT_PCT > 0
    uint8_t bounced = (uint8_t)((amp * CONFIG_FINE40_RIPPLE_REFLECT_PCT) / 100);
    if (bounced > 0) {
        draw_ring(buf, -cx, cy, r, bounced, aspect);
        draw_ring(buf, 2 * (W - 1) - cx, cy, r, bounced, aspect);
        draw_ring(buf, cx, -cy, r, bounced, aspect);
        draw_ring(buf, cx, 2 * (H - 1) - cy, r, bounced, aspect);
    }
#endif
}

static void draw_hline(uint8_t *buf, int x0, int x1, int y, uint8_t amp) {
    for (int x = x0; x <= x1; x++) {
        put(buf, x, y, amp);
    }
}

static void draw_vline(uint8_t *buf, int x, int y0, int y1, uint8_t amp) {
    for (int y = y0; y <= y1; y++) {
        put(buf, x, y, amp);
    }
}

/*
 * An encoder swipe is a plane wave: a straight wavefront crossing the panel,
 * with a couple of weaker fronts trailing it. Returns false once the whole
 * train, tail included, has left the far edge.
 */
static bool draw_swipe(uint8_t *buf, uint8_t mode, int travelled) {
    bool horizontal =
        (mode == FINE40_RIPPLE_SWIPE_RIGHT || mode == FINE40_RIPPLE_SWIPE_LEFT);

    /* A wavefront is perpendicular to its travel, so it spans the other axis. */
    bool along_fb_x = horizontal ? HORIZ_ALONG_FB_X : !HORIZ_ALONG_FB_X;
    int span = along_fb_x ? W : H;

    int tail = SWIPE_RING_COUNT * CONFIG_FINE40_RIPPLE_WAVELENGTH;
    if (travelled > span + tail) {
        return false;
    }

    bool forward = (mode == FINE40_RIPPLE_SWIPE_RIGHT || mode == FINE40_RIPPLE_SWIPE_DOWN);
#if IS_ENABLED(CONFIG_FINE40_RIPPLE_SWIPE_FLIP_H)
    if (horizontal) {
        forward = !forward;
    }
#endif
#if IS_ENABLED(CONFIG_FINE40_RIPPLE_SWIPE_FLIP_V)
    if (!horizontal) {
        forward = !forward;
    }
#endif

    int lead = forward ? travelled : (span - 1 - travelled);

    for (int k = 0; k < SWIPE_RING_COUNT; k++) {
        int offset = k * CONFIG_FINE40_RIPPLE_WAVELENGTH;
        int pos = forward ? (lead - offset) : (lead + offset);
        if (pos < 0 || pos >= span) {
            continue;
        }

        if (along_fb_x) {
            draw_vline(buf, pos, 0, H - 1, ring_weight[k]);
        } else {
            draw_hline(buf, 0, W - 1, pos, ring_weight[k]);
        }
    }

    return true;
}

/*
 * Geometry check. Drawn with no aspect correction on purpose, so it measures
 * the panel rather than the compensation:
 *
 *   - the outer rectangle marks where the firmware thinks the edges are. If it
 *     is off-screen, cut off, or doubled, width/height/multiplex-ratio in the
 *     overlay do not match the real panel.
 *   - the square and the circle inside it have the same size. On square pixels
 *     the circle touches the square at the midpoint of each side. If the circle
 *     is a wide oval, the panel's pixels are wider than they are tall, and
 *     CONFIG_FINE40_RIPPLE_ASPECT_PCT is the correction.
 */
static void draw_test_pattern(uint8_t *buf) {
    int side = MIN(W, H) - 6;
    int left = (W - side) / 2;
    int top = (H - side) / 2;

    draw_hline(buf, 0, W - 1, 0, 16);
    draw_hline(buf, 0, W - 1, H - 1, 16);
    draw_vline(buf, 0, 0, H - 1, 16);
    draw_vline(buf, W - 1, 0, H - 1, 16);

    draw_hline(buf, left, left + side, top, 16);
    draw_hline(buf, left, left + side, top + side, 16);
    draw_vline(buf, left, top, top + side, 16);
    draw_vline(buf, left + side, top, top + side, 16);

    draw_ring(buf, W / 2, H / 2, side / 2, 16, 100);
}

static void ripple_spawn(struct zmk_widget_ripple *widget, int16_t x, int16_t y, uint8_t mode) {
    struct fine40_ripple *slot = &widget->ripples[0];

    /* Prefer a free slot; otherwise recycle the one nearest the end of its life. */
    for (int i = 0; i < CONFIG_FINE40_RIPPLE_MAX_ACTIVE; i++) {
        if (!widget->ripples[i].active) {
            slot = &widget->ripples[i];
            break;
        }
        if (widget->ripples[i].age > slot->age) {
            slot = &widget->ripples[i];
        }
    }

    const int8_t *nudge = spawn_jitter[spawn_seq++ & 7];

    slot->cx = x + nudge[0];
    slot->cy = y + nudge[1];
    slot->age = 0;
    slot->active = 1;
    slot->mode = mode;
    widget->dirty = true;
}

static void ripple_render(lv_timer_t *timer) {
    struct zmk_widget_ripple *widget = (struct zmk_widget_ripple *)timer->user_data;
    uint8_t *buf = (uint8_t *)widget->cbuf;

    struct ripple_impact impact;
    bool struck = false;
    while (k_msgq_get(&ripple_impact_q, &impact, K_NO_WAIT) == 0) {
        ripple_spawn(widget, impact.x, impact.y, impact.mode);
        struck = true;
    }

    /* The geometry check stays up until the first key press clears it. */
    if (widget->test_pattern && struck) {
        widget->test_pattern = false;
        widget->dirty = true;
    }

    bool caps = atomic_get(&caps_lock_on) != 0;
    if (caps != widget->caps) {
        widget->caps = caps;
        widget->dirty = true;
#if IS_ENABLED(CONFIG_FINE40_RIPPLE_CAPS_SHOCKWAVE)
        ripple_spawn(widget, W / 2, H / 2, FINE40_RIPPLE_EXPAND);
#endif
    }

    bool any = widget->dirty || widget->test_pattern;
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
    bool field_is_black = !caps;
#if IS_ENABLED(CONFIG_FINE40_RIPPLE_INVERT)
    field_is_black = !field_is_black;
#endif

    /* One byte per pixel at LV_COLOR_DEPTH 1; any non-zero byte reads as set. */
    uint8_t background = field_is_black ? 0x00 : 0xFF;
    memset(buf, background, sizeof(widget->cbuf));
    draw_ink = (uint8_t)~background;

    if (widget->test_pattern) {
        draw_xor = false;
        draw_test_pattern(buf);
        lv_obj_invalidate(widget->obj);
        widget->dirty = false;
        return;
    }

    draw_xor = IS_ENABLED(CONFIG_FINE40_RIPPLE_INTERFERENCE);

    int still_active = 0;

    for (int i = 0; i < CONFIG_FINE40_RIPPLE_MAX_ACTIVE; i++) {
        struct fine40_ripple *r = &widget->ripples[i];
        if (!r->active) {
            continue;
        }

        int travelled = (r->age * CONFIG_FINE40_RIPPLE_SPEED_X10) / 10;

        if (r->mode >= FINE40_RIPPLE_SWIPE_RIGHT) {
            /* Swipes cross the panel on their own, faster clock. */
            int swept = (r->age * CONFIG_FINE40_RIPPLE_SWIPE_SPEED_X10) / 10;
            if (!draw_swipe(buf, r->mode, swept)) {
                r->active = 0;
                continue;
            }
        } else if (r->mode == FINE40_RIPPLE_IMPLODE) {
            const int start = CONFIG_FINE40_RIPPLE_RELEASE_RADIUS;

            int lead = start - travelled;
            if (lead <= 0) {
                r->active = 0;
                continue;
            }

            /*
             * A converging wave concentrates its energy into a shrinking
             * circumference, so it brightens as it collapses onto the key —
             * the exact inverse of an expanding ripple fading as it spreads.
             * The floor keeps it faintly visible from the moment it appears.
             */
            int env = (16 * travelled) / start;
            env = CLAMP(env, 3, 16);

            for (int k = 0; k < IMPLODE_RING_COUNT; k++) {
                /*
                 * Trailing wavefronts sit *outside* a converging wave: they
                 * are the parts that have not caught up to the front yet.
                 * Reflections are skipped — this wave is running away from
                 * the walls, not into them.
                 */
                int radius = lead + (k * CONFIG_FINE40_RIPPLE_WAVELENGTH);
                uint8_t amp = (uint8_t)((env * ring_weight[k]) / 16);
                if (amp > 0) {
                    draw_ring(buf, r->cx, r->cy, radius, amp, CONFIG_FINE40_RIPPLE_ASPECT_PCT);
                }
            }
        } else {
            int lead = travelled;

            /* Energy bleeds off as the wave spreads; at max radius it is gone. */
            int env =
                ((CONFIG_FINE40_RIPPLE_MAX_RADIUS - lead) * 16) / CONFIG_FINE40_RIPPLE_MAX_RADIUS;
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
                if (amp > 0) {
                    emit_ring(buf, r->cx, r->cy, radius, amp);
                }
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
    if (ev == NULL) {
        return ZMK_EV_EVENT_BUBBLE;
    }
    if (!ev->state && !IS_ENABLED(CONFIG_FINE40_RIPPLE_ON_RELEASE)) {
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
        .mode = ev->state ? FINE40_RIPPLE_EXPAND : FINE40_RIPPLE_IMPLODE,
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

#if IS_ENABLED(CONFIG_FINE40_RIPPLE_SWIPE_ON_ENCODER)

static int64_t last_swipe_uptime;

static int ripple_sensor_listener(const zmk_event_t *eh) {
    const struct zmk_sensor_event *ev = as_zmk_sensor_event(eh);
    if (ev == NULL || ev->channel_data_size < 1) {
        return ZMK_EV_EVENT_BUBBLE;
    }

    /*
     * Only the sign matters here. The EC11 driver's units are not worth
     * depending on: how many events arrive per detent varies with the encoder's
     * resolution, so rather than trying to reconstruct detents this rate-limits
     * to one swipe per FINE40_RIPPLE_SWIPE_MIN_MS. A fast spin then reads as a
     * steady train of swipes, which is the right feel anyway.
     */
    struct sensor_value value = ev->channel_data[0].value;
    int delta = (value.val1 != 0) ? value.val1 : value.val2;
    if (delta == 0) {
        return ZMK_EV_EVENT_BUBBLE;
    }

    int64_t now = k_uptime_get();
    if (now - last_swipe_uptime < CONFIG_FINE40_RIPPLE_SWIPE_MIN_MS) {
        return ZMK_EV_EVENT_BUBBLE;
    }
    last_swipe_uptime = now;

    /*
     * Follow the encoder's keymap bindings: the base layer scrolls the cursor
     * left and right, every layer above it moves up and down.
     */
    bool clockwise = (delta > 0);
    bool vertical = (zmk_keymap_highest_layer_active() > 0);

    uint8_t mode;
    if (vertical) {
        mode = clockwise ? FINE40_RIPPLE_SWIPE_DOWN : FINE40_RIPPLE_SWIPE_UP;
    } else {
        mode = clockwise ? FINE40_RIPPLE_SWIPE_RIGHT : FINE40_RIPPLE_SWIPE_LEFT;
    }

    /* A swipe spans the whole panel, so its origin is unused. */
    struct ripple_impact impact = {.x = 0, .y = 0, .mode = mode};
    k_msgq_put(&ripple_impact_q, &impact, K_NO_WAIT);

    return ZMK_EV_EVENT_BUBBLE;
}

ZMK_LISTENER(fine40_ripple_sensor, ripple_sensor_listener);
ZMK_SUBSCRIPTION(fine40_ripple_sensor, zmk_sensor_event);

#endif /* CONFIG_FINE40_RIPPLE_SWIPE_ON_ENCODER */

int zmk_widget_ripple_init(struct zmk_widget_ripple *widget, lv_obj_t *parent) {
    widget->obj = lv_canvas_create(parent);
    lv_canvas_set_buffer(widget->obj, widget->cbuf, W, H, LV_IMG_CF_TRUE_COLOR);

    memset(widget->ripples, 0, sizeof(widget->ripples));
    widget->caps = false;
    widget->dirty = true;
    widget->test_pattern = IS_ENABLED(CONFIG_FINE40_RIPPLE_TEST_PATTERN);

    widget->timer = lv_timer_create(ripple_render, CONFIG_FINE40_RIPPLE_FRAME_MS, widget);

    return 0;
}

lv_obj_t *zmk_widget_ripple_obj(struct zmk_widget_ripple *widget) { return widget->obj; }
