/*
 * NINO — Notetaking Input, Networkless Output
 *
 * The panel is mounted a quarter turn over, so everything here is drawn in
 * "screen space": sx runs left to right across the narrow 32-pixel dimension,
 * sy runs top to bottom down the long 128-pixel one. screen_plot() is the only
 * place that knows about the rotation, and it is also where the carriage-return
 * shake offset is applied, so a shake costs nothing but two added integers.
 *
 * Top of screen    margin bell, flashes as you approach the right margin
 * Middle           the carriage: a bar stepping one pixel per character
 * Bottom           a wireframe globe the encoder turns
 *
 * The panel is 1 bit per pixel, so there is no such thing as a dim pixel.
 * Anything that needs to fade is drawn through an ordered (Bayer) dither
 * instead: fewer and fewer of its pixels get set, which reads as a fade.
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
#include <zmk/events/keycode_state_changed.h>
#include <zmk/events/position_state_changed.h>
#include <zmk/events/sensor_event.h>
#include <zmk/keymap.h>

#include "nino.h"

#define FB_W NINO_FB_W
#define FB_H NINO_FB_H

/*
 * Screen space. With the module mounted a quarter turn over, the framebuffer's
 * long axis is physically vertical, so the screen is 32 wide by 128 tall.
 */
#if IS_ENABLED(CONFIG_NINO_PANEL_ROTATED)
#define SCREEN_W FB_H
#define SCREEN_H FB_W
#else
#define SCREEN_W FB_W
#define SCREEN_H FB_H
#endif

/* HID keyboard usage IDs we care about. */
#define KC_ENTER 0x28
#define KC_BACKSPACE 0x2A
#define KC_SPACE 0x2C

/* Layout, in screen space. The track starts near the top now that the bell is
 * the globe rather than a glyph in the corner. */
#define TRACK_TOP 8
#define CARRIAGE_POINT 4 /* height of the chevron under the carriage bar */

#define BALL_CX (SCREEN_W / 2)
#define BALL_CY (SCREEN_H - 22)

/* The globe's own flash: a knob click, or the margin bell. */
#define GLOBE_FLASH_TOTAL 10

/* A ruler tick every this many characters, and a nudge when one is crossed. */
#define RULER_MAJOR 25
#define TICK_SHAKE_FRAMES 2
#define TICK_SHAKE_MAG 1

/*
 * 4x4 ordered dither. A pixel is drawn when its threshold is below the
 * requested amplitude (0-16), so amplitude 16 is solid and 0 is empty.
 */
static const uint8_t bayer4[4][4] = {
    {0, 8, 2, 10},
    {12, 4, 14, 6},
    {3, 11, 1, 9},
    {15, 7, 13, 5},
};

/*
 * Input arrives on the ZMK event threads but the canvas is only ever touched
 * from the display work queue, so events are handed over through a queue rather
 * than written straight into the widget. Caps Lock is a level, not an event, so
 * a plain atomic is enough for it.
 */
struct nino_input {
    int8_t value;
    uint8_t kind;
};

K_MSGQ_DEFINE(nino_input_q, sizeof(struct nino_input), 24, 4);

static atomic_t caps_lock_on = ATOMIC_INIT(0);

static uint8_t draw_ink;
static int8_t shake_dx;
static int8_t shake_dy;

/* ---------------------------------------------------------------- pixels -- */

static inline void put_fb(uint8_t *buf, int px, int py, uint8_t amp) {
    if (px < 0 || px >= FB_W || py < 0 || py >= FB_H) {
        return;
    }
    if (bayer4[py & 3][px & 3] >= amp) {
        return;
    }
    buf[py * FB_W + px] = draw_ink;
}

/*
 * The single place that knows the panel is rotated, and the single place the
 * carriage-return shake is applied. Both rotations are true quarter turns
 * rather than a transpose, because a transpose would mirror the splash text.
 */
static inline void screen_plot(uint8_t *buf, int sx, int sy, uint8_t amp) {
    sx += shake_dx;
    sy += shake_dy;

    if (sx < 0 || sx >= SCREEN_W || sy < 0 || sy >= SCREEN_H) {
        return;
    }

#if IS_ENABLED(CONFIG_NINO_PANEL_ROTATED)
#if IS_ENABLED(CONFIG_NINO_PANEL_FLIP)
    put_fb(buf, FB_W - 1 - sy, sx, amp);
#else
    put_fb(buf, sy, FB_H - 1 - sx, amp);
#endif
#else
    put_fb(buf, sx, sy, amp);
#endif
}

static void screen_hline(uint8_t *buf, int x0, int x1, int y, uint8_t amp) {
    for (int x = x0; x <= x1; x++) {
        screen_plot(buf, x, y, amp);
    }
}

static void screen_vline(uint8_t *buf, int x, int y0, int y1, uint8_t amp) {
    for (int y = y0; y <= y1; y++) {
        screen_plot(buf, x, y, amp);
    }
}

static void screen_rect(uint8_t *buf, int x, int y, int w, int h, uint8_t amp) {
    for (int j = 0; j < h; j++) {
        screen_hline(buf, x, x + w - 1, y + j, amp);
    }
}

/*
 * Midpoint circle. The octant boundaries are not special-cased here the way
 * they were for the old XOR wave field: everything now draws solid, so a
 * double-plot is harmless.
 */
static void screen_circle(uint8_t *buf, int cx, int cy, int r, uint8_t amp) {
    if (r <= 0 || amp == 0) {
        return;
    }

    int x = r;
    int y = 0;
    int err = 1 - r;

    while (x >= y) {
        screen_plot(buf, cx + x, cy + y, amp);
        screen_plot(buf, cx + y, cy + x, amp);
        screen_plot(buf, cx - y, cy + x, amp);
        screen_plot(buf, cx - x, cy + y, amp);
        screen_plot(buf, cx - x, cy - y, amp);
        screen_plot(buf, cx - y, cy - x, amp);
        screen_plot(buf, cx + y, cy - x, amp);
        screen_plot(buf, cx + x, cy - y, amp);

        y++;
        if (err < 0) {
            err += 2 * y + 1;
        } else {
            x--;
            err += 2 * (y - x) + 1;
        }
    }
}

static void screen_disc(uint8_t *buf, int cx, int cy, int r, uint8_t amp) {
    for (int dy = -r; dy <= r; dy++) {
        int half = 0;
        while (half * half + dy * dy <= r * r) {
            half++;
        }
        half--; /* Step back to the last column that actually fit. */
        if (half >= 0) {
            screen_hline(buf, cx - half, cx + half, cy + dy, amp);
        }
    }
}

/* ------------------------------------------------------------ fixed point -- */

/*
 * sin over one quadrant, scaled by 4096, at 16 steps to the quarter turn. The
 * other three quadrants come from symmetry, which keeps this to 34 bytes of
 * flash rather than a full table, and avoids needing any floating point — ZMK
 * does not guarantee CONFIG_FPU, and a float here could drag in soft-float.
 */
static const int16_t sin_quadrant[17] = {
    0,    401,  799,  1189, 1567, 1931, 2276, 2598, 2896,
    3166, 3406, 3612, 3784, 3920, 4017, 4076, 4096,
};

/* Angle is an index into a 64-step turn. */
static int fx_sin(int a) {
    a &= 63;
    if (a <= 16) {
        return sin_quadrant[a];
    }
    if (a <= 32) {
        return sin_quadrant[32 - a];
    }
    if (a <= 48) {
        return -sin_quadrant[a - 32];
    }
    return -sin_quadrant[64 - a];
}

static int fx_cos(int a) { return fx_sin(a + 16); }

/* Glide toward the target rather than snapping to it. */
static int32_t ease(int32_t current, int32_t target) {
    int32_t delta = target - current;
    if (delta == 0) {
        return current;
    }
    int32_t step = delta / 4;
    if (step == 0) {
        step = (delta > 0) ? 1 : -1;
    }
    return current + step;
}

/* ---------------------------------------------------------------- splash -- */

/*
 * NINO, in 5x5 blocks blown up so the letters read as ASCII art. Stacked down
 * the long axis because the panel is only 32 pixels wide — four letters side by
 * side would leave 8 pixels each.
 */
static const uint8_t glyph_n[5] = {0x11, 0x19, 0x15, 0x13, 0x11};
static const uint8_t glyph_i[5] = {0x1F, 0x04, 0x04, 0x04, 0x1F};
static const uint8_t glyph_o[5] = {0x0E, 0x11, 0x11, 0x11, 0x0E};

static const uint8_t *const nino_letters[4] = {glyph_n, glyph_i, glyph_n, glyph_o};

#define SPLASH_SCALE 5
#define SPLASH_CELL (5 * SPLASH_SCALE)
#define SPLASH_GAP 2
#define SPLASH_BLOCK (4 * SPLASH_CELL + 3 * SPLASH_GAP)

/*
 * The name types itself in a letter at a time and is backspaced away again.
 * Erasing runs at twice the speed of typing, the way a held backspace does.
 */
#define SPLASH_TYPE_FRAMES 6
#define SPLASH_ERASE_FRAMES 3
#define SPLASH_IN (4 * SPLASH_TYPE_FRAMES)
#define SPLASH_OUT (4 * SPLASH_ERASE_FRAMES)

/* How many of the four letters are on screen this frame. */
static int splash_letters_shown(uint16_t remaining, uint16_t total) {
    uint16_t elapsed = total - remaining;

    if (elapsed < SPLASH_IN) {
        return (elapsed / SPLASH_TYPE_FRAMES) + 1;
    }
    if (remaining <= SPLASH_OUT) {
        /* Biased so each count holds for exactly SPLASH_ERASE_FRAMES and the
         * run ends on blank frames — otherwise the last letter is still on
         * screen when the splash hands over, and the name appears to snap
         * away rather than finish. */
        return (remaining - 1) / SPLASH_ERASE_FRAMES;
    }
    return 4;
}

static void draw_splash(uint8_t *buf, int letters, bool caret) {
    if (letters <= 0) {
        return;
    }

    int top = (SCREEN_H - SPLASH_BLOCK) / 2;
    int left = (SCREEN_W - SPLASH_CELL) / 2;

    /* Rules top and bottom, so the name sits on something. */
    screen_hline(buf, 2, SCREEN_W - 3, top - 6, 16);
    screen_hline(buf, 2, SCREEN_W - 3, top + SPLASH_BLOCK + 5, 16);

    /*
     * The cursor waits at the top of the next empty cell rather than under the
     * last filled one — below the fourth letter it would collide with the
     * bottom rule.
     */
    if (caret && letters < 4) {
        int cell_top = top + letters * (SPLASH_CELL + SPLASH_GAP);
        screen_rect(buf, left, cell_top, SPLASH_CELL, 2, 16);
    }

    for (int letter = 0; letter < letters && letter < 4; letter++) {
        const uint8_t *rows = nino_letters[letter];
        int cell_top = top + letter * (SPLASH_CELL + SPLASH_GAP);

        for (int row = 0; row < 5; row++) {
            for (int col = 0; col < 5; col++) {
                /* Bit 4 is the leftmost column of the glyph. */
                if (!(rows[row] & (1 << (4 - col)))) {
                    continue;
                }
                screen_rect(buf, left + col * SPLASH_SCALE, cell_top + row * SPLASH_SCALE,
                            SPLASH_SCALE, SPLASH_SCALE, 16);
            }
        }
    }
}

/* --------------------------------------------------------------- carriage -- */

static void draw_carriage(uint8_t *buf, const struct zmk_widget_nino *widget) {
    const int track_len = CONFIG_NINO_LINE_LENGTH;

    /*
     * Everything the carriage has already passed over is filled in, standing
     * for text on the line. Dithered rather than solid: on a 1-bit panel a
     * solid block would be as bright as the carriage bar itself and the two
     * would read as one shape. A stipple sits visibly below it instead, which
     * is as close to a second grey as this display gets.
     *
     * Drawn first so the ruler and the bar land on top of it, and it drains
     * on its own during a slam — the fill is defined by where the carriage
     * is, so the carriage flying home empties it without any extra state.
     */
    int filled_to = TRACK_TOP + widget->carriage_pos;
    for (int y = TRACK_TOP; y < filled_to; y++) {
        screen_hline(buf, 1, SCREEN_W - 2, y, CONFIG_NINO_FILL_AMP);
    }

    /*
     * A ruler rather than a rail: one tick per five characters on each side,
     * a longer one every twenty-five. Position becomes countable instead of
     * being a bar sliding along a featureless line.
     */
    for (int c = 0; c <= track_len; c += 5) {
        int y = TRACK_TOP + c;
        int len = (c % RULER_MAJOR == 0) ? 3 : 1;
        screen_hline(buf, 0, len - 1, y, 16);
        screen_hline(buf, SCREEN_W - len, SCREEN_W - 1, y, 16);
    }

    /* Margin: a dotted line all the way across, so it reads as a threshold. */
    int margin_y = TRACK_TOP + CONFIG_NINO_BELL_COLUMN;
    for (int x = 0; x < SCREEN_W; x += 2) {
        screen_plot(buf, x, margin_y, 16);
    }

    /*
     * The carriage is a bar with a chevron under it pointing the way it is
     * travelling: down while typing, flipping up the instant it slams back.
     */
    int y = TRACK_TOP + widget->carriage_pos;
    int dir = widget->slamming ? -1 : 1;

    screen_hline(buf, 1, SCREEN_W - 2, y, 16);

    for (int i = 1; i <= CARRIAGE_POINT; i++) {
        int half = CARRIAGE_POINT - i;
        screen_hline(buf, (SCREEN_W / 2) - half, (SCREEN_W / 2) + half, y + (i * dir), 16);
    }
}

/* ------------------------------------------------------------------ globe -- */

/* Latitudes and longitudes of the wire grid, as sine-table indices. */
static const int8_t globe_latitudes[3] = {-7, 0, 7};
static const int8_t globe_meridians[4] = {0, 8, 16, 24};

static void globe_point(int x, int y, int z, int sin_y, int cos_y, int sin_p, int cos_p, int *ox,
                        int *oy, int *oz) {
    int x1 = (x * cos_y + z * sin_y) >> 12;
    int z1 = (z * cos_y - x * sin_y) >> 12;

    *ox = x1;
    *oy = (y * cos_p - z1 * sin_p) >> 12;
    *oz = (y * sin_p + z1 * cos_p) >> 12;
}

static void draw_globe(uint8_t *buf, const struct zmk_widget_nino *widget) {
    const int r = CONFIG_NINO_BALL_RADIUS;
    /* Kicked upward by a carriage slam, settling back over a few frames. */
    const int cy = BALL_CY - widget->ball_recoil;
    const uint8_t ink = draw_ink;

    /*
     * The flash is handled first and always in the normal ink. Doing it after
     * the Caps Lock inversion below would draw the shockwave in the background
     * colour, leaving it invisible whenever Caps Lock happened to be on.
     *
     * Two frames filled solid, then a shockwave ring expanding out and fading.
     * The grid is skipped during the solid part so it reads as one bright
     * pulse rather than a busy one.
     */
    if (widget->globe_flash > GLOBE_FLASH_TOTAL - 2) {
        screen_disc(buf, BALL_CX, cy, r, 16);
        return;
    }

    if (widget->globe_flash > 0) {
        int t = (GLOBE_FLASH_TOTAL - 2) - widget->globe_flash + 1;
        int amp = 16 - (t * 2);
        if (amp > 0) {
            screen_circle(buf, BALL_CX, cy, r + (t * 2), (uint8_t)amp);
        }
    }

    /*
     * Caps Lock inverts the globe and nothing else: a filled disc with the
     * grid knocked back out of it in the background colour. Local, legible,
     * and far less violent than flipping the whole panel.
     */
    if (widget->caps) {
        screen_disc(buf, BALL_CX, cy, r, 16);
        draw_ink = (uint8_t)~ink;
    }

    /* Silhouette, so it always reads as a sphere whatever the grid is doing. */
    screen_circle(buf, BALL_CX, cy, r, 16);

    int yaw = (int)(widget->yaw >> 4);
    int pitch = (int)(widget->pitch >> 4);
    int sin_y = fx_sin(yaw);
    int cos_y = fx_cos(yaw);
    int sin_p = fx_sin(pitch);
    int cos_p = fx_cos(pitch);

    /* Model coordinates in 1/256 pixel, which keeps the rotation exact enough
     * that the grid does not wobble between frames. */
    int rm = r * 256;

    for (int i = 0; i < 3; i++) {
        int lat = globe_latitudes[i];
        int ring_y = (rm * fx_sin(lat)) >> 12;
        int ring_r = (rm * fx_cos(lat)) >> 12;

        for (int step = 0; step < 64; step += 2) {
            int x = (ring_r * fx_cos(step)) >> 12;
            int z = (ring_r * fx_sin(step)) >> 12;

            int ox, oy, oz;
            globe_point(x, ring_y, z, sin_y, cos_y, sin_p, cos_p, &ox, &oy, &oz);
            if (oz > 0) {
                continue; /* Back of the sphere: hidden, so rotation reads. */
            }
            screen_plot(buf, BALL_CX + (ox >> 8), cy + (oy >> 8), 16);
        }
    }

    for (int i = 0; i < 4; i++) {
        int lon = globe_meridians[i];
        int cos_lon = fx_cos(lon);
        int sin_lon = fx_sin(lon);

        for (int step = 0; step < 64; step += 2) {
            int ring = (rm * fx_cos(step)) >> 12;
            int x = (ring * cos_lon) >> 12;
            int z = (ring * sin_lon) >> 12;
            int y = (rm * fx_sin(step)) >> 12;

            int ox, oy, oz;
            globe_point(x, y, z, sin_y, cos_y, sin_p, cos_p, &ox, &oy, &oz);
            if (oz > 0) {
                continue;
            }
            screen_plot(buf, BALL_CX + (ox >> 8), cy + (oy >> 8), 16);
        }
    }

    draw_ink = ink;
}

/* ------------------------------------------------------------------- test -- */

/*
 * Geometry check: the panel edges, a square, and a circle inscribed in it. On
 * correct geometry the rectangle sits on the glass edges and the circle touches
 * the square at the midpoint of each side.
 */
static void draw_test_pattern(uint8_t *buf) {
    int side = MIN(SCREEN_W, SCREEN_H) - 6;
    int left = (SCREEN_W - side) / 2;
    int top = (SCREEN_H - side) / 2;

    screen_hline(buf, 0, SCREEN_W - 1, 0, 16);
    screen_hline(buf, 0, SCREEN_W - 1, SCREEN_H - 1, 16);
    screen_vline(buf, 0, 0, SCREEN_H - 1, 16);
    screen_vline(buf, SCREEN_W - 1, 0, SCREEN_H - 1, 16);

    screen_hline(buf, left, left + side, top, 16);
    screen_hline(buf, left, left + side, top + side, 16);
    screen_vline(buf, left, top, top + side, 16);
    screen_vline(buf, left + side, top, top + side, 16);

    screen_circle(buf, SCREEN_W / 2, SCREEN_H / 2, side / 2, 16);
}

/* ----------------------------------------------------------------- render -- */

/*
 * Flash the globe. extra queues further pulses once this one finishes, so the
 * margin bell (two) is distinguishable from a knob click (one). Deliberately
 * independent of the carriage: typing on through a ring must not cut it short.
 */
static void nino_globe_strike(struct zmk_widget_nino *widget, uint8_t extra) {
    widget->globe_flash = GLOBE_FLASH_TOTAL;
    widget->globe_flash_pending = extra;
}

static void nino_shake(struct zmk_widget_nino *widget, uint8_t frames, uint8_t mag) {
    widget->shake_frames = frames;
    widget->shake_mag = mag;
}

static void nino_return(struct zmk_widget_nino *widget);

static void nino_advance(struct zmk_widget_nino *widget) {
    if (widget->column < CONFIG_NINO_LINE_LENGTH) {
        widget->column++;
    }

    widget->slamming = false;
    widget->carriage_pos = (int16_t)widget->column;

    /* A nudge each time the carriage passes one of the long ruler ticks. */
    if (widget->column > 0 && (widget->column % RULER_MAJOR) == 0) {
        nino_shake(widget, TICK_SHAKE_FRAMES, TICK_SHAKE_MAG);
    }

    /* Ring once per line, on the way past the margin. Two pulses, so the
     * warning is distinguishable from the single pulse of a knob click. */
    if (!widget->bell_rung && widget->column >= CONFIG_NINO_BELL_COLUMN) {
        widget->bell_rung = true;
        nino_globe_strike(widget, 1);
    }

    /*
     * A full line returns itself. carriage_pos is deliberately left where it
     * is so the bar slams back from the end of the track rather than jumping.
     */
    if (widget->column >= CONFIG_NINO_LINE_LENGTH) {
        nino_return(widget);
    }
}

static void nino_backspace(struct zmk_widget_nino *widget) {
    if (widget->column > 0) {
        widget->column--;
    }
    if (widget->column < CONFIG_NINO_BELL_COLUMN) {
        widget->bell_rung = false;
    }
    widget->slamming = false;
    widget->carriage_pos = (int16_t)widget->column;
}

static void nino_return(struct zmk_widget_nino *widget) {
    widget->column = 0;
    widget->bell_rung = false;
    widget->slamming = true;

    /* The slam owns the whole screen: a full-panel invert, a hard shake, and
     * the globe kicked upward by the impact. */
    widget->strike_timer = CONFIG_NINO_SLAM_FLASH_FRAMES;
    widget->ball_recoil = CONFIG_NINO_BALL_RECOIL;
    nino_shake(widget, CONFIG_NINO_SHAKE_FRAMES, CONFIG_NINO_SHAKE_MAG);
}

static void nino_render(lv_timer_t *timer) {
    struct zmk_widget_nino *widget = (struct zmk_widget_nino *)timer->user_data;
    uint8_t *buf = (uint8_t *)widget->cbuf;

    struct nino_input in;
    bool had_input = false;
    while (k_msgq_get(&nino_input_q, &in, K_NO_WAIT) == 0) {
        had_input = true;
        switch (in.kind) {
        case NINO_INPUT_CHAR:
            nino_advance(widget);
            break;
        case NINO_INPUT_BACKSPACE:
            nino_backspace(widget);
            break;
        case NINO_INPUT_RETURN:
            nino_return(widget);
            break;
        case NINO_INPUT_KNOB_TURN:
            if (zmk_keymap_highest_layer_active() > 0) {
                widget->pitch_target += in.value * CONFIG_NINO_BALL_STEP;
            } else {
                widget->yaw_target += in.value * CONFIG_NINO_BALL_STEP;
            }
            break;
        case NINO_INPUT_KNOB_CLICK:
            nino_globe_strike(widget, 0);
            break;
        default:
            break;
        }
    }

    /* Any input dismisses the splash early. */
    if (had_input && widget->splash_frames > 0) {
        widget->splash_frames = 0;
        widget->dirty = true;
    }

    bool caps = atomic_get(&caps_lock_on) != 0;
    if (caps != widget->caps) {
        widget->caps = caps;
        widget->dirty = true;
    }

    bool globe_moving = (widget->yaw != widget->yaw_target) ||
                        (widget->pitch != widget->pitch_target) || (widget->globe_flash > 0) ||
                        (widget->globe_flash_pending > 0) || (widget->ball_recoil != 0);
    bool busy = had_input || widget->dirty || widget->test_pattern || globe_moving ||
                widget->slamming || widget->splash_frames > 0 || widget->shake_frames > 0 ||
                widget->strike_timer > 0;

    if (!busy) {
        /* Nothing moving — leave the panel alone rather than resending it. */
        return;
    }

    /*
     * Caps Lock inverts the whole panel. NINO_INVERT flips the convention if
     * the panel reads the other way round than expected.
     */
    bool field_is_black = true;
#if IS_ENABLED(CONFIG_NINO_INVERT)
    field_is_black = !field_is_black;
#endif

    /*
     * The slam flips every pixel: the cheapest whole-screen effect there is,
     * since it is only the polarity the frame is cleared to.
     *
     * These three effects each latch a "was running this frame" flag before
     * they tick down. Without that, the last inverted frame decrements the
     * timer to zero, dirty goes false, the next frame is skipped as idle, and
     * the panel is left stuck inverted — which is exactly the white-screen
     * hang. Owing one more frame after each effect ends fixes it for all of
     * them, and it is independent of whatever the carriage is doing.
     */
    bool was_striking = (widget->strike_timer > 0);
    if (was_striking) {
        field_is_black = !field_is_black;
        widget->strike_timer--;
    }

    /* One byte per pixel at LV_COLOR_DEPTH 1; any non-zero byte reads as set. */
    uint8_t background = field_is_black ? 0x00 : 0xFF;
    memset(buf, background, sizeof(widget->cbuf));
    draw_ink = (uint8_t)~background;

    /* The shake is a whole-screen offset, applied inside screen_plot. */
    bool was_shaking = (widget->shake_frames > 0);
    if (was_shaking) {
        int mag = widget->shake_mag;
        shake_dx = (widget->shake_frames & 1) ? mag : -mag;
        shake_dy = (widget->shake_frames & 1) ? -mag : mag;
        widget->shake_frames--;
    } else {
        shake_dx = 0;
        shake_dy = 0;
    }

    if (widget->test_pattern) {
        draw_test_pattern(buf);
        lv_obj_invalidate(widget->obj);
        widget->dirty = false;
        return;
    }

    if (widget->splash_frames > 0) {
        int letters = splash_letters_shown(widget->splash_frames, widget->splash_total);
        /* No cursor while backspacing — it only belongs where typing is still
         * expected to happen. */
        bool caret = (widget->splash_frames > SPLASH_OUT) && (((widget->splash_frames / 4) & 1) != 0);

        draw_splash(buf, letters, caret);
        widget->splash_frames--;

        lv_obj_invalidate(widget->obj);
        /* Always owe another frame: the splash is animating every frame, and
         * the one after it ends has to draw the real screen. */
        widget->dirty = true;
        return;
    }

    /* The slam: the carriage flies back to zero far faster than it advanced. */
    if (widget->slamming) {
        widget->carriage_pos -= CONFIG_NINO_SLAM_SPEED;
        if (widget->carriage_pos <= 0) {
            widget->carriage_pos = 0;
            widget->slamming = false;
        }
    }

    draw_carriage(buf, widget);

    /* Drawn before the decrement, so the flash gets its full two solid frames. */
    draw_globe(buf, widget);

    bool was_flashing = (widget->globe_flash > 0);
    if (was_flashing) {
        widget->globe_flash--;
        if (widget->globe_flash == 0 && widget->globe_flash_pending > 0) {
            widget->globe_flash_pending--;
            widget->globe_flash = GLOBE_FLASH_TOTAL;
        }
    }

    /* The globe settles back down from the slam's kick. */
    bool was_recoiling = (widget->ball_recoil != 0);
    widget->ball_recoil = (int8_t)(widget->ball_recoil / 2);

    lv_obj_invalidate(widget->obj);

    widget->yaw = ease(widget->yaw, widget->yaw_target);
    widget->pitch = ease(widget->pitch, widget->pitch_target);

    globe_moving = (widget->yaw != widget->yaw_target) ||
                   (widget->pitch != widget->pitch_target) || (widget->globe_flash > 0) ||
                   (widget->globe_flash_pending > 0);

    /* One more frame is owed once everything settles, to draw the resting state. */
    widget->dirty = globe_moving || widget->slamming || was_striking || was_shaking ||
                    was_flashing || was_recoiling;
}

/* -------------------------------------------------------------- listeners -- */

static bool is_printable(uint32_t keycode) {
    if (keycode >= 0x04 && keycode <= 0x27) {
        return true; /* a-z, 1-0 */
    }
    if (keycode == KC_SPACE) {
        return true;
    }
    if (keycode >= 0x2D && keycode <= 0x38) {
        return true; /* punctuation */
    }
    return false;
}

static void nino_post(uint8_t kind, int8_t value) {
    struct nino_input in = {.value = value, .kind = kind};
    /* Dropping one is better than blocking an event thread. */
    k_msgq_put(&nino_input_q, &in, K_NO_WAIT);
}

static int nino_keycode_listener(const zmk_event_t *eh) {
    const struct zmk_keycode_state_changed *ev = as_zmk_keycode_state_changed(eh);
    if (ev == NULL || !ev->state || ev->usage_page != HID_USAGE_KEY) {
        return ZMK_EV_EVENT_BUBBLE;
    }

    if (ev->keycode == KC_ENTER) {
        nino_post(NINO_INPUT_RETURN, 0);
    } else if (ev->keycode == KC_BACKSPACE) {
        nino_post(NINO_INPUT_BACKSPACE, 0);
    } else if (is_printable(ev->keycode)) {
        nino_post(NINO_INPUT_CHAR, 0);
    }

    return ZMK_EV_EVENT_BUBBLE;
}

ZMK_LISTENER(nino_keycode, nino_keycode_listener);
ZMK_SUBSCRIPTION(nino_keycode, zmk_keycode_state_changed);

static int nino_position_listener(const zmk_event_t *eh) {
    const struct zmk_position_state_changed *ev = as_zmk_position_state_changed(eh);
    if (ev == NULL || !ev->state) {
        return ZMK_EV_EVENT_BUBBLE;
    }

    /* The encoder's push-switch is an ordinary matrix key. */
    if (ev->position == CONFIG_NINO_KNOB_POSITION) {
        nino_post(NINO_INPUT_KNOB_CLICK, 0);
    }

    return ZMK_EV_EVENT_BUBBLE;
}

ZMK_LISTENER(nino_position, nino_position_listener);
ZMK_SUBSCRIPTION(nino_position, zmk_position_state_changed);

static int nino_indicators_listener(const zmk_event_t *eh) {
    const struct zmk_hid_indicators_changed *ev = as_zmk_hid_indicators_changed(eh);
    if (ev != NULL) {
        /* HID keyboard LED report: bit 0 Num, bit 1 Caps, bit 2 Scroll. */
        atomic_set(&caps_lock_on, (ev->indicators & BIT(1)) ? 1 : 0);
    }
    return ZMK_EV_EVENT_BUBBLE;
}

ZMK_LISTENER(nino_indicators, nino_indicators_listener);
ZMK_SUBSCRIPTION(nino_indicators, zmk_hid_indicators_changed);

static int64_t last_knob_uptime;

static int nino_sensor_listener(const zmk_event_t *eh) {
    const struct zmk_sensor_event *ev = as_zmk_sensor_event(eh);
    if (ev == NULL || ev->channel_data_size < 1) {
        return ZMK_EV_EVENT_BUBBLE;
    }

    /*
     * Only the sign matters. How many events arrive per detent varies with the
     * encoder's resolution, so rather than trying to reconstruct detents this
     * rate-limits to one nudge per NINO_BALL_MIN_MS.
     */
    struct sensor_value value = ev->channel_data[0].value;
    int delta = (value.val1 != 0) ? value.val1 : value.val2;
    if (delta == 0) {
        return ZMK_EV_EVENT_BUBBLE;
    }

    int64_t now = k_uptime_get();
    if (now - last_knob_uptime < CONFIG_NINO_BALL_MIN_MS) {
        return ZMK_EV_EVENT_BUBBLE;
    }
    last_knob_uptime = now;

    int8_t direction = (delta > 0) ? 1 : -1;
#if IS_ENABLED(CONFIG_NINO_BALL_INVERT)
    direction = -direction;
#endif

    nino_post(NINO_INPUT_KNOB_TURN, direction);

    return ZMK_EV_EVENT_BUBBLE;
}

ZMK_LISTENER(nino_sensor, nino_sensor_listener);
ZMK_SUBSCRIPTION(nino_sensor, zmk_sensor_event);

/* ------------------------------------------------------------------- init -- */

int zmk_widget_nino_init(struct zmk_widget_nino *widget, lv_obj_t *parent) {
    widget->obj = lv_canvas_create(parent);
    lv_canvas_set_buffer(widget->obj, widget->cbuf, FB_W, FB_H, LV_IMG_CF_TRUE_COLOR);

    widget->column = 0;
    widget->carriage_pos = 0;
    widget->slamming = false;
    widget->bell_rung = false;
    widget->shake_frames = 0;
    widget->shake_mag = 0;
    widget->strike_timer = 0;
    widget->globe_flash = 0;
    widget->globe_flash_pending = 0;
    widget->ball_recoil = 0;
    widget->caps = false;

    widget->dirty = true;
    widget->test_pattern = IS_ENABLED(CONFIG_NINO_TEST_PATTERN);
    /* Typing in and backspacing out are fixed costs; whatever is left over is
     * hold time. Clamped so a short SPLASH_MS truncates the hold rather than
     * cutting the animation itself in half. */
    uint16_t frames = CONFIG_NINO_SPLASH_MS / CONFIG_NINO_FRAME_MS;
    if (frames < SPLASH_IN + SPLASH_OUT) {
        frames = SPLASH_IN + SPLASH_OUT;
    }
    widget->splash_frames = frames;
    widget->splash_total = frames;

    /* Start off-axis so the globe reads as a sphere from the first frame. */
    widget->yaw = widget->yaw_target = 5 * 16;
    widget->pitch = widget->pitch_target = 3 * 16;

    widget->timer = lv_timer_create(nino_render, CONFIG_NINO_FRAME_MS, widget);

    return 0;
}

lv_obj_t *zmk_widget_nino_obj(struct zmk_widget_nino *widget) { return widget->obj; }
