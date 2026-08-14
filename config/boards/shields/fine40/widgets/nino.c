/*
 * NINO — Notetaking Input, Networkless Output
 *
 * The panel is mounted a quarter turn over, so everything here is drawn in
 * "screen space": sx runs left to right across the narrow 32-pixel dimension,
 * sy runs top to bottom down the long 128-pixel one. screen_plot() is the only
 * place that knows about the rotation, and it is also where the carriage-return
 * shake offset is applied, so a shake costs nothing but two added integers.
 *
 * Top of screen    the carriage: a bar working its way down a filling track
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
#include "nino_font.h"

#if IS_ENABLED(CONFIG_NINO_NOTES)
#include "../notes/nino_destow.h"
#include "../notes/nino_store.h"
#endif

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
#define KC_LSHIFT 0xE1
#define KC_RSHIFT 0xE5

/* Layout, in screen space. The track starts near the top: there is nothing
 * above it to leave room for. */
#define TRACK_TOP 8

/*
 * Height of the chevron under the carriage bar. It sharpens with typing speed,
 * from the resting height up to the hot one — see the heat comment below.
 */
#define CARRIAGE_POINT 4
#define CARRIAGE_POINT_HOT 7

/*
 * A cartridge holds more characters than the track has pixels, so travel is
 * scaled rather than being one pixel per character. Everything the carriage
 * does is expressed in characters; carriage_px() is the only conversion.
 */
#define TRACK_LEN CONFIG_NINO_TRACK_LENGTH
#define CART_CHARS CONFIG_NINO_CARTRIDGE_CHARS

#define BALL_CX (SCREEN_W / 2)
#define BALL_CY (SCREEN_H - 22)

/*
 * The globe's own flash, fired by a knob click: two solid frames and then ten
 * of shockwave. Sized so the last ring is the last frame — any longer and the
 * timer keeps the panel awake for frames that draw nothing.
 */
#define GLOBE_FLASH_TOTAL 12

/*
 * The distance mark: every so many detents the globe rings once and the panel
 * ticks. A sense of how far has been turned, which a freely spinning ball
 * cannot give on its own.
 *
 * Three frames of a solid ring stepping outward — solid rather than fading,
 * because the ring is the machine and mechanism does not dither. The tick jolts
 * along the track, not across it, so it can never be mistaken for a slam.
 */
#define ORB_RING_FRAMES 3
#define ORB_RING_STEP 2
#define ORB_TICK_FRAMES 2
#define ORB_TICK_MAG 1

/* Backspace flips the chevron for this long, so erasing does not feel like
 * typing played backwards. */
#define ERASE_FRAMES 3

/*
 * THE MATERIALS
 *
 * Three, and everything drawn on this panel is exactly one of them.
 *
 *   MECHANISM — solid, and perfectly still. The carriage bar, the ruler, the
 *     end stop, the wrap sweep, the store gauge. Machined parts do not have
 *     texture and they do not drift.
 *
 *   INK — dithered, and slowly flowing. Only the track fill. The dither matrix
 *     is sampled through an offset that creeps a cell at a time, so the texture
 *     migrates across the paper instead of sitting on it.
 *
 *     The distinction that makes this work rather than look broken: incoherent
 *     per-pixel toggling reads as flicker, which is a fault. Coherent motion of
 *     a whole pattern reads as flow, which is a material. The old temporal
 *     dither was the former and had to go; this is the latter, and it is the
 *     same mechanism used honestly.
 *
 *   DRIED INK — the pass pips. Ink that has stopped moving: the same dither
 *     family as the fill, at the top of the ladder, but fixed in place. A
 *     record of ink rather than ink, which is exactly what a pass count is.
 *
 * On a 1-bit panel contrast is the entire budget. Anything added later picks
 * one of the three.
 */

/*
 * Ink drift. One cell of the matrix every INK_DRIFT_FRAMES, diagonally, so the
 * texture crawls rather than scans. Slow on purpose — fast enough to be alive
 * when you look at it, slow enough that it never competes with the carriage.
 */
#define INK_DRIFT_FRAMES 9

/*
 * The ruler is graduated in characters, and passing a graduation strikes it:
 * the mark reaches further into the track for a few frames. Same furniture,
 * momentarily emphasised — not a second mark drawn beside it. Brightness is
 * not available for this, because mechanism is always solid.
 */
#define TICK_FLASH_FRAMES 4
#define TICK_DASH 3
#define TICK_DASH_STRUCK 8

/* Dried ink: the top of the ladder, which puts 2-3 lit cells in every 2x2 —
 * always visible however a pip lands on the matrix, and still textured. */
#define PIP_AMP AMP_LEVEL(10)
#define PIP_SIZE 2

/*
 * The cartridge reaching its end: a rule sweeping up the track from the stop to
 * the top, which is the platen taking the page back. Not an event in the sense
 * the slam is — no invert, no shake, nothing that demands attention — but a
 * whole-track movement, which is impossible to miss and over in a quarter of a
 * second.
 */
#define WRAP_SWEEP_FRAMES 7

/*
 * One pip per completed pass, in rows under the track. The fill ladder runs out
 * of density after eight passes; the pips do not, so the panel keeps saying how
 * much has been written long after the texture has stopped changing.
 */
#define PASS_PIP_STRIDE 3
#define PASS_PIPS_PER_ROW 10
#define PASS_PIP_ROWS 2
#define PASS_MAX (PASS_PIPS_PER_ROW * PASS_PIP_ROWS)

/*
 * Typing heat: every character adds HEAT_PER_KEY, and it drains proportionally
 * so the value settles at a level that tracks your cadence rather than spiking
 * on each keystroke. The drain has to be proportional — a flat one per frame
 * turns this into a knife edge, where every speed below some threshold rests
 * at the bottom and everything above it pins to the top, with nothing between.
 *
 * As tuned, sustained typing runs the chevron from its resting height at 20wpm
 * up to one short of hot at 80. The last pixel is reserved for bursts: two keys
 * landing inside one frame, which is what a roll looks like from here.
 */
#define HEAT_PER_KEY 12
#define HEAT_MAX 34
#define HEAT_DRAIN(h) (1 + (h) / 16)

/* Every key on the board at once, and then some. Only a clamp. */
#define NINO_MAX_HELD 12

/* Idle threshold, converted from milliseconds to frames once here. */
#define NINO_IDLE_FRAMES (CONFIG_NINO_IDLE_MS / CONFIG_NINO_FRAME_MS)

/*
 * 4x4 ordered dither, purely spatial. The matrix supplies thresholds 0..30 in
 * steps of two against an amplitude of 0..AMP_SOLID, so a pixel lights when its
 * threshold is under the amplitude and the number of lit cells in each 4x4 is
 * exactly amp/2. That gives sixteen honest levels, every one of them a fixed
 * pattern that does not move between frames.
 *
 * There was a temporal dither here — half a level added on alternate frames to
 * squeeze a seventeenth grey out of the matrix. It is gone. Flicker fusion at
 * 25fps is marginal at the best of times, and on a panel this small a texture
 * that shimmers reads as a fault rather than as a shade.
 */
#define AMP_SOLID 32

/* Ten fill levels, one to ten cells of every sixteen. Ten is where the ladder
 * has to stop: at eleven the fill is close enough to the solid bar that the
 * two stop reading as different materials. */
#define AMP_LEVEL(n) ((n) * 2)
#define NINO_FILL_LEVELS 10

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

/*
 * How many keys are physically down. Tracked from the matrix rather than from
 * keycodes because presses and releases there always pair up — a keycode can
 * come back different from the one that went down if the layer moved under it,
 * which would leave this stuck above zero and the carriage stuck depressed.
 */
static atomic_t keys_held = ATOMIC_INIT(0);

/*
 * The destow chord: Space and Shift held, then the knob clicked. Tracked from
 * keycodes rather than positions so it follows the keys wherever the keymap
 * puts them, and because both of these are things the user is aware of holding.
 */
static atomic_t space_held = ATOMIC_INIT(0);
static atomic_t shift_held = ATOMIC_INIT(0);

static uint8_t draw_ink;
static int8_t shake_dx;
static int8_t shake_dy;

/*
 * Where the dither matrix is sampled from. Zero for everything except the
 * fill, which sets it to the drifting phase and puts it back afterwards —
 * mechanism must never inherit ink's movement.
 */
static uint8_t dither_ox;
static uint8_t dither_oy;

/* Advances one cell every INK_DRIFT_FRAMES, and only on drawn frames, so a
 * resting panel is genuinely still. */
static uint16_t ink_clock;

/*
 * Vertical clip, in screen rows. Only the sheet in the machine is ever clipped
 * — the stops and the globe are the machine itself and are drawn whole.
 */
static int16_t clip_lo;
static int16_t clip_hi = SCREEN_H;

static inline void clip_rows(int lo, int hi) {
    clip_lo = (int16_t)lo;
    clip_hi = (int16_t)hi;
}

static inline void clip_none(void) {
    clip_lo = 0;
    clip_hi = SCREEN_H;
}

/* ---------------------------------------------------------------- pixels -- */

static inline void put_fb(uint8_t *buf, int px, int py, uint8_t amp) {
    if (px < 0 || px >= FB_W || py < 0 || py >= FB_H) {
        return;
    }
    if ((uint8_t)(bayer4[(py + dither_oy) & 3][(px + dither_ox) & 3] * 2) >= amp) {
        return;
    }
    buf[py * FB_W + px] = draw_ink;
}

/* Bracket a run of ink drawing. Nested use would be wrong, so it is not. */
static inline void ink_begin(void) {
    uint8_t step = (uint8_t)(ink_clock / INK_DRIFT_FRAMES);
    dither_ox = step & 3;
    dither_oy = (step >> 1) & 3; /* half the rate on the other axis: diagonal */
}

static inline void ink_end(void) {
    dither_ox = 0;
    dither_oy = 0;
}

/*
 * The single place that knows the panel is rotated, and the single place the
 * carriage-return shake is applied. Both rotations are true quarter turns
 * rather than a transpose, because a transpose would mirror the splash text.
 */
static inline void screen_plot(uint8_t *buf, int sx, int sy, uint8_t amp) {
    /* Clipped before the shake, so a screen change wiping through the track
     * cuts along a fixed line rather than one that jitters with the panel. */
    if (sy < clip_lo || sy >= clip_hi) {
        return;
    }

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

static int text_width(const char *s) {
    int n = 0;
    for (; *s != '\0'; s++) {
        n++;
    }
    return n * NINO_GLYPH_ADVANCE - (n > 0 ? 1 : 0);
}

static void draw_text(uint8_t *buf, int x, int y, const char *s, uint8_t amp) {
    for (; *s != '\0'; s++, x += NINO_GLYPH_ADVANCE) {
        const uint8_t *rows = nino_glyphs[nino_glyph_index(*s)];

        for (int row = 0; row < NINO_GLYPH_H; row++) {
            for (int col = 0; col < NINO_GLYPH_W; col++) {
                if (rows[row] & (1 << (NINO_GLYPH_W - 1 - col))) {
                    screen_plot(buf, x + col, y + row, amp);
                }
            }
        }
    }
}

static void draw_text_centred(uint8_t *buf, int y, const char *s, uint8_t amp) {
    draw_text(buf, (SCREEN_W - text_width(s)) / 2, y, s, amp);
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

/*
 * Spring toward the target rather than gliding to it, so the globe carries
 * some mass: a detent lands with a small overshoot and settles, and a fast spin
 * carries well past before it comes back.
 *
 * Two details keep this from ringing forever in integer arithmetic. The
 * position always moves at least one unit toward the target, because damping
 * truncates a velocity of 1 straight to 0 and the globe would otherwise stall
 * a few units short with the panel stuck awake redrawing it. And anything
 * inside SPRING_SNAP is finished off outright — at 1024 units to a turn that
 * is under a degree and a half, far below what the wireframe can show.
 */
#define SPRING_K 5
#define SPRING_DAMP_NUM 2
#define SPRING_DAMP_DEN 3
#define SPRING_SNAP 4

/*
 * One full turn, in the same units. Subtracting it from an angle and its
 * target together leaves the difference — and so the spring, and the rendered
 * orientation — bit-for-bit identical, which is what makes it safe to do at
 * any moment rather than only when the two have converged.
 *
 * Needed because the idle spin drives yaw_target up forever. It would take
 * well over a year of accumulated idle time to overflow an int32_t, but that
 * is signed overflow rather than a wrap, so it is undefined behaviour rather
 * than a glitch, and this costs two comparisons a frame to rule out.
 */
#define SPRING_TURN 1024

static void spring_wrap(int32_t *current, int32_t *target) {
    while (*current > SPRING_TURN && *target > SPRING_TURN) {
        *current -= SPRING_TURN;
        *target -= SPRING_TURN;
    }
    while (*current < -SPRING_TURN && *target < -SPRING_TURN) {
        *current += SPRING_TURN;
        *target += SPRING_TURN;
    }
}

static void spring(int32_t *current, int32_t *velocity, int32_t target) {
    int32_t delta = target - *current;

    if (delta > -SPRING_SNAP && delta < SPRING_SNAP && *velocity > -SPRING_SNAP &&
        *velocity < SPRING_SNAP) {
        *current = target;
        *velocity = 0;
        return;
    }

    *velocity += delta / SPRING_K;
    *velocity = (*velocity * SPRING_DAMP_NUM) / SPRING_DAMP_DEN;

    int32_t move = *velocity;
    if (move == 0) {
        move = (delta > 0) ? 1 : -1;
    }
    *current += move;
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
 * The name is struck onto the paper rather than printed into it, in four
 * phases: the platen seats itself, each letter is hammered on in turn, the
 * finished word sits under a blinking caret, and then the machine throws the
 * sheet clear with the same carriage return it uses for everything else.
 *
 * Nothing here fades or slides at a constant rate. Every movement either
 * overshoots and settles or accelerates away, because a constant rate is the
 * one thing a mechanism never does.
 */
#define SPLASH_FEED 8   /* platen seating */
#define SPLASH_REVEAL 20 /* the pass that lays the name down */
#define SPLASH_RETURN 7 /* sheet thrown clear */
#define SPLASH_FIXED (SPLASH_FEED + SPLASH_REVEAL + SPLASH_RETURN)

/*
 * The platen dropping into position: down hard, past the stop, and back. The
 * numbers are hand-set rather than computed — this is eight frames of travel,
 * and a curve that reads right matters more here than one that is derivable.
 */
static const int8_t splash_feed_offset[SPLASH_FEED] = {-13, -9, -6, -3, -1, 2, 3, 1};

/*
 * And the sheet being ripped out at the end: away, accelerating, gone. The
 * last entry has to carry the whole block clear of the glass, bottom rule
 * included — stopping short leaves a stray line and a sliver of the last
 * letter on screen at the moment the carriage screen takes over.
 */
static const int8_t splash_eject_offset[SPLASH_RETURN] = {0, -8, -20, -38, -62, -92, -127};

/*
 * No two strikes on a real machine ink quite alike. A couple of levels off
 * solid is a scattering of missing pixels — not obviously wrong, just not
 * mechanically identical, which is the whole point.
 */
static const uint8_t splash_letter_ink[4] = {AMP_SOLID, AMP_SOLID - 3, AMP_SOLID - 1,
                                             AMP_SOLID - 2};

/* Frames since the splash began. */
static inline uint16_t splash_elapsed(const struct zmk_widget_nino *widget) {
    return widget->splash_total - widget->splash_frames;
}

/* Rows clipped to a boundary, which is how the reveal pass eats into a glyph. */
static void screen_rect_clipped(uint8_t *buf, int x, int y, int w, int h, uint8_t amp, int max_y) {
    for (int j = 0; j < h; j++) {
        if (y + j >= max_y) {
            return;
        }
        screen_hline(buf, x, x + w - 1, y + j, amp);
    }
}

static void draw_splash(uint8_t *buf, const struct zmk_widget_nino *widget) {
    uint16_t elapsed = splash_elapsed(widget);
    uint16_t remaining = widget->splash_frames;

    int top = (SCREEN_H - SPLASH_BLOCK) / 2;
    int left = (SCREEN_W - SPLASH_CELL) / 2;

    /* The paper moves as one: seating at the start, thrown clear at the end. */
    if (elapsed < SPLASH_FEED) {
        top += splash_feed_offset[elapsed];
    } else if (remaining <= SPLASH_RETURN) {
        /* remaining is 1..SPLASH_RETURN here — the caller only draws while the
         * splash is still running — so the index cannot leave the table. */
        top += splash_eject_offset[SPLASH_RETURN - remaining];
    }

    /*
     * The name is laid down by a pass of the carriage, not hammered on letter
     * by letter — the same movement the track uses, so the boot screen and the
     * working screen speak the same language. A rule travels down the block; a
     * light texture follows it; the letters exist only where it has been.
     *
     * The span runs from the top rule to just past the bottom one, so the pass
     * clears the whole sheet rather than stopping on the last letter.
     */
    int span = SPLASH_BLOCK + 12;
    int sweep_top = top - 6;
    int boundary;

    if (elapsed < SPLASH_FEED) {
        boundary = sweep_top; /* paper in, nothing laid down yet */
    } else if (elapsed < SPLASH_FEED + SPLASH_REVEAL) {
        int t = (int)elapsed - SPLASH_FEED + 1;
        boundary = sweep_top + (t * span) / SPLASH_REVEAL;
    } else {
        boundary = sweep_top + span;
    }

    bool sweeping = (elapsed >= SPLASH_FEED) && (elapsed < SPLASH_FEED + SPLASH_REVEAL);

    /* Top rule always: the paper is in the machine, it is simply blank. */
    screen_hline(buf, 2, SCREEN_W - 3, top - 6, AMP_SOLID);

    /* The texture the pass leaves behind it, at the lightest of the eight
     * levels — enough to show the sheet has been written on, not enough to
     * compete with the letters sitting in it. */
    for (int y = sweep_top + 1; y < boundary && y < top + SPLASH_BLOCK + 5; y++) {
        screen_hline(buf, 2, SCREEN_W - 3, y, AMP_LEVEL(1));
    }

    /* The bottom rule is the stop, and only exists once the pass has reached
     * it — otherwise the sheet looks finished before it is. */
    if (!sweeping) {
        screen_hline(buf, 2, SCREEN_W - 3, top + SPLASH_BLOCK + 5, AMP_SOLID);
    }

    int letters = sweeping ? 4 : (elapsed < SPLASH_FEED ? 0 : 4);

    /*
     * The caret waits at the top of the next empty cell while there is one,
     * and parks in the margin under the finished word once there is not —
     * below the fourth letter it would otherwise collide with the bottom rule.
     */
    /* Starts lit, so the very first thing on the glass is the caret waiting on
     * blank paper rather than two bare rules for the length of a blink. */
    bool caret = ((elapsed / 10) & 1) == 0;
    if (caret && remaining > SPLASH_RETURN) {
        if (letters < 4) {
            screen_rect(buf, left, top + letters * (SPLASH_CELL + SPLASH_GAP), SPLASH_CELL, 2,
                        AMP_SOLID);
        } else {
            screen_rect(buf, left, top + SPLASH_BLOCK + 2, SPLASH_CELL, 2, AMP_SOLID);
        }
    }

    for (int letter = 0; letter < letters && letter < 4; letter++) {
        const uint8_t *rows = nino_letters[letter];
        int cell_top = top + letter * (SPLASH_CELL + SPLASH_GAP);
        uint8_t amp = splash_letter_ink[letter];

        for (int row = 0; row < 5; row++) {
            for (int col = 0; col < 5; col++) {
                /* Bit 4 is the leftmost column of the glyph. */
                if (!(rows[row] & (1 << (4 - col)))) {
                    continue;
                }
                screen_rect_clipped(buf, left + col * SPLASH_SCALE, cell_top + row * SPLASH_SCALE,
                                    SPLASH_SCALE, SPLASH_SCALE, amp, boundary);
            }
        }
    }

    /*
     * The carriage itself, last, so it rides over the letters it is uncovering
     * rather than being buried by them.
     */
    if (sweeping) {
        screen_hline(buf, 0, SCREEN_W - 1, boundary, AMP_SOLID);
    }
}

/* --------------------------------------------------------------- carriage -- */

/*
 * Density of the fill laid down by a given pass. Each cartridge lays a slightly
 * thicker texture than the one before, up to a ceiling kept below solid so the
 * fill never gets as bright as the carriage bar and the two keep reading as
 * separate things.
 */
static uint8_t fill_amp(uint8_t layer) {
    int amp = CONFIG_NINO_FILL_AMP_BASE + (int)layer * CONFIG_NINO_FILL_AMP_STEP;
    return (uint8_t)MIN(amp, CONFIG_NINO_FILL_AMP_MAX);
}

/* True once further passes would no longer be any denser. */
/*
 * Passes past the eighth no longer change the texture — fill_amp() clamps — so
 * nothing here needs to know where the ladder stops. The pip row is what keeps
 * counting from there.
 */

/* Where on the track a given character count puts the bar. */
static int carriage_px(uint16_t column) {
    return (int)(((uint32_t)column * TRACK_LEN) / CART_CHARS);
}

/* The chevron sharpens with typing speed. */
static int carriage_point(const struct zmk_widget_nino *widget) {
    return CARRIAGE_POINT + ((int)widget->heat * (CARRIAGE_POINT_HOT - CARRIAGE_POINT)) / HEAT_MAX;
}

/*
 * Where the bar actually sits, which is not quite where the character count
 * puts it: any key physically down presses the whole carriage a pixel into the
 * track, and the slam's rebound carries it briefly above the top of it.
 */
static int carriage_y(const struct zmk_widget_nino *widget) {
    return TRACK_TOP + widget->carriage_pos + (widget->pressed ? CONFIG_NINO_PRESS_DIP : 0);
}

/* The working sheet: what the carriage has laid down, and where it is. The
 * stops around it belong to the machine and are drawn by draw_chrome. */
static void draw_sheet_main(uint8_t *buf, const struct zmk_widget_nino *widget) {
    /*
     * The track carries the texture the cartridge has laid down. Dithered
     * rather than solid: on a 1-bit panel a solid block would be as bright as
     * the carriage bar itself and the two would read as one shape. A stipple
     * sits visibly below it instead, which is as close to a second grey as
     * this display gets.
     *
     * Two bands, split at the bar: below it is what the previous pass left,
     * above it is the current pass's heavier texture. Because a denser dither
     * is a strict superset of a thinner one within the same Bayer cell, the
     * second band can simply be drawn over the first — the boundary tracks the
     * carriage for free, and it drains on its own during a slam.
     *
     * Drawn first so the ruler and the bar land on top of it.
     */
    int track_end = TRACK_TOP + TRACK_LEN;
    /* Clamped because the rebound carries the bar above the top of the track,
     * and an unclamped boundary would spill the fill out over the edge. */
    int filled_to = CLAMP(TRACK_TOP + widget->carriage_pos, TRACK_TOP, track_end);

    uint8_t behind = (widget->fill_layer > 0) ? fill_amp(widget->fill_layer - 1) : 0;
    uint8_t ahead = fill_amp(widget->fill_layer);

    /*
     * A transfer takes the track over. The cartridge is already the thing that
     * says "this much of a fixed capacity is used", which is exactly what a
     * progress bar says, so nothing new is invented for it — the same bar moves
     * down the same track, driven by the host instead of by your hands.
     *
     * It is unmistakably not typing, though: the whole track fills at the top
     * of the ladder rather than at the current pass's density, and the ladder
     * of accumulated passes is not disturbed. The transfer borrows the track
     * and gives it back.
     */
    if (widget->transferring) {
        filled_to = TRACK_TOP + ((int)widget->transfer_progress * TRACK_LEN) / 255;
        behind = 0;
        ahead = fill_amp(NINO_FILL_LEVELS - 1);
    }

    /* Ink, and only ink, is sampled through the drifting phase. */
    ink_begin();
    if (behind > 0) {
        for (int y = filled_to; y < track_end; y++) {
            screen_hline(buf, 1, SCREEN_W - 2, y, behind);
        }
    }
    for (int y = TRACK_TOP; y < filled_to; y++) {
        screen_hline(buf, 1, SCREEN_W - 2, y, ahead);
    }
    ink_end();

    /*
     * The ruler is now graduated in characters, not pixels: a mark every
     * NINO_TICK_CHARS. That makes it the same scale the cadence mark counts on,
     * so passing a graduation is what lights it — the tick is not a separate
     * event drawn near the ruler, it IS the ruler mark, briefly struck.
     *
     * A layer being held drops every other graduation, halving the scale. The
     * track changes character with nothing added to the screen, which is the
     * whole reason the old layer bar came off.
     */
    bool layered = (zmk_keymap_highest_layer_active() > 0);
    int lit_mark = (widget->tick_flash > 0) ? widget->tick_mark : -1;

    for (int n = 0; n * CONFIG_NINO_TICK_CHARS <= CART_CHARS; n++) {
        if (layered && (n & 1)) {
            continue;
        }

        int y = TRACK_TOP + carriage_px((uint16_t)(n * CONFIG_NINO_TICK_CHARS));
        /* A struck graduation reaches further in; it does not change brightness,
         * because mechanism is always solid. */
        int len = (n == lit_mark) ? TICK_DASH_STRUCK : TICK_DASH;

        screen_hline(buf, 0, len - 1, y, AMP_SOLID);
        screen_hline(buf, SCREEN_W - len, SCREEN_W - 1, y, AMP_SOLID);
    }

    /*
     * Completed passes, one pip each, in rows below the stop. Counted rather
     * than shaded: density tops out after eight passes, and a countable mark
     * keeps working long after the texture has stopped telling you anything.
     */
    for (int p = 0; p < widget->fill_layer && p < PASS_MAX; p++) {
        int row = p / PASS_PIPS_PER_ROW;
        int col = p % PASS_PIPS_PER_ROW;
        int row_w = PASS_PIPS_PER_ROW * PASS_PIP_STRIDE - (PASS_PIP_STRIDE - PIP_SIZE);
        int x = ((SCREEN_W - row_w) / 2) + col * PASS_PIP_STRIDE;

        /* Dried ink: the fill's texture at the top of the ladder, and pointedly
         * NOT sampled through the drift — this is ink that has stopped. At this
         * density every 2x2 catches two or three cells, so a pip is always
         * visible however it lands on the matrix, and still reads as textured
         * rather than machined. */
        screen_rect(buf, x, track_end + 3 + row * (PIP_SIZE + 1), PIP_SIZE, PIP_SIZE, PIP_AMP);
    }

    /*
     * The page being taken back. Drawn over the fill and under the carriage, so
     * it sweeps through the texture rather than across the top of everything.
     */
    if (widget->wrap_sweep > 0) {
        int t = WRAP_SWEEP_FRAMES - widget->wrap_sweep;
        int sweep_y = track_end - ((t * TRACK_LEN) / WRAP_SWEEP_FRAMES);
        screen_hline(buf, 0, SCREEN_W - 1, sweep_y, AMP_SOLID);
        screen_hline(buf, 2, SCREEN_W - 3, sweep_y + 1, AMP_LEVEL(4));
    }

    /*
     * The carriage is a bar with a chevron under it pointing the way it is
     * travelling: down while typing, flipping up while it slams back and for a
     * few frames after a backspace. Its height tracks how fast you are going.
     */
    int y = carriage_y(widget);
    int dir = (widget->slamming || widget->erase_frames > 0) ? -1 : 1;
    int point = carriage_point(widget);

    screen_hline(buf, 1, SCREEN_W - 2, y, AMP_SOLID);

    for (int i = 1; i <= point; i++) {
        int half = point - i;
        screen_hline(buf, (SCREEN_W / 2) - half, (SCREEN_W / 2) + half, y + (i * dir), AMP_SOLID);
    }

}

/* ----------------------------------------------------------------- destow -- */

#if IS_ENABLED(CONFIG_NINO_NOTES)

/*
 * Destow is not a different screen. It is a different sheet in the same
 * cartridge: the stops and the globe stay exactly where they are, and only
 * what lies between them changes.
 *
 * The sheet divides into a gauge band and a list. The gauge is the store, so
 * it is ink and it drifts. The divider, the caret and the words are mechanism
 * and draw solid. Labels are cut to three letters not for space — there is
 * room for five — but so the column has one width, and the eye reads position
 * rather than length.
 */

#define SHEET_GAUGE_TOP (TRACK_TOP + 2)
#define SHEET_GAUGE_H 20
#define SHEET_DIVIDER (SHEET_GAUGE_TOP + SHEET_GAUGE_H + 3)
#define SHEET_ROW_TOP (SHEET_DIVIDER + 5)
#define SHEET_ROW 12
#define SHEET_TEXT_X 10

static const char *const destow_items[NINO_ITEM_COUNT] = {"SND", "WPE", "EXT"};

/*
 * Selection is ink, not brightness. The chosen row is a block of the densest
 * fill with its letters knocked back out of it in the background colour —
 * unprinted paper inside printed. Unselected rows are bare paper with the
 * letters printed on them, which is the ordinary way round.
 *
 * This is the only place a row's material changes, and it changes completely:
 * there is no half-lit intermediate for the eye to have to interpret.
 */
#define ROW_PAD_X 2
#define ROW_PAD_Y 2

static void draw_option(uint8_t *buf, int y, const char *label, bool selected) {
    if (!selected) {
        draw_text(buf, SHEET_TEXT_X, y, label, AMP_SOLID);
        return;
    }

    ink_begin();
    screen_rect(buf, ROW_PAD_X, y - ROW_PAD_Y, SCREEN_W - 2 * ROW_PAD_X,
                NINO_GLYPH_H + 2 * ROW_PAD_Y, AMP_LEVEL(10));
    ink_end();

    /* Knocked out: the letters are where the ink is not. */
    uint8_t ink = draw_ink;
    draw_ink = (uint8_t)~ink;
    draw_text(buf, SHEET_TEXT_X, y, label, AMP_SOLID);
    draw_ink = ink;
}

/* How full the store is, in the top band of the sheet. Fills downward, the
 * same direction the carriage lays ink down on the working sheet. */
static void draw_gauge_band(uint8_t *buf) {
    size_t capacity = nino_store_capacity();
    size_t used = capacity ? nino_store_used() : 0;
    int filled = capacity ? (int)(((uint64_t)used * SHEET_GAUGE_H) / capacity) : 0;

    /* Anything at all shows as one line, so a nearly empty store reads as
     * holding something rather than as a fault. */
    if (used > 0 && filled == 0) {
        filled = 1;
    }

    ink_begin();
    for (int i = 0; i < filled; i++) {
        screen_hline(buf, 1, SCREEN_W - 2, SHEET_GAUGE_TOP + i, AMP_LEVEL(6));
    }
    ink_end();

    screen_hline(buf, 2, SCREEN_W - 3, SHEET_DIVIDER, AMP_SOLID);
}

static void draw_rows(uint8_t *buf, const char *const *labels, int count, int selected) {
    for (int i = 0; i < count; i++) {
        draw_option(buf, SHEET_ROW_TOP + i * SHEET_ROW, labels[i], i == selected);
    }
}

/* A transfer, in or out: the cartridge itself is the bar. */
static void draw_sheet_transfer(uint8_t *buf, uint8_t progress, const char *label) {
    int filled = ((int)progress * TRACK_LEN) / 255;

    ink_begin();
    for (int i = 0; i < filled; i++) {
        screen_hline(buf, 1, SCREEN_W - 2, TRACK_TOP + i, AMP_LEVEL(8));
    }
    ink_end();

    /* The leading edge is the mechanism's position, so it is solid. */
    screen_hline(buf, 0, SCREEN_W - 1, TRACK_TOP + filled, AMP_SOLID);

    draw_text(buf, SHEET_TEXT_X, TRACK_TOP + TRACK_LEN - 12, label, AMP_SOLID);
}

static const char *const confirm_items[2] = {"NO", "YES"};

static void draw_sheet_destow(uint8_t *buf, uint8_t view) {
    draw_gauge_band(buf);

    switch (view) {
    case NINO_VIEW_DESTOW_MENU:
        draw_rows(buf, destow_items, NINO_ITEM_COUNT, nino_destow_selection());
        break;

    case NINO_VIEW_DESTOW_CONFIRM:
        draw_rows(buf, confirm_items, 2, nino_destow_selection());
        break;

    case NINO_VIEW_DESTOW_SENT:
        draw_text(buf, SHEET_TEXT_X, SHEET_ROW_TOP, "SNT", AMP_SOLID);
        break;

    default: /* EMPTY */
        draw_text(buf, SHEET_TEXT_X, SHEET_ROW_TOP, "NIL", AMP_SOLID);
        break;
    }
}

#endif /* CONFIG_NINO_NOTES */

/* ------------------------------------------------------------------ views -- */

/*
 * The sheet occupies the track and the pip rows under it. Everything below
 * that is the globe, which is machine and never wipes.
 */
#define SHEET_TOP TRACK_TOP
#define SHEET_BOTTOM (TRACK_TOP + TRACK_LEN + 12)

/*
 * The reveal travels slower than the slam that precedes it. The slam is the
 * machine discarding something and wants to feel violent; the reveal is the
 * machine presenting something and has to be readable as it arrives.
 */
#define REVEAL_SPEED 7

static uint8_t current_view(void) {
#if IS_ENABLED(CONFIG_NINO_NOTES)
    if (nino_store_busy()) {
        return NINO_VIEW_XFER_IN;
    }
    if (nino_store_reading()) {
        return NINO_VIEW_XFER_OUT;
    }

    switch (nino_destow_screen()) {
    case NINO_DESTOW_MENU:
        return NINO_VIEW_DESTOW_MENU;
    case NINO_DESTOW_CONFIRM:
        return NINO_VIEW_DESTOW_CONFIRM;
    case NINO_DESTOW_SENT:
        return NINO_VIEW_DESTOW_SENT;
    case NINO_DESTOW_EMPTY:
        return NINO_VIEW_DESTOW_EMPTY;
    default:
        break;
    }
#endif
    return NINO_VIEW_MAIN;
}

/* ------------------------------------------------------------------ globe -- */

/* Latitudes and longitudes of the wire grid, as sine-table indices. */
static const int8_t globe_latitudes[3] = {-7, 0, 7};
static const int8_t globe_meridians[4] = {0, 8, 16, 24};

#if IS_ENABLED(CONFIG_NINO_NOTES)
/*
 * How full the note store is, as a ring of pips around the globe. This is the
 * one piece of state on the keyboard worth acting on — it decides whether a
 * week's writing will fit — and until now it appeared nowhere at all, while the
 * globe sat there being decorative. A gauge is part of the machine, so the lit
 * pips are solid; the unlit ones are the scale they are read against, and stay
 * at the faintest level on the ladder.
 */
static void draw_store_gauge(uint8_t *buf, int cy) {
    size_t capacity = nino_store_capacity();
    if (capacity == 0) {
        return;
    }

    int radius = CONFIG_NINO_BALL_RADIUS + 3;
    int lit = (int)(((uint64_t)nino_store_used() * CONFIG_NINO_GAUGE_PIPS) / capacity);

    for (int i = 0; i < CONFIG_NINO_GAUGE_PIPS; i++) {
        /* Anticlockwise from the top, so filling reads the way a dial does. */
        int angle = (i * 64) / CONFIG_NINO_GAUGE_PIPS;
        int px = (radius * fx_sin(angle)) >> 12;
        int py = -((radius * fx_cos(angle)) >> 12);

        screen_plot(buf, BALL_CX + px, cy + py, (i < lit) ? AMP_SOLID : AMP_LEVEL(1));
    }
}
#endif

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
        screen_disc(buf, BALL_CX, cy, r, AMP_SOLID);
        return;
    }

    /*
     * The shockwave expands a pixel a frame, not two. At two it outruns its
     * own fade and the last rings are still faintly visible at a radius that
     * reaches up past the end of the carriage track, which reads as a stray
     * arc crossing the ruler rather than as a ring around the globe. At one
     * the ring dies at radius 22, comfortably clear of everything above it.
     */
    if (widget->globe_flash > 0) {
        int t = (GLOBE_FLASH_TOTAL - 2) - widget->globe_flash + 1;
        int amp = AMP_SOLID - (t * 3);
        if (amp > 0) {
            screen_circle(buf, BALL_CX, cy, r + t, (uint8_t)amp);
        }
    }

    /*
     * Caps Lock inverts the globe and nothing else: a filled disc with the
     * grid knocked back out of it in the background colour. Local, legible,
     * and far less violent than flipping the whole panel.
     */
    if (widget->caps) {
        screen_disc(buf, BALL_CX, cy, r, AMP_SOLID);
        draw_ink = (uint8_t)~ink;
    }

    /* Silhouette, so it always reads as a sphere whatever the grid is doing. */
    screen_circle(buf, BALL_CX, cy, r, AMP_SOLID);

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
            screen_plot(buf, BALL_CX + (ox >> 8), cy + (oy >> 8), AMP_SOLID);
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
            screen_plot(buf, BALL_CX + (ox >> 8), cy + (oy >> 8), AMP_SOLID);
        }
    }

    draw_ink = ink;

    /*
     * The distance mark, outside the Caps Lock inversion for the same reason
     * the gauge is: it is the machine speaking, not part of what the globe is
     * showing.
     */
    if (widget->orb_ring > 0) {
        int t = ORB_RING_FRAMES - widget->orb_ring;
        screen_circle(buf, BALL_CX, cy, r + 2 + (t * ORB_RING_STEP), AMP_SOLID);
    }

#if IS_ENABLED(CONFIG_NINO_NOTES)
    /* Outside the Caps Lock inversion: the gauge is an instrument, and an
     * instrument that inverts with a shift key is one you cannot read. */
    draw_store_gauge(buf, cy);
#endif
}

/* ------------------------------------------------------------- composition -- */

static void draw_sheet(uint8_t *buf, const struct zmk_widget_nino *widget, uint8_t view) {
    if (view == NINO_VIEW_MAIN) {
        draw_sheet_main(buf, widget);
        return;
    }
#if IS_ENABLED(CONFIG_NINO_NOTES)
    if (view == NINO_VIEW_XFER_IN) {
        draw_sheet_transfer(buf, widget->transfer_progress, "IN");
        return;
    }
    if (view == NINO_VIEW_XFER_OUT) {
        draw_sheet_transfer(buf, nino_store_read_progress(), "OUT");
        return;
    }
    draw_sheet_destow(buf, view);
#endif
}

/*
 * The machine: the cartridge's two stops and the globe. Drawn after the sheet
 * and never clipped, so a wipe passes through the paper without the frame
 * around it moving — which is what makes a screen change read as the sheet
 * being swapped rather than the whole panel being replaced.
 */
static void draw_chrome(uint8_t *buf, const struct zmk_widget_nino *widget) {
    screen_hline(buf, 0, SCREEN_W - 1, TRACK_TOP - 1, AMP_SOLID);
    screen_hline(buf, 0, SCREEN_W - 1, TRACK_TOP + TRACK_LEN, AMP_SOLID);
    draw_globe(buf, widget);
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

    screen_hline(buf, 0, SCREEN_W - 1, 0, AMP_SOLID);
    screen_hline(buf, 0, SCREEN_W - 1, SCREEN_H - 1, AMP_SOLID);
    screen_vline(buf, 0, 0, SCREEN_H - 1, AMP_SOLID);
    screen_vline(buf, SCREEN_W - 1, 0, SCREEN_H - 1, AMP_SOLID);

    screen_hline(buf, left, left + side, top, AMP_SOLID);
    screen_hline(buf, left, left + side, top + side, AMP_SOLID);
    screen_vline(buf, left, top, top + side, AMP_SOLID);
    screen_vline(buf, left + side, top, top + side, AMP_SOLID);

    screen_circle(buf, SCREEN_W / 2, SCREEN_H / 2, side / 2, AMP_SOLID);
}

/* ----------------------------------------------------------------- render -- */

/*
 * Flash the globe, on a knob click. Deliberately independent of the carriage:
 * typing on through the flash must not cut it short.
 */
static void nino_globe_strike(struct zmk_widget_nino *widget) {
    widget->globe_flash = GLOBE_FLASH_TOTAL;
}

/*
 * Shake the whole screen. The axis is what separates one event from another:
 * a ruler tick jolts straight down the way the carriage is travelling, while
 * the slam throws the screen sideways the way the carriage flies home. Two
 * different sensations rather than the same jiggle at two sizes.
 */
static void nino_shake(struct zmk_widget_nino *widget, uint8_t frames, uint8_t mag,
                       bool lateral) {
    widget->shake_frames = frames;
    widget->shake_mag = mag;
    widget->shake_lateral = lateral;
}

static void nino_advance(struct zmk_widget_nino *widget) {
    int prev_px = widget->carriage_pos;

    widget->column++;
    widget->slamming = false;
    widget->rebounding = false;
    widget->erase_frames = 0;

    /* Cadence, not keystrokes: this rises faster than it drains only while you
     * keep typing, and the chevron follows it. */
    widget->heat = (uint8_t)MIN(widget->heat + HEAT_PER_KEY, HEAT_MAX);

    /*
     * Cartridge full. The bar goes straight back to the top and starts the
     * next pass in a heavier texture — no slam, no shake, nothing that reads
     * as a carriage return. Only Return or the knob does that, and only that
     * clears the accumulated fill.
     */
    if (widget->column >= CART_CHARS) {
        widget->column = 0;
        widget->carriage_pos = 0;
        /* Counted past the point the fill stops thickening — the pips carry it
         * from there, so the counter is no longer clamped to the ladder. */
        if (widget->fill_layer < PASS_MAX) {
            widget->fill_layer++;
        }
        widget->wrap_sweep = WRAP_SWEEP_FRAMES;
        return;
    }

    widget->carriage_pos = (int16_t)carriage_px(widget->column);

    /* Crossing a graduation strikes it. Rare enough to punctuate rather than
     * accompany, which is what the old per-space ring never managed. */
    if ((widget->column % CONFIG_NINO_TICK_CHARS) == 0) {
        widget->tick_flash = TICK_FLASH_FRAMES;
        widget->tick_mark = (int8_t)(widget->column / CONFIG_NINO_TICK_CHARS);
    }

    /*
     * The ruler ticks used to jolt the screen as the carriage crossed them.
     * They no longer do. At a tick every twenty-five pixels of a seventy-two
     * pixel track, ordinary typing was setting the whole panel shivering every
     * second or so, and a machine that trembles continuously does not read as
     * responsive — it reads as loose. The ticks are visible; they do not also
     * need to be felt. Impact is reserved for the return, which is the one
     * moment that genuinely is one.
     */
    ARG_UNUSED(prev_px);
}

static void nino_backspace(struct zmk_widget_nino *widget) {
    if (widget->column > 0) {
        widget->column--;
    } else if (widget->fill_layer > 0) {
        /* Backing off the top of a pass unwinds into the end of the one
         * before it, so a wrap can be undone the same way anything else can. */
        widget->fill_layer--;
        widget->column = CART_CHARS - 1;
    }
    widget->slamming = false;
    widget->rebounding = false;
    widget->carriage_pos = (int16_t)carriage_px(widget->column);

    /*
     * Erasing is not typing in reverse: the chevron turns to face the way the
     * carriage is being dragged. The flip alone carries it — a shake here as
     * well meant holding backspace shook the panel continuously, which was the
     * worst of the lot.
     */
    widget->erase_frames = ERASE_FRAMES;
}

/*
 * The slam, and the only thing that resets the cartridge: fired by Return or
 * by the encoder click, never by running out of characters. carriage_pos is
 * deliberately left where it is so the bar flies home from wherever it had got
 * to rather than jumping.
 */
static void nino_return(struct zmk_widget_nino *widget) {
    widget->column = 0;
    widget->fill_layer = 0;
    widget->slamming = true;
    widget->rebounding = false;
    widget->tick_flash = 0;
    widget->erase_frames = 0;

    /* The slam owns the whole screen: a full-panel invert, a hard lateral
     * shake, and the globe kicked upward by the impact. */
    widget->strike_timer = CONFIG_NINO_SLAM_FLASH_FRAMES;
    widget->ball_recoil = CONFIG_NINO_BALL_RECOIL;
    nino_shake(widget, CONFIG_NINO_SHAKE_FRAMES, CONFIG_NINO_SHAKE_MAG, true);
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
        /*
         * The globe answers the knob on every screen, always. It is the one
         * thing the panel never takes away, so it is what tells you the knob
         * is connected at all — on a menu that would otherwise only move a
         * caret, and on a screen with nothing to move.
         */
        case NINO_INPUT_KNOB_TURN:
            if (zmk_keymap_highest_layer_active() > 0) {
                widget->pitch_target += in.value * CONFIG_NINO_BALL_STEP;
            } else {
                widget->yaw_target += in.value * CONFIG_NINO_BALL_STEP;
            }

            /*
             * Counted in detents, not in direction: this marks how far the knob
             * has been worked, so turning back and forth still gets there. The
             * globe otherwise spins without ever arriving anywhere.
             */
            if (++widget->knob_count >= CONFIG_NINO_KNOB_MARK) {
                widget->knob_count = 0;
                widget->orb_ring = ORB_RING_FRAMES;
                nino_shake(widget, ORB_TICK_FRAMES, ORB_TICK_MAG, false);
            }
#if IS_ENABLED(CONFIG_NINO_NOTES)
            if (nino_destow_active()) {
                nino_destow_turn(in.value);
                widget->dirty = true;
            }
#endif
            break;

        case NINO_INPUT_KNOB_CLICK:
            nino_globe_strike(widget);
#if IS_ENABLED(CONFIG_NINO_NOTES)
            if (nino_destow_active()) {
                /* No slam: the carriage is not on this sheet, and throwing it
                 * home would be the machine answering the wrong question. */
                nino_destow_click();
                widget->dirty = true;
                break;
            }
#endif
            /* The keymap also binds this position to Return, so the queue
             * usually carries both events. Slamming twice in one frame lands
             * on the same state, and doing it here keeps the knob working if
             * that binding ever changes. */
            nino_return(widget);
            break;
#if IS_ENABLED(CONFIG_NINO_NOTES)
        case NINO_INPUT_DESTOW:
            nino_destow_toggle();
            widget->dirty = true;
            break;
#endif
        default:
            break;
        }
    }

    /*
     * Input dismisses the splash, but by throwing the sheet out rather than by
     * cutting to black — the eject is the whole point of the gesture, and a
     * keypress that simply erased the screen would waste it. Skipping straight
     * to the last SPLASH_RETURN frames plays that wipe and nothing else, so an
     * impatient keypress still gets the machine finishing its movement.
     */
    if (had_input && widget->splash_frames > SPLASH_RETURN) {
        widget->splash_frames = SPLASH_RETURN;
        widget->dirty = true;
    }

    bool caps = atomic_get(&caps_lock_on) != 0;
    if (caps != widget->caps) {
        widget->caps = caps;
        widget->dirty = true;
    }

#if IS_ENABLED(CONFIG_NINO_NOTES)
    /*
     * Polled rather than pushed. The link thread has no business knowing a
     * display exists, and a transfer moves far faster than the frame rate
     * anyway — sampling once a frame is exactly the resolution the panel can
     * show. The transition either way is what forces a redraw.
     */
    {
        bool busy = nino_store_busy();
        uint8_t progress = busy ? nino_store_progress() : 0;

        if (busy != widget->transferring || progress != widget->transfer_progress) {
            widget->transferring = busy;
            widget->transfer_progress = progress;
            widget->dirty = true;
        }
    }
#endif

    /* A level rather than an event, like Caps Lock: what matters is whether a
     * key is down right now, not that one was struck. */
    bool pressed = atomic_get(&keys_held) > 0;
    if (pressed != widget->pressed) {
        widget->pressed = pressed;
        widget->dirty = true;
    }

    /*
     * Idle: after a spell with nothing touched, the globe starts turning by
     * itself and keeps turning until something happens. Nothing else moves —
     * the track is holding your cartridge progress, and animating that would
     * throw information away for the sake of decoration.
     */
    if (had_input || pressed) {
        widget->idle_frames = 0;
    } else if (widget->idle_frames < NINO_IDLE_FRAMES) {
        widget->idle_frames++;
    }

    bool idling = CONFIG_NINO_IDLE_SPIN > 0 && widget->idle_frames >= NINO_IDLE_FRAMES &&
                  widget->splash_frames == 0 && !widget->test_pattern;
    if (idling) {
        widget->yaw_target += CONFIG_NINO_IDLE_SPIN;
    }

    bool globe_moving = (widget->yaw != widget->yaw_target) ||
                        (widget->pitch != widget->pitch_target) || (widget->globe_flash > 0) || (widget->orb_ring > 0) ||
                        (widget->ball_recoil != 0);
    bool busy = had_input || widget->dirty || widget->test_pattern || globe_moving || idling ||
                widget->slamming || widget->rebounding || widget->splash_frames > 0 ||
                widget->shake_frames > 0 || widget->strike_timer > 0 || widget->tick_flash > 0 ||
                widget->erase_frames > 0 || widget->heat > 0 || widget->wrap_sweep > 0 ||
                widget->transferring || widget->shift_phase != NINO_SHIFT_NONE ||
                current_view() != widget->view;

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

    /*
     * The shake is a whole-screen offset, applied inside screen_plot. A tick
     * jolts purely along the track; a slam throws the screen across it, with
     * just enough of the other axis to keep it from looking like a glitch.
     */
    bool was_shaking = (widget->shake_frames > 0);
    if (was_shaking) {
        int mag = widget->shake_mag;
        int sign = (widget->shake_frames & 1) ? 1 : -1;

        if (widget->shake_lateral) {
            shake_dx = (int8_t)(sign * mag);
            shake_dy = (int8_t)(-sign * (mag / 3));
        } else {
            shake_dx = 0;
            shake_dy = (int8_t)(sign * mag);
        }
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

        /*
         * The hand-over is the carriage return itself, not a cut to black:
         * full-panel invert, a lateral throw, and the globe kicked by the
         * impact, all running on into the first frames of the real screen. The
         * first thing the machine ever does is the gesture it does most.
         */
        if (widget->splash_frames == SPLASH_RETURN) {
            widget->strike_timer = CONFIG_NINO_SLAM_FLASH_FRAMES;
            widget->ball_recoil = CONFIG_NINO_BALL_RECOIL;
            nino_shake(widget, CONFIG_NINO_SHAKE_FRAMES, CONFIG_NINO_SHAKE_MAG, true);
        }

        draw_splash(buf, widget);
        widget->splash_frames--;

        lv_obj_invalidate(widget->obj);
        /* Always owe another frame: the splash is animating every frame, and
         * the one after it ends has to draw the real screen. */
        widget->dirty = true;
        return;
    }

    /*
     * The slam: the carriage flies back far faster than it advanced, and it
     * does not stop politely at zero. It drives past the stop by the bounce
     * distance and springs back down, which is what turns the return from a
     * jump cut into an impact. Halving is exact here — C truncates toward
     * zero, so -3 goes to -1 and then to 0 rather than creeping.
     */
    if (widget->slamming) {
        widget->carriage_pos -= CONFIG_NINO_SLAM_SPEED;
        /* Tested against the stop, not against the bounce depth: landing just
         * short of full depth would otherwise miss the test and dip twice. */
        if (widget->carriage_pos <= 0) {
            widget->carriage_pos = -CONFIG_NINO_SLAM_BOUNCE;
            widget->slamming = false;
            widget->rebounding = (CONFIG_NINO_SLAM_BOUNCE > 0);
        }
    } else if (widget->rebounding) {
        widget->carriage_pos = (int16_t)(widget->carriage_pos / 2);
        if (widget->carriage_pos == 0) {
            widget->rebounding = false;
        }
    }

    /*
     * The sheet, then the machine around it. A view change wipes one rule down
     * the cartridge, outgoing sheet below it and incoming above — the stops
     * and the globe never move, so a screen change reads as paper being
     * swapped rather than the panel being replaced.
     */
    uint8_t target = current_view();
    if (target != widget->view) {
        widget->view_prev = widget->view;
        widget->view = target;

        /* Start the bar wherever the outgoing sheet had it, so the throw
         * begins from a real position rather than from the bottom of the
         * track every time. */
        widget->shift_phase = NINO_SHIFT_SLAM;
        widget->shift_pos = (widget->view_prev == NINO_VIEW_MAIN)
                                ? CLAMP(widget->carriage_pos, 0, TRACK_LEN)
                                : TRACK_LEN;

        /* The full gesture, because this IS the gesture: the panel throws, the
         * globe takes the impact, and the whole thing flashes. */
        widget->strike_timer = CONFIG_NINO_SLAM_FLASH_FRAMES;
        widget->ball_recoil = CONFIG_NINO_BALL_RECOIL;
        nino_shake(widget, CONFIG_NINO_SHAKE_FRAMES, CONFIG_NINO_SHAKE_MAG, true);
    }

    bool was_shifting = (widget->shift_phase != NINO_SHIFT_NONE);

    if (widget->shift_phase == NINO_SHIFT_SLAM) {
        /*
         * The outgoing sheet withdraws behind the bar: everything above it has
         * already been taken, so the cartridge empties from the top down as
         * the bar climbs.
         */
        clip_rows(SHEET_TOP + widget->shift_pos, SHEET_BOTTOM);
        draw_sheet(buf, widget, widget->view_prev);
        clip_none();

        screen_hline(buf, 0, SCREEN_W - 1, SHEET_TOP + widget->shift_pos, AMP_SOLID);

        widget->shift_pos -= CONFIG_NINO_SLAM_SPEED;
        if (widget->shift_pos <= 0) {
            widget->shift_pos = 0;
            widget->shift_phase = NINO_SHIFT_REVEAL;
        }
    } else if (widget->shift_phase == NINO_SHIFT_REVEAL) {
        /* And the incoming one is uncovered as the bar travels back down. */
        clip_rows(SHEET_TOP, SHEET_TOP + widget->shift_pos);
        draw_sheet(buf, widget, widget->view);
        clip_none();

        screen_hline(buf, 0, SCREEN_W - 1, SHEET_TOP + widget->shift_pos, AMP_SOLID);

        widget->shift_pos += REVEAL_SPEED;
        if (widget->shift_pos >= SHEET_BOTTOM - SHEET_TOP) {
            widget->shift_pos = 0;
            widget->shift_phase = NINO_SHIFT_NONE;
        }
    } else {
        draw_sheet(buf, widget, widget->view);
    }

    /* Drawn before the decrement, so the flash gets its full two solid frames. */
    draw_chrome(buf, widget);

    bool was_flashing = (widget->globe_flash > 0);
    if (was_flashing) {
        widget->globe_flash--;
    }

    bool was_ringing = (widget->orb_ring > 0);
    if (was_ringing) {
        widget->orb_ring--;
    }

    /* Same latch as the effects above: the tick's last frame has to be drawn
     * before the counter reaching zero is allowed to let the panel go idle. */
    bool was_ticking = (widget->tick_flash > 0);
    if (was_ticking) {
        widget->tick_flash--;
    }

    bool was_erasing = (widget->erase_frames > 0);
    if (was_erasing) {
        widget->erase_frames--;
    }

    bool was_wrapping = (widget->wrap_sweep > 0);
    if (was_wrapping) {
        widget->wrap_sweep--;
    }

    /*
     * Ink creeps only on frames that are actually drawn, so a resting panel
     * stops rather than drifting in the dark and jumping when it wakes.
     */
    ink_clock++;
    if (ink_clock >= INK_DRIFT_FRAMES * 4) {
        ink_clock = 0;
    }

    /* Proportional drain, so the chevron relaxes when you stop. */
    bool was_hot = (widget->heat > 0);
    if (was_hot) {
        uint8_t drain = HEAT_DRAIN(widget->heat);
        widget->heat = (widget->heat > drain) ? (uint8_t)(widget->heat - drain) : 0;
    }

    /* The globe settles back down from the slam's kick. */
    bool was_recoiling = (widget->ball_recoil != 0);
    widget->ball_recoil = (int8_t)(widget->ball_recoil / 2);

    lv_obj_invalidate(widget->obj);

    spring(&widget->yaw, &widget->yaw_vel, widget->yaw_target);
    spring(&widget->pitch, &widget->pitch_vel, widget->pitch_target);
    spring_wrap(&widget->yaw, &widget->yaw_target);
    spring_wrap(&widget->pitch, &widget->pitch_target);

    globe_moving = (widget->yaw != widget->yaw_target) ||
                   (widget->pitch != widget->pitch_target) || (widget->globe_flash > 0);

    /* One more frame is owed once everything settles, to draw the resting state. */
    widget->dirty = was_shifting || was_ringing || current_view() != widget->view || globe_moving ||
                    widget->slamming || widget->rebounding || was_striking ||
                    was_shaking || was_flashing || was_recoiling || was_ticking || was_wrapping ||
                    was_erasing ||
                    was_hot;
}

/* -------------------------------------------------------------- listeners -- */

static bool is_printable(uint32_t keycode) {
    if (keycode >= 0x04 && keycode <= 0x27) {
        return true; /* a-z, 1-0 */
    }
    if (keycode == KC_SPACE) {
        return true; /* just another character; it no longer has an effect of its own */
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
    if (ev == NULL || ev->usage_page != HID_USAGE_KEY) {
        return ZMK_EV_EVENT_BUBBLE;
    }

    /* Chord state is a level, so it tracks both edges — everything below this
     * point only cares about presses. */
    if (ev->keycode == KC_SPACE) {
        atomic_set(&space_held, ev->state ? 1 : 0);
    } else if (ev->keycode == KC_LSHIFT || ev->keycode == KC_RSHIFT) {
        atomic_set(&shift_held, ev->state ? 1 : 0);
    }

    if (!ev->state) {
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
    if (ev == NULL) {
        return ZMK_EV_EVENT_BUBBLE;
    }

    /*
     * Physical key count, for the carriage's press dip. Clamped at both ends
     * rather than trusted: the matrix pairs its edges up, but a dropped event
     * would otherwise leave the carriage depressed for good.
     */
    if (ev->state) {
        /* Zephyr's atomic_inc/dec return the value from *before* the update. */
        if (atomic_inc(&keys_held) >= NINO_MAX_HELD) {
            atomic_set(&keys_held, NINO_MAX_HELD);
        }
    } else {
        if (atomic_dec(&keys_held) <= 0) {
            atomic_set(&keys_held, 0);
        }
    }

    /* The encoder's push-switch is an ordinary matrix key. */
    if (ev->state && ev->position == CONFIG_NINO_KNOB_POSITION) {
#if IS_ENABLED(CONFIG_NINO_NOTES)
        /*
         * Space and Shift held turns the click into the destow gesture instead
         * of a carriage return. Consumed rather than bubbled, so the Return the
         * keymap has on this position never reaches the host — a mode change
         * should not also type into whatever is focused.
         *
         * This depends on running before ZMK's own keymap listener. Listener
         * order is link order, so if a stray newline ever appears when opening
         * destow, that is what happened.
         */
        if (atomic_get(&space_held) && atomic_get(&shift_held)) {
            nino_post(NINO_INPUT_DESTOW, 0);
            return ZMK_EV_EVENT_HANDLED;
        }
#endif
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
    widget->fill_layer = 0;
    widget->slamming = false;
    widget->rebounding = false;
    widget->shake_frames = 0;
    widget->shake_mag = 0;
    widget->shake_lateral = false;
    widget->erase_frames = 0;
    widget->heat = 0;
    widget->pressed = false;
    widget->idle_frames = 0;
    widget->tick_flash = 0;
    widget->tick_mark = -1;
    widget->transferring = false;
    widget->transfer_progress = 0;

    /* Starts settled on the working sheet, so the first frame after the splash
     * is not a wipe from a view that was never on screen. */
    widget->knob_count = 0;
    widget->orb_ring = 0;

    widget->view = NINO_VIEW_MAIN;
    widget->view_prev = NINO_VIEW_MAIN;
    widget->shift_phase = NINO_SHIFT_NONE;
    widget->shift_pos = 0;
    widget->strike_timer = 0;
    widget->globe_flash = 0;
    widget->ball_recoil = 0;
    widget->caps = false;

    widget->dirty = true;
    widget->test_pattern = IS_ENABLED(CONFIG_NINO_TEST_PATTERN);
    /* Seating, striking and the return are fixed costs; whatever is left over
     * is hold time. Clamped so a short SPLASH_MS truncates the hold rather
     * than cutting the mechanism itself in half. */
    uint16_t frames = CONFIG_NINO_SPLASH_MS / CONFIG_NINO_FRAME_MS;
    if (frames < SPLASH_FIXED) {
        frames = SPLASH_FIXED;
    }
    widget->splash_frames = frames;
    widget->splash_total = frames;

    /* Start off-axis so the globe reads as a sphere from the first frame. */
    widget->yaw = widget->yaw_target = 5 * 16;
    widget->pitch = widget->pitch_target = 3 * 16;
    widget->yaw_vel = widget->pitch_vel = 0;

    widget->timer = lv_timer_create(nino_render, CONFIG_NINO_FRAME_MS, widget);

    return 0;
}

lv_obj_t *zmk_widget_nino_obj(struct zmk_widget_nino *widget) { return widget->obj; }
