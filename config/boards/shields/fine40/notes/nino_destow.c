/*
 * NINO — interactive destow
 *
 * The keyboard does not read its own notes out. It asks the host to.
 *
 * An earlier version typed the whole store as base32 keystrokes: chunked,
 * checksummed, and about five minutes of untouchable keyboard for a full
 * cartridge. It worked, but it was solving the wrong problem — the host
 * already has destow.py and a serial port that moves the same bytes in a
 * second. All the keyboard actually needs to do is say go.
 *
 * So it types a command. Twenty-odd keystrokes into a focused terminal, and
 * the script does the reading over the fast path. Everything that made the
 * old version delicate — chunk indices, CRCs, dropped characters, host HID
 * throughput — belongs to a problem that no longer exists.
 *
 * SPDX-License-Identifier: MIT
 */

#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>

#include <dt-bindings/zmk/hid_usage_pages.h>
#include <zmk/events/keycode_state_changed.h>
#include <zmk/keys.h>

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

#include "nino_destow.h"
#include "nino_store.h"

#define USAGE_ENTER 0x28
#define USAGE_LSHIFT 0xE1

/* How long "SENT" sits on the panel before it hands back. */
#define SENT_FRAMES_MS 900

/*
 * ASCII 0x20..0x7E to HID usage, US layout. Bit 7 means the key needs Shift.
 * A command is short and may be a path, so unlike the old base32 stream this
 * cannot get away with unshifted characters only.
 */
#define SH 0x80
static const uint8_t ascii_usage[95] = {
    0x2C,     0x1E | SH, 0x34 | SH, 0x20 | SH, 0x21 | SH, 0x22 | SH, 0x24 | SH, /*  !"#$%& */
    0x34,     0x26 | SH, 0x27 | SH, 0x25 | SH, 0x2E | SH, 0x36,     0x2D,     /* '()*+,- */
    0x37,     0x38,                                                       /* ./ */
    0x27,     0x1E,     0x1F,     0x20,     0x21,     0x22,     0x23,     /* 0123456 */
    0x24,     0x25,     0x26,                                             /* 789 */
    0x33 | SH, 0x33,     0x36 | SH, 0x2E,     0x37 | SH, 0x38 | SH, 0x1F | SH, /* :;<=>?@ */
    0x04 | SH, 0x05 | SH, 0x06 | SH, 0x07 | SH, 0x08 | SH, 0x09 | SH, 0x0A | SH, /* ABCDEFG */
    0x0B | SH, 0x0C | SH, 0x0D | SH, 0x0E | SH, 0x0F | SH, 0x10 | SH, 0x11 | SH, /* HIJKLMN */
    0x12 | SH, 0x13 | SH, 0x14 | SH, 0x15 | SH, 0x16 | SH, 0x17 | SH, 0x18 | SH, /* OPQRSTU */
    0x19 | SH, 0x1A | SH, 0x1B | SH, 0x1C | SH, 0x1D | SH,                     /* VWXYZ */
    0x2F,     0x31,     0x30,     0x23 | SH, 0x2D | SH, 0x35,               /* [\]^_` */
    0x04,     0x05,     0x06,     0x07,     0x08,     0x09,     0x0A,     /* abcdefg */
    0x0B,     0x0C,     0x0D,     0x0E,     0x0F,     0x10,     0x11,     /* hijklmn */
    0x12,     0x13,     0x14,     0x15,     0x16,     0x17,     0x18,     /* opqrstu */
    0x19,     0x1A,     0x1B,     0x1C,     0x1D,                         /* vwxyz */
    0x2F | SH, 0x31 | SH, 0x30 | SH, 0x35 | SH,                               /* {|}~ */
};
#undef SH

static struct {
    enum nino_destow_screen screen;
    uint8_t selection;
} state;

/* ------------------------------------------------------------------ keys -- */

static void key_event(uint32_t usage, bool pressed) {
    raise_zmk_keycode_state_changed_from_encoded(ZMK_HID_USAGE(HID_USAGE_KEY, usage), pressed,
                                                 k_uptime_get());
    k_sleep(K_MSEC(CONFIG_NINO_DESTOW_KEY_MS));
}

/*
 * Shift is pressed and released around the key rather than folded into the
 * keycode's implicit modifiers. Four events instead of two, but it does not
 * depend on how ZMK happens to pack modifiers into an encoded keycode, and at
 * twenty keystrokes the cost is nothing.
 */
static void type_char(char c) {
    if (c == '\n') {
        key_event(USAGE_ENTER, true);
        key_event(USAGE_ENTER, false);
        return;
    }

    if (c < 0x20 || c > 0x7E) {
        return;
    }

    uint8_t entry = ascii_usage[c - 0x20];
    bool shifted = (entry & 0x80) != 0;
    uint32_t usage = entry & 0x7F;

    if (shifted) {
        key_event(USAGE_LSHIFT, true);
    }
    key_event(usage, true);
    key_event(usage, false);
    if (shifted) {
        key_event(USAGE_LSHIFT, false);
    }
}

/* ---------------------------------------------------------------- typing -- */

/*
 * Still its own thread rather than a work item. It is only a few hundred
 * milliseconds now, but every keystroke sleeps between its edges, and sleeping
 * on the system work queue blocks every other work item on the board.
 */
static K_SEM_DEFINE(destow_go, 0, 1);

static void destow_thread(void *a, void *b, void *c) {
    ARG_UNUSED(a);
    ARG_UNUSED(b);
    ARG_UNUSED(c);

    while (true) {
        k_sem_take(&destow_go, K_FOREVER);

        for (const char *p = CONFIG_NINO_DESTOW_COMMAND; *p != '\0'; p++) {
            type_char(*p);
        }
        type_char('\n');

        /* Long enough to be read, short enough not to be in the way. */
        k_sleep(K_MSEC(SENT_FRAMES_MS));

        if (state.screen == NINO_DESTOW_SENT) {
            state.screen = NINO_DESTOW_OFF;
        }
    }
}

K_THREAD_DEFINE(nino_destow_tid, CONFIG_NINO_DESTOW_STACK_SIZE, destow_thread, NULL, NULL, NULL,
                CONFIG_NINO_DESTOW_PRIORITY, 0, 0);

/* ------------------------------------------------------------------- api -- */

enum nino_destow_screen nino_destow_screen(void) { return state.screen; }

uint8_t nino_destow_selection(void) { return state.selection; }

void nino_destow_toggle(void) {
    if (state.screen == NINO_DESTOW_OFF) {
        state.screen = (nino_store_used() > 0) ? NINO_DESTOW_MENU : NINO_DESTOW_EMPTY;
        state.selection = 0;
        return;
    }

    /* Not while the command is going out — half a command line is worse than
     * none, and it is over in well under a second anyway. */
    if (state.screen == NINO_DESTOW_SENT) {
        return;
    }

    state.screen = NINO_DESTOW_OFF;
}

void nino_destow_turn(int8_t direction) {
    if (state.screen == NINO_DESTOW_MENU) {
        int sel = (int)state.selection + direction;
        state.selection = (uint8_t)CLAMP(sel, 0, NINO_ITEM_COUNT - 1);
    } else if (state.screen == NINO_DESTOW_CONFIRM) {
        state.selection ^= 1; /* no / yes */
    }
}

void nino_destow_click(void) {
    switch (state.screen) {
    case NINO_DESTOW_MENU:
        switch (state.selection) {
        case NINO_ITEM_SEND:
            state.screen = NINO_DESTOW_SENT;
            k_sem_give(&destow_go);
            break;
        case NINO_ITEM_WIPE:
            state.screen = NINO_DESTOW_CONFIRM;
            state.selection = 0;
            break;
        default:
            state.screen = NINO_DESTOW_OFF;
            break;
        }
        break;

    case NINO_DESTOW_CONFIRM:
        if (state.selection == 1) {
            nino_store_erase();
        }
        state.screen = NINO_DESTOW_OFF;
        break;

    case NINO_DESTOW_EMPTY:
        state.screen = NINO_DESTOW_OFF;
        break;

    default:
        break;
    }
}
