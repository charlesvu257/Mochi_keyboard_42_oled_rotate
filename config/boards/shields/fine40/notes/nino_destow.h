/*
 * NINO — interactive destow
 *
 * A small screen on the knob: check what is stored, ask the host to come and
 * get it, or wipe it. The keyboard types a command into a focused terminal
 * and destow.py does the actual reading over the serial link.
 *
 * This module owns the state and the typing. It draws nothing — the widget
 * asks it what to show.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

enum nino_destow_screen {
    NINO_DESTOW_OFF = 0,
    NINO_DESTOW_MENU,    /* choosing an action */
    NINO_DESTOW_SENT,    /* the command is going out */
    NINO_DESTOW_CONFIRM, /* wipe? */
    NINO_DESTOW_EMPTY,   /* nothing stored */
};

enum nino_destow_item {
    NINO_ITEM_SEND = 0,
    NINO_ITEM_WIPE,
    NINO_ITEM_EXIT,
    NINO_ITEM_COUNT,
};

enum nino_destow_screen nino_destow_screen(void);
uint8_t nino_destow_selection(void);

static inline bool nino_destow_active(void) {
    return nino_destow_screen() != NINO_DESTOW_OFF;
}

/* Input, routed from the widget's listeners. */
void nino_destow_toggle(void);
void nino_destow_turn(int8_t direction);
void nino_destow_click(void);
