/*
 * NINO — 3x5 uppercase font
 *
 * The panel is mounted a quarter turn over, so LVGL's own fonts are unusable
 * here: text drawn through the canvas API would come out rotated ninety
 * degrees. Everything on this screen is hand-plotted in screen space, and text
 * has to be too.
 *
 * Three pixels wide, plus one of tracking, is four per character — eight
 * characters across a 32-pixel panel. Every word the interface uses is chosen
 * to fit in eight.
 *
 * Each glyph is five rows; bit 2 is the leftmost column.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

#include <stdint.h>

#define NINO_GLYPH_W 3
#define NINO_GLYPH_H 5
#define NINO_GLYPH_ADVANCE 4

/* Index 0 is space, 1..26 are A..Z, 27..36 are 0..9. */
static const uint8_t nino_glyphs[37][NINO_GLYPH_H] = {
    {0, 0, 0, 0, 0},       /* space */
    {7, 5, 7, 5, 5},       /* A */
    {6, 5, 6, 5, 6},       /* B */
    {3, 4, 4, 4, 3},       /* C */
    {6, 5, 5, 5, 6},       /* D */
    {7, 4, 6, 4, 7},       /* E */
    {7, 4, 6, 4, 4},       /* F */
    {3, 4, 5, 5, 3},       /* G */
    {5, 5, 7, 5, 5},       /* H */
    {7, 2, 2, 2, 7},       /* I */
    {1, 1, 1, 5, 2},       /* J */
    {5, 5, 6, 5, 5},       /* K */
    {4, 4, 4, 4, 7},       /* L */
    {7, 7, 5, 5, 5},       /* M */
    {6, 5, 5, 5, 5},       /* N */
    {7, 5, 5, 5, 7},       /* O */
    {7, 5, 7, 4, 4},       /* P */
    {7, 5, 5, 7, 1},       /* Q */
    {7, 5, 7, 6, 5},       /* R */
    {3, 4, 7, 1, 6},       /* S */
    {7, 2, 2, 2, 2},       /* T */
    {5, 5, 5, 5, 7},       /* U */
    {5, 5, 5, 5, 2},       /* V */
    {5, 5, 5, 7, 7},       /* W */
    {5, 5, 2, 5, 5},       /* X */
    {5, 5, 2, 2, 2},       /* Y */
    {7, 1, 2, 4, 7},       /* Z */
    {7, 5, 5, 5, 7},       /* 0 */
    {2, 6, 2, 2, 7},       /* 1 */
    {7, 1, 7, 4, 7},       /* 2 */
    {7, 1, 7, 1, 7},       /* 3 */
    {5, 5, 7, 1, 1},       /* 4 */
    {7, 4, 7, 1, 7},       /* 5 */
    {7, 4, 7, 5, 7},       /* 6 */
    {7, 1, 2, 2, 2},       /* 7 */
    {7, 5, 7, 5, 7},       /* 8 */
    {7, 5, 7, 1, 7},       /* 9 */
};

/* Anything unrepresentable becomes a space rather than a wrong glyph. */
static inline uint8_t nino_glyph_index(char c) {
    if (c >= 'a' && c <= 'z') {
        c = (char)(c - 'a' + 'A');
    }
    if (c >= 'A' && c <= 'Z') {
        return (uint8_t)(1 + (c - 'A'));
    }
    if (c >= '0' && c <= '9') {
        return (uint8_t)(27 + (c - '0'));
    }
    return 0;
}
