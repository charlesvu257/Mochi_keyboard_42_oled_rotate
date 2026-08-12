/*
 * NINO — custom status screen
 *
 * Replaces ZMK's four-corner status screen. The NINO widget owns the whole
 * panel; there is no battery, output or layer readout, by design.
 *
 * SPDX-License-Identifier: MIT
 */

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

#include "widgets/nino.h"

#if IS_ENABLED(CONFIG_NINO_DISPLAY)
static struct zmk_widget_nino nino_widget;
#endif

lv_obj_t *zmk_display_status_screen() {
    lv_obj_t *screen = lv_obj_create(NULL);

    /* No padding, border or scrolling — the canvas is the entire screen. */
    lv_obj_set_style_pad_all(screen, 0, LV_PART_MAIN);
    lv_obj_set_style_border_width(screen, 0, LV_PART_MAIN);
    lv_obj_clear_flag(screen, LV_OBJ_FLAG_SCROLLABLE);

#if IS_ENABLED(CONFIG_NINO_DISPLAY)
    zmk_widget_nino_init(&nino_widget, screen);
    lv_obj_align(zmk_widget_nino_obj(&nino_widget), LV_ALIGN_TOP_LEFT, 0, 0);
#endif

    return screen;
}
