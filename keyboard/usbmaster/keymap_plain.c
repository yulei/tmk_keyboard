/*
Copyright 2012,2013 Jun Wako <wakojun@gmail.com>

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 2 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "keycode.h"
#include "action.h"
#include "action_macro.h"
#include "report.h"
#include "host.h"
#include "print.h"
#include "debug.h"
#include "keymap.h"

#define KEYMAP( \
    K00, K01, K02, K03, K04, K05, K06, K07, K08, K09, K0a, K0b, K0c, K0d, \
    K10, K11, K12, K13, K14, K15, K16, K17, K18, K19, K1a, K1b, K1c, \
    K20, K21, K22, K23, K24, K25, K26, K27, K28, K29, K2a, K2b, K2c, \
    K30, K31, K32,      K35,      K38,      K39, K3a, K3b \
) { \
    { KC_##K00, KC_##K01, KC_##K02, KC_##K03, KC_##K04, KC_##K05, KC_##K06, KC_##K07, KC_##K08, KC_##K09, KC_##K0a, KC_##K0b, KC_##K0c }, \
    { KC_##K10, KC_##K11, KC_##K12, KC_##K13, KC_##K14, KC_##K15, KC_##K16, KC_##K17, KC_##K18, KC_##K19, KC_##K1a, KC_##K1b, KC_##K1c }, \
    { KC_##K20, KC_##K21, KC_##K22, KC_##K23, KC_##K24, KC_##K25, KC_##K26, KC_##K27, KC_##K28, KC_##K29, KC_##K2a, KC_##K2b, KC_##K2c }, \
    { KC_##K30, KC_##K31, KC_##K32, KC_NO,    KC_NO,    KC_##K35, KC_NO,    KC_NO,    KC_##K38, KC_##K39, KC_##K3a, KC_##K3b, KC_##K0d }, \
}

const uint8_t keymaps[][MATRIX_ROWS][MATRIX_COLS] PROGMEM = {

    [0] = KEYMAP(ESC, Q,   W,   E,   R,   T,   Y,   U,   I,   O,   P,   LBRC,BSPC,      HOME,
                 LCTL,A,   S,   D,   F,   G,   H,   J,   K,   L,   SCLN,ENT,            END,
                 LSFT,Z,   X,   C,   V,   B,   N,   M,   COMM,DOT, RSFT,          UP,   FN0,
                 FN1, LGUI,LALT,          SPC,                FN2,           LEFT,DOWN,RGHT),

    [1] = KEYMAP(ESC, Q,   W,   E,   R,   T,   Y,   U,   I,   O,   P,   LBRC,BSPC,      HOME,
                 LCTL,A,   S,   D,   F,   G,   H,   J,   K,   L,   SCLN,ENT,            END,
                 LSFT,Z,   X,   C,   V,   B,   N,   M,   COMM,DOT, RSFT,          UP,   FN0,
                 FN1, LGUI,LALT,          SPC,                FN2,           LEFT,DOWN,RGHT),

    [2] = KEYMAP(ESC, Q,   W,   E,   R,   T,   Y,   U,   I,   O,   P,   LBRC,BSPC,      HOME,
                 LCTL,A,   S,   D,   F,   G,   H,   J,   K,   L,   SCLN,ENT,            END,
                 LSFT,Z,   X,   C,   V,   B,   N,   M,   COMM,DOT, RSFT,          UP,   FN0,
                 FN1, LGUI,LALT,          SPC,                FN2,           LEFT,DOWN,RGHT),

    [3] = KEYMAP(ESC, Q,   W,   E,   R,   T,   Y,   U,   I,   O,   P,   LBRC,BSPC,      HOME,
                 LCTL,A,   S,   D,   F,   G,   H,   J,   K,   L,   SCLN,ENT,            END,
                 LSFT,Z,   X,   C,   V,   B,   N,   M,   COMM,DOT, RSFT,          UP,   FN0,
                 FN1, LGUI,LALT,          SPC,                FN2,           LEFT,DOWN,RGHT)
};



/*
 * Fn action definition
 */
const action_t fn_actions[] PROGMEM = {
    [0]  = ACTION_LAYER_MOMENTARY(1),
    [1]  = ACTION_LAYER_MOMENTARY(2),
    [2]  = ACTION_LAYER_MOMENTARY(3),
};

/* translates key to keycode */
uint8_t keymap_key_to_keycode(uint8_t layer, keypos_t key)
{
    return keymaps[(layer)][(key.row)][(key.col)];
}

/* translates Fn keycode to action */
action_t keymap_fn_to_action(uint8_t keycode)
{
    return fn_actions[FN_INDEX(keycode)];
}
