/* Copyright (C) 2013-2016, The Regents of The University of Michigan.
All rights reserved.

This software was developed in the APRIL Robotics Lab under the
direction of Edwin Olson, ebolson@umich.edu. This software may be
available under alternative licensing terms; contact the address above.

This library is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 2.1 of the License, or (at your option) any later version.

This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License along with this library; if not, see <http://www.gnu.org/licenses/>.
*/

#ifndef _VX_CONSOLE_H
#define _VX_CONSOLE_H

#include "vx.h"

#include "common/zarray.h"
#include "common/string_util.h"

typedef struct vx_console vx_console_t;
struct vx_console
{
    vx_buffer_t *draw_buffer;   // owned by vx

    void (*console_command)(vx_console_t *vc, vx_layer_t * vl, const char *cmd, void *user);
    zarray_t* (*console_tab)(vx_console_t *vc, vx_layer_t * vl, const char *cmd, void *user);
    void *user;

    int has_keyboard_focus;

    zarray_t *output;         // strings, owned by vx_console, oldest values first
    int max_lines;
    int max_chars_width;

    // When the user actually commits a command, it gets added to the
    // history, up to history_maxsize
    zarray_t *command_history; // only for up-arrow based completions
    int command_history_maxsize;

    // as the user navigates through history, possibly editing
    // historical items, we create copies of all the (possibly edited)
    // history commands here. This will always have
    // size(command_history) + 1 entries, though if NULL, the
    // corresponding entry in command_history should be used.
    zarray_t *current_commands;

    // initialized to size(command_history).
    int current_commands_idx;

    // where in the current line are we?
    int cursor_position;

    char *prompt;                // how the prompt is displayed (including formatting)
    char *format_cursor;
    char *format_active_command; // the currently edited line
    char *format_history_command;
    char *format_tab_complete_command; // used when there are multiple completions
};

/**
 * Create a vx console. Must provide function pointers for a function to
 * receive commands, and a function to perform list possible commands during
 * tab completion. Callbacks occur in the GUI thread, in vx key event callbacks.
 *
 * draw_buffer:    a vx_buffer where the console will actually be drawn
 * console_command: passed a full command
 * console_tab:     a tab-complete function. Passed the current command. Returns
 *                  a zarray of one string per possible completion. Spaces
 *                  should be used in the results when appropriate.
 * user:            a user pointer to pass to callback functions
 */
vx_console_t*
vx_console_create(vx_buffer_t *draw_buffer,
                  void (*console_command)(vx_console_t *vc, vx_layer_t * vl, const char *cmd, void *user),
                  zarray_t* (*console_tab)(vx_console_t *vc, vx_layer_t * vl, const char *cmd, void *user),
                  void *user);

void vx_console_setup_layer(vx_console_t *vc, vx_layer_t *vl);

// clear all output
void vx_console_clear(vx_console_t *vc);

/**
 * Destroy a vx console.
 */
void vx_console_destroy(vx_console_t *vc);

void vx_console_printf(vx_console_t *vc, const char *fmt, ...) __attribute__ ((format (printf, 2, 3)));

#endif
