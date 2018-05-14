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

#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <pthread.h>
#include <string.h>
#include <stdbool.h>

#include "vx_console.h"

static void draw_console(vx_console_t *vc)
{
    // assemble a string for vxo_text in lower left
    string_buffer_t *sb = string_buffer_create();

    // render the console output. The format of this data is specified
    // by the user in console_printf; don't mess with it.
    for (int i = 0; i < zarray_size(vc->output); i++) {
        char *line;

        zarray_get(vc->output, i, &line);
        string_buffer_append_string(sb, vc->format_history_command);
        string_buffer_append_string(sb, line);
        if (i != zarray_size(vc->output)-1  || vc->has_keyboard_focus)
            string_buffer_append(sb, '\n'); // add newlines for each command
    }

    // render command if active
    if (vc->has_keyboard_focus) {
        char *command;
        zarray_get(vc->current_commands, vc->current_commands_idx, &command);
        assert(command != NULL);

        string_buffer_append_string(sb, vc->prompt);
        string_buffer_append_string(sb, vc->format_active_command);

        int commandlen = strlen(command);
        assert(vc->cursor_position <= commandlen);

        // output the portion of the current command BEFORE the cursor
        if (1) {
            char tmp[commandlen + 1];
            memcpy(tmp, command, vc->cursor_position);
            tmp[vc->cursor_position] = 0;
            string_buffer_append_string(sb, tmp);
        }

        // output the character UNDER the cursor
        if (1) {
            string_buffer_append_string(sb, vc->format_cursor);
            char tmp[2];
            tmp[0] = '_'; // default cursor character
            tmp[1] = 0;
            if (vc->cursor_position < commandlen)
                tmp[0] = command[vc->cursor_position];
            string_buffer_append_string(sb, tmp);
            string_buffer_append_string(sb, vc->format_active_command);
        }

        // output the portion of the current command AFTER the cursor
        int remainlen = commandlen - vc->cursor_position - 1;
        if (remainlen > 0) {
            char tmp[remainlen + 1];
            memcpy(tmp, &command[vc->cursor_position + 1], remainlen);
            tmp[remainlen] = 0;
            string_buffer_append_string(sb, tmp);
        }
    }

    // render console
    char *sb_str = string_buffer_to_string(sb);
    vx_object_t *vt = vxo_text(VXO_TEXT_ANCHOR_BOTTOM_LEFT, sb_str);
    free(sb_str);
    string_buffer_destroy(sb);
    vx_buffer_add_back(vc->draw_buffer,
                       vxo_depth_test(0,
                                      vxo_pixcoords(VXO_PIXCOORDS_BOTTOM_LEFT, VXO_PIXCOORDS_SCALE_MODE_ONE,
                                                    vt,
                                                    NULL),
                                      NULL),
                       NULL);

    vx_buffer_swap(vc->draw_buffer);
}

static void handle_tab_completions(vx_console_t *vc, const zarray_t *completions)
{
    if (zarray_size(completions) == 1) {
        // swap out the the command buffer
        // assume that tab completion function matches the last token
        // Match again tokens, not full string,
        // preserving whitespace

        char *completion;
        zarray_get(completions, 0, &completion);

        char *cmd;
        zarray_get(vc->current_commands, vc->current_commands_idx, &cmd);

        char *completed = sprintf_alloc("%s%s", completion, &cmd[vc->cursor_position]);
        zarray_set(vc->current_commands, vc->current_commands_idx, &completed, NULL);

        vc->cursor_position = strlen(completion);

    } else if (zarray_size(completions) > 1) {

        // auto-complete the command to the longest common substring
        // in the list of completions.
        if (1) {
            char *comp = NULL;
            for (int i = 0; i < zarray_size(completions); i++) {
                char *c;
                zarray_get(completions, i, &c);

                if (comp == NULL)
                    comp = strdup(c);

                int match_length = 0;
                while (c[match_length] == comp[match_length] && c[match_length] != 0)
                    match_length++;

                comp[match_length] = 0;
            }

            char *old_command;
            zarray_get(vc->current_commands, vc->current_commands_idx, &old_command);
            free(old_command);
            zarray_set(vc->current_commands, vc->current_commands_idx, &comp, NULL);
            vc->cursor_position = strlen(comp);
        }

        // create list of completions

        // find maximum length
        int col_width = 0;
        for (int i = 0; i < zarray_size(completions); i++) {
            char *tok;
            zarray_get(completions, i, &tok);
            int len = strlen(tok);
            if (len > col_width)
                col_width = len;
        }
        col_width += 3;

        string_buffer_t *sb = string_buffer_create();
        string_buffer_append_string(sb, vc->format_tab_complete_command);
        string_buffer_append_string(sb, "\n  ");

        int nr_chars = 0;
        for (int i = 0; i < zarray_size(completions); i++) {
            char *tok;
            zarray_get(completions, i, &tok);
            int len = strlen(tok);

            // we need a newline if we can't fit this completion on
            // our current line.
            if (nr_chars > 0 && len + nr_chars > vc->max_chars_width) {
                string_buffer_append_string(sb, "\n  ");
                nr_chars = 0;
            }

            // output this completion.
            string_buffer_appendf(sb, "%s", tok);
            for (int i = len; i < col_width; i++)
                string_buffer_append(sb, ' ');
            nr_chars += col_width;
        }

        string_buffer_append(sb, '\n');

        char *tab_complete_str = string_buffer_to_string(sb);
        vx_console_printf(vc, "%s", tab_complete_str);
        free(tab_complete_str);
    } else {
        vx_console_printf(vc, "no completions available\n");
    }
}

static int vx_console_on_event(vx_layer_t *vl, const vx_event_t *ev, void *user)
{
    // check whether we are already in console mode
    // lock out keyboard commands until hitting either enter or escape
    vx_console_t *vc = user;
//    int shift = ev->flags & VX_EVENT_FLAGS_SHIFT;

    int result = 0;

    if (ev->type != VX_EVENT_KEY_DOWN
        && ev->type != VX_EVENT_KEY_PRESSED
        && ev->type != VX_EVENT_KEY_UP)
        return result;

    vx_lock();

    if (!vc->has_keyboard_focus) {
        if (ev->u.key.key_code == ':') {
            vc->has_keyboard_focus = 1;
            result = 1;
            if (vc->current_commands) {
                zarray_vmap(vc->current_commands, free);
                zarray_destroy(vc->current_commands);
            }

            // create a copy of the history plus a new line.
            vc->current_commands = zarray_create(sizeof(char*));
            for (int i = 0; i < zarray_size(vc->command_history); i++) {
                char *t;
                zarray_get(vc->command_history, i, &t);
                t = strdup(t);
                zarray_add(vc->current_commands, &t);
            }
            char *t = strdup("");
            zarray_add(vc->current_commands, &t);
            vc->current_commands_idx = zarray_size(vc->current_commands) - 1;
            vc->cursor_position = 0;

            draw_console(vc);

        }

        goto done;
    }

    if (ev->type == VX_EVENT_KEY_DOWN) {

        switch (ev->u.key.key_code) {
            case VX_KEY_ESC: {
                // break out of focus and clear line with escape
                vc->has_keyboard_focus = 0;
                break;
            }

            case '\t': {
                char *cmd;
                zarray_get(vc->current_commands, vc->current_commands_idx, &cmd);

                // get completions for the portion of the command between the beginning and the
                // cursor position.
                char *cmdprefix = strdup(cmd);
                cmdprefix[vc->cursor_position] = 0;

                zarray_t *completions = (vc->console_tab)(vc, vl, cmdprefix, vc->user);
                handle_tab_completions(vc, completions);
                free(cmdprefix);

                zarray_vmap(completions, free);
                zarray_destroy(completions);
                break;
            }

            case VX_KEY_LEFT: {
                if (vc->cursor_position > 0)
                    vc->cursor_position--;
                break;
            }

            case VX_KEY_RIGHT: {
                char *cmd;
                zarray_get(vc->current_commands, vc->current_commands_idx, &cmd);
                int cmdlen = strlen(cmd);

                if (vc->cursor_position < cmdlen)
                    vc->cursor_position++;
                break;
            }

            case VX_KEY_BACKSPACE: {
	      if (vc->cursor_position > 0) {
                    char *cmd;
                    zarray_get(vc->current_commands, vc->current_commands_idx, &cmd);

		    int chars_to_delete = 1;
		    if (ev->flags & VX_EVENT_FLAGS_ALT) {
		      while (chars_to_delete < vc->cursor_position && isalnum(cmd[-1 + vc->cursor_position - chars_to_delete]))
			chars_to_delete++;
		    }

                    // copy every character except the one before the cursor.
                    int inpos = 0, outpos = 0, inlen = strlen(cmd);
                    for (int inpos = 0; inpos < inlen; inpos++) {
		      if (inpos < vc->cursor_position - chars_to_delete || inpos >= vc->cursor_position)
                            cmd[outpos++] = cmd[inpos];
                    }
                    cmd[outpos] = 0;
                    vc->cursor_position -= chars_to_delete;
                }
                break;
            }

            case VX_KEY_UP: {
                if (vc->current_commands_idx - 1 >= 0)
                    vc->current_commands_idx--;

                char *cmd;
                zarray_get(vc->current_commands, vc->current_commands_idx, &cmd);
                int cmdlen = strlen(cmd);
                vc->cursor_position = cmdlen;
                break;
            }

            case VX_KEY_DOWN: {
                int s = zarray_size(vc->current_commands);
                if (vc->current_commands_idx + 1 < s)
                    vc->current_commands_idx++;

                char *cmd;
                zarray_get(vc->current_commands, vc->current_commands_idx, &cmd);
                int cmdlen = strlen(cmd);
                vc->cursor_position = cmdlen;

                break;
            }
        }
    }

    if (ev->type == VX_EVENT_KEY_PRESSED) {

        switch (ev->u.key.key_code) {
            case 1: { // control-A
                vc->cursor_position = 0;
                break;
            }

            case 4: { // control-D
                // delete the character *at* the cursor position
                char *cmd;
                zarray_get(vc->current_commands, vc->current_commands_idx, &cmd);
                // copy every character except the one before the cursor.
                int inpos = 0, outpos = 0, inlen = strlen(cmd);

                if (vc->cursor_position < inlen) {
                    for (int inpos = 0; inpos < inlen; inpos++) {
                        if (inpos != vc->cursor_position)
                            cmd[outpos++] = cmd[inpos];
                    }
                    cmd[outpos] = 0;
                }

                break;
            }

            case 5: { // control-E
                char *cmd;
                zarray_get(vc->current_commands, vc->current_commands_idx, &cmd);
                int cmdlen = strlen(cmd);
                vc->cursor_position = cmdlen;
                break;
            }

            case 11: { // control-K, delete to end of line.
                char *cmd;
                zarray_get(vc->current_commands, vc->current_commands_idx, &cmd);
                cmd[vc->cursor_position] = 0;
                break;
            }

            case '\r':
            case '\n': {
                // execute command for enter and release focus
                char *cmd;
                zarray_get(vc->current_commands, vc->current_commands_idx, &cmd);

                vc->has_keyboard_focus = 0;
                if (strlen(cmd) > 0) {
                    char *h = strdup(cmd);
                    zarray_add(vc->command_history, &h);
                    vx_unlock();
                    (vc->console_command)(vc, vl, cmd, vc->user);
                    vx_lock();
                }

                break;
            }

            default: {
                // all other commands get inserted into current line
                if (ev->u.key.key_code >= ' ' && ev->u.key.key_code <= '~') {
                    char *cmd;
                    zarray_get(vc->current_commands, vc->current_commands_idx, &cmd);

                    // copy every character plus the new character at the cursor position
                    int inpos = 0, outpos = 0, inlen = strlen(cmd);

                    // make more space for a new char (plus \0)
                    char *cmdout = malloc(inlen + 2);

                    for (int inpos = 0; inpos <= inlen; inpos++) {
                        if (inpos == vc->cursor_position)
                            cmdout[outpos++] = ev->u.key.key_code;
                        if (inpos < inlen)
                            cmdout[outpos++] = cmd[inpos];
                    }
                    cmdout[outpos] = 0;
                    zarray_set(vc->current_commands, vc->current_commands_idx, &cmdout, NULL);
                    free(cmd);

                    vc->cursor_position++;
                }

                break;
            }
        }
    }

    draw_console(vc); // draw current state
    result = 1;

  done:
    vx_unlock();

    return result;
}

void vx_console_setup_layer(vx_console_t *vc, vx_layer_t *vl)
{
    vx_layer_add_event_handler(vl, vx_console_on_event, -1000, vc);
}

static void vx_console_print(vx_console_t *vc, const char *str)
{
    vx_lock();

    if (zarray_size(vc->output) == vc->max_lines)
        zarray_remove_index(vc->output, 0, 0);

    char *copied_str = strdup(str);
    zarray_add(vc->output, &copied_str);

    vx_unlock();

    draw_console(vc);
}

void vx_console_printf(vx_console_t *vc, const char *fmt, ...) {
    assert(vc != NULL);
    assert(fmt != NULL);

    va_list args;

    va_start(args,fmt);
    char *buf = vsprintf_alloc(fmt, args);
    va_end(args);

    vx_console_print(vc, buf);
    free(buf);
}

vx_console_t*
vx_console_create(vx_buffer_t *draw_buffer,
                  void (*console_command)(vx_console_t *vc, vx_layer_t * vl, const char *cmd, void *user),
                  zarray_t* (*console_tab)(vx_console_t *vc, vx_layer_t * vl, const char *cmd, void *user),
                  void *user)
{
    vx_console_t *vc = calloc(1, sizeof(vx_console_t));

    vc->draw_buffer = draw_buffer;
    vc->console_command = console_command;
    vc->console_tab = console_tab;
    vc->user = user;

    vc->has_keyboard_focus = 0;
    vc->output = zarray_create(sizeof(char*));
    vc->command_history = zarray_create(sizeof(char*));

    vc->prompt = strdup("<<left,#ffff00>>:");
    vc->format_cursor = strdup("<<left,invert,#ffffff>>");
    vc->format_active_command = strdup("<<left,#ffffff>>");
    vc->format_history_command = strdup("<<left,#888888>>");
    vc->format_tab_complete_command = strdup("<<left,#ff0000>>");

    vc->max_lines = 5;
    vc->command_history_maxsize = 50;
    vc->max_chars_width = 100;

    return vc;
}

// clear the output
void vx_console_clear(vx_console_t *vc)
{
    for (int i = 0; i < zarray_size(vc->output); i++) {
        char *line;
        zarray_get(vc->output, i, &line);
        free(line);
    }
    zarray_clear(vc->output);

    draw_console(vc);
}
