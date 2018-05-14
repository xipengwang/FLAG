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

#include <assert.h>
#include <ncurses.h>
#include <unistd.h>
#include <stdio.h>
#include <fcntl.h>
#include <pthread.h>
#include <lcm/lcm.h>

#include "common/string_util.h"
#include "common/time_util.h"
#include "common/zarray.h"

#include "procman_core.h"

#include "lcmtypes/procman_command_t.h"
#include "lcmtypes/procman_output_t.h"
#include "lcmtypes/procman_process_list_t.h"
#include "lcmtypes/procman_process_t.h"
#include "lcmtypes/procman_status_list_t.h"
#include "lcmtypes/procman_status_t.h"

#define PAIR_YELLOW 1
#define PAIR_RED 2

typedef struct _procman_ncurses_gui procman_ncurses_gui_t;
struct _procman_ncurses_gui
{
    lcm_t                  *lcm;
    procman_process_list_t_subscription_t    *plsub;
    procman_status_list_t_subscription_t     *slsub;

    uint8_t                 running;

    int64_t                 current_init_utime;
    uint32_t                max_proc_id;

    pthread_mutex_t         process_list_mutex;
    procman_process_list_t *process_list;

    pthread_mutex_t         ncurses_mutex;
    uint8_t                 colors;
};

static procman_ncurses_gui_t global_png;

static void
process_handler(const lcm_recv_buf_t *rbuf,
                const char* channel,
                const procman_process_list_t *msg,
                void *usr)
{
    procman_ncurses_gui_t *png = (procman_ncurses_gui_t*) usr;

    // ignore old controllers
    if (msg->init_utime < png->current_init_utime)
        return;

    // switch to newly-inited controller
    if (msg->init_utime > png->current_init_utime)
        png->current_init_utime = msg->init_utime;

    // update global process list
    pthread_mutex_lock(&png->process_list_mutex);
    if (png->process_list != NULL)
        procman_process_list_t_destroy(png->process_list);
    png->process_list = procman_process_list_t_copy(msg);
    pthread_mutex_unlock(&png->process_list_mutex);

    // draw
    pthread_mutex_lock(&png->ncurses_mutex);
    pthread_mutex_lock(&png->process_list_mutex);

    mvprintw( 1, 1, "%lu: Process list message received", msg->utime);

    if (png->colors && msg->exit) attron(COLOR_PAIR(PAIR_RED));
    mvprintw( 1, 49, "(system exit: %s)", msg->exit ? "true!" : "false");
    if (png->colors && msg->exit) attron(COLOR_PAIR(PAIR_RED));

    if (png->colors) attron(COLOR_PAIR(PAIR_YELLOW));
    mvprintw( 2, 1, "  id name                           cmd                                                          host      autorestart restartdelay enabled");
    if (png->colors) attroff(COLOR_PAIR(PAIR_YELLOW));

    for (int i=0; i < msg->nprocs; i++) {
        procman_process_t process = msg->processes[i];

        if (process.procid > png->max_proc_id)
            png->max_proc_id = process.procid;

        char *shortname = strndup(process.name,    30);
        char *shortcmd  = strndup(process.cmdline, 60);
        char *shorthost = strndup(process.host,    10);
        int row = 3+process.procid;

        mvprintw( row, 1,
                  "%4d %-30s %-60s %-10s",
                  process.procid,
                  shortname,
                  shortcmd,
                  shorthost);

        if (png->colors && !process.auto_restart) attron(COLOR_PAIR(PAIR_RED));
        mvprintw( row, 108, "%-11s", process.auto_restart ? "true" : "false");
        if (png->colors && !process.auto_restart) attroff(COLOR_PAIR(PAIR_RED));

        mvprintw( row, 120, "%12d", process.restart_delay_ms);

        if (png->colors && !process.enabled) attron(COLOR_PAIR(PAIR_RED));
        mvprintw( row, 133, "%-7s", process.enabled ? "true" : "false");
        if (png->colors && !process.enabled) attroff(COLOR_PAIR(PAIR_RED));

        free(shortname);
        free(shortcmd);
        free(shorthost);
    }

    // clear lines from old controllers
    for (int i = msg->nprocs; i <= png->max_proc_id; i++) {
        mvprintw( 3+i,   1, "                                                                                                           ");
        mvprintw( 3+i, 108, "           ");
        mvprintw( 3+i, 120, "            ");
        mvprintw( 3+i, 133, "       ");
    }

    refresh();
    pthread_mutex_unlock(&png->process_list_mutex);
    pthread_mutex_unlock(&png->ncurses_mutex);
}

static void
status_handler(const lcm_recv_buf_t *rbuf,
               const char* channel,
               const procman_status_list_t *msg,
               void *usr)
{
    procman_ncurses_gui_t *png = (procman_ncurses_gui_t*) usr;

    // only listen to messages from the same controller as the process list
    if (msg->received_init_utime != png->current_init_utime)
        return;

    pthread_mutex_lock(&png->ncurses_mutex);
    pthread_mutex_lock(&png->process_list_mutex);
    mvprintw( 1, 145, "%lu: Status list message received", msg->utime);
    if (png->colors) attron(COLOR_PAIR(PAIR_YELLOW));
    mvprintw( 2, 145, "running     pid restarts lastexitcode");
    if (png->colors) attroff(COLOR_PAIR(PAIR_YELLOW));

    for (int i=0; i < msg->nprocs; i++) {
        procman_status_t status = msg->statuses[i];

        for (int j = 0; j < png->process_list->nprocs; j++) {
            procman_process_t process = png->process_list->processes[j];

            if (status.procid != process.procid)
                continue;

            // only draw if this status message is from the hostname that this
            // process is supposed to run on (localhost always draws)
            if (strcmp(process.host, "localhost") != 0 && // not supposed to run on localhost
                strcmp(process.host, msg->host) != 0)     // not supposed to run on the reported machine
                continue;

            if (status.procid > png->max_proc_id)
                png->max_proc_id = status.procid;

            int row = 3+status.procid;

            if (png->colors && !status.running) attron(COLOR_PAIR(PAIR_RED));
            mvprintw( row, 145, "%-7s", status.running ? "true" : "false");
            if (png->colors && !status.running) attroff(COLOR_PAIR(PAIR_RED));

            mvprintw( row, 153, "%7d %8d %12d",
                      status.pid,
                      status.restarts,
                      status.last_exit_code
                     );
        }
    }

    // clear lines from old controllers
    for (int i = msg->nprocs; i <= png->max_proc_id; i++) {
        mvprintw( 3+i, 145, "       ");
        mvprintw( 3+i, 153, "                             ");
    }

    refresh();
    pthread_mutex_unlock(&png->process_list_mutex);
    pthread_mutex_unlock(&png->ncurses_mutex);
}

void publish_startstop(procman_ncurses_gui_t *png, int procid, int enabled)
{
    pthread_mutex_lock(&png->process_list_mutex);

    if (png->process_list != NULL)
    {
        procman_publish_procman_command(png->lcm,
                                        utime_now(), // blacklist-ignore
                                        png->process_list->utime,
                                        png->process_list->init_utime,
                                        procid,
                                        enabled);
    }

    pthread_mutex_unlock(&png->process_list_mutex);
}

void *loop_lcmhandle(void *td)
{
    procman_ncurses_gui_t *png = (procman_ncurses_gui_t*) td;

    while (png->running)
    {
        lcm_handle(png->lcm);
    }

    pthread_exit(NULL);
}

void *loop_stdin(void *td)
{
    procman_ncurses_gui_t *png = (procman_ncurses_gui_t*) td;

    char buf[80], cmd[80], spaces[80];
    memset(cmd, 0, sizeof(cmd));
    memset(spaces, ' ', sizeof(spaces));

    int offset = 0;
    int newline = 0;
    while (png->running)
    {
        memset(buf, 0, sizeof(buf));
        int n = read(STDIN_FILENO, buf, sizeof(buf));

        if (png->max_proc_id == 0)
            continue;

        int blankline  = 4 + png->max_proc_id;
        int outputline = 5 + png->max_proc_id;
        int inputline  = 6 + png->max_proc_id;

        for (int i=0; i < n && offset < sizeof(cmd); i++) {

            char c = buf[i];

            // first character must be a ':'
            if (offset == 0) {
                if (c == ':') {
                    cmd[offset] = c;
                    offset++;
                    newline = 0;
                }
            }
            // if we're past the first character
            else {
                if (c == 127) {                 // backspace
                    if (offset > 0) {
                        offset--;
                        cmd[offset] = '\0';
                    }
                    newline = 0;
                }
                else if (c == '\r') {           // newline tells us to parse the cmd
                    newline = 1;
                }
                else if (c == 32 ||             // space
                        (c >= 48 && c <= 57) || // number
                        (c >= 65 && c <= 90) || // capital letter
                        (c >= 97 && c <= 122))  // lowercase letter
                {
                    cmd[offset] = c;
                    offset++;
                    newline = 0;
                }
            }
        }

        pthread_mutex_lock(&png->ncurses_mutex);
        mvprintw(blankline, 0, "%s", spaces);
        mvprintw(inputline, 0, "%s", spaces);
        mvprintw(inputline, 0, "%s", cmd);

        // only parse if we just got a newline
        if (newline)
        {
            zarray_t *tokens = str_split(cmd+1, " "); // copy after the :
            int ntokens = zarray_size(tokens);

            {
                int clear = 0;

                if (ntokens == 1) {

                    char *tok0 = NULL;
                    zarray_get(tokens, 0, &tok0);

                    if (strcmp("exit", tok0) == 0) {
                        mvprintw(outputline, 0, "Exiting");
                        clear = 1;
                        png->running = 0;
                    }
                    else if (strcmp("quit", tok0) == 0) {
                        mvprintw(outputline, 0, "Exiting");
                        clear = 1;
                        png->running = 0;
                    }
                    else {
                        mvprintw(outputline, 0, "Error: Argument #1 '%s' not recognized", tok0);
                        clear = 1;
                    }
                }
                else if (ntokens == 2) {

                    char *tok0 = NULL;
                    zarray_get(tokens, 0, &tok0);

                    if (strcmp("enable", tok0) == 0 ||
                        strcmp("Enable", tok0) == 0 ||
                        strcmp("Start",  tok0) == 0 ||
                        strcmp("start",  tok0) == 0)
                    {
                        char *tok1 = NULL;
                        zarray_get(tokens, 1, &tok1);

                        char *endptr = 0;
                        long int procid = strtol(tok1, &endptr, 10);

                        mvprintw(outputline, 0, "%s", spaces);

                        if (tok1 != endptr) {// did we actually get a number?
                            mvprintw(outputline, 0, "Enable #%d", (int) procid);
                            publish_startstop(png, (int) procid, 1); // 1: start
                        }
                        else
                            mvprintw(outputline, 0, "Error: Argument #2 '%s' not a number", tok1);

                        clear = 1;
                    }
                    else if (strcmp("disable", tok0) == 0 ||
                             strcmp("Disable", tok0) == 0 ||
                             strcmp("stop",    tok0) == 0 ||
                             strcmp("Stop",    tok0) == 0)
                    {
                        char *tok1 = NULL;
                        zarray_get(tokens, 1, &tok1);

                        char *endptr = 0;
                        long int procid = strtol(tok1, &endptr, 10);

                        mvprintw(outputline, 0, "%s", spaces);

                        if (tok1 != endptr) {// did we actually get a number?
                            mvprintw(outputline, 0, "Disable #%d", (int) procid);
                            publish_startstop(png, (int) procid, 0); // 0: stop
                        }
                        else
                            mvprintw(outputline, 0, "Error: Argument #2 '%s' not a number", tok1);

                        clear = 1;
                    }
                    else {
                        mvprintw(outputline, 0, "Error: Argument #1 '%s' not recognized", tok0);
                        clear = 1;
                    }
                }
                else {
                    mvprintw(outputline, 0, "Error: Too many arguments in '%s'", cmd+1);
                    clear = 1;
                }

                if (clear) {
                    memset(cmd, '\0', sizeof(cmd));
                    offset = 0;
                    newline = 0;
                }
            }

            zarray_vmap(tokens, free);
            zarray_destroy(tokens);
        }
        else {
            mvprintw(outputline, 0, "%s", spaces);
        }

        refresh();
        pthread_mutex_unlock(&png->ncurses_mutex);
    }

    pthread_exit(NULL);
}

void cleanup(procman_ncurses_gui_t *png)
{
    if (png->lcm != NULL) {
        if (png->plsub != NULL) procman_process_list_t_unsubscribe(png->lcm, png->plsub);
        if (png->slsub != NULL) procman_status_list_t_unsubscribe(png->lcm, png->slsub);
        lcm_destroy(png->lcm);
    }

    pthread_mutex_lock(&png->process_list_mutex);
    if (png->process_list != NULL) procman_process_list_t_destroy(png->process_list);
    pthread_mutex_unlock(&png->process_list_mutex);

    if (pthread_mutex_destroy(&png->process_list_mutex) != 0) {
        printf("Error: Failed to destroy pthread mutex\n");
        exit(1);
    }
}

int main(int argc, char **argv)
{
    setlinebuf(stdout);
    setlinebuf(stderr);

    procman_ncurses_gui_t *png = &global_png;

    memset(png, 0, sizeof(procman_ncurses_gui_t));
    png->lcm = NULL;
    png->running = 1;
    png->current_init_utime = 0;
    png->max_proc_id = 0;
    png->process_list = NULL;
    if (pthread_mutex_init(&png->ncurses_mutex, NULL) != 0) {
        printf("Error: Failed to init ncurses_mutex\n");
        exit(1);
    }
    if (pthread_mutex_init(&png->process_list_mutex, NULL) != 0) {
        printf("Error: Failed to init process_list_mutex\n");
        exit(1);
    }

    // initialize screen for ncurses
    initscr();

    // hide cursor
    curs_set(0);

    png->colors = has_colors();
    if (png->colors) {
        start_color();
        init_pair(PAIR_YELLOW, COLOR_YELLOW, COLOR_BLACK);
        init_pair(PAIR_RED, COLOR_RED, COLOR_BLACK);
    }

    png->lcm = lcm_create(NULL);
    if (png->lcm == NULL) {
        printf("Error: Could not create LCM\n");
        exit(1);
    }

    png->plsub = procman_process_list_t_subscribe(png->lcm, "PROCMAN_PROCESS_LIST", &process_handler, (void*) png);
    png->slsub = procman_status_list_t_subscribe(png->lcm, "PROCMAN_STATUS_LIST", &status_handler, (void*) png);

    pthread_t threads[2];

    pthread_attr_t attr;
    pthread_attr_init(&attr);
    pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);

    int rc;
    rc = pthread_create(&threads[0], &attr, &loop_lcmhandle, (void*) png);
    if (rc != 0) {
        printf("ERROR: failed to start lcm_handle thread (code %d)\n", rc);
        exit(1);
    }
    rc = pthread_create(&threads[1], &attr, &loop_stdin, (void*) png);
    if (rc != 0) {
        printf("ERROR: failed to start stdin thread (code %d)\n", rc);
        exit(1);
    }
    pthread_attr_destroy(&attr);

    void *status;
    rc = pthread_join(threads[0], &status);
    if (rc) {
        printf("ERROR: failed to join loop_lcmhandle thread (code %d)\n", rc);
        exit(1);
    }
    rc = pthread_join(threads[1], &status);
    if (rc) {
        printf("ERROR: failed to join loop_stdin thread (code %d)\n", rc);
        exit(1);
    }

    cleanup(png);

    endwin();

    pthread_exit(NULL);
}

