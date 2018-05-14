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

#ifndef _PROCMAN_CORE_H
#define _PROCMAN_CORE_H

#include <pthread.h>
#include <lcm/lcm.h>
#include "common/config.h"

#ifdef __cplusplus
extern "C" {
#endif

// Procman controller process info structure
typedef struct _proc_info proc_info_t;
struct _proc_info
{
    int32_t     procid;

    char       *name;
    char       *cmd;

    const char *hostname;
    uint8_t     autostart;
    uint8_t     autorestart;
    uint32_t    restart_delay_ms;
    const char *group;
    uint8_t     enabled; // *should* the process be running?

    // status information received from controlling daemon
    int64_t     status_received_utime; // time for last status list
    uint8_t     status_running; // is it alive?
    int32_t     status_pid;
    int32_t     status_restarts;
    int32_t     status_last_exit_code;
};

// Procman daemon process info structure
typedef struct _proc_status proc_status_t;
struct _proc_status
{
    int32_t     procid;

    uint8_t     running;

    uint8_t     died;
    int64_t     time_of_death;
    int32_t     last_exit_code; // negative for signals
    int32_t     restarts;
    int32_t     shutdown_attempts;
    int64_t     shutdown_utime;

    pid_t       pid;
    pthread_t   reader_thread;
};

////////////////////////////////////////
// lcm utility functions

int procman_publish_process_status_list(lcm_t *lcm, int64_t current_utime,
                                        int64_t received_utime, int64_t received_init_utime,
                                        const char *hostname, int nprocesses, proc_status_t *processes);

int procman_publish_process_list(lcm_t *lcm, int nprocesses, proc_info_t *processes,
                                 int64_t current_utime, int64_t init_utime, int exit);

int procman_publish_output(lcm_t *lcm, int64_t current_utime, int64_t received_init_utime,
                           const char *hostname, int32_t procid, int streamtype, char *data);

int procman_publish_procman_command(lcm_t *lcm, int64_t current_utime,
                                    int64_t received_utime, int64_t received_init_utime,
                                    int32_t procid, int enabled);

////////////////////////////////////////
// core utilities

/**
 * Parse a procman config file and find all process entry names. A number of
 * fields are required:
 *     procman_process: (boolean) declare this config 'key' to be a process entry
 *     cmd: (string) command to run
 *     host: (string) hostname of the computer to run this process
 *     group: (string) name of 'process group' for GUI display
 *     auto-start: (boolean) start this process on launch?
 *     auto-restart: (boolean) restart this process if it dies?
 *     restart-delay-ms: (int) time to wait before restarting this process
 *
 * If the procman_process field is present but 'false', the process will be skipped.
 * This is a useful way to prevent GUI display of undesired processes
 */
zarray_t *procman_load_process_names(config_t *config);

/**
 * Load proc_info_t structs for all processes defined by the
 * procman_load_process_names() function
 */
proc_info_t *procman_load_process_info(config_t *config, uint32_t *_nprocesses);

void procman_process_info_destroy(uint32_t nprocesses, proc_info_t *processes);

#ifdef __cplusplus
}
#endif

#endif
