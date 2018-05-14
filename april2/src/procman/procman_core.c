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

#include <strings.h>
#include <assert.h>
#include <lcm/lcm.h>

#include "common/config.h"
#include "common/string_util.h"
#include "common/zarray.h"
#include "common/sys_util.h"

#include "lcmtypes/procman_command_t.h"
#include "lcmtypes/procman_output_t.h"
#include "lcmtypes/procman_process_list_t.h"
#include "lcmtypes/procman_process_t.h"
#include "lcmtypes/procman_status_list_t.h"
#include "lcmtypes/procman_status_t.h"

#include "procman_core.h"

int procman_publish_process_status_list(lcm_t *lcm, int64_t current_utime,
                                        int64_t received_utime, int64_t received_init_utime,
                                        const char *hostname, int nprocesses, proc_status_t *processes)
{
    procman_status_list_t *status_list = malloc(sizeof(procman_status_list_t));
    if (status_list == NULL)
        return -1;

    status_list->utime = current_utime;

    status_list->received_utime = received_utime;
    status_list->received_init_utime = received_init_utime;

    status_list->host = strdup(hostname);

    status_list->nprocs = nprocesses;

    status_list->statuses = malloc(nprocesses * sizeof(procman_status_t));
    if (status_list->statuses == NULL) {
        free(status_list->host);
        free(status_list);
        return -1;
    }

    for (int i=0; i < nprocesses; i++) {

        procman_status_t *status = &status_list->statuses[i];
        proc_status_t *proc_status = &processes[i];

        status->procid = proc_status->procid;
        assert(status->procid == i);

        status->running = proc_status->running;

        status->pid = proc_status->pid;

        status->restarts = proc_status->restarts;

        status->last_exit_code = proc_status->last_exit_code;
    }

    procman_status_list_t_publish(lcm, "PROCMAN_STATUS_LIST", status_list);

    free(status_list->statuses);
    free(status_list->host);
    free(status_list);

    return 0;
}

int procman_publish_process_list(lcm_t *lcm, int nprocesses, proc_info_t *processes,
                                 int64_t current_utime, int64_t init_utime, int exit)
{
    procman_process_list_t *process_list = malloc(sizeof(procman_process_list_t));
    if (process_list == NULL)
        return -1;

    process_list->utime = current_utime;
    process_list->init_utime = init_utime;

    process_list->exit = exit;

    process_list->nprocs = nprocesses;

    process_list->processes = malloc(nprocesses * sizeof(procman_process_t));
    if (process_list->processes == NULL) {
        free(process_list);
        return -1;
    }

    for (int i=0; i < nprocesses; i++) {

        procman_process_t *process = &process_list->processes[i];
        proc_info_t *proc_info = &processes[i];

        process->procid = i;
        assert(i == proc_info->procid);

        process->name = proc_info->name;
        process->cmdline = strdup(proc_info->cmd);
        process->host = strdup(proc_info->hostname);

        process->auto_restart = proc_info->autorestart;
        process->restart_delay_ms = proc_info->restart_delay_ms;
        process->group = strdup(proc_info->group);

        process->enabled = proc_info->enabled;
    }

    procman_process_list_t_publish(lcm, "PROCMAN_PROCESS_LIST", process_list);

    for (int i=0; i < nprocesses; i++) {
        procman_process_t *process = &process_list->processes[i];
        free(process->cmdline);
        free(process->host);
    }

    free(process_list->processes);
    free(process_list);

    return 0;
}

int procman_publish_output(lcm_t *lcm, int64_t current_utime, int64_t received_init_utime,
                           const char *hostname, int32_t procid, int streamtype, char *data)
{
    procman_output_t *output = malloc(sizeof(procman_output_t));
    if (output == NULL)
        return -1;

    output->utime = current_utime;

    output->received_init_utime = received_init_utime;

    output->host = strdup(hostname);
    output->procid = procid;

    output->stream = streamtype;

    output->data = data;

    procman_output_t_publish(lcm, "PROCMAN_OUTPUT", output);

    free(output->host);
    free(output);

    return 0;
}

int procman_publish_procman_command(lcm_t *lcm, int64_t current_utime,
                                    int64_t received_utime, int64_t received_init_utime,
                                    int32_t procid, int enabled)
{
    procman_command_t *command = malloc(sizeof(procman_command_t));
    if (command == NULL)
        return -1;

    command->utime = current_utime;

    command->received_utime = received_utime;
    command->received_init_utime = received_init_utime;
    command->procid = procid;
    command->enabled = enabled;

    procman_command_t_publish(lcm, "PROCMAN_COMMAND", command);

    free(command);

    return 0;
}

zarray_t *procman_load_process_names(config_t *config)
{
    const zarray_t *keys = config_get_keys(config);

    zarray_t *process_names = zarray_create(sizeof(char*));

    // get all process names containing "<process name>.procman_process = true"
    for (int i=0; i < zarray_size(keys); i++) {

        char* key = NULL;
        zarray_get(keys, i, &key);

        // key must include "procman_process" field
        if (!str_ends_with(key, ".procman_process")) {
            continue;
        }

        // key must be valid boolean
        // if set to false, process entry will be ignored
        if (!config_require_boolean(config, key)) {
            continue;
        }

        // copy process name
        int index = str_indexof(key, ".");
        if (index <= 0) {
            printf("Warning: invalid process entry '%s'\n", key);
            continue;
        }

        char *name = malloc(index+1);
        memcpy(name, key, index);
        name[index] = '\0';

        // store name
        zarray_add(process_names, &name);
    }

    zarray_sort(process_names, zstrcmp);

    return process_names;
}

/** Loads the proc_info_t structs from a config file. Caller responsible for freeing
  * result.
  */
proc_info_t *procman_load_process_info(config_t *config, uint32_t *_nprocesses)
{
    if (config == NULL)
        return NULL;

    ////////////////////////////////////////
    // get unique process names (e.g. proc3)
    zarray_t *process_names = procman_load_process_names(config);
    uint32_t nprocesses = zarray_size(process_names);
    if (nprocesses == 0) {
        zarray_vmap(process_names, free);
        zarray_destroy(process_names);
        return NULL;
    }

    ////////////////////////////////////////
    // load each process's settings
    proc_info_t *processes = calloc(nprocesses, sizeof(proc_info_t));

    char *key;
    for (int i = 0; i < nprocesses; i++) {

        char *proc_name = NULL;
        zarray_get(process_names, i, &proc_name);

        proc_info_t *proc_info = &processes[i];

        // procid
        proc_info->procid = i;

        // commmand
        key = sprintf_alloc("%s.cmd", proc_name);
        char *cmd = strdup(config_require_string(config, key));

        // expand controller environment variables
        if (str_indexof(cmd, "$controller:") >= 0) {
            // escape all variables to avoid unintentional expansion
            char *tmp1 = str_replace(cmd, "$", "\\$");
            free(cmd);
            // expand $controller: variables
            char *tmp2 = str_replace(tmp1, "\\$controller:", "$");
            free(tmp1);
            tmp1 = expand_environment_variables(tmp2);
            free(tmp2);
            // unescape non-controller variables
            cmd = str_replace(tmp1, "\\$", "$");
            free(tmp1);
        }
        proc_info->cmd = cmd;
        free(key);

        // process name
        proc_info->name = strdup(proc_name);

        // hostname
        key = sprintf_alloc("%s.host", proc_name);
        proc_info->hostname = config_require_string(config, key);
        free(key);

        // group
        key = sprintf_alloc("%s.group", proc_name);
        proc_info->group = config_get_string(config, key, "DEFAULT");
        free(key);

        // auto-start
        key = sprintf_alloc("%s.auto-start", proc_name);
        proc_info->autostart = config_require_boolean(config, key);
        free(key);

        // auto-restart
        key = sprintf_alloc("%s.auto-restart", proc_name);
        proc_info->autorestart = config_require_boolean(config, key);
        free(key);

        // restart-delay-ms
        key = sprintf_alloc("%s.restart-delay-ms", proc_name);
        proc_info->restart_delay_ms = config_require_int(config, key);
        free(key);

        // initialize other variables
        proc_info->enabled = 0;
        proc_info->status_received_utime = 0;
        proc_info->status_running = 0;
        proc_info->status_pid = -1;
        proc_info->status_restarts = 0;
        proc_info->status_last_exit_code = 0;
    }

    zarray_vmap(process_names, free);
    zarray_destroy(process_names);
    *_nprocesses = nprocesses;

    return processes;
}

void procman_process_info_destroy(uint32_t nprocesses, proc_info_t *processes)
{
    for (int i=0; i < nprocesses; i++)
    {
        proc_info_t *p = &processes[i];
        if (p->cmd != NULL) free(p->cmd);
        if (p->name != NULL) free(p->name);
    }

    free(processes);
}
