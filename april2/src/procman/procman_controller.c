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
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/wait.h>
#include <errno.h>
#include <string.h>
#include <fcntl.h>
#include <stdarg.h>
#include <sys/stat.h>
#include <signal.h>
#include <pthread.h>
#include <lcm/lcm.h>
#include <inttypes.h>

#include "common/config.h"
#include "common/getopt.h"
#include "common/string_util.h"
#include "common/time_util.h"
#include "common/notice.h"

#include "lcmtypes/procman_command_t.h"
#include "lcmtypes/procman_output_t.h"
#include "lcmtypes/procman_process_list_t.h"
#include "lcmtypes/procman_process_t.h"
#include "lcmtypes/procman_status_list_t.h"
#include "lcmtypes/procman_status_t.h"

#include "procman_core.h"

typedef struct _procman_controller procman_controller_t;
struct _procman_controller
{
    config_t           *config;
    lcm_t              *lcm;
    procman_status_list_t_subscription_t *slsub;
    procman_command_t_subscription_t *csub;

    int64_t             init_utime;

    uint32_t            nprocesses;
    proc_info_t        *processes;

    volatile uint8_t    controller_running;
    volatile uint8_t    lcm_running;
    pthread_mutex_t     process_mutex;
};

static procman_controller_t global_pmc;

void signal_handler(int sig)
{
    procman_controller_t *pmc = &global_pmc;

    switch(sig) {
        case SIGHUP:
            nwarnf("procman_controller: hangup signal caught. Doing nothing.\n");
            break;
        case SIGINT:
            nwarnf("procman_controller: interrupt signal caught. Beginning shutdown process.\n");
            pmc->controller_running = 0;
            break;
        case SIGTERM:
            nwarnf("procman_controller: terminate signal caught. Beginning shutdown process.\n");
            pmc->controller_running = 0;
            break;
    }
}

void set_signal_handlers()
{
    signal(SIGCHLD, SIG_IGN); /* ignore child */
    signal(SIGTSTP, SIG_IGN); /* ignore tty signals */
    signal(SIGTTOU, SIG_IGN);
    signal(SIGTTIN, SIG_IGN);
    signal(SIGINT,  signal_handler); // catch ctrl+c
    signal(SIGHUP,  signal_handler); // catch hangup signal
    signal(SIGTERM, signal_handler); // catch kill signal
}

void *loop_lcmhandle(void *td)
{
    procman_controller_t *pmc = (procman_controller_t*) td;

    while (pmc->lcm_running)
    {
        lcm_handle(pmc->lcm);
    }

    ninfof("Done in loop_lcmhandle\n");

    pthread_exit(NULL);
}

static void
command_handler(const lcm_recv_buf_t *rbuf,
                const char* channel,
                const procman_command_t *command,
                void *usr)
{
    procman_controller_t *pmc = (procman_controller_t*) usr;

    if (command->received_init_utime != pmc->init_utime)
        return;

    pthread_mutex_lock(&pmc->process_mutex);

    if (pmc->processes != NULL)
    {
        for (int i=0; i < pmc->nprocesses; i++) {
            proc_info_t *proc_info = &pmc->processes[i];

            if (proc_info->procid == command->procid) {
                proc_info->enabled = command->enabled;
                ninfof("Set 'enabled' for process %d to '%s'\n",
                       proc_info->procid, proc_info->enabled ? "true" : "false");
            }
        }
    }

    pthread_mutex_unlock(&pmc->process_mutex);
}

static void
status_list_handler(const lcm_recv_buf_t *rbuf,
                    const char* channel,
                    const procman_status_list_t *msg,
                    void *usr)
{
    procman_controller_t *pmc = (procman_controller_t*) usr;

    pthread_mutex_lock(&pmc->process_mutex);

    // only use status messages that are listening to us
    if (msg->received_init_utime == pmc->init_utime)
    {
        assert(pmc->nprocesses == msg->nprocs);

        for (int i=0; i < pmc->nprocesses; i++)
        {
            proc_info_t *proc_info = &pmc->processes[i];
            procman_status_t *status = &msg->statuses[i];
            assert(i == proc_info->procid);
            assert(i == status->procid);

            // if this process is supposed to run on localhost or is running on
            // the hostname specified, we'll pay attention to the status reported
            if (strcmp(proc_info->hostname, "localhost") != 0 && // not supposed to run on localhost
                strcmp(proc_info->hostname, msg->host) != 0)     // not supposed to run on the reported machine
                continue;

            // is this message stale?
            if (proc_info->status_received_utime > msg->received_utime) {
                nwarnf("procman_controller: warning: stale process status ignored\n");
                continue;
            }

            // update local status info for this process
            proc_info->status_received_utime = msg->received_utime;
            proc_info->status_running        = status->running;
            proc_info->status_pid            = status->pid;
            proc_info->status_restarts       = status->restarts;
            proc_info->status_last_exit_code = status->last_exit_code;
        }
    }

    pthread_mutex_unlock(&pmc->process_mutex);
}

uint8_t any_process_alive(procman_controller_t *pmc)
{
    int any_alive = 0;

    pthread_mutex_lock(&pmc->process_mutex);

    for (int i=0; i < pmc->nprocesses; i++)
    {
        proc_info_t *proc_info = &pmc->processes[i];

        if (proc_info->status_running)
            any_alive = 1;
    }

    pthread_mutex_unlock(&pmc->process_mutex);

    return any_alive;
}

void *loop_monitor_processes(void *td)
{
    procman_controller_t *pmc = (procman_controller_t*) td;
    int sleep_us = 1000 * config_get_int(pmc->config, "procman.monitoring_period_ms", 100);

    // set autostart processes to enabled
    pthread_mutex_lock(&pmc->process_mutex);
    for (int i=0; i < pmc->nprocesses; i++) {

        proc_info_t *proc_info = &pmc->processes[i];

        if (proc_info->autostart) proc_info->enabled = 1;
        else                      proc_info->enabled = 0;
    }
    pthread_mutex_unlock(&pmc->process_mutex);

    // publish status until we get a shutdown signal
    while (pmc->controller_running)
    {
        timeutil_usleep(sleep_us);

        pthread_mutex_lock(&pmc->process_mutex);
        int res = procman_publish_process_list(pmc->lcm,
                                               pmc->nprocesses,
                                               pmc->processes,
                                               utime_now(), // blacklist-ignore
                                               pmc->init_utime,
                                               0); // 0: don't shutdown
        pthread_mutex_unlock(&pmc->process_mutex);
        if (res != 0) {
            printf("Warning: Failed to publish process list (%d)\n", res);
        }
    }

    // beginning shutdown - set all processes to disabled
    for (int i=0; i < pmc->nprocesses; i++) {
        proc_info_t *proc_info = &pmc->processes[i];
        proc_info->enabled = 0;
    }

    // send shutdown message a few times
    int any_alive = any_process_alive(pmc);
    while (any_alive)
    {
        timeutil_usleep(sleep_us);

        pthread_mutex_lock(&pmc->process_mutex);
        int res = procman_publish_process_list(pmc->lcm,
                                               pmc->nprocesses,
                                               pmc->processes,
                                               utime_now(), // blacklist-ignore
                                               pmc->init_utime,
                                               1); // 1: *do* shutdown
        pthread_mutex_unlock(&pmc->process_mutex);
        if (res != 0) {
            nwarnf("Warning: Failed to publish process list (%d)\n", res);
        }

        any_alive = any_process_alive(pmc);
    }

    ninfof("Done in loop_monitor_processes\n");

    // lcm can quit now
    pmc->lcm_running = 0;

    pthread_exit(NULL);
}

void cleanup(procman_controller_t *pmc)
{
    if (pmc->lcm != NULL) {
        if (pmc->slsub != NULL) procman_status_list_t_unsubscribe(pmc->lcm, pmc->slsub);
        if (pmc->csub != NULL) procman_command_t_unsubscribe(pmc->lcm, pmc->csub);
        lcm_destroy(pmc->lcm);
    }

    pthread_mutex_lock(&pmc->process_mutex);
    if (pmc->processes != NULL)     procman_process_info_destroy(pmc->nprocesses, pmc->processes);
    pthread_mutex_unlock(&pmc->process_mutex);

    if (pmc->config != NULL)        config_destroy(pmc->config);
    if (pthread_mutex_destroy(&pmc->process_mutex) != 0) {
        nfailf("Failed to destroy pthread mutex\n");
        exit(EXIT_FAILURE);
    }
    notice_destroy();
}

void print_process_info(procman_controller_t *pmc)
{
    for (int i=0; i < pmc->nprocesses; i++)
    {
        proc_info_t *p = &pmc->processes[i];
        printf("Process %d\n", i);
        printf("    procid: %d\n", p->procid);
        printf("    name: '%s'\n", p->name);
        printf("    cmd: '%s'\n", p->cmd);
        printf("    hostname: '%s'\n", p->hostname);
        printf("    autostart: %d\n", p->autostart);
        printf("    autorestart: %d\n", p->autorestart);
        printf("    restart_delay_ms: %d\n", p->restart_delay_ms);
        printf("    enabled: %d\n", p->enabled);
        printf("    status_received_utime: %" PRIu64 "\n", p->status_received_utime);
        printf("    status_running: %d\n", p->status_running);
        printf("    status_pid: %d\n", p->status_pid);
        printf("    status_restarts: %d\n", p->status_restarts);
        printf("    status_last_exit_code: %d\n", p->status_last_exit_code);
    }
}

int main(int argc, char ** argv)
{
    setlinebuf(stdout);
    setlinebuf(stderr);
    notice_init(argc, argv, NULL);

    procman_controller_t *pmc = &global_pmc;

    memset(pmc, 0, sizeof(procman_controller_t));
    pmc->config = NULL;
    pmc->lcm = NULL;
    pmc->init_utime = utime_now(); // blacklist-ignore
    pmc->nprocesses = 0;
    pmc->processes = NULL;
    pmc->controller_running = 1;
    pmc->lcm_running = 1;
    if (pthread_mutex_init(&pmc->process_mutex, NULL) != 0) {
        nfailf("Failed to init pthread mutex\n");
        exit(EXIT_FAILURE);
    }

    ////////////////////////////////////////
    // argument handling
    getopt_t *gopt = getopt_create();
    getopt_add_bool(gopt, 'h',"help", 0,"Show this");
    getopt_add_string(gopt, 'c', "config", "", "Config file");
    getopt_add_bool(gopt, 'p',"print", 0,"Print parsed process info");

    // parse and print help
    if (!getopt_parse(gopt, argc, argv, 1)) {
        fatal("Unable to parse command-line arguments\n");
    }

    if (getopt_get_bool(gopt,"help")) {
        printf("Usage: %s [options]\n\n", argv[0]);
        getopt_do_usage(gopt);
        exit(EXIT_SUCCESS);
    }

    int print = getopt_get_bool(gopt, "print");
    const char *configpath = getopt_get_string(gopt, "config");
    if (configpath == NULL || strlen(configpath) == 0) {
        nfailf("No config path supplied\n");
        printf("Usage: %s [options]\n\n", argv[0]);
        getopt_do_usage(gopt);
        exit(EXIT_FAILURE);
    }

    ////////////////////////////////////////
    // load config file
    pmc->config = config_create_path(configpath);
    if (pmc->config == NULL) {
        nfailf("Failed to get config file at '%s'\n", configpath);
        exit(EXIT_FAILURE);
    }

    ////////////////////////////////////////
    // create process objects
    pthread_mutex_lock(&pmc->process_mutex);
    pmc->processes = procman_load_process_info(pmc->config, &pmc->nprocesses);
    pthread_mutex_unlock(&pmc->process_mutex);
    if (pmc->processes == NULL || pmc->nprocesses == 0) {
        config_destroy(pmc->config);
        nfailf("Error loading processes from config file. (nprocesses %d)\n",
                        pmc->nprocesses);
        exit(EXIT_FAILURE);
    }

    if (print)
        print_process_info(pmc);

    ////////////////////////////////////////
    // lcm
    pmc->lcm = lcm_create(NULL);
    if (pmc->lcm == NULL) {
        pthread_mutex_lock(&pmc->process_mutex);
        free(pmc->processes);
        pthread_mutex_unlock(&pmc->process_mutex);
        config_destroy(pmc->config);
        fatal("Failed to create LCM\n");
    }

    pmc->slsub = procman_status_list_t_subscribe(pmc->lcm, "PROCMAN_STATUS_LIST", &status_list_handler, (void*) pmc);
    pmc->csub = procman_command_t_subscribe(pmc->lcm, "PROCMAN_COMMAND", &command_handler, (void*) pmc);

    ////////////////////////////////////////
    // start threads
    pthread_t threads[2];

    set_signal_handlers();

    pthread_attr_t attr;
    pthread_attr_init(&attr);
    pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);

    if (pthread_create(&threads[0], &attr, &loop_monitor_processes, (void*) pmc) != 0) {
        cleanup(pmc);
        fatal("Failed to start monitor thread\n");
    }
    if (pthread_create(&threads[1], &attr, &loop_lcmhandle, (void*) pmc) != 0) {
        cleanup(pmc);
        fatal("Failed to start lcmhandle thread\n");
    }
    pthread_attr_destroy(&attr);

    void *status;
    // worker
    if (pthread_join(threads[0], &status) != 0) {
        cleanup(pmc);
        fatal("Failed to join monitor thread\n");
    }
    // lcm
    pthread_cancel(threads[1]);
    if (pthread_join(threads[1], &status) != 0) {
        cleanup(pmc);
        fatal("Failed to join lcmhandle thread\n");
    }

    getopt_destroy(gopt);

    cleanup(pmc);

    printf("Done in main\n");

    pthread_exit(NULL);
}
