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
#include <unistd.h>
#include <sys/wait.h>
#include <errno.h>
#include <string.h>
#include <fcntl.h>
#include <stdarg.h>
#include <sys/stat.h>
#include <signal.h>
#include <pthread.h>
#include <assert.h>
#include <bits/local_lim.h>
#include <poll.h>
#include <inttypes.h>

#include <lcm/lcm.h>

#include "common/getopt.h"
#include "common/string_util.h"
#include "common/sys_util.h"
#include "common/time_util.h"
#include "common/zarray.h"

#include "lcmtypes/procman_output_t.h"
#include "lcmtypes/procman_process_list_t.h"
#include "lcmtypes/procman_process_t.h"
#include "lcmtypes/procman_status_list_t.h"
#include "lcmtypes/procman_status_t.h"

#include "procman_core.h"

/** libbot procman interesting functionality:
  *   Attach function pointer to IO signal which is automatically called when a child prints
  *   Schedule future events using timeouts (timers?)
  */

typedef struct _procman_daemon procman_daemon_t;
struct _procman_daemon
{
    uint8_t                 survive; // survive after the controller shuts down
    int32_t                 delay_ms;
    int32_t                 kill_delay_ms; // how long to wait before sending a SIGKILL when shutting down?

    lcm_t                  *lcm;
    procman_process_list_t_subscription_t *plsub;

    sigset_t                sigset;
    sigset_t                old_sigset;

    const char             *hostname;
    const char             *logfile;
    volatile uint8_t        daemon_running;
    volatile uint8_t        all_running;

    pthread_mutex_t         proc_status_mutex;
    uint32_t                nprocesses;
    proc_status_t          *processes;
    char                  **process_names;

    pthread_mutex_t         process_list_mutex;
    procman_process_list_t *new_process_list;
    procman_process_list_t *current_process_list;
    int64_t                 current_init_utime;
    int64_t                 last_utime;

    pthread_t               monitorthread;
    pthread_t               lcmthread;
    pthread_t               sigwaitthread;
};

static procman_daemon_t global_pmd;

typedef struct _reader_thread reader_thread_t;
struct _reader_thread
{
    procman_daemon_t *pmd;
    int32_t procid;
    int stdout_fd;
    int stderr_fd;
    // "init utime" from the controller on creation of this child process
    int64_t received_init_utime;
};

void cleanup(procman_daemon_t *pmd)
{
    if (pmd->lcm != NULL) {
        if (pmd->plsub != NULL) procman_process_list_t_unsubscribe(pmd->lcm, pmd->plsub);
        lcm_destroy(pmd->lcm);
    }

    pthread_mutex_lock(&pmd->proc_status_mutex);
    if (pmd->processes != NULL)              free(pmd->processes);
    if (pmd->process_names != NULL)          free(pmd->process_names);
    pthread_mutex_unlock(&pmd->proc_status_mutex);

    if (pthread_mutex_destroy(&pmd->proc_status_mutex) != 0) {
        fprintf(stderr,"Failed to destory status pthread mutex\n");
        exit(EXIT_FAILURE);
    }

    pthread_mutex_lock(&pmd->process_list_mutex);
    if (pmd->new_process_list != NULL)       procman_process_list_t_destroy(pmd->new_process_list);
    if (pmd->current_process_list != NULL)   procman_process_list_t_destroy(pmd->current_process_list);
    pthread_mutex_unlock(&pmd->process_list_mutex);

    if (pthread_mutex_destroy(&pmd->process_list_mutex) != 0) {
        fprintf(stderr,"Failed to destory process pthread mutex\n");
        exit(EXIT_FAILURE);
    }

    close(STDIN_FILENO);
    close(STDOUT_FILENO);
    close(STDERR_FILENO);
}

// streamnum: 0 stdout 1 stderr
void read_pipe(lcm_t* lcm, int64_t received_init_utime,
               const char* hostname, int32_t procid, int fd, int streamnum)
{
    assert(streamnum == 0 || streamnum == 1);

    char buf[1024];
    int nreads = 0;
    int totalbytes = 0;
    int n;

    do {
        memset(buf, 0, sizeof(buf));
        n = read(fd, buf, sizeof(buf));

        if (n == -1)
            break;

        totalbytes += n;

        if (n > 0) {
            buf[n] = '\0';

            // XXX dynamically resize buf and only publish one message?
            if (procman_publish_output(lcm,
                                       utime_now(), // blacklist-ignore
                                       received_init_utime,
                                       hostname, procid, streamnum, buf) != 0) {
                perror("procman_publish_output()");
                fprintf(stderr,"Failed to publish process output message\n");
                exit(EXIT_FAILURE);
            }
        }

        nreads++;

    } while (n == sizeof(buf));
}

void *loop_read_pipes(void *td)
{
    // copy and free the reader_thread_t
    reader_thread_t *rt = (reader_thread_t*) td;
    procman_daemon_t *pmd = rt->pmd;
    int32_t procid = rt->procid;
    int stdout_fd  = rt->stdout_fd;
    int stderr_fd  = rt->stderr_fd;
    int64_t received_init_utime = rt->received_init_utime;
    free(rt);

    struct pollfd pfds[2];
    pfds[0].fd      = stdout_fd;
    pfds[1].fd      = stderr_fd;
    pfds[0].events  = POLLIN;
    pfds[1].events  = POLLIN;

    // monitor the pipes
    int32_t timeout_ms = 200;
    while (pmd->all_running)
    {
        int res = poll(pfds, 2, timeout_ms);
        if (res < 0) {
            fprintf(stderr,"CHILD: Poll returned %d\n", res);
            break;
        }

        // poll timed out
        if (res == 0)
            continue;

        // if stdout has data, read it
        if (pfds[0].revents & POLLIN)
            read_pipe(pmd->lcm, received_init_utime, pmd->hostname, procid, stdout_fd, 0);

        // if stderr has data, read it
        if (pfds[1].revents & POLLIN)
            read_pipe(pmd->lcm, received_init_utime, pmd->hostname, procid, stderr_fd, 1);

        if ((pfds[0].revents & POLLERR) || (pfds[1].revents & POLLERR)) {
            fprintf(stderr,"CHILD: Poll error has occurred (error). Stopping reader thread\n");
            break;
        }

        if ((pfds[0].revents & POLLHUP) || (pfds[1].revents & POLLHUP)) {
            fprintf(stderr,"CHILD: Poll error has occurred (hangup). Stopping reader thread\n");
            break;
        }

        if ((pfds[0].revents & POLLNVAL) || (pfds[1].revents & POLLNVAL)) {
            fprintf(stderr,"CHILD: Poll error has occurred (invalid fds). Stopping reader thread\n");
            break;
        }
    }

    // close pipes
    if (close(stdout_fd) != 0) {
        perror("close()");
        fprintf(stderr,"CHILD: Failed to close stdout for procid %d (fd %d)\n",
               procid, stdout_fd);
        exit(EXIT_FAILURE);
    }

    if (close(stderr_fd) != 0) {
        perror("close()");
        fprintf(stderr,"CHILD: Failed to close stdout for procid %d (fd %d)\n",
               procid, stderr_fd);
        exit(EXIT_FAILURE);
    }

    pthread_exit(NULL);
}

/** Is *this* machine the right one to run this process?
  * Returns 1 if true, 0 if false
  */
int run_on_this_machine(const char *machine_hostname, const char *desired_hostname)
{
    // if the specified hostname is not localhost and it does not match this
    // machine's hostname, don't fork this process
    if ((strcmp("localhost", desired_hostname) != 0) &&
        (strcmp(machine_hostname, desired_hostname) != 0))
        return 0;

    return 1;
}

void fork_process(procman_daemon_t *pmd, procman_process_t *process, proc_status_t *status)
{
    // reset state
    status->pid = -1;

    // open pipes
    int outfd[2];
    int errfd[2];
    if (pipe(outfd)) {
        fprintf(stderr,"Failed to create stdout pipe for child '%s'\n", process->name);
        exit(EXIT_FAILURE);
    }
    if (pipe(errfd)) {
        fprintf(stderr,"Failed to create stderr pipe for child '%s'\n", process->name);
        exit(EXIT_FAILURE);
    }

    // expand environment variables in commands using wordexp
    char *cmd_expanded = expand_environment_variables(process->cmdline);
    fprintf(stderr,"Expanded command '%s' to '%s'\n", process->cmdline, cmd_expanded);

    // fork to create child process
    pid_t pid = fork();

    // parent process, error condition
    if (pid < 0) {
        perror("fork()");
        fprintf(stderr,"Failed to fork for child '%s'. PID %d\n", process->name, pid);
        exit(EXIT_FAILURE);
    }

    ////////////////////////////////////////
    // child process, success
    if (pid == 0) {

        int fd = open("/dev/null", O_RDONLY);
        if (fd < 0)
            perror("open(/dev/null)");
        else {
            dup2(fd, STDIN_FILENO);
            close(fd);
        }

        dup2(outfd[1], STDOUT_FILENO); // changes stdout, but closes stdout first!
        dup2(errfd[1], STDERR_FILENO); // changes stderr, but closes stderr first!

        close(outfd[1]); // clean up
        close(errfd[1]); // clean up

        close(outfd[0]); // close the parent's end of the pipe
        close(errfd[0]); // close the parent's end of the pipe

        zarray_t *token_array = str_split(cmd_expanded, " ");
        char **tokens = malloc((zarray_size(token_array)+1) * sizeof(char*));
        for (int i=0; i < zarray_size(token_array); i++)
            zarray_get(token_array, i, &tokens[i]);
        tokens[zarray_size(token_array)] = NULL;

        // reset signal mask. necessary to make sure we don't block signals
        // from reaching the children, even though we blocked them from
        // reaching the other threads
        int err;
        if ((err = sigprocmask(SIG_SETMASK, &pmd->old_sigset, NULL)) != 0) {
            perror("sigprocmask()");
            fprintf(stderr,"CHILD: Error: sigprocmask returned %d\n", err);
            exit(EXIT_FAILURE);
        }

        // exec the process using the command and arguments. execvp will search
        // the PATH environment variable if the command doesn't contain a /
        // the default environment (unistd.h's "char **environ") is used
        //
        // exec should only return when the process dies
        int status = execvp(tokens[0], tokens);

        if (status != 0) {
            perror("Child error");
            fprintf(stderr,"CHILD: Exec of '%s' returned with status %d (command expanded to '%s')\n",
                   process->cmdline, status, cmd_expanded);
        } else {
            fprintf(stderr,"CHILD: Exec of '%s' returned with status %d (command expanded to '%s')\n",
                   process->cmdline, status, cmd_expanded);
        }

        fflush(stdout);
        fflush(stderr);

        zarray_vmap(token_array, free);
        zarray_destroy(token_array);
        free(tokens);
        free(cmd_expanded);

        exit(status);
    }

    ////////////////////////////////////////
    // parent process, success

    free(cmd_expanded);

    close(outfd[1]); // close the child's end of the pipe
    close(errfd[1]); // close the child's end of the pipe

    status->running = 1;
    status->died = 0;
    status->time_of_death = 0;
    status->shutdown_attempts = 0;
    status->shutdown_utime = 0;

    status->pid = pid;

    // reader thread will free this object
    reader_thread_t *rt = malloc(sizeof(reader_thread_t));
    if (rt == NULL) {
        perror("malloc()");
        fprintf(stderr,"Failed to malloc procman_reader_thread_t\n");
        exit(EXIT_FAILURE);
    }

    rt->pmd = pmd;
    rt->procid = status->procid;
    rt->stdout_fd = outfd[0];
    rt->stderr_fd = errfd[0];
    // don't need to lock the process_list_mutex, fork is called in the loop that
    // updates the current_process_list
    rt->received_init_utime = pmd->current_process_list->init_utime;

    // create reader thread
    if (pthread_create(&status->reader_thread, NULL, &loop_read_pipes, (void*) rt) != 0) {
        perror("pthread_create()");
        fprintf(stderr,"failed to start child reader thread\n");
        pmd->daemon_running = 0;
        //exit(EXIT_FAILURE); // exiting causes all child processes to be orphaned
    }
}

void daemonize()
{
    pid_t pid;

    if (getppid() == 1)
        return; // already a daemon

    pid = fork();

    if (pid < 0) {
        perror("fork to background");
        fprintf(stderr,"failed to daemonize\n");
        exit(EXIT_FAILURE);
    }

    if (pid > 0)
        exit(0); // parent exits

    // child (daemon) continues
    setpgrp(); // obtain a new process group
    umask(027); // set newly created file permissions
}

void open_logfile(procman_daemon_t *pmd)
{
    int fd = open("/dev/null", O_RDONLY);
    if (fd < 0) {
        perror("/dev/null");
    }
    else {
        close(STDIN_FILENO);
        if (dup2(fd, STDIN_FILENO) < 0) {
            perror("dup2()");
            fprintf(stderr,"Could not set STDIN to /dev/null\n");
            exit(EXIT_FAILURE);
        }
        close(fd);
    }

    int logfd = open(pmd->logfile, O_WRONLY | O_TRUNC | O_CREAT, 0644);
    if (logfd < 0) {
        perror("open()");
        fprintf(stderr,"Procman log file could not be opened ('%s')\n", pmd->logfile);
        exit(EXIT_FAILURE);
    } else {
        close(STDOUT_FILENO);
        close(STDERR_FILENO);
        if (dup2(logfd, STDOUT_FILENO) < 0) {
            perror("dup2()");
            fprintf(stderr,"Could not set STDOUT to log file ('%s')\n", pmd->logfile);
            exit(EXIT_FAILURE);
        }
        if (dup2(logfd, STDERR_FILENO) < 0) {
            perror("dup2()");
            fprintf(stderr,"Could not set STDERR to log file ('%s')\n", pmd->logfile);
            exit(EXIT_FAILURE);
        }
        close(logfd);
        setlinebuf(stdout);
        setlinebuf(stderr);
    }

    printf("Opened log at '%s'\n", pmd->logfile);
}

void check_for_dead_children(procman_daemon_t *pmd)
{
    pthread_mutex_lock(&pmd->proc_status_mutex);

    for (int i=0; i < pmd->nprocesses; i++)
    {
        proc_status_t *proc_status = &pmd->processes[i];

        if (proc_status->running == 0 || proc_status->pid < 0)
            continue;

        int32_t status = 0;
        pid_t res = waitpid(proc_status->pid, &status, WNOHANG);

        // if wait succeeded, child is now dead
        if (res == proc_status->pid) {

            if (WIFEXITED(status)) {

                int exitcode = WEXITSTATUS(status);
                //printf("procman_daemon: [%d] pid %d exited with status %d\n",
                //       i, proc_status->pid, exitcode);

                proc_status->running            = 0;
                proc_status->died               = 1;
                proc_status->time_of_death      = utime_now(); // blacklist-ignore
                proc_status->pid                = -1;
                proc_status->shutdown_attempts  = 0;
                proc_status->shutdown_utime     = 0;
                proc_status->last_exit_code     = exitcode;
            }

            if (WIFSIGNALED(status)) {

                int signal = WTERMSIG(status);
                //printf("procman_daemon: [%d] pid %d failed to catch signal %d\n",
                //       i, proc_status->pid, signal);

                proc_status->running            = 0;
                proc_status->died               = 1;
                proc_status->time_of_death      = utime_now(); // blacklist-ignore
                proc_status->pid                = -1;
                proc_status->shutdown_attempts  = 0;
                proc_status->shutdown_utime     = 0;
                proc_status->last_exit_code     = -signal; // XXX the right choice?
            }

            if (WIFSTOPPED(status)) {
                //printf("procman_daemon: [%d] pid %d stopped with status %d\n",
                //       i, proc_status->pid, WSTOPSIG(status));
            }

            if (WIFCONTINUED(status)) {
                //printf("procman_daemon: [%d] pid %d continued\n",
                //       i, proc_status->pid);
            }
        }

        // child is alive
        if (res == 0) {
            //printf("procman_daemon: [%d] pid %d is alive\n",
            //       i, proc_status->pid);

            proc_status->running            = 1;
            proc_status->died               = 0;
            proc_status->time_of_death      = 0;
            proc_status->shutdown_attempts  = 0;
            proc_status->shutdown_utime     = 0;
        }

        // child was already dead
        if (res == -1) {
            //printf("procman_daemon: [%d] pid %d was already dead\n",
            //       i, proc_status->pid);

            proc_status->running            = 0;
            proc_status->died               = 1;
            proc_status->time_of_death      = utime_now(); // blacklist-ignore
            proc_status->shutdown_attempts  = 0;
            proc_status->shutdown_utime     = 0;
            proc_status->last_exit_code     = -3;
        }
    }

    pthread_mutex_unlock(&pmd->proc_status_mutex);
}

void block_signals(procman_daemon_t *pmd)
{
    sigemptyset(&pmd->sigset);
    sigaddset(&pmd->sigset, SIGCHLD);
    sigaddset(&pmd->sigset, SIGHUP);
    sigaddset(&pmd->sigset, SIGINT);
    sigaddset(&pmd->sigset, SIGTERM);
    sigaddset(&pmd->sigset, SIGUSR1);
    int sigmaskerr;
    if ((sigmaskerr = pthread_sigmask(SIG_BLOCK, &pmd->sigset, &pmd->old_sigset)) != 0) {
        perror("pthread_sigmask()");
        fprintf(stderr,"SIG_BLOCK error (%d)\n", sigmaskerr);
        exit(EXIT_FAILURE);
    }
}

void handle_signal(procman_daemon_t *pmd, int sig)
{
    switch(sig) {
        case SIGCHLD:
            fprintf(stderr,"Daemon: child has died. cleaning up.\n");
            check_for_dead_children(pmd);
            break;
        case SIGHUP:
            fprintf(stderr,"Daemon: hangup signal caught. Doing nothing\n");
            break;
        case SIGINT:
            fprintf(stderr,"Daemon: interrupt signal caught. Beginning shutdown process.\n");
            pmd->daemon_running = 0;
            break;
        case SIGTERM:
            fprintf(stderr,"Daemon: terminate signal caught. Beginning shutdown process.\n");
            pmd->daemon_running = 0;
            break;
        default:
            fprintf(stderr,"Daemon: unknown signal (%d) caught. Doing nothing.\n", sig);
            break;
    }
}

void *loop_sigwait(void *td)
{
    procman_daemon_t *pmd = (procman_daemon_t*) td;

    sigset_t *mask = &pmd->sigset;

    int err, signo;
    while (pmd->all_running)
    {
        err = sigwait(mask, &signo);

        if (err != 0) {
            perror("sigwait()");
            fprintf(stderr,"loop_sigwait: sigwait failed (%d)\n", err);
            exit(EXIT_FAILURE);
        }

        if (signo == SIGUSR1)
            break;

        handle_signal(pmd, signo);
    }

    pthread_exit(NULL);
}

void *loop_lcmhandle(void *td)
{
    procman_daemon_t *pmd = (procman_daemon_t*) td;

    while (pmd->all_running)
    {
        lcm_handle(pmd->lcm);
    }

    pthread_exit(NULL);
}

static void
process_list_handler(const lcm_recv_buf_t *rbuf,
                     const char* channel,
                     const procman_process_list_t *msg,
                     void *usr)
{
    procman_daemon_t *pmd = (procman_daemon_t*) usr;

    pthread_mutex_lock(&pmd->process_list_mutex);
    //printf("Daemon: Received new process list\n");

    if (pmd->new_process_list != NULL)
    {
        //printf("Daemon: Destroying unused process list\n");
        procman_process_list_t_destroy(pmd->new_process_list);
    }

    pmd->new_process_list = procman_process_list_t_copy(msg);

    //printf("Daemon: Copied process list\n");
    pthread_mutex_unlock(&pmd->process_list_mutex);
}

void update_process_list(procman_daemon_t *pmd)
{
    pthread_mutex_lock(&pmd->process_list_mutex);

    // if there's a new list, we should take it
    if (pmd->new_process_list != NULL) {
        //printf("Daemon: New process list waiting\n");

        // we have to get rid of the current list first
        if (pmd->current_process_list != NULL)
        {
            //printf("Daemon: Destroying current process list\n");
            procman_process_list_t_destroy(pmd->current_process_list);
        }

        // no one else needs the list, so we can avoid the copy
        pmd->current_process_list = pmd->new_process_list;
        pmd->new_process_list = NULL;
        //printf("Daemon: Took new process list\n");
    }

    pthread_mutex_unlock(&pmd->process_list_mutex);
}

void clear_current_process_list(procman_daemon_t *pmd)
{
    pthread_mutex_lock(&pmd->process_list_mutex);

    if (pmd->current_process_list != NULL)
        procman_process_list_t_destroy(pmd->current_process_list);

    pmd->current_process_list = NULL;

    pthread_mutex_unlock(&pmd->process_list_mutex);
}

char** copy_process_names(procman_process_list_t *plist)
{
    char **names = malloc(plist->nprocs * sizeof(char*));
    if (names == NULL)
        return NULL;

    for (int i=0; i < plist->nprocs; i++)
        names[i] = strdup(plist->processes[i].name);

    return names;
}

proc_status_t *init_processes(procman_process_list_t *plist, uint32_t *_n)
{
    proc_status_t *statuses = calloc(plist->nprocs, sizeof(proc_status_t));
    if (statuses == NULL) {
        perror("calloc()");
        fprintf(stderr,"Failed to calloc proc_status_t array\n");
        exit(EXIT_FAILURE);
    }

    *_n = plist->nprocs;
    uint32_t n = *_n;

    for (int i=0; i < n; i++) {

        procman_process_t *process = &plist->processes[i];
        proc_status_t *status = &statuses[i];

        status->procid = process->procid;
        status->running = 0;
        status->died = 0;
        status->time_of_death = 0;
        status->last_exit_code = 0;
        status->restarts = 0;
        status->shutdown_attempts = 0;
        status->shutdown_utime = 0;

        status->pid = -1;
    }

    return statuses;
}

void shutdown_process(procman_daemon_t *pmd, char *name, proc_status_t *status)
{
    if (status->running == 0 || status->pid < 0)
        return;

    int64_t now = utime_now(); // blacklist-ignore

    if (status->shutdown_attempts == 0)
        status->shutdown_utime = now;

    int64_t dt = now - status->shutdown_utime;

    // escalate kill signals
    int signal = SIGTERM;
    char *signame = "SIGTERM";
    if (dt > pmd->kill_delay_ms * 1000) { // did we first try 3 seconds ago? then really kill it!
        signal = SIGKILL;
        signame = "SIGKILL";
    }

    status->shutdown_attempts++;

    // try to kill the process
    int result = kill(status->pid, signal);

    // did the signal send?
    if (result != 0)
        fprintf(stderr,"Shutdown '%s': Failed to send '%s'!!!\n",
               name, signame);
}

void check_on_process(procman_daemon_t *pmd, procman_process_t *process, proc_status_t *status)
{
    assert(process->procid == status->procid);

    // if appropriate, restart the process
    if (process->enabled == 1                                   &&    // the controller wants it to run
        run_on_this_machine(pmd->hostname, process->host) == 1  &&    // this is the right machine to run on
        status->running == 0)                                         // the process is not currently running
    {
        // if not running and hasn't died, it was never started
        if (status->died == 0)
        {
            fork_process(pmd, process, status);
            fprintf(stderr,"Started '%s' (PID is %d)\n",
                   process->name, status->pid);
        }
        // otherwise, it died and might need to be auto-restarted
        else if (process->auto_restart == 1)
        {
            int64_t now = utime_now(); // blacklist-ignore
            int64_t elapsed_us = now - status->time_of_death;
            int64_t wait_us = 1000 * process->restart_delay_ms;

            if (elapsed_us > wait_us)
            {
                fork_process(pmd, process, status);
                status->restarts++;
                fprintf(stderr,"Restarted '%s' (PID is %d)\n",
                       process->name, status->pid);
            }
        }
    }

    // kill if appropriate
    if (status->running == 1 && // the process is running
        process->enabled == 0)  // the process should *not* be running
    {
        shutdown_process(pmd, process->name, status);
    }
}

void *loop_monitor_processes(void *td)
{
    procman_daemon_t *pmd = (procman_daemon_t*) td;

    int sleep_us = 1000 * pmd->delay_ms;

    while (pmd->daemon_running)
    {
        int shutdown = 0;
        // as long as we have the same controller, stay in this inner loop
        while (pmd->daemon_running)
        {
            timeutil_usleep(sleep_us);
            update_process_list(pmd);

            // continue or break if we can't proceed
            if (pmd->current_process_list == NULL ||                              // haven't received anything, or
                pmd->current_process_list->nprocs == 0 ||                         // nothing to run, or
                pmd->current_process_list->init_utime < pmd->current_init_utime)  // we've seen a newer controller
            {
                continue;
            }

            // if we aren't initialized, exit messages should be ignored and cleared
            if (pmd->processes == NULL && pmd->current_process_list->exit == 1) {
                clear_current_process_list(pmd);
                continue;
            }

            // initialize if necessary
            if (pmd->processes == NULL)
            {
                pthread_mutex_lock(&pmd->proc_status_mutex);
                pmd->process_names       = copy_process_names(pmd->current_process_list);
                if (pmd->process_names == NULL) fprintf(stderr,"Failed to copy process names.\n");
                pmd->processes           = init_processes(pmd->current_process_list, &pmd->nprocesses);
                pmd->current_init_utime  = pmd->current_process_list->init_utime;
                fprintf(stderr,"Initialized processes using init_utime list from %" PRIu64 "\n", pmd->current_init_utime);
                pthread_mutex_unlock(&pmd->proc_status_mutex);
            }

            // shut down current jobs if we just got a message from a newer controller
            // XXX we shouldn't compare these utimes, actually, the clocks could be off
            if (pmd->current_process_list->init_utime > pmd->current_init_utime) {
                shutdown = 1;
                fprintf(stderr,"Shut down processes because a newer controller connected (new %" PRIu64 " old %" PRIu64 ")\n",
                       pmd->current_process_list->init_utime, pmd->current_init_utime);
                break;
            }

            // shut down if requested
            if (pmd->current_process_list->exit == 1) {
                shutdown = 1;
                fprintf(stderr,"Shut down processes as requested by the controller\n");
                break;
            }

            pthread_mutex_lock(&pmd->proc_status_mutex);
            for (int i=0; i < pmd->nprocesses; i++) {

                procman_process_t *process = &pmd->current_process_list->processes[i];
                proc_status_t *status = &pmd->processes[i];

                assert(i == process->procid);
                assert(i == status->procid);

                check_on_process(pmd, process, status);
            }
            pthread_mutex_unlock(&pmd->proc_status_mutex);

            pthread_mutex_lock(&pmd->proc_status_mutex);
            procman_publish_process_status_list(pmd->lcm,
                                                utime_now(), // blacklist-ignore
                                                pmd->current_process_list->utime,
                                                pmd->current_process_list->init_utime,
                                                pmd->hostname,
                                                pmd->nprocesses,
                                                pmd->processes);
            pthread_mutex_unlock(&pmd->proc_status_mutex);

            pmd->last_utime = pmd->current_process_list->utime;
        }

        // we have to stop the current processes
        if (shutdown || !pmd->daemon_running)
        {
            int any_alive = 1;
            while (any_alive)
            {
                pthread_mutex_lock(&pmd->proc_status_mutex);
                // try shutting down all processes
                for (int i=0; i < pmd->nprocesses; i++) {
                    proc_status_t *status = &pmd->processes[i];
                    shutdown_process(pmd, pmd->process_names[i], status);
                }
                pthread_mutex_unlock(&pmd->proc_status_mutex);

                timeutil_usleep(sleep_us);

                pthread_mutex_lock(&pmd->proc_status_mutex);
                // check if any processes are still alive
                any_alive = 0;
                for (int i=0; i < pmd->nprocesses; i++) {
                    proc_status_t *status = &pmd->processes[i];

                    if (status->running == 1)
                        any_alive = 1;
                }
                pthread_mutex_unlock(&pmd->proc_status_mutex);

                pthread_mutex_lock(&pmd->proc_status_mutex);
                procman_publish_process_status_list(pmd->lcm,
                                                    utime_now(), // blacklist-ignore
                                                    pmd->last_utime,
                                                    pmd->current_init_utime,
                                                    pmd->hostname,
                                                    pmd->nprocesses,
                                                    pmd->processes);
                pthread_mutex_unlock(&pmd->proc_status_mutex);
            }

            fprintf(stderr,"Daemon: All processes shut down successfully.\n");

            // if we just shut down because of an stop message, we should clear the process list
            if (shutdown &&
                pmd->current_process_list != NULL &&
                pmd->current_process_list->exit == 1)
            {
                clear_current_process_list(pmd);
            }

            pthread_mutex_lock(&pmd->proc_status_mutex);
            pmd->nprocesses = 0;
            if (pmd->process_names != NULL) {
                free(pmd->process_names);
                pmd->process_names = NULL;
            }
            if (pmd->processes != NULL) {
                free(pmd->processes);
                pmd->processes = NULL;
            }
            pthread_mutex_unlock(&pmd->proc_status_mutex);

            // now that we're done shutting down, we can update the init_utime if this message is from a newer controller
            if (pmd->current_process_list != NULL &&
                pmd->current_process_list->init_utime > pmd->current_init_utime)
            {
                fprintf(stderr,"Controller init_utime updated to %" PRIu64 "\n", pmd->current_process_list->init_utime);
                pmd->current_init_utime = pmd->current_process_list->init_utime;
            }
            // if we did *not* shut down because of a new controller, the daemon should
            // shut down if it was requested at the command line
            else if (pmd->survive == 0) {
                pmd->daemon_running = 0;
            }
        }
    }

    // other threads can shut down now
    pmd->all_running = 0;

    int error;
    error = pthread_kill(pmd->sigwaitthread, SIGUSR1);
    if (error != 0) {
        perror("pthread_kill()");
        fprintf(stderr,"Daemon: pthread_kill returned error %d\n", error);
    }

    pthread_exit(NULL);
}

int main(int argc, char ** argv)
{
    int ret;

    setlinebuf(stdout);
    setlinebuf(stderr);

    ////////////////////////////////////////
    // get hostname
    char hostname[HOST_NAME_MAX];
    ret = gethostname(hostname, sizeof(hostname));
    if (ret != 0) {
        fprintf(stderr,"gethostname() failed: %d\n", ret);
        exit(EXIT_FAILURE);
    }

    ////////////////////////////////////////
    // argument handling
    getopt_t *gopt = getopt_create();
    getopt_add_bool(gopt, 'h',"help", 0,"Show this");
    getopt_add_bool(gopt, 'd', "daemon", 0, "Enable daemon mode (disconnect from terminal)");
    getopt_add_bool(gopt, 's', "survive", 0, "Survive after the controller shuts down");
    getopt_add_string(gopt, 'l', "logpath", "/tmp/procman_daemon.log", "Procman daemon log file");
    getopt_add_string(gopt, 'n', "hostname", hostname, "Hostname for daemon to use"); // XXX using getopt bug with -- syntax
    getopt_add_int(gopt, 'p', "period", "100", "Delay between status checks (ms)");
    getopt_add_int(gopt, 'k', "killdelay", "3000", "Delay during process shutdown before sending SIGKILL (ms)");

    // parse and print help
    if (!getopt_parse(gopt, argc, argv, 1)) {
        fprintf(stderr,"Unable to parse command-line arguments\n");
        exit(EXIT_FAILURE);
    }

    if (getopt_get_bool(gopt,"help")) {
        printf("Usage: %s [options]\n\n", argv[0]);
        getopt_do_usage(gopt);
        exit(EXIT_SUCCESS);
    }

    uint8_t daemon_mode = getopt_get_bool(gopt, "daemon");
    uint8_t survive = getopt_get_bool(gopt, "survive");
    int32_t delay_ms = getopt_get_int(gopt, "period");
    int32_t kill_delay_ms = getopt_get_int(gopt, "killdelay");
    const char *logfile = getopt_get_string(gopt, "logpath");
    if (logfile == NULL || strlen(logfile) == 0) {
        fprintf(stderr,"No log file supplied\n");
        printf("Usage: %s [options]\n\n", argv[0]);
        getopt_do_usage(gopt);
        exit(EXIT_FAILURE);
    }


    ////////////////////////////////////////
    // global state object
    procman_daemon_t *pmd = &global_pmd;

    memset(pmd, 0, sizeof(procman_daemon_t));
    pmd->survive = survive;
    pmd->delay_ms = delay_ms;
    pmd->kill_delay_ms = kill_delay_ms;
    pmd->lcm = NULL;
    pmd->hostname = getopt_get_string(gopt, "hostname");
    pmd->logfile = logfile;
    pmd->daemon_running = 1;
    pmd->all_running = 1;
    ret = pthread_mutex_init(&pmd->proc_status_mutex, NULL);
    if (ret != 0) {
        fprintf(stderr,"Failed to init status pthread mutex: %d\n", ret);
        exit(EXIT_FAILURE);
    }
    pmd->nprocesses = 0;
    pmd->processes = NULL;
    pmd->process_names = NULL;
    ret = pthread_mutex_init(&pmd->process_list_mutex, NULL);
    if (ret != 0) {
        fprintf(stderr,"Failed to init process pthread mutex: %d\n", ret);
        exit(EXIT_FAILURE);
    }
    pmd->new_process_list = NULL;
    pmd->current_process_list = NULL;
    pmd->current_init_utime = 0;
    pmd->last_utime = 0;

    ////////////////////////////////////////
    // switch file descriptors to log file
    printf("Opening log file ('%s')\n", pmd->logfile);
    printf("Monitor (tail -f) log file for PID changes\n");

    open_logfile(pmd);

    ////////////////////////////////////////
    // (optionally) enter daemon mode

    if (daemon_mode) {
        printf("Entering daemon mode. Daemon PID *will* change.\n");
        daemonize();
        printf("Daemon call succeeded. New PID is %d\n", getpid());
    }

    ////////////////////////////////////////
    // block signals for pthreads
    block_signals(pmd);

    ////////////////////////////////////////
    // setup lcm
    pmd->lcm = lcm_create(NULL);
    if (pmd->lcm == NULL) {
        fprintf(stderr,"Failed to create LCM\n");
        exit(EXIT_FAILURE);
    }

    pmd->plsub = procman_process_list_t_subscribe(pmd->lcm, "PROCMAN_PROCESS_LIST", &process_list_handler, (void*) pmd);

    ////////////////////////////////////////
    // start threads

    pthread_attr_t attr;
    pthread_attr_init(&attr);
    pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);

    // worker loop
    ret = pthread_create(&pmd->monitorthread, &attr, &loop_monitor_processes, (void*) pmd);
    if (ret != 0) {
        fprintf(stderr,"failed to start monitor thread: %d\n", ret);
        cleanup(pmd);
        exit(EXIT_FAILURE);
    }
    // lcm
    ret = pthread_create(&pmd->lcmthread, &attr, &loop_lcmhandle, (void*) pmd);
    if (ret != 0) {
        fprintf(stderr,"failed to start lcmhandle thread: %d\n", ret);
        cleanup(pmd);
        exit(EXIT_FAILURE);
    }
    // sigwait
    ret = pthread_create(&pmd->sigwaitthread, &attr, &loop_sigwait, (void*) pmd);
    if (ret != 0) {
        fprintf(stderr,"failed to start sigwait thread: %d\n", ret);
        cleanup(pmd);
        exit(EXIT_FAILURE);
    }
    pthread_attr_destroy(&attr);

    void *status;
    // worker loop
    ret = pthread_join(pmd->monitorthread, &status);
    if (ret != 0) {
        fprintf(stderr,"failed to join monitor thread: %d\n", ret);
        cleanup(pmd);
        exit(EXIT_FAILURE);
    }
    printf("Join for monitor thread returned\n");
    // lcm
    pthread_cancel(pmd->lcmthread);
    ret = pthread_join(pmd->lcmthread, &status);
    if (ret != 0) {
        fprintf(stderr,"failed to join lcmhandle thread: %d\n", ret);
        cleanup(pmd);
        exit(EXIT_FAILURE);
    }
    printf("Join for lcm thread returned\n");
    // sigwait
    ret = pthread_join(pmd->sigwaitthread, &status);
    if (ret != 0) {
        fprintf(stderr,"failed to join sigwait thread: %d\n", ret);
        cleanup(pmd);
        exit(EXIT_FAILURE);
    }
    printf("Join for sigwait thread returned\n");

    printf("Cleaning up main and exiting.\n");
    getopt_destroy(gopt);
    cleanup(pmd);

    pthread_exit(NULL);
}
