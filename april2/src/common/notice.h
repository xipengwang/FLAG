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

/**
 * The notice utility provides a consistent mechanism for outputting user-level
 * feedback from a process. The simple printf-like code interface encapsulates
 * the level of severity of the message being output. Once compiled, the types
 * and level of output can be controlled through environment variables without needing
 * to recompile the code. Varying levels of output can be directed to stdout/stderr,
 * transmitted through an LCM notice_t message, and/or sent to e-speak through an
 * LCM speak_t message by setting the appropriate environment variable.
 *
 * All notice infrastructure can be eliminated by defining NO_NOTICE at compile time.
 *
 * Example usage:
 *
 *   #include "notice.h"
 *
 *   int main(int argc, char *argv[])
 *   {
 *      notice_init(argc, argv, NULL);
 *
 *      nfailf("Process encountered unrecoverable error\n");
 *      nwarnf("Process encountered recoverable anomaly\n");
 *      ninfof("Notification information\n");
 *      ndebugf("Debugging message which is output at all debugging levels\n");
 *      ndebug1f("Debugging message which is output at moderately verbose debugging levels\n");
 *      ndebug2f("Debugging message which is output only at the most verbose debugging level\n");
 *      nprintf(NOTICE_T_COMPROMISED, "Process now has compromised run state\n");
 *      nprintf(NOTICE_T_DEGRADED, "Process continuing to run with degraded performance\n");
 *      nprintf(NOTICE_T_SWAMPED, "Process cannot keep up with data influx\n");
 *
 *      notice_destroy();
 *      return 0;
 *   }
 *
 *   shell> export NOTICE_PRINT="debug1"  # print moderate debugging information and all info, warnings, and errors
 *   shell> export NOTICE_SEND="warn"     # send LCM notices for severities >= warnings
 *   shell> export NOTICE_SPEAK="fail"    # send espeak notices for failures only
 *
 */


#ifndef NGV_NOTICE_H_
#define NGV_NOTICE_H_

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <unistd.h>
#include <limits.h>
#include <libgen.h>
#include <pthread.h>
#include <lcm/lcm.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <assert.h>

#include "common/string_util.h"

#include "lcmtypes/notice_t.h"
#include "lcmtypes/speak_t.h"

#ifdef __cplusplus
extern "C" {
#endif


// NOTE: Severity levels are inherited from the notice_t LCM message and changes
//   to those values will invalidate notice_t contents in saved LCM log files

// Severity level masks for use with output control (not valid as severity levels)
#define NOTICE_MASK_NONE     0                 // mask for disabling all notices
#define NOTICE_MASK_ERROR    NOTICE_T_DEGRADED // an error is considered any loss of performance
#define NOTICE_MASK_STDERR   NOTICE_T_WARNING  // mask for all print output going to stderr instead of stdout
#define NOTICE_MASK_ALL      127               // mask for enabling all notices


// Structure used to define string equivalents to severity modes
typedef struct severity_name {
    const char *name;
    int8_t level;
} severity_mode_t;

// Lookup table of string equivalents to severity modes which can be used in environment variables
static severity_mode_t severity_nametab[] = {
        // available string values for environment variables
        {"none",        NOTICE_MASK_NONE},
        {"failure",     NOTICE_T_FAILURE},
        {"compromised", NOTICE_T_COMPROMISED},
        {"swamped",     NOTICE_T_SWAMPED},
        {"degraded",    NOTICE_T_DEGRADED},
        {"warning",     NOTICE_T_WARNING},
        {"info",        NOTICE_T_INFO},
        {"debug0",      NOTICE_T_DEBUG0},
        {"debug1",      NOTICE_T_DEBUG1},
        {"debug2",      NOTICE_T_DEBUG2},
        {"all",         NOTICE_MASK_ALL},

        // common abbreviations and masks for ease of use
        {"fail",        NOTICE_T_FAILURE},
        {"warn",        NOTICE_T_WARNING},
        {"debug",       NOTICE_T_DEBUG0},
        {"error",       NOTICE_MASK_ERROR},
        {"err",         NOTICE_MASK_ERROR},
        {"stderr",      NOTICE_MASK_STDERR},

        {NULL, 0} // terminator
};


// Name of the environment variable which defines the minimum notice severity for
//   printing notices to stdout/stderr
// See severity_nametab[] for available values
// Default: "info"
#define NOTICE_ENV_PRINT     "NOTICE_PRINT"

// Name of the environment variable which defines the minimum notice severity for
//   sending LCM messages
// See severity_nametab[] for available values
// Default: "warning"
#define NOTICE_ENV_SEND      "NOTICE_SEND"

// Name of the environment variable which defines the minimum notice severity for
//   emitting audible alarms through espeak
// See severity_nametab[] for available values
// Default: "error"
#define NOTICE_ENV_SPEAK     "NOTICE_SPEAK"


// Channel on which notice_t messages will be sent
#define NOTICE_CHANNEL_NOTICE "NOTICE"

// Channel on which speak_t messages will be sent
#define NOTICE_CHANNEL_SPEAK  "SPEAK"


// The name of the process publishing notices
static char *notice_process_name = NULL;
// The command-line arguments of the process publishing notices
static char *notice_process_args = NULL;
// The hostname on which the notice process is running
static char *notice_process_hostname = NULL;
// The minimum notice severity for printing to stdout/stderr
static int8_t notice_level_print = NOTICE_T_INFO;
// The minimum notice severity for sending notice_t LCM messages
static int8_t notice_level_send  = NOTICE_T_WARNING;
// The minimum notice severity for sending speak_t LCM messages
static int8_t notice_level_speak = NOTICE_T_DEGRADED;
// The LCM object used to send notice_t and speak_t LCM messages.
static lcm_t *notice_lcm = NULL;
// The URL to be used by LCM
static char *notice_lcm_url = NULL;
// Mutex to ensure thread safety
static pthread_mutex_t notice_mutex = PTHREAD_MUTEX_INITIALIZER;
// Arguments for speak_t messages
static char *notice_speak_args = NULL;


/**
 * Translates a string representation of a severity level into its corresponding
 *   NOTICE_T_* value using the severity_nametab[] lookup table.
 *
 * Returns the corresponding severity level, or -1 if the string value was not
 *   recognized.
 */
static int8_t notice_str_to_severity(const char *env_value) __attribute__ ((unused));
static int8_t notice_str_to_severity(const char *env_value)
{
    severity_mode_t *entry;

    for (entry = severity_nametab ; entry->name != NULL ; entry++)
        if (strcasecmp(env_value, entry->name) == 0)
            return entry->level;

    return -1;
}


/**
 * Should be called prior to any call to notice_dispatch() or one of its associated
 *   macros to set the source of outgoing notice_t LCM messages (not mandatory).
 * Will be called by the first call to notice_dispatch() if not previously called.
 *
 * Loads and stores the current values of the NOTICE_ENV_* environment variables.
 *
 * May be called again to subsequently change the process name setting, LCM URL,
 *   and/or reload environment variable values.
 *
 * 'process_name' should be a string which uniquely defines the calling process
 *   i.e. basename(argv[0]), but may be NULL to send an empty string as the
 *   source of notice_t messages.
 *
 * 'lcm_url' will be the URL used by LCM for sending notice_t and speak_t messages
 *   and may be NULL.
 */
static void notice_init(int argc, char *argv[], const char *lcm_url) __attribute__ ((unused));
static void notice_init(int argc, char *argv[], const char *lcm_url)
{
    const char *env_value;
    int8_t severity;

    pthread_mutex_lock(&notice_mutex);
    {
        // set the output levels based on current environment variable settings
        env_value = getenv(NOTICE_ENV_PRINT);
        if (env_value != NULL) {
            severity = notice_str_to_severity(env_value);
            if (severity < 0) {
                fprintf(stderr, "WARNING: Unrecognized notice severity level in $%s: '%s'\n",
                        NOTICE_ENV_PRINT, env_value);
            } else {
                notice_level_print = severity;
            }
        }

        env_value = getenv(NOTICE_ENV_SEND);
        if (env_value != NULL) {
            severity = notice_str_to_severity(env_value);
            if (severity < 0) {
                fprintf(stderr, "WARNING: Unrecognized notice severity level in $%s: '%s'\n",
                        NOTICE_ENV_SEND, env_value);
            } else {
                notice_level_send = severity;
            }
        }

        env_value = getenv(NOTICE_ENV_SPEAK);
        if (env_value != NULL) {
            severity = notice_str_to_severity(env_value);
            if (severity < 0) {
                fprintf(stderr, "WARNING: Unrecognized notice severity level in $%s: '%s'\n",
                        NOTICE_ENV_SPEAK, env_value);
            } else {
                notice_level_speak = severity;
            }
        }

        // save the LCM URL for future incantations of lcm_create()
        if (notice_lcm_url)
            free(notice_lcm_url);
        if (lcm_url)
            notice_lcm_url = strdup(lcm_url);
        else
            notice_lcm_url = NULL;

        // use system-wide speak arguments
        if (notice_speak_args == NULL)
            notice_speak_args = strdup("");

        // determine command-line argument string, if argv[] was supplied
        if (notice_process_args == NULL) {
            string_buffer_t *args_buffer = string_buffer_create();
            if (argv != NULL) {
                for (int i = 1 ; i < argc ; i++) {
                    string_buffer_append_string(args_buffer, argv[i]);
                    if (i < argc - 1)
                        string_buffer_append(args_buffer, ' ');
                }
            }
            notice_process_args = string_buffer_to_string(args_buffer);
            string_buffer_destroy(args_buffer);
        }

        // determine local hostname
        if (notice_process_hostname == NULL) {
            notice_process_hostname = (char *) calloc(HOST_NAME_MAX, sizeof(char));
            gethostname(notice_process_hostname, HOST_NAME_MAX);
        }

        // determine process name from argv[] list, if supplied
        // defines the process's name and signals that notice_init() has been called
        if (notice_process_name)
            free(notice_process_name);
        if (argc > 0 && argv != NULL) {
            char *base = strdup(argv[0]);
            notice_process_name = strdup(basename(base));
            free(base);
        } else {
            notice_process_name = strdup("");
        }
    }
    pthread_mutex_unlock(&notice_mutex);
}


/**
 * Determines whether the given severity level would generate printed output to
 * the stdout/stderr streams.
 */
static inline bool notice_print_enabled(int8_t severity)
{
    return (severity != NOTICE_MASK_NONE && severity <= notice_level_print);
}


/**
 * Determines whether the given severity level would send a notice_t LCM
 * message.
 */
static inline bool notice_send_enabled(int8_t severity)
{
    return (severity != NOTICE_MASK_NONE && severity <= notice_level_send);
}


/**
 * Determines whether the given severity level would generate speak_t LCM messages.
 */
static inline bool notice_speak_enabled(int8_t severity)
{
    return (severity != NOTICE_MASK_NONE && severity <= notice_level_speak);
}


/**
 * Outputs a notice on any of the output channels which are currently configured
 *   (via environment variables) for notices having at least the given
 *   severity level.
 *
 * Should not be called directly, but through one of the following macros:
 *    nprintf(severity, format, ...)
 *    nfailf(format, ...)
 *    nwarnf(format, ...)
 *    ninfof(format, ...)
 *    ndebugf(format, ...)
 *    ndebug1f(format, ...)
 *    ndebug2f(format, ...)
 *
 * 'severity' is one of NOTICE_T_*:
 *   if 'severity' <= env(NOTICE_ENV_PRINT):
 *     print to stderr (severity <= NOTICE_MASK_STDERR) or stdout (severity > NOTICE_MASK_STDERR)
 *   if 'severity' <= env(NOTICE_ENV_SEND):
 *     send notice_t LCM message
 *   if 'severity' <= env(NOTICE_ENV_SPEAK):
 *     send speak_t LCM message
 *
 * notice_init() should be called prior to calls to this function to ensure that
 *   the source strings in notice_t messages are set correctly (not mandatory) and/or
 *   if a specific LCM URL is to be used when sending messages.
 *
 * Returns true if a notice was dispatched for the given severity level on any of
 *   the available output channels, else false, indicating that the notice could
 *   not have been received by any party.
 */
static bool notice_dispatch(int8_t severity, const char *format, ...) __attribute__ ((unused)) __attribute__ ((format (printf, 2, 3)));
static bool notice_dispatch(int8_t severity, const char *format, ...)
{
    bool dispatched = false;
    va_list args;

    va_start(args, format);
    char *message = vsprintf_alloc(format, args);
    va_end(args);

    if (notice_process_name == NULL)
        notice_init(0, NULL, NULL); // notice_init() has not be called, use defaults

    pthread_mutex_lock(&notice_mutex);
    {
        // print to stdout/stderr
        if (notice_print_enabled(severity)) {
            FILE *outfile = (severity <= NOTICE_MASK_STDERR) ? stderr : stdout;
            if (severity <= NOTICE_T_FAILURE) {
                fprintf(outfile, "FAILURE: %s", message);
            } else if (severity <= NOTICE_T_WARNING) {
                fprintf(outfile, "WARNING: %s", message);
            } else if (severity >= NOTICE_T_DEBUG0) {
                fprintf(outfile, "DEBUG: %s", message);
            } else {
                fprintf(outfile, "%s", message);
            }

            dispatched = true;
        }

        // send notice_t LCM message
        if (notice_send_enabled(severity)) {
            // initialize necessary infrastructure for publishing LCM messages
            if (notice_lcm == NULL) {
                notice_lcm = lcm_create(notice_lcm_url);
            }

            // publish notice_t LCM message (assumes notice_t_publish() is inherently thread safe)
            notice_t notice_msg;
            notice_msg.utime = utime_now();
            notice_msg.severity = severity;
            notice_msg.process = notice_process_name;
            notice_msg.args = notice_process_args;
            notice_msg.hostname = notice_process_hostname;
            notice_msg.description = message;

            assert(notice_msg.process != NULL);
            assert(notice_msg.args != NULL);
            assert(notice_msg.hostname != NULL);
            assert(notice_msg.description != NULL);
            notice_t_publish(notice_lcm, NOTICE_CHANNEL_NOTICE, &notice_msg);

            dispatched = true;
        }

        // send speak_t LCM message to espeak
        if (notice_speak_enabled(severity)) {
            // initialize necessary infrastructure for publishing LCM messages
            if (notice_lcm == NULL) {
                notice_lcm = lcm_create(notice_lcm_url);
            }

            // publish speak_t LCM message (assumes speak_t_publish() is inherently thread safe)
            speak_t speak_msg;
            speak_msg.utime = utime_now();
            speak_msg.priority = severity;
            speak_msg.args = notice_speak_args;
            speak_msg.message = message;

            speak_t_publish(notice_lcm, NOTICE_CHANNEL_SPEAK, &speak_msg);

            dispatched = true;
        }
    }
    pthread_mutex_unlock(&notice_mutex);

    free(message);

    return dispatched;
}


/**
 * Should be called immediately before the process exists to free any resources
 * associated with the notification infrastructure.
 */
static void notice_destroy() __attribute__ ((unused));
static void notice_destroy()
{
    pthread_mutex_lock(&notice_mutex);
    {
        if (notice_lcm != NULL) {
            lcm_destroy(notice_lcm);
            notice_lcm = NULL;
        }

        if (notice_lcm_url != NULL) {
            free(notice_lcm_url);
            notice_lcm_url = NULL;
        }

        if (notice_process_name != NULL) {
            free(notice_process_name);
            notice_process_name = NULL;
        }

        if (notice_process_args != NULL) {
            free(notice_process_args);
            notice_process_args = NULL;
        }

        if (notice_process_hostname != NULL) {
            free(notice_process_hostname);
            notice_process_hostname = NULL;
        }

        if (notice_speak_args != NULL) {
            free(notice_speak_args);
            notice_speak_args = NULL;
        }

    }
    pthread_mutex_unlock(&notice_mutex);
}


#ifndef NO_NOTICE

#define nprintf(severity, ...)   notice_dispatch(severity, __VA_ARGS__)
#define nfailf(...)              notice_dispatch(NOTICE_T_FAILURE, __VA_ARGS__)
#define nwarnf(...)              notice_dispatch(NOTICE_T_WARNING, __VA_ARGS__)
#define ninfof(...)              notice_dispatch(NOTICE_T_INFO, __VA_ARGS__)
#define ndebugf(...)             notice_dispatch(NOTICE_T_DEBUG0, __VA_ARGS__)
#define ndebug1f(...)            notice_dispatch(NOTICE_T_DEBUG1, __VA_ARGS__)
#define ndebug2f(...)            notice_dispatch(NOTICE_T_DEBUG2, __VA_ARGS__)

#else

#define nprintf(severity, ...)   false
#define nfailf(...)              false
#define nwarnf(...)              false
#define ninfof(...)              false
#define ndebugf(...)             false
#define ndebug1f(...)            false
#define ndebug2f(...)            false

#endif // NO_NOTICE


// Define a macro which will assert if assertions are enabled, otherwise will
//   exit with a failure status
#ifdef NDEBUG
#define fatal(...) do { nfailf(__VA_ARGS__); exit(EXIT_FAILURE); } while (0)
#else
#define fatal(...) do { nfailf(__VA_ARGS__); assert(0); } while (0)
#endif


#ifdef __cplusplus
}
#endif

#endif // NGV_NOTICE_H_
