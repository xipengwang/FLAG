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
#include <string.h>
#include <stdint.h>
#include <pthread.h>

#include "lcmtypes/speak_t.h"
//#include "lcmtypes/comment_t.h"

#include "common/getopt.h"
#include "common/config.h"
#include "common/time_util.h"
#include "common/string_util.h"
#include "common/zhash.h"
#include "common/zmaxheap.h"
#include "common/sys_util.h"
//#include "common/active_time_sync.h"

#define SPEAK_CHAN "SPEAK.*"


typedef struct state state_t;
struct state {
  config_t *config;
    lcm_t *lcm;
    zarray_t *queue;
    pthread_mutex_t mutex;
    //uint8_t log_comment;
};

void*
speaker (void *user) {

    state_t *state = user;
    //active_time_sync_t *ats = ats_create (NULL);
    speak_t *s = NULL;
    zmaxheap_t *heap = zmaxheap_create(50);
    while (1) {
        pthread_mutex_lock(&state->mutex);

        zhash_t *seen_messages = zhash_create(sizeof(char*),
                                              sizeof(char*),
                                              zhash_str_hash,
                                              zhash_str_equals);

        for(int i = 0; i < zarray_size(state->queue); i++){
            char *old_key = NULL;
            speak_t *old_value = NULL;
            zarray_get(state->queue, i, &s);

            char *msg = strdup(s->message);
            speak_t *copy = speak_t_copy(s);
            zhash_put(seen_messages, &msg, &copy, &old_key, &old_value);

            if(old_key){
                free(old_key);
                old_key = NULL;

                if(old_value){
                    speak_t_destroy(old_value);
                    old_value = NULL;
                }
                zarray_remove_index(state->queue, i, true);
                speak_t_destroy(s);
                i--;
            }
        }
        zhash_vmap_values(seen_messages, speak_t_destroy);
        zhash_vmap_keys(seen_messages, free);
        zhash_destroy(seen_messages);

        s = NULL;

        while(zarray_size(state->queue)){
            zarray_get(state->queue, 0, &s);
            zarray_remove_index(state->queue, 0, true);
            zmaxheap_add(heap, &s, s->priority);
        }

        s = NULL;

        while(zmaxheap_size(heap)){
            zmaxheap_remove_max(heap, &s, NULL);
            zarray_add(state->queue, &s);
        }

        s = NULL;

        // pop off a speak_t if avaliable
        if (zarray_size(state->queue) > 0) {
            zarray_get(state->queue, 0, &s);
            zarray_remove_index(state->queue, 0, true);
        }

        pthread_mutex_unlock (&state->mutex);

        if (s != NULL) {
	  const char *default_cmd_fmt = "espeak %s \"%s\"";
	  const char *cmd_fmt = config_get_string(state->config, "espeak.cmd", default_cmd_fmt);
	  
            char *cmd = sprintf_alloc (cmd_fmt, s->args, s->message);

            //if(state->log_comment){
            //    comment_t comment = {
            //        .utime = ats_utime_now(ats),
            //        .comment = sprintf_alloc("espeak-%s", s->message),
            //    };
            //    comment_t_publish(state->lcm, "COMMENT", &comment);
            //    free(comment.comment);
            //}
            if(-1 == system (cmd)) {
                printf ("Failed to espeak!\n");
            }

            speak_t_destroy (s);
            free (cmd);
        }

        timeutil_usleep(5e5);
    }

    zmaxheap_vmap(heap, speak_t_destroy);
    zmaxheap_destroy(heap);

    //ats_destroy (ats);
    return NULL;
}

static void
speak_t_handler(const lcm_recv_buf_t *rbuf, const char* channel,
                const speak_t *msg, void *user) {

    state_t *state = user;
    // push this mesage onto the queue
    // add sorted insert based on priority ?
    pthread_mutex_lock (&state->mutex);
    speak_t *copy = speak_t_copy(msg);
    zarray_add(state->queue, &copy);
    pthread_mutex_unlock (&state->mutex);
}

int
main(int argc, char **argv)
{
    setlinebuf(stdout);
    setlinebuf(stderr);

    getopt_t *gopt = getopt_create();
    getopt_add_bool   (gopt, 'h', "help", 0, "Show help");
    getopt_add_string (gopt, 'c', "config", "/", "Config file path");
    //getopt_add_bool   (gopt, 'l', "log_comments", 0, "Log messages as lcm_comments also");

    // parse and print help
    if (!getopt_parse(gopt, argc, argv, 1) || getopt_get_bool(gopt,"help")) {
        printf ("Usage: %s [options]\n\n", argv[0]);
        getopt_do_usage (gopt);
        exit (EXIT_FAILURE);
    }

    char *configpath = expand_environment_variables(getopt_get_string(gopt, "config"));
    if (configpath == NULL || strlen(configpath) == 0) {
        printf ("Error: No config path supplied\n");
        printf ("Usage: %s [options]\n\n", argv[0]);
        getopt_do_usage(gopt);
        exit(EXIT_FAILURE);
    }

    state_t *state = calloc(1, sizeof(state_t));
    pthread_mutex_init(&state->mutex, NULL);
    state->queue = zarray_create(sizeof(speak_t*));
    state->lcm = lcm_create(NULL);
    state->config = config_create_path(configpath);
    if (state->config == NULL) {
        printf ("Error: Failed to get config file at '%s'\n", configpath);
        exit (EXIT_FAILURE);
    }
    free(configpath);
    //config_debug(config);

    if (!state->lcm) {
        printf ("ERROR: lcm_create() failed!\n");
        exit (EXIT_FAILURE);
    }

    // create thread
    pthread_t speaker_thread;
    pthread_create(&speaker_thread, NULL, speaker, state);

    speak_t_subscription_t *ssub = speak_t_subscribe (state->lcm, SPEAK_CHAN, &speak_t_handler, state);

    while (1) {
        lcm_handle (state->lcm);
    }

    // clean up
    speak_t_unsubscribe(state->lcm, ssub);
    pthread_join(speaker_thread, NULL);
    pthread_mutex_destroy (&state->mutex);
    zarray_vmap(state->queue, speak_t_destroy);
    zarray_destroy(state->queue);
    lcm_destroy(state->lcm);
    config_destroy(state->config);
    getopt_destroy(gopt);

    return EXIT_SUCCESS;

}
