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
#include <stdint.h>
#include <ctype.h>
#include <string.h>
#include <unistd.h>
#include <stdarg.h>
#include <stdlib.h>
#include <assert.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <inttypes.h>
#include "lcmgen.h"
#include <lcm/lcm.h>
#include <lcm/lcm_coretypes.h>

#include "common/getopt.h"
#include "common/zhash.h"
#include "common/time_util.h"
#include "tokenize.h"

typedef struct state
{
    lcm_t * lcm;
    lcmgen_t * gen;
    bool list;
    zhash_t * timing;

}state_t;

typedef struct time_data
{
    int64_t last;
    int64_t ave;
    int64_t variance;
}time_data_t;



void on_msg (const lcm_recv_buf_t *rbuf,
        const char *channel, void *user_data)
{
    state_t * state = user_data;
    volatile int64_t time = rbuf->recv_utime;

    if(utime_now() - time > 10000)
    {
        static int warned_latent = 0;
        if(warned_latent < 10)
        {
            printf("WARN: dropping messages due to latency\n");
            warned_latent = 1;
        }
        return;
    }
    time_data_t * td;
    zhash_get(state->timing, &channel, &td);
    if(td == NULL)
    {
        td = calloc(1, sizeof(time_data_t));
        zhash_put(state->timing, &channel, &td, NULL, NULL);
    }

    int64_t hash;
    int64_t thislen = __int64_t_decode_array(rbuf->data, 0, rbuf->data_size, &hash, 1);

    lcm_struct_t * st = NULL;
    int ntypes = zarray_size(state->gen->structs);
    for(int i = 0; i < ntypes; i++)
    {
        lcm_struct_t * this_st = NULL;
        zarray_get(state->gen->structs, i, &this_st);
        int64_t th = this_st->rhash;
        if(th == hash)
        {
            st = this_st;
            break;
        }
    }

    int64_t diff = time - td->last;
    //td->last = time;
    //td->ave = 0.5 * diff + 0.5 * td->ave;
    //td->variance = 0.5 * abs(diff - td->ave) + 0.5 * td->variance;

    printf("%3f %3" PRId64 " %-30s %-s\n",
           1000000./td->ave,
           td->variance/1000,
           channel,
           (st?st->structname->shortname:"????"));

    if(st && !state->list)
        decode_type_recurse(state->gen, st, rbuf->data, rbuf->data_size, thislen, 0);
}

int main(int argc, char *argv[])
{
    getopt_t *gopt = getopt_create();

    getopt_add_bool  (gopt, 'h',  "help",     0,    "Show this help");
    getopt_add_bool  (gopt, 't',  "tokenize", 0,    "Show tokenization");
    getopt_add_bool  (gopt, 'l',  "list",     0,    "List channels, don't expand");
    getopt_add_bool  (gopt, 'd',  "debug",    0,    "Show parsed file");
    getopt_add_string(gopt, 'u',  "url",      "",    "lcm url");
    getopt_add_string(gopt, 'r',  "regex",    ".*", "channel regex");

    if (!getopt_parse(gopt, argc, argv, 1) || getopt_get_bool(gopt,"help")) {
        printf("Usage: %s [options] <input files>\n\n", argv[0]);
        getopt_do_usage(gopt);
        return 0;
    }

    lcmgen_t *lcmg = lcmgen_create();
    lcmg->gopt = gopt;

    const zarray_t * files = getopt_get_extra_args(gopt);
    for (unsigned int i = 0; i < zarray_size(files); i++) {
        char *path;
        zarray_get(files, i, &path);

        int res = lcmgen_handle_file(lcmg, path);
        if (res)
            return res;
    }

    if (getopt_get_bool(gopt, "tokenize")) {
        return 0;
    }

    if (getopt_get_bool(gopt, "debug")) {
        lcmgen_dump(lcmg);
    }


    state_t * state = calloc(1,sizeof(state_t));
    state->lcm = lcm_create(getopt_get_string(gopt, "url"));
    state->gen = lcmg;
    state->list = getopt_get_bool(gopt, "list");
    state->timing = zhash_create(sizeof(char*), sizeof(time_data_t), zhash_str_hash, zhash_str_equals);
    resolve_hashes(lcmg);

    lcm_subscribe(state->lcm, getopt_get_string(gopt, "regex"), on_msg, state);

    while(1)
    {
        lcm_handle(state->lcm);
    }

    return 0;
}
