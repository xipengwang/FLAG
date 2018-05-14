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

#include "http_advertiser.h"
#include "lcmtypes/http_advert_t.h"

#include <unistd.h>
#include <pthread.h>
#include <stdbool.h>
#include <assert.h>

#include "common/time_util.h"

struct http_advertiser
{
    lcm_t * lcm;
    int running;
    pthread_t pt;
    http_advert_t advert;
};

void * run_thread(void * impl)
{
    http_advertiser_t * ad = impl;
    while(ad->running)
    {
        ad->advert.utime = utime_now();
        http_advert_t_publish(ad->lcm, "HTTP_ADVERTS", &ad->advert);
        sleep(1);
    }

    free(ad->advert.name);
    free(ad->advert.desc);
    free(ad);
    return NULL;
}

http_advertiser_t * http_advertiser_create(lcm_t * lcm, int32_t port, char* name, char* desc)
{
    assert(lcm);
    assert(name);
    assert(desc);
    http_advertiser_t * ad = calloc(1, sizeof(http_advertiser_t));
    ad->running = true;
    ad->lcm = lcm;
    ad->advert.port = port;
    ad->advert.name = strdup(name);
    ad->advert.desc = strdup(desc);

    pthread_create(&ad->pt, NULL, run_thread, ad);
    return ad;
}

void http_advertiser_destroy(http_advertiser_t * ad)
{
    ad->running = false;
    pthread_join(ad->pt, NULL);
}
