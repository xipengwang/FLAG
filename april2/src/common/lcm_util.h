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

#ifndef _LCM_UTIL_H
#define _LCM_UTIL_H

#include <lcm/lcm.h>
#include <assert.h>
#include <poll.h>
#include <pthread.h>

#ifdef __cplusplus
extern "C" {
#endif

int lcm_handle_timeout_mutex (lcm_t *lcm, int timeout_milis, pthread_mutex_t * mutex)
{
    assert(lcm != NULL);
    assert(mutex != NULL);
    assert(timeout_milis != 0);

    int lcm_fd = lcm_get_fileno(lcm);
    assert(lcm_fd > 0);

    struct pollfd pollfds[] = { { .fd = lcm_fd, .events = POLLIN, .revents = 0 } };

    int poll_res = poll(pollfds, 1, timeout_milis);

    int lcm_handle_result = -1;

    if (pollfds[0].revents == POLLIN) {
        pthread_mutex_lock(mutex);
        lcm_handle_result = lcm_handle(lcm);
        pthread_mutex_unlock(mutex);
    }

    return lcm_handle_result;
}

int lcm_handle_mutex (lcm_t *lcm, pthread_mutex_t * mutex)
{
    return lcm_handle_timeout_mutex (lcm, -1, mutex);
}

#ifdef __cplusplus
}
#endif

#endif
