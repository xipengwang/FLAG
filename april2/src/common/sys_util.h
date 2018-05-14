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

#ifndef __SYS_UTIL_H_
#define __SYS_UTIL_H_


#include <wordexp.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Returns a newly-allocated version of 'str' in which all environment variables
 * have been expanded.
 *
 * NOTE: Leading/trailing whitespace of 'str' may not be maintained.
 *
 * It is the caller's responsibility to free the returned string.
 */
char *expand_environment_variables(const char *str);


#ifdef __cplusplus
}
#endif



#endif
