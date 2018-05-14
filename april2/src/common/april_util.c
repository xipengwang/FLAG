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

#include "april_util.h"
#include <stdlib.h>
#include <stdio.h>

#include "string_util.h"

static char * april_util_root_path_char = NULL;

// Return the path to the root of the april2 tree;
// This can be set by:
//  * environment variable APRIL2_PATH
char * april_util_root_path()
{
    if(april_util_root_path_char)
        return april_util_root_path_char;

    char* envpath = getenv("APRIL2_PATH");
    april_util_root_path_char = envpath;
    if (april_util_root_path_char)
        return april_util_root_path_char;

    char fname[1028] = __FILE__;
    char* esc = "/";
    int last = str_last_indexof(fname, esc);
    assert(last >= 0);
    fname[last] = '\0';
    char fullname[2048];
    snprintf(fullname, 2047, "%s/../../",fname);
    april_util_root_path_char = realpath(fullname, NULL);
    return april_util_root_path_char;

}
