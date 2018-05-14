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
#include <string.h>
#include <assert.h>

#include "common/base64.h"

int main(int argc, char *argv[])
{
    const char *tests[] = { "YW55IGNhcm5hbCBwbGVhc3VyZS4=", "any carnal pleasure.",
                            "YW55IGNhcm5hbCBwbGVhc3VyZQ==", "any carnal pleasure",
                            "YW55IGNhcm5hbCBwbGVhc3Vy", "any carnal pleasur",
                            "YW55IGNhcm5hbCBwbGVhc3U=", "any carnal pleasu",
                            "YW55IGNhcm5hbCBwbGVhcw==", "any carnal pleas",
                            "", "",
                            NULL };

    for (int i = 0; tests[i] != NULL; i += 2) {
        const char *encoded = tests[i];
        char decoded[1024];
        int decoded_len;

        int res = base64_decode(encoded, decoded, &decoded_len);
//        printf("%s %s (%d) %d\n", encoded, decoded, decoded_len, res);
        assert(decoded_len == strlen(tests[i+1]));
        assert(!strcmp(tests[i+1], decoded));

        char encoded2[1024];
        int encoded2_len;

        int res2 = base64_encode(tests[i+1], strlen(tests[i+1]), encoded2, &encoded2_len);
//        printf("%s %s (%d) %d\n", tests[i], encoded2, encoded2_len, res);
        assert(encoded2_len == strlen(tests[i]));
        assert(!strcmp(tests[i], encoded2));
    }
}
