/* Copyright (C) 2013-2016, The Regents of The University of Michigan.
All rights reserved.

This software was developed in the APRIL Robotics Lab under the
direction of Edwin Olson, ebolson@umich.edu. This software may be
available under alternative licensing terms; contact the address above.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are those
of the authors and should not be interpreted as representing official policies,
either expressed or implied, of the Regents of The University of Michigan.
*/

#include <regex.h>
#include <assert.h>
#include <ctype.h>
#include <errno.h>
#include <string.h>
#include <stdarg.h>
#include <stdlib.h>
#include <stdio.h>

#include "string_util.h"
#include "zarray.h"

zarray_t *str_match_regex(const char *str, const char *regex)
{
    assert(str != NULL);
    assert(regex != NULL);

    regex_t regex_comp;
    if (regcomp(&regex_comp, regex, REG_EXTENDED) != 0)
        return NULL;

    zarray_t *matches = zarray_create(sizeof(char*));

    void *ptr = (void*) str;
    regmatch_t match;
    while (regexec(&regex_comp, ptr, 1, &match, 0) != REG_NOMATCH)
    {
        if (match.rm_so == -1)
            break;

        if (match.rm_so == match.rm_eo) // ambiguous regex
            break;

        size_t length = (size_t) match.rm_eo - match.rm_so;
        char *substr = malloc(length+1);
        memset(substr, 0, length+1);

        memcpy(substr, ptr+match.rm_so, length);

        ptr = ptr + match.rm_eo;

        zarray_add(matches, &substr);
    }

    regfree(&regex_comp);
    return matches;
}

zarray_t *str_split_regex(const char *str, const char *regex)
{
    assert(str != NULL);
    assert(regex != NULL);

    regex_t regex_comp;
    if (regcomp(&regex_comp, regex, REG_EXTENDED) != 0)
        return NULL;

    zarray_t *matches = zarray_create(sizeof(char*));

    size_t origlen = strlen(str);
    void *end = ((void*) str) + origlen;
    void *ptr = (void*) str;

    regmatch_t match;
    while (regexec(&regex_comp, ptr, 1, &match, 0) != REG_NOMATCH)
    {
        //printf("so %3d eo %3d\n", match.rm_so, match.rm_eo);

        if (match.rm_so == -1)
            break;

        if (match.rm_so == match.rm_eo) // ambiguous regex
            break;

        void *so = ((void*) ptr) + match.rm_so;

        int length = so - ptr;

        if (length > 0) {
            char *substr = malloc(length+1);
            memset(substr, 0, length+1); // XXX a bit silly given next line
            memcpy(substr, ptr, length);

            //printf("match '%s'\n", substr);
            //fflush(stdout);

            zarray_add(matches, &substr);
        }

        ptr = ptr + match.rm_eo;
    }

    if (ptr - (void*) str != origlen) {
        int length = end - ptr;
        char *substr = malloc(length+1);
        memset(substr, 0, length+1);

        memcpy(substr, ptr, length);

        zarray_add(matches, &substr);
    }


    regfree(&regex_comp);
    return matches;
}

zarray_t *str_split_regex_all(const char *str, const char *regex)
{
    assert(str != NULL);
    assert(regex != NULL);

    regex_t regex_comp;
    if (regcomp(&regex_comp, regex, REG_EXTENDED) != 0)
        return NULL;

    zarray_t *matches = zarray_create(sizeof(char*));

    size_t origlen = strlen(str);
    void *end = ((void*) str) + origlen;
    void *ptr = (void*) str;

    regmatch_t match;
    while (regexec(&regex_comp, ptr, 1, &match, 0) != REG_NOMATCH)
    {
        //printf("so %3d eo %3d\n", match.rm_so, match.rm_eo);

        if (match.rm_so == -1)
            break;

        if (match.rm_so == match.rm_eo) // ambiguous regex
            break;

        void *so = ((void*) ptr) + match.rm_so;
        int length = so - ptr;

        // Copy the token
        if (match.rm_so > 0 && length > 0) {
            char *substr = malloc(length+1);
            memset(substr, 0, length+1);
            memcpy(substr, ptr, length);

            //printf("match '%s'\n", substr);
            //fflush(stdout);

            zarray_add(matches, &substr);
        }

        // Copy the delimeters
        length = match.rm_eo-match.rm_so;
        if (length > 0) {
            char *substr = malloc(length+1);
            memset(substr, 0, length+1);
            memcpy(substr, ptr+match.rm_so, length);
            zarray_add(matches, &substr);
        }


        ptr = ptr + match.rm_eo;
    }

    if (ptr - (void*) str != origlen) {
        int length = end - ptr;
        char *substr = malloc(length+1);
        memset(substr, 0, length+1);

        memcpy(substr, ptr, length);

        zarray_add(matches, &substr);
    }


    regfree(&regex_comp);
    return matches;
}

int str_regcmp(const char *str, const char *regex)
{
    assert(str != NULL);
    assert(regex != NULL);

    regex_t regex_comp;
    if (regcomp(&regex_comp, regex, REG_EXTENDED) != 0)
        return 1; // default to no match on error

    int result = regexec(&regex_comp, str, 0, NULL, 0);

    regfree(&regex_comp);
    return result;
}

char * string_feeder_next_regex(string_feeder_t *sf, const char *regex)
{
    assert(sf != NULL);
    assert(regex != NULL);

    regex_t regex_comp;
    if (regcomp(&regex_comp, regex, REG_EXTENDED) != 0)
        return NULL;

    regmatch_t match;
    if (regexec(&regex_comp, &sf->s[sf->pos], 1, &match, 0) == REG_NOMATCH)
        return NULL;

    int length = match.rm_eo;

    char *s = malloc(length + 1);
    memcpy(s, &sf->s[sf->pos], length);
    s[length] = 0;
    sf->pos += length;
    return s;
}
