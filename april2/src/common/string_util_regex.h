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

#ifndef _STRING_UTIL_REGEX_H
#define _STRING_UTIL_REGEX_H

#include <stdarg.h>
#include <stdbool.h>
#include <ctype.h>

#include "zarray.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Identical to str_split() except that the delimiter can be an extended
 * regular expression, e.g.:
 *   '[[:space:]]+' or '\\s' will tokenize on any whitespace character
 *
 * Note: using an asterisk anywhere in the regex will assume a ^ at the beginning
 *
 * Returns NULL if 'regex' is not a valid regular expression.
 */
zarray_t *str_split_regex(const char *str, const char *regex);

/**
 * Identical to str_split_regex() except that delimeters matched by
 * the regex will also be returned as tokens, e.g.:
 *   zarray_t *za = str_split_regex_all(" this  is a   haystack  ", "\\s+");
 *      => [" ", "this", "  ", "is", " ", "a", "   ", "haystack", "  "]
 *
 * Returns NULL if 'regex' is not a valid regular expression.
 */
zarray_t *str_split_regex_all(const char *str, const char *regex);

/**
 * Returns an array of all of the substrings within 'str' which match the
 * supplied extended regular expression 'regex'. The original string will remain
 * unchanged, and all strings contained within the array will be newly-allocated.
 *
 * Note: using an asterisk anywhere in the regex will assume a ^ at the beginning
 *
 * It is the caller's responsibilty to free the returned zarray, as well as
 * the strings contained within it, e.g.:
 *
 *   zarray_t *za = str_match_regex("needles in a haystack", "[aeioyu]{2}");
 *      => ["ee", "ay"]
 *   zarray_vmap(za, free);
 *   zarray_destroy(za);
 */
zarray_t *str_match_regex(const char *str, const char *regex);

/**
 * Wrapper to regcomp and regexec to check if 'str' matches POSIX extended
 * regular expression 'regex'.
 * Returns 0 if a match exists (same sense as strcmp)
 */
int str_regcmp(const char *str, const char *regex);

/**
 * Returns the string up to and including the next matching string. If
 * no match is found, NULL is returned. Usually, the regular
 * expression will begin with '^', forcing the next token to match the
 * regex exactly. **/
char* string_feeder_next_regex(string_feeder_t *sf, const char *s);

#ifdef __cplusplus
}
#endif

#endif
