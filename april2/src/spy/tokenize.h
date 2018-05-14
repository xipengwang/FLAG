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

#ifndef _TOKENIZE_H
#define _TOKENIZE_H

#include <stdio.h>

typedef struct tokenize tokenize_t;

typedef enum {
    LCM_TOK_INVALID,
    LCM_TOK_EOF,
    LCM_TOK_COMMENT,
    LCM_TOK_OTHER
} lcm_token_type_t;

/** Tokenizer incrementally tokenizes an input stream. **/
struct tokenize
{
    // current token
    char *token;

    // bytes allocated for token.
    int token_capacity;

    int token_line, token_column;

    // info about the last returned character from next_char.
    int current_char;
    int current_line, current_column;

    // If there is an ungetc() pending, unget_char >0 and contains the
    // char. unget_line and unget_column are the line and column of
    // the unget'd char.
    int unget_char;
    int unget_line, unget_column;

    // the current line, and our position in the input stream.
    // (ignoring the occurence of ungets.)
    char *buffer;
    int buffer_line, buffer_column;
    int buffer_len;

    char *path;
    FILE *f;

    // do we have a token ready?
    int hasnext;

    lcm_token_type_t token_type;
};

tokenize_t *tokenize_create(const char *path);
void tokenize_destroy(tokenize_t *t);
int tokenize_next(tokenize_t *t);
int tokenize_peek(tokenize_t *t);

#endif
