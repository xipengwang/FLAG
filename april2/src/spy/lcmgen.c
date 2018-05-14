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

#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <inttypes.h>
#include <ctype.h>
#include <unistd.h>
#include <float.h>
#include <sys/types.h>
#include <sys/stat.h>

#include <lcm/lcm_coretypes.h>

#include "common/json.h"
#include "common/string_util.h"
#include "common/zhash.h"

#include "lcmgen.h"
#include "tokenize.h"


/** The LCM grammar is implemented here with a recursive-descent parser.
    handle_file is the top-level function, which calls parse_struct/parse_enum,
    and so on.

    Every LCM type has an associated "signature", which is a hash of
    various components of its delcaration. If the declaration of a
    signature changes, the hash changes with high probability.

    Note that this implementation is sloppy about memory allocation:
    we don't worry about freeing memory since the program will exit
    after parsing anyway.
**/

lcm_struct_t *parse_struct(lcmgen_t *lcm, const char *lcmfile, tokenize_t *t);
lcm_enum_t *parse_enum(lcmgen_t *lcm, const char *lcmfile, tokenize_t *t);

// lcm's built-in types. Note that unsigned types are not present
// because there is no safe java implementation. Really, you don't
// want to add unsigned types.
static const char *primitive_types[] = { "int8_t",
                                         "int16_t",
                                         "int32_t",
                                         "int64_t",
                                         "byte",
                                         "float",
                                         "double",
                                         "string",
                                         "boolean",
                                         NULL};

// which types can be legally used as array dimensions?
static const char *array_dimension_types[] = { "int8_t",
                                               "int16_t",
                                               "int32_t",
                                               "int64_t",
                                               NULL};

// which types can be legally used as const values?
static const char *const_types[] = { "int8_t",
                                     "int16_t",
                                     "int32_t",
                                     "int64_t",
                                     "float",
                                     "double",
                                     NULL };

// Given NULL-terminated array of strings "ts", does "t" appear in it?
static int string_in_array(const char *t, const char **ts)
{
    for (int i = 0; ts[i] != NULL; i++) {
        if (!strcmp(ts[i], t))
            return 1;
    }

    return 0;
}

int lcm_is_primitive_type(const char *t)
{
    return string_in_array(t, primitive_types);
}

int lcm_is_array_dimension_type(const char *t)
{
    return string_in_array(t, array_dimension_types);
}

int lcm_is_legal_member_name(const char *t)
{
    return isalpha(t[0]) || t[0]=='_';
}

int lcm_is_legal_const_type(const char *t)
{
    return string_in_array(t, const_types);
}

// Make the hash dependent on the value of the given character. The
// order that hash_update is called in IS important.
static int64_t hash_update(int64_t v, char c)
{
    v = ((v<<8) ^ (v>>55)) + c;

    return v;
}

// Make the hash dependent on each character in a string.
static int64_t hash_string_update(int64_t v, const char *s)
{
    v = hash_update(v, strlen(s));

    for (; *s != 0; s++)
        v = hash_update(v, *s);

    return v;
}

// Create a parsing context
lcmgen_t *lcmgen_create()
{
    lcmgen_t *lcmgen = (lcmgen_t*) calloc(1, sizeof(lcmgen_t));
    lcmgen->structs = zarray_create(sizeof(lcm_struct_t*));
    lcmgen->enums = zarray_create(sizeof(lcm_enum_t*));
    lcmgen->package = strdup("");
    lcmgen->comment_doc = NULL;

    return lcmgen;
}

// Parse a type into package and class name.  If no package is
// specified, we will try to use the package from the last specified
// "package" directive, like in Java.
lcm_typename_t *lcm_typename_create(lcmgen_t *lcmgen, const char *lctypename)
{
    lcm_typename_t *lt = (lcm_typename_t*) calloc(1, sizeof(lcm_typename_t));

    lt->lctypename = strdup(lctypename);

    // package name: everything before the last ".", or "" if there is no "."
    //
    // shortname: everything after the last ".", or everything if
    // there is no "."
    //
    char *tmp = strdup(lctypename);
    char *rtmp = strrchr(tmp, '.');
    if (rtmp == NULL) {
        lt->shortname = tmp;
        if (lcm_is_primitive_type(lt->shortname)) {
            lt->package = strdup("");
        } else {
            // we're overriding the package name using the last directive.
            lt->package = strdup(lcmgen->package);
            lt->lctypename = sprintf_alloc("%s%s%s", lt->package,
                                          strlen(lcmgen->package)>0 ? "." : "",
                                          lt->shortname);
        }
    } else {
        lt->package = tmp;
        *rtmp = 0;
        lt->shortname = &rtmp[1];
    }

    const char* package_prefix = "";
    if (strlen(package_prefix)>0 && !lcm_is_primitive_type(lt->shortname)){
        lt->package = sprintf_alloc("%s%s%s",
                                      package_prefix,
                                      strlen(lt->package) > 0 ? "." : "",
                                      lt->package);
        lt->lctypename = sprintf_alloc("%s.%s", lt->package, lt->shortname);
    }

    return lt;
}

lcm_struct_t *lcm_struct_create(lcmgen_t *lcmgen, const char *lcmfile, const char *structname)
{
    lcm_struct_t *lr = (lcm_struct_t*) calloc(1, sizeof(lcm_struct_t));
    lr->lcmfile    = strdup(lcmfile);
    lr->structname = lcm_typename_create(lcmgen, structname);
    lr->members    = zarray_create(sizeof(lcm_member_t*));
    lr->constants  = zarray_create(sizeof(lcm_constant_t*));
    lr->enums      = zarray_create(sizeof(lcm_enum_t*));
    return lr;
}

lcm_constant_t *lcm_constant_create(const char *type, const char *name, const char *val_str)
{
    lcm_constant_t *lc = (lcm_constant_t*) calloc(1, sizeof(lcm_constant_t));
    lc->lctypename = strdup(type);
    lc->membername = strdup(name);
    lc->val_str = strdup(val_str);
    // don't fill in the value
    return lc;
}

void lcm_constant_destroy(lcm_constant_t *lc)
{
    free(lc->lctypename);
    free(lc->membername);
    free(lc);
}

lcm_enum_t *lcm_enum_create(lcmgen_t *lcmgen, const char *lcmfile, const char *name)
{
    lcm_enum_t *le = (lcm_enum_t*) calloc(1, sizeof(lcm_enum_t));
    le->lcmfile  = strdup(lcmfile);
    le->enumname = lcm_typename_create(lcmgen, name);
    le->values   = zarray_create(sizeof(lcm_enum_value_t*));

    return le;
}

lcm_enum_value_t *lcm_enum_value_create(const char *name)
{
    lcm_enum_value_t *lev = (lcm_enum_value_t*) calloc(1, sizeof(lcm_enum_t));

    lev->valuename = strdup(name);

    return lev;
}

lcm_member_t *lcm_member_create()
{
    lcm_member_t *lm = (lcm_member_t*) calloc(1, sizeof(lcm_member_t));
    lm->dimensions = zarray_create(sizeof(lcm_dimension_t*));
    return lm;
}

//lcm_constant_t *lcm_constant_create(lcmgen_t *lcmgen, const char *lcmfile

int64_t lcm_struct_hash(lcm_struct_t *lr)
{
    int64_t v = 0x12345678;

    // NO: Purposefully, we do NOT include the structname in the hash.
    // this allows people to rename data types and still have them work.
    //
    // In contrast, we DO hash the types of a structs members (and their names).
    //  v = hash_string_update(v, lr->structname);

    for (unsigned int i = 0; i < zarray_size(lr->members); i++) {
        lcm_member_t *lm = NULL;
        zarray_get(lr->members, i, &lm);

        // hash the member name
        v = hash_string_update(v, lm->membername);

        // if the member is a primitive type, include the type
        // signature in the hash. Do not include them for compound
        // members, because their contents will be included, and we
        // don't want a struct's name change to break the hash.
        if (lcm_is_primitive_type(lm->type->lctypename))
            v = hash_string_update(v, lm->type->lctypename);

        // hash the dimensionality information
        int ndim = zarray_size(lm->dimensions);
        v = hash_update(v, ndim);
        for (int j = 0; j < ndim; j++) {
            lcm_dimension_t *dim = NULL;
            zarray_get(lm->dimensions, j, &dim);
            v = hash_update(v, dim->mode);
            v = hash_string_update(v, dim->size);
        }
    }

    return v;
}

// The hash for LCM enums is defined only by the name of the enum;
// this allows bit declarations to be added over time.
int64_t lcm_enum_hash(lcm_enum_t *le)
{
    int64_t v = 0x87654321;

    v = hash_string_update(v, le->enumname->shortname);
    return v;
}

// semantic error: it parsed fine, but it's illegal. (we don't try to
// identify the offending token). This function does not return.
void semantic_error(tokenize_t *t, const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);

    printf("\n");
    vprintf(fmt, ap);
    printf("\n");

    printf("%s : %i\n", t->path, t->token_line);
    printf("%s", t->buffer);

    va_end(ap);
    fflush(stdout);
}

// semantic warning: it parsed fine, but it's dangerous.
void semantic_warning(tokenize_t *t, const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);

    printf("\n");
    vprintf(fmt, ap);
    printf("\n");

    printf("%s : %i\n", t->path, t->token_line);
    printf("%s", t->buffer);

    va_end(ap);
}

// parsing error: we cannot continue. This function does not return.
void parse_error(tokenize_t *t, const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);

    printf("\n");
    vprintf(fmt, ap);
    printf("\n");

    printf("%s : %i\n", t->path, t->token_line);
    printf("%s", t->buffer);
    for (int i = 0; i < t->token_column; i++) {
        if (isspace(t->buffer[i]))
            printf("%c", t->buffer[i]);
        else
            printf(" ");
    }
    printf("^\n");

    va_end(ap);
    fflush(stdout);
    _exit(1);
}

// Consume any available comments and store them in lcmgen->comment_doc
void parse_try_consume_comment(lcmgen_t* lcmgen, tokenize_t* t,
    int store_comment_doc)
{
    if (store_comment_doc) {
        free(lcmgen->comment_doc);
        lcmgen->comment_doc = NULL;
    }

    while (tokenize_peek(t) != EOF && t->token_type == LCM_TOK_COMMENT) {
        tokenize_next(t);

        if (store_comment_doc) {
            if (!lcmgen->comment_doc) {
                lcmgen->comment_doc = strdup(t->token);
            } else {
                char* orig = lcmgen->comment_doc;
                lcmgen->comment_doc = sprintf_alloc("%s\n%s", orig, t->token);
                free(orig);
            }
        }
    }
}

// If the next non-comment token is "tok", consume it and return 1. Else,
// return 0
int parse_try_consume(tokenize_t *t, const char *tok)
{
    parse_try_consume_comment(NULL, t, 0);
    int res = tokenize_peek(t);
    if (res == EOF)
        parse_error(t, "End of file while looking for %s.", tok);

    res = (t->token_type != LCM_TOK_COMMENT && !strcmp(t->token, tok));

    // consume if the token matched
    if (res)
        tokenize_next(t);

    return res;
}

// Consume the next token. If it's not "tok", an error is emitted and
// the program exits.
void parse_require(tokenize_t *t, char *tok)
{
    parse_try_consume_comment(NULL, t, 0);
    int res;
    do {
        res = tokenize_next(t);
    } while (t->token_type == LCM_TOK_COMMENT);

    if (res == EOF || strcmp(t->token, tok))
        parse_error(t, "expected token %s", tok);

}

// require that the next token exist (not EOF). Description is a
// human-readable description of what was expected to be read.
void tokenize_next_or_fail(tokenize_t *t, const char *description)
{
    int res = tokenize_next(t);
    if (res == EOF)
        parse_error(t, "End of file reached, expected %s.", description);
}

int parse_const(lcmgen_t *lcmgen, lcm_struct_t *lr, tokenize_t *t)
{
    parse_try_consume_comment(lcmgen, t, 0);
    tokenize_next_or_fail(t, "type identifier");

    // get the constant type
    if (!lcm_is_legal_const_type(t->token))
        parse_error(t, "invalid type for const");
    char *lctypename = strdup(t->token);

another_constant:
    // get the member name
    parse_try_consume_comment(lcmgen, t, 0);
    tokenize_next_or_fail(t, "name identifier");
    if (!lcm_is_legal_member_name(t->token))
        parse_error(t, "Invalid member name: must start with [a-zA-Z_].");
    char *membername = strdup(t->token);

    // make sure this name isn't already taken.
    if (lcm_find_member(lr, t->token) != NULL)
        semantic_error(t, "Duplicate member name '%s'.", t->token);
    if (lcm_find_const(lr, t->token) != NULL)
        semantic_error(t, "Duplicate member name '%s'.", t->token);

    // get the value
    parse_require(t, "=");
    parse_try_consume_comment(lcmgen, t, 0);
    tokenize_next_or_fail(t, "constant value");

    // create a new const member
    lcm_constant_t *lc = lcm_constant_create(lctypename, membername, t->token);

    // Attach the last comment if one was defined.
    if (lcmgen->comment_doc) {
      lc->comment = lcmgen->comment_doc;
      lcmgen->comment_doc = NULL;
    }

    char *endptr = NULL;
    if (!strcmp(lctypename, "int8_t")) {
        long long v = strtoll(t->token, &endptr, 0);
        if (endptr == t->token || *endptr != '\0')
            parse_error(t, "Expected integer value");
        if (v < INT8_MIN || v > INT8_MAX)
            semantic_error(t, "Integer value out of bounds for int8_t");
        lc->val.i8 = (int8_t)v;
    } else if (!strcmp(lctypename, "int16_t")) {
        long long v = strtoll(t->token, &endptr, 0);
        if (endptr == t->token || *endptr != '\0')
            parse_error(t, "Expected integer value");
        if (v < INT16_MIN || v > INT16_MAX)
            semantic_error(t, "Integer value out of range for int16_t");
        lc->val.i16 = (int16_t)v;
    } else if (!strcmp(lctypename, "int32_t")) {
        long long v = strtoll(t->token, &endptr, 0);
        if (endptr == t->token || *endptr != '\0')
            parse_error(t, "Expected integer value");
        if (v < INT32_MIN || v > INT32_MAX)
            semantic_error(t, "Integer value out of range for int32_t");
        lc->val.i32 = (int32_t)v;
    } else if (!strcmp(lctypename, "int64_t")) {
        long long v = strtoll(t->token, &endptr, 0);
        if (endptr == t->token || *endptr != '\0')
            parse_error(t, "Expected integer value");
        lc->val.i64 = (int64_t)v;
    } else if (!strcmp(lctypename, "float")) {
        double v = strtod(t->token, &endptr);
        if (endptr == t->token || *endptr != '\0')
            parse_error(t, "Expected floating point value");
        if (v > FLT_MAX || v < -FLT_MAX)
            semantic_error(t, "Floating point value out of range for float");
        lc->val.f = (float)v;
    } else if (!strcmp(lctypename, "double")) {
        double v = strtod(t->token, &endptr);
        if (endptr == t->token || *endptr != '\0')
            parse_error(t, "Expected floating point value");
        lc->val.d = v;
    } else {
        fprintf(stderr, "[%s]\n", t->token);
        assert(0);
    }

    zarray_add(lr->constants, &lc);

    free(membername);

    if (parse_try_consume(t, ",")) {
        goto another_constant;
    }

    free(lctypename);

    parse_require(t, ";");
    return 0;
}

// parse a member declaration. This looks long and scary, but most of
// the code is for semantic analysis (error checking)
int parse_member(lcmgen_t *lcmgen, lcm_struct_t *lr, tokenize_t *t)
{
    lcm_typename_t *lt = NULL;

    // Read a type specification. Then read members (multiple
    // members can be defined per-line.) Each member can have
    // different array dimensionalities.

    // inline type declaration?
    if (parse_try_consume(t, "struct")) {
        parse_error(t, "recursive structs not implemented.");
    } else if (parse_try_consume(t, "enum")) {
        parse_error(t, "recursive enums not implemented.");
    } else if (parse_try_consume(t, "const")) {
        return parse_const(lcmgen, lr, t);
    }

    // standard declaration
    parse_try_consume_comment(lcmgen, t, 0);
    tokenize_next_or_fail(t, "type identifier");

    if (!isalpha(t->token[0]) && t->token[0]!='_')
        parse_error(t, "invalid type name");

    // A common mistake is use 'int' as a type instead of 'intN_t'
    if(!strcmp(t->token, "int"))
        semantic_warning(t, "int type should probably be int8_t, int16_t, int32_t, or int64_t");

    lt = lcm_typename_create(lcmgen, t->token);

    while (1) {

        // get the lcm type name
        parse_try_consume_comment(lcmgen, t, 0);
        tokenize_next_or_fail(t, "name identifier");

        if (!lcm_is_legal_member_name(t->token))
            parse_error(t, "Invalid member name: must start with [a-zA-Z_].");

        // make sure this name isn't already taken.
        if (lcm_find_member(lr, t->token) != NULL)
            semantic_error(t, "Duplicate member name '%s'.", t->token);
        if (lcm_find_const(lr, t->token) != NULL)
            semantic_error(t, "Duplicate member name '%s'.", t->token);

        // create a new member
        lcm_member_t *lm = lcm_member_create();
        lm->type = lt;
        lm->membername = strdup(t->token);
        if (lcmgen->comment_doc) {
            lm->comment = lcmgen->comment_doc;
            lcmgen->comment_doc = NULL;
        }
        zarray_add(lr->members, &lm);

        // (multi-dimensional) array declaration?
        while (parse_try_consume(t, "[")) {

            // pull out the size of the dimension, either a number or a variable name.
            parse_try_consume_comment(lcmgen, t, 0);
            tokenize_next_or_fail(t, "array size");

            lcm_dimension_t *dim = (lcm_dimension_t*) calloc(1, sizeof(lcm_dimension_t));

            if (isdigit(t->token[0])) {
                // we have a constant size array declaration.
                int sz = strtol(t->token, NULL, 0);
                if (sz <= 0)
                    semantic_error(t, "Constant array size must be > 0");

                dim->mode = LCM_CONST;
                dim->size = strdup(t->token);

            } else {
                // we have a variable sized declaration.
                if (t->token[0]==']')
                    semantic_error(t, "Array sizes must be declared either as a constant or variable.");
                if (!lcm_is_legal_member_name(t->token))
                    semantic_error(t, "Invalid array size variable name: must start with [a-zA-Z_].");

                // make sure the named variable is
                // 1) previously declared and
                // 2) an integer type
                int okay = 0;

                for (unsigned int i = 0; i < zarray_size(lr->members); i++) {
                    lcm_member_t *thislm;
                    zarray_get(lr->members, i, &thislm);
                    if (!strcmp(thislm->membername, t->token)) {
                        if (zarray_size(thislm->dimensions) != 0)
                            semantic_error(t, "Array dimension '%s' must be not be an array type.", t->token);
                        if (!lcm_is_array_dimension_type(thislm->type->lctypename))
                            semantic_error(t, "Array dimension '%s' must be an integer type.", t->token);
                        okay = 1;
                        break;
                    }
                }
                if (!okay)
                    semantic_error(t, "Unknown variable array index '%s'. Index variables must be declared before the array.", t->token);

                dim->mode = LCM_VAR;
                dim->size = strdup(t->token);
            }
            parse_require(t, "]");

            // increase the dimensionality of the array by one dimension.
            zarray_add(lm->dimensions, &dim);
        }

        if (!parse_try_consume(t, ","))
            break;
    }

    parse_require(t, ";");

    return 0;
}

int parse_enum_value(lcm_enum_t *le, tokenize_t *t)
{
    tokenize_next_or_fail(t, "enum name");

    lcm_enum_value_t *lev = lcm_enum_value_create(t->token);

    if (parse_try_consume(t, "=")) {
        tokenize_next_or_fail(t, "enum value literal");

        lev->value = strtol(t->token, NULL, 0);
    } else {
        // the didn't specify the value, compute the next largest value
        int32_t max = 0;

        for (unsigned int i = 0; i < zarray_size(le->values); i++) {
            lcm_enum_value_t *tmp;
            zarray_get(le->values, i, &tmp);
            if (tmp->value > max)
                max = tmp->value;
        }

        lev->value = max + 1;
    }

    // make sure there aren't any duplicate values
    for (unsigned int i = 0; i < zarray_size(le->values); i++) {
        lcm_enum_value_t *tmp;
        zarray_get(le->values, i, &tmp);
        if (tmp->value == lev->value)
            semantic_error(t, "Enum values %s and %s have the same value %d!", tmp->valuename, lev->valuename, lev->value);
        if (!strcmp(tmp->valuename, lev->valuename))
            semantic_error(t, "Enum value %s declared twice!", tmp->valuename);
    }

    zarray_add(le->values, &lev);
    return 0;
}

/** assume the "struct" token is already consumed **/
lcm_struct_t *parse_struct(lcmgen_t *lcmgen, const char *lcmfile, tokenize_t *t)
{
    char     *name;

    parse_try_consume_comment(lcmgen, t, 0);
    tokenize_next_or_fail(t, "struct name");
    name = strdup(t->token);

    lcm_struct_t *lr = lcm_struct_create(lcmgen, lcmfile, name);

    if (lcmgen->comment_doc) {
        lr->comment = lcmgen->comment_doc;
        lcmgen->comment_doc = NULL;
    }

    parse_require(t, "{");

    while (!parse_try_consume(t, "}")) {
        // Check for leading comments that will be used to document the member.
        parse_try_consume_comment(lcmgen, t, 1);

        if (parse_try_consume(t, "}")) {
            break;
        }
        parse_member(lcmgen, lr, t);
    }

    lr->hash = lcm_struct_hash(lr);

    free(name);
    return lr;
}

/** assumes the "enum" token is already consumed **/
lcm_enum_t *parse_enum(lcmgen_t *lcmgen, const char *lcmfile, tokenize_t *t)
{
    char     *name;

    tokenize_next_or_fail(t, "enum name");
    name = strdup(t->token);

    lcm_enum_t *le = lcm_enum_create(lcmgen, lcmfile, name);
    parse_require(t, "{");

    while (!parse_try_consume(t, "}")) {
        parse_enum_value(le, t);

        parse_try_consume(t, ",");
        parse_try_consume(t, ";");
    }

    le->hash = lcm_enum_hash(le);
    free(name);
    return le;
}

const lcm_struct_t* find_struct(lcmgen_t* lcmgen, const char* package,
    const char* name) {
    for (int i=0; i < zarray_size(lcmgen->structs); i++) {
        lcm_struct_t* lr;
        zarray_get(lcmgen->structs, i, &lr);
        if (!strcmp(package, lr->structname->package) &&
            !strcmp(name, lr->structname->shortname))
            return lr;
    }
    return NULL;
}

/** parse entity (top-level construct), return EOF if eof. **/
int parse_entity(lcmgen_t *lcmgen, const char *lcmfile, tokenize_t *t)
{
    int res;

    parse_try_consume_comment(lcmgen, t, 1);

    res = tokenize_next(t);
    if (res==EOF)
        return EOF;

    if (!strcmp(t->token, "package")) {
        parse_try_consume_comment(lcmgen, t, 0);
        tokenize_next_or_fail(t, "package name");
        lcmgen->package = strdup(t->token);
        parse_require(t, ";");
        return 0;
    }

    if (!strcmp(t->token, "struct")) {
        lcm_struct_t *lr = parse_struct(lcmgen, lcmfile, t);

        // check for duplicate types
        const lcm_struct_t* prior = find_struct(lcmgen,
                lr->structname->package, lr->structname->shortname);
        if (prior) {
            printf("WARNING: duplicate type %s declared in %s\n",
                    lr->structname->lctypename, lcmfile);
            printf("         %s was previously declared in %s\n",
                    lr->structname->lctypename, prior->lcmfile);
            // TODO destroy lr.
            return 1;
        } else {
            zarray_add(lcmgen->structs, &lr);
        }
        return 0;
    }

    if (!strcmp(t->token, "enum")) {
        lcm_enum_t *le = parse_enum(lcmgen, lcmfile, t);
        zarray_add(lcmgen->enums, &le);
        return 0;
    }

    semantic_error(t,"Missing struct/enum/package token.");
    return 1;
}

int lcmgen_handle_file(lcmgen_t *lcmgen, const char *path)
{
    tokenize_t *t = tokenize_create(path);

    if (t==NULL) {
        perror(path);
        return 0;
    }

    if (getopt_get_bool(lcmgen->gopt, "tokenize")) {
        int ntok = 0;
        printf("%6s %6s %6s: %s\n", "tok#", "line", "col", "token");

        while (tokenize_next(t)!=EOF)
            printf("%6i %6i %6i: %s\n", ntok++, t->token_line, t->token_column, t->token);
        return 0;
    }

    int res;
    do {
        res = parse_entity(lcmgen, path, t);
    } while (res >= 0 && res != EOF);

    tokenize_destroy(t);
    if (res >= 0 || res == EOF)
        return 0;
    else
        return res;
}

void lcm_typename_dump(lcm_typename_t *lt)
{
    char buf[1024];
    int pos = 0;

    pos += sprintf(&buf[pos], "%s", lt->lctypename);

    printf("\t%-20s", buf);
}

void lcm_member_dump(lcm_member_t *lm)
{
    lcm_typename_dump(lm->type);

    printf("  ");

    printf("%s", lm->membername);

    int ndim = zarray_size(lm->dimensions);
    for (int i = 0; i < ndim; i++) {
        lcm_dimension_t *dim = NULL;
        zarray_get(lm->dimensions, i, &dim);
        switch (dim->mode)
        {
        case LCM_CONST:
            printf(" [ (const) %s ]", dim->size);
            break;
        case LCM_VAR:
            printf(" [ (var) %s ]", dim->size);
            break;
        default:
            // oops! unhandled case
            assert(0);
        }
    }

    printf("\n");
}

void lcm_enum_dump(lcm_enum_t *le)
{
    printf("enum %s\n", le->enumname->lctypename);
    for (unsigned int i = 0; i < zarray_size(le->values); i++) {
        lcm_enum_value_t *lev = NULL;
        zarray_get(le->values, i, &lev);
        printf("        %-20s  %i\n", lev->valuename, lev->value);
    }
}

void lcm_struct_dump(lcm_struct_t *lr)
{
    printf("struct %s [hash=0x%16"PRId64"]\n", lr->structname->lctypename, lr->hash);
    for (unsigned int i = 0; i < zarray_size(lr->members); i++) {
        lcm_member_t *lm = NULL;
        zarray_get(lr->members, i, &lm);
        lcm_member_dump(lm);
    }

    for (unsigned int i = 0; i < zarray_size(lr->enums); i++) {
        lcm_enum_t *le = NULL;
        zarray_get(lr->enums, i, &le);
        lcm_enum_dump(le);
    }
}

void lcmgen_dump(lcmgen_t *lcmgen)
{
    for (unsigned int i = 0; i < zarray_size(lcmgen->enums); i++) {
        lcm_enum_t *le = NULL;
        zarray_get(lcmgen->enums, i, &le);
        lcm_enum_dump(le);
    }

    for (unsigned int i = 0; i < zarray_size(lcmgen->structs); i++) {
        lcm_struct_t *lr = NULL;
        zarray_get(lcmgen->structs, i, &lr);
        lcm_struct_dump(lr);
    }
}

/** Find and return the member whose name is name. **/
lcm_member_t *lcm_find_member(const lcm_struct_t *lr, const char *name)
{
    for (unsigned int i = 0; i < zarray_size(lr->members); i++) {
        lcm_member_t *lm = NULL;
        zarray_get(lr->members, i, &lm);
        if (!strcmp(lm->membername, name))
            return lm;
    }

    return NULL;
}

/** Find and return the const whose name is name. **/
lcm_constant_t *lcm_find_const(lcm_struct_t *lr, const char *name)
{
    for (unsigned int i = 0; i < zarray_size(lr->constants); i++) {
        lcm_constant_t *lc = NULL;
        zarray_get(lr->constants, i, &lc);
        if (!strcmp(lc->membername, name))
            return lc;
    }
    return NULL;
}

int lcm_needs_generation(lcmgen_t *lcmgen, const char *declaringfile, const char *outfile)
{
    struct stat instat, outstat;
    int res;

    if (!getopt_get_bool(lcmgen->gopt, "lazy"))
        return 1;

    res = stat(declaringfile, &instat);
    if (res) {
        printf("Funny error: can't stat the .lcm file");
        perror(declaringfile);
        return 1;
    }

    res = stat(outfile, &outstat);
    if (res)
        return 1;

    return instat.st_mtime > outstat.st_mtime;
}

/** Is the member an array of constant size? If it is not an array, it returns zero. **/
int lcm_is_constant_size_array(lcm_member_t *lm)
{
    int ndim = zarray_size(lm->dimensions);

    if (ndim == 0)
        return 1;

    for (int i = 0; i < ndim; i++) {
        lcm_dimension_t *dim = NULL;
        zarray_get(lm->dimensions, i, &dim);

        if (dim->mode == LCM_VAR)
            return 0;
    }

    return 1;
}

typedef struct hash_par hash_par_t;
struct hash_par
{
    hash_par_t * parent;
    const lcm_struct_t * st;
};

int64_t resolve_hash_recurse(lcmgen_t * gen, const lcm_struct_t * st, hash_par_t *p)
{
    const hash_par_t *fp;
    for (fp = p; fp != NULL; fp = fp->parent)
        if (fp->st == st)
            return 0;

    hash_par_t cp;
    cp.parent = p;
    cp.st = st;

    int64_t hash = st->hash;

    for(unsigned int m = 0; m < zarray_size(st->members); m++)
    {
        lcm_member_t *lm;
        zarray_get(st->members, m, &lm);

        if (lcm_is_primitive_type(lm->type->shortname))
            continue;

        const lcm_struct_t * this_st = find_struct(gen, lm->type->package, lm->type->shortname);
        if(this_st == NULL)
        {
            printf("WRN: can't find %s package %s this likely won't work!\n",
                   lm->type->shortname,
                   lm->type->package);
        }
        else
        {
            hash += resolve_hash_recurse(gen, this_st, &cp);
        }
    }

    hash = (hash<<1) + ((hash>>63)&1);
    printf("%s\t", st->structname->shortname);
    return hash;
}

void resolve_hashes(lcmgen_t * gen)
{
    printf("loaded:\n");
    int ntypes = zarray_size(gen->structs);
    for(int i = 0; i < ntypes; i++)
    {
        lcm_struct_t * this_st = NULL;
        zarray_get(gen->structs, i, &this_st);
        this_st->rhash = resolve_hash_recurse(gen, this_st, NULL);
    }
    printf("\n");
}

int64_t array_size_read(uint8_t *buf, int64_t offset, int max, char * name)
{
    if(!strcmp(name,"int8_t"))
    {
        int8_t out = 0;
        __int8_t_decode_array(buf, offset, max, &out, 1);
        return out;
    }
    if(!strcmp(name,"int16_t"))
    {
        int16_t out = 0;
        __int16_t_decode_array(buf, offset, max, &out, 1);
        return out;
    }
    if(!strcmp(name,"int32_t"))
    {
        int32_t out = 0;
        __int32_t_decode_array(buf, offset, max, &out, 1);
        return out;
    }
    if(!strcmp(name,"int64_t"))
    {
        int64_t out = 0;
        __int64_t_decode_array(buf, offset, max, &out, 1);
        return out;
    }
    return 0;
}

int64_t print_primitive_type(uint8_t * buf, int offset, int maxlen, char * name)
{
    if(!strcmp(name,"int8_t"))
    {
        int8_t out = 0;
        offset += __int8_t_decode_array(buf, offset, maxlen, &out, 1);
        printf("% "PRId8 " ", out);
    }
    if(!strcmp(name,"int16_t"))
    {
        int16_t out = 0;
        offset += __int16_t_decode_array(buf, offset, maxlen, &out, 1);
        printf("% "PRId16" ", out);
    }
    if(!strcmp(name,"int32_t"))
    {
        int32_t out = 0;
        offset += __int32_t_decode_array(buf, offset, maxlen, &out, 1);
        printf("% "PRId32" ", out);
    }
    if(!strcmp(name,"int64_t"))
    {
        int64_t out = 0;
        offset += __int64_t_decode_array(buf, offset, maxlen, &out, 1);
        printf("% "PRId64" ", out);
    }
    if(!strcmp(name,"byte"))
    {
        uint8_t out = 0;
        offset += __byte_decode_array(buf, offset, maxlen, &out, 1);
        printf("%02x ", out);
    }
    if(!strcmp(name,"float"))
    {
        float out = 0;
        offset += __float_decode_array(buf, offset, maxlen, &out, 1);
        printf("% f ", out);
    }
    if(!strcmp(name,"double"))
    {
        double out = 0;
        offset += __double_decode_array(buf, offset, maxlen, &out, 1);
        printf("% lf ", out);
    }
    if(!strcmp(name,"string"))
    {
        char* out = NULL;
        offset += __string_decode_array(buf, offset, maxlen, &out, 1);
        printf("%s ", out);
        free(out);
    }
    if(!strcmp(name,"boolean"))
    {
        int8_t out = 0;
        offset += __int8_t_decode_array(buf, offset, maxlen, &out, 1);
        printf("%s ", (out?"TRUE":"FALSE"));
    }
    return offset;
}

void indent_spaces(int indent)
{
    printf("%*s",indent*16," ");
}

int64_t decode_member_recurse(lcmgen_t *gen, const lcm_struct_t *parent, const lcm_member_t *lm, uint8_t *buf, int maxlen, int64_t offset, int depth, zhash_t * keys, int indent)
{
    //printf("%" PRId64 " \n", offset);
    if(depth == 0)
    {
        indent_spaces(indent);
        printf("%16s %16s ", lm->type->shortname, lm->membername);
    }

    if(zarray_size(lm->dimensions) - depth == 0)
    {
        if(lcm_is_array_dimension_type(lm->type->shortname))
        {
            zhash_put(keys, &lm->membername, &offset, NULL, NULL);
        }

        if(lcm_is_primitive_type(lm->type->shortname))
            offset = print_primitive_type(buf, offset, maxlen, lm->type->shortname);
        else
        {
            printf("\n");
            const lcm_struct_t * newparent = find_struct(gen, lm->type->package, lm->type->shortname);
            offset = decode_type_recurse(gen, newparent, buf, maxlen, offset, indent+1);
        }
    }
    else
    {
        int64_t elements;
        lcm_dimension_t *dm;
        zarray_get(lm->dimensions,depth,&dm);
        if(dm->mode == LCM_CONST)
        {
            elements = atoi(dm->size);
        }
        else
        {
            const lcm_member_t * ilm = lcm_find_member(parent,dm->size);
            int64_t this_off;
            int found = zhash_get(keys, &dm->size, &this_off);
            if(!ilm)
            {
                printf("no member dm->size '%s'\n", dm->size);
                elements = 0;
            }
            else if(!found)
            {
                printf("nohash dm->size %s\n", dm->size);
                assert(0);
                elements = 0;
            }
            else
            {
                elements = array_size_read(buf, this_off, maxlen, ilm->type->shortname);
            }
        }

        for(int i = 0; i < elements; i++)
        {
            offset = decode_member_recurse(gen, parent, lm, buf, maxlen, offset, depth+1, keys, indent);
            if(zarray_size(lm->dimensions) - depth > 1 && i < elements-1)
            {
                printf("\n                                  ");
                indent_spaces(indent);
            }
        }

    }
    return offset;
}

int64_t decode_type_recurse(lcmgen_t *gen, const lcm_struct_t *st, uint8_t *buf, int maxlen, int64_t offset, int indent)
{
    zhash_t *newkeys = zhash_create(sizeof(char*), sizeof(int64_t),
                                zhash_str_hash, zhash_str_equals);
    if(st->comment)
    {
        indent_spaces(indent);
        printf("//%s\n", st->comment);
    }
    for(unsigned int m = 0; m < zarray_size(st->members); m++)
    {
        lcm_member_t *lm;
        zarray_get(st->members, m, &lm);
        offset = decode_member_recurse(gen, st, lm, buf, maxlen, offset, 0, newkeys, indent);
        printf("\n");
        if(lm->comment)
        {
            indent_spaces(indent);
            printf("//%s\n", lm->comment);
        }
    }
    for(unsigned int c = 0; c < zarray_size(st->constants); c++)
    {
        lcm_constant_t *lc;
        zarray_get(st->constants, c, &lc);
        indent_spaces(indent);
        printf("%16s %16s %s", lc->lctypename, lc->membername, lc->val_str);
        printf("\n");
        if(lc->comment)
        {
            indent_spaces(indent);
            printf("//%s\n", lc->comment);
        }
    }
    zhash_destroy(newkeys);
    return offset;
}

static int primitive_to_json(uint8_t *buf, int offset, int maxlen,
                             char *name, json_object_t **json)
{
    if(!strcmp(name,"int8_t"))
    {
        int8_t out = 0;
        offset += __int8_t_decode_array(buf, offset, maxlen, &out, 1);
        *json = json_number_create(out);
    }
    if(!strcmp(name,"int16_t"))
    {
        int16_t out = 0;
        offset += __int16_t_decode_array(buf, offset, maxlen, &out, 1);
        *json = json_number_create(out);
    }
    if(!strcmp(name,"int32_t"))
    {
        int32_t out = 0;
        offset += __int32_t_decode_array(buf, offset, maxlen, &out, 1);
        *json = json_number_create(out);
    }
    if(!strcmp(name,"int64_t"))
    {
        int64_t out = 0;
        offset += __int64_t_decode_array(buf, offset, maxlen, &out, 1);
        char buf[100];
        snprintf(buf, sizeof(buf), "%" PRId64, out);
        *json = json_string_create_copy(buf);
    }
    if(!strcmp(name,"byte"))
    {
        uint8_t out = 0;
        offset += __byte_decode_array(buf, offset, maxlen, &out, 1);
        *json = json_number_create(out);
    }
    if(!strcmp(name,"float"))
    {
        float out = 0;
        offset += __float_decode_array(buf, offset, maxlen, &out, 1);
        *json = json_number_create(out);
    }
    if(!strcmp(name,"double"))
    {
        double out = 0;
        offset += __double_decode_array(buf, offset, maxlen, &out, 1);
        *json = json_number_create(out);
    }
    if(!strcmp(name,"string"))
    {
        char* out = NULL;
        offset += __string_decode_array(buf, offset, maxlen, &out, 1);
        *json = json_string_create_wrap(out);
    }
    if(!strcmp(name,"boolean"))
    {
        int8_t out = 0;
        offset += __int8_t_decode_array(buf, offset, maxlen, &out, 1);
        *json = json_boolean_create(out);
    }
    return offset;
}

static int decode_member_to_json(lcmgen_t *gen, const lcm_struct_t *st,
                                 const lcm_member_t *lm, uint8_t *buf,
                                 int maxlen, int offset, zhash_t * keys,
                                 json_object_t *parent, int depth, int nelements)
{
    if (depth == zarray_size(lm->dimensions)) {
        if(lcm_is_array_dimension_type(lm->type->shortname))
            zhash_put(keys, &lm->membername, &offset, NULL, NULL);

        // Non-array primitive or composite
        // If primitive: add "name" -> {"t":"type", "v":value}
        // If composite: recursively lcm_decode_to_json starting at offset
        //               add "name" -> {"t":"type", "v":value "o":[...]}
        if (depth == 0) {
            json_object_t *member = json_hash_create();
            if (lcm_is_primitive_type(lm->type->shortname)) {
                json_hash_add(member, "t",
                              json_string_create_copy(lm->type->shortname));
                json_object_t *value = NULL;
                offset = primitive_to_json(buf, offset, maxlen,
                                           lm->type->shortname, &value);
                json_hash_add(member, "v", value);
            } else {
                const lcm_struct_t * st = find_struct(gen, lm->type->package,
                                                      lm->type->shortname);
                offset = lcmgen_decode_to_json(gen, st, buf, maxlen, offset, member);
            }
            json_hash_add(parent, lm->membername, member);
        }
        // Within an array, omit the type field and just output value
        else {
            if (lcm_is_primitive_type(lm->type->shortname)) {
                json_object_t *value = NULL;
                offset = primitive_to_json(buf, offset, maxlen,
                                           lm->type->shortname, &value);
                if(nelements < 30)
                    json_array_add(parent, value);
                else
                    json_object_destroy(value);

            } else {
                const lcm_struct_t * st = find_struct(gen, lm->type->package,
                                                      lm->type->shortname);
                if(nelements < 30)
                    offset = lcmgen_decode_to_json(gen, st, buf, maxlen, offset, parent);
                else {
                    json_object_t * fake = json_array_create();
                    offset = lcmgen_decode_to_json(gen, st, buf, maxlen, offset, fake);
                    json_object_destroy(fake);
                }
            }
        }
    } else {
        // Resolve all dimensions in this array
        int *dimensions = calloc(zarray_size(lm->dimensions), sizeof(int));
        int total_elements = 1;
        for (int i = 0; i < zarray_size(lm->dimensions); i += 1) {
            lcm_dimension_t *dm;
            zarray_get(lm->dimensions, i, &dm);

            int elements = 0;
            if (dm->mode == LCM_CONST) {
                elements = atoi(dm->size);
            } else {
                const lcm_member_t * ilm = lcm_find_member(st, dm->size);
                int this_off;
                int found = zhash_get(keys, &dm->size, &this_off);
                if (!ilm) {
                    printf("no member dm->size '%s'\n", dm->size);
                    assert(0);
                } else if (!found) {
                    printf("nohash dm->size %s\n", dm->size);
                    assert(0);
                } else {
                    elements = array_size_read(buf, this_off, maxlen,
                            ilm->type->shortname);
                }
            }
            dimensions[i] = elements;
            total_elements *= elements;
        }

        // If array: add "name" -> {"t":"type", "v":[...], "d":[...]}
        json_object_t *array = json_array_create();
        if (depth == 0) {
            json_object_t *member = json_hash_create();
            json_hash_add(member, "t", json_string_create_copy(lm->type->shortname));
            // Dimensions
            json_object_t *dim = json_array_create();
            for (int i = 0; i < zarray_size(lm->dimensions); i += 1)
                json_array_add(dim, json_number_create(dimensions[i]));
            json_hash_add(member, "d", dim);

            json_hash_add(member, "v", array);
            if(!lcm_is_primitive_type(lm->type->shortname)) {

                const lcm_struct_t * st = find_struct(gen, lm->type->package,
                                                           lm->type->shortname);

                json_object_t *order = json_array_create();
                for (int i = 0; i < zarray_size(st->members); i += 1) {
                    lcm_member_t *lm;
                    zarray_get(st->members, i, &lm);
                    json_array_add(order, json_string_create_copy(lm->membername));
                }

                json_hash_add(member, "o", order);
            }

            json_hash_add(parent, lm->membername, member);
        } else {
            json_array_add(parent, array);
        }

        for (int i = 0; i < dimensions[depth]; i += 1) {
            offset = decode_member_to_json(gen, st, lm, buf, maxlen, offset,
                                           keys, array, depth+1, i*nelements);
        }
        free(dimensions);
    }

    return offset;
}

int lcmgen_decode_to_json(lcmgen_t *gen, const lcm_struct_t *st,
                          uint8_t *buf, int maxlen, int offset,
                          json_object_t *parent)
{
    if(st == NULL) return offset;
    // Maps membername -> offset for variable-length arrays
    zhash_t *keys = zhash_create(sizeof(char*), sizeof(int),
                                    zhash_str_hash, zhash_str_equals);
    // Add a json hash of membername -> value to parent
    json_object_t *value = json_hash_create();
    json_object_t *order = json_array_create();
    for (int i = 0; i < zarray_size(st->members); i += 1) {
        lcm_member_t *lm;
        zarray_get(st->members, i, &lm);

        offset = decode_member_to_json(gen, st, lm, buf, maxlen, offset,
                                       keys, value, 0, 1);
        json_array_add(order, json_string_create_copy(lm->membername));
    }
    for (int i = 0; i < zarray_size(st->constants); i += 1) {
        lcm_constant_t *lc;
        zarray_get(st->constants, i, &lc);

        json_object_t *member = json_hash_create();
        {
            json_hash_add(member, "t",
                          json_string_create_copy(lc->lctypename));
            json_object_t *value = json_string_create_copy(lc->val_str);
            json_hash_add(member, "v", value);
        }
        json_hash_add(value, lc->membername, member);

        json_array_add(order, json_string_create_copy(lc->membername));
    }

    if (parent->type == JSON_TYPE_HASH) {
        json_hash_add(parent, "t",
                      json_string_create_copy(st->structname->shortname));
        json_hash_add(parent, "v", value);
        json_hash_add(parent, "o", order);
    } else if (parent->type == JSON_TYPE_ARRAY) {
        json_array_add(parent, value);
        json_object_destroy(order);
    }

    zhash_destroy(keys);

    return offset;
}
//*/
