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

#ifndef _JSON_H
#define _JSON_H

#include <stdbool.h>

#include "common/zhash.h"
#include "common/zarray.h"

enum { JSON_TYPE_STRING, JSON_TYPE_NUMBER, JSON_TYPE_ARRAY, JSON_TYPE_HASH,
       JSON_TYPE_BOOLEAN, JSON_TYPE_NULL };

typedef struct json_object json_object_t;
struct json_object
{
    int type; // e.g. JSON_TYPE_STRING

    union {
        struct {
        } null;
        struct {
            char *v;
        } string;
        struct {
            double v;
        } number;
        struct {
            zarray_t *v;  // array of json_object_t*
        } array;
        struct {
            zhash_t *kv; // char* to json_object_t*
        } hash;
        struct {
            bool v;
        } boolean;
    } u;
};

static inline int json_is_null(json_object_t *jobj)
{
    return (jobj != NULL && jobj->type == JSON_TYPE_NULL);
}

static inline int json_is_string(json_object_t *jobj)
{
    return (jobj != NULL && jobj->type == JSON_TYPE_STRING);
}

static inline int json_is_number(json_object_t *jobj)
{
    return (jobj != NULL && jobj->type == JSON_TYPE_NUMBER);
}

static inline int json_is_hash(json_object_t *jobj)
{
    return (jobj != NULL && jobj->type == JSON_TYPE_HASH);
}

static inline int json_is_array(json_object_t *jobj)
{
    return (jobj != NULL && jobj->type == JSON_TYPE_ARRAY);
}

static inline int json_is_bool(json_object_t *jobj)
{
    return (jobj != NULL && jobj->type == JSON_TYPE_BOOLEAN);
}

json_object_t *json_object_create_from_file(const char *path);

json_object_t *json_number_create(double v);

json_object_t *json_boolean_create(bool v);

// json object becomes owner of s.
json_object_t *json_string_create_copy(const char *s);
json_object_t *json_string_create_wrap(char *s);

static inline const char *json_string_get(json_object_t *jobj)
{
    assert(json_is_string(jobj));
    return jobj->u.string.v;
}

////////////////////////////////////////////////
// arrays
////////////////////////////////////////////////

json_object_t *json_array_create();
void json_array_add(json_object_t *jobj, json_object_t *v);
json_object_t *json_array_get(json_object_t *jobj, int idx);
int json_array_size(json_object_t *jobj);

////////////////////////////////////////////////
// hashes (map to javascript objects)
////////////////////////////////////////////////

json_object_t *json_hash_create();
json_object_t *json_hash_get(json_object_t *jobj, const char *_key);
void json_hash_add(json_object_t *jobj, const char *_key, json_object_t *_value);

// if the value of the map is not found or is not a string, NULL is
// returned.  The string is the internal copy belonging to the json
// object.
const char *json_hash_get_string(json_object_t *jobj, const char *key);

////////////////////////////////////////////////
// generic utilities (apply to all json types)
////////////////////////////////////////////////

void json_object_destroy(json_object_t *jobj);

// caller owns s.
json_object_t *json_object_parse(const char *s);

void json_object_print(json_object_t *jobj);
char *json_object_tostring(const json_object_t *jobj);
void json_object_tostring_len(const json_object_t *jobj, char ** buf, int * len);
json_object_t *json_object_copy(json_object_t *jobj);

int json_object_write_to_file(const json_object_t *jobj, const char *path);

#endif
