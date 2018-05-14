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
#include <math.h>

#include "common/io_util.h"
#include "json.h"

static void do_debug(const char *s, int *spos, int slen)
{
    int v = *spos;
    for (int i = 0; i < v; i++)
        printf("%c", s[i]);
    printf("[==>]");
    for (int i = v; i < slen; i++)
        printf("%c", s[i]);
}

#define DEBUG(s, spos, slen, ...) if (1) { fprintf(stdout, "%s:%d ", __FILE__, __LINE__); fprintf(stdout, __VA_ARGS__); fprintf(stdout, "\n"); do_debug(s, spos, slen); fprintf(stdout, "\n"); }

json_object_t *json_null_create()
{
    json_object_t *jobj = calloc(1, sizeof(json_object_t));
    jobj->type = JSON_TYPE_NULL;
    return jobj;
}

json_object_t *json_number_create(double v)
{
    json_object_t *jobj = calloc(1, sizeof(json_object_t));
    jobj->type = JSON_TYPE_NUMBER;
    jobj->u.number.v = v;
    return jobj;
}

json_object_t *json_boolean_create(bool v)
{
    json_object_t *jobj = calloc(1, sizeof(json_object_t));
    jobj->type = JSON_TYPE_BOOLEAN;
    jobj->u.boolean.v = v;
    return jobj;
}

json_object_t *json_string_create_copy(const char *s)
{
    return json_string_create_wrap(strdup(s));
}

json_object_t *json_string_create_wrap(char *s)
{
    json_object_t *jobj = calloc(1, sizeof(json_object_t));
    jobj->type = JSON_TYPE_STRING;
    jobj->u.string.v = s;
    return jobj;
}

json_object_t *json_array_create()
{
    json_object_t *jobj = calloc(1, sizeof(json_object_t));
    jobj->type = JSON_TYPE_ARRAY;
    jobj->u.array.v = zarray_create(sizeof(json_object_t*));
    return jobj;
}

void json_array_add(json_object_t *jobj, json_object_t *v)
{
    assert(jobj->type == JSON_TYPE_ARRAY);

    zarray_add(jobj->u.array.v, &v);
}

int json_array_size(json_object_t *jobj)
{
    assert(jobj->type == JSON_TYPE_ARRAY);

    return zarray_size(jobj->u.array.v);
}

json_object_t *json_array_get(json_object_t *jobj, int idx)
{
    assert(jobj->type == JSON_TYPE_ARRAY);
    json_object_t *obj;

    zarray_get(jobj->u.array.v, idx, &obj);
    return obj;
}

json_object_t *json_hash_create()
{
    json_object_t *jobj = calloc(1, sizeof(json_object_t));
    jobj->type = JSON_TYPE_HASH;
    jobj->u.hash.kv = zhash_create(sizeof(char*), sizeof(json_object_t*),
                                   zhash_str_hash,
                                   zhash_str_equals);

    return jobj;
}

// the value becomes "owned" by the hash. The _key is copied, however!
void json_hash_add(json_object_t *jobj, const char *_key, json_object_t *value)
{
    assert(jobj->type == JSON_TYPE_HASH);

    char *key = strdup(_key);

    char *oldkey;
    json_object_t *oldvalue;

    if (zhash_put(jobj->u.hash.kv, &key, &value, &oldkey, &oldvalue)) {
        free(oldkey);
        json_object_destroy(oldvalue);
    }
}

json_object_t *json_hash_get(json_object_t *jobj, const char *_key)
{
    assert(jobj->type == JSON_TYPE_HASH);

    json_object_t *res = NULL;

    zhash_get(jobj->u.hash.kv, &_key, &res);

    return res;
}

// if the value of the map is not found or is not a string, NULL is
// returned.  The string is the internal copy belonging to the json
// object.
const char *json_hash_get_string(json_object_t *jobj, const char *key)
{
    json_object_t *jval = json_hash_get(jobj, key);

    if (jval == NULL || jval->type != JSON_TYPE_STRING)
        return NULL;

    return jval->u.string.v;
}

void json_object_destroy(json_object_t *jobj)
{
    if (!jobj)
        return;

    switch (jobj->type) {
        case JSON_TYPE_NULL:
            // nop
            break;
        case JSON_TYPE_STRING:
            free(jobj->u.string.v);
            break;
        case JSON_TYPE_NUMBER:
        case JSON_TYPE_BOOLEAN:
            // nop
            break;
        case JSON_TYPE_HASH: {
            zhash_iterator_t zit;
            zhash_iterator_init(jobj->u.hash.kv, &zit);
            char *key;
            json_object_t *value;
            while (zhash_iterator_next(&zit, &key, &value)) {
                free(key);
                json_object_destroy(value);
            }
            zhash_destroy(jobj->u.hash.kv);
            break;
        }
        case JSON_TYPE_ARRAY: {
            for (int i = 0; i < zarray_size(jobj->u.array.v); i++) {
                json_object_t *value;
                zarray_get(jobj->u.array.v, i, &value);
                json_object_destroy(value);
            }
            zarray_destroy(jobj->u.array.v);
            break;
        }
        default:
            assert(0);
    }

    free(jobj);
}

char *json_parse_string(const char *s, int *spos, int slen)
{
    // skip ahead to opening quote
    while (s[*spos] != '\"' && *spos < slen)
        (*spos)++;

    // no quote found?
    if (s[*spos] != '\"')
        return strdup(""); // XXX really an error

    // skip opening quote
    (*spos)++;

    int bufalloc = 16;
    int buflen = 0;
    char *buf = malloc(bufalloc);

    while (*spos < slen) {

        // grow output buffer
        if (buflen + 2 >= bufalloc) {
            bufalloc *= 2;
            buf = realloc(buf, bufalloc);
        }

        // end of string
        if (s[*spos] == '\"') {
            (*spos)++; // consume
            break;
        }

        // consume character
        if (s[*spos] == '\\') {
            // escape
            (*spos)++; // consume backslash
            char c = s[*spos];
            switch (c) {
                case 'n':
                    buf[buflen++] = '\n';
                    break;
                case 'r':
                    buf[buflen++] = '\r';
                    break;
            case 't':
                buf[buflen++] = '\t';
                break;

            default:
                    buf[buflen++] = c;
                    break;
            }
            (*spos)++;

        } else {

            buf[buflen++] = s[*spos];
            (*spos)++;
        }
    }

    // zero terminate
    buf[buflen] = 0;
    return buf;
}

static void skip_whitespace(const char *s, int *spos, int slen)
{
    while (*spos < slen && (s[*spos] == ' ' || s[*spos] == '\n' || s[*spos] == '\r' || s[*spos] == '\t'))
        (*spos)++;
}

json_object_t *json_parse_r(const char *s, int *spos, int slen)
{
    skip_whitespace(s, spos, slen);
    if (*spos >= slen) {
        DEBUG(s, spos, slen, "unexpected EOF");
        return NULL;
    }

    char c = s[*spos];

    if (c == '-' || (c >= '0' && c <= '9')) {
        // number
        char buf[slen + 1];
        int buflen = 0;

        while (*spos < slen) {
            c = s[*spos];
            // XXX doesn't match the structure of the regex.
            if (c == '.' || c == '+' || c == '-' || c == 'e' || c == 'E' || (c >= '0' && c <= '9')) {
                buf[buflen++] = c;
                (*spos)++;
                continue;
            }
            break;
        }

        buf[buflen] = 0;

        return json_number_create(atof(buf));
    }

    // null?
    if (c == 'n' && (*spos + 3) < slen && !strncmp("null", &s[*spos], 4)) {
        (*spos) +=4;
        return json_null_create();
    }

    if (!strncmp("true", &s[*spos], 4)) {
        (*spos) += 4;
        return json_boolean_create(1);
    }

    if (!strncmp("false", &s[*spos], 5)) {
        (*spos) += 5;
        return json_boolean_create(0);
    }

    if (c == '[') {
        // array
        (*spos)++; // consume brace
        json_object_t *jarray = json_array_create();

        while (*spos < slen) {
            skip_whitespace(s, spos, slen);

            if (*spos >= slen) {
                DEBUG(s, spos, slen, "unexpected EOF");
                break;
            }

            if (s[*spos] == ']') {
                (*spos)++; // consume the closing brace
                break;
            }

            json_object_t *jobj = json_parse_r(s, spos, slen);
//            if (!jobj)
//                break;

            zarray_add(jarray->u.array.v, &jobj);

            skip_whitespace(s, spos, slen);

            if (*spos >= slen)  {
                DEBUG(s, spos, slen, "unexpected EOF");
                break;
            }

            if (s[*spos] == ',') {
                (*spos)++; // consume the comma
                continue;
            } else if (s[*spos] == ']') {
                (*spos)++;
                break;
            } else {
                DEBUG(s, spos, slen, "unexpected character %c\n", s[*spos]);
                break;
            }
        }

        return jarray;
    }

    if (c == '{') {
        // object
        (*spos)++; // consume brace
        json_object_t *jhash = json_hash_create();

        while (*spos < slen) {
            skip_whitespace(s, spos, slen);

            if (*spos >= slen) {
                DEBUG(s, spos, slen, "unexpected EOF");
                break;
            }

            if (s[*spos] == '}') {
                (*spos)++;
                break;
            }

            char *key = json_parse_string(s, spos, slen);
            if (!key)
                break;

            skip_whitespace(s, spos, slen);

            if (*spos >= slen) {
                DEBUG(s, spos, slen, "unexpected EOF");
                break;
            }

            // consume colon
            if (s[*spos] != ':') {
                DEBUG(s, spos, slen, "expected colon, got %c\n", s[*spos]);
                break;
            }
            (*spos)++;

            json_object_t *value = json_parse_r(s, spos, slen);
            char *oldkey;
            json_object_t *oldvalue;
            if (zhash_put(jhash->u.hash.kv, &key, &value, &oldkey, &oldvalue)) {
                free(oldkey);
                json_object_destroy(oldvalue);
            }

            if (*spos >= slen)  {
                DEBUG(s, spos, slen, "unexpected EOF");
                break;
            }

            skip_whitespace(s, spos, slen);

            // consume comma
            if (s[*spos] == ',') {
                (*spos)++; // consume the comma
                continue;
            } else if (s[*spos] == '}') {
                (*spos)++;
                break;
            } else {
                DEBUG(s, spos, slen, "unexpected character %c", s[*spos]);
                break;
            }
        }

        return jhash;
    }

    // string
    if (c == '\"') {
        return json_string_create_wrap(json_parse_string(s, spos, slen));
    }

    // a proprietary extension; allow strings to be created delimited by
    // a symbol like $eof$. E.g. "foo" : $eof$Crazy!String!!$eof$
    if (c == '$') {
        int symoff = *spos;
        int symlen = 1; // include the first $

        while (symoff + symlen < slen && s[symoff + symlen] != '$')
            symlen++;

        // if we didn't run off the end of the buffer...
        if (s[symoff + symlen] == '$') {
            int literaloff = symoff + symlen + 1;

            // now search for the delimiter again at the end of the string.
            for (int literallen = 0; literaloff + literallen < slen; literallen++) {
                int ok = 1;
                for (int i = 0; i < symlen; i++) {
                    if (s[literaloff + literallen + i] != s[symoff + i]) {
                        ok = 0;
                        break;
                    }
                }

                if (ok) {
                    // we found it!
                    char *m = malloc(literallen + 1);
                    memcpy(m, &s[literaloff], literallen);
                    m[literallen] = 0;
                    *spos = literaloff + literallen + symlen + 1;
                    return json_string_create_wrap(m);
                }
            }
        }

        // otherwise something bad happened
        DEBUG(s, spos, slen, "bad escaped string; unterminated?");
        return NULL;
    }

    DEBUG(s, spos, slen, "bad character: %c, from string %s\n", c, s);
    return NULL;
}

json_object_t *json_object_parse(const char *s)
{
    int spos = 0;
    int slen = strlen(s);
    json_object_t *obj = json_parse_r(s, &spos, slen);
    return obj;
}

static void print_indent(int indent)
{
    for (int i = 0; i < 3*indent; i++)
        printf(" ");
}

void json_print_r(json_object_t *jobj, int indent)
{
    if (jobj == NULL) {
        printf("null\n");
        return;
    }

    switch (jobj->type) {
        case JSON_TYPE_NULL: {
            print_indent(indent);
            printf("null\n");
            break;
        }
        case JSON_TYPE_STRING:
            print_indent(indent);
            printf("\"%s\"\n", jobj->u.string.v);
            break;
        case JSON_TYPE_NUMBER:
            print_indent(indent);
            printf("%f\n", jobj->u.number.v);
            break;
        case JSON_TYPE_BOOLEAN:
            print_indent(indent);
            printf("%s\n", jobj->u.boolean.v ? "true" : "false");
            break;
        case JSON_TYPE_ARRAY: {
            print_indent(indent);
            printf("[\n");
            for (int i = 0; i < zarray_size(jobj->u.array.v); i++) {
                json_object_t *value;
                zarray_get(jobj->u.array.v, i, &value);
                json_print_r(value, indent+1);
            }
            print_indent(indent);
            printf("]\n");
            break;
        }
        case JSON_TYPE_HASH: {
            print_indent(indent);
            printf("{\n");
            zhash_iterator_t zit;
            zhash_iterator_init(jobj->u.hash.kv, &zit);
            char *key;
            json_object_t *value;
            while (zhash_iterator_next(&zit, &key, &value)) {
                print_indent(indent);
                printf("\"%s\" :\n", key);
                json_print_r(value, indent + 2);
            }
            print_indent(indent);
            printf("}\n");
            break;
        }
        default:
            assert(0);
    }
}

void json_object_print(json_object_t *jobj)
{
    json_print_r(jobj, 0);
}

static void _concat_char(char **buf, int *bufpos, int *bufalloc, char c)
{
    while (*bufpos + 1 >= *bufalloc) {
        (*bufalloc) *= 2;
        *buf = realloc(*buf, *bufalloc);
    }
    (*buf)[*bufpos] = c;
    (*bufpos)++;
    (*buf)[*bufpos] = 0;
}

static void _concat_string(char **buf, int *bufpos, int *bufalloc, const char *s)
{
    while (*s != 0) {
        _concat_char(buf, bufpos, bufalloc, *s);
        s++;
    }
}

void _concat_string_quoted(char **buf, int *bufpos, int *bufalloc, const char *in)
{
    _concat_char(buf, bufpos, bufalloc, '\"');
    for (const char *s = in; *s != 0; s++) {
        switch (*s) {
            case '\n':
                _concat_string(buf, bufpos, bufalloc, "\\n");
                break;
            case '\r':
                _concat_string(buf, bufpos, bufalloc, "\\r");
                break;
            case '\t':
                _concat_string(buf, bufpos, bufalloc, "\\t");
                break;
            case '"':
                _concat_string(buf, bufpos, bufalloc, "\\\"");
                break;
            case '\\':
                _concat_string(buf, bufpos, bufalloc, "\\\\");
                break;
            default:
                _concat_char(buf, bufpos, bufalloc, *s);
                break;
        }
    }
    _concat_char(buf, bufpos, bufalloc, '\"');
}

void json_tostring_r(const json_object_t *jobj, char **buf, int *bufpos, int *bufalloc)
{
    if (jobj == NULL) {
        _concat_string(buf, bufpos, bufalloc, "null");
        return;
    }

    switch (jobj->type) {
        case JSON_TYPE_NULL: {
            _concat_string(buf, bufpos, bufalloc, "null");
            break;
        }

        case JSON_TYPE_STRING: {
            _concat_string_quoted(buf, bufpos, bufalloc, jobj->u.string.v);
            break;
        }

        case JSON_TYPE_NUMBER: {
            if (isnan(jobj->u.number.v)) {
                _concat_string(buf, bufpos, bufalloc, "null");
            } else {
                char s[1024];
                sprintf(s, "%.15g", jobj->u.number.v);
                _concat_string(buf, bufpos, bufalloc, s);
            }
            break;
        }

        case JSON_TYPE_BOOLEAN: {
            _concat_string(buf, bufpos, bufalloc,
                           jobj->u.boolean.v ? "true" : "false");
            break;
        }

        case JSON_TYPE_HASH: {
            _concat_char(buf, bufpos, bufalloc, '{');
            zhash_iterator_t zit;
            zhash_iterator_init(jobj->u.hash.kv, &zit);
            char *key;
            json_object_t *value;
            int i = 0;
            while (zhash_iterator_next(&zit, &key, &value)) {
                if (i > 0)
                    _concat_char(buf, bufpos, bufalloc, ',');
                i++;
                _concat_string_quoted(buf, bufpos, bufalloc, key);
                _concat_char(buf, bufpos, bufalloc, ':');
                json_tostring_r(value, buf, bufpos, bufalloc);
            }
            _concat_char(buf, bufpos, bufalloc, '}');
            break;
        }

        case JSON_TYPE_ARRAY: {
            _concat_char(buf, bufpos, bufalloc, '[');
            for (int i = 0; i < zarray_size(jobj->u.array.v); i++) {
                if (i > 0)
                    _concat_char(buf, bufpos, bufalloc, ',');
                json_object_t *value;
                zarray_get(jobj->u.array.v, i, &value);
                json_tostring_r(value, buf, bufpos, bufalloc);
            }
            _concat_char(buf, bufpos, bufalloc, ']');
            break;
        }

        default:
            assert(0);
    }
}

json_object_t *json_object_copy(json_object_t *jobj)
{
    if (!jobj)
        return NULL;

    switch (jobj->type) {
        case JSON_TYPE_NULL: {
            return json_null_create();
        }

        case JSON_TYPE_STRING: {
            return json_string_create_copy(jobj->u.string.v);
        }

        case JSON_TYPE_NUMBER: {
            return json_number_create(jobj->u.number.v);
        }

        case JSON_TYPE_BOOLEAN: {
            return json_boolean_create(jobj->u.boolean.v);
        }

        case JSON_TYPE_HASH: {
            json_object_t *copy = json_hash_create();

            zhash_iterator_t zit;
            zhash_iterator_init(jobj->u.hash.kv, &zit);
            char *key;
            json_object_t *value;

            while (zhash_iterator_next(&zit, &key, &value)) {
                // don't strdup the key!
                json_hash_add(copy, key, json_object_copy(value));
            }

            return copy;
        }

        case JSON_TYPE_ARRAY: {
            json_object_t *copy = json_array_create();

            for (int i = 0; i < json_array_size(jobj); i++) {
                json_object_t *el = json_array_get(jobj, i);
                json_array_add(copy, json_object_copy(el));
            }

            return copy;
        }

        default:
            assert(0);
    }

    return NULL;
}

char *json_object_tostring(const json_object_t *jobj)
{
    int bufpos = 0;
    int bufalloc = 16;
    char *buf = malloc(bufalloc);
    json_tostring_r(jobj, &buf, &bufpos, &bufalloc);
    return buf;
}
void json_object_tostring_len(const json_object_t *jobj, char ** buf, int * len)
{
    int bufalloc = 16;
    *buf = malloc(bufalloc);
    json_tostring_r(jobj, buf, len, &bufalloc);
}

json_object_t *json_object_create_from_file(const char *path)
{
    char *buf;
    long len;

    if (ioutils_read_file(path, &buf, &len, -1))
        return NULL;

    json_object_t *jobj = json_object_parse(buf);
    free(buf);
    return jobj;
}

int json_object_write_to_file(const json_object_t *jobj, const char *path)
{
    FILE *f = fopen(path, "w");
    if (f == NULL) {
        return -1;
    }

    char *contents = json_object_tostring(jobj);
    int contents_len = strlen(contents);

    if (fwrite(contents, 1, contents_len, f) != contents_len) {
        free(contents);
        fclose(f);
        return -2;
    }

    free(contents);
    fclose(f);
    return 0;
}
