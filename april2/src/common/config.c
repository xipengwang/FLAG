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
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <assert.h>
#include <libgen.h>

#include "sys_util.h"
#include "config.h"
#include "generic_tokenizer.h"
#include "zarray.h"
#include "zhash.h"
#include "string_util.h"

struct config
{
    generic_tokenizer_t *gt;

    zhash_t *keys; // char* -> zarray_t<char*>
    zhash_t *unique_indices; // char* -> uint32_t

    zarray_t *cached_keys;   // all non-abstract keys (char*) (no memory duplication)
    zhash_t *cached_matis;   // cached int matrices (mati_t)
    zhash_t *cached_matds;   // cached double matrices (matd_t)

    zhash_t *cached_zarrays_booleans;   // cached zarrays for booleans
    zhash_t *cached_zarrays_ints;       // cached zarrays for ints
    zhash_t *cached_zarrays_doubles;    // cached zarrays for doubles

    zarray_t *included_files; // Stack of files that are parents of this file

    char *prefix;
};

static generic_tokenizer_t* config_tokenizer_create()
{
    generic_tokenizer_t *gt = generic_tokenizer_create("ERROR");

    generic_tokenizer_add(gt, "STRING", "\"((\\\\.)|[^\"])*\"");

    //    generic_tokenizer_add(gt, "STRING", "\"[^\"]*\"");
    generic_tokenizer_add_escape(gt, "OP", "+{ { } = : , ; [ ]");
    generic_tokenizer_add(gt, "SYMBOL", "[a-zA-Z_\\.0-9\\-\\+#]+");

    generic_tokenizer_add_ignore(gt, "#[^\n]*(\\n|\\$)"); // comment
    generic_tokenizer_add_ignore(gt, "//[^\n]*(\\n|\\$)"); // comment

    generic_tokenizer_add_ignore(gt, "\\s+");        // whitespace
    generic_tokenizer_add_ignore(gt, "\\$");

    return gt;
}

config_t *config_create(void)
{
    config_t *config = calloc(1, sizeof(config_t));

    config->gt = config_tokenizer_create();
    config->keys = zhash_create(sizeof(char*), sizeof(zarray_t*), zhash_str_hash, zhash_str_equals);
    config->unique_indices = zhash_create(sizeof(char*), sizeof(uint32_t), zhash_str_hash, zhash_str_equals);

    config->cached_keys  = NULL;
    config->cached_matis = zhash_create(sizeof(char*), sizeof(mati_t*), zhash_str_hash, zhash_str_equals);
    config->cached_matds = zhash_create(sizeof(char*), sizeof(matd_t*), zhash_str_hash, zhash_str_equals);
    config->cached_zarrays_booleans = zhash_create(sizeof(char*), sizeof(zarray_t*), zhash_str_hash, zhash_str_equals);
    config->cached_zarrays_ints = zhash_create(sizeof(char*), sizeof(zarray_t*), zhash_str_hash, zhash_str_equals);
    config->cached_zarrays_doubles = zhash_create(sizeof(char*), sizeof(zarray_t*), zhash_str_hash, zhash_str_equals);
    config->included_files = zarray_create (sizeof (char*));

    return config;
}

static void syntax_error(const config_t *config, generic_tokenizer_feeder_t *feeder, const char *reason)
{
    const struct gt_token *tok = generic_tokenizer_feeder_last_token(feeder);
    assert(tok != NULL);

    //fatal("Config: Syntax error at line %d, column %d: %s\n", tok->line, tok->column, reason);
    fprintf(stderr, "Config: Syntax error at line %d, column %d: %s\n", tok->line, tok->column, reason);
    exit(EXIT_FAILURE);
}

static zarray_t *copy_values(const zarray_t *in)
{
    assert(in != NULL);

    zarray_t *out = zarray_create(sizeof(void*));
    for (int i = 0; i < zarray_size(in); i++) {

        char *val = NULL, *valcopy = NULL;
        zarray_get(in, i, &val);
        valcopy = strdup(val);
        zarray_add(out, &valcopy);
    }

    return out;
}

static char* decode_string_literal(const char *s)
{
    assert(s != NULL);

    string_buffer_t *sb = string_buffer_create();

    int len = strlen(s);

    assert(s[0] == '\"');
    assert(s[len-1]== '\"');

    // skip over beginning and terminating quotation marks
    for (int idx = 1; idx+1 < len; idx++) {
        char c = s[idx];
        if (c == '\\' && idx+2 < len) {
            char escape = s[idx+1];
            switch (escape) {
            case 'n':
                escape = '\n';
                break;
            case 't':
                escape = '\t';
                break;
            case 'r':
                escape = '\r';
                break;
            }
            string_buffer_append(sb, escape);
            idx++;
            continue;
        }
        string_buffer_append(sb, c);
    }

    char *out = string_buffer_to_string(sb);
    string_buffer_destroy(sb);
    return out;
}

static void copy_properties(config_t *config, const char *src, const char *dest)
{
    assert(config != NULL);
    assert(src != NULL);
    assert(dest != NULL);

    zhash_t *newkeys = zhash_create(sizeof(char*), sizeof(zarray_t*), zhash_str_hash, zhash_str_equals); // zarray<char*>

    zhash_iterator_t vit;
    zhash_iterator_init(config->keys, &vit);

    char *key;
    zarray_t *values;

    while (zhash_iterator_next(&vit, &key, &values)) {

        if (str_starts_with(key, src)) {
            char *prop_name = str_substring(key, strlen(src), -1);
            char *new_key_name = sprintf_alloc("%s%s", dest, prop_name);

            char *tmp = str_replace(new_key_name, ":", "");
            free(new_key_name);
            new_key_name = tmp;

            if (str_starts_with(dest, ":")) {
                char *tmp = sprintf_alloc(":%s", new_key_name);
                free(new_key_name);
                new_key_name = tmp;
            }

            zhash_put(newkeys, &new_key_name, &values, NULL, NULL); // no previous key->value binding possible
            free(prop_name);
        }
    }

    zhash_iterator_init(newkeys, &vit);
    while (zhash_iterator_next(&vit, &key, &values)) {


        zarray_t *newvalues = NULL;
        zhash_get(newkeys, &key, &newvalues);

        zarray_t *newvalues_copy = copy_values(newvalues);

        char *oldkey;
        zarray_t *oldvalue;

        if (zhash_put(config->keys, &key, &newvalues_copy, &oldkey, &oldvalue)) {
            free(oldkey);
            zarray_vmap(oldvalue, free);
            zarray_destroy(oldvalue);
        }
    }

    zhash_destroy(newkeys);
}

static void parse(config_t *config, generic_tokenizer_feeder_t *feeder, const char *keyroot, int depth)
{
    assert(config != NULL);
    assert(keyroot != NULL);

    uint32_t unique_index;

    while (generic_tokenizer_feeder_has_next(feeder)) {

        if (generic_tokenizer_feeder_consume(feeder, "}")) {
            if (depth == 0) {
                syntax_error(config, feeder, "unmatched }");
            }
            return;
        }

        if (!generic_tokenizer_feeder_has_next(feeder))
            return;

        // invariant: maintain exactly one newly-allocated string in keypart.
        char *keypart = NULL;
        if (generic_tokenizer_feeder_consume(feeder, ":")) {
            keypart = sprintf_alloc(":%s", generic_tokenizer_feeder_next(feeder));
        } else {
            keypart = strdup(generic_tokenizer_feeder_next(feeder));
        }
        if (!strcmp (keypart, CONFIG_INCLUDE_KEYWORD)) {

            if (!generic_tokenizer_feeder_has_next(feeder)) {
                syntax_error(config, feeder, "premature EOF");
                return;
            }
            const char *file_tmp = generic_tokenizer_feeder_next (feeder);
            char *file = NULL;
            if (file_tmp[0] == '"') {
                file = str_substring (file_tmp, 1, strlen (file_tmp) - 1);
            }
            else {
                file = strdup (file_tmp);
            }

            char *victim = file;
            file = expand_environment_variables (file);
            free (victim);

            if (file[0] != '/') {
                char *cur_file = NULL ;
                zarray_get (config->included_files, zarray_size (config->included_files) - 1, &cur_file);


                // dirname can modify things passed to it
                // and returns a string that gets overwritten
                // to multiple calls to dirname
                char *tmp = strdup (cur_file);
                char *dir = strdup (dirname (tmp));
                free (tmp);
                char *victim = file;
                file = sprintf_alloc ("%s/%s", dir, file);
                free (victim);
                free (dir);
            }

            generic_tokenizer_feeder_require (feeder, ";");


            if (config_import_path (config, file)) {
                //fprintf(stderr,"Could not import path from %s\n", file);
                fprintf(stderr, "Could not import path from %s\n", file);
            }

            free (file);
            free (keypart);
            continue;

        }

        if (str_ends_with(keypart, "#")) {
            char *keybase = str_substring(keypart, 0, strlen(keypart) - 1);
            char *unique_key = sprintf_alloc("%s%s", keyroot, keybase);
            if (!zhash_get(config->unique_indices, &unique_key, &unique_index)) {
                unique_index = 0;
                zhash_put(config->unique_indices, &unique_key, &unique_index, NULL, NULL);
            }
            char *new_keypart = sprintf_alloc("%s%d", keybase, unique_index);
            unique_index++;
            char *old_key = NULL;
            zhash_put(config->unique_indices, &unique_key, &unique_index, &old_key, NULL);
            if (old_key != unique_key)
                free(old_key);
            free(keybase);
            free(keypart);
            keypart = new_keypart;
        }

        if (!generic_tokenizer_feeder_has_next(feeder)) {
            syntax_error(config, feeder, "premature EOF");
            return;
        }

        // inheriting?
        if (generic_tokenizer_feeder_consume(feeder, ":")) {
            while (1) {
                const char *superclass = generic_tokenizer_feeder_next(feeder);
                if (1) {
                    char *from = sprintf_alloc(":%s.", superclass);
                    char *to = sprintf_alloc("%s%s.", keyroot, keypart);
                    copy_properties(config, from, to);
                    free(from);
                    free(to);
                }
                if (1) {
                    char *from = sprintf_alloc("%s.", superclass);
                    char *to = sprintf_alloc("%s%s.", keyroot, keypart);
                    copy_properties(config, from, to);
                    free(from);
                    free(to);
                }

                if (!generic_tokenizer_feeder_consume(feeder, ","))
                    break;
            }
        }

        // non-inheriting enclosing block?
        if (generic_tokenizer_feeder_consume(feeder, "{")) {
            char *newkeyroot = sprintf_alloc("%s%s.", keyroot, keypart);
            parse(config, feeder, newkeyroot, depth + 1);
            free(newkeyroot);
            free(keypart);
            continue;
        }

/*  This form of inheritence is a bit useless?

        if (generic_tokenizer_feeder_consume(feeder, "+{")) {
            if (1) {
                char *from = sprintf_alloc("%s", keyroot);
                char *to = sprintf_alloc("%s%s.", keyroot, keypart);
                copy_properties(config, from, to);
                free(from);
                free(to);
            }

            char *newkeyroot = sprintf_alloc("%s%s.", keyroot, keypart);
            parse(config, feeder, newkeyroot, depth + 1);
            free(newkeyroot);
            free(keypart);
            continue;
        }
*/

        // otherwise, it's a key/value declaration with 'keypart' as the key.
        generic_tokenizer_feeder_require(feeder, "=");

        zarray_t *values = zarray_create(sizeof(char*));

        if (generic_tokenizer_feeder_consume(feeder, "[")) {
            // read a list of values
            while (1) {
                if (!generic_tokenizer_feeder_has_next(feeder)) {
                    syntax_error(config, feeder, "premature EOF");
                    return;
                }

                const char *_val = generic_tokenizer_feeder_next(feeder);
                if (!strcmp(_val, "]"))
                    break;

                char *val;
                if (_val[0] == '\"')
                    val = decode_string_literal(_val); // makes a copy
                else
                    val = strdup(_val);

                zarray_add(values, &val);

                // consume optional comma
                generic_tokenizer_feeder_consume(feeder, ",");
            }
        } else {
            // read a single value
            const char *_val = generic_tokenizer_feeder_next(feeder);
            char *val;

            if (_val[0] == '\"')
                val = decode_string_literal(_val); // makes a copy
            else
                val = strdup(_val);

            zarray_add(values, &val);
        }

        generic_tokenizer_feeder_require(feeder, ";");

        char *key = sprintf_alloc("%s%s", keyroot, keypart);

        char *oldkey;
        zarray_t *oldvalue;

        if (zhash_put(config->keys, &key, &values, &oldkey, &oldvalue)) {
            free(oldkey);
            zarray_vmap(oldvalue, free);
            zarray_destroy(oldvalue);
        }

        free(keypart);
    }
}

int config_import_path(config_t *config, const char *path)
{
    assert(config != NULL);

    char *full_path = realpath (path, NULL);
    if (full_path == NULL) {
        char *msg = sprintf_alloc("config_import_path calling realpath with path [%s]", path);
        perror(msg);
        free(msg);
        return 1;
    }

    for (int i = 0; i < zarray_size (config->included_files); i++) {

        char *file = NULL;
        zarray_get (config->included_files, i, &file);

        int unique_file = strcmp (file, full_path);
        if (!unique_file) {
            char *cur_file = NULL;
            zarray_get (config->included_files, zarray_size (config->included_files) - 1, &cur_file);
            //nfailf("Config file %s is trying to include %s which would create a circular dependency\n", cur_file, full_path);
            fprintf(stderr, "Config file %s is trying to include %s which would create a circular dependency\n", cur_file, full_path);
            assert (0 && "Config circular dependency in include files");
        }
    }

    zarray_add (config->included_files, &full_path);

    zarray_t *tokens = generic_tokenizer_tokenize_path(config->gt, path);
    if(tokens == NULL)
        return -1; // failed to open config file

    generic_tokenizer_feeder_t *feeder = generic_tokenizer_feeder_create(tokens);

    parse(config, feeder, "", 0);

    // Stack
    char *removed_file = NULL;
    int last_idx = zarray_size (config->included_files) - 1;
    zarray_get (config->included_files, last_idx, &removed_file);
    zarray_remove_index (config->included_files, last_idx, 0);

    free (removed_file);

    generic_tokenizer_feeder_destroy(feeder);
    generic_tokenizer_tokens_destroy(tokens);

    return 0;
}

config_t *config_create_path(const char *path)
{
    config_t *config = config_create();

    if (config_import_path(config, path)) {
        config_destroy(config);
        return NULL;
    }

    return config;
}

void config_debug(const config_t *config)
{
    assert(config != NULL);

    zhash_iterator_t vit;

    zhash_iterator_init(config->keys, &vit);
    const char *key;
    zarray_t *vals;
    while (zhash_iterator_next(&vit, &key, &vals)) {
        printf("key %s (%d values): ", key, zarray_size(vals));
        for (int i = 0; i < zarray_size(vals); i++) {
            char *val = NULL;
            zarray_get(vals, i, &val);
            printf("%s ", val);
        }
        printf("\n");
    }
}

void config_destroy(config_t *config)
{
    if (config == NULL)
        return;

    zhash_iterator_t vit;
    char *key;

    // destroy tokenizer
    generic_tokenizer_destroy(config->gt);

    // destroy keys and values
    zarray_t *vals;
    zhash_iterator_init(config->keys, &vit);
    while (zhash_iterator_next(&vit, &key, &vals)) {
        zhash_iterator_remove(&vit);
        zarray_vmap(vals, free);
        zarray_destroy(vals);
        free(key);
    }
    assert(zhash_size(config->keys) == 0);
    zhash_destroy(config->keys);

    // destroy unique-indices keys and hash
    uint32_t val;
    zhash_iterator_init(config->unique_indices, &vit);
    while (zhash_iterator_next(&vit, &key, &val)) {
        zhash_iterator_remove(&vit);
        free(key);
    }
    assert(zhash_size(config->unique_indices) == 0);
    zhash_destroy(config->unique_indices);

    // destroy list of "good" (non-abstract) keys
    // note: these were not copied, the originals are
    // in the main hash table (above)
    if (config->cached_keys != NULL)
        zarray_destroy(config->cached_keys);

    // destroy cached mati_t objects
    mati_t *mati;
    zhash_iterator_init(config->cached_matis, &vit);
    while (zhash_iterator_next(&vit, &key, &mati)) {
        zhash_iterator_remove(&vit);
        mati_destroy(mati);
        free(key);
    }
    assert(zhash_size(config->cached_matis) == 0);
    zhash_destroy(config->cached_matis);

    // destroy cached matd_t objects
    matd_t *matd;
    zhash_iterator_init(config->cached_matds, &vit);
    while (zhash_iterator_next(&vit, &key, &matd)) {
        zhash_iterator_remove(&vit);
        matd_destroy(matd);
        free(key);
    }
    assert(zhash_size(config->cached_matds) == 0);
    zhash_destroy(config->cached_matds);


    // destroy cached zarry_t objects for booleans
    zhash_vmap_values(config->cached_zarrays_booleans, zarray_destroy);
    zhash_vmap_keys(config->cached_zarrays_booleans, free);
    zhash_destroy(config->cached_zarrays_booleans);

    // destroy cached zarray_t objects for ints
    zhash_vmap_values(config->cached_zarrays_ints, zarray_destroy);
    zhash_vmap_keys(config->cached_zarrays_ints, free);
    zhash_destroy(config->cached_zarrays_ints);

    // destroy cached zarray_t objects for doubles
    zhash_vmap_values(config->cached_zarrays_doubles, zarray_destroy);
    zhash_vmap_keys(config->cached_zarrays_doubles, free);
    zhash_destroy(config->cached_zarrays_doubles);

    // destroy zset_t containing included filenames
    zarray_vmap (config->included_files, free);
    zarray_destroy (config->included_files);

    free(config->prefix);

    // finally..
    memset(config, 0, sizeof(config_t));
    free(config);
}

////////////////////////////////////////
// private helper methods

static int cast_to_int(const char * key, const char * val, int * out)
{
    char * end;
    *out = strtol (val, &end, 0);
    if (end == val || *end != '\0') {
        fprintf (stderr, "Error: key \"%s\" (\"%s\") did not cast "
                "properly to int\n", key, val);
        return -1;
    }
    return 0;
}

static int cast_to_boolean(const char * key, const char * val, int * out)
{
    if (!strcasecmp (val, "y") || !strcasecmp (val, "yes") ||
            !strcasecmp (val, "true") || !strcmp (val, "1"))
        *out = 1;
    else if (!strcasecmp (val, "n") || !strcasecmp (val, "no") ||
            !strcasecmp (val, "false") || !strcmp (val, "0"))
        *out = 0;
    else {
        fprintf (stderr, "Error: Config: key \"%s\" (\"%s\") did not cast "
                "properly to boolean\n", key, val);
        return -1;
    }
    return 0;
}

static int cast_to_double(const char * key, const char * val, double * out)
{
    char * end;
    *out = strtod (val, &end);
    if (end == val || *end != '\0') {
        fprintf (stderr, "Error: Config: key \"%s\" (\"%s\") did not cast "
                "properly to double\n", key, val);
        return -1;
    }
    return 0;
}

static void missing_required(const char * key)
{
    //fatal("Config: Required key '%s' not found\n", key);
    fprintf(stderr, "Config: Required key '%s' not found\n", key);
    assert(0);
    exit(EXIT_FAILURE);
}

static void required_array_casting_failed(const char * key, int i)
{
    //fatal("Config: Failed to cast required key '%s' at index %d\n", key, i);
    fprintf(stderr, "Config: Failed to cast required key '%s' at index %d\n", key, i);
    exit(EXIT_FAILURE);
}

static void required_matrix_parsing_failed(const char * key)
{
    //fatal("Config: Failed to parse required key '%s' as matrix\n", key);
    fprintf(stderr, "Config: Failed to parse required key '%s' as matrix\n", key);
    exit(EXIT_FAILURE);
}

static void required_matrix_casting_failed(const char * key, int row, int col)
{
    //fatal("Config: Failed to cast required key '%s' as matrix at row %d col %d\n",
    //        key, row, col);
    fprintf(stderr, "Config: Failed to cast required key '%s' as matrix at row %d col %d\n",
            key, row, col);
    exit(EXIT_FAILURE);
}

void config_set_prefix(config_t *config, const char *prefix)
{
    zhash_iterator_t vit;
    char *key;

    // destroy cached mati_t objects
    mati_t *mati;
    zhash_iterator_init(config->cached_matis, &vit);
    while (zhash_iterator_next(&vit, &key, &mati)) {
        zhash_iterator_remove(&vit);
        mati_destroy(mati);
        free(key);
    }
    assert(zhash_size(config->cached_matis) == 0);

    // destroy cached matd_t objects
    matd_t *matd;
    zhash_iterator_init(config->cached_matds, &vit);
    while (zhash_iterator_next(&vit, &key, &matd)) {
        zhash_iterator_remove(&vit);
        matd_destroy(matd);
        free(key);
    }
    assert(zhash_size(config->cached_matds) == 0);


    // destroy cached zarry_t objects for booleans
    zhash_vmap_values(config->cached_zarrays_booleans, zarray_destroy);
    zhash_vmap_keys(config->cached_zarrays_booleans, free);

    // destroy cached zarray_t objects for ints
    zhash_vmap_values(config->cached_zarrays_ints, zarray_destroy);
    zhash_vmap_keys(config->cached_zarrays_ints, free);

    // destroy cached zarray_t objects for doubles
    zhash_vmap_values(config->cached_zarrays_doubles, zarray_destroy);
    zhash_vmap_keys(config->cached_zarrays_doubles, free);

    free(config->prefix);

    config->prefix = NULL;
    if (prefix != NULL)
        config->prefix = strdup(prefix);
}

static int cfg_get_key(const config_t * conf, const char *desired_key, void * values_ptr)
{
    if (conf->prefix) {
        int len1 = strlen(conf->prefix);
        int len2 = strlen(desired_key);
        char buf[len1+len2+1];
        memcpy(buf, conf->prefix, len1);
        memcpy(&buf[len1], desired_key, len2);
        buf[len1+len2] = 0;

        const char * buf_ptr = buf;

        return (zhash_get(conf->keys, &buf_ptr, values_ptr) ? 0 : -1 );
    }
    return (zhash_get(conf->keys, &desired_key, values_ptr) ? 0 : -1 );
}

static zarray_t * cfg_require_key(const config_t * conf, const char * desired_key)
{
    zarray_t *values = NULL;
    if (cfg_get_key(conf, desired_key, &values))
        missing_required(desired_key);

    assert(values != NULL);
    return values;
}

static zarray_t * cfg_require_single_key(const config_t * conf, const char * desired_key)
{
    zarray_t *values = cfg_require_key(conf, desired_key);
    if (zarray_size(values) != 1) {
        //fatal("cfg_require_single_key(): Require zarray_size(values) == 1. Was %d\n",
        //        zarray_size(values));
        fprintf(stderr, "cfg_require_single_key(): Require zarray_size(values) == 1. Was %d\n",
                zarray_size(values));
        exit(EXIT_FAILURE);
    }

    return values;
}

static int cfg_get_single_string(const config_t *conf, const char * key, char **str)
{
    zarray_t *values;
    if (cfg_get_key(conf, key, &values) || zarray_size(values) != 1)
        return -1;

    zarray_get(values, 0, str);
    return 0;
}

static int cfg_get_multiple_strings(const config_t *conf, const char * key, zarray_t ** values)
{
    if (cfg_get_key(conf, key, values))
        return -1;

    return 0;
}

static int array_get_dimensions(const zarray_t *values, int * _nrows, int * _ncols)
{
    if (values == NULL)
        return -1;

    int n = zarray_size(values);
    if (n == 0) {
        *_nrows = 0;
        *_ncols = 0;
        return 0;
    }

    int nrows = 0;
    int ncols = -1;

    int cols = 0;
    for (int i=0; i < n; i++) {

        char *val = NULL;
        zarray_get(values, i, &val);

        // end of row
        if (strcmp(val, ";") == 0)
        {
            if (ncols == -1)
                ncols = cols;
            else if (ncols != cols)
                return -1;

            cols = 0;
        }
        else
        {
            // start of a new row?
            if (cols == 0)
                nrows++;

            cols++;
        }
    }
    if (ncols == -1)
        ncols = cols;  // vector

    *_nrows = nrows;
    *_ncols = ncols;

    return 0;
}

////////////////////////////////////////
// public accessors

const zarray_t *config_get_keys(config_t * conf)
{
    if (conf == NULL || conf->keys == NULL)
        return NULL;

    if (conf->cached_keys != NULL)
        return conf->cached_keys;

    zarray_t *realkeys = zarray_create(sizeof(char*));

    zhash_iterator_t vit;
    zhash_iterator_init(conf->keys, &vit);
    char* key;
    zarray_t *values;

    while (zhash_iterator_next(&vit, &key, &values)) {
        // add as long as it's not an abstract key
        if (str_starts_with(key, ":") == 0)
            zarray_add(realkeys, &key);
    }

    conf->cached_keys = realkeys;
    return realkeys;
}

int config_has_key(const config_t * conf, const char * desired_key)
{
    assert(conf != NULL);
    assert(desired_key != NULL);

    if (conf->keys == NULL) {
        //fatal("config_has_key(): Error: config keys are null\n");
        fprintf(stderr, "config_has_key(): Error: config keys are null\n");
        exit(EXIT_FAILURE);
    }

    if (conf->prefix) {
        int len1 = strlen(conf->prefix);
        int len2 = strlen(desired_key);
        char buf[len1+len2+1];
        memcpy(buf, conf->prefix, len1);
        memcpy(&buf[len1], desired_key, len2);
        buf[len1+len2] = 0;

        const char * buf_ptr = buf;

        return zhash_contains(conf->keys, &buf_ptr);
    }

    return zhash_contains(conf->keys, &desired_key);

}

const char* config_require_string(const config_t * conf, const char * key)
{
    assert(conf != NULL);
    assert(key != NULL);

    zarray_t *values = cfg_require_single_key(conf, key);
    char* str = NULL;
    zarray_get(values, 0, &str);

    return str;
}

int config_require_boolean(const config_t * conf, const char * key)
{
    assert(conf != NULL);
    assert(key != NULL);

    zarray_t *values = cfg_require_single_key(conf, key);
    char* str = NULL;
    zarray_get(values, 0, &str);

    int out = 0;
    if (cast_to_boolean(key, str, &out) != 0)
        missing_required(key);

    return out;
}

int config_require_int(const config_t * conf, const char * key)
{
    assert(conf != NULL);
    assert(key != NULL);

    zarray_t *values = cfg_require_single_key(conf, key);
    char* str = NULL;
    zarray_get(values, 0, &str);

    int out = 0;
    if (cast_to_int(key, str, &out) != 0)
        missing_required(key);

    return out;
}

double config_require_double(const config_t * conf, const char * key)
{
    assert(conf != NULL);
    assert(key != NULL);

    zarray_t *values = cfg_require_single_key(conf, key);
    char* str = NULL;
    zarray_get(values, 0, &str);

    double out = -1;
    if (cast_to_double(key, str, &out) != 0)
        missing_required(key);

    return out;
}

const char *config_get_string(const config_t * conf, const char * key, const char * def)
{
    assert(conf != NULL);
    assert(key != NULL);

    char *str = NULL;
    if (cfg_get_single_string(conf, key, &str))
        return def;

    return str;
}

int config_get_boolean(const config_t * conf, const char * key, int def)
{
    assert(conf != NULL);
    assert(key != NULL);

    char *str = NULL;
    if (cfg_get_single_string(conf, key, &str))
        return def;

    int out = 0;
    if (cast_to_boolean(key, str, &out))
        return def;

    return out;
}

int config_get_int(const config_t * conf, const char * key, int def)
{
    assert(conf != NULL);
    assert(key != NULL);

    char *str = NULL;
    if (cfg_get_single_string(conf, key, &str))
        return def;

    int out = 0;
    if (cast_to_int(key, str, &out))
        return def;

    return out;
}

double config_get_double(const config_t * conf, const char * key, double def)
{
    assert(conf != NULL);
    assert(key != NULL);

    char *str = NULL;
    if (cfg_get_single_string(conf, key, &str))
        return def;

    double out = 0;
    if (cast_to_double(key, str, &out))
        return def;

    return out;
}

const zarray_t *config_require_strings(const config_t * conf, const char * key)
{
    assert(conf != NULL);
    assert(key != NULL);

    zarray_t *values = cfg_require_key(conf, key);

    return values;
}

const zarray_t *config_require_booleans(const config_t * conf, const char * key)
{
    assert(conf != NULL);
    assert(key != NULL);

    if (0 && zhash_contains(conf->cached_zarrays_booleans, &key)) {
        zarray_t *result = NULL;
        zhash_get(conf->cached_zarrays_booleans, &key, &result);
        return result;
    }

    zarray_t *values = cfg_require_key(conf, key);
    int n = zarray_size(values);

    zarray_t *result = zarray_create(sizeof(int));
    for (int i=0; i < n; i++) {
        char* str = NULL;
        zarray_get(values, i, &str);

        int value = 0;

        if (cast_to_boolean(key, str, &value))
            required_array_casting_failed(key, i);
        else
            zarray_add(result, &value);
    }

    //char *keycopy = strdup(key);
    //zhash_put(conf->cached_zarrays_booleans, &keycopy, &result, NULL, NULL);
    return result;
}

const zarray_t *config_require_ints(const config_t * conf, const char * key)
{
    assert(conf != NULL);
    assert(key != NULL);

    if (0 && zhash_contains(conf->cached_zarrays_ints, &key)) {
        zarray_t *result = NULL;
        zhash_get(conf->cached_zarrays_ints, &key, &result);
        return result;
    }

    zarray_t *values = cfg_require_key(conf, key);
    int n = zarray_size(values);

    zarray_t *result = zarray_create(sizeof(int));
    for (int i=0; i < n; i++) {
        char *str = NULL;
        zarray_get(values, i, &str);

        int value = 0;

        if (cast_to_int(key, str, &value))
            required_array_casting_failed(key, i);
        else
            zarray_add(result, &value);
    }

    //char *keycopy = strdup(key);
    //zhash_put(conf->cached_zarrays_ints, &keycopy, &result, NULL, NULL);
    return result;
}

void config_require_doubles_len(const config_t * conf, const char * key, double *vs, int len)
{
    const zarray_t *za = config_require_doubles(conf, key);

    if (zarray_size(za) != len) {
        printf("key %s requires %d elements\n", key, len);
        exit(-1);
    }

    memcpy(vs, za->data, len * sizeof(double));
}

const zarray_t *config_require_doubles(const config_t * conf, const char * key)
{
    assert(conf != NULL);
    assert(key != NULL);

    if (0 && zhash_contains(conf->cached_zarrays_doubles, &key)) {
        zarray_t *result = NULL;
        zhash_get(conf->cached_zarrays_doubles, &key, &result);
        return result;
    }

    zarray_t *values = cfg_require_key(conf, key);
    int n = zarray_size(values);

    zarray_t *result = zarray_create(sizeof(double));
    for (int i=0; i < n; i++) {
        char* str = NULL;
        zarray_get(values, i, &str);

        double value = 0.0;

        if (cast_to_double(key, str, &value))
            required_array_casting_failed(key, i);
        else
            zarray_add(result, &value);
    }

    //char *keycopy = strdup(key);
    //zhash_put(conf->cached_zarrays_doubles, &keycopy, &result, NULL, NULL);
    return result;
}

const zarray_t *config_get_strings(const config_t * conf, const char * key, const zarray_t * def)
{
    assert(conf != NULL);
    assert(key != NULL);

    zarray_t *values;
    if (cfg_get_multiple_strings(conf, key, &values))
        return def;

    return values;
}

const zarray_t *config_get_booleans(const config_t * conf, const char * key, const zarray_t * def)
{
    assert(conf != NULL);
    assert(key != NULL);

    if (0 && zhash_contains(conf->cached_zarrays_booleans, &key)) {
        zarray_t *result = NULL;
        zhash_get(conf->cached_zarrays_booleans, &key, &result);
        return result;
    }

    zarray_t *values;
    if (cfg_get_multiple_strings(conf, key, &values))
        return def;

    int n = zarray_size(values);
    zarray_t *result = zarray_create(sizeof(int));

    for (int i=0; i < n; i++) {
        char* str = NULL;
        zarray_get(values, i, &str);

        int value = 0;

        if (cast_to_boolean(key, str, &value)) {
            zarray_destroy(result);
            return def;
        } else
            zarray_add(result, &value);
    }

    //char *keycopy = strdup(key);
    //zhash_put(conf->cached_zarrays_booleans, &keycopy, &result, NULL, NULL);
    return result;
}

const zarray_t *config_get_ints(const config_t * conf, const char * key, const zarray_t * def)
{
    assert(conf != NULL);
    assert(key != NULL);

    if (0 && zhash_contains(conf->cached_zarrays_ints, &key)) {
        zarray_t *result = NULL;
        zhash_get(conf->cached_zarrays_ints, &key, &result);
        return result;
    }

    zarray_t *values;
    if (cfg_get_multiple_strings(conf, key, &values))
        return def;

    int n = zarray_size(values);
    zarray_t *result = zarray_create(sizeof(int));
    for (int i=0; i < n; i++) {
        char* str = NULL;
        zarray_get(values, i, &str);

        int value = 0;

        if (cast_to_int(key, str, &value)) {
            zarray_destroy(result);
            return def;
        } else
            zarray_add(result, &value);
    }

    //char *keycopy = strdup(key);
    //zhash_put(conf->cached_zarrays_ints, &keycopy, &result, NULL, NULL);
    return result;
}

const zarray_t *config_get_doubles(const config_t  * conf, const char * key, const zarray_t * def)
{
    assert(conf != NULL);
    assert(key != NULL);

    if (0 && zhash_contains(conf->cached_zarrays_doubles, &key)) {
        zarray_t *result = NULL;
        zhash_get(conf->cached_zarrays_doubles, &key, &result);
        return result;
    }

    zarray_t *values;
    if (cfg_get_multiple_strings(conf, key, &values))
        return def;

    int n = zarray_size(values);
    zarray_t *result = zarray_create(sizeof(double));
    for (int i=0; i < n; i++) {
        char* str = NULL;
        zarray_get(values, i, &str);

        double value = 0.0;

        if (cast_to_double(key, str, &value)) {
            zarray_destroy(result);
            return def;
        } else
            zarray_add(result, &value);
    }

    //char *keycopy = strdup(key);
    //zhash_put(conf->cached_zarrays_doubles, &keycopy, &result, NULL, NULL);
    return result;
}

const mati_t *config_require_mati(const config_t * conf, const char * key)
{
    assert(conf != NULL);
    assert(key != NULL);

    if (0 && zhash_contains(conf->cached_matis, &key)) {
        mati_t *m = NULL;
        zhash_get(conf->cached_matis, &key, &m);
        return m;
    }

    zarray_t *values = cfg_require_key(conf, key);
    int n = zarray_size(values);

    int nrows = 0, ncols = 0;
    if (array_get_dimensions(values, &nrows, &ncols))
        required_matrix_parsing_failed(key);

    mati_t *m = mati_create(nrows, ncols);
    for (int r=0; r < nrows; r++) {
        for (int c=0; c < ncols; c++) {

            int index = r*(ncols+1) + c;
            assert(index < n);

            char* str = NULL;
            zarray_get(values, index, &str);

            if (cast_to_int(key, str, (int*) (m->data + r*ncols + c)))
                required_matrix_casting_failed(key, r, c);
        }
    }

    //char *keycopy = strdup(key);
    //zhash_put(conf->cached_matis, &keycopy, &m, NULL, NULL);
    return m;
}

const matd_t *config_require_matd(const config_t * conf, const char * key)
{
    assert(conf != NULL);
    assert(key != NULL);

    if (0 && zhash_contains(conf->cached_matds, &key)) {
        matd_t *m = NULL;
        zhash_get(conf->cached_matds, &key, &m);
        return m;
    }

    zarray_t *values = cfg_require_key(conf, key);
    int n = zarray_size(values);

    int nrows = 0, ncols = 0;
    if (array_get_dimensions(values, &nrows, &ncols))
        required_matrix_parsing_failed(key);

    matd_t *m = matd_create(nrows, ncols);
    for (int r=0; r < nrows; r++) {
        for (int c=0; c < ncols; c++) {

            int index = r*(ncols+1) + c;
            assert(index < n);

            char* str = NULL;
            zarray_get(values, index, &str);

            if (cast_to_double(key, str, (double*) (m->data + r*ncols + c)))
                required_matrix_casting_failed(key, r, c);
        }
    }

    //char *keycopy = strdup(key);
    //zhash_put(conf->cached_matds, &keycopy, &m, NULL, NULL);
    return m;
}

const mati_t *config_get_mati(const config_t * conf, const char * key, const mati_t * def)
{
    assert(conf != NULL);
    assert(key != NULL);

    if (0 && zhash_contains(conf->cached_matis, &key)) {
        mati_t *m = NULL;
        zhash_get(conf->cached_matis, &key, &m);
        return m;
    }

    zarray_t *values;
    if (cfg_get_multiple_strings(conf, key, &values))
        return def;

    int n = zarray_size(values);

    int nrows = 0, ncols = 0;
    if (array_get_dimensions(values, &nrows, &ncols))
        return def;

    mati_t *m = mati_create(nrows, ncols);
    for (int r=0; r < nrows; r++) {
        for (int c=0; c < ncols; c++) {

            int index = r*(ncols+1) + c;
            assert(index < n);

            char* str = NULL;
            zarray_get(values, index, &str);

            if (cast_to_int(key, str, (int*) (m->data + r*ncols + c))) {
                mati_destroy(m);
                return def;
            }
        }
    }

    //char *keycopy = strdup(key);
    //zhash_put(conf->cached_matis, &keycopy, &m, NULL, NULL);
    return m;
}

const matd_t *config_get_matd(const config_t * conf, const char * key, const matd_t * def)
{
    assert(conf != NULL);
    assert(key != NULL);

    if (0 && zhash_contains(conf->cached_matds, &key)) {
        matd_t *m = NULL;
        zhash_get(conf->cached_matds, &key, &m);
        return m;
    }

    zarray_t *values;
    if (cfg_get_multiple_strings(conf, key, &values))
        return def;

    int n = zarray_size(values);

    int nrows = 0, ncols = 0;
    if (array_get_dimensions(values, &nrows, &ncols))
        return def;

    matd_t *m = matd_create(nrows, ncols);
    for (int r=0; r < nrows; r++) {
        for (int c=0; c < ncols; c++) {

            int index = r*(ncols+1) + c;
            assert(index < n);

            char* str = NULL;
            zarray_get(values, index, &str);

            if (cast_to_double(key, str, (double*) (m->data + r*ncols + c))) {
                matd_destroy(m);
                return def;
            }
        }
    }

    //char *keycopy = strdup(key);
    //zhash_put(conf->cached_matds, &keycopy, &m, NULL, NULL);
    return m;
}
