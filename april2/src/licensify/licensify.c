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
#include <sys/types.h>
#include <sys/stat.h>
#include <dirent.h>
#include <assert.h>
#include <time.h>
#include <unistd.h>
#include <ctype.h>

#include "common/getopt.h"
#include "common/zarray.h"

typedef struct state state_t;
struct state
{
    getopt_t  *getopt;
    char      *outdir;

    zarray_t  *licenses_names;
    zarray_t  *licenses_values;
};

// sort so longer names first
int string_length_compare(const void *_a, const void *_b)
{
    const char *a = *((char**) _a);
    const char *b = *((char**) _b);

    return strlen(b) - strlen(a);
}

char *read_file(const char *path)
{
    FILE *f = fopen(path, "r");
    if (f == NULL) {
        return NULL;
    }

    fseek(f, 0, SEEK_END);
    long datalen = ftell(f);
    fseek(f, 0, SEEK_SET);

    char *data = calloc(1, datalen+1);
    if (datalen != fread(data, 1, datalen, f)) {
        printf("Failed to read file: %s\n", path);
        fclose(f);
        return NULL;
    }
    fclose(f);

    return data;
}

int is_substitution(char *in, char *out, int maxout)
{
    if (in[0] != '$')
        return 0;

    int out_pos = 0;
    int in_pos = 1;

    while (out_pos+1 < maxout && in[in_pos] != '\0' && (isalpha(in[in_pos]) || in[in_pos]=='_')) {
        out[out_pos++] = in[in_pos++];
    }

    out[out_pos] = 0;
    return 1;
}

char *string_substitute(char *in, zarray_t *needles, zarray_t *replacements, int maxlen, int warn, int *nsubst, const char *path)
{
    int in_len = strlen(in);
    int out_alloc = in_len + 8192;
    char *out = calloc(1, out_alloc);

    int in_pos = 0, out_pos = 0;
    int lineno = 1;

    *nsubst = 0;

    while (in_pos < in_len) {

        if (in[in_pos] == '\n')
            lineno += 1;

        char needle[1024];
        if (in_pos >= maxlen ||
                !is_substitution(&in[in_pos], needle, sizeof(needle))) {
            out[out_pos++] = in[in_pos++];
            continue;
        }

        // skip past token and the '$'
        in_pos += strlen(needle) + 1;

        int replaced = 0;

        for (int needleidx = 0; needleidx < zarray_size(needles); needleidx++) {
            char *this_needle;
            zarray_get(needles, needleidx, &this_needle);

            if (strcmp(this_needle, needle))
                continue;

            char *replacement;
            zarray_get(replacements, needleidx, &replacement);
            assert(replacement != NULL);

            int replacement_len = strlen(replacement);

            int need_len = out_pos + in_len - in_pos + 1 + replacement_len;
            if (out_alloc < need_len) {
                out = realloc(out, need_len);
                out_alloc = need_len;
            }

            for (int j = 0; j < replacement_len; j++) {
                out[out_pos++] = replacement[j];
            }

            (*nsubst)++;
            replaced = 1;
            break;
        }

        // if not matched, copy the literal "$TOKEN"
        if (!replaced) {
            if (warn) {
                printf("%s:%d: warning: unknown substitution \"%s\"\n",
                        path, lineno, needle);
            }
            for (int i = in_pos-strlen(needle)-1; i < in_pos; i += 1)
                out[out_pos++] = in[i];
            continue;
        }
    }

    return out;
}

// returns 0 on success
int do_input_file(state_t *state, const char *path)
{
    char *data = read_file(path);
    if (data == NULL) {
        printf("%s: failed to open input file\n", path);
        return -1;
    }

    int maxlen = strlen(data);

    if (getopt_get_bool(state->getopt, "only-first")) {
        for (int i = 0; i < maxlen - 1; i++) {
            if (data[i] == '*' && data[i+1] == '/') {
                maxlen = i;
                break;
            }
        }
    }

    int nsubst;
    char *out = string_substitute(data, state->licenses_names, state->licenses_values, maxlen,
                                  getopt_get_bool(state->getopt, "warn-unknown"), &nsubst, path);

    if (nsubst == 0) {
        printf("%s: warning: no license statements found\n", path);
    }

    char *outpath = calloc(1, strlen(path) + strlen(state->outdir) + 2);
    sprintf(outpath, "%s/%s", state->outdir, path);

    FILE *f = fopen(outpath, "r");
    if (f != NULL && !getopt_get_bool(state->getopt, "force")) {
        printf("%s: error: output file already exists: %s\n", path, outpath);
        fclose(f);
        return -1;
    }

    // make any directories needed along the way
    for (int i = 1; i < strlen(outpath); i++) {
        if (outpath[i] != '/')
            continue;

        char *dirpath = strdup(outpath);
        dirpath[i] = 0;
        struct stat buf;
        if (stat(dirpath, &buf)) {
            if (mkdir(dirpath, 0777)) {
                printf("failed to make directory: %s\n", dirpath);
                return -1;
            }
        }

        free(dirpath);
    }

    f = fopen(outpath, "w");
    if (f == NULL) {
        printf("ERROR: Couldn't open output path: %s\n", outpath);
        return -2;
    }

    fwrite(out, 1, strlen(out), f);
    fclose(f);

    free(data);

    return 0;
}

/* The input paths should be specified as "partial paths" that will be appended to "outdir".

   ./licensify src/common/foo.c /tmp/distribution-export/

   Will result in a file:

   /tmp/distribution-export/src/common/foo.c

   You can use the -C option to change the working directory so that
you can specify src/common/foo.c instead of a fully-qualified path .
*/

int main(int argc, char *argv[]) { state_t *state = calloc(1,
sizeof(state_t));

    state->getopt = getopt_create();

    getopt_add_bool(state->getopt, 'h', "help", 0, "Show this help");

    getopt_add_string(state->getopt, 'l', "licenses-dir", "/home/ebolson/magic2/licenses",
                      "Path to licenses directory");

    getopt_add_bool(state->getopt, '1', "only-first", 1,
                    "Only expand $DIRECTIVES in the first comment /* */ block");

    getopt_add_bool(state->getopt, 'w', "warn-unknown", 1,
                    "Warn when a $DIRECTIVES is unknown");
    getopt_add_bool(state->getopt, 'f', "force", 0,
                    "Overwrite if output file exists");

    getopt_add_string(state->getopt, 'C', "change-directory", ".",
                      "Change directory before accessing input files");

    if (!getopt_parse(state->getopt, argc, argv, 1) || getopt_get_bool(state->getopt, "help")) {
        printf("Usage: %s [options] <input files> <output path>\n", argv[0]);
        printf("(This program works a bit like 'cp', replacing $DIRECTIVES with license text.)\n\n");
        getopt_do_usage(state->getopt);
        exit(0);
    }

    if (chdir(getopt_get_string(state->getopt, "change-directory"))) {
        printf("Couldn't change directory to: %s\n", getopt_get_string(state->getopt, "change-directory"));
        exit(-1);
    }

    // read all licenses
    state->licenses_names = zarray_create(sizeof(char*));
    state->licenses_values = zarray_create(sizeof(char*));

    if (1) {
        const char *licensedir = getopt_get_string(state->getopt, "licenses-dir");
        DIR *dir = opendir(licensedir);
        if (dir == NULL) {
            printf("Couldn't open license directory: %s\n", licensedir);
            exit(-1);
        }

        zarray_t *lic_replacements_names = zarray_create(sizeof(char*));
        zarray_t *lic_replacements_values = zarray_create(sizeof(char*));

        if (1) {
            char *name = strdup("YEAR");
            time_t t = time(NULL);
            struct tm *tm = localtime(&t);

            char *value = malloc(1024);
            snprintf(value, 1024, "%d", tm->tm_year + 1900);

            zarray_add(lic_replacements_names, &name);
            zarray_add(lic_replacements_values, &value);
        }

        // make a list of all the license-file names.
        struct dirent *dirent;
        while ((dirent = readdir(dir)) != NULL) {
            if (dirent->d_name[0] == '.')
                continue;

            if (strchr(dirent->d_name, '~') != NULL)
                continue;

            char *license_name = strdup(dirent->d_name);
            zarray_add(state->licenses_names, &license_name);
        }

        // sort license names so that we try longer names first. Thus, if
        // we have LICENSE_BSD_2CLAUSE and LICENSE_BSD as valid license
        // names, we won't prematurely match a haystack containing
        // "LICENSE_BSD_2CLAUSE" as "LICENSE_BSD".
        zarray_sort(state->licenses_names, string_length_compare);

        // read the licenses, substituting $YEAR
        for (int idx = 0; idx < zarray_size(state->licenses_names); idx++) {
            char *license_name;
            zarray_get(state->licenses_names, idx, &license_name);

            int pathbuflen = strlen(licensedir) + 256 + 1;
            char *pathbuf = calloc(1, pathbuflen);

            snprintf(pathbuf, pathbuflen, "%s/%s", licensedir, license_name);

            char *data = read_file(pathbuf);
            if (data == NULL) {
                printf("failed to read path: %s\n", pathbuf);
                exit(-1);
            }

            int nsubst;
            char *data2 = string_substitute(data, lic_replacements_names, lic_replacements_values,
                                            strlen(data), 1, &nsubst, pathbuf);
            free(data);

            zarray_add(state->licenses_values, &data2);

            free(pathbuf);
        }

        closedir(dir);
    }

    const zarray_t *files = getopt_get_extra_args(state->getopt);
    if (zarray_size(files) == 0) {
        printf("No input or output directory specified.\n");
        exit(-2);
    }

    if (zarray_size(files) == 1) {
        printf("No output directory specified.\n");
    }

    zarray_get(files, zarray_size(files) - 1, &state->outdir);

    for (int fileidx = 0; fileidx < zarray_size(files)-1; fileidx++) {
        char *path;
        zarray_get(files, fileidx, &path);
        do_input_file(state, path);
    }

    return 0;
}
