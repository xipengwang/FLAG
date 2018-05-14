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

#include <string.h>
#include <stdint.h>
#include <stdlib.h>
#include <inttypes.h>
#include <stdio.h>

#include "image_source.h"
#include "url_parser.h"

#include "common/string_util.h"

struct image_source_provider *image_source_provider_head;

void image_source_null_setup(void);
void image_source_filedir_setup(void);
void image_source_tcp_setup(void);
void image_source_islog_setup(void);
void image_source_v4l2_setup(void);
void image_source_pgusb_setup(void);
void image_source_dc1394_setup(void);

void image_source_print_features(image_source_t *isrc);

image_source_t *image_source_open(const char *url)
{
   static int setup_complete = 0;

   if (!setup_complete) {
	  setup_complete = 1;
	  image_source_null_setup();
	  image_source_filedir_setup();
	  image_source_tcp_setup();
	  image_source_islog_setup();
	  image_source_v4l2_setup();
	  image_source_dc1394_setup();
	  image_source_pgusb_setup();
   }

   image_source_t *isrc = NULL;

    // get feature key/value pairs
    url_parser_t *urlp = url_parser_create(url);
    if (urlp == NULL) // bad URL format
        return NULL;

    const char *protocol = url_parser_get_protocol(urlp);

	for (struct image_source_provider *provider = image_source_provider_head; provider; provider = provider->next) {
	   if (!strcmp(protocol, provider->protocol))
		  return provider->open(urlp);
	}

    // handle parameters
    if (isrc != NULL) {
        int found[url_parser_num_parameters(urlp)];

        for (int param_idx = 0; param_idx < url_parser_num_parameters(urlp); param_idx++) {
            const char *key = url_parser_get_parameter_name(urlp, param_idx);
            const char *value = url_parser_get_parameter_value(urlp, param_idx);

            if (!strcmp(key, "fidx")) {
                fprintf(stderr, "image_source.c: set feature %30s = %15s\n", key, value);
                int fidx = atoi(url_parser_get_parameter(urlp, "fidx", "0"));
                fprintf(stderr, "SETTING fidx %d\n", fidx);
                isrc->set_format(isrc, fidx);
                found[param_idx] = 1;
                continue;
            }

            if (!strcmp(key, "format")) {
                fprintf(stderr, "image_source.c: set feature %30s = %15s\n", key, value);
                isrc->set_named_format(isrc, value);
                found[param_idx] = 1;
                continue;
            }

            if (!strcmp(key, "print")) {
                image_source_print_features(isrc);
                continue;
            }

            // pass through a device-specific parameter.
            for (int feature_idx = 0; feature_idx < isrc->num_features(isrc); feature_idx++) {

                if (!strcmp(isrc->get_feature_name(isrc, feature_idx), key)) {
                    char *endptr = NULL;
                    double dv = strtod(value, &endptr);
                    if (endptr != value + strlen(value)) {
                        fprintf(stderr, "Parameter for key '%s' is invalid. Must be a number.\n",
                               isrc->get_feature_name(isrc, feature_idx));
                        goto cleanup;
                    }

                    int res = isrc->set_feature_value(isrc, feature_idx, dv);
                    if (res != 0)
                        fprintf(stderr, "Error setting feature: key %s value %s, error code %d\n",
                               key, value, res);

                    double setvalue = isrc->get_feature_value(isrc, feature_idx);
                    fprintf(stderr, "image_source.c: set feature %30s = %15s (double %12.6f). Actually set to %8.3f\n", key, value, dv, setvalue);

                    found[param_idx] = 1;
                    break;
                }
            }
        }

        for (int param_idx = 0; param_idx < url_parser_num_parameters(urlp); param_idx++) {
            if (found[param_idx] != 1) {
                const char *key = url_parser_get_parameter_name(urlp, param_idx);
                const char *value = url_parser_get_parameter_value(urlp, param_idx);

                fprintf(stderr, "Parameter not found. Key: %s Value: %s\n", key, value);
            }
        }
    }

cleanup:
    url_parser_destroy(urlp);

    return isrc;
}

zarray_t *image_source_enumerate()
{
    zarray_t *urls = zarray_create(sizeof(char*));

	for (struct image_source_provider *provider = image_source_provider_head; provider; provider = provider->next) {
	   if (provider->enumerate)
		  provider->enumerate(urls);
	}

    return urls;
}

void image_source_enumerate_free(zarray_t *urls)
{
    if (urls == NULL)
        return;

    for (int i = 0; zarray_size(urls); i++) {
        char *url;
        zarray_get(urls, i, &url);
        free(url);
    }

    zarray_destroy(urls);
}

void image_source_print_features(image_source_t *isrc)
{
    printf("Features:\n");

    int n = isrc->num_features(isrc);
    for (int i = 0; i < n; i++) {

        const char *name = isrc->get_feature_name(isrc, i);
        const char *type = isrc->get_feature_type(isrc, i);
        double value = isrc->get_feature_value(isrc, i);

        printf("    %-30s : ", name);

        if (type[0] == 'b') {
            printf("Boolean %20s", ((int) value) ? "True" : "False");
        } else if (type[0] == 'i') {

            zarray_t *tokens = str_split(type, ",");

            if (zarray_size(tokens) == 3 || zarray_size(tokens) == 4) {
                char *min = NULL, *max = NULL;
                zarray_get(tokens, 1, min);
                zarray_get(tokens, 2, max);
                char *inc = "1";
                if (zarray_size(tokens) == 4)
                    zarray_get(tokens, 3, inc);

                printf("Int     %20i Min %20s Max %20s Inc %20s", (int) value, min, max, inc);
            }

            zarray_map(tokens, free);
            zarray_destroy(tokens);

        } else if (type[0] == 'f') {

            zarray_t *tokens = str_split(type, ",");

            if (zarray_size(tokens) == 3 || zarray_size(tokens) == 4) {
                char *min = NULL, *max = NULL;
                zarray_get(tokens, 1, min);
                zarray_get(tokens, 2, max);
                char *inc = NULL;
                if (zarray_size(tokens) == 4)
                    zarray_get(tokens, 3, inc);

                printf("Float   %20.15f Min %20s Max %20s", value, min, max);
                if (inc) printf("Inc %20s", inc);
            }

            zarray_map(tokens, free);
            zarray_destroy(tokens);

        } else if (type[0] == 'c') {

            zarray_t *tokens = str_split(type, ",");

            printf("Enum    %20i (", (int) value);

            char *c = NULL;
            for (int i = 1; i < zarray_size(tokens); i++) {
                zarray_get(tokens, i, c);
                printf("%s%s", c, (i+1 == zarray_size(tokens)) ? "" : ", ");
            }

            printf(")");

            zarray_map(tokens, free);
            zarray_destroy(tokens);
        }

        printf("\n");
    }
}

