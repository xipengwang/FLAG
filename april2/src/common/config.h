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

#ifndef _CONFIG_H
#define _CONFIG_H

// A configuration file parser that supports block-level organization
// of key-value pairs, along with inheritance and easy auto-numbering
// of sections.
//
// ** Key concept **
//
//    A config file ultimately produces a set of key/value pairs. The
//    keys can be thought of as being hierarchically scoped, e.g.:
//
//    laser.position.quaternion = [1 0 0 0];
//
//    Commas can be included for legibility but are otherwise meaningless.
//
//    laser.position.quaterion = [1, 0, 0, 0 ];
//
// ** Key concept **
//
//    The special syntaxes supported by this config file parser are
//    simply syntactic sugar for listing fully-qualified key/value
//    pairs individually.
//
// ** Key concept **
//
//    The config file supports only one value type: a list of
//    strings. All other interpretations of data are done
//    on-demand. Even if data appears to be numeric in the file (see
//    laser.position.quaternion example above), it is internally
//    represented as a string. Quotation marks are necessary only when
//    a single value contains whitespace or special characters.
//
//    Use of square brackets is required when representing multiple
//    values.
//
//    intro = "it was a dark and stormy night"; <-- 1 value.
//
//    intro = [ it was a dark and stormy night ]; <-- 7 values
//
//    intro = it was a dark and stormy night; <-- syntax error
//
// Note also that whitespace (such as indentation) is meaningless.
//
// =========================================================
// Block-based scoping and naming
//
// Key/value pairs given within a scoped block are prefixed by the
// namespace of their enclosing blocks.
//
// Example:
//
// zoo {
//   name = "Detroit Zoo";
//   monkey-house {
//      occupant = "rhesus";
//   }
// }
//
// Is *exactly* equivalent to:
//
// zoo.name = "Detroit Zoo";
// zoo.monkey-house.occupant = "rhesus";
//
// =========================================================
// Named Inheritance
//
// Often many key/value pairs are repeated. This config file format
// allows oft-repeated values to be specified only once, increasing
// readability and avoiding inconsistencies and typos.
//
// :zoo-building {
//   cleaning-schedule = [ "Monday" "Wednesday" "Friday" ];
// }
//
// zoo {
//   monkey-house : zoo-building {
//      occupant = "rhesus";
//   }
//   reptile-house : zoo-building {
//      occupant = "monitor";
//   }
//   aquatic-house {
//      occupant = "dolphin";
//   }
//   visitor-center {
//      occupant = "human";
//      cleaning-schedule = [ "Saturday", "Sunday" ];
// }
//
// Is *exactly* equivalent to:
//
// zoo.monkey-house.occupant = "rhesus";
// zoo.monkey-house.cleaning-schedule = [ "Monday" "Wednesday" "Friday" ];
// zoo.reptile-house.occupant = "monitor";
// zoo.reptile-house.cleaning-schedule = [ "Monday" "Wednesday" "Friday" ];
// zoo.aquatic-house.occupant = "dolphin";
// zoo.visitor-center.occupant = "human";
// zoo.visitor-center.cleaning-schedule = [ "Saturday" "Sunday" ];
//
// Comments: inheritance must be explicitly listed (note: no
// "aquatic-house.cleaning-schedule" key.) It can also be over-ridden
// (see visitor-house.cleaning-schedule).
//
// This example also makes use of an "abstract" base class
// ":zoo-building". The leading colon prevents "zoo-building" from
// becoming a top-level key/value pair. Without the colon, we would also have:
//
// zoo-building.cleaning-schedule = [ "Monday" "Wednesday" "Friday" ];
//
// (In the current implementation, the key
// :zoo-building.cleaning-schedule is actually created and visible,
// however, the presence of the leading colon makes it easy to tell
// that it is not a "proper" key/value pair.)
//
// =========================================================
// Unique numbering
//
// It is sometimes useful to organize information into multiple blocks
// such that those blocks can be iterated over. (procman is a prime
// example--- we want to iterate over process descriptions). However, the
// names of those blocks isn't important, so long as they're unique.
//
// A '#' added to the end of block name will be replaced with a unique
// identifier.
//
// :mammal {
//     birth = "live";
// }
//
// :reptile {
//   birth = "egg";
// }
//
// animal# : mammal {
//    name = "dolphin";
// }
//
// animal# : reptile {
//    name = "monitor";
// }
//
// Is equivalent to:
//
// animal0.birth = "live";
// animal0.name = "dolphin";
// animal1.birth = "egg";
// animal1.name = "monitor";

#include "mati.h"
#include "matd.h"
#include "zarray.h"

#ifdef __cplusplus
extern "C" {
#endif

#define CONFIG_INCLUDE_KEYWORD "include"

typedef struct config config_t;

config_t *config_create(void);

// convenience method: equiv to _create, _import_path(path)
config_t *config_create_path(const char *path);

// returns non-zero on error
int config_import_path(config_t *config, const char *path);

void config_destroy(config_t *config);
void config_debug(const config_t *config);

// silently prepend the string "prefix" to all future get/require
// methods. Can set to NULL to clear the prefix. No dot is implicitly
// added; most users will thus want a value like "parent.".
void config_set_prefix(config_t *config, const char *prefix);

////////////////////////////////////////
// public access functions

// NOTE: All zarray_t, char*, mati_t, and matd_t objects are owned by the
// config and freed on config_destroy(). On first access, all zarray_t,
// mati_t, and matd_t objects are cached for later reuse. Scalar types
// are not cached and are re-interpreted on every access.

const zarray_t   *config_get_keys         (config_t * conf);
int               config_has_key          (const config_t * conf, const char * key);

const char       *config_require_string   (const config_t * conf, const char * key);
int               config_require_boolean  (const config_t * conf, const char * key);
int               config_require_int      (const config_t * conf, const char * key);
double            config_require_double   (const config_t * conf, const char * key);

const char       *config_get_string       (const config_t * conf, const char * key, const char * def);
int               config_get_boolean      (const config_t * conf, const char * key, int def);
int               config_get_int          (const config_t * conf, const char * key, int def);
double            config_get_double       (const config_t * conf, const char * key, double def);

const zarray_t   *config_require_strings  (const config_t * conf, const char * key);
const zarray_t   *config_require_booleans (const config_t * conf, const char * key);
const zarray_t   *config_require_ints     (const config_t * conf, const char * key);
const zarray_t   *config_require_doubles  (const config_t * conf, const char * key);
void              config_require_doubles_len(const config_t * conf, const char * key, double *vs, int len);

const zarray_t   *config_get_strings      (const config_t * conf, const char * key, const zarray_t * def);
const zarray_t   *config_get_booleans     (const config_t * conf, const char * key, const zarray_t * def);
const zarray_t   *config_get_ints         (const config_t * conf, const char * key, const zarray_t * def);
const zarray_t   *config_get_doubles      (const config_t * conf, const char * key, const zarray_t * def);

const mati_t     *config_require_mati     (const config_t * conf, const char * key);
const matd_t     *config_require_matd     (const config_t * conf, const char * key);

const mati_t     *config_get_mati         (const config_t * conf, const char * key, const mati_t * def);
const matd_t     *config_get_matd         (const config_t * conf, const char * key, const matd_t * def);

#ifdef __cplusplus
}
#endif

#endif
