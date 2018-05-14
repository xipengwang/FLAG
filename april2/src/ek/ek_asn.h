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

#ifndef _EK_ASN_H
#define _EK_ASN_H

#include "common/zarray.h"

typedef struct ek_asn_type ek_asn_type_t;
typedef struct ek_asn_field ek_asn_field_t;

enum { EK_ASN_TYPE_BOOLEAN = 1, EK_ASN_TYPE_INTEGER = 2, EK_ASN_TYPE_BIT_STRING = 3,
       EK_ASN_TYPE_OCTET_STRING = 4, EK_ASN_TYPE_NULL = 5, EK_ASN_TYPE_OBJECT_IDENTIFIER = 6,
       EK_ASN_TYPE_REAL = 9, EK_ASN_TYPE_ENUMERATED = 10, EK_ASN_TYPE_PRINTABLE_STRING = 19,
       EK_ASN_TYPE_T61STRING = 20, EK_ASN_TYPE_IA5STRING = 22, EK_ASN_TYPE_UTCTIME = 23 };

enum { EK_ASN_ERROR_EOF = -1, EK_ASN_ERROR_WRONG_TAG = -2, EK_ASN_ERROR_UNEXPECTED_FIELD = -3 };


typedef struct ek_asn_result ek_asn_result_t;
struct ek_asn_result
{
    uint8_t *data; // a copy of the original buffer passed in.
    int datalen;

    zarray_t *za;
};

// represents a "member" in an EK_ASN record, comprised of a name, type, and modifiers.
struct ek_asn_field
{
    char *name;
    ek_asn_type_t *type;
    int tag;  // if -1, no type specified. otherwise, tag must match.

    int is_optional;
};

// represents an EK_ASN type specification.
struct ek_asn_type
{
    int nfields;
    ek_asn_field_t **fields;

    int is_seq; // if "1", it is a sequence. if "0", then this is a "seq_of" or "set_of"
};

// For every element in an EK_ASN input file, we generate one of these
// structures. NOTE: The 'header' and 'payload' pointers are pointers
// to the originally passed-in buffer.
struct ek_asn_element
{
    const uint8_t *header; // pointer to the EK_ASN header for this record
    int cls;
    int tag;
    int structured;

    const uint8_t *payload; // pointer to the data field for this record
    int payload_len;

    int depth; // how many times is this nested within structures?

    char *fqn; // fully-qualified name

    char *type_name; // e.g. INTEGER. Note that this can be "wrong" if
                     // the field is using an explicit tag type.
};


// Parse a DER file according to the specified EK_ASN type. If the EK_ASN
// type is not NULL and the input data does not match teh
// specification, NULL will be returned and 'error' set
// accordingly. However, 'type' can be NULL, in which case any valid
// EK_ASN data will be parsed.
ek_asn_result_t *ek_asn_result_create_from_der(const void *_buf, int sz, ek_asn_type_t *ek_asn_type, int *error);
void ek_asn_result_print(ek_asn_result_t *res);

// returns non-zero on error
int ek_asn_result_find(const ek_asn_result_t *res, const char *fqn, struct ek_asn_element **_el);

void ek_asn_result_destroy(ek_asn_result_t *res);

//////////////
// create a field that does not require a specific tag value
ek_asn_field_t *ek_asn_field(const char *name, ek_asn_type_t *type);

// create a field that requires a specific tag value [recommended versus ek_asn_field]
ek_asn_field_t *ek_asn_field_tag(const char *name, ek_asn_type_t *type, int tag);

// define a SEQUENCE structure
ek_asn_type_t *ek_asn_seq(ek_asn_field_t** fields);

// define a SEQUENCE OF or SET OF
ek_asn_type_t *ek_asn_seq_of(ek_asn_field_t* field);

// wraps a field inside a structured field with a fixed tag.
// 'name': usually ""
// tag: the explicit tag (for the outer SEQUENCE_OF)
// inner_field: the actual payload, usually with a proper tag.
ek_asn_field_t *ek_asn_explicit_field(const char *name, int tag, ek_asn_field_t *inner_field);

// make a previously-defined field optional. Returns the original pointer.
ek_asn_field_t *ek_asn_make_optional(ek_asn_field_t *field);

ek_asn_type_t *ek_asn_type_create_x509_cacert();

ek_asn_type_t *ek_asn_type_create_x509_privkey();

// within a
ek_asn_type_t *ek_asn_type_create_x509_rsapublic();

#endif
