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
#include <stdint.h>
#include <assert.h>
#include <string.h>

#include "ek.h"
#include "ek_asn.h"
#include "common/string_util.h"
#include "common/io_util.h"

// returns zero on success.
static int ek_asn_decode_der_r(const void *_buf, int sz, int *pos, int depth, ek_asn_type_t *type,
                            const char *prefix, zarray_t *za)
{
    const uint8_t *buf = _buf;
    int field_idx = 0; // as we parse through a 'structured', which field are we handling?

    int set_size = 0; // if we're handling a 'set of' or 'sequence of', how many elements have we processed?

    while (*pos < sz) {

        const uint8_t *header = &buf[*pos];

        if (type && field_idx >= type->nfields) {
            printf("ek_asn_field_idx: %d, nfields: %d\n", field_idx, type->nfields);
            return EK_ASN_ERROR_UNEXPECTED_FIELD;
        }

        uint8_t cls = buf[*pos] >> 6;
        uint8_t structured = (buf[*pos] & 0x20) ? 1 : 0;
        int tag = buf[*pos] & 0x1f;
        (*pos)++;

        if (tag == 0x1f) {
            printf("EXTENDED\n");
            // extended tag
            tag = 0;

            do {
                tag = (tag << 7) | (buf[*pos] & 0x7f);
                (*pos)++;
            } while (*pos < sz && buf[*pos] & 0x80);

            // not handled
            assert(0);
        }

        int len = buf[*pos];
        (*pos)++;

        if (len == 128) {
            assert(0); // does this mean length = 128?
        } else if (len > 128) {
            int nbytes = len - 128;

            if (*pos + nbytes >= sz)
                return EK_ASN_ERROR_EOF;

            len = 0;
            for (int i = 0; *pos < sz && i < nbytes; i++) {
                len = (len << 8) + buf[*pos];
                (*pos)++;
            }
        }

        if ((*pos) + len > sz) {
            return EK_ASN_ERROR_EOF;
        }


        ek_asn_field_t *field = type ? type->fields[field_idx] : NULL;

        // if this field is optional and doesn't match, search ahead.
        while (field &&
               field->is_optional &&
               field->tag >= 0 && field->tag != tag &&
               field_idx + 1 < type->nfields) {
            field_idx++;
            field = type->fields[field_idx];
        }

        // It's an error if the tag is specified and it doesn't match.
        if (field &&
            field->tag >= 0 && field->tag != tag) {

            return EK_ASN_ERROR_WRONG_TAG;
        }

        // What's the name for this field? Will be deallocated in a moment...
        char *name = NULL;
        if (type && field_idx < type->nfields)
            name = strdup(type->fields[field_idx]->name);

        // use the tag number as a name
        if (name == NULL)
            name = sprintf_alloc("%d", tag);

        // we unow create a fully-qualified name (a "key") for this field.
        char *key;
        if (1) {
            if (tag == 17 || (type && !type->is_seq)) {
                // for sets, add brackets to count the elements.
                if (!prefix || prefix[0] == 0)
                    key = sprintf_alloc("%s[%d]", name, set_size);
                else if (prefix[strlen(prefix)-1] == '.')
                    key = sprintf_alloc("%s%s[%d]", prefix, name, set_size);
                else
                    key = sprintf_alloc("%s.%s[%d]", prefix, name, set_size);
                set_size++;
            } else {
                if (!prefix || prefix[0] == 0)
                    key = sprintf_alloc("%s", name);
                else if (prefix[strlen(prefix) - 1] == '.')
                    key = sprintf_alloc("%s%s", prefix, name);
                else
                    key = sprintf_alloc("%s.%s", prefix, name);
            }
        }

        struct ek_asn_element el = { .header = header,
                                  .cls = cls,
                                  .tag = tag,
                                  .structured = structured,
                                  .depth = depth,
                                  .payload = &buf[*pos],
                                  .payload_len = len,
                                  .fqn = key };

        switch (tag) {
            case EK_ASN_TYPE_BOOLEAN:
                el.type_name = strdup("BOOLEAN");
                break;
            case EK_ASN_TYPE_INTEGER:
                el.type_name = strdup("INTEGER");
                break;
            case EK_ASN_TYPE_BIT_STRING:
                el.type_name = strdup("BIT STRING");

                // the first octet is the number of padding bits required to form
                // an integer number of octets. HOWEVER: we cannot safely remove/adjust
                // here because specific EK_ASN types "reuse" the BIT STRING type... this
                // many not actually be a BIT STRING. (example: tbsCertificate)

/*                if (el.payload[0] != 0) {
                    // unhandled padding!
                    ek_print_buffer(el.payload, el.payload_len);

                    printf("ek_ek_asn BIT_STRING non-zero padding (%d)\n", el.payload[0]);
                } else {
                    printf("ek_ek_asn BIT_STRING zero padding\n");
                }


                el.payload++;
                el.payload_len--;
*/
                break;
            case EK_ASN_TYPE_OCTET_STRING:
                 el.type_name = strdup("OCTET STRING");
                break;
            case EK_ASN_TYPE_NULL:
                 el.type_name = strdup("NULL");
                break;
            case EK_ASN_TYPE_OBJECT_IDENTIFIER:
                 el.type_name = strdup("OBJECT IDENTIFIER");
                break;
            case 16:
                 el.type_name = strdup("SEQUENCE/SEQUENCE OF");
                break;
            case 17:
                 el.type_name = strdup("SET/SET OF");
                break;
            case EK_ASN_TYPE_PRINTABLE_STRING:
                 el.type_name = strdup("PrintableString");
                break;
            case EK_ASN_TYPE_T61STRING:
                 el.type_name = strdup("T61String");
                break;
            case EK_ASN_TYPE_IA5STRING:
                 el.type_name = strdup("IA5String");
                break;
            case EK_ASN_TYPE_UTCTIME:
                 el.type_name = strdup("UTCTime");
                break;
            default:
                el.type_name = sprintf_alloc("TAG-0x%04x", tag);
                break;
        }

        zarray_add(za, &el);

        free(name);

        // recurse on structured elements
        if (structured) {
            int newpos = *pos;

            int res = ek_asn_decode_der_r(buf, *pos + len, &newpos, depth + 1, field ? field->type : NULL, key, za);

            if (res)
                return res;
        }

        if (type && type->is_seq)
            field_idx++;

        *pos += len;
    }

    // XXX make sure there are no non-optional fields that have not yet been filled
    while (type && type->is_seq && field_idx < type->nfields) {
        if (!type->fields[field_idx]->is_optional)
            return EK_ASN_ERROR_EOF;
        field_idx++;
    }

    return 0;
}

void ek_asn_result_destroy(ek_asn_result_t *res)
{
    if (!res)
        return;

    for (int i = 0; i < zarray_size(res->za); i++) {
        struct ek_asn_element *el;
        zarray_get_volatile(res->za, i, &el);

        free(el->fqn);
        free(el->type_name);
    }

    zarray_destroy(res->za);
    free(res->data);
    free(res);
}

void ek_asn_result_print(ek_asn_result_t *res)
{
    for (int i = 0; i < zarray_size(res->za); i++) {
        struct ek_asn_element *el;
        zarray_get_volatile(res->za, i, &el);

        printf("%04x: ", (int) (el->header - res->data));
        for (int i = 0; i < el->depth; i++)
            printf("  ");

        printf("cls 0x%02x, tag 0x%02x: %s  [%s]\n", el->cls, el->tag, el->fqn, el->type_name);

        if (!el->structured)
            ek_print_buffer_indent(el->payload, el->payload_len, el->depth + 4, "  ");
    }
}

// Parse a DER file according to the specified EK_ASN type. If the EK_ASN
// type is not NULL and the input data does not match teh
// specification, NULL will be returned and 'error' set
// accordingly. However, 'type' can be NULL, in which case any valid
// EK_ASN data will be parsed.
ek_asn_result_t *ek_asn_result_create_from_der(const void *_buf, int sz, ek_asn_type_t *ek_asn_type, int *error)
{
    const uint8_t *buf = _buf;

    ek_asn_result_t *res = calloc(1, sizeof(ek_asn_result_t));
    res->data = malloc(sz);
    memcpy(res->data, buf, sz);
    res->datalen = sz;
    res->za = zarray_create(sizeof(struct ek_asn_element));

    int pos = 0;
    *error = ek_asn_decode_der_r(buf, sz, &pos, 0, ek_asn_type, "", res->za);

    if (*error) {
        ek_asn_result_destroy(res);
        return NULL;
    }

    return res;
}

ek_asn_result_t *ek_asn_result_create_from_der_file(const char *der_path, ek_asn_type_t *ek_asn_type, int *error)
{
    uint8_t *der;
    long der_len;

    if (ioutils_read_file(der_path, &der, &der_len, -1)) {
        return NULL;
    }

    ek_asn_result_t *res = calloc(1, sizeof(ek_asn_result_t));
    res->data = der;
    res->datalen = der_len;
    res->za = zarray_create(sizeof(struct ek_asn_element));

    int pos = 0;
    *error = ek_asn_decode_der_r(der, der_len, &pos, 0, ek_asn_type, "", res->za);

    if (*error) {
        ek_asn_result_destroy(res);
        return NULL;
    }

    return res;
}

ek_asn_field_t *ek_asn_field(const char *name, ek_asn_type_t *type)
{
    ek_asn_field_t *v = calloc(1, sizeof(ek_asn_field_t));
    v->name = strdup(name);
    v->type = type;
    v->tag = -1;
    return v;
}

ek_asn_field_t *ek_asn_field_tag(const char *name, ek_asn_type_t *type, int tag)
{
    ek_asn_field_t *v = calloc(1, sizeof(ek_asn_field_t));
    v->name = strdup(name);
    v->type = type;
    v->tag = tag;
    return v;
}

ek_asn_type_t *ek_asn_seq(ek_asn_field_t** fields)
{
    ek_asn_type_t *v = calloc(1, sizeof(ek_asn_type_t));

    if (fields) {
        while (fields[v->nfields] != NULL)
            v->nfields++;

        v->fields = calloc(v->nfields, sizeof(ek_asn_field_t*));
        for (int i = 0; i < v->nfields; i++)
            v->fields[i] = fields[i];
    }

    v->is_seq = 1;

    return v;
}

ek_asn_type_t *ek_asn_seq_of(ek_asn_field_t* field)
{
    ek_asn_type_t *v = calloc(1, sizeof(ek_asn_type_t));
    v->fields = calloc(1, sizeof(ek_asn_field_t*));
    v->fields[0] = field;
    v->nfields = 1;

    return v;
}

// wraps a field inside a structured field with a fixed tag.
// 'name': usually ""
// tag: the explicit tag (for the outer SEQUENCE_OF)
// inner_field: the actual payload, usually with a proper tag.
ek_asn_field_t *ek_asn_explicit_field(const char *name, int tag, ek_asn_field_t *inner_field)
{
    ek_asn_field_t *field = ek_asn_field("", ek_asn_seq((ek_asn_field_t*[]) { inner_field, NULL }));
    field->tag = tag;
    return field;
}

ek_asn_field_t *ek_asn_make_optional(ek_asn_field_t *field)
{
    field->is_optional = 1;
    return field;
}

int ek_asn_result_find(const ek_asn_result_t *res, const char *fqn, struct ek_asn_element **_el)
{
    for (int i = 0; i < zarray_size(res->za); i++) {
        struct ek_asn_element *el;
        zarray_get_volatile(res->za, i, &el);

        if (!strcmp(el->fqn, fqn)) {
            *_el = el;
            return 0;
        }
    }

    return -1;
}

ek_asn_type_t *ek_asn_type_create_x509_cacert()
{
    ek_asn_type_t *name = ek_asn_seq_of(ek_asn_field("rdnSequence", ek_asn_seq_of(
                                                ek_asn_field("attributeTypeAndValue", ek_asn_seq((ek_asn_field_t*[]) {
                                                            ek_asn_field("type", NULL),
                                                                ek_asn_field("value", NULL),
                                                                NULL })))));

    ek_asn_type_t *extension = ek_asn_seq((ek_asn_field_t*[]) {
            ek_asn_field_tag("objid", NULL, EK_ASN_TYPE_OBJECT_IDENTIFIER),
                ek_asn_make_optional(ek_asn_field_tag("critical", NULL, EK_ASN_TYPE_BOOLEAN)),
                ek_asn_field_tag("data", NULL, EK_ASN_TYPE_OCTET_STRING),
                NULL });

    ek_asn_type_t *tbscert = ek_asn_seq((ek_asn_field_t*[]) {
            ek_asn_explicit_field("", 0, ek_asn_field_tag("version", NULL, EK_ASN_TYPE_INTEGER)),
                ek_asn_field_tag("serialNumber", NULL, EK_ASN_TYPE_INTEGER),
                ek_asn_field("signature", NULL),
                ek_asn_field("issuer", name),
                ek_asn_field("validity", ek_asn_seq((ek_asn_field_t*[]) {
                            ek_asn_field_tag("not_before", NULL, EK_ASN_TYPE_UTCTIME),
                                ek_asn_field_tag("not_after", NULL, EK_ASN_TYPE_UTCTIME),
                                NULL })),
                ek_asn_field("subject", name),
                ek_asn_field("subjectPublicKeyInfo", ek_asn_seq((ek_asn_field_t*[]) {
                                ek_asn_field("algorithmIdentifier", NULL),
                                ek_asn_field("subjectPublicKey", NULL),
                                NULL })),
                ek_asn_make_optional(ek_asn_explicit_field("", 1, ek_asn_field("issuerUniqueID", NULL))),
                ek_asn_make_optional(ek_asn_explicit_field("", 2, ek_asn_field("subjectUniqueID", NULL))),
                ek_asn_make_optional(ek_asn_explicit_field("", 3, ek_asn_field("extensions", ek_asn_seq_of(ek_asn_field("extension", extension))))),
                NULL });

    ek_asn_type_t *cert = ek_asn_seq((ek_asn_field_t*[]) {
                                    ek_asn_field("tbsCertificate", tbscert),
                                        ek_asn_field("signatureAlgorithm", NULL),
                                        ek_asn_field("signatureValue", NULL),
                                        NULL });

    ek_asn_type_t *certfile = ek_asn_seq_of(ek_asn_field("certificate", cert));

    return certfile;
}

ek_asn_type_t *ek_asn_type_create_x509_privkey()
{
    ek_asn_type_t *privkey = ek_asn_seq((ek_asn_field_t*[]) {
                ek_asn_field_tag("version", NULL, EK_ASN_TYPE_INTEGER),
                ek_asn_field_tag("modulus", NULL, EK_ASN_TYPE_INTEGER),
                ek_asn_field_tag("publicExponent", NULL, EK_ASN_TYPE_INTEGER),
                ek_asn_field_tag("privateExponent", NULL, EK_ASN_TYPE_INTEGER),
                ek_asn_field_tag("prime1", NULL, EK_ASN_TYPE_INTEGER),
                ek_asn_field_tag("prime2", NULL, EK_ASN_TYPE_INTEGER),
                ek_asn_field_tag("exponent1", NULL, EK_ASN_TYPE_INTEGER),
                ek_asn_field_tag("exponent2", NULL, EK_ASN_TYPE_INTEGER),
                ek_asn_field_tag("coefficient", NULL, EK_ASN_TYPE_INTEGER),
                NULL });

    ek_asn_type_t *privkeyfile = ek_asn_seq((ek_asn_field_t*[]) { ek_asn_field("privkey", privkey),
                NULL });

    return privkeyfile;
}

ek_asn_type_t *ek_asn_type_create_x509_rsapublic()
{
    ek_asn_type_t *pubkey = ek_asn_seq((ek_asn_field_t*[]) {
                ek_asn_field_tag("modulus", NULL, EK_ASN_TYPE_INTEGER),
                ek_asn_field_tag("publicExponent", NULL, EK_ASN_TYPE_INTEGER),
                NULL });

    ek_asn_type_t *pubkeyfile = ek_asn_seq((ek_asn_field_t*[]) { ek_asn_field("pubkey", pubkey),
                NULL });

    return pubkeyfile;
}
