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

#ifndef _SHA1_H
#define _SHA1_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    unsigned long int total[2];
    unsigned long int state[5];
    unsigned char buffer[64];
}
sha1_context;

/*
 * Core SHA-1 functions
 */
void sha1_starts( sha1_context *ctx );
void sha1_update( sha1_context *ctx, const void *input, unsigned int length );
void sha1_finish( sha1_context *ctx, unsigned char digest[20] );

/*
 * Output SHA-1(file contents), returns 0 if successful.
 */
int sha1_file( char *filename, unsigned char digest[20] );

/*
 * Output SHA-1(buf)
 */
void sha1_csum( unsigned char *buf, unsigned int buflen, unsigned char digest[20] );

/*
 * Output HMAC-SHA-1(key,buf)
 */
void sha1_hmac( unsigned char *key, unsigned int keylen, unsigned char *buf, unsigned int buflen,
                unsigned char digest[20] );

/*
 * Checkup routine
 */
int sha1_self_test( void );

#ifdef __cplusplus
}
#endif

#endif /* sha1.h */
