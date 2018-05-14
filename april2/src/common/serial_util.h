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

#ifndef _SERIAL_H
#define _SERIAL_H

/** Creates a basic fd, setting baud to 9600, raw data i/o (no flow
    control, no fancy character handling. Configures it for blocking
    reads.  8 data bits, 1 stop bit, no parity.

    Returns the fd, -1 on error
**/
int serial_open(const char *port, int baud, int blocking);

/** Set the communication mode. Returns -1 on error. **/
int serial_set_mode(int fd, int databits, int parity, int stopbits);

/** Set the baud rate, where the baudrate is just the integer value
    desired.

    Returns non-zero on error.
**/
int serial_set_baud(int fd, int baudrate);

/** Enable cts/rts flow control.
    Returns non-zero on error.
**/
int serial_enablectsrts(int fd);
/** Enable xon/xoff flow control.
    Returns non-zero on error.
**/
int serial_enablexon(int fd);

/** Set the port to 8 data bits, 2 stop bits, no parity.
    Returns non-zero on error.
 **/
int serial_set_N82 (int fd);

int serial_close(int fd);

#endif
