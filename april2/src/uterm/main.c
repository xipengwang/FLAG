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

#define _XOPEN_SOURCE
#define _XOPEN_SOURCE_EXTENDED

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <arpa/inet.h>
#include <signal.h>
#include "common/serial_util.h"
#include "common/getopt.h"


#ifndef STDIN
#define STDIN 0
#endif

#define MAXLEN 1000

int g_fd = -1;

void do_terminal(getopt_t *gopt, int fd)
{
    struct pollfd pfds[2];

    pfds[0].fd = STDIN;
    pfds[0].events = POLLIN;
    pfds[1].fd = fd;
    pfds[1].events = POLLIN;

    ssize_t len = 1;

    int crlf = getopt_get_bool(gopt, "crlf");
    int hex = getopt_get_bool(gopt, "hex");
    int hexpos = 0;

    while (len > 0) {

        poll(pfds, 2, -1);
        char buf[MAXLEN];

        for (int i = 0; i <= 1; i++)
        {
            int j = 1-i;

            if (pfds[i].revents&POLLIN) {

                if (i == 1 && hex) {
                    if ((hexpos & 0x0f) == 0)
                        printf("%08x : ", hexpos);
                    len = read(pfds[i].fd, buf, 1);
                    printf("%02x ", buf[0] & 0xff);

                    if ((hexpos & 0x0f) == 0x0f)
                        printf("\n");

                    hexpos++;
                    continue;
                }

                if (crlf) {
                    len = read(pfds[i].fd, buf, 1);
                    if (len == 1 && buf[0]=='\n') {
                        if (2 != write(pfds[j].fd, "\r\n", 2))
                            break;
                    } else {
                        if (len != write(pfds[j].fd, buf, len))
                            break;
                    }
                } else {
                    len = read(pfds[i].fd, buf, MAXLEN);
                    if (len != write(pfds[j].fd, buf, len))
                        break;
                }
            }
        }
    }
}

void ctrl_c_func()
{
    printf("CTRL-C\n");

    char buf[1] = { 3 };
    if (g_fd > 0)
        write(g_fd, buf, 1);
}

void ctrl_z_func()
{
    printf("CTRL-Z\n");

    char buf[1] = { 26 };
    if (g_fd > 0)
        write(g_fd, buf, 1);
}

void ctrl_backslash_func()
{
    printf("CTRL-\\\n");

    // restore echo
    struct termios term;
    tcgetattr(STDIN, &term);
    term.c_lflag &= ~ICANON;
    term.c_lflag |=  ECHO;
    tcsetattr(STDIN, TCSANOW, &term);
    setbuf(stdin, NULL);

    exit(0);
}

//////////////////////////////////////////////////////////////////////////
int main(int argc, char *argv[])
{
    getopt_t *gopt = getopt_create();
    getopt_add_bool(gopt,  'h', "help",   0,              "Show this");
    getopt_add_string(gopt,  0, "device", "/dev/ttyUSB0", "Device to connect to");
    getopt_add_bool(gopt,    0, "echo", 0,       "Echo STDIN");
    getopt_add_bool(gopt,    0, "raw", 0,       "Open raw fd (not setting baud, etc)");
    getopt_add_int(gopt,     0, "baud", "115200", "Baud rate");
    getopt_add_bool(gopt,    0, "retry", 1,       "Keep retrying if disconnected");
    getopt_add_string(gopt, 'c', "config", "8N1",  "Data bits, parity, stop bits");
    getopt_add_bool(gopt,    0, "xon-tx", 0,  "Enable XON flow control on output");
    getopt_add_bool(gopt,    0, "xon-rx", 0,  "Enable XON flow control on input");
    getopt_add_bool(gopt,    0, "crlf", 0, "Convert CR -> CRLF");
    getopt_add_bool(gopt,    0, "hex",  0, "Display output in hexidecimal, ala hexdump");
    getopt_add_bool(gopt,    0, "traps", 0, "Trap control-c, control-z, etc.");

    if (!getopt_parse(gopt, argc, argv, 1) || getopt_get_bool(gopt,"help")) {
        getopt_do_usage(gopt);
        return 0;
    }

    if (getopt_get_bool(gopt, "traps")) {
        printf("uterm: Control-C and Control-Z will be passed through. Use Control-\\ to quit.\n");

        sigset(SIGTSTP, ctrl_z_func);
        sigset(SIGINT, ctrl_c_func);
        sigset(SIGQUIT, ctrl_backslash_func);
    } else {
        sigset(SIGINT, ctrl_backslash_func);
    }

    ////////////////// terminal //////////////////
    struct termios term;
    tcgetattr(STDIN, &term);
    term.c_lflag &= ~ICANON;
    if (!getopt_get_bool(gopt, "echo"))
        term.c_lflag &= ~ ECHO;
    tcsetattr(STDIN, TCSANOW, &term);
    setbuf(stdin, NULL);

    while (1) {

        printf("----------------------terminal-----------------------\n");

        ////////////// connect to target board ////////////////
        if (getopt_get_bool(gopt, "raw")) {
            g_fd = open(getopt_get_string(gopt, "device"), O_RDWR | O_NOCTTY);
            serial_set_baud(g_fd, getopt_get_int(gopt, "baud"));
        } else {
            g_fd = serial_open(getopt_get_string(gopt, "device"),
                               getopt_get_int(gopt, "baud"),
                               1);

            struct termios term;
            tcgetattr(g_fd, &term);
            if (getopt_get_bool(gopt, "xon-rx"))
                term.c_iflag |= IXOFF;
            if (getopt_get_bool(gopt, "xon-tx"))
                term.c_iflag |= IXON;
            tcsetattr(g_fd, TCSANOW, &term);

            const char *config = getopt_get_string(gopt, "config");

            int databits = config[0] - '0';
            int parity = -1;
            switch (config[1]) {
            case 'N':
                parity = 0; break;
            case 'O':
                parity = 1; break;
            case 'E':
                parity = 2; break;
            }
            int stopbits = config[2] - '0';

            if (databits < 5 || databits > 8 || parity < 0 || parity > 2 || stopbits < 1 || stopbits > 2) {
                printf("Invalid configuration mode. Examples: 8N1, 7E2\n");
                return -1;
            }

            serial_set_mode(g_fd, databits, parity, stopbits);
        }

        if (g_fd < 0)
            perror(getopt_get_string(gopt, "device"));

        if (g_fd >= 0)
            do_terminal(gopt, g_fd);

        close(g_fd);

        if (!getopt_get_bool(gopt, "retry"))
            break;

        sleep(1);
    }

    printf("\n");
//    exitfunc();
    return 0;
}
