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

#include <math.h>
#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#include <limits.h>
#include <stdlib.h>
#include <inttypes.h>

#include "common/getopt.h"
#include "common/string_util.h"
#include "lcmtypes/string_t.h"
#include "lcmtypes/gps_t.h"
#include "lcmtypes/ubx_t.h"
#include "lcm/lcm.h"

#include "ublox.h"

static lcm_t *lcm;
static uint8_t verbose = 0;
static int32_t nMsgs = 0;

// TODO: The packet processing code depends on the platform being little-
// endian, as it does no byte order conversion
#if __BYTE_ORDER__ != __ORDER_LITTLE_ENDIAN__
#error "This code expects a little-endian platform"
#endif

typedef struct {
    uint16_t pending[6];
    uint8_t usage[6];
    uint8_t peakUsage[6];
    uint8_t tUsage;
    uint8_t tPeakUsage;
    uint8_t errors;
    uint8_t reserved;
} ubx_mon_txbuf_t;

// read 32 bits, but discard last 8 bits.
uint32_t decode_sfw(const uint8_t *in, uint32_t *inpos, uint32_t inlen)
{
    if ((*inpos) + 4 > inlen)
        return 0;

    uint32_t v = 0;
    v += in[(*inpos)++] << 16;
    v += in[(*inpos)++] << 8;
    v += in[(*inpos)++] << 0;
//    v += in[(*inpos)++] << 0;
    (*inpos)++;

    return v;
}

void print_buffer(uint8_t *buf, int len)
{
    int cols = 16;

    for (int i = 0; i < len; i++) {
        if ((i % cols) == 0)
            printf("%04x: ", i);
        printf("%02x ", buf[i]);
        if ((i % cols) == (cols - 1))
            printf("\n");
    }
    printf("\n");
}

void ubx_callback(int64_t time, ublox_packet_t *packet)
{
    nMsgs += 1;

    // Handle MON-TXBUF status messages
    if (packet->class == 0x0A && packet->id == 0x08) {
	// XXX This depends on the platform being little-endian
        ubx_mon_txbuf_t *data = (ubx_mon_txbuf_t*)(packet->payload);
        // Port index 3 is USB
        printf("[%11l" PRId64 ".%06" PRId64 "] ublox  msgs: %-7d ",
               (long long)(time/1000000), (long)(time%1000000), nMsgs);
        printf("txbuf: %-4d usage: %3d%%  errors: %x\n",
               data->pending[3], data->usage[3], data->errors);
    }
    // Print NAK messages
    else if (packet->class == 0x05 && packet->id == 0x00) {
        printf("Warning: NAK received for CFG message %02x %02x\n",
               packet->payload[0], packet->payload[1]);
    } else {
	// Print unknown messages
        if (verbose)
            ublox_packet_print(packet);

        // Publish everything
        ubx_t raw;
        raw.utime = time;
        raw.cls = packet->class;
        raw.id = packet->id;
        raw.len = packet->length;
        raw.data = packet->payload;
        ubx_t_publish(lcm, "UBX", &raw);

        if (raw.cls == 0x02 && raw.id == 0x30) {
            // RXM-RAW
//            printf("RXM-ALM size %d\n", raw.len);
        }
        if (raw.cls == 0x02 && raw.id == 0x31) {
            // RXM-RAW
//            printf("RXM-EPH size %d\n", raw.len);
        }
        if (raw.cls == 0x02 && raw.id == 0x10) {
            // RXM-RAW
//            printf("RXM-RAW size %d\n", raw.len);
        }
        if (raw.cls == 0x02 && raw.id == 0x11) {
            // RXM-SRFB
            printf("RXM-SFRB size %d, channel %d, sv %d\n", raw.len, raw.data[0], raw.data[1]);

            uint32_t rawpos = 2;
            uint32_t tlm = decode_sfw(raw.data, &rawpos, raw.len);
            uint32_t how = decode_sfw(raw.data, &rawpos, raw.len);

            printf("%08x %08x subframe %d\n", tlm, how, (how >> 21) & 7);
            print_buffer(&raw.data[2], raw.len - 2);
        }
    }
}
int verify_checksum(const char *msg)
{
    int idx = 1;
    char check = 0;

    while(true){
        if(msg[idx] == '\0')
            return false;
        if(msg[idx] == '*')
            break;
        check ^= msg[idx++];
    }
    idx++;
    int sum = strtol(&msg[idx], NULL, 16);

    return check == sum;

}

int64_t hhmmss_ss_to_utime(double time)
{
    int64_t utime = 0;

    int itime = ((int) time);

    double seconds = fmod(time, 100.0);
    uint8_t minutes = (itime % 10000) / 100;
    uint8_t hours =  itime / 10000;

    utime += seconds *   100;
    utime += minutes *  6000;
    utime += hours   *360000;

    utime *= 10000;

    return utime;
}

double ddmm_to_d(double ddmm)
{
    double degrees = ((int) ddmm) / 100;
    degrees += fmod(ddmm, 100) / 60.0;
    return degrees;
}

void nmea_to_gps_t(int64_t time, const char *msg)
{
    if(!str_starts_with(msg, "$PUBX,00,"))
        return;
    if(!verify_checksum(msg))
        return;

    gps_t gps;
    memset(&gps, 0, sizeof(gps));
    gps.host_utime = time;


    char *idx = strchr(msg, ',')+1; //Past PUBX
    if(idx == NULL+1) return;
    idx = strchr(idx, ',')+1; // past 00 msg type
    if(idx == NULL+1) return;
    gps.utc_utime = hhmmss_ss_to_utime(strtod(idx, &idx));
    idx++;
    gps.lat = ddmm_to_d(strtod(idx, &idx));
    idx++;
    if(*idx == 'S')
        gps.lat *= -1;
    else if(*idx != 'N')
        return;
    idx += 2;
    gps.lon = ddmm_to_d(strtod(idx, &idx));
    idx++;
    if(*idx == 'W')
        gps.lon *= -1;
    else if(*idx != 'E')
        return;
    idx += 2;
    gps.elevation = strtod(idx, &idx);
    idx++;
    if(str_starts_with(idx, "D3"))
        gps.status = GPS_T_GPS_STATUS_DGPS_LOCK;
    else if(str_starts_with(idx, "G3"))
        gps.status = GPS_T_GPS_STATUS_LOCK;
    else if(str_starts_with(idx, "NF"))
        gps.status = GPS_T_GPS_STATUS_NO_LOCK;
    else
        gps.status = GPS_T_GPS_STATUS_ERROR;
    idx += 3;
    gps.err_z = strtod(idx, &idx);
    idx++;
    gps.err_x = strtod(idx, &idx);
    gps.err_y = gps.err_x;
    idx++;
    strtod(idx, &idx); //SOG
    idx++;
    strtod(idx, &idx); //COG
    idx++;
    strtod(idx, &idx); //Vert vel.
    idx++;
    strtod(idx, &idx); //correction age
    idx++;
    gps.horiz_dop = strtod(idx, &idx);
    idx++;
    strtod(idx, &idx); //VDOP
    idx++;
    gps.time_dop = strtod(idx, &idx);
    idx++;
    gps.nsats = strtol(idx, &idx, 10);

    gps_t_publish(lcm, "GPS", &gps);
}

void nmea_callback(int64_t time, const char *msg)
{
    nMsgs += 1;

    string_t nm;
    nm.utime = time;
    nm.data = (char*)msg;

    string_t_publish(lcm, "NMEA", &nm);

    nmea_to_gps_t(time, msg);

    if (verbose)
        printf("%s\n", msg);
}

// Set measurement rate (ublox 6T supports up to 5Hz, or 200ms)
void set_measurement_rate(ublox_t *ub, uint16_t rate_ms)
{
    struct {
        uint16_t measRate;
        uint16_t navRate;
        uint16_t timeRef;
    } s;
    s.measRate = rate_ms;
    s.navRate = 1;
    s.timeRef = 1;
    ublox_command(ub, 0x06, 0x08, 6, (uint8_t*)&s);
}

// Rate is relative to the measurement rate;
// 1 = one message every measurement, 2 = one message every 2 measurements
void set_msg_rate(ublox_t *ub, uint8_t class, uint8_t id, uint8_t rate)
{
    uint8_t data[] = {class, id, rate};
    ublox_command(ub, 0x06, 0x01, 3, data);
}

void set_nav_engine(ublox_t *ub, uint8_t dynmodel)
{
  // These are the defaults except dynmodel (data[2])
  uint8_t data[] = {0xff, 0xff, dynmodel, 0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x05, 0x00, 0xfa, 0x00, 0xfa, 0x00, 0x64, 0x00, 0x2c, 0x01, 0x00, 0x3c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  ublox_command(ub, 0x06, 0x24, 36, data);
}

// Enable or disable the UART ports (portID=1 or 2)
void configure_uart(ublox_t *ub, uint8_t portId, uint8_t enable)
{
    struct {
        uint8_t portId;
        uint8_t reserved0;
        uint16_t txReady;
        uint32_t mode;
        uint32_t baudRate;
        uint16_t inProtoMask;
        uint16_t outProtoMask;
        uint16_t reserved4;
        uint16_t reserved5;
    } data = {0};

    data.portId = portId;
    data.mode = 0x8D0; // 8 data bits, no parity, 1 stop bit
    data.baudRate = 115200;
    // enable/disable both NMEA and UBX protocols
    data.inProtoMask = data.outProtoMask = enable ? 0x3 : 0;
    ublox_command(ub, 0x06, 0x00, 20, (uint8_t*)&data);
}

int main (int argc, char *argv[])
{
    getopt_t *gopt;
    ublox_t *ub;

    setlinebuf(stdout);

    gopt = getopt_create();
    getopt_add_string(gopt, 'd', "device", "/dev/ttyACM0",
                      "GPS serial device");
    getopt_add_int(gopt, 'b', "baud", "115200", "Serial baud rate");
    getopt_add_int(gopt, 'h', "hz", "5", "Navigation update rate in Hz");
    getopt_add_bool(gopt, 'v', "verbose", 0, "Enable verbose output");
    getopt_add_bool(gopt, 'r', "raw", 1, "Enable raw output");

    if (!getopt_parse(gopt, argc, argv, 1)) {
        getopt_do_usage(gopt);
        return 1;
    }

    const char *port = getopt_get_string(gopt, "device");
    verbose = getopt_get_bool(gopt, "verbose");
    int hz = getopt_get_int(gopt, "hz");

    lcm = lcm_create(NULL);

    char device[PATH_MAX];
    if (realpath(port, device) == NULL) {
        printf("Error resolving real device path (e.g. following symlinks) for '%s'\n", port);
        return 1;
    }

    printf("Resolved '%s' to device '%s'\n", port, device);

    ub = ublox_create(device , getopt_get_int(gopt, "baud"));
    if (ub == NULL) {
        printf("Error opening device %s\n", device);
        return 1;
    }
    ublox_set_ubx_callback(ub, ubx_callback);
    ublox_set_nmea_callback(ub, nmea_callback);

    // Disable UART (port 1)
    configure_uart(ub, 1, 0);

    // Set nav engine settings to portable
    set_nav_engine(ub, 0);
    // Set nav engine settings to airborne 4g (less filtering)
    //set_nav_engine(ub, 8);
    ublox_command(ub, 0x06, 0x24, 0, NULL);

    // By default enabled: GGA, GLL, GSA, GSV, RMC, VTG, TXT
    // Disable GGA
    set_msg_rate(ub, 0xF0, 0x00, 0);
    // Disable GLL
    set_msg_rate(ub, 0xF0, 0x01, 0);
    // Disable VTG
    set_msg_rate(ub, 0xF0, 0x05, 0);
    // Disable GSV
    set_msg_rate(ub, 0xF0, 0x03, 0);
    // Disable ZDA
    set_msg_rate(ub, 0xF0, 0x08, 0);
    // Disable GSA
    set_msg_rate(ub, 0xF0, 0x02, 0);
    // Disable RMC
    set_msg_rate(ub, 0xF0, 0x04, 0);

    // Enable UBX,00 (LatLong + Velocity)
    set_msg_rate(ub, 0xF1, 0x00, 1);

    // Enable binary protocol messages
    // Enable I/O debug status MON-TXBUF
    set_msg_rate(ub, 0x0A, 0x08, 0);

    // Enable NAV-DOP
    set_msg_rate(ub, 0x01, 0x04, 0);
    // Enable NAV-POSLLH
    set_msg_rate(ub, 0x01, 0x02, 0);
    // Enable NAV-SBAS
    set_msg_rate(ub, 0x01, 0x32, 0);
    // Enable NAV-SOL (includes POSECEF and VELECEF)
    set_msg_rate(ub, 0x01, 0x06, 0);
    // Enable NAV-SVINFO
    set_msg_rate(ub, 0x01, 0x30, 0);
    // Enable NAV-VELNED
    set_msg_rate(ub, 0x01, 0x12, 0);

    // Set measurement period
    // LEA-6T-0 up to 200ms (5Hz)
    // LEA-6T-1 up to 500ms (2Hz)
    set_measurement_rate(ub, 1000/hz);

    // Enable RXM-RAW messages
    uint8_t enableRaw = getopt_get_bool(gopt, "raw");
    if (enableRaw) {
        set_msg_rate(ub, 0x02, 0x10, 1);
        set_msg_rate(ub, 0x02, 0x11, 1);
    } else {
        set_msg_rate(ub, 0x02, 0x10, 0);
        set_msg_rate(ub, 0x02, 0x11, 0);
    }

    ublox_start(ub);

    while (1) {
        // Poll for all RXM-EPH
        if (enableRaw) {
            ublox_command(ub, 0x02, 0x31, 0, NULL);
            sleep(2);
            ublox_command(ub, 0x02, 0x30, 0, NULL);
        }
        sleep(15);
    }

    ublox_destroy(ub);
}
