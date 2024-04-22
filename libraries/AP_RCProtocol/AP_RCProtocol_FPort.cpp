/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
  FRSky FPort implementation, with thanks to BetaFlight for
  specification and code reference
 */

#include "AP_RCProtocol_FPort.h"

#if AP_RCPROTOCOL_FPORT_ENABLED

#include <AP_Vehicle/AP_Vehicle_Type.h>
#include <RC_Channel/RC_Channel.h>
#include <AP_Math/AP_Math.h>
#include <AP_Math/crc.h>

extern const AP_HAL::HAL& hal;

#define FRAME_HEAD 0x7E
#define FRAME_DLE  0x7D
#define FRAME_XOR  0x20
#define FRAME_LEN_CONTROL 0x19
#define FRAME_LEN_DOWNLINK 0x08
#define MIN_FRAME_SIZE 12
#define MAX_CHANNELS 16

#define FLAGS_FAILSAFE_BIT	3
#define FLAGS_FRAMELOST_BIT	2

#define CHAN_SCALE_FACTOR1 1000U
#define CHAN_SCALE_FACTOR2 1600U
#define CHAN_SCALE_OFFSET 875U

#define FPORT_TYPE_CONTROL 0
#define FPORT_TYPE_DOWNLINK 1
#define FPORT_PRIM_NULL 0x00
#define FPORT_PRIM_DATA 0x10
#define FPORT_PRIM_READ 0x30
#define FPORT_PRIM_WRITE 0x31

#define MAX_FPORT_CONSECUTIVE_FRAMES 2

struct PACKED FPort_Frame {
    uint8_t header; // 0x7E
    uint8_t len;    // 0x19 for control, 0x08 for downlink
    uint8_t type;
    union {
        struct PACKED {
            uint8_t data[22]; // 16 11-bit channels
            uint8_t flags;
            uint8_t rssi;
            uint8_t crc;
            uint8_t end;
        } control;
        struct PACKED {
            uint8_t prim;
            uint16_t appid;
            uint8_t data[4];
            uint8_t crc;
            uint8_t end;
        } downlink;
    };
};

static_assert(sizeof(FPort_Frame) == FPORT_CONTROL_FRAME_SIZE, "FPort_Frame incorrect size");

// constructor
AP_RCProtocol_FPort::AP_RCProtocol_FPort(AP_RCProtocol &_frontend, bool _inverted) :
    AP_RCProtocol_Backend(_frontend),
    inverted(_inverted)
{}

// decode a full FPort control frame
void AP_RCProtocol_FPort::decode_control(const FPort_Frame &frame)
{
    uint16_t values[MAX_CHANNELS];

    decode_11bit_channels(frame.control.data, MAX_CHANNELS, values, CHAN_SCALE_FACTOR1, CHAN_SCALE_FACTOR2, CHAN_SCALE_OFFSET);

    bool failsafe = ((frame.control.flags & (1 << FLAGS_FAILSAFE_BIT)) != 0);

    // fport rssi 0-50, ardupilot rssi 0-255, scale factor 255/50=5.1
    const uint8_t scaled_rssi = MIN(frame.control.rssi * 5.1f, 255);

    add_input(MAX_CHANNELS, values, failsafe, scaled_rssi);
}

/*
  decode a full FPort downlink frame
*/
void AP_RCProtocol_FPort::decode_downlink(const FPort_Frame &frame)
{

}

/*
  process a FPort input pulse of the given width
 */
void AP_RCProtocol_FPort::process_pulse(uint32_t width_s0, uint32_t width_s1)
{
    if (have_UART()) {
        // if we can use a UART we would much prefer to, as it allows
        // us to send SPORT data out
        return;
    }
    uint32_t w0 = width_s0;
    uint32_t w1 = width_s1;
    if (inverted) {
        w0 = saved_width;
        w1 = width_s0;
        saved_width = width_s1;
    }
    uint8_t b;
    if (ss.process_pulse(w0, w1, b)) {
        _process_byte(ss.get_byte_timestamp_us(), b);
    }
}

// support byte input
void AP_RCProtocol_FPort::_process_byte(uint32_t timestamp_us, uint8_t b)
{
    const bool have_frame_gap = (timestamp_us - byte_input.last_byte_us >= 2000U);

    byte_input.last_byte_us = timestamp_us;

    if (have_frame_gap) {
        // if we have a frame gap then this must be the start of a new
        // frame
        byte_input.ofs = 0;
        byte_input.got_DLE = false;
    }
    if (b != FRAME_HEAD && byte_input.ofs == 0) {
        // definately not FPort, missing header byte
        return;
    }

    // handle byte-stuffing decode
    if (byte_input.got_DLE) {
        b ^= FRAME_XOR;
        byte_input.got_DLE = false;
    } else if (b == FRAME_DLE) {
        byte_input.got_DLE = true;
        return;
    }

    byte_input.buf[byte_input.ofs++] = b;

    const FPort_Frame *frame = (const FPort_Frame *)&byte_input.buf[0];

    if (byte_input.ofs == 2) {
        // check for valid lengths
        if (frame->len != FRAME_LEN_CONTROL &&
            frame->len != FRAME_LEN_DOWNLINK) {
            // invalid, reset
            goto reset;
        }
    }

    if (byte_input.ofs == 3) {
        // check for valid lengths
        if ((frame->type == FPORT_TYPE_CONTROL && frame->len != FRAME_LEN_CONTROL) ||
            (frame->type == FPORT_TYPE_DOWNLINK && frame->len != FRAME_LEN_DOWNLINK)) {
            goto reset;
        }
        if (frame->type != FPORT_TYPE_CONTROL && frame->type != FPORT_TYPE_DOWNLINK) {
            // invalid type
            goto reset;
        }
    }

    if (frame->type == FPORT_TYPE_CONTROL && byte_input.ofs == FRAME_LEN_CONTROL + 4) {
        log_data(AP_RCProtocol::FPORT, timestamp_us, byte_input.buf, byte_input.ofs);
        if (check_checksum()) {
            decode_control(*frame);
        }
        goto reset;
    } else if (frame->type == FPORT_TYPE_DOWNLINK && byte_input.ofs == FRAME_LEN_DOWNLINK + 4) {
        log_data(AP_RCProtocol::FPORT, timestamp_us, byte_input.buf, byte_input.ofs);
        if (check_checksum()) {
            decode_downlink(*frame);
        }
        goto reset;
    }
    if (byte_input.ofs == sizeof(byte_input.buf)) {
        goto reset;
    }
    return;

reset:
    byte_input.ofs = 0;
    byte_input.got_DLE = false;
}

// check checksum byte
bool AP_RCProtocol_FPort::check_checksum(void)
{
    const uint8_t len = byte_input.buf[1]+2;
    return crc_sum8_with_carry(&byte_input.buf[1], len) == 0x00;
}

// support byte input
void AP_RCProtocol_FPort::process_byte(uint8_t b, uint32_t baudrate)
{
    if (baudrate != 115200) {
        return;
    }
    _process_byte(AP_HAL::micros(), b);
}

#endif  // AP_RCPROTOCOL_FPORT_ENABLED
