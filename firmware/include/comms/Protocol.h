#pragma once
//
// Wire protocol for the ground link — the framing sketched in
// docs/communication.md, made concrete.
//
//   [SYNC1][SYNC2][TYPE][LEN][PAYLOAD ... LEN bytes][CRC_LO][CRC_HI]
//
// The CRC is not decoration. A single flipped bit on a SET_MOTOR command is the
// difference between "spin motor 0 at 5%" and "spin motor 3 at 90%", so every
// frame is rejected outright unless its checksum matches. Corrupt input must
// never reach the motors.
//
// Multi-byte fields are little-endian, matching both the ESP32 and any x86 host.

#include <stddef.h>
#include <stdint.h>

namespace tardigrade {

inline constexpr uint8_t kSync1 = 0xA7;
inline constexpr uint8_t kSync2 = 0x5E;

// Payload cap. Deliberately small: every buffer this protocol touches is
// statically sized, and nothing we send today comes close.
inline constexpr uint8_t kMaxPayload = 64;

// Framing overhead: 2 sync + type + len + 2 CRC.
inline constexpr size_t kFrameOverhead = 6;
inline constexpr size_t kMaxFrame = kFrameOverhead + kMaxPayload;

enum class MsgType : uint8_t {
    // ---- host -> vehicle ----
    // Proof the host is still there. The Safety watchdog is fed by ANY valid
    // command, but a UI that is merely idle still has to send these or the
    // vehicle will correctly conclude the link is gone.
    Heartbeat = 0x01,
    Arm       = 0x02,
    Disarm    = 0x03,
    SetMotor  = 0x04,  // [index:u8][value:u16 0..1000]
    GetState  = 0x05,

    // ---- vehicle -> host ----
    State = 0x80,      // see encodeState()
    Ack   = 0x81,      // [echoed type:u8][accepted:u8][reason:u8]
    Text  = 0x82,      // human-readable string, no NUL
};

// Why a command was refused. Sent back in Ack so the UI can say something
// useful instead of silently doing nothing.
enum class AckReason : uint8_t {
    Ok             = 0,
    NotArmed       = 1,
    LinkLost       = 2,
    BadIndex       = 3,
    BadValue       = 4,
    NotPermitted   = 5,  // right command, wrong vehicle state
};

// Status bits in the State payload.
inline constexpr uint8_t kFlagArmed        = 1 << 0;
inline constexpr uint8_t kFlagStateValid   = 1 << 1;
inline constexpr uint8_t kFlagAltitudeOk   = 1 << 2;
inline constexpr uint8_t kFlagLinkOk       = 1 << 3;

// CRC-16/CCITT-FALSE over TYPE, LEN and PAYLOAD. Bit-serial rather than
// table-driven: at 20 Hz telemetry the cycles are free and the table is not.
uint16_t crc16(const uint8_t* data, size_t len);

// Build a complete frame into `out`. Returns bytes written, or 0 if the payload
// is oversized or the buffer too small.
size_t encodeFrame(MsgType type, const uint8_t* payload, uint8_t len,
                   uint8_t* out, size_t out_cap);

// Little-endian field writers. Each advances `offset`.
void putU16(uint8_t* buf, size_t& offset, uint16_t value);
void putI16(uint8_t* buf, size_t& offset, int16_t value);
void putU32(uint8_t* buf, size_t& offset, uint32_t value);

uint16_t getU16(const uint8_t* buf, size_t offset);

}  // namespace tardigrade
