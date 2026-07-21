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
    // [index:u8][value:i16 -1000..+1000] — signed thousandths of full thrust,
    // 0 = stopped. Signed even on the hopcopter so that the value meaning
    // "stop" is the same number on every vehicle. See MotorOutput in types.h.
    SetMotor  = 0x04,
    GetState  = 0x05,
    // Fused pose from the Jetson (robosub). See PoseFrame below. Injected onto
    // the same serial link as operator commands; the Jetson bridge multiplexes
    // both. Does NOT feed Safety's link deadman — pose freshness is a separate
    // failsafe owned by ExternalEstimator, exactly as the IMU is on the
    // hopcopter. Losing the operator must still disarm even while pose flows.
    Pose      = 0x06,
    // Live tuning. SetParameter: [id:u16][value:f32]. GetParameters: empty, asks
    // the vehicle to stream back every current value as Parameter frames so the
    // ground station's controls start where the firmware actually is.
    SetParameter    = 0x07,
    GetParameters   = 0x08,
    SaveParameters  = 0x09,  // persist current values to flash
    ResetParameters = 0x0A,  // restore compiled defaults + clear flash

    // ---- vehicle -> host ----
    State     = 0x80,  // see encodeState()
    Ack       = 0x81,  // [echoed type:u8][accepted:u8][reason:u8]
    Text      = 0x82,  // human-readable string, no NUL
    Parameter = 0x83,  // [id:u16][value:f32] — one current parameter value
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
inline constexpr uint8_t kFlagPoseOk       = 1 << 4;  // robosub: pose link fresh

// Pose frame payload (Jetson -> ESP), little-endian. float32 rather than the
// scaled int16s used for telemetry: this is the EKF's authoritative output and
// the values (metres, unit quaternion, m/s, rad/s) don't share a natural fixed
// scale. Carries the FULL pose so the wire format need not change when
// VehicleState later gains x/y position; ExternalEstimator maps the subset the
// contract currently holds.
//
//   seq   : u16   sequence counter, for drop detection
//   pos   : f32*3 x, y, z   world frame (ENU per REP-103), metres
//   quat  : f32*4 w, x, y, z   body-to-world, normalized
//   linvel: f32*3 x, y, z   world frame, m/s
//   angvel: f32*3 x, y, z   body frame, rad/s
// = 2 + 12 + 16 + 12 + 12 = 54 bytes.
inline constexpr uint8_t kPoseFrameLen = 54;

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

void putF32(uint8_t* buf, size_t& offset, float value);

uint16_t getU16(const uint8_t* buf, size_t offset);
int16_t getI16(const uint8_t* buf, size_t offset);
float getF32(const uint8_t* buf, size_t offset);

}  // namespace tardigrade
