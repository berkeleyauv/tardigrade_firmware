#include "comms/JetsonLink.h"

#include "comms/Protocol.h"

namespace tardigrade {

bool JetsonLink::onPoseFrame(const uint8_t* payload, uint8_t len,
                             uint32_t now_us) {
    if (len != kPoseFrameLen) {
        return false;  // malformed; keep the previous sample untouched
    }

    size_t o = 0;
    PoseSample s;
    s.seq = getU16(payload, o); o += 2;
    s.position.x = getF32(payload, o); o += 4;
    s.position.y = getF32(payload, o); o += 4;
    s.position.z = getF32(payload, o); o += 4;
    s.orientation.w = getF32(payload, o); o += 4;
    s.orientation.x = getF32(payload, o); o += 4;
    s.orientation.y = getF32(payload, o); o += 4;
    s.orientation.z = getF32(payload, o); o += 4;
    s.linear_velocity.x = getF32(payload, o); o += 4;
    s.linear_velocity.y = getF32(payload, o); o += 4;
    s.linear_velocity.z = getF32(payload, o); o += 4;
    s.angular_velocity.x = getF32(payload, o); o += 4;
    s.angular_velocity.y = getF32(payload, o); o += 4;
    s.angular_velocity.z = getF32(payload, o); o += 4;
    s.received_us = now_us;
    s.valid = true;

    // Count gaps in the sequence. A rising drop rate on an otherwise live link
    // points at serial saturation or a struggling bridge — worth being able to
    // see rather than silently absorbing.
    if (seq_started_) {
        const uint16_t expected = static_cast<uint16_t>(last_seq_ + 1);
        if (s.seq != expected) {
            drops_ += static_cast<uint16_t>(s.seq - expected);
        }
    }
    seq_started_ = true;
    last_seq_ = s.seq;

    latest_ = s;
    return true;
}

}  // namespace tardigrade
