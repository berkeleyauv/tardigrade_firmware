#pragma once
//
// PacketParser — incremental frame decoder for the ground link.
//
// Fed one byte at a time and never blocks or waits for the rest of a frame, so
// a half-received packet costs nothing and a host that dies mid-frame cannot
// stall the control loop. Resynchronisation is automatic: any byte that does
// not fit the expected position drops the parser back to hunting for SYNC1.
//
// CRC failures are counted rather than reported as packets. A rising crcErrors()
// on an otherwise quiet link is the signature of a bad cable or a baud mismatch,
// which is worth being able to see from the ground station.

#include "comms/Protocol.h"

namespace tardigrade {

class PacketParser {
public:
    // Returns true exactly once per complete, CRC-valid frame. The accessors
    // below stay valid until the next feed() call.
    bool feed(uint8_t byte);

    MsgType type() const { return static_cast<MsgType>(type_); }
    const uint8_t* payload() const { return payload_; }
    uint8_t length() const { return length_; }

    uint32_t crcErrors() const { return crc_errors_; }
    uint32_t framesDecoded() const { return frames_decoded_; }

    void reset() { state_ = State::Sync1; index_ = 0; }

private:
    enum class State : uint8_t {
        Sync1,
        Sync2,
        Type,
        Length,
        Payload,
        CrcLow,
        CrcHigh,
    };

    State state_ = State::Sync1;
    uint8_t type_ = 0;
    uint8_t length_ = 0;
    uint8_t index_ = 0;
    uint16_t crc_received_ = 0;
    uint8_t payload_[kMaxPayload] = {};

    uint32_t crc_errors_ = 0;
    uint32_t frames_decoded_ = 0;
};

}  // namespace tardigrade
