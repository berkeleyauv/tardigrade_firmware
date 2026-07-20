#pragma once
//
// CommandLink — ground-link command handling and telemetry encoding.
//
// Drains whatever bytes the serial port has, decodes complete frames, and
// routes them. Every command is gated through Safety; this class never touches
// a motor directly, so there is exactly one place where "may this spin?" is
// decided.
//
// Non-blocking by construction: it reads only what is already buffered and
// returns, so a silent or half-speaking host costs the control loop nothing.

#include "comms/PacketParser.h"
#include "core/types.h"
#include "safety/Safety.h"

class Stream;  // Arduino

namespace tardigrade {

class VerticalEstimator;

class CommandLink {
public:
    CommandLink(Stream& io, Safety& safety, uint8_t motor_count);

    // Optional: lets State frames carry the raw ToF ranges. Left unset, those
    // fields report 0xFFFF ("no sensor") and everything else works unchanged.
    void setVerticalEstimator(const VerticalEstimator* vertical) {
        vertical_ = vertical;
    }

    // Call every tick. Drains the input buffer and dispatches whole frames.
    // `state` is what a GetState request will be answered with.
    void update(uint32_t now_us, const VehicleState& state);

    void sendText(const char* text);
    void sendState(const VehicleState& state);

    uint32_t crcErrors() const { return parser_.crcErrors(); }
    uint32_t framesDecoded() const { return parser_.framesDecoded(); }

private:
    void dispatch(uint32_t now_us, const VehicleState& state);
    void sendAck(MsgType echoed, bool accepted, AckReason reason);
    void sendFrame(MsgType type, const uint8_t* payload, uint8_t len);

    Stream& io_;
    Safety& safety_;
    PacketParser parser_;
    const VerticalEstimator* vertical_ = nullptr;
    uint8_t motor_count_;
};

}  // namespace tardigrade
