#include "comms/CommandLink.h"

#include <Arduino.h>
#include <math.h>

#include "estimator/VerticalEstimator.h"

namespace tardigrade {

namespace {
// Cap on bytes consumed per tick. A host that floods the port must not be able
// to starve the control loop by making the parser run forever.
constexpr int kMaxBytesPerUpdate = 256;

constexpr float kRadToCentiDeg = 5729.5779f;  // (180/pi) * 100
}  // namespace

CommandLink::CommandLink(Stream& io, Safety& safety, uint8_t motor_count)
    : io_(io), safety_(safety), motor_count_(motor_count) {}

void CommandLink::update(uint32_t now_us, const VehicleState& state) {
    int budget = kMaxBytesPerUpdate;
    while (io_.available() > 0 && budget-- > 0) {
        if (parser_.feed(static_cast<uint8_t>(io_.read()))) {
            dispatch(now_us, state);
        }
    }
}

void CommandLink::dispatch(uint32_t now_us, const VehicleState& state) {
    const MsgType type = parser_.type();

    // Pose frames come from the Jetson, not the operator. They must NOT feed
    // the operator deadman — losing the ground station has to disarm the
    // vehicle even while pose keeps streaming. Their own freshness is tracked
    // by ExternalEstimator. Route and return before noteTraffic().
    if (type == MsgType::Pose) {
        if (jetson_ != nullptr) {
            jetson_->onPoseFrame(parser_.payload(), parser_.length(), now_us);
        }
        return;
    }

    // Every OTHER valid frame is proof the operator is alive, whatever it asked.
    safety_.noteTraffic(now_us);

    switch (type) {
        case MsgType::Heartbeat:
            // noteTraffic() above is the whole point; nothing further to do.
            break;

        case MsgType::Arm:
            if (safety_.arm(now_us)) {
                sendAck(type, true, AckReason::Ok);
            } else {
                sendAck(type, false, AckReason::LinkLost);
            }
            break;

        case MsgType::Disarm:
            safety_.disarm(DisarmReason::Commanded);
            sendAck(type, true, AckReason::Ok);
            break;

        case MsgType::SetMotor: {
            if (parser_.length() < 3) {
                sendAck(type, false, AckReason::BadValue);
                break;
            }
            const uint8_t index = parser_.payload()[0];
            const int16_t raw = getI16(parser_.payload(), 1);
            if (raw < -1000 || raw > 1000) {
                sendAck(type, false, AckReason::BadValue);
                break;
            }
            const float value = raw * 0.001f;

            if (!safety_.armed()) {
                sendAck(type, false, AckReason::NotArmed);
                break;
            }
            if (index >= motor_count_) {
                sendAck(type, false, AckReason::BadIndex);
                break;
            }
            const bool ok = safety_.commandMotor(index, value, motor_count_);
            sendAck(type, ok, ok ? AckReason::Ok : AckReason::NotPermitted);
            break;
        }

        case MsgType::GetState:
            sendState(state);
            break;

        case MsgType::SetParameter: {
            if (parser_.length() < 6 || params_ == nullptr) {
                sendAck(type, false, AckReason::NotPermitted);
                break;
            }
            const uint16_t id = getU16(parser_.payload(), 0);
            const float value = getF32(parser_.payload(), 2);
            const bool ok = params_->setParameter(id, value);
            sendAck(type, ok, ok ? AckReason::Ok : AckReason::BadValue);
            break;
        }

        case MsgType::GetParameters:
            if (params_ != nullptr) {
                const uint16_t n = params_->parameterCount();
                for (uint16_t i = 0; i < n; ++i) {
                    uint16_t id;
                    float value;
                    if (params_->parameterAt(i, id, value)) {
                        sendParameter(id, value);
                    }
                }
            }
            break;

        case MsgType::SaveParameters: {
            const bool ok = params_ != nullptr && params_->saveParameters();
            sendAck(type, ok, ok ? AckReason::Ok : AckReason::NotPermitted);
            break;
        }

        case MsgType::ResetParameters: {
            const bool ok = params_ != nullptr && params_->resetParameters();
            sendAck(type, ok, ok ? AckReason::Ok : AckReason::NotPermitted);
            // Stream the restored values back so the ground station's fields
            // update to the defaults it just reverted to.
            if (ok) {
                const uint16_t n = params_->parameterCount();
                for (uint16_t i = 0; i < n; ++i) {
                    uint16_t id;
                    float value;
                    if (params_->parameterAt(i, id, value)) {
                        sendParameter(id, value);
                    }
                }
            }
            break;
        }

        default:
            sendAck(type, false, AckReason::NotPermitted);
            break;
    }
}

void CommandLink::sendParameter(uint16_t id, float value) {
    uint8_t payload[6];
    size_t n = 0;
    putU16(payload, n, id);
    putF32(payload, n, value);
    sendFrame(MsgType::Parameter, payload, static_cast<uint8_t>(n));
}

void CommandLink::sendState(const VehicleState& state) {
    uint8_t payload[kMaxPayload];
    size_t n = 0;

    putU32(payload, n, state.timestamp_us);
    putI16(payload, n, static_cast<int16_t>(state.roll_rad * kRadToCentiDeg));
    putI16(payload, n, static_cast<int16_t>(state.pitch_rad * kRadToCentiDeg));
    putI16(payload, n, static_cast<int16_t>(state.yaw_rad * kRadToCentiDeg));
    putI16(payload, n, static_cast<int16_t>(state.altitude_m * 1000.0f));
    putI16(payload, n,
           static_cast<int16_t>(state.vertical_velocity_mps * 1000.0f));

    uint8_t flags = 0;
    if (safety_.armed())      flags |= kFlagArmed;
    if (state.valid)          flags |= kFlagStateValid;
    if (state.altitude_valid) flags |= kFlagAltitudeOk;
    if (!safety_.linkLost())  flags |= kFlagLinkOk;
    payload[n++] = flags;

    // Raw ToF ranges, mm. 0xFFFF = no sensor / no recent reading — the ground
    // station shows the dropout instead of a stale number.
    for (uint8_t i = 0; i < kMaxRangeSensors; ++i) {
        putU16(payload, n,
               vertical_ != nullptr ? vertical_->lastRangeMm(i) : 0xFFFF);
    }

    sendFrame(MsgType::State, payload, static_cast<uint8_t>(n));
}

void CommandLink::sendAck(MsgType echoed, bool accepted, AckReason reason) {
    const uint8_t payload[3] = {
        static_cast<uint8_t>(echoed),
        static_cast<uint8_t>(accepted ? 1 : 0),
        static_cast<uint8_t>(reason),
    };
    sendFrame(MsgType::Ack, payload, 3);
}

void CommandLink::sendText(const char* text) {
    if (text == nullptr) {
        return;
    }
    uint8_t payload[kMaxPayload];
    uint8_t n = 0;
    while (text[n] != '\0' && n < kMaxPayload) {
        payload[n] = static_cast<uint8_t>(text[n]);
        ++n;
    }
    sendFrame(MsgType::Text, payload, n);
}

void CommandLink::sendFrame(MsgType type, const uint8_t* payload, uint8_t len) {
    uint8_t frame[kMaxFrame];
    const size_t n = encodeFrame(type, payload, len, frame, sizeof(frame));
    if (n > 0) {
        io_.write(frame, n);
    }
}

}  // namespace tardigrade
