#include "comms/PacketParser.h"

namespace tardigrade {

bool PacketParser::feed(uint8_t byte) {
    switch (state_) {
        case State::Sync1:
            if (byte == kSync1) {
                state_ = State::Sync2;
            }
            return false;

        case State::Sync2:
            // A second SYNC1 is not a failure — it may be the real frame start
            // after a truncated one, so hold position rather than dropping out.
            if (byte == kSync2) {
                state_ = State::Type;
            } else if (byte != kSync1) {
                state_ = State::Sync1;
            }
            return false;

        case State::Type:
            type_ = byte;
            state_ = State::Length;
            return false;

        case State::Length:
            if (byte > kMaxPayload) {
                // Impossible length: this was noise that happened to look like
                // a header. Resynchronise rather than trust it.
                state_ = State::Sync1;
                return false;
            }
            length_ = byte;
            index_ = 0;
            state_ = (length_ == 0) ? State::CrcLow : State::Payload;
            return false;

        case State::Payload:
            payload_[index_++] = byte;
            if (index_ >= length_) {
                state_ = State::CrcLow;
            }
            return false;

        case State::CrcLow:
            crc_received_ = byte;
            state_ = State::CrcHigh;
            return false;

        case State::CrcHigh: {
            crc_received_ |= static_cast<uint16_t>(byte) << 8;
            state_ = State::Sync1;

            // Recompute over TYPE, LEN and PAYLOAD, matching encodeFrame().
            uint8_t scratch[kMaxPayload + 2];
            scratch[0] = type_;
            scratch[1] = length_;
            for (uint8_t i = 0; i < length_; ++i) {
                scratch[2 + i] = payload_[i];
            }
            const uint16_t crc = crc16(scratch, static_cast<size_t>(length_) + 2);

            if (crc != crc_received_) {
                ++crc_errors_;
                return false;
            }
            ++frames_decoded_;
            return true;
        }
    }
    return false;
}

}  // namespace tardigrade
