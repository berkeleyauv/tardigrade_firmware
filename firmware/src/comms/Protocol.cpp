#include "comms/Protocol.h"

namespace tardigrade {

uint16_t crc16(const uint8_t* data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; ++i) {
        crc ^= static_cast<uint16_t>(data[i]) << 8;
        for (uint8_t bit = 0; bit < 8; ++bit) {
            crc = (crc & 0x8000) ? static_cast<uint16_t>((crc << 1) ^ 0x1021)
                                 : static_cast<uint16_t>(crc << 1);
        }
    }
    return crc;
}

size_t encodeFrame(MsgType type, const uint8_t* payload, uint8_t len,
                   uint8_t* out, size_t out_cap) {
    if (len > kMaxPayload || out == nullptr) {
        return 0;
    }
    const size_t total = kFrameOverhead + len;
    if (out_cap < total) {
        return 0;
    }

    out[0] = kSync1;
    out[1] = kSync2;
    out[2] = static_cast<uint8_t>(type);
    out[3] = len;
    for (uint8_t i = 0; i < len; ++i) {
        out[4 + i] = payload[i];
    }

    // Checksum covers TYPE, LEN and PAYLOAD — not the sync bytes, which carry
    // no information and would only ever match themselves.
    const uint16_t crc = crc16(&out[2], static_cast<size_t>(len) + 2);
    out[4 + len] = static_cast<uint8_t>(crc & 0xFF);
    out[5 + len] = static_cast<uint8_t>((crc >> 8) & 0xFF);
    return total;
}

void putU16(uint8_t* buf, size_t& offset, uint16_t value) {
    buf[offset++] = static_cast<uint8_t>(value & 0xFF);
    buf[offset++] = static_cast<uint8_t>((value >> 8) & 0xFF);
}

void putI16(uint8_t* buf, size_t& offset, int16_t value) {
    putU16(buf, offset, static_cast<uint16_t>(value));
}

void putU32(uint8_t* buf, size_t& offset, uint32_t value) {
    buf[offset++] = static_cast<uint8_t>(value & 0xFF);
    buf[offset++] = static_cast<uint8_t>((value >> 8) & 0xFF);
    buf[offset++] = static_cast<uint8_t>((value >> 16) & 0xFF);
    buf[offset++] = static_cast<uint8_t>((value >> 24) & 0xFF);
}

uint16_t getU16(const uint8_t* buf, size_t offset) {
    return static_cast<uint16_t>(buf[offset]) |
           (static_cast<uint16_t>(buf[offset + 1]) << 8);
}

}  // namespace tardigrade
