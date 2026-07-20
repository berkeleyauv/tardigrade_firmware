"""Tardigrade wire protocol — single Python source of truth.

Mirrors firmware/include/comms/Protocol.h. Imported by gcs.py (CLI),
gcs_server.py (remote backend), and pose_bridge.py (Jetson). Keeping one copy is
the whole point: three hand-maintained CRC implementations would drift, and a
CRC that disagrees rejects every frame.

Run `python tardigrade_protocol.py` to self-test against the firmware.
"""

import struct

SYNC1 = 0xA7
SYNC2 = 0x5E
MAX_PAYLOAD = 64

# Host -> vehicle
HEARTBEAT, ARM, DISARM, SET_MOTOR, GET_STATE, POSE = 0x01, 0x02, 0x03, 0x04, 0x05, 0x06
# Vehicle -> host
STATE, ACK, TEXT = 0x80, 0x81, 0x82

POSE_FRAME_LEN = 54  # kPoseFrameLen

TYPE_NAMES = {
    HEARTBEAT: "HEARTBEAT", ARM: "ARM", DISARM: "DISARM", SET_MOTOR: "SET_MOTOR",
    GET_STATE: "GET_STATE", POSE: "POSE", STATE: "STATE", ACK: "ACK", TEXT: "TEXT",
}
ACK_REASONS = {
    0: "Ok", 1: "NotArmed", 2: "LinkLost",
    3: "BadIndex", 4: "BadValue", 5: "NotPermitted",
}


def crc16(data: bytes) -> int:
    """CRC-16/CCITT-FALSE. Must match crc16() in Protocol.cpp."""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def encode(msg_type: int, payload: bytes = b"") -> bytes:
    if len(payload) > MAX_PAYLOAD:
        raise ValueError("payload too large")
    body = bytes([msg_type, len(payload)]) + payload
    return bytes([SYNC1, SYNC2]) + body + struct.pack("<H", crc16(body))


def encode_set_motor(index: int, value: float) -> bytes:
    v = max(-1000, min(1000, int(round(value * 1000))))
    return encode(SET_MOTOR, bytes([index & 0xFF]) + struct.pack("<h", v))


def encode_pose(seq, pos, quat, linvel, angvel) -> bytes:
    """pos/linvel/angvel: (x, y, z); quat: (w, x, y, z). See kPoseFrameLen."""
    payload = struct.pack(
        "<H13f", seq & 0xFFFF,
        pos[0], pos[1], pos[2],
        quat[0], quat[1], quat[2], quat[3],
        linvel[0], linvel[1], linvel[2],
        angvel[0], angvel[1], angvel[2],
    )
    assert len(payload) == POSE_FRAME_LEN
    return encode(POSE, payload)


def decode_state(p: bytes) -> dict:
    """State telemetry payload -> dict, or {} if malformed."""
    if len(p) < 15:
        return {}
    ts, roll, pitch, yaw, alt, vz = struct.unpack("<Ihhhhh", p[:14])
    flags = p[14]
    out = {
        "ts": ts, "roll": roll / 100, "pitch": pitch / 100, "yaw": yaw / 100,
        "alt": alt / 1000, "vz": vz / 1000,
        "armed": bool(flags & 1), "state_ok": bool(flags & 2),
        "alt_ok": bool(flags & 4), "link_ok": bool(flags & 8),
        "pose_ok": bool(flags & 16),
    }
    if len(p) >= 19:
        a, b = struct.unpack("<HH", p[15:19])
        out["tof"] = [None if v == 0xFFFF else v for v in (a, b)]
    return out


class Parser:
    """Mirror of firmware PacketParser: same resync behaviour."""

    def __init__(self):
        self.buf = bytearray()
        self.crc_errors = 0

    def feed(self, chunk: bytes):
        """Yield (type, payload) for each valid frame found."""
        self.buf.extend(chunk)
        while True:
            start = self.buf.find(bytes([SYNC1, SYNC2]))
            if start < 0:
                del self.buf[:-1]  # keep one byte for a straddling sync pair
                return
            del self.buf[:start]
            if len(self.buf) < 6:
                return
            length = self.buf[3]
            if length > MAX_PAYLOAD:
                del self.buf[:2]
                continue
            total = 6 + length
            if len(self.buf) < total:
                return
            body = bytes(self.buf[2:4 + length])
            got = struct.unpack("<H", bytes(self.buf[4 + length:total]))[0]
            if crc16(body) != got:
                self.crc_errors += 1
                del self.buf[:2]
                continue
            frame = (self.buf[2], bytes(self.buf[4:4 + length]))
            del self.buf[:total]
            yield frame


def _selftest() -> int:
    fails = 0

    def check(cond, what):
        nonlocal fails
        print(f"{'[ ok ]' if cond else '[FAIL]'}  {what}")
        fails += 0 if cond else 1

    check(crc16(b"123456789") == 0x29B1, "CRC standard vector 0x29B1")

    f = encode_set_motor(2, 0.25)
    out = list(Parser().feed(f))
    check(len(out) == 1 and out[0][0] == SET_MOTOR, "set_motor decodes")
    check(out and struct.unpack("<h", out[0][1][1:3])[0] == 250, "value 250")

    fn = encode_set_motor(5, -0.30)
    on = list(Parser().feed(fn))
    check(on and struct.unpack("<h", on[0][1][1:3])[0] == -300, "reverse -300")

    pf = encode_pose(7, (1.0, 2.0, -0.5), (1.0, 0.0, 0.0, 0.0),
                     (0.1, 0.0, 0.0), (0.0, 0.0, 0.2))
    check(len(pf) == 6 + POSE_FRAME_LEN, "pose frame length 54+6")
    po = list(Parser().feed(pf))
    check(len(po) == 1 and po[0][0] == POSE, "pose decodes")
    seq = struct.unpack("<H", po[0][1][:2])[0]
    px = struct.unpack("<f", po[0][1][2:6])[0]
    check(seq == 7 and abs(px - 1.0) < 1e-6, "pose seq + position round trip")

    bad = bytearray(f); bad[5] ^= 1
    p2 = Parser()
    check(len(list(p2.feed(bytes(bad)))) == 0 and p2.crc_errors == 1,
          "bit flip rejected")

    print(f"\n{'FAILURES' if fails else 'ALL PASSED'} ({fails})")
    return 1 if fails else 0


if __name__ == "__main__":
    raise SystemExit(_selftest())
