#!/usr/bin/env python3
"""Minimal ground station for Tardigrade — host side of docs/communication.md.

Reference implementation of the wire protocol plus an interactive console for
bench testing. Deliberately dependency-light: everything except the serial port
itself is standard library, and --selftest runs with no hardware at all.

  python tools/gcs.py --selftest
  python tools/gcs.py --port COM5
  python tools/gcs.py --port COM5 --motor 2 --value 0.15

Interactive commands: arm, disarm, m <index> <0..1>, state, quit
While armed the tool heartbeats automatically; stop it and the firmware
watchdog should disarm on its own. That is the behaviour worth verifying.
"""

import argparse
import struct
import sys
import threading
import time

SYNC1 = 0xA7
SYNC2 = 0x5E
MAX_PAYLOAD = 64

# Host -> vehicle
HEARTBEAT, ARM, DISARM, SET_MOTOR, GET_STATE = 0x01, 0x02, 0x03, 0x04, 0x05
# Vehicle -> host
STATE, ACK, TEXT = 0x80, 0x81, 0x82

TYPE_NAMES = {
    HEARTBEAT: "HEARTBEAT", ARM: "ARM", DISARM: "DISARM",
    SET_MOTOR: "SET_MOTOR", GET_STATE: "GET_STATE",
    STATE: "STATE", ACK: "ACK", TEXT: "TEXT",
}
ACK_REASONS = {
    0: "Ok", 1: "NotArmed", 2: "LinkLost",
    3: "BadIndex", 4: "BadValue", 5: "NotPermitted",
}


def crc16(data: bytes) -> int:
    """CRC-16/CCITT-FALSE. Must match crc16() in firmware/src/comms/Protocol.cpp."""
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


class Parser:
    """Mirror of firmware PacketParser: same states, same resync behaviour."""

    def __init__(self):
        self.buf = bytearray()
        self.crc_errors = 0

    def feed(self, chunk: bytes):
        """Yield (type, payload) for each valid frame found."""
        self.buf.extend(chunk)
        while True:
            start = self.buf.find(bytes([SYNC1, SYNC2]))
            if start < 0:
                # Keep one byte in case a sync pair straddles the boundary.
                del self.buf[:-1]
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


def decode_state(p: bytes) -> str:
    if len(p) < 15:
        return f"STATE malformed ({len(p)} bytes)"
    ts, roll, pitch, yaw, alt, vz = struct.unpack("<Ihhhhh", p[:14])
    flags = p[14]
    bits = [n for b, n in ((1, "ARMED"), (2, "STATE_OK"), (4, "ALT_OK"), (8, "LINK_OK"))
            if flags & b]
    tof = ""
    if len(p) >= 19:  # raw ToF ranges appended; 0xFFFF = no recent reading
        mm = struct.unpack("<HH", p[15:19])
        tof = "  tof=(" + ",".join("---" if v == 0xFFFF else f"{v}mm" for v in mm) + ")"
    return (f"t={ts:>10}us  rpy=({roll/100:7.2f},{pitch/100:7.2f},{yaw/100:7.2f})deg  "
            f"alt={alt/1000:6.3f}m  vz={vz/1000:6.3f}m/s{tof}  [{' '.join(bits) or '-'}]")


def selftest() -> int:
    fails = 0

    def check(cond, what):
        nonlocal fails
        print(f"{'[ ok ]' if cond else '[FAIL]'}  {what}")
        if not cond:
            fails += 1

    # Standard check value for CRC-16/CCITT-FALSE pins the algorithm itself.
    check(crc16(b"123456789") == 0x29B1,
          f"CRC matches standard vector (got 0x{crc16(b'123456789'):04X}, want 0x29B1)")

    frame = encode(SET_MOTOR, bytes([2]) + struct.pack("<H", 250))
    check(len(frame) == 6 + 3, "frame length = overhead + payload")
    check(frame[0] == SYNC1 and frame[1] == SYNC2, "sync bytes present")

    p = Parser()
    out = list(p.feed(frame))
    check(len(out) == 1, "one frame decoded")
    check(out and out[0][0] == SET_MOTOR, "type round trips")
    check(out and out[0][1][0] == 2, "motor index round trips")
    check(out and struct.unpack("<H", out[0][1][1:3])[0] == 250, "value round trips")

    bad = bytearray(frame)
    bad[5] ^= 0x01
    p2 = Parser()
    check(len(list(p2.feed(bytes(bad)))) == 0, "single bit flip rejected")
    check(p2.crc_errors == 1, "CRC error counted")

    p3 = Parser()
    noisy = bytes([0x00, 0xFF, 0xA7, 0x12, 0x5E, 0x99]) + frame
    check(len(list(p3.feed(noisy))) == 1, "resyncs past junk")

    p4 = Parser()
    both = frame + encode(HEARTBEAT)
    check(len(list(p4.feed(both))) == 2, "back-to-back frames decode")

    p5 = Parser()
    got = []
    for i in range(len(frame)):  # one byte at a time
        got += list(p5.feed(frame[i:i + 1]))
    check(len(got) == 1, "decodes when split byte-by-byte")

    print(f"\n{'FAILURES' if fails else 'ALL PASSED'} ({fails} failures)")
    return 1 if fails else 0


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--port")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--selftest", action="store_true")
    ap.add_argument("--motor", type=int)
    ap.add_argument("--value", type=float, default=0.0)
    args = ap.parse_args()

    if args.selftest or not args.port:
        if not args.selftest:
            ap.error("--port is required (or use --selftest)")
        return selftest()

    try:
        import serial  # pyserial
    except ImportError:
        print("pyserial required:  pip install pyserial", file=sys.stderr)
        return 1

    ser = serial.Serial(args.port, args.baud, timeout=0.05)
    parser = Parser()
    armed = threading.Event()
    stop = threading.Event()

    def rx():
        while not stop.is_set():
            data = ser.read(256)
            if not data:
                continue
            for mtype, payload in parser.feed(data):
                if mtype == STATE:
                    print("  " + decode_state(payload))
                elif mtype == ACK:
                    echoed, ok, reason = payload[0], payload[1], payload[2]
                    print(f"  ACK {TYPE_NAMES.get(echoed, hex(echoed))}: "
                          f"{'accepted' if ok else 'REFUSED'} ({ACK_REASONS.get(reason)})")
                elif mtype == TEXT:
                    print("  TEXT: " + payload.decode(errors="replace"))

    def hb():
        # Heartbeat only while armed — that is what makes stopping the tool a
        # genuine test of the firmware watchdog.
        while not stop.is_set():
            if armed.is_set():
                ser.write(encode(HEARTBEAT))
            time.sleep(0.05)

    threading.Thread(target=rx, daemon=True).start()
    threading.Thread(target=hb, daemon=True).start()

    if args.motor is not None:
        ser.write(encode(ARM))
        armed.set()
        time.sleep(0.2)
        v = max(0, min(1000, int(args.value * 1000)))
        ser.write(encode(SET_MOTOR, bytes([args.motor]) + struct.pack("<H", v)))
        time.sleep(1.0)
        ser.write(encode(DISARM))
        time.sleep(0.2)
        stop.set()
        return 0

    print("commands: arm | disarm | m <index> <0..1> | state | quit")
    try:
        while True:
            line = input("> ").strip().split()
            if not line:
                continue
            cmd = line[0].lower()
            if cmd in ("quit", "q", "exit"):
                break
            elif cmd == "arm":
                ser.write(encode(ARM)); armed.set()
            elif cmd == "disarm":
                armed.clear(); ser.write(encode(DISARM))
            elif cmd == "state":
                ser.write(encode(GET_STATE))
            elif cmd == "m" and len(line) == 3:
                v = max(0, min(1000, int(float(line[2]) * 1000)))
                ser.write(encode(SET_MOTOR,
                                 bytes([int(line[1])]) + struct.pack("<H", v)))
            else:
                print("  ?")
    except (KeyboardInterrupt, EOFError):
        pass
    finally:
        armed.clear()
        ser.write(encode(DISARM))
        stop.set()
        ser.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
