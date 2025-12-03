import serial
import time
import struct
import binascii


# ============================================================
#                     RS485 CONFIG
# ============================================================
PORT    = "COM7"
BAUD    = 9600
TIMEOUT = 1


# ============================================================
#                     CONSTANTS
# ============================================================
RS485_START_CELL   = 0xAA
RS485_END_CELL     = 0x55

RS485_START_BASIC  = 0xBB
RS485_END_BASIC    = 0x66

RS485_START_READ_PARAMETER = 0xFE
RS485_END_READ_PARAMETER   = 0x98

CMD_CELL_VOLTAGES  = 0x01
CMD_BASIC_INFO     = 0x02
CMD_CONFIG_WRITE   = 0x03

STATUS_OK = 0x00

cell_voltages_info = [0x0001]
basic_info         = [0x0002]

WRITE_BUTTON_CLICK = 0
READ_BUTTON_CLICK  = 1


# ============================================================
#                     UTILITY FUNCTIONS
# ============================================================
def rs485_checksum(data: bytes) -> int:
    """16-bit checksum"""
    return sum(data) & 0xFFFF


def le16(lo, hi) -> int:
    """Little-endian 16-bit"""
    return lo | (hi << 8)


def pack_threshold(set_v, rel_v, delay):
    """Pack a 3-value threshold block"""
    return struct.pack("<HHH", set_v, rel_v, delay)


# ============================================================
#                GENERIC PACKET BUILDER
# ============================================================
def build_packet(start, cmd, status, values, end) -> bytes:
    data = b"".join(struct.pack("<H", v) for v in values)
    checksum_data = bytes([status, len(data)]) + data
    checksum = struct.pack(">H", rs485_checksum(checksum_data))

    return (
        bytes([start, cmd, status, len(data)]) +
        data +
        checksum +
        bytes([end])
    )


# ============================================================
#               DECODE CELL VOLTAGES
# ============================================================
def decode_cell_voltages(payload: bytes):
    if len(payload) != 32:
        print(f"❌ Invalid cell payload: {len(payload)} bytes")
        return

    print("\n======== CELL VOLTAGES ========")
    for i in range(0, 32, 2):
        mv = le16(payload[i], payload[i + 1])
        idx = i // 2 + 1
        print(f"Cell {idx:02d}: {mv} mV ({mv/1000:.3f} V)")
    print("================================\n")


# ============================================================
#               DECODE BASIC INFO
# ============================================================
def decode_basic_info(payload: bytes):
    if len(payload) < 31:
        print("❌ Basic info payload too short")
        return

    idx = 0
    def get16():
        nonlocal idx
        v = le16(payload[idx], payload[idx+1])
        idx += 2
        return v

    pv  = get16() / 100
    pc  = get16()
    pc = pc - 65536 if pc > 32767 else pc
    pc /= 100

    rc  = get16() / 1000
    nc  = get16() / 1000
    cyc = get16()
    pd  = get16()

    bal0 = payload[idx]; idx += 1
    bal1 = payload[idx]; idx += 1
    p1   = payload[idx]; idx += 1
    p2   = payload[idx]; idx += 1
    p3   = payload[idx]; idx += 1
    p4   = payload[idx]; idx += 1
    sw   = payload[idx]; idx += 1
    soc  = payload[idx]; idx += 1
    fet  = payload[idx]; idx += 1
    cells= payload[idx]; idx += 1
    ntc_count = payload[idx]; idx += 1

    ntc = []
    for _ in range(ntc_count):
        ntc.append(get16())

    print("\n============ BASIC INFO ============")
    print(f"Pack Voltage  : {pv:.2f} V")
    print(f"Current       : {pc:.2f} A")
    print(f"Remain Cap    : {rc:.2f} Ah")
    print(f"Nominal Cap   : {nc:.2f} Ah")
    print(f"Cycles        : {cyc}")
    print(f"Prod Date     : 0x{pd:04X}")
    print(f"Balance       : {bal0:02X} {bal1:02X}")
    print(f"Protection    : {p1:02X} {p2:02X} {p3:02X} {p4:02X}")
    print(f"SW Version    : {sw}")
    print(f"SOC           : {soc}%")
    print(f"FET           : CHG={fet&1}  DSG={(fet>>1)&1}")
    print(f"Cells         : {cells}")
    print(f"NTC Count     : {ntc_count}")
    print(f"NTC Values    : {ntc}")
    print("====================================\n")


# ============================================================
#              BUILD FULL CONFIG PAYLOAD
# ============================================================
def build_full_payload():
    p = b""

    # (A) BASIC THRESHOLDS
    basic_thresholds = [
        (3655, 3500, 5),
        (1000, 2500, 5),
        (60000, 58000, 5),
        (22000, 25000, 5),
        (0, 5, 5),
        (70, 45, 5),
        (75, 50, 3),
        (65, 55, 5),
        (5000, 15, 5),
        (10000, 30, 5)
    ]
    for t in basic_thresholds:
        p += pack_threshold(*t)

    # (B) ADVANCED PROTECTION
    adv = [
        (20, 1280, 0),
        (40, 400, 0),
        (3900, 8, 0),
        (2000, 16, 0),
    ]
    for t in adv:
        p += pack_threshold(*t)

    # (C) FUNCTION CONFIG
    p += struct.pack("<???????", 0, 0, 0, 0, 0, 0, 0)

    # (D) BALANCE CONFIG
    p += struct.pack("<HHH", 3600, 50, 10)

    # (E) CAPACITY CONFIG
    p += struct.pack("<IIHHB", 10000, 8000, 4150, 3000, 1)

    return p


# ============================================================
#              CONFIG FRAME BUILDER
# ============================================================
def build_config_frame(payload: bytes) -> bytes:
    cmd = CMD_CONFIG_WRITE
    status = 0
    length = len(payload)

    frame = bytearray([0xFD, cmd, status, length])
    frame += payload

    crc = binascii.crc32(bytes([cmd, status, length]) + payload)
    frame += struct.pack("<I", crc & 0xFFFFFFFF)
    frame.append(0x99)
    return frame


# ============================================================
#               RS485 INTERFACE CLASS
# ============================================================
class RS485Interface:
    def __init__(self, port, baud, timeout=1):
        self.ser = serial.Serial(port, baud, timeout=timeout)
        self.buffer = bytearray()

    def send(self, packet: bytes):
        print("TX:", packet.hex().upper())
        self.ser.write(packet)
        self.ser.flush()

    def read_raw(self, size=300):
        return self.ser.read(size)

    def read_frames(self):
        chunk = self.ser.read(64)
        if chunk:
            self.buffer.extend(chunk)

        frames = []
        while len(self.buffer) >= 6:
            if self.buffer[0] != 0xDD:
                self.buffer.pop(0)
                continue

            length = self.buffer[3]
            needed = 4 + length + 2 + 1

            if len(self.buffer) < needed:
                break

            frame = self.buffer[:needed]
            del self.buffer[:needed]
            frames.append(frame)

        return frames

    def close(self):
        self.ser.close()


# ============================================================
#              RAW READ FRAME PARSER
# ============================================================
def parse_rs485_frame(frame: bytes):
    if len(frame) < 8:
        return None, "Frame too short"

    if frame[0] != 0xDD:
        return None, "Invalid start byte"

    cmd    = frame[1]
    status = frame[2]
    length = frame[3]

    expected = 4 + length + 2 + 2
    if len(frame) < expected:
        return None, "Incomplete frame"

    if frame[-2:] != bytes([0x77, 0x00]):
        return None, "Invalid end bytes"

    data = frame[4:4+length]
    crc  = frame[4+length:4+length+2]

    return {
        "cmd": cmd,
        "status": status,
        "length": length,
        "data_raw": data,
        "checksum": crc
    }, None


# ============================================================
#                DECODE BMS CONFIG STRUCT
# ============================================================
def decode_bms_config(data):
    fmt = (
        "<"
        "H"*27 +
        "H"*12 +
        "B"*7 +
        "H"*3 +
        "I"*2 +
        "H"*2 +
        "B"
    )

    try:
        values = struct.unpack(fmt, data)
    except struct.error:
        return {"ERROR": "Data size mismatch"}

    names = (
        # Basic
        [f"Basic_{i}" for i in range(27)] +
        # Advanced
        [f"Adv_{i}" for i in range(12)] +
        # Function
        [f"Func_{i}" for i in range(7)] +
        # Balance
        ["cellOpenVoltage", "balanceDriftVoltage", "ipsOffDelay"] +
        # Capacity
        ["nominalCapacity", "cycleCapacity", "fullSetVoltage", "endOfVoltage", "selfDischargeRate"]
    )

    return dict(zip(names, values))


# ============================================================
#                       MAIN LOOP
# ============================================================
def main():
    rs = RS485Interface(PORT, BAUD, TIMEOUT)

    try:
        while True:

            # ---------------- CELL VOLTAGES ----------------
            pkt = build_packet(RS485_START_CELL, CMD_CELL_VOLTAGES,
                               STATUS_OK, cell_voltages_info, RS485_END_CELL)
            rs.send(pkt)

            for frame in rs.read_frames():
                if frame[1] == 0x04:
                    decode_cell_voltages(frame[4:4 + frame[3]])

            time.sleep(1)

            # ---------------- BASIC INFO ----------------
            pkt = build_packet(RS485_START_BASIC, CMD_BASIC_INFO,
                               STATUS_OK, basic_info, RS485_END_BASIC)
            rs.send(pkt)

            for frame in rs.read_frames():
                if frame[1] == 0x05:
                    decode_basic_info(frame[4:4 + frame[3]])

            time.sleep(1)

            # ---------------- WRITE CONFIG ----------------
            if WRITE_BUTTON_CLICK:
                print("\n=== SENDING CONFIG WRITE ===")
                payload = build_full_payload()
                frame = build_config_frame(payload)
                rs.send(frame)
                time.sleep(2)

            # ---------------- READ CONFIG ----------------
            if READ_BUTTON_CLICK:
                pkt = build_packet(
                    RS485_START_READ_PARAMETER,
                    CMD_BASIC_INFO,
                    STATUS_OK,
                    basic_info,
                    RS485_END_READ_PARAMETER
                )
                rs.send(pkt)

                raw = rs.read_raw(300)
                if not raw:
                    continue

                print("RAW:", raw.hex(" ").upper())
                frame, err = parse_rs485_frame(raw)
                if err:
                    print("ERROR:", err)
                    continue

                cfg = decode_bms_config(frame["data_raw"])
                print("\n=========== BMS CONFIG ===========")
                for k, v in cfg.items():
                    print(f"{k:<25}: {v}")
                print("===================================\n")

                time.sleep(2)

    except KeyboardInterrupt:
        print("Stopped by user.")
    finally:
        rs.close()


if __name__ == "__main__":
    main()
