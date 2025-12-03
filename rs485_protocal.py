import serial
import time
import struct

# =============================
# RS485 CONFIG
# =============================
PORT = "COM7"
BAUD = 9600
TIMEOUT = 1

# =============================
# Constants
# =============================
RS485_START_BYTE_CELL  = 0xAA
RS485_END_BYTE_CELL    = 0x55

RS485_START_BYTE_BASIC = 0xBB
RS485_END_BYTE_BASIC   = 0x66

CMD_CELL_VOLTAGES = 0x01
CMD_BASIC_INFO    = 0x02
STATUS_OK         = 0x00

# Test data
cell_voltages_info = [0x0001]
basic_info         = [0x0002]


# =============================
# Helper Functions
# =============================
def rs485_checksum(byte_array: bytes) -> int:
    """Calculate 16-bit checksum."""
    return sum(byte_array) & 0xFFFF


def le16(lo, hi):
    """Little-endian 16-bit decode."""
    return lo | (hi << 8)


def build_packet(start_byte, cmd, status, data_list, end_byte) -> bytes:
    """Generic packet builder for RS485."""
    data_buff = b"".join(struct.pack("<H", v) for v in data_list)
    data_length = len(data_buff)

    checksum_buffer = bytes([status, data_length]) + data_buff
    chksum = rs485_checksum(checksum_buffer)
    checksum_be = struct.pack(">H", chksum)

    packet = (
        bytes([start_byte, cmd, status, data_length])
        + data_buff
        + checksum_be
        + bytes([end_byte])
    )
    return packet


# =============================
# Decoders
# =============================
def decode_cell_voltages(payload: bytes):
    if len(payload) != 32:
        print(f"❌ Invalid cell payload len={len(payload)} (expected 32)")
        return

    cells = [le16(payload[i], payload[i+1]) for i in range(0, 32, 2)]

    print("\n===== RS485 CELL VOLTAGES =====")
    for idx, mV in enumerate(cells, 1):
        print(f"Cell {idx:02d}: {mV} mV ({mV/1000:.3f} V)")
    print("================================\n")


def decode_basic_info(payload):
    if len(payload) < 31:
        print("❌ Payload too short:", len(payload))
        return

    i = 0
    packVoltage = le16(payload[i], payload[i+1]); i += 2
    packCurrent = le16(payload[i], payload[i+1]); i += 2
    if packCurrent > 32767:
        packCurrent -= 65536  # convert signed int16

    remainingCap = le16(payload[i], payload[i+1]); i += 2
    nominalCap   = le16(payload[i], payload[i+1]); i += 2
    cycles       = le16(payload[i], payload[i+1]); i += 2
    productionDate = le16(payload[i], payload[i+1]); i += 2

    balance0 = payload[i]; i += 1
    balance1 = payload[i]; i += 1
    prot1 = payload[i]; i += 1
    prot2 = payload[i]; i += 1
    prot3 = payload[i]; i += 1
    prot4 = payload[i]; i += 1

    swVersion = payload[i]; i += 1
    soc = payload[i]; i += 1

    fetStatus = payload[i]; i += 1
    cells = payload[i]; i += 1
    ntcCount = payload[i]; i += 1

    ntc = []
    for n in range(ntcCount):
        if i + 1 < len(payload):
            ntc.append(le16(payload[i], payload[i+1]))
        i += 2

    print("\n===== RS485 BASIC INFO =====")
    print(f"Pack Voltage     : {packVoltage/100:.2f} V")
    print(f"Pack Current     : {packCurrent/100:.2f} A")
    print(f"Remaining Cap    : {remainingCap/1000:.2f} Ah")
    print(f"Nominal Cap      : {nominalCap/1000:.2f} Ah")
    print(f"Cycles           : {cycles}")
    print(f"Production Date  : 0x{productionDate:04X}")
    print(f"Balance0         : 0x{balance0:02X}")
    print(f"Balance1         : 0x{balance1:02X}")
    print(f"Protection       : {prot1:02X} {prot2:02X} {prot3:02X} {prot4:02X}")
    print(f"SW Version       : {swVersion}")
    print(f"SOC              : {soc}%")
    print(f"FET Status       : CHG={(fetStatus&1)} DSG={(fetStatus>>1)&1}")
    print(f"Cells In Series  : {cells}")
    print(f"NTC Count        : {ntcCount}")
    print(f"NTC Values       : {ntc}")
    print("================================\n")


# =============================
# RS485 Interface Class
# =============================
class RS485Interface:
    def __init__(self, port, baud, timeout=1):
        self.ser = serial.Serial(port, baud, timeout=timeout)
        self.buffer = bytearray()

    def send_packet(self, packet: bytes):
        self.ser.write(packet)
        self.ser.flush()
        print("TX:", packet.hex().upper())

    def read_frames(self):
        chunk = self.ser.read(64)
        if chunk:
            self.buffer.extend(chunk)

        frames = []

        while len(self.buffer) >= 6:
            if self.buffer[0] != 0xDD:
                self.buffer.pop(0)
                continue

            cmd = self.buffer[1]
            status = self.buffer[2]
            length = self.buffer[3]

            frame_len = 1 + 1 + 1 + 1 + length + 2 + 1

            if len(self.buffer) < frame_len:
                break

            frame = self.buffer[:frame_len]
            del self.buffer[:frame_len]
            frames.append(frame)

        return frames

    def close(self):
        self.ser.close()


# =============================
# Main Loop
# =============================
def main():
    rs485 = RS485Interface(PORT, BAUD, TIMEOUT)

    try:
        while True:
            # -----------------------------
            # 1. SEND CELL VOLTAGE CMD
            # -----------------------------
            pkt = build_packet(
                RS485_START_BYTE_CELL,
                CMD_CELL_VOLTAGES,
                STATUS_OK,
                cell_voltages_info,
                RS485_END_BYTE_CELL
            )
            rs485.send_packet(pkt)

            for frame in rs485.read_frames() or []:
                print("RAW:", frame.hex(" ").upper())
                cmd = frame[1]
                payload = frame[4 : 4 + frame[3]]
                if cmd == 0x04:
                    decode_cell_voltages(payload)

            time.sleep(1)

            # -----------------------------
            # 2. SEND BASIC INFO CMD
            # -----------------------------
            pkt = build_packet(
                RS485_START_BYTE_BASIC,
                CMD_BASIC_INFO,
                STATUS_OK,
                basic_info,
                RS485_END_BYTE_BASIC
            )
            rs485.send_packet(pkt)

            for frame in rs485.read_frames() or []:
                print("RAW:", frame.hex(" ").upper())
                cmd = frame[1]
                payload = frame[4 : 4 + frame[3]]

                if cmd == 0x05:     # <-- CHANGE IF NEEDED
                    decode_basic_info(payload)

            time.sleep(2)

    except KeyboardInterrupt:
        print("Stopped by user.")

    finally:
        rs485.close()


if __name__ == "__main__":
    main()
