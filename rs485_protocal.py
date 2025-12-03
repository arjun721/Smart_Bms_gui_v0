import serial
import time
import struct

# =============================
# RS485 CONFIG
# =============================
PORT = "COM7"
BAUD = 9600

ser = serial.Serial(PORT, BAUD, timeout=1)

# =============================
# CONSTANTS (same as C)
# =============================
RS485_START_BYTE = 0xAA
RS485_END_BYTE   = 0x55
CELL_VOLTAGES_CMD = 0x01
RS485_STATUS_OK   = 0x00

# test data = one uint16 (2 bytes)
cell_voltages_info = [0x0001]

# correct data length = number of bytes in data_buff
DATA_LENGTH = len(cell_voltages_info) * 2    # 1 value * 2 bytes = 2


def rs485_checksum(byte_array):
    return sum(byte_array) & 0xFFFF


def build_cell_packet():

    # create 2-byte uint16 data
    data_buff = b"".join(struct.pack("<H", v) for v in cell_voltages_info)

    # checksum buffer: status + data_length + data
    checksum_buffer = bytes([
        RS485_STATUS_OK,
        DATA_LENGTH
    ]) + data_buff

    # calculate checksum (same as C)
    chksum = rs485_checksum(checksum_buffer)

    # checksum big-endian (revMemcpy in C)
    checksum_be = struct.pack(">H", chksum)

    # final packet
    packet = bytes([
        RS485_START_BYTE,
        CELL_VOLTAGES_CMD,
        RS485_STATUS_OK,
        DATA_LENGTH
    ]) + data_buff + checksum_be + bytes([RS485_END_BYTE])

    return packet


print("Sending RS485 data every 1 sec...")

try:
    while True:
        pkt = build_cell_packet()
        ser.write(pkt)
        ser.flush()
        print("TX:", pkt.hex().upper())
        time.sleep(1)

except KeyboardInterrupt:
    print("Stopped.")

finally:
    ser.close()
