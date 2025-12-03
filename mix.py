import serial
import time

# ---------------------------
# Helper: little-endian 16-bit
# ---------------------------
def le16(lo, hi):
    return lo | (hi << 8)

# ---------------------------
# Decode Cell Voltages Frame
# ---------------------------
def decode_cell_voltages(payload):
    """payload must contain 32 bytes = 16 x 2-byte voltages"""
    if len(payload) != 32:
        print(f"❌ Invalid cell payload len={len(payload)} (expected 32)")
        return

    cells = []
    for i in range(0, 32, 2):
        val = le16(payload[i], payload[i+1])
        cells.append(val)

    print("\n===== RS485 CELL VOLTAGES =====")
    for idx, mV in enumerate(cells, 1):
        print(f"Cell {idx:02d}: {mV} mV  ({mV/1000:.3f} V)")
    print("================================\n")

# ---------------------------
# Main RS485 Frame Reader
# ---------------------------
def main():
    ser = serial.Serial(
        port="COM11",
        baudrate=9600,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        timeout=0.2
    )

    print("RS485 serial opened on COM11 @ 9600")
    print("Reading incoming data...\n")

    buffer = bytearray()

    while True:
        chunk = ser.read(64)
        if chunk:
            buffer.extend(chunk)

            # Minimum frame = 1 + 1 + 1 + 1 + data + 2 + 1
            while len(buffer) >= 6:
                if buffer[0] != 0xDD:
                    buffer.pop(0)
                    continue

                cmd     = buffer[1]
                status  = buffer[2]
                length  = buffer[3]

                frame_len = 1 + 1 + 1 + 1 + length + 2 + 1

                if len(buffer) < frame_len:
                    break  # Wait for more bytes

                frame = buffer[:frame_len]
                del buffer[:frame_len]

                print("RAW:", frame.hex(" ").upper())

                payload = frame[4 : 4 + length]

                if cmd == 0x04:     # CELL_VOLTAGES_CMD
                    decode_cell_voltages(payload)

        time.sleep(0.01)


if __name__ == "__main__":
    main()