import serial
import time

# ==========================
# RS485 CONFIGURE YOUR PORT
# ==========================
PORT = "COM7"      # change this to your COM port
BAUD = 9600        # change to your device baud rate

# ==========================
# OPEN SERIAL PORT
# ==========================
ser = serial.Serial(
    port=PORT,
    baudrate=BAUD,
    bytesize=serial.EIGHTBITS,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    timeout=1
)

print(f"Listening on {PORT} @ {BAUD}...\n")

# ==========================
# READ LOOP
# ==========================
while True:
    raw = ser.read(100)        # read up to 100 bytes
    if raw:
        # Print raw bytes in hex format
        hex_str = " ".join(f"{b:02X}" for b in raw)
        print("RAW:", hex_str)

        # OR if you want length:
        # print(f"{len(raw)} bytes:", hex_str)

    time.sleep(0.05)
