import serial
import time
import struct
import binascii
import csv


# ============================================================
#                     RS485 CONFIG
# ============================================================
PORT    = "COM7"
BAUD    = 9600
TIMEOUT = 1


# ============================================================
#                     CONSTANTS
# ============================================================
RS485_START_CELL  = 0xAA
RS485_END_CELL    = 0x55

RS485_START_BASIC = 0xBB
RS485_END_BASIC   = 0x66
RS485_START_READ_PARAMETER   = 0x0FE
RS485_END_READ_PARAMETER   = 0x98

RS485_START_WRITE_CAL  = 0x0CC
RS485_END_WRITE_CAL   = 0x77

CMD_CELL_VOLTAGES = 0x01
CMD_BASIC_INFO    = 0x02
CMD_CONFIG_WRITE  = 0x03

STATUS_OK         = 0x00

# Test Data
cell_voltages_info = [0x0001]
basic_info         = [0x0002]


WRITE_BUTTON_CLICK  = 0
READ_BUTTON_CLICK  = 0

CURRENT_AND_VOLT_CALIBRATE = 0
calibrate_info = [0x10,0x20]

GET_ALL_LOG = 1

RS485_START_WRITE_LOG  = 0xDD
RS485_END_WRITE_LOG   = 0x88

# ============================================================
#                  UTILITY FUNCTIONS
# ============================================================
def rs485_checksum(data: bytes) -> int:
    """16-bit checksum"""
    return sum(data) & 0xFFFF


def le16(lo, hi):
    """Little-endian decode"""
    return lo | (hi << 8)


def pack_threshold(set_v, release_v, delay_v):
    """Pack a triple threshold block"""
    return struct.pack("<HHH", set_v, release_v, delay_v)


# ============================================================
#                GENERIC PACKET BUILDER
# ============================================================
def build_packet(start_byte, cmd, status, data_list, end_byte) -> bytes:
    data_buff = b"".join(struct.pack("<H", v) for v in data_list)
    data_length = len(data_buff)

    checksum_data = bytes([status, data_length]) + data_buff
    checksum = rs485_checksum(checksum_data)
    checksum_be = struct.pack(">H", checksum)

    packet = (
        bytes([start_byte, cmd, status, data_length]) +
        data_buff +
        checksum_be +
        bytes([end_byte])
    )
    return packet


# ============================================================
#                BASIC INFORMATION DECODER
# ============================================================
def decode_cell_voltages(payload: bytes):
    if len(payload) != 32:
        print(f"❌ Invalid cell payload length: {len(payload)}")
        return

    cells = [le16(payload[i], payload[i+1]) for i in range(0, 32, 2)]

    print("\n===== CELL VOLTAGES =====")
    for i, mV in enumerate(cells, 1):
        print(f"Cell {i:02d}: {mV} mV ({mV/1000:.3f} V)")
    print("=========================\n")


def decode_basic_info(payload: bytes):
    if len(payload) < 31:
        print("❌ Payload too short")
        return

    i = 0
    pv = le16(payload[i], payload[i+1]); i += 2
    pc = le16(payload[i], payload[i+1]); i += 2
    if pc > 32767:
        pc -= 65536

    rc  = le16(payload[i], payload[i+1]); i += 2
    nc  = le16(payload[i], payload[i+1]); i += 2
    cyc = le16(payload[i], payload[i+1]); i += 2
    pd  = le16(payload[i], payload[i+1]); i += 2

    bal0 = payload[i]; i += 1
    bal1 = payload[i]; i += 1
    p1   = payload[i]; i += 1
    p2   = payload[i]; i += 1
    p3   = payload[i]; i += 1
    p4   = payload[i]; i += 1

    sw  = payload[i]; i += 1
    soc = payload[i]; i += 1

    fet = payload[i]; i += 1
    cells = payload[i]; i += 1
    ntc_count = payload[i]; i += 1

    ntc = []
    for _ in range(ntc_count):
        if i + 1 < len(payload):
            ntc.append(le16(payload[i], payload[i+1]))
        i += 2

    print("\n===== BASIC INFO =====")
    print(f"Pack Voltage : {pv/100:.2f} V")
    print(f"Current      : {pc/100:.2f} A")
    print(f"Remain Cap   : {rc/1000:.2f} Ah")
    print(f"Nominal Cap  : {nc/1000:.2f} Ah")
    print(f"Cycles       : {cyc}")
    print(f"Prod Date    : 0x{pd:04X}")
    print(f"Balance      : {bal0:02X} {bal1:02X}")
    print(f"Protection   : {p1:02X} {p2:02X} {p3:02X} {p4:02X}")
    print(f"SW Version   : {sw}")
    print(f"SOC          : {soc}%")
    print(f"FET Status   : CHG={fet&1} DSG={(fet>>1)&1}")
    print(f"Cells        : {cells}")
    print(f"NTC Count    : {ntc_count}")
    print(f"NTC Values   : {ntc}")
    print("=======================\n")


# ============================================================
#        BUILD FULL CONFIG PAYLOAD FOR WRITE CONFIG
# ============================================================
def build_full_payload():
    p = b""

    # 1. Basic thresholds
    p += pack_threshold(3657, 3500, 5)
    p += pack_threshold(1000, 2500, 5)
    p += pack_threshold(60000, 58000, 5)
    p += pack_threshold(22000, 25000, 5)
    p += pack_threshold(0, 5, 5)
    p += pack_threshold(70, 45, 5)
    p += pack_threshold(75, 50, 3)
    p += pack_threshold(65, 55, 5)
    p += pack_threshold(5000, 15, 5)
    p += pack_threshold(10000, 30, 5)

    # 2. Advanced protection
    p += pack_threshold(20, 1280, 0)
    p += pack_threshold(40, 400, 0)
    p += pack_threshold(3900, 8, 0)
    p += pack_threshold(2000, 16, 0)

    # 3. Function config
    p += struct.pack("<???????", 0, 0, 0, 0, 0, 0, 0)

    # 4. Balancing
    p += struct.pack("<HHH", 3600, 50, 10)

    # 6. Capacity config
    p += struct.pack("<IIHHB",
                     10000, 8000, 4150, 3000, 1)

    return p


# ============================================================
#               BUILD FINAL CONFIG FRAME
# ============================================================
def build_config_frame(payload):
    CMD     = CMD_CONFIG_WRITE
    STATUS  = 0x00
    LENGTH  = len(payload)

    frame = bytearray([0xFD, CMD, STATUS, LENGTH])
    frame += payload

    crc = binascii.crc32(bytes([CMD, STATUS, LENGTH]) + payload) & 0xFFFFFFFF
    frame += struct.pack("<I", crc)
    frame.append(0x99)

    return frame


# ============================================================
#              RS485 INTERFACE CLASS
# ============================================================

class RS485Interface:
    def __init__(self, port, baud, timeout=1):
        self.ser = serial.Serial(port, baud, timeout=timeout)
        self.buffer = bytearray()

    def send(self, packet):
        self.ser.write(packet)
        self.ser.flush()
        print("TX:", packet.hex().upper())

    def read(self, size=256):
        """Raw read for READ_PARAMETER frames"""
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

            cmd = self.buffer[1]
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

 

# ==========================================================
# 1) PARSE RAW RS485 FRAME
# ==========================================================
def parse_rs485_frame(frame):

    if len(frame) < 8:  # minimum usable frame
        return None, "Frame too short"

    # HEADER
    if frame[0] != 0xDD:
        return None, "Invalid start byte"

    cmd = frame[1]
    status = frame[2]
    length = frame[3]

    # expected total frame size:
    expected_len = 4 + length + 2 + 2   # start..len + DATA + checksum(2) + END(2)

    if len(frame) < expected_len:
        return None, "Incomplete frame"

    # END BYTE MUST BE 77 00
    if not (frame[-2] == 0x77 and frame[-1] == 0x00):
        return None, "Invalid end byte (expect 0x77 0x00)"

    data_start = 4
    data_end   = 4 + length

    data = frame[data_start:data_end]
    checksum = frame[data_end:data_end + 2]

    return {
        "cmd": cmd,
        "status": status,
        "length": length,
        "data_raw": data,
        "checksum": checksum
    }, None



# ==========================================================
# 2) DECODE BmsConfig_struct (Matches Your C Struct)
# ==========================================================
def decode_bms_config(data):

    fmt = "<" + \
          "H"*27 + \
          "H"*12 + \
          "B"*7 + \
          "H"*3 + \
          "I"*2 + \
          "H"*2 + \
          "B"

    try:
        values = struct.unpack(fmt, data)
    except struct.error:
        return {"ERROR": "Data size mismatch for BMS config struct"}

    names = [
        # (A) BASIC PROTECTION
        "Cell_OV_set","Cell_OV_rel","Cell_OV_delay",
        "Cell_UV_set","Cell_UV_rel","Cell_UV_delay",
        "Pack_OV_set","Pack_OV_rel","Pack_OV_delay",
        "Pack_UV_set","Pack_UV_rel","Pack_UV_delay",
        "Chg_UT_set","Chg_UT_rel","Chg_UT_delay",
        "Dsg_OT_set","Dsg_OT_rel","Dsg_OT_delay",
        "Chg_OT_set","Chg_OT_rel","Chg_OT_delay",
        "Chg_OC_set","Chg_OC_rel","Chg_OC_delay",
        "Dsg_OC_set","Dsg_OC_rel","Dsg_OC_delay",

        # (B) ADVANCED PROTECTION
        "Dsg_OC2_set","Dsg_OC2_rel","Dsg_OC2_delay",
        "SC_set","SC_rel","SC_delay",
        "HCell_OV_set","HCell_OV_rel","HCell_OV_delay",
        "LCell_UV_set","LCell_UV_rel","LCell_UV_delay",

        # (C) FUNCTION CONFIG
        "load_en","balance_en","chg_balance",
        "ntc1","ntc2","ntc3","ntc4",

        # (D) BALANCE CONFIG
        "cellOpenVoltage",
        "balanceDriftVoltage",
        "ipsOffDelay",

        # (E) CAPACITY CONFIG
        "nominalCapacity",
        "cycleCapacity",
        "fullSetVoltage",
        "endOfVoltage",
        "selfDischargeRate"
    ]

    return dict(zip(names, values))
#######################################################################
#logging

START_BYTE  = 0x12
CMD_BYTE    = 0x34
STATUS_BYTE = 0x01

PACKET_SIZE = 51
CSV_FILE = "fix_time87.csv"


# ---------------------------------------------------------
# MOS State Decode
# ---------------------------------------------------------
def decode_mos_state(code: int) -> str:
    return {
        3: "ALL ON",
        2: "CHG ON",
        1: "DSG ON",
        0: "ALL OFF"
    }.get(code, "UNKNOWN")


# ---------------------------------------------------------
# FAULT BIT DEFINITIONS
# ---------------------------------------------------------
FAULT_MAP = {
    1 << 0:  "OverVoltage",
    1 << 1:  "UnderVoltage",
    1 << 2:  "UnderPackVolt",
    1 << 3:  "OverPackVolt",
    1 << 4:  "OverCharge",
    1 << 5:  "OverDischarge",
    1 << 6:  "ThermalRunaway",
    1 << 7:  "CellDiff",
    1 << 8:  "OnBoard_UnderTemp",
    1 << 9:  "OnBoard_OverTemp",
    1 << 10: "Ext_OverTemp_Charge",
    1 << 11: "Ext_OverTemp_Discharge",
    1 << 12: "Balancing_OK",
    1 << 13: "ShortCircuit",
    1 << 14: "UnderTemp",
    1 << 15: "HardwareFault"
}

def decode_faults(fault_word):
    lst = [name for bit, name in FAULT_MAP.items() if fault_word & bit]
    return lst if lst else ["No Fault"]


# ---------------------------------------------------------
# RTC TIME DECODE (Correct!)
# raw_time  = 0xMMHHDDWW   (min, hour, day, week)
# raw_time2 = 0xYYMM0000   (year, month)
# ---------------------------------------------------------
def decode_time(raw_time, raw_time2):
    minute = (raw_time >> 24) & 0xFF
    hour   = (raw_time >> 16) & 0xFF
    day    = (raw_time >>  8) & 0xFF
    week   = (raw_time >>  0) & 0xFF

    month  = (raw_time2 >> 24) & 0xFF
    year   = (raw_time2 >> 16) & 0xFF
    year  += 2000

    formatted = f"{day:02d}-{month:02d}-{year} {hour:02d}:{minute:02d}"

    return formatted, {
        "day": day,
        "month": month,
        "year": year,
        "hour": hour,
        "minute": minute,
        "week": week
    }


# ---------------------------------------------------------
# CSV Create Header
# ---------------------------------------------------------
def init_csv():
    try:
        with open(CSV_FILE, "x", newline="") as f:
            writer = csv.writer(f)
            writer.writerow([
                "No.",
                "Time",
                "Event Types",
                "PackVol(V)",
                "PackCur(A)",
                "RmCap(Ah)",
                "FCC(Ah)",
                "MaxCellVol(V)",
                "MaxCellNum",
                "MinCellVol(V)",
                "MinCellNum",
                "MaxT(°C)",
                "MinT(°C)",
                "MOS State"
            ])
    except FileExistsError:
        pass


# ---------------------------------------------------------
# CSV Logging
# ---------------------------------------------------------
def log_to_csv(data):
    with open(CSV_FILE, "a", newline="") as f:
        writer = csv.writer(f)
        writer.writerow([
            data["tick"],
            data["datetime"],
            ", ".join(data["faultNames"]),
            data["packVolt"],
            data["packCurrent"],
            data["RmCap"],
            data["FcCap"],
            data["maxCellVoltage"],
            data["maxVoltage_cellno"],
            data["minCellVoltage"],
            data["minVoltage_cellno"],
            data["maxTemp"],
            data["minTemp"],
            data["mosStateText"],
        ])


# ---------------------------------------------------------
# RS485 Frame Decode
# ---------------------------------------------------------
def decode_rs485_frame(frame: bytes):

    idx = 4  # Skip 0x12, 0x34, 0x01

    def u8():
        nonlocal idx
        v = frame[idx]
        idx += 1
        return v

    def u16():
        nonlocal idx
        v = int.from_bytes(frame[idx:idx+2], "little")
        idx += 2
        return v

    def s16():
        nonlocal idx
        v = int.from_bytes(frame[idx:idx+2], "little", signed=True)
        idx += 2
        return v

    def u32():
        nonlocal idx
        v = int.from_bytes(frame[idx:idx+4], "little")
        idx += 4
        return v

    data = {}

    data["tick"]               = u32()
    data["time"]               = u32()
    data["time2"]              = u32()
    data["faults"]             = u16()
    data["packVolt"]           = u16() / 1000.0
    data["packCurrent"]        = s16() / 100.0
    data["RmCap"]              = u8()
    data["FcCap"]              = u8()
    data["maxCellVoltage"]     = u16() / 1000.0
    data["maxVoltage_cellno"]  = u8()
    data["minCellVoltage"]     = u16() / 1000.0
    data["minVoltage_cellno"]  = u8()
    data["maxTemp"]            = s16() / 100.0
    data["minTemp"]            = s16() / 100.0
    data["mosFet"]             = u8()

    return data



# ============================================================
#                      MAIN PROGRAM
# ============================================================
def main():
    print("Reading RS485 data...\n")
    init_csv()
    print("Reading RS485 data...\n")
    rs = RS485Interface(PORT, BAUD, TIMEOUT)

    try:
        while True:
            
            cmdx = int(input("command: "))
            
            
            if cmdx == 9:
                # -------- Send Cell Voltage Request --------
                pkt = build_packet(RS485_START_CELL, CMD_CELL_VOLTAGES,
                               STATUS_OK, cell_voltages_info, RS485_END_CELL)
                rs.send(pkt)

                for frame in rs.read_frames():
                   cmd = frame[1]
                   payload = frame[4:4 + frame[3]]
                   if cmd == 0x04:
                    decode_cell_voltages(payload)
                time.sleep(1)
 

            if cmdx == 9:
               # -------- Send Basic Info Request --------
                pkt = build_packet(RS485_START_BASIC, CMD_BASIC_INFO,
                               STATUS_OK, basic_info, RS485_END_BASIC)
                rs.send(pkt)

                for frame in rs.read_frames():
                   cmd = frame[1]
                   payload = frame[4:4 + frame[3]]
                   if cmd == 0x05:
                     decode_basic_info(payload)
            time.sleep(1)
            print(cmdx)

            # -------- Send Full Config Write --------
            if cmdx == 5:
                payload = build_full_payload()
                frame = build_config_frame(payload)
                print("\n=== SENDING FULL CONFIG FRAME ===")
                print(frame.hex(" ").upper(), "\n")
                rs.send(frame)
                time.sleep(2)
            
             # -------- Send Basic Info Request --------
            if cmdx == 6:
                pkt = build_packet(RS485_START_READ_PARAMETER, CMD_BASIC_INFO,STATUS_OK, basic_info, RS485_END_READ_PARAMETER)
                rs.send(pkt) 
                time.sleep(2) 
                raw = rs.read(300)
                if not raw:
                    continue

                print("RAW:", " ".join(f"{b:02X}" for b in raw))

                frame, err = parse_rs485_frame(raw)
                if err:
                    print("ERROR:", err)
                    continue
                cfg = decode_bms_config(frame["data_raw"])
                print("\n========== DECODED BMS CONFIG DATA ==========")
                for k, v in cfg.items():
                    print(f"{k:<22} : {v}")
                print("cmd = 0 ==============================================\n")
                time.sleep(2)

            if cmdx == 3:
                pkt = build_packet(RS485_START_WRITE_CAL, CMD_BASIC_INFO,STATUS_OK, calibrate_info, RS485_END_WRITE_CAL)
                rs.send(pkt) 
                time.sleep(2) 
            
            if cmdx == 1:
                pkt = build_packet(RS485_START_WRITE_LOG, CMD_BASIC_INFO,STATUS_OK, calibrate_info, RS485_END_WRITE_LOG)
                rs.send(pkt) 
                time.sleep(2)
                while True:
                    b = rs.read(1)
                    if not b:
                        continue
                    if b[0] != START_BYTE:
                        continue
                    header = rs.read(2)
                    if len(header) != 2 or header[0] != CMD_BYTE or header[1] != STATUS_BYTE:
                        continue
                    rest = rs.read(PACKET_SIZE - 3)
                    if len(rest) != (PACKET_SIZE - 3):
                        print("cmd = 0!")
                        cmdx = 0
                        break
                    frame = b + header + rest
                    parsed = decode_rs485_frame(frame)
                    # Decode datetime
                    parsed["datetime"], _ = decode_time(parsed["time"], parsed["time2"])
                    parsed["mosStateText"] = decode_mos_state(parsed["mosFet"])
                    parsed["faultNames"]   = decode_faults(parsed["faults"])

                    # RAW PRINTS
                    print("\n-------------------------------------")
                    print(f"Decoded Time: {parsed['datetime']}")
                    print(f"RAW time  = {parsed['time']}   (0x{parsed['time']:08X})")
                    print(f"RAW time2 = {parsed['time2']}   (0x{parsed['time2']:08X})")
                    print(f"Tick: {parsed['tick']}")
                    print(f"Faults: {parsed['faultNames']}")
                    print(f"Pack V: {parsed['packVolt']:.3f} V")
                    print(f"Pack I: {parsed['packCurrent']:.2f} A")
                    print(f"MOS State: {parsed['mosFet']} -> {parsed['mosStateText']}")
                    # CSV log
                    log_to_csv(parsed)
                 



    except KeyboardInterrupt:
        print("Stopped by user.")

    finally:
        rs.close()


# ============================================================
#                       ENTRY POINT
# ============================================================
if __name__ == "__main__":
    main()
