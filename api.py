#!/usr/bin/env python3
"""
Single-file Flask app exposing 4 endpoints:
 /5  -> send full config write
 /6  -> read BMS config (basic info / read parameter)
 /9  -> request pack info (cell voltages + basic info)
 /1  -> start reading logs (returns JSON array, also appends CSV)

CORS allowed for http://localhost:9393
"""

import time
import struct
import binascii
import csv
import threading
from flask import Flask, jsonify, request, make_response
import serial
from flask_cors import CORS
import serial
from serial.tools import list_ports  # ← add this
from flask_cors import CORS
PREFERRED_PORTS = ["COM4", "COM6"]  # ports you like first
PORT    = "AUTO"
BAUD    = 9600
DEFAULT_TIMEOUT = 1.0  # seconds to wait for responses in endpoints
# RS485 framing bytes used in original script
RS485_START_CELL  = 0xAA
RS485_END_CELL    = 0x55

RS485_START_BASIC = 0xBB
RS485_END_BASIC   = 0x66
RS485_START_READ_PARAMETER   = 0x0FE
RS485_END_READ_PARAMETER   = 0x98

RS485_START_WRITE_CAL  = 0x0CC
RS485_END_WRITE_CAL   = 0x77

RS485_START_WRITE_LOG  = 0xDD
RS485_END_WRITE_LOG   = 0x88

CMD_CELL_VOLTAGES = 0x01
CMD_BASIC_INFO    = 0x02
CMD_CONFIG_WRITE  = 0x03

STATUS_OK         = 0x00

# Logging packet constants (from your code)
START_BYTE  = 0x12
CMD_BYTE    = 0x34
STATUS_BYTE = 0x01
PACKET_SIZE = 51
CSV_FILE = "fix_time87.csv"


def auto_detect_port(preferred_ports=None):
  
    if preferred_ports is None:
        preferred_ports = PREFERRED_PORTS

    ports = list(list_ports.comports())
    if not ports:
        raise RuntimeError("No serial ports found on this system.")

    # 1) try preferred names
    for p in ports:
        if p.device in preferred_ports:
            print(f"[AUTO PORT] Using preferred port: {p.device}")
            return p.device

    # 2) otherwise just pick the first one
    chosen = ports[0].device
    print(f"[AUTO PORT] Preferred ports not found, using first available: {chosen}")
    return chosen

# ----------------------------
# Helper binary utilities
# ----------------------------
def rs485_checksum(data: bytes) -> int:
    """16-bit checksum (sum & 0xFFFF)"""
    return sum(data) & 0xFFFF

def le16(lo, hi):
    """Little-endian decode of two bytes (already integers)"""
    return lo | (hi << 8)

def pack_threshold(set_v, release_v, delay_v):
    return struct.pack("<HHH", set_v, release_v, delay_v)

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

def decode_cell_voltages(payload: bytes):
    """Return dict with list of cell mV values and voltages in V"""
    if len(payload) != 32:
        return {"error": "Invalid cell payload length", "length": len(payload)}
    cells = [le16(payload[i], payload[i+1]) for i in range(0, 32, 2)]
    return {
        "cells_mV": cells,
        "cells_V": [m/1000.0 for m in cells]
    }

def decode_basic_info(payload: bytes):
    """Return dict of basic info fields (same as working CLI script)"""

    if len(payload) < 31:
        return {"error": "Payload too short", "length": len(payload)}

    i = 0

    def r16():
        nonlocal i
        v = le16(payload[i], payload[i+1])
        i += 2
        return v

    # 16-bit fields
    pv  = r16()
    pc  = r16()
    if pc > 32767:
        pc -= 65536
    rc  = r16()
    nc  = r16()
    cyc = r16()
    pd  = r16()

    # 8-bit fields
    bal0 = payload[i]; i += 1
    bal1 = payload[i]; i += 1
    p1   = payload[i]; i += 1
    p2   = payload[i]; i += 1
    p3   = payload[i]; i += 1
    p4   = payload[i]; i += 1

    sw  = payload[i]; i += 1
    soc = payload[i]; i += 1

    fet   = payload[i]; i += 1
    cells = payload[i]; i += 1
    ntc_count = payload[i]; i += 1

    # NTC values (2 bytes each, little-endian)
    ntc = []
    for _ in range(ntc_count):
        if i + 1 < len(payload):
            ntc.append(le16(payload[i], payload[i+1]))
        i += 2

    return {
        "packVoltage_V":       pv / 100.0,
        "current_A":           pc / 100.0,
        "remain_capacity_Ah":  rc / 1000.0,
        "nominal_capacity_Ah": nc / 1000.0,
        "cycles":              cyc,
        "prod_date":           f"0x{pd:04X}",
        "balance_bytes":       [bal0, bal1],
        "protection_bytes":    [p1, p2, p3, p4],
        "sw_version":          sw,
        "soc_percent":         soc,
        "fet":                 fet,
        "cells_count":         cells,
        "ntc_count":           ntc_count,
        "ntc_values":          ntc,
    }

def build_config_frame(payload):
    CMD     = CMD_CONFIG_WRITE
    STATUS  = 0x00
    LENGTH  = len(payload)
    frame = bytearray([0xFD, CMD, STATUS, LENGTH])
    frame += payload
    crc = binascii.crc32(bytes([CMD, STATUS, LENGTH]) + payload) & 0xFFFFFFFF
    frame += struct.pack("<I", crc)
    frame.append(0x99)
    return bytes(frame)

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

def log_to_csv(data):
    init_csv()
    with open(CSV_FILE, "a", newline="") as f:
        writer = csv.writer(f)
        writer.writerow([
            data.get("tick"),
            data.get("datetime"),
            ", ".join(data.get("faultNames", [])),
            data.get("packVolt"),
            data.get("packCurrent"),
            data.get("RmCap"),
            data.get("FcCap"),
            data.get("maxCellVoltage"),
            data.get("maxVoltage_cellno"),
            data.get("minCellVoltage"),
            data.get("minVoltage_cellno"),
            data.get("maxTemp"),
            data.get("minTemp"),
            data.get("mosState"),
        ])

FAULT_MAP = {
    1 << 0:  "OverVoltage",
    1 << 1:  "UnderVoltage",
    1 << 2:  "OverPackVolt",
    1 << 3:  "UnderPackVolt",
    1 << 4:  "chargeOverTemp",
    1 << 5:  "chargeUnderTemp",
    1 << 6:  "DischargeOverTemp",
    1 << 7:  "DischargeUnderTemp",
    1 << 8:  "ChargeOverCurrent",
    1 << 9:  "DischargeOverCurrent",
    1 << 10: "ShortCircuit",
    1 << 11: "HardwareFault",
    1 << 12: "ThermalRunaway",
    1 << 13: "CellDifference",
    1 << 14: "OnboardTemp",
    1 << 15: ""
}
def decode_faults(fault_word):
    lst = [name for bit, name in FAULT_MAP.items() if fault_word & bit]
    return lst if lst else ["No Fault"]

def decode_mos_state(code: int) -> str:
    return {
        3: "ALL ON",
        2: "CHG ON",
        1: "DSG ON",
        0: "ALL OFF"
    }.get(code, "UNKNOWN")

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
        "day": day, "month": month, "year": year,
        "hour": hour, "minute": minute, "week": week
    }

def decode_rs485_frame(frame: bytes):
    """Decode the fixed 0x12 0x34 0x01 log-style frame into a dict (like your original)."""
    if len(frame) < 4 + 4 + 4 + 2 + 2 + 2 + 1:  # rough lower bound
        return None
    idx = 4  # Skip 0x12, 0x34, 0x01, ??? (original used idx=4)
    def u8():
        nonlocal idx
        v = frame[idx]; idx += 1; return v
    def u16():
        nonlocal idx
        v = int.from_bytes(frame[idx:idx+2], "little"); idx += 2; return v
    def s16():
        nonlocal idx
        v = int.from_bytes(frame[idx:idx+2], "little", signed=True); idx += 2; return v
    def u32():
        nonlocal idx
        v = int.from_bytes(frame[idx:idx+4], "little"); idx += 4; return v
    try:
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
    except Exception:
        return None

# ----------------------------
# Frame parsing for read-parameter responses (0xDD..  end 0x77 0x00)
# ----------------------------
def parse_rs485_frame(frame):
    if len(frame) < 8:
        return None, "Frame too short"
    if frame[0] != 0xDD:
        return None, "Invalid start byte"
    cmd = frame[1]; status = frame[2]; length = frame[3]
    expected_len = 4 + length + 2 + 2
    if len(frame) < expected_len:
        return None, "Incomplete frame"
    if not (frame[-2] == 0x77 and frame[-1] == 0x00):
        return None, "Invalid end byte (expect 0x77 0x00)"
    data_start = 4; data_end = 4 + length
    data = frame[data_start:data_end]
    checksum = frame[data_end:data_end + 2]
    return {
        "cmd": cmd,
        "status": status,
        "length": length,
        "data_raw": data,
        "checksum": checksum
    }, None

# decode_bms_config based on original (attempt to unpack)
def decode_bms_config(data):
    fmt = "<" + "H"*30 + "H"*12 + "B"*7 + "H"*3 + "I"*2 + "H"*2 + "B"
    try:
        values = struct.unpack(fmt, data)
    except struct.error:
        return {"ERROR": "Data size mismatch for BMS config struct", "received_len": len(data)}
    names = [
        "Cell_OV_set","Cell_OV_rel","Cell_OV_delay",
        "Cell_UV_set","Cell_UV_rel","Cell_UV_delay",
        "Pack_OV_set","Pack_OV_rel","Pack_OV_delay",
        "Pack_UV_set","Pack_UV_rel","Pack_UV_delay",
        "Chg_UT_set","Chg_UT_rel","Chg_UT_delay",
        "Dsg_UT_set","Dsg_UT_rel","Dsg_UT_delay",


        "Dsg_OT_set","Dsg_OT_rel","Dsg_OT_delay",
        "Chg_OT_set","Chg_OT_rel","Chg_OT_delay",
        "Chg_OC_set","Chg_OC_rel","Chg_OC_delay",
        "Dsg_OC_set","Dsg_OC_rel","Dsg_OC_delay",
        "Dsg_OC2_set","Dsg_OC2_rel","Dsg_OC2_delay",
        "SC_set","SC_rel","SC_delay",
        "HCell_OV_set","HCell_OV_rel","HCell_OV_delay",
        "LCell_UV_set","LCell_UV_rel","LCell_UV_delay",
        "load_en","balance_en","chg_balance",
        "ntc1","ntc2","ntc3","ntc4",
        "cellOpenVoltage","balanceDriftVoltage","ipsOffDelay",
        "nominalCapacity","cycleCapacity","fullSetVoltage","endOfVoltage","selfDischargeRate"
    ]
    return dict(zip(names, values))

# ----------------------------
# Serial helper class (lightweight)
# ----------------------------
class RS485Interface:
    def __init__(self, port=PORT, baud=BAUD, timeout=1.0):
        # If user (or global) asked for AUTO, detect a real port first
        if port in (None, "", "AUTO"):
            port = auto_detect_port()

        # Now open the resolved port
        self.ser = serial.Serial(port, baud, timeout=timeout)
        self.buffer = bytearray()

    def send(self, packet: bytes):
        self.ser.write(packet)
        self.ser.flush()

    def read_frames(self):
        """Mimic the working CLI read_frames() logic."""
        chunk = self.ser.read(64)
        if chunk:
            self.buffer.extend(chunk)

        frames = []

        while len(self.buffer) >= 6:
            if self.buffer[0] != 0xDD:
                # discard until start byte
                self.buffer.pop(0)
                continue

            cmd = self.buffer[1]
            length = self.buffer[3]

            # Original CLI assumed: start + cmd + status + len + data + checksum(2) + end(1)
            frame_len = 1 + 1 + 1 + 1 + length + 2 + 1

            if len(self.buffer) < frame_len:
                break

            frame = self.buffer[:frame_len]
            del self.buffer[:frame_len]
            frames.append(frame)

        return frames


    def read(self, size=256):
        return self.ser.read(size)

    def close(self):
        try:
            self.ser.close()
        except Exception:
            pass

# ----------------------------
# Flask app
# ----------------------------
app = Flask(__name__)
CORS(app, 
     origins="http://localhost:3000", 
     allow_methods=["GET", "POST", "OPTIONS"],
     allow_headers=["Content-Type", "Authorization"]
)
def make_json_response(payload, status_code=200):
    resp = make_response(jsonify(payload), status_code)
    # Allow CORS for local UI at localhost:9393
    resp.headers["Access-Control-Allow-Origin"] = "http://localhost:3000"
    resp.headers["Access-Control-Allow-Methods"] = "GET,POST,OPTIONS"
    resp.headers["Access-Control-Allow-Headers"] = "Content-Type,Authorization"
    return resp

# Test data used earlier
cell_voltages_info = [0x0001]
basic_info         = [0x0002]
calibrate_info = [0x10, 0x20]
@app.route("/5", methods=["GET", "POST"])
def api_write_full_config():
    timeout = float(request.args.get("timeout", DEFAULT_TIMEOUT))
    try:
        rs = RS485Interface(timeout=timeout)
    except Exception as e:
        return make_json_response({"status": "error", "message": f"Serial open failed: {e}"}, 500)

    try:
        # ---------- Robust helpers ----------
        def as_float(v):
            try:
                if v is None or v == "":
                    return None
                return float(v)
            except Exception:
                return None

        def volts_to_mV(v):
            """Accept volts (e.g. 3.657) or mV (>=1000). Return 0..65535 int (mV)."""
            f = as_float(v)
            if f is None:
                return 0
            # If value looks like mV already, accept it
            if abs(f) >= 1000:
                val = int(round(f))
            else:
                val = int(round(f * 1000.0))
            return max(0, min(val, 65535))

        def amps_to_mA(a):
            """Accept A or mA. Heuristic: if abs(val) < 100 treat as A, else treat as mA."""
            f = as_float(a)
            if f is None:
                return 0
            if abs(f) < 100:   # likely given in amps (e.g. 1.28, 20)
                val = int(round(f * 1000.0))
            else:              # likely already in mA
                val = int(round(f))
            return max(0, min(val, 65535))
        def amps_to_mA_d(a):
            """Accept A or mA. Heuristic: if abs(val) < 100 treat as A, else treat as mA."""
            f = as_float(a)
            if f is None:
                return 0
            val = int(round(f))
            return max(0, min(val, 65535))

        def int_or_default(v, default=0):
            try:
                if v is None or v == "":
                    return default
                val = int(round(float(v)))
                return max(0, min(val, 65535))
            except Exception:
                return default

        def temp_to_int_signed(v, default=0):
            """Return signed integer (°C) clamped to reasonable range."""
            f = as_float(v)
            if f is None:
                val = int(default)
            else:
                val = int(round(f))
            # clamp to device-sane range
           
            return val

        def signed_to_uint16(v):
            """Convert signed int to two's complement uint16 integer for packing."""
            return v & 0xFFFF

        # ---------- Build payload depending on method ----------
        p = b""
        debug_vals = {}

        if request.method == "POST":
            data = request.get_json(silent=True) or {}
            print('POST /5 data:', data)

            # --- 1: cell OV (charge_ov_c) - volt fields accept V or mV
            cell_OV_P_mV = volts_to_mV(data.get("cell_OV_P", 3.657))
            cell_OV_R_mV = volts_to_mV(data.get("cell_OV_R", 3.500))
            cell_OV_Delay = int_or_default(data.get("cell_OV_Delay", 5))
            p += pack_threshold(cell_OV_P_mV, cell_OV_R_mV, cell_OV_Delay)

            # --- 2: cell UV (charge_uv_c)
            cell_UV_P_mV = volts_to_mV(data.get("cell_UV_P", 1.000))
            cell_UV_R_mV = volts_to_mV(data.get("cell_UV_R", 2.500))
            cell_UV_Delay = int_or_default(data.get("cell_UV_Delay", 5))
            p += pack_threshold(cell_UV_P_mV, cell_UV_R_mV, cell_UV_Delay)

            # --- 3: pack OV (pack_ov_c)
            pack_OV_P_mV = volts_to_mV(data.get("pack_OV_P", 60.0))
            pack_OV_R_mV = volts_to_mV(data.get("pack_OV_R", 58.0))
            pack_OV_Delay = int_or_default(data.get("pack_OV_Delay", 5))
            p += pack_threshold(pack_OV_P_mV, pack_OV_R_mV, pack_OV_Delay)

            # --- 4: pack UV (pack_uv_c)
            pack_UV_P_mV = volts_to_mV(data.get("pack_UV_P", 22.0))
            pack_UV_R_mV = volts_to_mV(data.get("pack_UV_R", 25.0))
            pack_UV_Delay = int_or_default(data.get("pack_UV_Delay", 5))
            p += pack_threshold(pack_UV_P_mV, pack_UV_R_mV, pack_UV_Delay)

            # --- 5: charge UNDER-temp (charge_ut)  <-- restored to original place
            chg_UT_P_s = temp_to_int_signed(data.get("chg_UT_P", 1))
            chg_UT_R_s = temp_to_int_signed(data.get("chg_UT_R", 4))
            chg_UT_Delay = int_or_default(data.get("chg_UT_Delay", 4))
            p += pack_threshold(signed_to_uint16(chg_UT_P_s), signed_to_uint16(chg_UT_R_s), chg_UT_Delay)

            # --- 6: discharge OVER-temp (discharge_ot)
            dsg_OT_P_s = temp_to_int_signed(data.get("dsg_OT_P", 71))
            dsg_OT_R_s = temp_to_int_signed(data.get("dsg_OT_R", 45))
            dsg_OT_Delay = int_or_default(data.get("dsg_OT_Delay", 5))
            p += pack_threshold(signed_to_uint16(dsg_OT_P_s), signed_to_uint16(dsg_OT_R_s), dsg_OT_Delay)

            # --- 7: discharge UNDER-temp (discharge_ut)
            dsg_UT_P_s = temp_to_int_signed(data.get("dsg_UT_P", 10))
            dsg_UT_R_s = temp_to_int_signed(data.get("dsg_UT_R", 10))
            dsg_UT_Delay = int_or_default(data.get("dsg_UT_Delay", 10))
            p += pack_threshold(signed_to_uint16(dsg_UT_P_s), signed_to_uint16(dsg_UT_R_s), dsg_UT_Delay)

            # --- 8: charge OVER-temp (charge_ot)
            chg_OT_P_s = temp_to_int_signed(data.get("chg_OT_P", 66))
            chg_OT_R_s = temp_to_int_signed(data.get("chg_OT_R", 56))
            chg_OT_Delay = int_or_default(data.get("chg_OT_Delay", 6))
            p += pack_threshold(signed_to_uint16(chg_OT_P_s), signed_to_uint16(chg_OT_R_s), chg_OT_Delay)

            # --- 9: charge OC (charge_oc) - currents in mA
            chg_OC_P_mA = amps_to_mA_d(data.get("chg_OC_P", 5.001))   # protect
            chg_OC_R_mA = amps_to_mA(data.get("chg_OC_R", 11.0))    # release
            chg_OC_Delay = int_or_default(data.get("chg_OC_Delay", 8))
            p += pack_threshold(chg_OC_P_mA, chg_OC_R_mA, chg_OC_Delay)

            # --- 10: discharge OC (dis_oc)
            dsg_OC_P_mA = amps_to_mA_d(data.get("dsg_OC_P", 10.001))
            dsg_OC_R_mA = amps_to_mA(data.get("dsg_OC_R", 31.0))
            dsg_OC_Delay = int_or_default(data.get("dsg_OC_Delay", 7))
            p += pack_threshold(dsg_OC_P_mA, dsg_OC_R_mA, dsg_OC_Delay)

            # --- 11: Dsg_OC2 block (new explicit handling; fallback to adv1)
            # treat as integer threshold (legacy adv-style) unless devices use another unit
            dsg_oc2_P = int_or_default(data.get("Dsg_OC2_set",
                                        data.get("Dsg_OC2Set",
                                                 data.get("Dsg_OC2_set",
                                                          data.get("adv1_P", 21)))))
            dsg_oc2_R = int_or_default(data.get("Dsg_OC2_rel",
                                        data.get("Dsg_OC2Rel",
                                                 data.get("Dsg_OC2_rel",
                                                          data.get("adv1_R", 1281)))))
            dsg_oc2_Delay = int_or_default(data.get("Dsg_OC2_delay",
                                        data.get("Dsg_OC2Delay",
                                                 data.get("Dsg_OC2_delay",
                                                          data.get("adv1_Delay", 1)))))
            p += pack_threshold(dsg_oc2_P, dsg_oc2_R, dsg_oc2_Delay)

            # --- 12: SC block (short-circuit or system check-like) - fallback to adv2
            sc_P = int_or_default(data.get("SC_set", data.get("SCSet", data.get("SC_set", data.get("adv2_P", 41)))))
            sc_R = int_or_default(data.get("SC_rel", data.get("SCRel", data.get("SC_rel", data.get("adv2_R", 401)))))
            sc_Delay = int_or_default(data.get("SC_delay", data.get("SCDelay", data.get("SC_delay", data.get("adv2_Delay", 1)))))
            p += pack_threshold(sc_P, sc_R, sc_Delay)

            # --- 13: HCell_OV (High cell over-voltage) - volt field
            hcell_OV_set_mV = volts_to_mV(data.get("HCell_OV_set", data.get("HCell_OVSet", data.get("HCell_OV_set", 3901))))
            hcell_OV_rel_mV = volts_to_mV(data.get("HCell_OV_rel", data.get("HCell_OVRel", data.get("HCell_OV_rel", 1))))
            hcell_OV_delay = int_or_default(data.get("HCell_OV_delay", data.get("HCell_OVDelay", data.get("HCell_OV_delay", 1))))
            p += pack_threshold(hcell_OV_set_mV, hcell_OV_rel_mV, hcell_OV_delay)

            # --- 14: LCell_UV (Low cell under-voltage) - volt field
            lcell_UV_set_mV = volts_to_mV(data.get("LCell_UV_set", data.get("LCell_UVSet", data.get("LCell_UV_set", 3000))))
            lcell_UV_rel_mV = volts_to_mV(data.get("LCell_UV_rel", data.get("LCell_UVRel", data.get("LCell_UV_rel", 15))))
            lcell_UV_delay = int_or_default(data.get("LCell_UV_delay", data.get("LCell_UVDelay", data.get("LCell_UV_delay", 1))))
            p += pack_threshold(lcell_UV_set_mV, lcell_UV_rel_mV, lcell_UV_delay)

            # --- Function config (7 bytes)
            func_flags = []
            for i in range(7):
                val = bool(data.get(f"func_{i}", False))
                func_flags.append(1 if val else 0)
            p += struct.pack("<7B", *func_flags)

            # --- Balancing: voltage (mV), current (mA), delay
            # Accept either "cellOpenVoltage" or legacy keys; volts_to_mV will handle V->mV if needed.
            bal_v = volts_to_mV(
                data.get("cellOpenVoltage",
                         data.get("cell_open_voltage",
                                  data.get("balance_voltage_V", 3.601)))
            )

            # balanceDriftVoltage is used as the balancing current (mA). Prefer explicit key,
            # otherwise fallback to legacy "balance_current".
            bal_current = int_or_default(
                data.get("balanceDriftVoltage",
                         data.get("balance_drift_voltage",
                                  data.get("balance_current", 51)))
            )

            bal_delay = int_or_default(data.get("balance_delay", 11))
            p += struct.pack("<HHH", bal_v, bal_current, bal_delay)

            # --- Capacity config (keep exact layout)
            # design_Ah = float(data.get("design_capacity_Ah", 20.000))
            # cycle_mAh = int(max(0, min(int(design_Ah * 1000), 0xFFFFFFFF)))
            # fcc_mAh = int_or_default(data.get("fcc_mAh", 2000))
            # fullSet = int_or_default(data.get("fullSetVoltage_mV", 2150))
            # endOfV = int_or_default(data.get("endOfVoltage_mV", 2000))
            # p += struct.pack("<IIHHB", cycle_mAh, fcc_mAh, fullSet, endOfV, int_or_default(data.get("last_byte", 2)))
            def cap_to_mAh(v, default_mAh):
                """
                Accept Ah or mAh.
                Heuristic: if abs(value) < 1000 -> treat as Ah, else as mAh.
                """
                f = as_float(v)
                if f is None:
                    return default_mAh
                if abs(f) < 1000:      # likely given in Ah
                    val = int(round(f * 1000.0))
                else:                  # likely already mAh
                    val = int(round(f))
                return max(0, min(val, 0xFFFFFFFF))

            # Nominal (design) capacity
            nominal_mAh = cap_to_mAh(
                data.get("nominal_capacity",
                         data.get("nominal_capacity_mAh",
                                  data.get("design_capacity_Ah",
                                           data.get("design_capacity_mAh", 10.0)))),
                10000  # default 10Ah, same as GET path
            )

            # Cycle capacity
            cycle_mAh = cap_to_mAh(
                data.get("cycle_capacity",
                         data.get("cycle_capacity_mAh",
                                  data.get("cycle_capacity_Ah", 8.0))),
                8000   # default 8Ah, same as GET path
            )

            fullSet = int_or_default(data.get("fullSetVoltage_mV", 4150))
            endOfV  = int_or_default(data.get("endOfVoltage_mV", 3000))
            last_b  = int_or_default(data.get("last_byte", 1))

            p += struct.pack("<IIHHB", nominal_mAh, cycle_mAh, fullSet, endOfV, last_b)

            # Add to debug output
            debug_vals.update({
                "nominal_mAh": nominal_mAh,
                "cycle_mAh": cycle_mAh,
                "fullSet_mV": fullSet,
                "endOfV_mV": endOfV,
                "last_byte": last_b,
            })
            # ---------- Debug dump ----------
            debug_vals.update({
                "cell_OV_P_mV": cell_OV_P_mV, "cell_OV_R_mV": cell_OV_R_mV, "cell_OV_Delay": cell_OV_Delay,
                "cell_UV_P_mV": cell_UV_P_mV, "cell_UV_R_mV": cell_UV_R_mV, "cell_UV_Delay": cell_UV_Delay,
                "pack_OV_P_mV": pack_OV_P_mV, "pack_OV_R_mV": pack_OV_R_mV, "pack_OV_Delay": pack_OV_Delay,
                "pack_UV_P_mV": pack_UV_P_mV, "pack_UV_R_mV": pack_UV_R_mV, "pack_UV_Delay": pack_UV_Delay,
                "chg_UT_P_s": chg_UT_P_s, "chg_UT_R_s": chg_UT_R_s, "chg_UT_Delay": chg_UT_Delay,
                "dsg_OT_P_s": dsg_OT_P_s, "dsg_OT_R_s": dsg_OT_R_s, "dsg_OT_Delay": dsg_OT_Delay,
                "dsg_UT_P_s": dsg_UT_P_s, "dsg_UT_R_s": dsg_UT_R_s, "dsg_UT_Delay": dsg_UT_Delay,
                "chg_OT_P_s": chg_OT_P_s, "chg_OT_R_s": chg_OT_R_s, "chg_OT_Delay": chg_OT_Delay,
                "chg_OC_P_mA": chg_OC_P_mA, "chg_OC_R_mA": chg_OC_R_mA, "chg_OC_Delay": chg_OC_Delay,
                "dsg_OC_P_mA": dsg_OC_P_mA, "dsg_OC_R_mA": dsg_OC_R_mA, "dsg_OC_Delay": dsg_OC_Delay,
                "dsg_oc2_P": dsg_oc2_P, "dsg_oc2_R": dsg_oc2_R, "dsg_oc2_Delay": dsg_oc2_Delay,
                "sc_P": sc_P, "sc_R": sc_R, "sc_Delay": sc_Delay,
                "hcell_OV_set_mV": hcell_OV_set_mV, "hcell_OV_rel_mV": hcell_OV_rel_mV, "hcell_OV_delay": hcell_OV_delay,
                "lcell_UV_set_mV": lcell_UV_set_mV, "lcell_UV_rel_mV": lcell_UV_rel_mV, "lcell_UV_delay": lcell_UV_delay,
                "bal_v_mV": bal_v, "bal_current_mA": bal_current, "bal_delay": bal_delay,
                # "cycle_mAh": cycle_mAh, "fcc_mAh": fcc_mAh,
                "fullSet_mV": fullSet, "endOfV_mV": endOfV,
                "cellOpenVoltage_input": data.get("cellOpenVoltage"),
                "balanceDriftVoltage_input": data.get("balanceDriftVoltage"),
            })

            print("=== DEBUG PACK VALUES ===")
            for k, v in debug_vals.items():
                print(f"{k}: {v!r}")
            print("payload length:", len(p))
            print("payload hex:", p.hex())

        else:
            # GET: preserve backward compatibility and same ordering as POST (so device expects same field order)
            p += pack_threshold(4657, 3500, 5)    # cell OV
            p += pack_threshold(1000, 2500, 5)    # cell UV
            p += pack_threshold(60000, 58000, 5)  # pack OV
            p += pack_threshold(22000, 25000, 5)  # pack UV
            p += pack_threshold(0, 5, 5)          # placeholder

            p += pack_threshold(70, 45, 5)        # chg OT
            p += pack_threshold(75, 50, 3)        # chg UT

            p += pack_threshold(65, 55, 5)        # dsg OT
            p += pack_threshold(0, 0, 0)          # dsg UT (default zeros)

            p += pack_threshold(5000, 15, 5)      # dsg OC (defaults)
            p += pack_threshold(10000, 30, 5)     # chg OC

            # Defaults for slots 11..14 mapped to the new fields:
            # 11: Dsg_OC2 defaults
            p += pack_threshold(21, 1281, 1)
            # 12: SC defaults
            p += pack_threshold(41, 401, 1)
            # 13: HCell_OV defaults (mV)
            p += pack_threshold(3901, 1, 1)
            # 14: LCell_UV defaults (mV)
            p += pack_threshold(3000, 15, 1)

            p += struct.pack("<7B", 0, 0, 0, 0, 0, 0, 0)  # func flags
            # default balancing: cell open voltage 3600 mV, bal current 50 mA, delay 10s
            p += struct.pack("<HHH", 3600, 50, 10)
            p += struct.pack("<IIHHB", 10000, 8000, 4150, 3000, 1)
            print("default payload length:", len(p))

        # Build frame and send
        frame = build_config_frame(p)
        rs.send(frame)

        # Try reading any immediate response
        start = time.time()
        raw = b""
        while time.time() - start < timeout:
            chunk = rs.read(256)
            if chunk:
                raw += chunk
            else:
                time.sleep(0.05)
        try:
            rs.close()
        except Exception:
            pass

        return make_json_response({
            "status": "ok",
            "message": "Full config frame sent",
            "sent_frame_hex": frame.hex().upper(),
            "raw_response_hex": raw.hex().upper()
        })
    except Exception as e:
        try:
            rs.close()
        except Exception:
            pass
        return make_json_response({"status": "error", "message": f"Exception: {e}"}, 500)


@app.route("/3", methods=["POST"])
def api_write_calibration():
    """
    POST /3
    Body JSON: { "value1": 21, "value2": 20 }

    Both values are decimal integers.
    This builds the same calibration frame that your old
    cmdx == 3 branch did, and sends it via RS485.
    """
    # optional timeout query param, same style as others
    timeout = float(request.args.get("timeout", DEFAULT_TIMEOUT))

    # parse JSON body
    data = request.get_json(silent=True) or {}
    v1 = data.get("value1")
    v2 = data.get("value2")

    # basic validation
    try:
        v1 = int(v1)
        v2 = int(v2)
    except (TypeError, ValueError):
        return make_json_response(
            {
                "status": "error",
                "message": "value1 and value2 must be integers in JSON body, e.g. {\"value1\": 21, \"value2\": 20}"
            },
            400
        )

    # clamp to uint16 range (same as struct.pack('<H') expectation)
    if not (0 <= v1 <= 0xFFFF and 0 <= v2 <= 0xFFFF):
        return make_json_response(
            {
                "status": "error",
                "message": "value1 and value2 must be between 0 and 65535"
            },
            400
        )

    # open serial
    try:
        rs = RS485Interface(timeout=timeout)
    except Exception as e:
        return make_json_response(
            {"status": "error", "message": f"Serial open failed: {e}"},
            500
        )

    try:
        # this mirrors: build_packet(RS485_START_WRITE_CAL, CMD_BASIC_INFO,
        #                            STATUS_OK, calibrate_info_1, RS485_END_WRITE_CAL)
        data_list = [v1, v2]
        pkt = build_packet(
            RS485_START_WRITE_CAL,
            CMD_BASIC_INFO,
            STATUS_OK,
            data_list,
            RS485_END_WRITE_CAL
        )

        rs.send(pkt)

        # optional: read back any immediate response
        start = time.time()
        raw = b""
        while time.time() - start < timeout:
            chunk = rs.read(256)
            if chunk:
                raw += chunk
            else:
                time.sleep(0.05)

        rs.close()

        return make_json_response(
            {
                "status": "ok",
                "message": "Calibration frame sent",
                "sent_values": {"value1": v1, "value2": v2},
                "packet_hex": pkt.hex().upper(),
                "raw_response_hex": raw.hex().upper()
            }
        )
    except Exception as e:
        try:
            rs.close()
        except Exception:
            pass
        return make_json_response(
            {"status": "error", "message": f"Exception while sending: {e}"},
            500
        )

# ----------------------------
# Endpoint: /6  (read bms config / basic info)
# ----------------------------
@app.route("/6", methods=["GET"])
def api_read_bms_config():
    timeout = float(request.args.get("timeout", DEFAULT_TIMEOUT))
    try:
        rs = RS485Interface(timeout=0.5)
    except Exception as e:
        return make_json_response({"status": "error", "message": f"Serial open failed: {e}"}, 500)

    try:
        # Use RS485_START_READ_PARAMETER and CMD_BASIC_INFO
        pkt = build_packet(RS485_START_READ_PARAMETER, CMD_BASIC_INFO, STATUS_OK, basic_info, RS485_END_READ_PARAMETER)
        rs.send(pkt)

        # read up to timeout seconds and attempt to parse frame(s)
        start = time.time()
        raw = b""
        while time.time() - start < timeout:
            chunk = rs.read(300)
            if chunk:
                raw += chunk
            else:
                time.sleep(0.05)

        if not raw:
            rs.close()
            return make_json_response({"status": "error", "message": "No response received", "raw_response": ""}, 504)

        # Try to find a valid 0xDD ... 0x77 0x00 frame inside raw
        found = None
        for i in range(len(raw)):
            if raw[i] == 0xDD and i + 6 < len(raw):
                # try to parse from i to end
                candidate = raw[i:]
                parsed, err = parse_rs485_frame(candidate)
                if parsed and not err:
                    found = parsed
                    break
                # else continue scanning
        rs.close()

        if not found:
            return make_json_response({"status": "error", "message": "No valid config frame parsed from device", "raw_hex": raw.hex().upper()}, 502)

        cfg = decode_bms_config(found["data_raw"])
        print('cfg',cfg)
        return make_json_response({"status": "ok", "message": "BMS config read", "config": cfg, "raw_hex": raw.hex().upper()})
    except Exception as e:
        try:
            rs.close()
        except Exception:
            pass
        return make_json_response({"status": "error", "message": f"Exception: {e}"}, 500)

# ----------------------------
# Endpoint: /9  (pack info: cell voltages + basic info)
# ----------------------------
@app.route("/9", methods=["GET"])
def api_pack_info():
    timeout = float(request.args.get("timeout", DEFAULT_TIMEOUT))
    try:
        rs = RS485Interface(timeout=0.4)
    except Exception as e:
        return make_json_response({"status": "error", "message": f"Serial open failed: {e}"}, 500)

    result = {"status": "ok", "cells": None, "basic_info": None}

    try:
        # Send cell voltages request
        pkt_cells = build_packet(RS485_START_CELL, CMD_CELL_VOLTAGES,
                                 STATUS_OK, cell_voltages_info, RS485_END_CELL)
        rs.send(pkt_cells)
        time.sleep(0.05)

        # Send basic info request
        pkt_basic = build_packet(RS485_START_BASIC, CMD_BASIC_INFO,
                                 STATUS_OK, basic_info, RS485_END_BASIC)
        rs.send(pkt_basic)

        start = time.time()
        while time.time() - start < timeout:
            frames = rs.read_frames()
            if not frames:
                time.sleep(0.02)
                continue

            for frame in frames:
                cmd = frame[1]
                length = frame[3]
                payload = frame[4:4 + length]

                if cmd == 0x04 and result["cells"] is None:
                    # cell voltages
                    result["cells"] = decode_cell_voltages(payload)

                elif cmd == 0x05 and result["basic_info"] is None:
                    # BASIC INFO (this is where NTC comes from)
                    print("BASIC PAYLOAD:", payload.hex().upper())
                    result["basic_info"] = decode_basic_info(payload)

            if result["cells"] is not None and result["basic_info"] is not None:
                break

        rs.close()
        return make_json_response(result)

    except Exception as e:
        try:
            rs.close()
        except Exception:
            pass
        return make_json_response({"status": "error", "message": f"Exception: {e}"}, 500)

# ----------------------------
# Endpoint: /1  (logs) -- read up to `count` records or until `timeout` seconds
# ----------------------------
@app.route("/1", methods=["GET"])
def api_logs():
    # query params: count, timeout
    try:
        count = int(request.args.get("count", "5000"))
    except ValueError:
        count = 500
    timeout = float(request.args.get("timeout", "180.0"))

    try:
        rs = RS485Interface(timeout=0.5)
    except Exception as e:
        return make_json_response({"status": "error", "message": f"Serial open failed: {e}"}, 500)

    try:
        # send write-log command (as in original)
        pkt = build_packet(RS485_START_WRITE_LOG, CMD_BASIC_INFO, STATUS_OK, calibrate_info, RS485_END_WRITE_LOG)
        rs.send(pkt)
        time.sleep(0.05)

        results = []
        start_time = time.time()
        # The original code read byte-by-byte and expected START_BYTE then header
        while len(results) < count and (time.time() - start_time) < timeout:
            b = rs.read(1)
            if not b:
                # no byte available right now, continue until timeout
                time.sleep(0.01)
                continue
            if b[0] != START_BYTE:
                # keep scanning
                continue
            header = rs.read(2)
            if len(header) != 2 or header[0] != CMD_BYTE or header[1] != STATUS_BYTE:
                # malformed header; continue
                continue
            rest = rs.read(PACKET_SIZE - 3)
            if len(rest) != (PACKET_SIZE - 3):
                # incomplete; break or continue; we'll break to avoid infinite waits
                break
            frame = b + header + rest
            parsed = decode_rs485_frame(frame)
            if not parsed:
                continue
            # Decode datetime
            parsed["datetime"], _ = decode_time(parsed["time"], parsed["time2"])
            parsed["mosState"] = decode_mos_state(parsed["mosFet"])
            parsed["faultNames"]   = decode_faults(parsed["faults"])
            # Write to CSV
            try:
                log_to_csv(parsed)
            except Exception:
                pass
            results.append(parsed)

        rs.close()
        return make_json_response({"status": "ok", "records_returned": len(results), "records": results})
    except Exception as e:
        try:
            rs.close()
        except Exception:
            pass
        return make_json_response({"status": "error", "message": f"Exception: {e}"}, 500)

# ----------------------------
# OPTIONS handler for CORS preflight
# ----------------------------
@app.route("/", methods=["OPTIONS"])
@app.route("/<path:anypath>", methods=["OPTIONS"])
def options(anypath=None):
    resp = make_response("", 204)
    resp.headers["Access-Control-Allow-Origin"] = "http://localhost:3000"
    resp.headers["Access-Control-Allow-Methods"] = "GET,POST,OPTIONS"
    resp.headers["Access-Control-Allow-Headers"] = "Content-Type,Authorization"
    
    # ADDED: Cache preflight response for 3600 seconds (1 hour)
    resp.headers["Access-Control-Max-Age"] = "3600" 
    
    return resp

# ----------------------------
# Run
# ----------------------------
if __name__ == "__main__":
    print("Starting BMS Flask API (single-file).")
    print("Endpoints: /5, /6, /9, /1")
    print("CORS allowed for http://localhost:3000")
    app.run(host="0.0.0.0", port=9393, debug=True)



# @app.route("/5", methods=["GET", "POST"])
# def api_write_full_config():
#     print('called')
#     timeout = float(request.args.get("timeout", DEFAULT_TIMEOUT))
#     try:
#         rs = RS485Interface(timeout=timeout)
#     except Exception as e:
#         return make_json_response({"status": "error", "message": f"Serial open failed: {e}"}, 500)

#     try:
#         # ---------- Robust helpers ----------
#         def as_float(v):
#             try:
#                 if v is None or v == "":
#                     return None
#                 return float(v)
#             except Exception:
#                 return None

#         def volts_to_mV(v):
#             """Accept volts (e.g. 3.657) or mV (>=1000). Return 0..65535 int."""
#             f = as_float(v)
#             if f is None:
#                 return 0
#             # If value looks like mV already, accept it
#             if abs(f) >= 1000:
#                 val = int(round(f))
#             else:
#                 val = int(round(f * 1000.0))
#             return max(0, min(val, 65535))

#         def amps_to_mA(a):
#             """Accept A or mA. Heuristic: if abs(val) < 100 treat as A, else treat as mA."""
#             f = as_float(a)
#             if f is None:
#                 return 0
#             if abs(f) < 100:   # likely given in amps (e.g. 1.28, 20)
#                 val = int(round(f * 1000.0))
#             else:              # likely already in mA
#                 val = int(round(f))
#             return max(0, min(val, 65535))

#         def int_or_default(v, default=0):
#             try:
#                 if v is None or v == "":
#                     return default
#                 val = int(round(float(v)))
#                 return max(0, min(val, 65535))
#             except Exception:
#                 return default

#         def temp_to_int_signed(v, default=0):
#             """Return signed integer (°C) clamped to reasonable range."""
#             f = as_float(v)
#             if f is None:
#                 val = int(default)
#             else:
#                 val = int(round(f))
#             # clamp to device-sane range
#             if val < -55:
#                 val = -55
#             if val > 150:
#                 val = 150
#             return val

#         def signed_to_uint16(v):
#             """Convert signed int to two's complement uint16 integer for packing."""
#             return v & 0xFFFF

#         # ---------- Build payload depending on method ----------
#         p = b""
#         debug_vals = {}

#         if request.method == "POST":
#             data = request.get_json(silent=True) or {}
#             print('data', data)

#             # --- 1: cell OV (charge_ov_c)
#             cell_OV_P_mV = volts_to_mV(data.get("cell_OV_P", 3.657))
#             cell_OV_R_mV = volts_to_mV(data.get("cell_OV_R", 3.500))
#             cell_OV_Delay = int_or_default(data.get("cell_OV_Delay", 5))
#             p += pack_threshold(cell_OV_P_mV, cell_OV_R_mV, cell_OV_Delay)

#             # --- 2: cell UV (charge_uv_c)
#             cell_UV_P_mV = volts_to_mV(data.get("cell_UV_P", 1.000))
#             cell_UV_R_mV = volts_to_mV(data.get("cell_UV_R", 2.500))
#             cell_UV_Delay = int_or_default(data.get("cell_UV_Delay", 5))
#             p += pack_threshold(cell_UV_P_mV, cell_UV_R_mV, cell_UV_Delay)

#             # --- 3: pack OV (pack_ov_c)
#             pack_OV_P_mV = volts_to_mV(data.get("pack_OV_P", 60.0))
#             pack_OV_R_mV = volts_to_mV(data.get("pack_OV_R", 58.0))
#             pack_OV_Delay = int_or_default(data.get("pack_OV_Delay", 5))
#             p += pack_threshold(pack_OV_P_mV, pack_OV_R_mV, pack_OV_Delay)

#             # --- 4: pack UV (pack_uv_c)
#             pack_UV_P_mV = volts_to_mV(data.get("pack_UV_P", 22.0))
#             pack_UV_R_mV = volts_to_mV(data.get("pack_UV_R", 25.0))
#             pack_UV_Delay = int_or_default(data.get("pack_UV_Delay", 5))
#             p += pack_threshold(pack_UV_P_mV, pack_UV_R_mV, pack_UV_Delay)

#             # --- 5: charge UNDER-temp (charge_ut)  <-- restored to original place
#             chg_UT_P_s = temp_to_int_signed(data.get("chg_UT_P", 1))
#             chg_UT_R_s = temp_to_int_signed(data.get("chg_UT_R", 4))
#             chg_UT_Delay = int_or_default(data.get("chg_UT_Delay", 4))
#             p += pack_threshold(signed_to_uint16(chg_UT_P_s), signed_to_uint16(chg_UT_R_s), chg_UT_Delay)

#             # --- 6: discharge OVER-temp (discharge_ot)
#             dsg_OT_P_s = temp_to_int_signed(data.get("dsg_OT_P", 71))
#             dsg_OT_R_s = temp_to_int_signed(data.get("dsg_OT_R", 45))
#             dsg_OT_Delay = int_or_default(data.get("dsg_OT_Delay", 5))
#             p += pack_threshold(signed_to_uint16(dsg_OT_P_s), signed_to_uint16(dsg_OT_R_s), dsg_OT_Delay)

#             # --- 7: discharge UNDER-temp (discharge_ut)
#             dsg_UT_P_s = temp_to_int_signed(data.get("dsg_UT_P", 10))
#             dsg_UT_R_s = temp_to_int_signed(data.get("dsg_UT_R", 10))
#             dsg_UT_Delay = int_or_default(data.get("dsg_UT_Delay", 10))
#             p += pack_threshold(signed_to_uint16(dsg_UT_P_s), signed_to_uint16(dsg_UT_R_s), dsg_UT_Delay)

#             # --- 8: charge OVER-temp (charge_ot)
#             chg_OT_P_s = temp_to_int_signed(data.get("chg_OT_P", 66))
#             chg_OT_R_s = temp_to_int_signed(data.get("chg_OT_R", 56))
#             chg_OT_Delay = int_or_default(data.get("chg_OT_Delay", 6))
#             p += pack_threshold(signed_to_uint16(chg_OT_P_s), signed_to_uint16(chg_OT_R_s), chg_OT_Delay)

#             # --- 9: charge OC (charge_oc) - currents in mA
#             chg_OC_P_mA = amps_to_mA(data.get("chg_OC_P", 5.001))   # protect
#             chg_OC_R_mA = amps_to_mA(data.get("chg_OC_R", 11.0))    # release
#             chg_OC_Delay = int_or_default(data.get("chg_OC_Delay", 8))
#             p += pack_threshold(chg_OC_P_mA, chg_OC_R_mA, chg_OC_Delay)

#             # --- 10: discharge OC (dis_oc)
#             dsg_OC_P_mA = amps_to_mA(data.get("dsg_OC_P", 10.001))
#             dsg_OC_R_mA = amps_to_mA(data.get("dsg_OC_R", 31.0))
#             dsg_OC_Delay = int_or_default(data.get("dsg_OC_Delay", 7))
#             p += pack_threshold(dsg_OC_P_mA, dsg_OC_R_mA, dsg_OC_Delay)

#             # --- 11..14: Advanced protection / placeholders (exact 4 blocks)
#             p += pack_threshold(int_or_default(data.get("adv1_P", 21)), int_or_default(data.get("adv1_R", 1281)), int_or_default(data.get("adv1_Delay", 1)))
#             p += pack_threshold(int_or_default(data.get("adv2_P", 41)), int_or_default(data.get("adv2_R", 401)), int_or_default(data.get("adv2_Delay", 1)))
#             p += pack_threshold(int_or_default(data.get("adv3_P", 3901)), int_or_default(data.get("adv3_R", 1)), int_or_default(data.get("adv3_Delay", 1)))
#             p += pack_threshold(int_or_default(data.get("adv4_P", 3000)), int_or_default(data.get("adv4_R", 15)), int_or_default(data.get("adv4_Delay", 1)))

#             # --- Function config (7 bytes)
#             func_flags = []
#             for i in range(7):
#                 val = bool(data.get(f"func_{i}", False))
#                 func_flags.append(1 if val else 0)
#             p += struct.pack("<7B", *func_flags)

#             # --- Balancing: voltage (mV), current (mA), delay
#             bal_v = volts_to_mV(data.get("balance_voltage_V", 3.601))
#             bal_current = amps_to_mA(data.get("balance_current", 51))  # reuse amps_to_mA logic
#             bal_delay = int_or_default(data.get("balance_delay", 11))
#             p += struct.pack("<HHH", bal_v, bal_current, bal_delay)

#             # --- Capacity config (keep exact layout)
#             design_Ah = float(data.get("design_capacity_Ah", 20.000))
#             cycle_mAh = int(max(0, min(int(design_Ah * 1000), 0xFFFFFFFF)))
#             fcc_mAh = int_or_default(data.get("fcc_mAh", 2000))
#             fullSet = int_or_default(data.get("fullSetVoltage_mV", 2150))
#             endOfV = int_or_default(data.get("endOfVoltage_mV", 2000))
#             p += struct.pack("<IIHHB", cycle_mAh, fcc_mAh, fullSet, endOfV, int_or_default(data.get("last_byte", 2)))

#             # ---------- Debug dump ----------
#             debug_vals.update({
#                 "cell_OV_P_mV": cell_OV_P_mV, "cell_OV_R_mV": cell_OV_R_mV, "cell_OV_Delay": cell_OV_Delay,
#                 "cell_UV_P_mV": cell_UV_P_mV, "cell_UV_R_mV": cell_UV_R_mV, "cell_UV_Delay": cell_UV_Delay,
#                 "pack_OV_P_mV": pack_OV_P_mV, "pack_OV_R_mV": pack_OV_R_mV, "pack_OV_Delay": pack_OV_Delay,
#                 "pack_UV_P_mV": pack_UV_P_mV, "pack_UV_R_mV": pack_UV_R_mV, "pack_UV_Delay": pack_UV_Delay,
#                 "chg_UT_P_s": chg_UT_P_s, "chg_UT_R_s": chg_UT_R_s, "chg_UT_Delay": chg_UT_Delay,
#                 "dsg_OT_P_s": dsg_OT_P_s, "dsg_OT_R_s": dsg_OT_R_s, "dsg_OT_Delay": dsg_OT_Delay,
#                 "dsg_UT_P_s": dsg_UT_P_s, "dsg_UT_R_s": dsg_UT_R_s, "dsg_UT_Delay": dsg_UT_Delay,
#                 "chg_OT_P_s": chg_OT_P_s, "chg_OT_R_s": chg_OT_R_s, "chg_OT_Delay": chg_OT_Delay,
#                 "chg_OC_P_mA": chg_OC_P_mA, "chg_OC_R_mA": chg_OC_R_mA, "chg_OC_Delay": chg_OC_Delay,
#                 "dsg_OC_P_mA": dsg_OC_P_mA, "dsg_OC_R_mA": dsg_OC_R_mA, "dsg_OC_Delay": dsg_OC_Delay,
#                 "bal_v_mV": bal_v, "bal_current_mA": bal_current, "bal_delay": bal_delay,
#                 "cycle_mAh": cycle_mAh, "fcc_mAh": fcc_mAh,
#                 "fullSet_mV": fullSet, "endOfV_mV": endOfV
#             })
#             print("=== DEBUG PACK VALUES ===")
#             for k, v in debug_vals.items():
#                 print(f"{k}: {v!r}")
#             print("payload length:", len(p))
#             print("payload hex:", p.hex())


#         else:
#             # GET: preserve backward compatibility and same ordering as POST (so device expects same field order)
#             # We must mirror the same field blocks as above — here we use reasonable defaults
#             p += pack_threshold(4657, 3500, 5)    # cell OV
#             p += pack_threshold(1000, 2500, 5)    # cell UV
#             p += pack_threshold(60000, 58000, 5)  # pack OV
#             p += pack_threshold(22000, 25000, 5)  # pack UV
#             p += pack_threshold(0, 5, 5)          # placeholder

#             p += pack_threshold(70, 45, 5)        # chg OT
#             p += pack_threshold(75, 50, 3)        # chg UT

#             p += pack_threshold(65, 55, 5)        # dsg OT
#             p += pack_threshold(0, 0, 0)          # dsg UT (default zeros)

#             p += pack_threshold(5000, 15, 5)      # dsg OC (defaults)
#             p += pack_threshold(10000, 30, 5)     # chg OC

#             p += pack_threshold(20, 1280, 0)
#             p += pack_threshold(40, 400, 0)
#             p += pack_threshold(3900, 8, 0)
#             p += pack_threshold(2000, 16, 0)

#             p += struct.pack("<7B", 0, 0, 0, 0, 0, 0, 0)  # func flags
#             p += struct.pack("<HHH", 3600, 50, 10)        # balance config
#             p += struct.pack("<IIHHB", 10000, 8000, 4150, 3000, 1)
#             print("default payload length:", len(p))

#         frame = build_config_frame(p)
#         rs.send(frame)

#         # Try reading any immediate response
#         start = time.time()
#         raw = b""
#         while time.time() - start < timeout:
#             chunk = rs.read(256)
#             if chunk:
#                 raw += chunk
#             else:
#                 time.sleep(0.05)
#         rs.close()

#         return make_json_response({
#             "status": "ok",
#             "message": "Full config frame sent",
#             "sent_frame_hex": frame.hex().upper(),
#             "raw_response_hex": raw.hex().upper()
#         })
#     except Exception as e:
#         try:
#             rs.close()
#         except Exception:
#             pass
#         return make_json_response({"status": "error", "message": f"Exception: {e}"}, 500)
