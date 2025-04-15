import struct
import tkinter as tk
from io import BufferedReader
from tkinter import filedialog
from typing import Any, Dict, List, Tuple

import pandas as pd
import yaml

MAGIC_EXPECTED = 0x46474F4C  # ASCII: 'LOGF'

def read_log_header(f):
    # read the 6-byte magic + version + endianness header
    header = f.read(6)
    if len(header) < 6:
        raise ValueError("Incomplete log file header")

    # try both endianness options
    magic_le = int.from_bytes(header[0:4], byteorder="little")
    magic_be = int.from_bytes(header[0:4], byteorder="big")

    if magic_le == MAGIC_EXPECTED:
        endian = "little"
    elif magic_be == MAGIC_EXPECTED:
        endian = "big"
    else:
        raise ValueError("Invalid magic value — not a valid log file")

    version = header[4]
    declared_endian = header[5]

    print(f"Detected log version: {version}, magic endianness: {endian}, declared endian: {declared_endian}")

    if (endian == "little" and declared_endian != 1) or (endian == "big" and declared_endian != 0):
        print("[!] Warning: Declared endian and magic endian disagree")

    return endian

def skip_optional_text_headers(f: BufferedReader) -> str:
    log_start_time = None

    while True:
        pos: int = f.tell()
        line: bytes = f.readline()
        if not line:
            break  # EOF
        # there's a line break between the header and datetime string
        if not line.startswith(b"#") and not line.startswith(b'\n'):
            f.seek(pos)
            break

        # Try to match log start time
        line_str: str = line.decode("utf-8").strip()
        if line_str.startswith("# Log Start Time:"):
            log_start_time: str = line_str.replace("# Log Start Time:", "").strip()

    return log_start_time

def crc16_ccitt(data: bytes, crc=0xFFFF) -> int:
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc

def load_schema() -> dict:
    with open("log_schema.yaml", "r") as f:
        return yaml.safe_load(f)

def decode_payload(msg_id: int, 
                   payload: bytes, 
                   schema: dict, 
                   endian_prefix: int, 
                   timestamp: float) -> Dict[str, Any]:
    # locate the schema entry for the message ID
    msg_key = f"0x{msg_id:02x}"
    if msg_key not in schema:
        print(f"[{timestamp}] Unknown message ID: {msg_id:04x}")
        return

    entry = schema[msg_key]
    dtype = entry["type"]
    count = entry["length"]
    fields = entry.get("fields", None)

    # Build the struct format string
    fmt_map = {"float32": "f", "int16": "h", "uint8": "B"}
    if dtype not in fmt_map:
        raise ValueError(f"Unsupported data type: {dtype}")

    struct_fmt = endian_prefix + fmt_map[dtype] * count
    values = struct.unpack(struct_fmt, payload)

    # allocate results dict
    return_dict: Dict[str, Any] = {}
    return_dict["timestamp"] = timestamp

    if fields and len(fields) == count:
        for field, val in zip(fields, values):
            return_dict[field] = val
    else:
        # this is an incomplete signal payload!
        # the CRC should have caught this
        return {}
    
    return return_dict

def parse_log(file_path: str, target_ids: List[int]) -> Tuple[Dict[int, pd.DataFrame], str]:
    # make a dict to hold each set of frames
    df_dict: Dict[int, pd.DataFrame] = {}

    # hold the row sets for each ID prior to constructing the dataframes
    rows_dict: Dict[int, List[Any]] = {}

    with open(file_path, "rb") as f:
        endian = read_log_header(f)
        log_start_time = skip_optional_text_headers(f)

        print(f"[i] Log started at: {log_start_time}")
        
        endian_prefix = "<" if endian == "little" else ">"

        HEADER_STRUCT = struct.Struct(endian_prefix + "I H B B B")
        CRC_STRUCT = struct.Struct(endian_prefix + "H")

        schema: dict = load_schema()

        while True:
            # read the header
            header_data: bytes = f.read(HEADER_STRUCT.size)
            if len(header_data) < HEADER_STRUCT.size:
                break
            
            # use the header struct to unpack all the header fields from the bytes
            timestamp, msg_id, flags, node_id, dlc = HEADER_STRUCT.unpack(header_data)

            if msg_id not in target_ids:
                # skip this message
                f.seek(dlc + CRC_STRUCT.size, 1)
                continue

            # read the payload and crc -- note that the f.read that made header data already moved the file pointer
            payload: bytes = f.read(dlc)
            crc_data: bytes = f.read(CRC_STRUCT.size)

            # concatenate the header and payload for CRC check
            crc_input: bytes = header_data + payload
            expected_crc: int = CRC_STRUCT.unpack(crc_data)[0]
            actual_crc: int = crc16_ccitt(crc_input)

            if expected_crc != actual_crc:
                print(f"[!] CRC Mismatch @ {timestamp} msg 0x{msg_id:02x}")
                continue

            # decode the payload and associate with fields from schema
            log_row: Dict[str, Any] = decode_payload(msg_id, payload, schema, endian_prefix, timestamp)
            
            # if the row is incomplete, skip it
            if not log_row:
                continue
            
            if msg_id not in rows_dict:
                rows_dict[msg_id] = []

            rows_dict[msg_id].append(log_row)

        # convert the list of rows to a pandas dataframe
        for id, rows in rows_dict.items():
            df = pd.DataFrame(rows)
            df.set_index("timestamp", inplace=True)
            df_dict[id] = df

        return df_dict, log_start_time

if __name__ == "__main__":
    def browse_file():
        root = tk.Tk()
        root.withdraw()  # Hide the root window
        file_path = filedialog.askopenfilename(title="Select Log File", filetypes=[("Log Files", "*.bin"), ("All Files", "*.*")])
        return file_path

    file_path = browse_file()
    if file_path:
        print(f"Selected file: {file_path}")
        # TODO - need to pull target IDs first
        parse_log(file_path)
    else:
        print("No file selected.")
