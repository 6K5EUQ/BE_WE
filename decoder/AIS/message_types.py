import math
from ais_payload import get_uint, get_int, get_string, get_bool


NAV_STATUS = {
    0:  "Under way using engine",
    1:  "At anchor",
    2:  "Not under command",
    3:  "Restricted manoeuvrability",
    4:  "Constrained by draught",
    5:  "Moored",
    6:  "Aground",
    7:  "Engaged in fishing",
    8:  "Under way sailing",
    9:  "Reserved (HSC)",
    10: "Reserved (WIG)",
    11: "Reserved",
    12: "Reserved",
    13: "Reserved",
    14: "AIS-SART active",
    15: "Not defined",
}

MANEUVER_INDICATOR = {
    0: "N/A",
    1: "No special maneuver",
    2: "Special maneuver",
}

EPFD_TYPE = {
    0:  "Undefined",
    1:  "GPS",
    2:  "GLONASS",
    3:  "Combined GPS/GLONASS",
    4:  "Loran-C",
    5:  "Chayka",
    6:  "Integrated navigation system",
    7:  "Surveyed",
    8:  "Galileo",
    15: "Internal GNSS",
}

# Full list in ITU-R M.1371 Annex
SHIP_TYPE = {
    0:  "Not available",
    20: "WIG",                  21: "WIG - Hazardous A",    22: "WIG - Hazardous B",
    23: "WIG - Hazardous C",    24: "WIG - Hazardous D",
    30: "Fishing",              31: "Towing",               32: "Towing (large)",
    33: "Dredging",             34: "Diving",
    35: "Military",             36: "Sailing",              37: "Pleasure craft",
    40: "HSC",                  41: "HSC - Hazardous A",    42: "HSC - Hazardous B",
    43: "HSC - Hazardous C",    44: "HSC - Hazardous D",    49: "HSC - No info",
    50: "Pilot vessel",         51: "SAR",                  52: "Tug",
    53: "Port tender",          54: "Anti-pollution",       55: "Law enforcement",
    56: "Local vessel",         57: "Local vessel",
    58: "Medical transport",    59: "Noncombatant ship",
    60: "Passenger",            61: "Passenger - Haz A",    62: "Passenger - Haz B",
    63: "Passenger - Haz C",    64: "Passenger - Haz D",    69: "Passenger - No info",
    70: "Cargo",                71: "Cargo - Haz A",        72: "Cargo - Haz B",
    73: "Cargo - Haz C",        74: "Cargo - Haz D",        79: "Cargo - No info",
    80: "Tanker",               81: "Tanker - Haz A",       82: "Tanker - Haz B",
    83: "Tanker - Haz C",       84: "Tanker - Haz D",       89: "Tanker - No info",
    90: "Other",                91: "Other - Haz A",        92: "Other - Haz B",
    93: "Other - Haz C",        94: "Other - Haz D",        99: "Other - No info",
}


def _lookup(table: dict, val: int, default: str = "Unknown") -> str:
    return table.get(val, default)


def _format_lon(raw: int) -> str:
    if raw == 0x6791AC0:  # 181 * 600000, unavailable
        return "N/A"
    deg = raw / 600000.0
    direction = "E" if deg >= 0 else "W"
    return f"{abs(deg):.6f} {direction}"


def _format_lat(raw: int) -> str:
    if raw == 0x3412140:  # 91 * 600000, unavailable
        return "N/A"
    deg = raw / 600000.0
    direction = "N" if deg >= 0 else "S"
    return f"{abs(deg):.6f} {direction}"


def _format_rot(raw: int) -> str:
    if raw == -128:
        return "N/A"
    if raw == 0:
        return "0 deg/min"
    if raw == 127:
        return "> 5 deg/min (starboard)"
    if raw == -127:
        return "> 5 deg/min (port)"
    # Back-calculate sensor value: ROT_sensor = 4.733 * sqrt(|ROT_AIS|)
    rot_deg = math.copysign(4.733 * math.sqrt(abs(raw)), raw)
    return f"{rot_deg:.1f} deg/min"


def _format_sog(raw: int) -> str:
    if raw == 1023:
        return "N/A"
    if raw == 1022:
        return ">= 102.2 kn"
    return f"{raw / 10.0:.1f} kn"


def _format_cog(raw: int) -> str:
    if raw == 3600:
        return "N/A"
    return f"{raw / 10.0:.1f} deg"


def _format_heading(raw: int) -> str:
    if raw == 511:
        return "N/A"
    return f"{raw} deg"


def parse_msg_123(bits: str) -> dict:
    # Types 1, 2, 3 share the same structure. 168 bits total.
    msg_type    = get_uint(bits,   0,  6)
    repeat      = get_uint(bits,   6,  2)
    mmsi        = get_uint(bits,   8, 30)
    nav_status  = get_uint(bits,  38,  4)
    rot_raw     = get_int (bits,  42,  8)
    sog_raw     = get_uint(bits,  50, 10)
    pos_acc     = get_uint(bits,  60,  1)
    lon_raw     = get_int (bits,  61, 28)
    lat_raw     = get_int (bits,  89, 27)
    cog_raw     = get_uint(bits, 116, 12)
    heading_raw = get_uint(bits, 128,  9)
    timestamp   = get_uint(bits, 137,  6)
    maneuver    = get_uint(bits, 143,  2)
    raim        = get_bool(bits, 148)
    radio       = get_uint(bits, 149, 19)

    return {
        "Message Type":    f"{msg_type} (Class A Position Report)",
        "Repeat":          repeat,
        "MMSI":            f"{mmsi:09d}",
        "Nav Status":      _lookup(NAV_STATUS, nav_status),
        "ROT":             _format_rot(rot_raw),
        "SOG":             _format_sog(sog_raw),
        "Position Acc.":   "High (DGPS)" if pos_acc else "Low",
        "Longitude":       _format_lon(lon_raw),
        "Latitude":        _format_lat(lat_raw),
        "COG":             _format_cog(cog_raw),
        "True Heading":    _format_heading(heading_raw),
        "Timestamp":       f"{timestamp}s" if timestamp < 60 else "N/A",
        "Maneuver":        _lookup(MANEUVER_INDICATOR, maneuver),
        "RAIM":            "On" if raim else "Off",
        "Radio Status":    f"0x{radio:05X}",
    }


def parse_msg_4(bits: str) -> dict:
    msg_type = get_uint(bits,   0,  6)
    repeat   = get_uint(bits,   6,  2)
    mmsi     = get_uint(bits,   8, 30)
    year     = get_uint(bits,  38, 14)
    month    = get_uint(bits,  52,  4)
    day      = get_uint(bits,  56,  5)
    hour     = get_uint(bits,  61,  5)
    minute   = get_uint(bits,  66,  6)
    second   = get_uint(bits,  72,  6)
    pos_acc  = get_uint(bits,  78,  1)
    lon_raw  = get_int (bits,  79, 28)
    lat_raw  = get_int (bits, 107, 27)
    epfd     = get_uint(bits, 134,  4)
    raim     = get_bool(bits, 148)
    radio    = get_uint(bits, 149, 19)

    utc_str = f"{year:04d}-{month:02d}-{day:02d} {hour:02d}:{minute:02d}:{second:02d} UTC"
    if year == 0:
        utc_str = "N/A"

    return {
        "Message Type": f"{msg_type} (Base Station Report)",
        "Repeat":       repeat,
        "MMSI":         f"{mmsi:09d}",
        "UTC Time":     utc_str,
        "Position Acc.": "High (DGPS)" if pos_acc else "Low",
        "Longitude":    _format_lon(lon_raw),
        "Latitude":     _format_lat(lat_raw),
        "EPFD":         _lookup(EPFD_TYPE, epfd),
        "RAIM":         "On" if raim else "Off",
        "Radio Status": f"0x{radio:05X}",
    }


def parse_msg_5(bits: str) -> dict:
    msg_type      = get_uint(bits,   0,  6)
    repeat        = get_uint(bits,   6,  2)
    mmsi          = get_uint(bits,   8, 30)
    ais_version   = get_uint(bits,  38,  2)
    imo           = get_uint(bits,  40, 30)
    callsign      = get_string(bits,  70,  42)  # 7 chars
    shipname      = get_string(bits, 112, 120)  # 20 chars
    shiptype      = get_uint(bits, 232,  8)
    dim_bow       = get_uint(bits, 240,  9)
    dim_stern     = get_uint(bits, 249,  9)
    dim_port      = get_uint(bits, 258,  6)
    dim_starboard = get_uint(bits, 264,  6)
    epfd          = get_uint(bits, 270,  4)
    eta_month     = get_uint(bits, 274,  4)
    eta_day       = get_uint(bits, 278,  5)
    eta_hour      = get_uint(bits, 283,  5)
    eta_minute    = get_uint(bits, 288,  6)
    draught_raw   = get_uint(bits, 294,  8)
    destination   = get_string(bits, 302, 120)  # 20 chars
    dte           = get_uint(bits, 422,  1)

    overall_length = dim_bow + dim_stern
    overall_width  = dim_port + dim_starboard

    eta_str = "N/A"
    if eta_month > 0 and eta_day > 0:
        eta_str = f"{eta_month:02d}-{eta_day:02d} {eta_hour:02d}:{eta_minute:02d} UTC"

    return {
        "Message Type": f"{msg_type} (Static and Voyage Data)",
        "Repeat":       repeat,
        "MMSI":         f"{mmsi:09d}",
        "AIS Version":  ais_version,
        "IMO Number":   imo if imo > 0 else "N/A",
        "Call Sign":    callsign if callsign else "N/A",
        "Ship Name":    shipname if shipname else "N/A",
        "Ship Type":    _lookup(SHIP_TYPE, shiptype, f"Code {shiptype}"),
        "Dimensions":   f"{overall_length}m x {overall_width}m  (bow {dim_bow}m  stern {dim_stern}m  port {dim_port}m  stbd {dim_starboard}m)",
        "EPFD":         _lookup(EPFD_TYPE, epfd),
        "ETA":          eta_str,
        "Draught":      f"{draught_raw / 10.0:.1f}m" if draught_raw > 0 else "N/A",
        "Destination":  destination if destination else "N/A",
        "DTE":          "Ready" if dte == 0 else "Not ready",
    }


PARSERS = {
    1: parse_msg_123,
    2: parse_msg_123,
    3: parse_msg_123,
    4: parse_msg_4,
    5: parse_msg_5,
}


def parse_message(bits: str) -> dict:
    if len(bits) < 6:
        return {"Error": "Bitstream too short"}

    msg_type = get_uint(bits, 0, 6)
    parser = PARSERS.get(msg_type)

    if parser is None:
        return {
            "Message Type":  msg_type,
            "Status":        f"Type {msg_type} not supported",
            "Bit Length":    len(bits),
        }

    try:
        return parser(bits)
    except (IndexError, ValueError) as e:
        return {
            "Message Type": msg_type,
            "Error":        f"Parse error: {e}",
            "Bit Length":   len(bits),
        }
