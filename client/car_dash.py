"""
how to launch:
python car_dashboard.py (com port here)
"""

import argparse
import re
import time
import serial
import serial.tools.list_ports

try:
    import curses
    CURSES_AVAILABLE = True
except ImportError:
    CURSES_AVAILABLE = False



class VehicleState:
    def __init__(self):
        # Engine & Powertrain
        self.engine_rpm = 0
        self.throttle_position = 0   # 0-254, 255=kickdown
        self.speed = 0.0             # MPH
        self.engine_temp = 0         # °C
        self.torque = 0.0            # Nm
        self.battery_voltage = 0.0   # V
        self.engine_flag_from_can = False
        self.key_state_raw = 0x00
        self.key_state_available = False
        self.gear_position_raw = 0x00

        # Brakes & dynamics
        self.braking = False
        self.brake_status = 0        # 0-255
        self.parking_brake_on = False
        self.steering_angle = 0.0    # degrees

        # Doors & locks
        self.door_locked = False
        self.door_open_driver_front = False
        self.door_open_passenger_front = False
        self.door_open_driver_rear = False
        self.door_open_passenger_rear = False
        self.mirrors_retracted = False
        self.seat_belt_plugged = False
        # Windows (0-255, scaled)
        self.window_driver_front = 0
        self.window_passenger_front = 0
        self.window_driver_rear = 0
        self.window_passenger_rear = 0

        # Fuel & Range
        self.odometer = 0            # KM
        self.fuel_level = 0          # Litres
        self.range_km = 0.0          # KM

        # Climate
        self.fan_speed_raw = 0       # 0-7 raw
        self.fan_on = False
        self.driver_temp = 0         # °C
        self.passenger_temp = 0      # °C
        self.ac_active = False
        self.blower_state = 0        # 0=AUTO, bitmask otherwise
        self.driver_seat_heater = 0  # 0-3
        self.passenger_seat_heater = 0

        # Raw register matrix: {can_id: [byte, ...]}
        self.registers = {}

    # ── Derived properties ──────────────────────────────────────────────────────

    def get_key_state(self):
        if not self.key_state_available:
            return "Unknown"
        return {
            0x00: "Engine Off",
            0x40: "Inserting",
            0x41: "Position 1",
            0x45: "Position 2",
            0x55: "Cranking",
        }.get(self.key_state_raw, "Unknown")

    def is_engine_running(self):
        if self.key_state_available:
            return self.key_state_raw in (0x45, 0x55) and self.engine_rpm > 400
        return self.engine_flag_from_can and self.engine_rpm > 400

    def is_engine_cranking(self):
        if self.key_state_available:
            return self.key_state_raw == 0x55 and self.engine_rpm < 400
        return self.engine_flag_from_can and 0 < self.engine_rpm < 400

    def get_ignition_status(self):
        if self.engine_rpm > 400:
            return "Running"
        if self.key_state_available:
            if self.key_state_raw in (0x00, 0x40, 0x41):
                return "Off"
            return "On (IGN2)"
        return "On (IGN2)" if self.engine_flag_from_can else "Off"

    def get_gear(self):
        if 0xC5 <= self.gear_position_raw <= 0xCA:
            return "Drive"
        return {
            0xE3: "Park",
            0xC2: "Reverse",
            0xD1: "Neutral",
        }.get(self.gear_position_raw, "---")

    def get_drive_gear(self):
        if 0xC5 <= self.gear_position_raw <= 0xCA:
            return self.gear_position_raw - 0xC4
        return None

    def get_fan_speed(self):
        if not self.fan_on:
            return 0
        return self.fan_speed_raw

    def get_blower_str(self):
        if self.blower_state == 0:
            return "Auto"
        parts = []
        if self.blower_state & 0x01:
            parts.append("Windshield")
        if self.blower_state & 0x02:
            parts.append("Center")
        if self.blower_state & 0x04:
            parts.append("Footwell")
        return ", ".join(parts) if parts else "Auto"

    def get_heater_str(self, level):
        return ("Off", "Low", "Med", "High")[min(level, 3)]

    def get_open_doors(self):
        doors = []
        if self.door_open_driver_front:     doors.append("Dr.F")
        if self.door_open_passenger_front:  doors.append("Ps.F")
        if self.door_open_driver_rear:      doors.append("Dr.R")
        if self.door_open_passenger_rear:   doors.append("Ps.R")
        return ", ".join(doors) if doors else "Closed"

    def get_throttle_str(self):
        if self.throttle_position == 255:
            return "KICKDOWN"
        pct = (self.throttle_position * 100) // 254
        return f"{pct}%"

    def get_power(self):
        if not self.is_engine_running():
            return 0.0
        return (self.engine_rpm * self.torque) / 9549.2965855


#actual can frame parser here:

def parse_line(line: str):
    """Parse a CAN frame line. Returns (can_id_int, data_list) or (None, None)."""
    match = re.match(
        r'(?:RX:\s*)?0x([0-9A-Fa-f]+)\s+(?:\[\d+\]|Data:)\s*((?:[0-9A-Fa-f]{2}\s*)+)',
        line.strip()
    )
    if not match:
        return None, None
    can_id = int(match.group(1), 16)
    data = [int(b, 16) for b in match.group(2).strip().split()]
    return can_id, data


def update_state(state: VehicleState, can_id: int, data: list):
    """Update vehicle state from a parsed CAN frame."""
    n = len(data)
    if n == 0:
        return

    def bit(b, pos):
        return bool(b & (1 << pos))

    def nibble(b, idx):
        return b & 0x0F if idx == 0 else (b >> 4) & 0x0F

    # ── CarControl ──────────────────────────────────────────────────────────────

    if can_id == 0x0A8:  # Braking, torque
        if n > 1:
            state.braking = nibble(data[1], 1) == 6
        if n > 2:
            raw = (data[2] << 8) | data[1]
            if raw >= 0x8000:
                raw -= 0x10000
            state.torque = raw / 32.0

    elif can_id == 0x0AA:  # RPM, throttle
        if n > 5:
            state.engine_rpm = ((data[5] << 8) | data[4]) // 4
        if n > 3:
            raw_thr = (data[3] << 8) | data[2]
            if n > 6 and data[6] == 0xB4:
                state.throttle_position = 255
            elif raw_thr <= 255:
                state.throttle_position = 0
            else:
                state.throttle_position = min(((raw_thr - 255) * 254) // 64809, 254)

    elif can_id == 0x130:  # Key state
        if n > 0:
            state.key_state_raw = data[0]
            state.key_state_available = True

    elif can_id == 0x1A1:  # Speed
        if n > 3:
            state.speed = ((data[3] << 8) | data[2]) / 100.0

    elif can_id == 0x0C8:  # Steering angle
        if n > 1:
            raw = (data[1] << 8) | data[0]
            signed = raw - 65536 if raw > 32767 else raw
            state.steering_angle = signed / 23.0

    elif can_id == 0x0E2:  # Door lock
        if n > 0:
            state.door_locked = data[0] == 2

    elif can_id == 0x0F6:  # Mirrors
        if n > 0:
            state.mirrors_retracted = data[0] == 0xF3

    elif can_id == 0x1B4:  # Parking brake
        if n > 5:
            state.parking_brake_on = data[5] == 0x32

    elif can_id == 0x1D0:  # Engine temp (byte 0: raw - 48 = °C)
        if n > 0:
            state.engine_temp = data[0] - 48

    elif can_id == 0x1E1:  # Driver door
        if n > 2:
            state.door_open_driver_front = nibble(data[2], 0) == 1

    elif can_id == 0x2B2:  # Brake status
        if n > 0:
            v = min(data[0], 0x80)
            state.brake_status = (v * 255) // 0x80

    elif can_id == 0x2F1:  # Seat belt
        if n > 2:
            state.seat_belt_plugged = bool(nibble(data[2], 0) & 0x01)

    elif can_id == 0x2FC:  # Individual door states
        if n > 1:
            state.door_open_driver_front     = bit(data[1], 0)
            state.door_open_passenger_front  = bit(data[1], 2)
            state.door_open_driver_rear      = bit(data[1], 4)
            state.door_open_passenger_rear   = bit(data[1], 6)

    elif can_id == 0x304:  # Gear position
        if n > 0 and (data[0] in (0xE3, 0xC2, 0xD1) or 0xC5 <= data[0] <= 0xCA):
            state.gear_position_raw = data[0]

    elif can_id == 0x330:  # Odometer, fuel, range
        if n > 2:
            state.odometer = (data[2] << 16) | (data[1] << 8) | data[0]
        if n > 3:
            state.fuel_level = data[3]
        if n > 7:
            state.range_km = ((data[7] << 8) | data[6]) / 16.0

    elif can_id == 0x3B4:  # Battery voltage, engine flag
        if n > 1:
            raw = (data[1] << 8) | data[0]
            state.battery_voltage = (raw - 0xF000) / 68.0
        if n > 2:
            state.engine_flag_from_can = data[2] == 0x00

    elif can_id == 0x3B6:  # Driver front window
        if n > 0:
            state.window_driver_front = (min(data[0], 0x50) * 255) // 0x50

    elif can_id == 0x3B7:  # Driver rear window
        if n > 0:
            state.window_driver_rear = (min(data[0], 0x50) * 255) // 0x50

    elif can_id == 0x3B8:  # Passenger front window
        if n > 0:
            state.window_passenger_front = (min(data[0], 0x50) * 255) // 0x50

    elif can_id == 0x3B9:  # Passenger rear window
        if n > 0:
            state.window_passenger_rear = (min(data[0], 0x50) * 255) // 0x50

    # ── ClimateControl ──────────────────────────────────────────────────────────

    elif can_id == 0x2E6:  # Fan speed, driver temp, blower distribution
        if n > 2:
            if data[0] == 0x00 and data[1] == 0x64 and data[2] == 0x1E:
                state.blower_state = 0  # AUTO
            else:
                bl = 0
                if data[0] > 0: bl |= 0x01  # Windshield
                if data[1] > 0: bl |= 0x02  # Center
                if data[2] > 0: bl |= 0x04  # Footwell
                state.blower_state = bl if bl else 0
        if n > 5:
            state.fan_speed_raw = data[5] & 0x07
        if n > 7:
            raw_t = data[7]
            if 0x20 <= raw_t <= 0x38:
                state.driver_temp = 16 + ((raw_t - 0x20) * 12) // 24

    elif can_id == 0x2EA:  # Passenger temperature
        if n > 7:
            raw_t = data[7]
            if 0x20 <= raw_t <= 0x38:
                state.passenger_temp = 16 + ((raw_t - 0x20) * 12) // 24

    elif can_id == 0x242:  # AC status, fan on/off
        if n > 0:
            state.ac_active = bit(data[0], 0)
        if n > 2:
            state.fan_on = bit(data[2], 0)

    elif can_id == 0x232:  # Driver seat heater
        if n > 0:
            state.driver_seat_heater = (data[0] & 0xF0) >> 4

    elif can_id == 0x22A:  # Passenger seat heater
        if n > 0:
            state.passenger_seat_heater = (data[0] & 0xF0) >> 4


# ─── Input Sources ──────────────────────────────────────────────────────────────

class SerialSource:
    def __init__(self, port, baudrate):
        self.ser = serial.Serial(port, baudrate, timeout=0.05)
        self._buf = b""

    def read_lines(self):
        chunk = self.ser.read(1024)
        if not chunk:
            return []
        self._buf += chunk
        lines = []
        while b'\n' in self._buf:
            line, self._buf = self._buf.split(b'\n', 1)
            lines.append(line.decode('utf-8', errors='ignore').strip())
        return lines

    def close(self):
        self.ser.close()


class FileSource:
    def __init__(self, path):
        self.f = open(path, 'r', encoding='utf-8')
        self._done = False

    def read_lines(self):
        if self._done:
            return []
        batch = []
        for _ in range(50):
            line = self.f.readline()
            if not line:
                self._done = True
                break
            batch.append(line.strip())
        return batch

    def close(self):
        self.f.close()


# ─── Curses Dashboard ───────────────────────────────────────────────────────────

C_TITLE  = 1
C_LABEL  = 2
C_VALUE  = 3
C_GOOD   = 4
C_WARN   = 5
C_DIM    = 6
C_HEADER = 7


def _bar(pct, width=10):
    filled = max(0, min(width, int(pct * width // 100)))
    return '█' * filled + '░' * (width - filled)


def draw(stdscr, state: VehicleState, source_name: str, show_reg: bool, frames: int):
    stdscr.erase()
    max_y, max_x = stdscr.getmaxyx()

    def put(row, col, text, attr=0):
        if row < 0 or row >= max_y or col < 0 or col >= max_x - 1:
            return
        text = text[:max(0, max_x - col - 1)]
        try:
            stdscr.addstr(row, col, text, attr)
        except curses.error:
            pass

    def kv(row, col, label, value, vc=C_VALUE):
        put(row, col,      f"{label:<22}",  curses.color_pair(C_LABEL))
        put(row, col + 22, str(value),       curses.color_pair(vc))

    # ── Header ─────────────────────────────────────────────────────────────────
    hdr = f" BMW E90 CAN Dashboard  │  {source_name}  │  frames: {frames} "
    put(0, 0, hdr.ljust(max_x - 1), curses.color_pair(C_TITLE) | curses.A_BOLD)
    put(1, 0, '─' * min(max_x - 1, 120), curses.color_pair(C_DIM))

    L = 0    # left column x
    R = 42   # right column x
    r = 2    # left row cursor
    r2 = 2   # right row cursor

    # ── Left: Engine & Powertrain ───────────────────────────────────────────────
    put(r, L, "ENGINE & POWERTRAIN", curses.color_pair(C_HEADER) | curses.A_BOLD); r += 1

    ign = state.get_ignition_status()
    ign_c = C_GOOD if ign == "Running" else (C_WARN if "IGN2" in ign else C_DIM)
    kv(r, L, "  Engine:", ign, ign_c); r += 1
    kv(r, L, "  Key:", state.get_key_state()); r += 1

    gear = state.get_gear()
    kv(r, L, "  Gear:", gear, C_VALUE if gear != "---" else C_DIM); r += 1
    drive_gear = state.get_drive_gear()
    kv(r, L, "  Drive Gear:", drive_gear if drive_gear is not None else "---",
       C_VALUE if drive_gear is not None else C_DIM); r += 1

    kv(r, L, "  Speed:", f"{state.speed:.1f} MPH",
       C_WARN if state.speed > 0 else C_VALUE); r += 1
    kv(r, L, "  RPM:", str(state.engine_rpm)); r += 1
    kv(r, L, "  Eng Temp:", f"{state.engine_temp}°C"); r += 1

    torque = state.torque if state.is_engine_running() else 0.0
    kv(r, L, "  Torque:", f"{torque:.1f} Nm"); r += 1
    kv(r, L, "  Power:", f"{state.get_power():.1f} KW"); r += 1
    kv(r, L, "  Throttle:", state.get_throttle_str()); r += 1

    bat_c = C_WARN if 0 < state.battery_voltage < 12.0 else C_VALUE
    kv(r, L, "  Battery:", f"{state.battery_voltage:.2f}V", bat_c); r += 1
    kv(r, L, "  Cranking:", "YES" if state.is_engine_cranking() else "No",
       C_WARN if state.is_engine_cranking() else C_VALUE); r += 2

    # ── Left: Fuel & Range ──────────────────────────────────────────────────────
    put(r, L, "FUEL & RANGE", curses.color_pair(C_HEADER) | curses.A_BOLD); r += 1
    kv(r, L, "  Odometer:", f"{state.odometer} KM"); r += 1
    kv(r, L, "  Fuel:", f"{state.fuel_level} L",
       C_WARN if 0 < state.fuel_level < 10 else C_VALUE); r += 1
    kv(r, L, "  Range:", f"{state.range_km:.0f} KM"); r += 2

    # ── Left: Vehicle Status ────────────────────────────────────────────────────
    put(r, L, "VEHICLE STATUS", curses.color_pair(C_HEADER) | curses.A_BOLD); r += 1

    bp = (state.brake_status * 100) // 255
    kv(r, L, "  Braking:", f"{bp}%", C_WARN if bp > 0 else C_VALUE); r += 1
    kv(r, L, "  Parking Brake:", "ON" if state.parking_brake_on else "Off",
       C_WARN if state.parking_brake_on else C_VALUE); r += 1
    kv(r, L, "  Steering:", f"{state.steering_angle:.1f}°"); r += 1
    kv(r, L, "  Doors:", "Locked" if state.door_locked else "Unlocked",
       C_GOOD if state.door_locked else C_WARN); r += 1
    doors_open = state.get_open_doors()
    kv(r, L, "  Doors Open:", doors_open,
       C_WARN if doors_open != "Closed" else C_VALUE); r += 1

    kv(r, L, "  Seatbelt:", "Plugged" if state.seat_belt_plugged else "Unplugged",
       C_GOOD if state.seat_belt_plugged else C_WARN); r += 1
    kv(r, L, "  Mirrors:", "Retracted" if state.mirrors_retracted else "Out"); r += 1

    # ── Right: Climate ──────────────────────────────────────────────────────────
    put(r2, R, "CLIMATE CONTROL", curses.color_pair(C_HEADER) | curses.A_BOLD); r2 += 1

    fan = state.get_fan_speed()
    fan_bar = '●' * fan + '○' * (7 - fan)
    kv(r2, R, "  Fan:", f"{fan}/7  {fan_bar}"); r2 += 1

    kv(r2, R, "  Blower:", state.get_blower_str()); r2 += 1
    kv(r2, R, "  Driver Temp:", f"{state.driver_temp}°C"); r2 += 1
    kv(r2, R, "  Pass. Temp:", f"{state.passenger_temp}°C"); r2 += 1
    kv(r2, R, "  AC:", "ON" if state.ac_active else "Off",
       C_GOOD if state.ac_active else C_DIM); r2 += 1

    dsh = state.get_heater_str(state.driver_seat_heater)
    kv(r2, R, "  Dr. Seat Heater:", dsh,
       C_WARN if state.driver_seat_heater > 0 else C_VALUE); r2 += 1
    psh = state.get_heater_str(state.passenger_seat_heater)
    kv(r2, R, "  Ps. Seat Heater:", psh,
       C_WARN if state.passenger_seat_heater > 0 else C_VALUE); r2 += 2

    # ── Right: Windows ──────────────────────────────────────────────────────────
    put(r2, R, "WINDOWS", curses.color_pair(C_HEADER) | curses.A_BOLD); r2 += 1

    for label, val in (
        ("  Driver Front: ", state.window_driver_front),
        ("  Pass. Front:  ", state.window_passenger_front),
        ("  Driver Rear:  ", state.window_driver_rear),
        ("  Pass. Rear:   ", state.window_passenger_rear),
    ):
        pct = (val * 100) // 255
        kv(r2, R, label, f"{_bar(pct, 10)} {pct:3d}%"); r2 += 1

    r2 += 1

    # ── Register matrix ─────────────────────────────────────────────────────────
    sep = max(r, r2) + 1
    if sep < max_y - 3:
        put(sep, 0, '─' * min(max_x - 1, 120), curses.color_pair(C_DIM))

        if show_reg:
            put(sep + 1, 0,
                f"CAN REGISTERS  [{len(state.registers)} IDs]  press 'r' to hide",
                curses.color_pair(C_DIM) | curses.A_BOLD)
            rr = sep + 2
            col_w = 26
            ncols = max(1, (max_x - 1) // col_w)
            for i, cid in enumerate(sorted(state.registers)):
                if rr >= max_y - 1:
                    break
                rx = rr + i // ncols
                cx = (i % ncols) * col_w
                if rx >= max_y - 1:
                    break
                hex_str = ' '.join(f'{b:02X}' for b in state.registers[cid][:8])
                put(rx, cx, f"{cid:03X}: {hex_str}"[:col_w - 1], curses.color_pair(C_DIM))
        else:
            put(sep + 1, 0,
                f"  {len(state.registers)} CAN IDs seen  │  press 'r' to show register matrix",
                curses.color_pair(C_DIM))

    put(max_y - 1, 0, " q:quit  r:registers ", curses.color_pair(C_DIM))
    stdscr.refresh()


def run_curses(stdscr, source, source_name: str):
    curses.start_color()
    curses.use_default_colors()
    curses.init_pair(C_TITLE,  curses.COLOR_BLACK,  curses.COLOR_GREEN)
    curses.init_pair(C_LABEL,  curses.COLOR_WHITE,  -1)
    curses.init_pair(C_VALUE,  curses.COLOR_CYAN,   -1)
    curses.init_pair(C_GOOD,   curses.COLOR_GREEN,  -1)
    curses.init_pair(C_WARN,   curses.COLOR_YELLOW, -1)
    curses.init_pair(C_DIM,    curses.COLOR_WHITE,  -1)
    curses.init_pair(C_HEADER, curses.COLOR_BLUE,   -1)

    curses.curs_set(0)
    stdscr.nodelay(True)

    state = VehicleState()
    show_reg = False
    frames = 0
    last_draw = 0.0

    while True:
        try:
            key = stdscr.getch()
            if key in (ord('q'), ord('Q')):
                break
            elif key in (ord('r'), ord('R')):
                show_reg = not show_reg
        except Exception:
            pass

        for line in source.read_lines():
            can_id, data = parse_line(line)
            if can_id is not None:
                state.registers[can_id] = data
                update_state(state, can_id, data)
                frames += 1

        now = time.monotonic()
        if now - last_draw >= 0.1:
            draw(stdscr, state, source_name, show_reg, frames)
            last_draw = now
        else:
            time.sleep(0.01)


# ─── Simple fallback ────────────────────────────────────────────────────────────

def run_simple(source, source_name: str):
    state = VehicleState()
    frames = 0
    last_print = 0.0

    print(f"BMW E90 CAN Dashboard  [{source_name}]  Ctrl+C to stop\n")
    try:
        while True:
            for line in source.read_lines():
                can_id, data = parse_line(line)
                if can_id is not None:
                    state.registers[can_id] = data
                    update_state(state, can_id, data)
                    frames += 1

            now = time.monotonic()
            if now - last_print >= 1.0:
                print(f"\n=== {frames} frames | {len(state.registers)} CAN IDs ===")
                print(f"Engine: {state.get_ignition_status()} | RPM: {state.engine_rpm} | Speed: {state.speed:.1f} MPH")
                drive_gear = state.get_drive_gear()
                drive_gear_str = drive_gear if drive_gear is not None else "---"
                print(f"Gear: {state.get_gear()} | Drive Gear: {drive_gear_str} | Key: {state.get_key_state()} | Cranking: {state.is_engine_cranking()}")
                print(f"Eng Temp: {state.engine_temp}C | Battery: {state.battery_voltage:.2f}V | Throttle: {state.get_throttle_str()}")
                print(f"Torque: {state.torque:.1f} Nm | Power: {state.get_power():.1f} KW")
                print(f"Odometer: {state.odometer} KM | Fuel: {state.fuel_level} L | Range: {state.range_km:.0f} KM")
                print(f"Doors: {'Locked' if state.door_locked else 'Unlocked'} | Open: {state.get_open_doors()}")
                print(f"Parking Brake: {'ON' if state.parking_brake_on else 'Off'} | Seatbelt: {'Plugged' if state.seat_belt_plugged else 'Unplugged'}")
                print(f"Climate: Fan {state.get_fan_speed()}/7 | Blower: {state.get_blower_str()} | AC: {'ON' if state.ac_active else 'Off'}")
                print(f"  Temps: Driver={state.driver_temp}C  Passenger={state.passenger_temp}C")
                print(f"  Seat Heaters: Driver={state.get_heater_str(state.driver_seat_heater)} Passenger={state.get_heater_str(state.passenger_seat_heater)}")
                wdf = state.window_driver_front * 100 // 255
                wpf = state.window_passenger_front * 100 // 255
                wdr = state.window_driver_rear * 100 // 255
                wpr = state.window_passenger_rear * 100 // 255
                print(f"Windows: DF={wdf}% PF={wpf}% DR={wdr}% PR={wpr}%")
                last_print = now
            else:
                time.sleep(0.01)
    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        source.close()


# ─── Entry Point ────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description="BMW E90 CAN Dashboard - Real-time vehicle status",
        epilog="Requires: pip install pyserial windows-curses (Windows only)"
    )
    parser.add_argument("port", nargs="?", help="Serial port (e.g., COM3, /dev/ttyUSB0)")
    parser.add_argument("-b", "--baudrate", type=int, default=115200,
                        help="Baud rate (default: 115200)")
    parser.add_argument("-l", "--list", action="store_true",
                        help="List available serial ports")
    parser.add_argument("-s", "--simple", action="store_true",
                        help="Simple text output (no curses)")
    parser.add_argument("--replay", type=str, metavar="FILE",
                        help="Replay from a log file instead of serial")
    args = parser.parse_args()

    if args.list:
        for p in serial.tools.list_ports.comports():
            print(f"  {p.device} - {p.description}")
        return

    if args.replay:
        source = FileSource(args.replay)
        source_name = f"Replay: {args.replay}"
    elif args.port:
        source = SerialSource(args.port, args.baudrate)
        source_name = f"{args.port} @ {args.baudrate}"
    else:
        print("Error: specify a serial port or --replay FILE\n")
        parser.print_help()
        return

    if args.simple or not CURSES_AVAILABLE:
        if not CURSES_AVAILABLE:
            print("Note: curses unavailable, using simple mode. Install: pip install windows-curses\n")
        run_simple(source, source_name)
    else:
        try:
            curses.wrapper(lambda s: run_curses(s, source, source_name))
        finally:
            source.close()


if __name__ == "__main__":
    main()
