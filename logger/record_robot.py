"""
record_robot.py - record the rover's log (rover_logger_v2.ino) to CSV, over USB serial or WiFi UDP.

install once:   pip install pyserial            (only needed for USB serial)

USB cable:      python record_robot.py --port COM12 --label run1
WiFi (UDP):     python record_robot.py --udp 4210 --label run1        (sketch with USE_WIFI 1)
list ports:     python record_robot.py --list

While recording press a key in this window:
    m = marker (e.g. "robot reached C")   z = zero yaw   c = re-calibrate gyro (robot still!)   q = stop
Ctrl+C also stops.

Writes into --out (default: ./data next to this script):
    raw_robot_data_<label>_<date_time>.csv        the data (same format the analysis reads)
    raw_robot_data_<label>_<date_time>_meta.json  label, notes, robot geometry, device messages, markers
"""
import argparse, csv, json, os, socket, sys, threading, time
from datetime import datetime

DEFAULT_HEADER = ("timestamp_ms,front_mm,right_mm,left_mm,back_mm,acc_x,acc_y,acc_z,"
                  "gyro_x,gyro_y,gyro_z,yaw_deg,gz_peak,st_f,st_r,st_l,st_b").split(",")
HERE = os.path.dirname(os.path.abspath(__file__))


# ------------------------------------------------------------------ transports
class SerialLink:
    def __init__(self, port, baud):
        import serial                                    # pip install pyserial
        self.s = serial.Serial(port, baud, timeout=0.2)
        self.buf = b""

    def lines(self):
        data = self.s.read(self.s.in_waiting or 1)
        self.buf += data
        *done, self.buf = self.buf.split(b"\n")
        return [d.decode("utf-8", "replace").strip() for d in done]

    def send(self, ch):
        self.s.write(ch.encode())

    def close(self):
        self.s.close()


class UdpLink:
    def __init__(self, port, esp_ip=None):
        self.port = port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        self.sock.bind(("0.0.0.0", port))
        self.sock.settimeout(0.2)
        self.esp = esp_ip
        self.last_hello = 0.0

    def lines(self):
        now = time.time()
        if now - self.last_hello > 1.0:                  # tell the ESP32 where we are
            self.send("h"); self.last_hello = now
        out = []
        try:
            while True:
                data, addr = self.sock.recvfrom(4096)
                txt = data.decode("utf-8", "replace")
                if txt.strip() in ("h", "H", "m", "z", "c"):   # our own broadcast echoed back
                    continue
                if self.esp is None:
                    self.esp = addr[0]
                    print(f"\n[udp] data from ESP32 at {self.esp}")
                out += [t.strip() for t in txt.splitlines() if t.strip()]
                self.sock.settimeout(0.0)
        except (socket.timeout, BlockingIOError):
            pass
        finally:
            self.sock.settimeout(0.2)
        return out

    def send(self, ch):
        dst = self.esp or "255.255.255.255"
        try:
            self.sock.sendto(ch.encode(), (dst, self.port))
        except OSError:
            pass

    def close(self):
        self.sock.close()


# ------------------------------------------------------------------ keyboard (non-blocking)
class Keys:
    def __init__(self):
        self.q = []
        if os.name == "nt":
            import msvcrt
            self.msvcrt = msvcrt
        else:
            self.msvcrt = None
            threading.Thread(target=self._stdin, daemon=True).start()

    def _stdin(self):                                     # Linux/macOS: type a letter + Enter
        for line in sys.stdin:
            self.q += list(line.strip())

    def get(self):
        if self.msvcrt:
            while self.msvcrt.kbhit():
                self.q.append(self.msvcrt.getwch())
        out, self.q = self.q, []
        return out


# ------------------------------------------------------------------ recorder
def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    g = ap.add_mutually_exclusive_group()
    g.add_argument("--port", help="serial port, e.g. COM12 or /dev/ttyUSB0")
    g.add_argument("--udp", type=int, nargs="?", const=4210, help="listen for WiFi UDP data on this port (default 4210)")
    g.add_argument("--list", action="store_true", help="list serial ports and exit")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--esp-ip", help="ESP32 IP for UDP commands (optional, learned automatically)")
    ap.add_argument("--label", default="run", help="short name for this run, e.g. arena2_manual")
    ap.add_argument("--out", default=os.path.join(HERE, "data"), help="output folder")
    ap.add_argument("--notes", default="", help="free text saved in the meta file (how it was driven, arena changes...)")
    ap.add_argument("--sensor-front", type=float, help="robot centre -> front ToF (mm), saved for the analysis")
    ap.add_argument("--sensor-back", type=float, help="robot centre -> back ToF (mm)")
    ap.add_argument("--sensor-side", type=float, help="robot centre -> left/right ToF (mm)")
    ap.add_argument("--duration", type=float, default=0, help="stop automatically after N seconds (0 = manual)")
    a = ap.parse_args()

    if a.list:
        from serial.tools import list_ports
        for p in list_ports.comports():
            print(p.device, "-", p.description)
        return
    if not a.port and a.udp is None:
        ap.error("give --port COMx (USB) or --udp (WiFi)")

    os.makedirs(a.out, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    base = os.path.join(a.out, f"raw_robot_data_{a.label}_{stamp}")
    link = SerialLink(a.port, a.baud) if a.port else UdpLink(a.udp, a.esp_ip)
    keys = Keys()
    meta = dict(label=a.label, started=datetime.now().isoformat(timespec="seconds"), notes=a.notes,
                transport=f"serial {a.port}" if a.port else f"udp {a.udp}",
                sensor_offsets_mm=dict(front=a.sensor_front, back=a.sensor_back, side=a.sensor_side),
                device_messages=[], markers=[])

    print(f"recording -> {base}.csv")
    print("keep the robot STILL until '#CAL' appears (gyro calibration).  keys: m=marker z=zero-yaw c=calibrate q=quit")
    if a.port:
        time.sleep(0.5)          # opening the port resets the ESP32; it calibrates and prints the header
    else:
        link.send("H")

    header, fcsv, writer = None, None, None
    rows = bad = 0
    first_ms = last_ms = None
    invalid = [0, 0, 0, 0]
    t_start = time.time(); t_status = 0.0; last_row = None
    try:
        while True:
            for k in keys.get():
                k = k.strip()
                if k == "q":
                    raise KeyboardInterrupt
                if k in ("m", "z", "c"):
                    link.send(k)
                    if k == "m":
                        meta["markers"].append(dict(pc_time=time.time() - t_start, last_timestamp_ms=last_ms))
                        print(f"\n[marker {len(meta['markers'])}] at device ms {last_ms}")
            for line in link.lines():
                if not line:
                    continue
                if line.startswith("#"):
                    meta["device_messages"].append(line)
                    print("\n[esp32]", line)
                    continue
                if line.startswith("timestamp_ms"):
                    new_header = line.split(",")
                    if header is None:
                        header = new_header
                    elif new_header != header:
                        print("\n[warn] header changed - ignoring new header")
                    continue
                parts = line.split(",")
                if header is None:
                    if len(parts) != len(DEFAULT_HEADER):
                        continue                          # wait for a header / a full line
                    header = DEFAULT_HEADER
                if len(parts) != len(header):
                    bad += 1; continue
                try:
                    vals = [float(p) for p in parts]
                except ValueError:
                    bad += 1; continue
                if writer is None:
                    fcsv = open(base + ".csv", "w", newline="")
                    writer = csv.writer(fcsv); writer.writerow(header)
                writer.writerow(parts)
                rows += 1
                ms = vals[0]
                first_ms = ms if first_ms is None else first_ms
                last_ms = ms
                for i in range(4):
                    if vals[1 + i] < 0:
                        invalid[i] += 1
                last_row = dict(zip(header, parts))
            now = time.time()
            if now - t_status > 1.0 and last_row:
                t_status = now
                dur = (last_ms - first_ms) / 1000 if rows > 1 else 0
                hz = (rows - 1) / dur if dur > 0 else 0
                yaw = last_row.get("yaw_deg", "-")
                print(f"\r{rows:6d} rows {hz:5.1f} Hz | F {last_row['front_mm']:>5} R {last_row['right_mm']:>5} "
                      f"L {last_row['left_mm']:>5} B {last_row['back_mm']:>5} | yaw {yaw:>8} deg | bad {bad}   ",
                      end="", flush=True)
                if fcsv:
                    fcsv.flush()
            if a.duration and now - t_start > a.duration:
                break
    except KeyboardInterrupt:
        pass
    finally:
        link.close()
        if fcsv:
            fcsv.close()
        dur = (last_ms - first_ms) / 1000 if rows > 1 else 0
        meta.update(rows=rows, bad_lines=bad, duration_s=round(dur, 2),
                    rate_hz=round((rows - 1) / dur, 2) if dur > 0 else 0,
                    invalid_counts=dict(zip(["front", "right", "left", "back"], invalid)),
                    stopped=datetime.now().isoformat(timespec="seconds"))
        with open(base + "_meta.json", "w") as f:
            json.dump(meta, f, indent=2)
        print(f"\n\nsaved {rows} rows ({dur:.1f} s, {meta['rate_hz']} Hz, {bad} bad lines) -> {base}.csv")
        print("invalid readings  front/right/left/back:", *invalid)


if __name__ == "__main__":
    main()
