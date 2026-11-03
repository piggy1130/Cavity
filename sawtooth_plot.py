#!/usr/bin/env python3
"""
sawtooth_plot.py - Raspberry Pi side of the Pico sawtooth DAC / ADC system.

The Pico sends one binary frame per sawtooth over USB serial (little-endian):
    header  8 x uint32 : magic 'SAWF', frame_id, num_points, point_period_us,
                         settle_us, t0_us, overruns, dropped
    DAC     num_points x uint16 : DAC code written for point i
    ADC     num_points x uint16 : ADC reading taken after DAC[i] had settled
    CRC-32  uint32 : zlib.crc32 of header + DAC + ADC

Each frame is checked, converted to volts and drawn over the previous one, so a
repeating sawtooth shows up as a steady picture.

    python3 sawtooth_plot.py                   # find the Pico automatically, live plot
    python3 sawtooth_plot.py --input-scale 1   # analog signal wired straight to GP26
    python3 sawtooth_plot.py --stats           # no window: one text line per frame
"""

import argparse
import struct
import sys
import threading
import time
import zlib
from dataclasses import dataclass

import numpy as np
import serial
from serial.tools import list_ports

# ----------------------------------------------------- must match sawtooth_daq.c
DAC_MAX_VALUE = 4095
DAC_FULL_SCALE_VOLTAGE = 5.0   # MCP4921 VREF = VCC = 5 V (DAC Click REFERENCE jumper = VCC)

# ----------------------------------------------------- ADC side (RP2040, GP26 = ADC0)
ADC_FULL_SCALE_COUNTS = 4096   # 12-bit SAR: V_pin = raw * ADC_VREF / 4096
ADC_VREF = 3.3                 # Pico ADC_VREF = 3V3 rail; measure it for better accuracy

# GP26 is wired directly to the DAC output: no input scaling, ADC volts = GP26 volts.
# Only if a divider is added (signal --R_TOP-- GP26 --R_BOTTOM-- AGND), pass
# --input-scale (R_TOP + R_BOTTOM) / R_BOTTOM.
ADC_INPUT_SCALE = 1.0

# ----------------------------------------------------- wire format
MAGIC = b"SAWF"
HEADER = struct.Struct("<4s7I")
CRC = struct.Struct("<I")
MAX_POINTS = 100_000
PICO_USB_VID = 0x2E8A          # Raspberry Pi
PICO_SDK_PIDS = (0x000A, 0x0009)   # Pico SDK USB serial = the sawtooth_daq firmware
OTHER_PIDS = {0x0005: "MicroPython", 0x000C: "Debug Probe UART", 0x0004: "picoprobe"}


@dataclass
class Frame:
    frame_id: int
    period_us: int
    settle_us: int
    t0_us: int
    overruns: int
    dropped: int
    dac: np.ndarray   # uint16 DAC codes, index i
    adc: np.ndarray   # uint16 ADC readings, same index i


def dac_volts(codes):
    return codes.astype(np.float64) * (DAC_FULL_SCALE_VOLTAGE / DAC_MAX_VALUE)


def adc_volts(raw, vref=ADC_VREF, input_scale=ADC_INPUT_SCALE):
    return raw.astype(np.float64) * (vref / ADC_FULL_SCALE_COUNTS) * input_scale


class FrameParser:
    """Rebuilds frames from an arbitrary byte stream and resynchronises after any
    garbage or a torn frame (search for the magic, then check the CRC).
    The magic cannot occur inside the DAC/ADC data: every 12-bit sample has a
    high byte <= 0x0F, while 'S','A','W','F' are all > 0x40."""

    def __init__(self):
        self.buf = bytearray()
        self.bad = 0                   # discarded headers / CRC failures

    def reset(self):
        self.buf.clear()

    def feed(self, data):
        self.buf += data
        frames = []
        while True:
            k = self.buf.find(MAGIC)
            if k < 0:                  # keep a possible partial magic at the end
                del self.buf[:max(0, len(self.buf) - (len(MAGIC) - 1))]
                return frames
            del self.buf[:k]
            if len(self.buf) < HEADER.size:
                return frames
            _, fid, n, period, settle, t0, overruns, dropped = HEADER.unpack_from(self.buf)
            if not 0 < n <= MAX_POINTS:
                self.bad += 1
                del self.buf[:1]
                continue
            size = HEADER.size + 4 * n + CRC.size
            if len(self.buf) < size:
                return frames
            body = bytes(self.buf[:size - CRC.size])
            (crc,) = CRC.unpack_from(self.buf, size - CRC.size)
            if zlib.crc32(body) != crc:
                self.bad += 1
                del self.buf[:1]
                continue
            del self.buf[:size]
            samples = np.frombuffer(body, dtype="<u2", offset=HEADER.size)
            frames.append(Frame(fid, period, settle, t0, overruns, dropped,
                                samples[:n].copy(), samples[n:].copy()))


def describe_port(p):
    if p is None:
        return "?"
    ids = f"{p.vid:04X}:{p.pid:04X}" if p.vid is not None else "no USB id"
    what = OTHER_PIDS.get(p.pid, "") if p.vid == PICO_USB_VID else ""
    return f"{p.device}  [{ids} {p.product or p.description or ''}{' = ' + what if what else ''}]"


def find_pico_port():
    """Only the Pico SDK USB serial (2E8A:000A) sends frames. Other Raspberry Pi
    devices (MicroPython board, Debug Probe) also show up as /dev/ttyACM* but stay silent."""
    ports = sorted((p for p in list_ports.comports()
                    if p.vid == PICO_USB_VID and p.pid in PICO_SDK_PIDS), key=lambda p: p.device)
    return ports[0] if ports else None


class FrameSource(threading.Thread):
    """Reads the serial port in the background and keeps only the newest frame,
    so the display never lags behind the Pico. Reconnects if the Pico resets."""

    def __init__(self, port=None):
        super().__init__(daemon=True)
        self.port = port
        self.parser = FrameParser()
        self.status = "looking for the Pico ..."
        self.connected = False
        self.port_info = None
        self.bytes_rx = 0              # raw bytes read from the port (diagnostics)
        self.first_bytes = b""
        self.received = 0
        self.missed = 0                # frame_id gaps seen on the Pi
        self.sweep_ms = float("nan")   # measured sawtooth period (Pico timer)
        self._prev = None
        self._latest = None
        self._cond = threading.Condition()

    @property
    def crc_errors(self):
        return self.parser.bad

    def run(self):
        while True:
            if self.port:
                port = self.port
                info = next((p for p in list_ports.comports() if p.device == port), None)
            else:
                info = find_pico_port()
                port = info.device if info else None
            if port is None:
                self.status = ("no Pico SDK serial port (USB id 2E8A:000A) found - is the Pico "
                               "plugged in and running sawtooth_daq.uf2?")
                time.sleep(1.0)
                continue
            try:
                # exclusive: refuse to share the port with another copy of this script
                with serial.Serial(port, timeout=0.2, exclusive=True) as ser:  # baud is ignored
                    ser.reset_input_buffer()
                    self.parser.reset()
                    self._prev = None
                    self.port_info = info
                    self.connected = True
                    self.status = "connected to " + (describe_port(info) if info else port)
                    while True:
                        data = ser.read(max(1, ser.in_waiting))
                        if data:
                            self.bytes_rx += len(data)
                            if len(self.first_bytes) < 48:
                                self.first_bytes += data[:48 - len(self.first_bytes)]
                        for frame in self.parser.feed(data):
                            self._publish(frame)
            except (serial.SerialException, OSError) as exc:
                self.connected = False
                self.status = f"serial error on {port} ({exc}); retrying"
                time.sleep(1.0)

    def diagnose(self):
        """One-line guess at why no frame has arrived yet."""
        if not self.connected:
            return ""
        p = self.port_info
        if p is not None and p.vid == PICO_USB_VID and p.pid in OTHER_PIDS:
            return (f"this port is a {OTHER_PIDS[p.pid]}, not the C firmware: flash "
                    "sawtooth_daq.uf2 onto the Pico or pick the right port with --port")
        if self.bytes_rx == 0:
            return ("nothing is arriving: the device on this port is not running "
                    "sawtooth_daq.uf2 (check that the Pico LED blinks ~3x per second)")
        if self.crc_errors:
            return ("data arrives but frames are incomplete: another program is probably "
                    "reading the same port - check with: sudo fuser -v /dev/ttyACM*")
        return f"data arrives but it is not frame data; first bytes: {self.first_bytes!r}"

    def _publish(self, f):
        if self._prev is not None:
            gap = (f.frame_id - self._prev[0]) & 0xFFFFFFFF
            if 0 < gap < 1 << 31:      # otherwise the Pico has restarted
                self.missed += gap - 1
                self.sweep_ms = ((f.t0_us - self._prev[1]) & 0xFFFFFFFF) / gap / 1000.0
        self._prev = (f.frame_id, f.t0_us)
        with self._cond:
            self._latest = f
            self.received += 1
            self._cond.notify_all()

    def wait_frame(self, timeout=None):
        """Newest frame not handed out yet, or None after `timeout` seconds."""
        with self._cond:
            if self._latest is None:
                self._cond.wait(timeout)
            f, self._latest = self._latest, None
            return f


def status_line(f, src, vref, input_scale):
    a = adc_volts(f.adc, vref, input_scale)
    return (f"frame {f.frame_id}   sweep {src.sweep_ms:7.3f} ms   "
            f"ADC {a.min():.3f}..{a.max():.3f} V   late pts {f.overruns}   "
            f"not sent {f.dropped}   missed {src.missed}   CRC err {src.crc_errors}")


def build_figure(first, vref, input_scale):
    """Figure with fixed axes; returns (fig, update) where update(frame, text) redraws
    the two traces in place."""
    import matplotlib.pyplot as plt

    n = len(first.dac)
    ms_per_point = first.period_us / 1000.0
    t_ms = np.arange(n) * ms_per_point

    fig, ax = plt.subplots(figsize=(11, 5.5), constrained_layout=True)
    (l_dac,) = ax.plot(t_ms, dac_volts(first.dac), lw=1.6, color="#1f77b4",
                       label="DAC output (MCP4921)")
    (l_adc,) = ax.plot(t_ms, adc_volts(first.adc, vref, input_scale), lw=0.8,
                       color="#d62728", label="ADC input GP26" +
                       ("" if input_scale == 1 else f" (x{input_scale:.4g} scale)"))
    # Highest voltage the ADC can report (raw 4095); anything above reads as this value.
    ax.axhline(vref * input_scale, color="gray", ls="--", lw=0.8,
               label=f"ADC full scale ({vref * input_scale:.2f} V)")
    ax.set_xlim(0, n * ms_per_point)
    ax.set_ylim(0, 1.04 * max(DAC_FULL_SCALE_VOLTAGE, vref * input_scale))
    ax.set_xlabel(f"time within sawtooth (ms)   [{first.period_us} us per point, "
                  f"ADC read {first.settle_us} us after each DAC update]")
    ax.set_ylabel("voltage (V)")
    top = ax.secondary_xaxis("top", functions=(lambda t: t / ms_per_point,
                                               lambda i: i * ms_per_point))
    top.set_xlabel("sample index")
    ax.grid(alpha=0.3)
    ax.legend(loc="upper left")
    txt = ax.text(0.995, 0.015, "", transform=ax.transAxes, ha="right", va="bottom",
                  fontsize=8, family="monospace")
    fig.canvas.manager.set_window_title("Pico sawtooth DAC / ADC")

    def update(f, text=""):
        if len(f.dac) != n:
            txt.set_text("frame length changed - restart this script")
            return
        l_dac.set_ydata(dac_volts(f.dac))
        l_adc.set_ydata(adc_volts(f.adc, vref, input_scale))
        txt.set_text(text)

    return fig, update


def first_frame(src):
    print("serial ports on this computer:")
    for p in sorted(list_ports.comports(), key=lambda p: p.device):
        print("   ", describe_port(p))
    print("waiting for the first frame from the Pico ...", flush=True)
    while True:
        f = src.wait_frame(timeout=2.0)
        if f is not None:
            print("  first frame received - " + src.status, flush=True)
            return f
        line = "  " + src.status
        if src.connected:
            line += f"   ({src.bytes_rx} bytes received, {src.crc_errors} bad frames)"
        print(line, flush=True)
        hint = src.diagnose()
        if hint:
            print("    -> " + hint, flush=True)


def run_live_plot(src, vref, input_scale):
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation

    fig, update = build_figure(first_frame(src), vref, input_scale)

    def tick(_):
        f = src.wait_frame(timeout=0)
        if f is not None:
            update(f, status_line(f, src, vref, input_scale))

    anim = FuncAnimation(fig, tick, interval=100, cache_frame_data=False)  # noqa: F841
    plt.show()


def run_stats(src, vref, input_scale):
    first_frame(src)
    while True:
        f = src.wait_frame(timeout=2.0)
        if f is None:
            print("  no frame for 2 s: " + src.status, flush=True)
            continue
        print(status_line(f, src, vref, input_scale), flush=True)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", help="serial device, e.g. /dev/ttyACM0 (default: auto-detect)")
    ap.add_argument("--adc-vref", type=float, default=ADC_VREF,
                    help=f"Pico ADC reference in volts (default {ADC_VREF})")
    ap.add_argument("--input-scale", type=float, default=ADC_INPUT_SCALE,
                    help="signal volts per GP26 volt: 1 for a direct wire (default), "
                         "(R_TOP+R_BOTTOM)/R_BOTTOM if a divider is added")
    ap.add_argument("--stats", action="store_true", help="print frame stats instead of plotting")
    args = ap.parse_args()

    src = FrameSource(args.port)
    src.start()
    try:
        if args.stats:
            run_stats(src, args.adc_vref, args.input_scale)
        else:
            run_live_plot(src, args.adc_vref, args.input_scale)
    except KeyboardInterrupt:
        pass
    return 0


if __name__ == "__main__":
    sys.exit(main())
