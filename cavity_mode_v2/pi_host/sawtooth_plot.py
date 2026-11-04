#!/usr/bin/env python3
"""
sawtooth_plot.py - Raspberry Pi side of the Pico sawtooth DAC / ADC system.

The Pico sends one binary frame per sawtooth over USB serial (little-endian):
    header  9 x uint32 : magic 'SAWF', frame_id, num_points, point_period_us,
                         settle_us, t0_us, overruns, dropped, trigger_delay_us
    DAC     num_points x uint16 : DAC code written for point i
    ADC     num_points x uint16 : ADC reading taken after DAC[i] had settled
    CRC-32  uint32 : zlib.crc32 of header + DAC + ADC

The Pi sends commands back over the same port, one ASCII line each:
    TRIG_DELAY_US=<whole microseconds>\\n   GP28 trigger delay after the start of each sawtooth

Each frame is checked, converted to volts and drawn over the previous one, so a
repeating sawtooth shows up as a steady picture.

Left-click on the plot to set the GP28 trigger delay to the time under the mouse:
the x value the toolbar shows, e.g. x=87.4 -> 87.4 ms (sent as 87400 us). The green
line marks the delay the Pico reports using; it is dashed while a new value is on its
way. Clicks are ignored while the toolbar's pan or zoom mode is on.

    python3 sawtooth_plot.py                   # find the Pico automatically, live plot
    python3 sawtooth_plot.py --input-scale 1   # analog signal wired straight to GP26
    python3 sawtooth_plot.py --trigger-ms 87.4 # also set the GP28 trigger delay at start-up
    python3 sawtooth_plot.py --stats           # no window: one text line per frame
"""

import argparse
import math
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
TRIGGER_PULSE_US = 10          # GP28 pulse width; limits the largest trigger delay

# ----------------------------------------------------- ADC side (RP2040, GP26 = ADC0)
ADC_FULL_SCALE_COUNTS = 4096   # 12-bit SAR: V_pin = raw * ADC_VREF / 4096
ADC_VREF = 3.3                 # Pico ADC_VREF = 3V3 rail; measure it for better accuracy

# GP26 is wired directly to the DAC output: no input scaling, ADC volts = GP26 volts.
# Only if a divider is added (signal --R_TOP-- GP26 --R_BOTTOM-- AGND), pass
# --input-scale (R_TOP + R_BOTTOM) / R_BOTTOM.
ADC_INPUT_SCALE = 1.0

# ----------------------------------------------------- wire format
MAGIC = b"SAWF"
HEADER = struct.Struct("<4s8I")
CRC = struct.Struct("<I")
TRIGGER_CMD = "TRIG_DELAY_US={:d}\n"   # Pi -> Pico, delay in whole microseconds
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
    trigger_us: int   # GP28 delay after t0 (us) the Pico used for this sawtooth
    dac: np.ndarray   # uint16 DAC codes, index i
    adc: np.ndarray   # uint16 ADC readings, same index i


def trigger_delay_max_us(num_points, period_us):
    """Largest delay the Pico accepts (TRIGGER_DELAY_MAX_US in sawtooth_daq.c)."""
    return (num_points - 3) * period_us - TRIGGER_PULSE_US


def ms_to_us(ms):
    """Plot time (ms, may be fractional) -> Pico timer units (whole us): 87.4 -> 87400."""
    return int(round(ms * 1000.0))


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
            _, fid, n, period, settle, t0, overruns, dropped, trig = HEADER.unpack_from(self.buf)
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
            frames.append(Frame(fid, period, settle, t0, overruns, dropped, trig,
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
        self._trig_lock = threading.Lock()
        self._trig_us = None           # GP28 delay asked for (us); None = never sent one
        self._trig_unsent = False

    @property
    def crc_errors(self):
        return self.parser.bad

    @property
    def trigger_request_us(self):
        return self._trig_us

    def set_trigger_delay_us(self, delay_us):
        """Queue a new GP28 trigger delay (whole microseconds) for the Pico. The serial
        thread sends it, and sends it again after a reconnect (a reset Pico is back at 50 ms)."""
        with self._trig_lock:
            self._trig_us = int(delay_us)
            self._trig_unsent = True

    def _send_trigger(self, ser):
        with self._trig_lock:
            if not self._trig_unsent:
                return
            delay_us, self._trig_unsent = self._trig_us, False
        ser.write(TRIGGER_CMD.format(delay_us).encode("ascii"))

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
                with serial.Serial(port, timeout=0.2, write_timeout=1.0,
                                   exclusive=True) as ser:  # baud is ignored
                    ser.reset_input_buffer()
                    self.parser.reset()
                    self._prev = None
                    self.port_info = info
                    self.connected = True
                    self.status = "connected to " + (describe_port(info) if info else port)
                    with self._trig_lock:        # the Pico may have been reset: send again
                        self._trig_unsent = self._trig_us is not None
                    while True:
                        self._send_trigger(ser)  # only this thread touches the port
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
            return ("data arrives but frames fail the CRC check: either the Pico runs an older "
                    "sawtooth_daq.uf2 (reflash the one built from this version), or another "
                    "program is reading the same port - check with: sudo fuser -v /dev/ttyACM*")
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
    line = (f"frame {f.frame_id}   sweep {src.sweep_ms:7.3f} ms   "
            f"ADC {a.min():.3f}..{a.max():.3f} V   late pts {f.overruns}   "
            f"not sent {f.dropped}   missed {src.missed}   CRC err {src.crc_errors}   "
            f"trig {f.trigger_us / 1000:.3f} ms")
    req = src.trigger_request_us
    if req is not None and req != f.trigger_us:
        line += f" (-> {req / 1000:.3f})"
    return line


def request_trigger_delay(src, delay_us, frame):
    """Check a delay (whole us) against the sweep and queue it for the Pico.
    Returns True if it will be sent."""
    max_us = trigger_delay_max_us(len(frame.dac), frame.period_us)
    if not 0 <= delay_us <= max_us:
        print(f"trigger delay {delay_us / 1000:.3f} ms not sent: the allowed range is "
              f"0 .. {max_us / 1000:.3f} ms", flush=True)
        return False
    src.set_trigger_delay_us(delay_us)
    print(f"trigger delay -> {delay_us / 1000:.3f} ms ({delay_us} us), sending to the Pico",
          flush=True)
    return True


def click_to_delay_us(ax, x_ms):
    """Round a clicked time to the digits the toolbar shows for the mouse position, then
    convert it to whole us. matplotlib prints about one screen pixel's worth of digits
    (ScalarFormatter.format_data_short); this uses the same rule, so clicking where the
    toolbar reads x=87.4 sends exactly 87400 us, not e.g. 87434 us."""
    x0, x1 = ax.get_xlim()
    ms_per_px = abs(x1 - x0) / max(ax.bbox.width, 1.0)
    digits = ((math.floor(math.log10(abs(x_ms))) + 1 if x_ms else 1)
              - math.floor(math.log10(ms_per_px)))
    return ms_to_us(float(f"{x_ms:.{max(1, digits)}g}"))


def build_figure(first, vref, input_scale, on_pick=None):
    """Figure with fixed axes; returns (fig, update) where update(frame, text, pending_us)
    redraws the traces and the GP28 trigger marker in place. A left click inside the plot
    calls on_pick(delay_us) with the clicked time in whole microseconds; on_pick returns
    True if that delay is being sent to the Pico."""
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
    # GP28 trigger: solid line = delay the Pico used for the frame on screen (frame header),
    # dashed line = a delay that was clicked and sent but is not in use yet.
    trig_color = "#2ca02c"
    l_trig = ax.axvline(first.trigger_us / 1000.0, color=trig_color, lw=1.2,
                        label="GP28 trigger (left-click to move)")
    l_pend = ax.axvline(0.0, color=trig_color, lw=1.2, ls="--", visible=False)
    trig_txt = ax.text(0.0, 0.76, "", transform=ax.get_xaxis_transform(), color=trig_color,
                       fontsize=9, va="center")
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

    trig = {"applied": first.trigger_us, "pending": None}   # us

    def draw_trigger():
        applied, pending = trig["applied"], trig["pending"]
        if pending == applied:
            pending = None
        l_trig.set_xdata([applied / 1000.0] * 2)
        l_pend.set_visible(pending is not None)
        x = (applied if pending is None else pending) / 1000.0
        if pending is not None:
            l_pend.set_xdata([x, x])
        x0, x1 = ax.get_xlim()
        trig_txt.set_x(x)
        trig_txt.set_ha("right" if (x - x0) > 0.8 * (x1 - x0) else "left")
        trig_txt.set_text(f" {x:.3f} ms{' (sending)' if pending is not None else ''} ")

    def on_click(event):
        # plain left click inside the plot area; not while the toolbar's pan / zoom is on
        if on_pick is None or event.button != 1 or event.dblclick:
            return
        if getattr(getattr(fig.canvas.manager, "toolbar", None), "mode", ""):
            return
        if not ax.bbox.contains(event.x, event.y):
            return
        x_ms = ax.transData.inverted().transform((event.x, event.y))[0]
        delay_us = click_to_delay_us(ax, x_ms)
        if on_pick(delay_us):
            trig["pending"] = delay_us
            draw_trigger()
            fig.canvas.draw_idle()

    fig.canvas.mpl_connect("button_press_event", on_click)
    draw_trigger()

    def update(f, text="", pending_us=None):
        if len(f.dac) != n:
            txt.set_text("frame length changed - restart this script")
            return
        l_dac.set_ydata(dac_volts(f.dac))
        l_adc.set_ydata(adc_volts(f.adc, vref, input_scale))
        trig["applied"], trig["pending"] = f.trigger_us, pending_us
        draw_trigger()
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


def run_live_plot(src, vref, input_scale, trigger_ms=None):
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation

    first = first_frame(src)
    if trigger_ms is not None:
        request_trigger_delay(src, ms_to_us(trigger_ms), first)
    fig, update = build_figure(first, vref, input_scale,
                               on_pick=lambda us: request_trigger_delay(src, us, first))

    def tick(_):
        f = src.wait_frame(timeout=0)
        if f is not None:
            update(f, status_line(f, src, vref, input_scale), src.trigger_request_us)

    anim = FuncAnimation(fig, tick, interval=100, cache_frame_data=False)  # noqa: F841
    plt.show()


def run_stats(src, vref, input_scale, trigger_ms=None):
    first = first_frame(src)
    if trigger_ms is not None:
        request_trigger_delay(src, ms_to_us(trigger_ms), first)
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
    ap.add_argument("--trigger-ms", type=float, metavar="MS",
                    help="GP28 trigger delay to send at start-up, in ms after the start of each "
                         "sawtooth (fractions allowed, e.g. 87.4). Without it the Pico keeps its "
                         "current delay (50 ms after a reset) until you click on the plot")
    ap.add_argument("--stats", action="store_true", help="print frame stats instead of plotting")
    args = ap.parse_args()

    src = FrameSource(args.port)
    src.start()
    try:
        if args.stats:
            run_stats(src, args.adc_vref, args.input_scale, args.trigger_ms)
        else:
            run_live_plot(src, args.adc_vref, args.input_scale, args.trigger_ms)
    except KeyboardInterrupt:
        pass
    return 0


if __name__ == "__main__":
    sys.exit(main())
