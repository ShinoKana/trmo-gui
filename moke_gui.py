"""
TR-MOKE control GUI  (moke_gui.py)
==================================
A desktop control panel for the TR-MOKE / TR-Faraday rig, modelled on the
previous LabVIEW program (Testbench.vi). It wraps the *already-verified* device drivers in
``auto_mapping.py`` (BBD delay line, SHOT-602 XZ stage, LI5645 lock-in) so you
set parameters, click Start, and watch the scan plot live — no command line.

Two workflows, one window (matching the two LabVIEW VIs):
  * Timescan (TDS) tab  -> Testbench.vi : sweep the BBD delay, read the lock-in,
                           live Amplitude-vs-Time(ps) trace + FFT view.
  * 2D Mapping tab      -> 2D Mapping system.vi : raster the SHOT-602 XZ stage at
                           one delay, live 2D heat-map. (Real signal needs the
                           XZ stage integrated into the beam path; until then use
                           the mock / mock-lockin modes for dry-scan tests.)

Device mode selector (top bar), maps to auto_mapping._make_devices:
  * "Mock (simulated)"          -> everything synthetic, no hardware touched.
  * "Real motion + mock signal" -> real BBD + real stage, synthetic lock-in
                                   (dry-scan the motion pipeline; safe while the
                                   real lock-in is busy on another experiment).
  * "Real (all hardware)"       -> real BBD + real stage + real LI5645 (FETC?).

Safety notes baked in (from the rig's operating rules):
  * The lock-in is READ-ONLY here (FETC? only) so the other experiment sharing
    the LI5645 keeps its configuration — the GUI never writes lock-in settings.
  * The BBD (D2XX) is single-owner: LabVIEW must DISCONNECT the delay-line motor
    before this GUI can open it. Disconnect / closing the window releases it.
  * BBD is returned to its pre-scan position when a timescan ends / is stopped.

Run on the lab PC (where the hardware lives); operate locally or over RDP.
  python moke_gui.py                # launch the GUI
  python moke_gui.py --selftest     # headless: run both scan loops on mock devices
"""

import os
import sys
import json
import time
import copy
import queue
import threading
from datetime import datetime

import numpy as np

import auto_mapping as am   # device drivers + CONFIG + _make_devices + _save_slice

# LI5645 voltage sensitivity (full-scale range) options, 10 nV .. 1 V, 1-2-5 steps.
SENS_RANGES = [
    ("1 V", 1.0), ("500 mV", 0.5), ("200 mV", 0.2), ("100 mV", 0.1), ("50 mV", 0.05),
    ("20 mV", 0.02), ("10 mV", 0.01), ("5 mV", 5e-3), ("2 mV", 2e-3), ("1 mV", 1e-3),
    ("500 uV", 5e-4), ("200 uV", 2e-4), ("100 uV", 1e-4), ("50 uV", 5e-5), ("20 uV", 2e-5),
    ("10 uV", 1e-5), ("5 uV", 5e-6), ("2 uV", 2e-6), ("1 uV", 1e-6),
    ("500 nV", 5e-7), ("200 nV", 2e-7), ("100 nV", 1e-7), ("50 nV", 5e-8), ("20 nV", 2e-8), ("10 nV", 1e-8),
]
SETTINGS_VERSION = 4   # 4 = physical delay again (t0 at overlap, reverse); see _migrate_settings
__version__ = "2.0"    # shown in the title bar; see CHANGELOG.md

# ---- Delay lines on this rig (2026-10-07) -------------------------------------------------
# Read from C:\ProgramData\Thorlabs\MotionControl\ThorlabsDeviceConfiguration.xml and the
# "Delay line ID" control of the previous LabVIEW program. All three are DDS-family stages
# (20000 counts/mm). Each profile keeps its OWN calibration; the active profile is copied into
# config["bbd_*"], so every existing code path keeps working on "the" delay line unchanged.
STAGE_KEYS = ("bbd_serial", "bbd_counts_per_mm", "bbd_is_rack_system", "bbd_channel",
              "bbd_t0_mm", "bbd_double_pass", "bbd_reverse", "bbd_travel_mm")
# Order and IDs = the "Delay line ID" case structure of the previous LabVIEW program.
STAGE_NAMES = {
    "10000000": "ID0 · Stage A (DDS600)",
    "10000001": "ID1 · Stage B (DDS300)",
    "10000002": "ID2 · Stage C (DDS220)",
}
DEFAULT_STAGES = [
    # profile 0 is filled from the current settings on first run (keeps the existing calibration)
    {"name": STAGE_NAMES["10000000"], "bbd_serial": "10000000", "calibrated": True},
    {"name": STAGE_NAMES["10000001"], "bbd_serial": "10000001",
     "bbd_counts_per_mm": 20000, "bbd_is_rack_system": True, "bbd_channel": 1, "bbd_t0_mm": 0.0,
     "bbd_double_pass": True, "bbd_reverse": False, "bbd_travel_mm": 300.0, "calibrated": False},
    {"name": STAGE_NAMES["10000002"], "bbd_serial": "10000002",
     "bbd_counts_per_mm": 20000, "bbd_is_rack_system": True, "bbd_channel": 1, "bbd_t0_mm": 0.0,
     "bbd_double_pass": True, "bbd_reverse": False, "bbd_travel_mm": 220.0, "calibrated": False},
]
SENS_LABELS = [lbl for lbl, _ in SENS_RANGES]
SENS_TO_V = dict(SENS_RANGES)


# ============================================================
# Delay <-> position: ONE source of truth
# ============================================================
# The GUI, the delay-line object and the saved CSV each used to convert um<->ps
# on their own. On 2026-07-27 they disagreed in SIGN — the GUI read bbd_reverse
# (slope -0.1499 mm/ps) while DeviceHub.connect built the real ThorlabsDelay
# without reverse= (slope +0.1499) — so a 549390 -> 0 um scan drove the stage
# UP past 600000 um instead of down. Everything below now goes through these.
def mm_per_ps_for(config):
    """Stage millimetres per picosecond of delay, signed by bbd_reverse.

    reverse=True is the physical case: t0 is the pump-probe overlap and moving
    the stage to a SMALLER position lengthens the delay -> negative slope."""
    v = 0.299792458 / (2.0 if config.get("bbd_double_pass", True) else 1.0)
    return -v if config.get("bbd_reverse", False) else v


def pos_um_to_ps(um, t0_mm, mm_per_ps):
    """An absolute stage position (um) -> delay (ps)."""
    return (um / 1000.0 - t0_mm) / mm_per_ps


def step_um_to_ps(um, mm_per_ps):
    """A step SIZE (um) -> ps. A delta, so no t0 offset."""
    return (um / 1000.0) / mm_per_ps


def ps_to_pos_um(ps, t0_mm, mm_per_ps):
    """A delay (ps) -> the absolute stage position (um) it parks at."""
    return (t0_mm + ps * mm_per_ps) * 1000.0


def plan_timescan(config, unit, start, end, step):
    """The Scan boxes -> (t0_mm, start_ps, end_ps, step_ps) the loop will run.

    unit "ps": the boxes are already delays; t0 is left alone.
    Anything else ("um"/"µm"): the boxes are ABSOLUTE stage positions in um and
    delay 0 is re-anchored to THIS scan's start ("0 ps = where the
    timescan started"), so the trace always reads 0 ps at the first point.

    step_ps comes back unsigned — the sweep direction is start->end, which
    timescan_loop derives on its own. That is why typing Step -1000 changes
    nothing; it is not the knob that reverses a scan.
    """
    t0_mm = float(config.get("bbd_t0_mm", 0.0))
    if unit == "ps":
        return t0_mm, float(start), float(end), abs(float(step))
    t0_mm = float(start) / 1000.0
    mmps = mm_per_ps_for(config)
    sp = pos_um_to_ps(float(start), t0_mm, mmps)
    ep = pos_um_to_ps(float(end), t0_mm, mmps)
    return (t0_mm,
            sp or 0.0,          # the start is exactly t0; keep it 0, not "-0"
            ep or 0.0,
            abs(step_um_to_ps(float(step), mmps)))


def timescan_out_of_travel(config, start_ps, end_ps):
    """Would this sweep drive the stage off its rails? -> (lo_um, hi_um) or None.

    The delay line only reaches 0 .. bbd_travel_mm; a delay outside that is not
    a small error, it is the carriage running into the hard stop."""
    mmps = mm_per_ps_for(config)
    t0_mm = float(config.get("bbd_t0_mm", 0.0))
    a = ps_to_pos_um(float(start_ps), t0_mm, mmps)
    b = ps_to_pos_um(float(end_ps), t0_mm, mmps)
    lo, hi = min(a, b), max(a, b)
    travel_um = float(config.get("bbd_travel_mm", 570.0)) * 1000.0
    return (lo, hi) if (lo < -1.0 or hi > travel_um + 1.0) else None

def _nearest_sens_label(vrms):
    """Closest sensitivity-range label for a measured Vrms (log distance)."""
    try:
        v = float(vrms)
        return min(SENS_RANGES, key=lambda kv: abs(np.log10(kv[1]) - np.log10(v)))[0]
    except (ValueError, TypeError):
        return "10 mV"


# ============================================================
# Signal keys the lock-in returns
# ============================================================
SIGNALS = ("R", "X", "Y", "theta")


def _avg_reading(lockin, num_averages, settle_between=0.05, stop_event=None):
    """Average num_averages lock-in read_all() calls into one {X,Y,R,theta} dict.

    Read-only: read_all() uses FETC? on the LI5645 (never writes config).
    Returns the averaged dict, or None if stopped mid-average.
    """
    acc = {k: [] for k in SIGNALS}
    n = max(1, int(num_averages))
    for j in range(n):
        if stop_event is not None and stop_event.is_set():
            return None
        r = lockin.read_all()
        for k in SIGNALS:
            acc[k].append(float(r[k]))
        if j < n - 1:
            time.sleep(settle_between)
    return {k: float(np.mean(acc[k])) for k in SIGNALS}


# ============================================================
# Device hub — connect / hold handles / jog / read (Tk-free)
# ============================================================
class DeviceHub:
    """Opens and holds the instrument handles so the GUI can jog the stage and
    read live values between scans, then reuse the same handles for a scan.

    mode: "mock" | "mock_lockin" | "real"
    need_stage: also open the SHOT-602 XZ controller.
    """

    def __init__(self):
        self.bbd = None
        self.lockin = None
        self.stage = None
        self.pump = None            # pump delay: its own SHOT-602 (2026-10-07)
        self.rm = None
        self.mode = None
        self.has_stage = False
        self.has_pump = False
        # pylablib's APT/D2XX stream is not thread-safe. Keep every BBD command
        # and readback on one serialized path so a queued live-monitor refresh
        # cannot overlap a scan move.
        self.bbd_lock = threading.RLock()
        self.bbd_faulted = False

    def connect(self, config, mode, need_stage, need_pump=False):
        """Open devices and return a per-device status dict {bbd,lockin,stage}.
        In real mode each device is opened independently so one failure is
        reported (not fatal) — the others still connect. Never raises for a
        device failure; the status string carries the error."""
        self.close()
        self.mode = mode
        self.bbd_faulted = False
        cfg = copy.deepcopy(config)
        status = {"bbd": "n/a", "lockin": "n/a", "stage": "n/a", "pump": "n/a"}

        if mode in ("mock", "mock_lockin"):
            cfg["mock"] = (mode == "mock")
            cfg["mock_lockin"] = (mode == "mock_lockin")
            self.bbd, self.lockin, self.stage, self.rm = am._make_devices(cfg, need_stage=need_stage)
            self.has_stage = need_stage and self.stage is not None
            status["bbd"] = status["lockin"] = "ok"
            status["stage"] = "ok" if self.has_stage else "n/a"
            if need_pump:
                try:
                    if mode == "mock":
                        self.pump = am.MockSHOT602()
                        self.pump.ppmm = 500     # the mock counts 500 pulses/mm internally
                    else:            # real motion + mock signal: the pump controller is real
                        self.pump = am.SHOT602(cfg["pump_port"], cfg["pump_baud"], cfg["dl2_pulses_per_mm"])
                    self.has_pump = True
                    status["pump"] = "ok"
                except Exception as e:
                    status["pump"] = f"FAILED: {type(e).__name__}: {e}"
            return status

        # real: each device independently
        try:
            self.bbd = am.ThorlabsDelay(
                serial_number=cfg["bbd_serial"], counts_per_mm=cfg["bbd_counts_per_mm"],
                is_rack_system=cfg["bbd_is_rack_system"], channel=cfg["bbd_channel"],
                t0_mm=cfg["bbd_t0_mm"], double_pass=cfg["bbd_double_pass"],
                reverse=cfg.get("bbd_reverse", False),   # MUST match mm_per_ps_for(config):
                port=cfg["bbd_port"])                    # without it the rig ran reverse=False
            self.sync_bbd_calibration(cfg)               # and every scan swept backwards
            status["bbd"] = "ok"
        except Exception as e:
            status["bbd"] = f"FAILED: {type(e).__name__}: {e}"
        try:
            self.rm = am._pyvisa().ResourceManager()
            self.lockin = am.LockInAmp(self.rm, cfg["lockin_addr"], cfg["lockin_model"])
            status["lockin"] = "ok"
        except Exception as e:
            status["lockin"] = f"FAILED: {type(e).__name__}: {e}"
        if need_stage:
            try:
                self.stage = am.SHOT602(cfg["shot602_port"], cfg["shot602_baud"], cfg["pulses_per_mm"])
                self.has_stage = True
                status["stage"] = "ok"
            except Exception as e:
                status["stage"] = f"FAILED: {type(e).__name__}: {e}"
        if need_pump:
            try:
                self.pump = am.SHOT602(cfg["pump_port"], cfg["pump_baud"], cfg["dl2_pulses_per_mm"])
                self.has_pump = True
                status["pump"] = "ok"
            except Exception as e:
                status["pump"] = f"FAILED: {type(e).__name__}: {e}"
        return status

    def reconnect_bbd(self, config):
        """Swap ONLY the main delay line (2026-10-07); lock-in / XZ / pump stay open.
        Returns 'ok' or 'FAILED: …' — never raises for a device failure."""
        cfg = copy.deepcopy(config)
        with self.bbd_lock:
            try:
                if self.bbd is not None:
                    self.bbd.close()
            except Exception:
                pass
            self.bbd = None
            self.bbd_faulted = False
            try:
                if self.mode == "mock":
                    self.bbd = am.MockBBD(t0_mm=cfg["bbd_t0_mm"], double_pass=cfg["bbd_double_pass"],
                                          reverse=cfg.get("bbd_reverse", False))
                else:
                    self.bbd = am.ThorlabsDelay(
                        serial_number=cfg["bbd_serial"], counts_per_mm=cfg["bbd_counts_per_mm"],
                        is_rack_system=cfg["bbd_is_rack_system"], channel=cfg["bbd_channel"],
                        t0_mm=cfg["bbd_t0_mm"], double_pass=cfg["bbd_double_pass"],
                        reverse=cfg.get("bbd_reverse", False), port=cfg["bbd_port"])
                    self.sync_bbd_calibration(cfg)
                if self.lockin is not None and hasattr(self.lockin, "_bbd"):
                    self.lockin._bbd = self.bbd          # mock signal follows the new delay line
                return "ok"
            except Exception as e:
                self.bbd = None
                return f"FAILED: {type(e).__name__}: {e}"

    def sync_bbd_calibration(self, config):
        """Push t0 / double-pass / reverse onto the LIVE delay line.

        The delay-line object keeps its own copy of the conversion, fixed when
        it was opened. Anything that changes the calibration afterwards (Apply
        to config, a timescan re-anchoring t0 to its start) must land here too,
        or the GUI and the motor disagree and the scan runs backwards."""
        if self.bbd is None:
            return
        with self.bbd_lock:
            self.bbd.t0_mm = float(config.get("bbd_t0_mm", 0.0))
            self.bbd.mm_per_ps = mm_per_ps_for(config)

    # ---- live readback -------------------------------------------------
    def read_position_mm(self):
        with self.bbd_lock:
            if self.bbd_faulted:
                raise RuntimeError("BBD state is unknown; reconnect before reading it.")
            return float(self.bbd.get_position_mm())

    def read_position_ps(self):
        with self.bbd_lock:
            if self.bbd_faulted:
                raise RuntimeError("BBD state is unknown; reconnect before reading it.")
            return float(self.bbd.get_position_ps())

    def _move_bbd(self, fn):
        """Run one BBD move exclusively; quarantine the handle after any failure.

        A wait timeout only stops waiting in pylablib; it does not stop the motor.
        Make a best-effort controlled stop, then require reconnect before another
        command so a second scan cannot collide with an unfinished move.
        """
        with self.bbd_lock:
            if self.bbd_faulted:
                raise RuntimeError("BBD state is unknown; Disconnect, wait for motion to stop, then Connect again.")
            try:
                return fn()
            except Exception:
                self.bbd_faulted = True
                try:
                    if hasattr(self.bbd, "is_moving") and self.bbd.is_moving():
                        self.bbd.stop(immediate=False, timeout=30.0)
                except Exception:
                    pass
                raise

    def move_bbd_to_mm(self, mm, timeout=300.0):
        return self._move_bbd(lambda: self.bbd.move_to_mm(float(mm), timeout=timeout))

    def move_bbd_to_ps(self, ps, timeout=300.0):
        return self._move_bbd(lambda: self.bbd.move_to_ps(float(ps), timeout=timeout))

    def home_bbd(self, timeout=120.0):
        return self._move_bbd(lambda: self.bbd.home(timeout=timeout))

    def read_lockin(self):
        """Read-only lock-in snapshot (FETC? on LI5645). {X,Y,R,theta} or None."""
        try:
            return self.lockin.read_all()
        except Exception:
            return None

    def stage_position_mm(self, pulses_per_mm):
        if not self.has_stage:
            return None
        xp, zp = self.stage.get_position()
        return xp / pulses_per_mm, zp / pulses_per_mm

    # ---- close ---------------------------------------------------------
    def close(self):
        for obj in (self.stage, self.lockin, self.pump):
            try:
                if obj is not None:
                    obj.close()
            except Exception:
                pass
        with self.bbd_lock:
            try:
                if self.bbd is not None:
                    self.bbd.close()
            except Exception:
                pass
        try:
            if self.rm is not None:
                self.rm.close()
        except Exception:
            pass
        self.bbd = self.lockin = self.stage = self.pump = self.rm = None
        self.mode = None
        self.has_stage = False
        self.has_pump = False
        self.bbd_faulted = False

    @property
    def connected(self):
        # True if ANY device opened — a partial (real) connect still counts so the
        # per-device panel shows and Disconnect can release what did open.
        return (self.bbd is not None) or (self.lockin is not None) or (self.stage is not None) or (self.pump is not None)


# ============================================================
# Scan loops (Tk-free; push dict messages to out_q, honour stop_event)
# ============================================================
def timescan_loop(config, hub, out_q, stop_event, repetition=1, after_scan="prescan",
                  pause_event=None, trace=None):
    """Sweep BBD delay over [time_start_ps, time_end_ps] step time_step_ps.

    Averages `repetition` full sweeps. Pushes:
      {"kind":"begin_timescan", "t_ps":[...], "signal":sig}
      {"kind":"point_timescan", "i":i, "t_ps":t, "rep":r, "reps":R, X,Y,R,theta}
      {"kind":"status", "text":...}
      {"kind":"done", "scan":"timescan", "dir":path}
      {"kind":"error", "text":...}
    Returns (t_ps, data[n,4]) or (None, None) on error.
    """
    try:
        # sign the step by scan direction so a decreasing scan (e.g. 570 mm -> 0,
        # i.e. high ps -> 0) works too, not just increasing ps.
        _step = abs(config["time_step_ps"]) or 1.0
        if config["time_end_ps"] < config["time_start_ps"]:
            _step = -_step
        t_ps = np.arange(
            config["time_start_ps"],
            config["time_end_ps"] + _step / 2.0,
            _step,
        )
        dl2 = _dl2_plan(config, t_ps)        # None unless delay line 2 is on
        if dl2 is not None:
            t_ps = dl2["bbd_ps"]               # BBD positions (constant when delay line 1 is fixed)
        x_ps = dl2["x_ps"] if dl2 is not None else t_ps   # plot / save axis
        n = len(t_ps)
        if n == 0:
            out_q.put({"kind": "error", "text": "Timescan range is empty."})
            return None, None
        dl2_home = None
        if dl2 is not None:
            if getattr(hub, "pump", None) is None:
                out_q.put({"kind": "error", "text": "The pump delay is not connected (tick 'Pump' and Connect)."})
                return None, None
            dl2_home = _dl2_read_mm_equiv(hub, config)   # restore target
            out_q.put({"kind": "status", "text": f"Pump delay (axis {config['dl2_axis']}): "
                       f"{dl2['um'][0]:,.0f} → {dl2['um'][-1]:,.0f} µm over {n} points"})

        out_q.put({"kind": "begin_timescan", "t_ps": x_ps.tolist(),
                   "signal": config.get("signal", "X")})

        start_pos_mm = hub.read_position_mm()   # for restore
        acc = np.zeros((n, 4))
        counts = np.zeros(n)
        reps = max(1, int(repetition))

        for rep in range(reps):
            for i, ps in enumerate(t_ps):
                if not _pause_gate(pause_event, stop_event) or stop_event.is_set():
                    raise KeyboardInterrupt
                hub.move_bbd_to_ps(float(ps))
                time.sleep(config["bbd_settle_s"])
                if dl2 is not None and (i == 0 or dl2["um"][i] != dl2["um"][i - 1]):
                    _dl2_move_um(hub, config, dl2["um"][i])
                    time.sleep(config.get("dl2_settle_s", 0.2))
                avg = _avg_reading(hub.lockin, config["num_averages"],
                                   stop_event=stop_event)
                if avg is None:
                    raise KeyboardInterrupt
                acc[i] += [avg["X"], avg["Y"], avg["R"], avg["theta"]]
                counts[i] += 1
                data = acc / np.maximum(counts, 1)[:, None]
                try:
                    pos_mm = hub.read_position_mm()   # ACTUAL position — confirms the BBD moved
                    pos_ps = hub.bbd.mm_to_ps(pos_mm)
                except Exception:
                    pos_mm = pos_ps = None
                out_q.put({"kind": "point_timescan", "i": int(i),
                           "t_ps": float(x_ps[i]), "rep": rep + 1, "reps": reps,
                           "X": data[i, 0], "Y": data[i, 1],
                           "R": data[i, 2], "theta": data[i, 3],
                           "pos_mm": pos_mm, "pos_ps": pos_ps})
        data = acc / np.maximum(counts, 1)[:, None]
        outdir = _save_timescan(config, x_ps, data, **_dl2_save_kw(dl2, t_ps))
        _dl2_restore(hub, dl2_home, out_q)
        _after_scan_move(hub, config, after_scan, start_pos_mm, t_ps, data,
                         out_q=out_q, trace=trace)
        out_q.put({"kind": "done", "scan": "timescan", "dir": outdir, "npts": int(n)})
        return t_ps, data

    except KeyboardInterrupt:
        try:
            data = acc / np.maximum(counts, 1)[:, None]
            outdir = _save_timescan(config, x_ps, data, partial=True, **_dl2_save_kw(dl2, t_ps))
            _dl2_restore(hub, dl2_home, out_q)
            _after_scan_move(hub, config, after_scan, start_pos_mm, t_ps, data,
                             out_q=out_q, trace=trace)
            out_q.put({"kind": "done", "scan": "timescan", "dir": outdir,
                       "stopped": True, "npts": int(n)})
        except Exception as e:
            if trace is not None:
                trace(f"stopped timescan cleanup ERROR {type(e).__name__}: {e}")
            out_q.put({"kind": "error", "text": f"{type(e).__name__}: {e}"})
        return None, None
    except Exception as e:
        if trace is not None:
            trace(f"timescan ERROR {type(e).__name__}: {e}")
        out_q.put({"kind": "error", "text": f"{type(e).__name__}: {e}"})
        return None, None


def _dl2_plan(config, t_ps):
    """Delay line 2 positions for one timescan, or None when it is off.
    Same number of points as delay line 1; if delay line 1 is fixed (one point) the
    point count comes from delay line 2's own step. x_ps = what the plot/CSV use:
    delay line 1's ps, or delay line 2's delay from its start when only it moves."""
    if not config.get("dl2_enabled"):
        return None
    s2, e2 = float(config["dl2_start_um"]), float(config["dl2_end_um"])
    t_ps = np.asarray(t_ps, dtype=float)
    bbd_fixed = len(t_ps) == 1
    if bbd_fixed:
        step2 = abs(float(config.get("dl2_step_um", 0.0))) or 1.0
        n = int(round(abs(e2 - s2) / step2)) + 1
        t_ps = np.full(n, t_ps[0])
    um = np.linspace(s2, e2, len(t_ps))
    mm_per_ps2 = 0.299792458 / 2.0          # delay line 2 assumed double-pass for the ps axis
    x_ps = (np.abs(um - s2) / 1000.0) / mm_per_ps2 if bbd_fixed else t_ps
    return {"um": um, "bbd_ps": t_ps, "x_ps": x_ps, "bbd_fixed": bbd_fixed}


def _dl2_ppmm_ratio(hub, config):
    stage_ppmm = float(getattr(hub.pump, "ppmm", config.get("dl2_pulses_per_mm", 500)) or 500)
    return float(config["dl2_pulses_per_mm"]) / stage_ppmm


def _dl2_move_um(hub, config, um):
    # the SHOT driver converts mm with ITS pulses/mm (the XZ setting); scale so the
    # pulses sent are um/1000 * dl2_pulses_per_mm for this axis
    hub.pump.move_abs(int(config["dl2_axis"]), float(um) / 1000.0 * _dl2_ppmm_ratio(hub, config))


def _list_delay_lines():
    """Serials of Thorlabs controllers on USB (read-only D2XX enumeration), or None if unavailable.
    A controller already OPEN (by us, LabVIEW, …) may come back without its serial."""
    try:
        from pylablib.devices import Thorlabs
        return sorted(str(sn) for sn, _ in Thorlabs.list_kinesis_devices() if str(sn).strip())
    except Exception:
        return None


def _dl2_read_um(hub, config):
    """Pump delay position in um (its own pulses/mm, not the XZ stage's)."""
    x, z = hub.pump.get_position()
    pulses = x if int(config["dl2_axis"]) == 1 else z
    return float(pulses) / float(config["dl2_pulses_per_mm"]) * 1000.0


def _dl2_read_mm_equiv(hub, config):
    try:
        x, z = hub.pump.get_position()
        stage_ppmm = float(getattr(hub.pump, "ppmm", config.get("dl2_pulses_per_mm", 500)) or 500)
        return (int(config["dl2_axis"]), (x if int(config["dl2_axis"]) == 1 else z) / stage_ppmm)
    except Exception:
        return None


def _dl2_restore(hub, home, out_q):
    if home is None:
        return
    try:
        hub.pump.move_abs(home[0], home[1])
    except Exception as e:
        out_q.put({"kind": "status", "text": f"Pump delay: return to start position failed: {e}"})


def _dl2_save_kw(dl2, bbd_ps):
    return {} if dl2 is None else {"dl2_um": dl2["um"], "bbd_ps": bbd_ps}


def _pause_gate(pause_event, stop_event):
    """Block while paused; return False if the scan was stopped during the pause."""
    while pause_event is not None and pause_event.is_set():
        if stop_event.is_set():
            return False
        time.sleep(0.1)
    return True


def mapping_loop(config, hub, out_q, stop_event, pause_event=None):
    """Raster the SHOT-602 XZ stage at one BBD delay, read the lock-in per point.

    Snake pattern (alternate Z direction). Pushes:
      {"kind":"begin_mapping", "x_mm":[...], "z_mm":[...], "signal":sig, "delay_ps":d}
      {"kind":"point_mapping", "i_x":,"i_z":,"x_mm":,"z_mm":,"count":,"total":, X,Y,R,theta}
      {"kind":"done", "scan":"mapping", "dir":path}
      {"kind":"error", "text":...}
    """
    try:
        x_pos = np.arange(config["x_start"], config["x_end"] + config["x_step"] / 2.0, config["x_step"])
        z_pos = np.arange(config["z_start"], config["z_end"] + config["z_step"] / 2.0, config["z_step"])
        n_x, n_z = len(x_pos), len(z_pos)
        total = n_x * n_z
        if total == 0:
            out_q.put({"kind": "error", "text": "Mapping range is empty."})
            return None
        delay_ps = config["delay_label_ps"]

        out_q.put({"kind": "begin_mapping", "x_mm": x_pos.tolist(),
                   "z_mm": z_pos.tolist(), "signal": config.get("signal", "R"),
                   "delay_ps": float(delay_ps)})

        # park the delay line
        hub.move_bbd_to_ps(float(delay_ps))
        time.sleep(config["bbd_settle_s"])
        out_q.put({"kind": "status",
                   "text": f"BBD parked at {hub.read_position_mm():.4f} mm "
                           f"({hub.read_position_ps():.2f} ps)"})

        hub.stage.set_speed(config["x_axis"], 500, config["stage_speed_fast"], 200)
        hub.stage.set_speed(config["z_axis"], 500, config["stage_speed_fast"], 200)

        data = np.zeros((n_x, n_z, 4))
        count = 0
        for i_x, x_mm in enumerate(x_pos):
            z_iter = list(enumerate(z_pos)) if i_x % 2 == 0 else list(reversed(list(enumerate(z_pos))))
            for i_z, z_mm in z_iter:
                if not _pause_gate(pause_event, stop_event) or stop_event.is_set():
                    raise KeyboardInterrupt
                hub.stage.move_abs_both(float(x_mm), float(z_mm), config["x_axis"], config["z_axis"])
                time.sleep(config["settling_time"])
                avg = _avg_reading(hub.lockin, config["num_averages"], stop_event=stop_event)
                if avg is None:
                    raise KeyboardInterrupt
                data[i_x, i_z] = [avg["X"], avg["Y"], avg["R"], avg["theta"]]
                count += 1
                out_q.put({"kind": "point_mapping", "i_x": int(i_x), "i_z": int(i_z),
                           "x_mm": float(x_mm), "z_mm": float(z_mm),
                           "count": count, "total": total,
                           "X": avg["X"], "Y": avg["Y"], "R": avg["R"], "theta": avg["theta"]})
        # after-scan: return the stage to the scan start (or leave it where it ended)
        if config.get("map_after_scan", "start") == "start":
            try:
                hub.stage.move_abs_both(float(x_pos[0]), float(z_pos[0]), config["x_axis"], config["z_axis"])
            except Exception:
                pass
        outdir = _save_mapping(config, x_pos, z_pos, data, delay_ps)
        out_q.put({"kind": "done", "scan": "mapping", "dir": outdir, "shape": [int(n_x), int(n_z)]})
        return data

    except KeyboardInterrupt:
        try:
            outdir = _save_mapping(config, x_pos, z_pos, data, delay_ps, partial=True)
            out_q.put({"kind": "done", "scan": "mapping", "dir": outdir,
                       "stopped": True, "shape": [int(n_x), int(n_z)]})
        except Exception:
            out_q.put({"kind": "done", "scan": "mapping", "dir": None, "stopped": True})
        return None
    except Exception as e:
        out_q.put({"kind": "error", "text": f"{type(e).__name__}: {e}"})
        return None


def mapping3d_loop(config, hub, out_q, stop_event, delays, pause_event=None):
    """3D: for each delay in `delays` (ps), park the BBD and raster a full XZ map.
    Builds a (n_delay, n_x, n_z, 4) stack. Pushes begin_map3d / point_map3d / done."""
    try:
        x_pos = np.arange(config["x_start"], config["x_end"] + config["x_step"] / 2.0, config["x_step"])
        z_pos = np.arange(config["z_start"], config["z_end"] + config["z_step"] / 2.0, config["z_step"])
        n_x, n_z = len(x_pos), len(z_pos)
        if n_x * n_z == 0 or len(delays) == 0:
            out_q.put({"kind": "error", "text": "3D range or delay list is empty."})
            return None
        out_q.put({"kind": "begin_map3d", "x_mm": x_pos.tolist(), "z_mm": z_pos.tolist(),
                   "delays": [float(d) for d in delays], "signal": config.get("signal", "R")})
        hub.stage.set_speed(config["x_axis"], 500, config["stage_speed_fast"], 200)
        hub.stage.set_speed(config["z_axis"], 500, config["stage_speed_fast"], 200)
        stack = np.full((len(delays), n_x, n_z, 4), np.nan)
        total = len(delays) * n_x * n_z
        count = 0
        for di, d in enumerate(delays):
            hub.move_bbd_to_ps(float(d))
            time.sleep(config["bbd_settle_s"])
            out_q.put({"kind": "status", "text": f"3D: delay {d:g} ps  ({di+1}/{len(delays)})"})
            for i_x, x_mm in enumerate(x_pos):
                z_iter = list(enumerate(z_pos)) if i_x % 2 == 0 else list(reversed(list(enumerate(z_pos))))
                for i_z, z_mm in z_iter:
                    if not _pause_gate(pause_event, stop_event) or stop_event.is_set():
                        raise KeyboardInterrupt
                    hub.stage.move_abs_both(float(x_mm), float(z_mm), config["x_axis"], config["z_axis"])
                    time.sleep(config["settling_time"])
                    avg = _avg_reading(hub.lockin, config["num_averages"], stop_event=stop_event)
                    if avg is None:
                        raise KeyboardInterrupt
                    stack[di, i_x, i_z] = [avg["X"], avg["Y"], avg["R"], avg["theta"]]
                    count += 1
                    out_q.put({"kind": "point_map3d", "di": int(di), "i_x": int(i_x), "i_z": int(i_z),
                               "x_mm": float(x_mm), "z_mm": float(z_mm), "delay": float(d),
                               "count": count, "total": total,
                               "X": avg["X"], "Y": avg["Y"], "R": avg["R"], "theta": avg["theta"]})
        if config.get("map_after_scan", "start") == "start":
            try:
                hub.stage.move_abs_both(float(x_pos[0]), float(z_pos[0]), config["x_axis"], config["z_axis"])
            except Exception:
                pass
        outdir = _save_map3d(config, x_pos, z_pos, stack, delays)
        out_q.put({"kind": "done", "scan": "map3d", "dir": outdir, "shape": [len(delays), n_x, n_z]})
        return stack
    except KeyboardInterrupt:
        try:
            outdir = _save_map3d(config, x_pos, z_pos, stack, delays, partial=True)
            out_q.put({"kind": "done", "scan": "map3d", "dir": outdir,
                       "stopped": True, "shape": [len(delays), n_x, n_z]})
        except Exception:
            out_q.put({"kind": "done", "scan": "map3d", "dir": None, "stopped": True})
        return None
    except Exception as e:
        out_q.put({"kind": "error", "text": f"{type(e).__name__}: {e}"})
        return None


def _restore_bbd(hub, config, pos_mm):
    """Return the delay line to where it was before the scan (best-effort)."""
    try:
        hub.move_bbd_to_mm(float(pos_mm))
    except Exception:
        pass


def _after_scan_move(hub, config, after_scan, prescan_mm, t_ps, data=None,
                     out_q=None, trace=None):
    """Where the delay line parks after a timescan (mirrors LabVIEW 'After scan
    move to'). 'stay' leaves it at the last delay point; **'max'/'min' go to the
    delay where the SIGNAL is largest/smallest (the peak/trough)** — not the scan
    end/start; 'prescan' restores the position from before the scan."""
    if after_scan == "stay":
        if trace is not None:
            trace("after-scan move: Stay (no move)")
        return

    target_ps = None
    target_mm = None
    label = after_scan
    if after_scan in ("max", "min") and data is not None:
        col = {"X": 0, "Y": 1, "R": 2, "theta": 3}.get(config.get("signal", "X"), 0)
        sig = np.asarray(data)[:, col]
        if np.isfinite(sig).any():
            idx = int(np.nanargmax(sig) if after_scan == "max" else np.nanargmin(sig))
            target_ps = float(t_ps[idx])
            label = f"{after_scan} signal"
    if target_ps is None and after_scan == "max":
        target_ps = float(t_ps[-1])
    elif target_ps is None and after_scan == "min":
        target_ps = float(t_ps[0])
    elif target_ps is None:
        target_mm = float(prescan_mm)
        label = "pre-scan position"

    target = (f"{target_mm * 1000:.0f} um" if target_mm is not None
              else f"{target_ps:.1f} ps")
    msg = f"Scan data saved; moving delay line to {label} ({target}). Please wait."
    if trace is not None:
        trace(f"after-scan move START: {label} target={target}")
    if out_q is not None:
        out_q.put({"kind": "status", "text": msg})

    # A full-stroke return can legitimately exceed the old 60 s default. Wait up
    # to five minutes. DeviceHub stops/quarantines the BBD if this still fails.
    if target_mm is not None:
        hub.move_bbd_to_mm(target_mm, timeout=300.0)
    else:
        hub.move_bbd_to_ps(target_ps, timeout=300.0)

    if trace is not None:
        trace(f"after-scan move DONE: {label} target={target}")
    if out_q is not None:
        out_q.put({"kind": "status", "text": f"Delay line reached {label} ({target})."})


# ============================================================
# Saving (npz + dat, format compatible with auto_mapping.plot_mapping)
# ============================================================
def _data_root(config):
    root = config.get("data_dir") or os.path.join(os.path.dirname(os.path.abspath(__file__)), "moke_data")
    os.makedirs(root, exist_ok=True)
    return root


def _save_timescan(config, t_ps, data, partial=False, dl2_um=None, bbd_ps=None):
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    name = f"timescan_{ts}" + ("_partial" if partial else "")
    outdir = os.path.join(_data_root(config), name)
    os.makedirs(outdir, exist_ok=True)
    extra = {} if dl2_um is None else {"dl2_um": np.asarray(dl2_um), "bbd_ps": np.asarray(bbd_ps)}
    np.savez(os.path.join(outdir, "timescan.npz"),
             t_ps=t_ps, data=data,
             t0_mm=config["bbd_t0_mm"], double_pass=config["bbd_double_pass"],
             config=str(config), **extra)
    with open(os.path.join(outdir, "timescan.dat"), "w") as f:
        f.write("t(ps)\tX(V)\tY(V)\tR(V)\ttheta(deg)\n")
        for i, ps in enumerate(t_ps):
            f.write(f"{ps:.4f}\t{data[i,0]:.6e}\t{data[i,1]:.6e}\t{data[i,2]:.6e}\t{data[i,3]:.4f}\n")
    # convenient CSV (comma, opens in Excel/pandas). Includes stage position (mm).
    # Signed by bbd_reverse — an unsigned slope here printed positions the stage
    # never visited (mirror-imaged about t0) whenever reverse was on.
    mm_per_ps = mm_per_ps_for(config)
    t0 = float(config.get("bbd_t0_mm", 0.0))
    with open(os.path.join(outdir, "timescan.csv"), "w", newline="") as f:
        f.write(f"# TR-MOKE timescan  |  {len(t_ps)} points (1D)  |  delay line: {config.get('stage_name', '?')}"
                f"  serial {config.get('bbd_serial', '?')}  t0 {config.get('bbd_t0_mm', 0.0)} mm"
                f"  double_pass {config.get('bbd_double_pass')}  reverse {config.get('bbd_reverse')}\n")
        f.write("t_ps,position_mm,position_um,X_V,Y_V,R_V,theta_deg" + (",dl2_um" if dl2_um is not None else "") + "\n")
        for i, ps in enumerate(t_ps):
            mm = t0 + (bbd_ps[i] if bbd_ps is not None else ps) * mm_per_ps   # delay line 1's real position
            f.write(f"{ps:.4f},{mm:.5f},{mm*1000:.2f},"
                    f"{data[i,0]:.6e},{data[i,1]:.6e},{data[i,2]:.6e},{data[i,3]:.4f}"
                    + (f",{dl2_um[i]:.2f}" if dl2_um is not None else "") + "\n")
    return outdir


def _save_mapping(config, x_pos, z_pos, data, delay_ps, partial=False):
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    name = f"mapping_{ts}_t{delay_ps}ps" + ("_partial" if partial else "")
    outdir = os.path.join(_data_root(config), name)
    os.makedirs(outdir, exist_ok=True)
    np.savez(os.path.join(outdir, "mapping_full.npz"),
             data=data, x_positions=x_pos, z_positions=z_pos,
             delay_label_ps=delay_ps, config=str(config))
    am._save_slice(outdir, delay_ps, x_pos, z_pos, data)

    nx, nz = data.shape[0], data.shape[1]
    # (a) long-format CSV — one row per pixel (all channels)
    with open(os.path.join(outdir, "mapping_long.csv"), "w", newline="") as f:
        f.write(f"# XZ map at delay {delay_ps} ps  |  {nx} x {nz} = {nx*nz} points\n")
        f.write("X_mm,Z_mm,X_V,Y_V,R_V,theta_deg\n")
        for i_x, x in enumerate(x_pos):
            for i_z, z in enumerate(z_pos):
                v = data[i_x, i_z]
                f.write(f"{x:.4f},{z:.4f},{v[0]:.6e},{v[1]:.6e},{v[2]:.6e},{v[3]:.4f}\n")
    # (b) 2D-grid CSV for R and X — rows=X(mm), cols=Z(mm); imports straight as an image
    for chan_name, col in (("R", 2), ("X", 0)):
        with open(os.path.join(outdir, f"mapping_{chan_name}_2D.csv"), "w", newline="") as f:
            f.write(f"# {chan_name} (V) at delay {delay_ps} ps  |  rows=X(mm), cols=Z(mm)  |  shape {nx}x{nz}\n")
            f.write("X\\Z," + ",".join(f"{z:.4f}" for z in z_pos) + "\n")
            for i_x, x in enumerate(x_pos):
                f.write(f"{x:.4f}," + ",".join(f"{data[i_x, i_z, col]:.6e}" for i_z in range(nz)) + "\n")
    return outdir


def _save_map3d(config, x_pos, z_pos, stack, delays, partial=False):
    """Save the 3D map stack: npz + one R 2D-grid CSV per delay."""
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    name = f"map3d_{ts}" + ("_partial" if partial else "")
    outdir = os.path.join(_data_root(config), name)
    os.makedirs(outdir, exist_ok=True)
    np.savez(os.path.join(outdir, "map3d_full.npz"), data=stack, x_positions=x_pos,
             z_positions=z_pos, delays_ps=np.array(delays, float), config=str(config))
    nz = len(z_pos)
    for di, d in enumerate(delays):
        with open(os.path.join(outdir, f"map3d_R_delay{d:g}ps.csv"), "w", newline="") as f:
            f.write(f"# R (V) at delay {d} ps  |  rows=X(mm), cols=Z(mm)\n")
            f.write("X\\Z," + ",".join(f"{z:.4f}" for z in z_pos) + "\n")
            for i_x, x in enumerate(x_pos):
                f.write(f"{x:.4f}," + ",".join(f"{stack[di, i_x, i_z, 2]:.6e}" for i_z in range(nz)) + "\n")
    return outdir


# ============================================================
# Headless self-test (no Tk): exercise both loops on mock devices
# ============================================================
def _selftest():
    print("=== moke_gui self-test (mock devices) ===")
    cfg = copy.deepcopy(am.CONFIG)
    cfg["data_dir"] = os.path.join(
        os.environ.get("TMPDIR", "/tmp"), "moke_selftest")
    cfg["time_start_ps"], cfg["time_end_ps"], cfg["time_step_ps"] = 0.0, 10.0, 2.0
    cfg["x_start"], cfg["x_end"], cfg["x_step"] = 6.0, 8.0, 1.0
    cfg["z_start"], cfg["z_end"], cfg["z_step"] = 3.0, 5.0, 1.0
    cfg["num_averages"] = 2
    cfg["signal"] = "X"

    hub = DeviceHub()
    stop = threading.Event()

    # timescan
    hub.connect(cfg, mode="mock", need_stage=False)
    q = queue.Queue()
    t_ps, data = timescan_loop(cfg, hub, q, stop, repetition=2)
    msgs = _drain(q)
    assert t_ps is not None and data.shape[0] == len(t_ps), "timescan produced no data"
    assert any(m["kind"] == "point_timescan" for m in msgs), "no timescan points"
    assert any(m["kind"] == "done" for m in msgs), "no timescan done"
    print(f"  timescan: {len(t_ps)} pts x2 reps OK -> "
          f"{[m['dir'] for m in msgs if m['kind']=='done'][0]}")
    hub.close()

    # mapping
    hub.connect(cfg, mode="mock", need_stage=True)
    q = queue.Queue()
    md = mapping_loop(cfg, hub, q, stop)
    msgs = _drain(q)
    assert md is not None, "mapping produced no data"
    pts = [m for m in msgs if m["kind"] == "point_mapping"]
    assert pts and pts[-1]["count"] == pts[-1]["total"], "mapping raster incomplete"
    print(f"  mapping: {pts[-1]['total']} pts OK -> "
          f"{[m['dir'] for m in msgs if m['kind']=='done'][0]}")
    hub.close()

    # stop-event mid-scan
    hub.connect(cfg, mode="mock", need_stage=False)
    q = queue.Queue()
    stop2 = threading.Event()
    stop2.set()   # stop immediately
    timescan_loop(cfg, hub, q, stop2, repetition=1)
    msgs = _drain(q)
    assert any(m["kind"] == "done" and m.get("stopped") for m in msgs), "stop not honoured"
    print("  stop-event honoured OK")
    hub.close()

    print("=== self-test PASSED ===")


def _drain(q):
    out = []
    while True:
        try:
            out.append(q.get_nowait())
        except queue.Empty:
            break
    return out


# ============================================================
# GUI  (Tk imports are lazy so --selftest works without a display)
# ============================================================
def run_app(smoke=False, test_hook=None):
    """test_hook(app) -> build the window, hand it to the hook, tear it down (no mainloop).
    Lets the GUI logic be unit-tested headlessly instead of only by hand at the microscope."""
    import tkinter as tk
    from tkinter import ttk, filedialog, messagebox
    import matplotlib
    matplotlib.use("TkAgg")
    from matplotlib.figure import Figure
    from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
    from matplotlib.widgets import RectangleSelector

    MODE_LABELS = {
        "Mock (simulated)": "mock",
        "Real motion + mock signal": "mock_lockin",
        "Real (all hardware)": "real",
    }

    class Tip:
        """Hover tooltip: keeps long explanations off the panels."""
        def __init__(self, widget, text):
            self.w, self.text, self.tw = widget, text, None
            widget.bind("<Enter>", self.show, add="+"); widget.bind("<Leave>", self.hide, add="+")

        def show(self, _=None):
            if self.tw or not self.text:
                return
            x, y = self.w.winfo_rootx() + 12, self.w.winfo_rooty() + self.w.winfo_height() + 4
            self.tw = tk.Toplevel(self.w); self.tw.wm_overrideredirect(True); self.tw.wm_geometry(f"+{x}+{y}")
            tk.Label(self.tw, text=self.text, justify="left", wraplength=420, background="#fffbe6",
                     relief="solid", borderwidth=1, padx=8, pady=5).pack()

        def hide(self, _=None):
            if self.tw:
                self.tw.destroy(); self.tw = None

    class MokeApp(tk.Tk):
        def __init__(self):
            super().__init__()
            self.title(f"TR-MOKE Control  v{__version__}")
            try:   # flatter, clearer look than the default Windows theme
                st = ttk.Style(self)
                if "clam" in st.theme_names():
                    st.theme_use("clam")
                base = ("Segoe UI", 10) if os.name == "nt" else ("Helvetica", 12)
                st.configure(".", font=base)
                st.configure("TLabelframe.Label", font=(base[0], base[1], "bold"), foreground="#1f3a5f")
                st.configure("TButton", padding=(8, 2))
                st.configure("Accent.TButton", foreground="white", background="#2563eb")
                st.map("Accent.TButton", background=[("active", "#1d4ed8"), ("disabled", "#9db4e8")])
                st.configure("Bar.TFrame", background="#eef2f7")
                st.configure("Bar.TLabel", background="#eef2f7")
                st.configure("Bar.TCheckbutton", background="#eef2f7")
                st.configure("Bad.TEntry", fieldbackground="#ffe4e6", foreground="#b91c1c")
            except Exception:
                pass
            self._connecting = False
            self._conn_status = {}
            self.geometry("1180x760")

            self.config_data = copy.deepcopy(am.CONFIG)
            if os.name == "nt" and self.config_data["shot602_port"].startswith("/dev/"):
                self.config_data["shot602_port"] = "COM5"
            self._apply_config_file()   # optional per-lab moke_gui_config.json overrides
            # delay line 2 = one axis of the SHOT controller (pump delay stage)
            for k, v in (("dl2_axis", 1), ("dl2_pulses_per_mm", int(self.config_data.get("pulses_per_mm", 500))),
                         ("dl2_start_um", 0.0), ("dl2_end_um", 0.0), ("dl2_step_um", 100.0),
                         ("dl2_settle_s", 0.2)):
                self.config_data.setdefault(k, v)
            self.config_data["dl2_enabled"] = False
            self.config_data.setdefault("pump_port", "COM6")   # the new SHOT-602; check Detect hardware
            self.config_data.setdefault("pump_baud", 9600)     # DIP default 9600 (label on the unit)
            # capture GUI-callback errors to the log so pythonw (no console) still debugs
            self.report_callback_exception = self._log_exc

            self.hub = DeviceHub()
            self.out_q = queue.Queue()
            self.stop_event = threading.Event()
            self.pause_event = threading.Event()   # set = paused
            self.worker = None
            self.monitor_on = tk.BooleanVar(value=True)

            # live plot state
            self._ts_t = None
            self._ts_y = None
            self._ts_pick_ps = None     # delay slice picked by clicking the timescan
            self._map_last_draw = 0.0   # live-redraw throttle
            self._map_arr = None
            self._map_x = None
            self._map_z = None
            self._map_delay = 0.0
            self.map_cursor = None
            self._map3d = None
            self._map3d_delays = []
            self._map3d_x = None
            self._map3d_z = None
            self._hw_busy = False   # a manual move/home/jog is talking to the BBD
            self._scan_t0 = None
            self._last_pos_mm = None   # last known delay-line position, for re-labelling
            self.stages = []            # delay-line profiles (see STAGE_KEYS / DEFAULT_STAGES)
            self.active_stage = None
            self._saved_stages = None
            self._saved_active = None

            self._build_topbar()
            self._nb = ttk.Notebook(self)
            self._nb.pack(fill="both", expand=True, padx=6, pady=4)
            # Mapping first — it's the primary (new) system; Timescan second.
            self._build_mapping_tab()
            self._build_timescan_tab()
            self._build_settings_tab()

            # restore last-session parameters, then refresh derived readouts
            self._load_settings()
            try:
                self._apply_settings()
            except Exception:
                pass
            self._init_stages()       # delay-line profiles: restore or build defaults
            self._pump_lim_refresh()
            self._map_mode_switch()   # restored define-by mode shows the right fields
            self._update_ts_estimate()
            self._update_map_estimate()

            self.protocol("WM_DELETE_WINDOW", self._on_close)
            self.after(120, self._poll_queue)
            self.after(800, self._poll_monitor)
            if smoke:
                self.after(2000, self.destroy)   # build-only smoke test
            else:
                self.after(1200, self._connect)  # auto-connect to devices on startup

        # -------------------------------------------------- top bar
        def _build_topbar(self):
            # row 1: devices — which delay line, what to open, connect, status lights
            bar = ttk.Frame(self, style="Bar.TFrame", padding=(8, 6))
            bar.pack(fill="x")
            ttk.Label(bar, text="Mode", style="Bar.TLabel").pack(side="left")
            self.mode_var = tk.StringVar(value="Real (all hardware)")   # default to real devices
            ttk.OptionMenu(bar, self.mode_var, "Real (all hardware)", *MODE_LABELS.keys()).pack(side="left", padx=(4, 14))

            ttk.Label(bar, text="Delay line", style="Bar.TLabel").pack(side="left")
            self.stage_sel_var = tk.StringVar(value="")
            self.stage_combo = ttk.Combobox(bar, textvariable=self.stage_sel_var, state="readonly", width=26)
            self.stage_combo.pack(side="left", padx=(4, 4))
            self.stage_combo.bind("<<ComboboxSelected>>", self._on_stage_selected)
            Tip(self.stage_combo, "ID0 10000000 · ID1 10000001 · ID2 10000002\n"
                                  "Each keeps its own calibration and scan range.")
            self.stage_usb_var = tk.StringVar(value="")
            self.stage_usb_lbl = ttk.Label(bar, textvariable=self.stage_usb_var, style="Bar.TLabel")
            self.stage_usb_lbl.pack(side="left")
            _rb = ttk.Button(bar, text="↻", width=2, command=self._scan_usb_bg)
            _rb.pack(side="left", padx=(2, 14))
            Tip(_rb, "Check again which delay lines are plugged in (USB + power).")

            self.stage_var = tk.BooleanVar(value=True)
            _xz = ttk.Checkbutton(bar, text="XZ", variable=self.stage_var, style="Bar.TCheckbutton")
            _xz.pack(side="left")
            Tip(_xz, "Also open the XZ stage (SHOT-602 on 'SHOT-602 port'). Used by 2D mapping on ID0.")
            self.pump_var = tk.BooleanVar(value=False)
            _pu = ttk.Checkbutton(bar, text="Pump", variable=self.pump_var, style="Bar.TCheckbutton")
            _pu.pack(side="left", padx=(6, 0))
            Tip(_pu, "Also open the pump delay (its own SHOT-602 on 'Pump SHOT port').")

            self.btn_connect = ttk.Button(bar, text="Connect", width=11, style="Accent.TButton",
                                          command=self._toggle_conn)
            self.btn_connect.pack(side="left", padx=(14, 10))
            # kept for older code paths that still set its state; the toggle above is what shows
            self.btn_disconnect = ttk.Button(bar, text="Disconnect", command=self._disconnect, state="disabled")

            self.chips = {}
            for key, lbl in (("bbd", "Delay"), ("lockin", "Lock-in"), ("stage", "XZ"), ("pump", "Pump")):
                c = tk.Label(bar, text=f"● {lbl}", fg="#9aa0a6", bg="#eef2f7", cursor="hand2")
                c.pack(side="left", padx=(0, 8))
                c.bind("<Button-1>", lambda _e, k=key: self._chip_detail(k))
                self.chips[key] = c

            ttk.Button(bar, text="Save as…", command=self._save_current_tab).pack(side="right")

            # row 2: lock-in settings + live readouts
            bar2 = ttk.Frame(self, padding=(8, 4))
            bar2.pack(fill="x")
            ttk.Checkbutton(bar2, text="Live", variable=self.monitor_on).pack(side="left")
            ttk.Label(bar2, text="Lock-in  TC").pack(side="left", padx=(14, 2))
            self.tc_var = tk.StringVar(value=str(self.config_data.get("time_constant_ms", 100)))
            ttk.Entry(bar2, textvariable=self.tc_var, width=6).pack(side="left")
            ttk.Label(bar2, text="ms").pack(side="left", padx=(2, 2))
            self.btn_set_tc = ttk.Button(bar2, text="Set", width=4, command=self._set_tc)
            self.btn_set_tc.pack(side="left")
            ttk.Label(bar2, text="Sens").pack(side="left", padx=(12, 2))
            self.sens_var = tk.StringVar(value=self.config_data.get("sensitivity_label", "10 mV"))
            ttk.OptionMenu(bar2, self.sens_var, self.sens_var.get(), *SENS_LABELS).pack(side="left")
            self.btn_set_sens = ttk.Button(bar2, text="Set", width=4, command=self._set_sensitivity)
            self.btn_set_sens.pack(side="left", padx=(2, 0))

            # Typing in the TC / Sens box does NOT touch the instrument until Set is pressed —
            # scanning with an unapplied box silently ruins the whole map. Say so, loudly.
            self._tc_dev_ms = None      # what the lock-in actually holds (None = unknown)
            self._sens_dev = None
            self.lockin_warn = tk.StringVar(value="")
            ttk.Label(bar2, textvariable=self.lockin_warn, foreground="#dc2626",
                      font=("Segoe UI", 9, "bold")).pack(side="left", padx=(8, 0))
            self.tc_var.trace_add("write", lambda *_: self._check_lockin_applied())
            self.sens_var.trace_add("write", lambda *_: self._check_lockin_applied())

            # right-side big readouts (Position um + R|X value), like LabVIEW
            self.val_var = tk.StringVar(value="—")
            self.val_sig_var = tk.StringVar(value="X")
            self.pos_var = tk.StringVar(value="Main delay: — um")
            box = ttk.Frame(bar2)
            box.pack(side="right")
            # fixed widths so the changing numbers don't reflow the layout (window jitter)
            ttk.Label(box, textvariable=self.pos_var, font=("TkDefaultFont", 10, "bold"),
                      width=34, anchor="e").pack(side="top", anchor="e")
            vrow = ttk.Frame(box); vrow.pack(side="top", anchor="e")
            ttk.OptionMenu(vrow, self.val_sig_var, "X", "R", "X", "Y", "theta").pack(side="left")
            ttk.Label(vrow, textvariable=self.val_var, font=("Consolas", 16, "bold"),
                      foreground="#0a6", width=13, anchor="e").pack(side="left", padx=6)

            self.status_var = tk.StringVar(value="Not connected.")
            self.system_var = tk.StringVar(value="System: —")
            sb = ttk.Frame(self); sb.pack(fill="x", side="bottom")
            # subtle detected-system label on the right, always visible
            ttk.Label(sb, textvariable=self.system_var, relief="sunken", anchor="e",
                      foreground="#666", width=44).pack(side="right")
            ttk.Label(sb, textvariable=self.status_var, relief="sunken",
                      anchor="w").pack(side="left", fill="x", expand=True)

        # -------------------------------------------------- scrollable left panel
        def _scroll_left(self, tab, width=380):
            """A vertically-scrollable left control panel, so adding controls never
            pushes Start/Stop off the bottom of the window."""
            wrap = ttk.Frame(tab); wrap.pack(side="left", fill="y", padx=(6, 0), pady=6)
            canvas = tk.Canvas(wrap, width=width, highlightthickness=0)
            vsb = ttk.Scrollbar(wrap, orient="vertical", command=canvas.yview)
            canvas.configure(yscrollcommand=vsb.set)
            vsb.pack(side="right", fill="y"); canvas.pack(side="left", fill="both", expand=True)
            inner = ttk.Frame(canvas)
            win = canvas.create_window((0, 0), window=inner, anchor="nw")
            inner.bind("<Configure>", lambda e: canvas.configure(scrollregion=canvas.bbox("all")))
            canvas.bind("<Configure>", lambda e: canvas.itemconfigure(win, width=e.width))

            def _wheel(e):
                canvas.yview_scroll(-1 if (getattr(e, "delta", 0) > 0 or getattr(e, "num", 0) == 4) else 1, "units")
            canvas.bind("<Enter>", lambda e: (canvas.bind_all("<MouseWheel>", _wheel),
                                              canvas.bind_all("<Button-4>", _wheel),
                                              canvas.bind_all("<Button-5>", _wheel)))
            canvas.bind("<Leave>", lambda e: (canvas.unbind_all("<MouseWheel>"),
                                              canvas.unbind_all("<Button-4>"),
                                              canvas.unbind_all("<Button-5>")))
            return inner

        # -------------------------------------------------- timescan tab
        def _build_timescan_tab(self):
            tab = ttk.Frame(self._nb); self._nb.add(tab, text="Timescan (TDS)")
            left = self._scroll_left(tab)

            p = ttk.LabelFrame(left, text="Scan — main delay line")
            self.scan_frame = p
            p.pack(fill="x", pady=4)

            urow = ttk.Frame(p); urow.pack(fill="x", pady=(2, 1))
            ttk.Label(urow, text="Range unit", width=12).pack(side="left")
            self.ts_unit = tk.StringVar(value="µm")   # default to position (µm), the usual workflow
            ttk.OptionMenu(urow, self.ts_unit, "µm", "ps", "µm",
                           command=lambda *_: self._update_ts_estimate()).pack(side="left")

            self.ts_vars = {}
            _ts_range_keys = ("time_start_ps", "time_end_ps", "time_step_ps")
            for lbl, key, default in [
                ("Start", "time_start_ps", 553550.0),   # µm defaults: 553550 -> 0. Start AT t0
                ("End", "time_end_ps", 0.0),            # (~553550 µm on the original rig) so delay-from-start
                ("Step", "time_step_ps", 500.0),        # = 0 at t0/ultrafast (left), rising L->R
                ("Averages", "num_averages", 3),
                ("Settle (s)", "bbd_settle_s", 0.2),
                ("Repetition", "repetition", 1),
            ]:
                row = ttk.Frame(p); row.pack(fill="x", pady=1)
                ttk.Label(row, text=lbl, width=12).pack(side="left")
                # range fields seed from the µm defaults; the rest from config
                seed = default if key in _ts_range_keys else self.config_data.get(key, default)
                v = tk.StringVar(value=str(seed))
                _e = ttk.Entry(row, textvariable=v, width=10)
                _e.pack(side="left")
                self.ts_entries = getattr(self, "ts_entries", {}); self.ts_entries[key] = _e
                v.trace_add("write", lambda *_: (self._anchor_t0_to_start(),
                                                 self._update_ts_estimate(),
                                                 self._check_ts_travel()))
                self.ts_vars[key] = v

            self.ts_est = tk.StringVar(value="—")
            ttk.Label(p, textvariable=self.ts_est, justify="left",
                      foreground="#333").pack(anchor="w", pady=(3, 1))
            self._update_ts_estimate()
            # Typing changes only the numbers; Set is what makes the hardware agree with them.
            self.ts_set_btn = ttk.Button(p, text="Set  (move delay line to Scan Start)",
                                         command=self._ts_set)
            self.ts_set_btn.pack(fill="x", pady=(1, 2))
            hr = ttk.Frame(p); hr.pack(fill="x", pady=(0, 2))
            ttk.Button(hr, text="Start = here", command=lambda: self._ts_here("time_start_ps")).pack(side="left", expand=True, fill="x")
            ttk.Button(hr, text="End = here", command=lambda: self._ts_here("time_end_ps")).pack(side="left", expand=True, fill="x", padx=(4, 0))
            self.ts_travel_var = tk.StringVar(value="")
            self.ts_travel_lbl = ttk.Label(p, textvariable=self.ts_travel_var, foreground="#555")
            self.ts_travel_lbl.pack(anchor="w")
            self.ts_unit.trace_add("write", lambda *_: self._check_ts_travel())
            self._check_ts_travel()

            row = ttk.Frame(p); row.pack(fill="x", pady=(4, 1))
            ttk.Label(row, text="Signal", width=12).pack(side="left")
            self.ts_signal = tk.StringVar(value="X")
            ttk.OptionMenu(row, self.ts_signal, "X", "R", "X", "Y", "theta",
                           command=lambda *_: self._redraw_timescan()).pack(side="left")
            # Overlay any subset of X/Y/R/theta live on the plot (all 4 are always recorded;
            # this only changes what's drawn). Lets you see if the signal is hiding in R/Y when X
            # looks like noise. The 'Signal' menu above is the PRIMARY channel (pick/FFT/after-scan).
            ttk.Label(row, text="  show").pack(side="left")
            self.ts_show = {}
            _lbl = {"theta": "θ", "auto": "X′(auto-φ)"}
            # X only out of the box — R rode along by default and cluttered every fresh
            # plot. Whatever is ticked here is saved, so the next launch restores it.
            for ch, on in (("X", True), ("Y", False), ("R", False), ("theta", False), ("auto", False)):
                v = tk.BooleanVar(value=on); self.ts_show[ch] = v
                ttk.Checkbutton(row, text=_lbl.get(ch, ch), variable=v,
                                command=lambda *_: self._redraw_timescan()).pack(side="left")
            row = ttk.Frame(p); row.pack(fill="x", pady=1)
            ttk.Label(row, text="View", width=12).pack(side="left")
            self.ts_view = tk.StringVar(value="TDS")
            ttk.OptionMenu(row, self.ts_view, "TDS", "TDS", "FFT", command=lambda *_: self._redraw_timescan()).pack(side="left")
            row = ttk.Frame(p); row.pack(fill="x", pady=1)
            ttk.Label(row, text="After scan", width=12).pack(side="left")
            self.ts_after = tk.StringVar(value="Pre-scan pos")
            ttk.OptionMenu(row, self.ts_after, "Pre-scan pos",
                           "Pre-scan pos", "Stay", "Max Position", "Min Position").pack(side="left")

            # delay line 2 (pump), swept together with delay line 1 (dual-delay scan)
            d2 = ttk.LabelFrame(left, text="Pump delay  (own SHOT-602)")
            d2.pack(fill="x", pady=4)
            self.dl2_on = tk.BooleanVar(value=False)
            ttk.Checkbutton(d2, text="Move during this scan", variable=self.dl2_on).pack(anchor="w")
            self.dl2_vars = {}
            for lbl, key in (("Axis (1/2)", "dl2_axis"), ("Pulses/mm", "dl2_pulses_per_mm"),
                             ("Start (µm)", "dl2_start_um"), ("End (µm)", "dl2_end_um"),
                             ("Step (µm)", "dl2_step_um"), ("Settle (s)", "dl2_settle_s")):
                row = ttk.Frame(d2); row.pack(fill="x", pady=1)
                ttk.Label(row, text=lbl, width=12).pack(side="left")
                v = tk.StringVar(value=str(self.config_data.get(key, "")))
                ttk.Entry(row, textvariable=v, width=10).pack(side="left")
                self.dl2_vars[key] = v
            _ph = ttk.Label(d2, foreground="#555", text="Start = End: waits  ·  Main Start = End: pump-only scan  (hover)")
            _ph.pack(anchor="w", pady=(2, 1))
            Tip(_ph, "Pump Start = End → the pump delay waits there while the main delay line scans.\n"
                     "Both ranges set → both move together, same number of points.\n"
                     "Main Start = End (in 'Scan') → only the pump delay scans; its Step sets the points.\n"
                     "Own SHOT-602: tick 'Pump' and Connect; port = 'Pump SHOT port' in Settings.")
            lr = ttk.Frame(d2); lr.pack(fill="x", pady=(3, 1))
            ttk.Label(lr, text="Limits (µm)", width=12).pack(side="left")
            self.pump_lim_vars = {"pump_min_um": tk.StringVar(value=""), "pump_max_um": tk.StringVar(value="")}
            ttk.Entry(lr, textvariable=self.pump_lim_vars["pump_min_um"], width=8).pack(side="left")
            ttk.Label(lr, text="to").pack(side="left", padx=2)
            ttk.Entry(lr, textvariable=self.pump_lim_vars["pump_max_um"], width=8).pack(side="left")
            self.pump_lim_note = tk.StringVar(value="")
            ttk.Label(lr, textvariable=self.pump_lim_note, foreground="#a00").pack(side="left", padx=4)
            for v in self.pump_lim_vars.values():
                v.trace_add("write", lambda *_: self._pump_lim_refresh())
            pr = ttk.Frame(d2); pr.pack(fill="x", pady=(4, 1))
            self.pump_pos_var = tk.StringVar(value="Pump delay position: —")
            ttk.Label(pr, textvariable=self.pump_pos_var).pack(side="left")
            ttk.Button(pr, text="Read", width=6, command=self._pump_read).pack(side="left", padx=4)
            gr = ttk.Frame(d2); gr.pack(fill="x", pady=1)
            ttk.Button(gr, text="→ Pump Start", command=lambda: self._pump_go("dl2_start_um")).pack(side="left")
            ttk.Button(gr, text="→ Pump End", command=lambda: self._pump_go("dl2_end_um")).pack(side="left", padx=2)
            jr = ttk.Frame(d2); jr.pack(fill="x", pady=1)
            ttk.Label(jr, text="Jog (µm)", width=12).pack(side="left")
            self.pump_jog_um = tk.StringVar(value="10")
            ttk.Entry(jr, textvariable=self.pump_jog_um, width=7).pack(side="left", padx=2)
            ttk.Button(jr, text="−", width=3, command=lambda: self._pump_jog(-1)).pack(side="left", padx=1)
            ttk.Button(jr, text="+", width=3, command=lambda: self._pump_jog(+1)).pack(side="left")
            tr2 = ttk.Frame(d2); tr2.pack(fill="x", pady=1)
            ttk.Label(tr2, text="Target (µm)", width=12).pack(side="left")
            self.pump_target_um = tk.StringVar(value="0")
            ttk.Entry(tr2, textvariable=self.pump_target_um, width=10).pack(side="left", padx=2)
            ttk.Button(tr2, text="Move to", command=self._pump_move_target).pack(side="left")

            # stage control (BBD delay line), mirrors LabVIEW "Stage control"
            sc = ttk.LabelFrame(left, text="Main delay line control")
            self.main_ctrl_frame = sc
            sc.pack(fill="x", pady=4)
            bg = ttk.Frame(sc); bg.pack(fill="x", pady=1)
            for i, (txt, cmd) in enumerate([
                ("→ Scan Start", lambda: self._bbd_go_ps("time_start_ps")),
                ("→ Scan End",   lambda: self._bbd_go_ps("time_end_ps")),
                ("→ Max signal", lambda: self._bbd_go_extreme("max")),
                ("→ Min signal", lambda: self._bbd_go_extreme("min")),
                ("Home",         self._bbd_home),
                ("Refresh pos",  self._refresh_position),
            ]):
                ttk.Button(bg, text=txt, width=13, command=cmd).grid(
                    row=i // 2, column=i % 2, padx=1, pady=1, sticky="ew")
            bg.columnconfigure(0, weight=1); bg.columnconfigure(1, weight=1)
            jr = ttk.Frame(sc); jr.pack(fill="x", pady=2)
            ttk.Label(jr, text="Jog").pack(side="left")
            self.ts_jog_um = tk.StringVar(value="5")
            ttk.Entry(jr, textvariable=self.ts_jog_um, width=7).pack(side="left", padx=2)
            ttk.Label(jr, text="um").pack(side="left")
            ttk.Button(jr, text="−", width=3, command=lambda: self._bbd_jog(-1)).pack(side="left", padx=1)
            ttk.Button(jr, text="+", width=3, command=lambda: self._bbd_jog(+1)).pack(side="left")
            tr = ttk.Frame(sc); tr.pack(fill="x", pady=2)
            ttk.Label(tr, text="Target (um)").pack(side="left")
            self.ts_target_um = tk.StringVar(value="")
            ttk.Entry(tr, textvariable=self.ts_target_um, width=10).pack(side="left", padx=2)
            ttk.Button(tr, text="Move to", command=self._bbd_move_target).pack(side="left")

            br = ttk.Frame(left); br.pack(fill="x", pady=6)
            self.ts_start = ttk.Button(br, text="Start", command=self._start_timescan)
            self.ts_start.pack(side="left", expand=True, fill="x", padx=2)
            self.ts_pause = ttk.Button(br, text="Pause", command=self._toggle_pause, state="disabled")
            self.ts_pause.pack(side="left", expand=True, fill="x", padx=2)
            self.ts_stop = ttk.Button(br, text="Stop", command=self._stop_scan, state="disabled")
            self.ts_stop.pack(side="left", expand=True, fill="x", padx=2)

            self.ts_prog = ttk.Progressbar(left, mode="determinate")
            self.ts_prog.pack(fill="x", pady=(4, 0))
            self.ts_time_var = tk.StringVar(value="Total —   /   To go —")
            ttk.Label(left, textvariable=self.ts_time_var, width=40, anchor="w").pack(anchor="w")
            ttk.Button(left, text="Save last scan as…", command=self._save_as).pack(fill="x", pady=(4, 0))

            # plot
            right = ttk.Frame(tab); right.pack(side="left", fill="both", expand=True, padx=4, pady=6)
            self.ts_fig = Figure(figsize=(6, 4.4), dpi=100)
            self.ts_ax = self.ts_fig.add_subplot(111)
            self.ts_ax.set_xlabel("Delay (ps)"); self.ts_ax.set_ylabel("Amplitude (V)")
            # white trace + open-square markers on black/yellow, like the LabVIEW graph
            self.ts_line, = self.ts_ax.plot([], [], "-s", lw=1, ms=4,
                                            mfc="none", mec="white", color="white", label="X")
            # one extra line per channel so X/Y/R/theta can be overlaid live (colours match LabVIEW-ish)
            self.ts_lines = {"X": self.ts_line}
            for ch, c, lb in (("Y", "#4fc3f7", "Y"), ("R", "#ffd54f", "R"),
                              ("theta", "#e57373", "θ"), ("auto", "#69f0ae", "X′(auto-φ)")):
                self.ts_lines[ch], = self.ts_ax.plot([], [], "-", lw=1.3, color=c, label=lb)
            # the delay slice picked by clicking the curve -> feeds the mapping tab
            self.ts_pick, = self.ts_ax.plot([], [], "o", ms=11, mfc="none", mec="red", mew=2)
            self._style_labview(self.ts_fig, self.ts_ax)
            self.ts_canvas = FigureCanvasTkAgg(self.ts_fig, master=right)
            self.ts_canvas.get_tk_widget().pack(fill="both", expand=True)
            self.ts_canvas.mpl_connect("button_press_event", self._on_ts_click)
            tbrow = ttk.Frame(right); tbrow.pack(fill="x")
            NavigationToolbar2Tk(self.ts_canvas, tbrow)
            ttk.Button(tbrow, text="Reset view", command=self._reset_ts_view).pack(side="right", padx=4)
            pk = ttk.Frame(right); pk.pack(fill="x", pady=(0, 2))
            ttk.Label(pk, text="Click the curve to pick the delay slice to map →").pack(side="left", padx=4)
            ttk.Button(pk, text="+ add to 3D delays", command=self._ts_pick_to_3d).pack(side="left", padx=2)
            ttk.Button(pk, text="clear", width=6, command=self._ts_pick_clear).pack(side="left")
            ttk.Button(pk, text="set t0 from this scan",
                       command=self._ts_t0_from_scan).pack(side="right", padx=4)

        # -------------------------------------------------- mapping tab
        def _build_mapping_tab(self):
            tab = ttk.Frame(self._nb); self._nb.add(tab, text="2D Mapping")
            left = self._scroll_left(tab)

            ttk.Label(left, wraplength=330, foreground="#a60", justify="left",
                      text="2D mapping: Stage A (ID0) only.").pack(anchor="w", pady=(2, 0))
            p = ttk.LabelFrame(left, text="Scan area (XZ)")
            p.pack(fill="x", pady=4)
            self.map_vars = {}

            def _me(parent, key, default, width=7):
                v = tk.StringVar(value=str(self.config_data.get(key, default)))
                ttk.Entry(parent, textvariable=v, width=width).pack(side="left", padx=1)
                v.trace_add("write", lambda *_: self._update_map_estimate())
                self.map_vars[key] = v

            mrow = ttk.Frame(p); mrow.pack(fill="x", pady=(2, 1))
            ttk.Label(mrow, text="Define by", width=10).pack(side="left")
            self.map_mode = tk.StringVar(value="Start/End")
            ttk.Radiobutton(mrow, text="Start/End", variable=self.map_mode, value="Start/End",
                            command=self._map_mode_switch).pack(side="left")
            ttk.Radiobutton(mrow, text="Center+Size", variable=self.map_mode, value="Center+Size",
                            command=self._map_mode_switch).pack(side="left")

            # only the active definition-mode's fields are shown (saves vertical space)
            self.map_mode_container = ttk.Frame(p); self.map_mode_container.pack(fill="x")
            self.map_se_frame = ttk.Frame(self.map_mode_container)
            r = ttk.Frame(self.map_se_frame); r.pack(fill="x", pady=1)
            ttk.Label(r, text="X start→end", width=11).pack(side="left"); _me(r, "x_start", 6.0); _me(r, "x_end", 12.0)
            r = ttk.Frame(self.map_se_frame); r.pack(fill="x", pady=1)
            ttk.Label(r, text="Z start→end", width=11).pack(side="left"); _me(r, "z_start", 3.0); _me(r, "z_end", 8.0)
            self.map_cs_frame = ttk.Frame(self.map_mode_container)
            self.map_cs = {}

            def _mcs(parent, key, default, width=7):
                v = tk.StringVar(value=str(default))
                ttk.Entry(parent, textvariable=v, width=width).pack(side="left", padx=1)
                v.trace_add("write", lambda *_: self._update_map_estimate())
                self.map_cs[key] = v
            r = ttk.Frame(self.map_cs_frame); r.pack(fill="x", pady=1)
            ttk.Label(r, text="X ctr, size", width=11).pack(side="left"); _mcs(r, "x_center", 9.0); _mcs(r, "x_size", 6.0)
            r = ttk.Frame(self.map_cs_frame); r.pack(fill="x", pady=1)
            ttk.Label(r, text="Z ctr, size", width=11).pack(side="left"); _mcs(r, "z_center", 5.5); _mcs(r, "z_size", 5.0)

            r = ttk.Frame(p); r.pack(fill="x", pady=1)
            ttk.Label(r, text="Step X, Z", width=11).pack(side="left"); _me(r, "x_step", 1.0); _me(r, "z_step", 1.0)
            r = ttk.Frame(p); r.pack(fill="x", pady=1)
            ttk.Label(r, text="Delay ps", width=11).pack(side="left"); _me(r, "delay_label_ps", 0.0)
            ttk.Label(r, text="avg").pack(side="left"); _me(r, "num_averages", 3, width=4)
            ttk.Label(r, text="settle").pack(side="left"); _me(r, "settling_time", 0.5, width=5)

            # ps <-> actual stage um, EDITABLE both ways (ps = position/149.9). Type in either box
            # and the other tracks it, so you can set the map delay by real stage position too.
            r = ttk.Frame(p); r.pack(fill="x", pady=(0, 3))
            ttk.Label(r, text="Pos um", width=11).pack(side="left")
            self.map_delay_um = tk.StringVar(value="")
            self.map_delay_um_lbl = ttk.Entry(r, textvariable=self.map_delay_um, width=10)
            self.map_delay_um_lbl.pack(side="left", padx=1)
            ttk.Label(r, text="um").pack(side="left")
            self.map_delay_go = ttk.Button(r, text="Move", width=6, command=self._map_delay_go)
            self.map_delay_go.pack(side="left", padx=1)
            self.map_delay_now = ttk.Button(r, text="Read pos", width=9, command=self._map_delay_from_current)
            self.map_delay_now.pack(side="left", padx=1)
            ttk.Label(p, foreground="#666", font=("Segoe UI", 8),
                      text="        Delay ps and Pos um are linked (ps = um/149.9). Move drives the "
                           "delay line here; Read pos fills it from where the line is now.").pack(anchor="w")
            self._delay_sync = False   # guard against the two traces ping-ponging
            self.map_vars["delay_label_ps"].trace_add("write", lambda *_: self._update_map_delay_um())
            self.map_delay_um.trace_add("write", lambda *_: self._sync_delay_from_um())
            self._update_map_delay_um()

            r = ttk.Frame(p); r.pack(fill="x", pady=(3, 1))
            ttk.Label(r, text="Signal", width=11).pack(side="left")
            self.map_signal = tk.StringVar(value="X")   # X (in-phase) is the MOKE signal, not R
            ttk.OptionMenu(r, self.map_signal, "X", "X", "R", "Y", "theta",
                           command=lambda *_: self._redraw_mapping()).pack(side="left")
            ttk.Label(r, text="cmap").pack(side="left")
            self.map_cmap = tk.StringVar(value="viridis")
            ttk.OptionMenu(r, self.map_cmap, "viridis", "viridis", "inferno", "gray", "RdBu_r", "jet", "hot",
                           command=lambda *_: self._change_cmap()).pack(side="left")

            r = ttk.Frame(p); r.pack(fill="x", pady=1)
            ttk.Label(r, text="Color scale", width=11).pack(side="left")
            self.map_clim_lock = tk.BooleanVar(value=False)
            ttk.Checkbutton(r, text="lock", variable=self.map_clim_lock, command=self._redraw_mapping).pack(side="left")
            self.map_clim_lo = tk.StringVar(value=""); self.map_clim_hi = tk.StringVar(value="")
            ttk.Entry(r, textvariable=self.map_clim_lo, width=7).pack(side="left", padx=1)
            ttk.Entry(r, textvariable=self.map_clim_hi, width=7).pack(side="left", padx=1)
            ttk.Button(r, text="←data", width=6, command=self._clim_from_data).pack(side="left")

            r = ttk.Frame(p); r.pack(fill="x", pady=1)
            ttk.Label(r, text="After scan", width=11).pack(side="left")
            self.map_after = tk.StringVar(value="Return to start")
            ttk.OptionMenu(r, self.map_after, "Return to start", "Return to start", "Stay").pack(side="left")
            # orientation: flip display axes to match the sample as seen from the laser side
            r = ttk.Frame(p); r.pack(fill="x", pady=1)
            ttk.Label(r, text="Orient", width=11).pack(side="left")
            self.map_flip_x = tk.BooleanVar(value=False)
            self.map_flip_z = tk.BooleanVar(value=False)
            ttk.Checkbutton(r, text="flip X", variable=self.map_flip_x, command=self._flip_changed).pack(side="left")
            ttk.Checkbutton(r, text="flip Z", variable=self.map_flip_z, command=self._flip_changed).pack(side="left")

            r = ttk.Frame(p); r.pack(fill="x", pady=(3, 1))
            self.map3d_on = tk.BooleanVar(value=False)
            ttk.Checkbutton(r, text="3D delays", variable=self.map3d_on).pack(side="left")
            self.map3d_delays_var = tk.StringVar(value="0, 100, 500")
            ttk.Entry(r, textvariable=self.map3d_delays_var, width=13).pack(side="left", padx=2)
            ttk.Label(r, text="view").pack(side="left")
            self.map3d_view = tk.StringVar(value="—")
            self.map3d_view_menu = ttk.OptionMenu(r, self.map3d_view, "—")
            self.map3d_view_menu.pack(side="left")

            self.map_est = tk.StringVar(value="Grid: —")
            self._map_mode_switch()   # show only the active definition-mode fields
            ttk.Label(left, textvariable=self.map_est, justify="left",
                      foreground="#333").pack(anchor="w", pady=(2, 0))
            self._update_map_estimate()

            # XZ jog
            sc = ttk.LabelFrame(left, text="Stage control (XZ)")
            sc.pack(fill="x", pady=4)
            xr = ttk.Frame(sc); xr.pack(fill="x", pady=1)
            ttk.Label(xr, text="X→(mm)").pack(side="left")
            self.map_x_to = tk.StringVar(value="")
            ttk.Entry(xr, textvariable=self.map_x_to, width=8).pack(side="left", padx=2)
            ttk.Button(xr, text="Move X", command=lambda: self._xz_move(1, self.map_x_to)).pack(side="left")
            zr = ttk.Frame(sc); zr.pack(fill="x", pady=1)
            ttk.Label(zr, text="Z→(mm)").pack(side="left")
            self.map_z_to = tk.StringVar(value="")
            ttk.Entry(zr, textvariable=self.map_z_to, width=8).pack(side="left", padx=2)
            ttk.Button(zr, text="Move Z", command=lambda: self._xz_move(2, self.map_z_to)).pack(side="left")
            ttk.Button(sc, text="Refresh XZ position", command=self._refresh_xz).pack(fill="x", pady=1)
            self.map_click_move = tk.BooleanVar(value=False)
            ttk.Checkbutton(sc, text="Click map → move stage there (disabled while scanning)",
                            variable=self.map_click_move).pack(anchor="w")
            # map -> timescan: park on an interesting pixel, then take its full delay trace
            ttk.Button(sc, text="Move to X,Z above → Timescan there",
                       command=self._goto_xz_then_timescan).pack(fill="x", pady=1)
            ttk.Button(sc, text="Select ROI on map → set scan area",
                       command=self._enable_roi).pack(fill="x", pady=1)
            self.map_xz_pos = tk.StringVar(value="XZ: —")
            ttk.Label(sc, textvariable=self.map_xz_pos).pack(anchor="w")

            br = ttk.Frame(left); br.pack(fill="x", pady=6)
            self.map_start = ttk.Button(br, text="Start", command=self._start_mapping)
            self.map_start.pack(side="left", expand=True, fill="x", padx=2)
            self.map_pause = ttk.Button(br, text="Pause", command=self._toggle_pause, state="disabled")
            self.map_pause.pack(side="left", expand=True, fill="x", padx=2)
            self.map_stop = ttk.Button(br, text="Stop", command=self._stop_scan, state="disabled")
            self.map_stop.pack(side="left", expand=True, fill="x", padx=2)

            self.map_prog = ttk.Progressbar(left, mode="determinate")
            self.map_prog.pack(fill="x", pady=(4, 0))
            self.map_time_var = tk.StringVar(value="Total —   /   To go —")
            ttk.Label(left, textvariable=self.map_time_var, width=40, anchor="w").pack(anchor="w")
            ttk.Button(left, text="Save last scan as…", command=self._save_as).pack(fill="x", pady=(4, 0))

            right = ttk.Frame(tab); right.pack(side="left", fill="both", expand=True, padx=4, pady=6)
            self.map_fig = Figure(figsize=(6, 4.6), dpi=100)
            self.map_fig.patch.set_facecolor("#ececec")
            self.map_ax = self.map_fig.add_subplot(111)
            self.map_ax.set_facecolor("black")
            self.map_ax.set_xlabel("Z (mm)"); self.map_ax.set_ylabel("X (mm)")
            self.map_im = None
            self.map_cbar = None
            self.map_canvas = FigureCanvasTkAgg(self.map_fig, master=right)
            self.map_canvas.get_tk_widget().pack(fill="both", expand=True)
            tbrow = ttk.Frame(right); tbrow.pack(fill="x")
            self.map_toolbar = NavigationToolbar2Tk(self.map_canvas, tbrow)
            ttk.Button(tbrow, text="Reset view", command=self._reset_map_view).pack(side="right", padx=4)
            self.map_canvas.mpl_connect("button_press_event", self._on_map_click)
            self._roi_selector = None

        # -------------------------------------------------- settings tab
        def _build_settings_tab(self):
            tab = ttk.Frame(self._nb); self._nb.add(tab, text="Settings / Devices")
            p = ttk.LabelFrame(tab, text="Device configuration")
            p.pack(fill="x", padx=8, pady=8)
            self.cfg_stage_lbl = tk.StringVar(value="")
            ttk.Label(p, textvariable=self.cfg_stage_lbl, foreground="#0a4").pack(anchor="w", pady=(0, 4))
            self.cfg_vars = {}
            for lbl, key in [
                ("BBD serial", "bbd_serial"),
                ("BBD channel", "bbd_channel"),
                ("BBD travel (mm)", "bbd_travel_mm"),
                ("BBD counts/mm", "bbd_counts_per_mm"),
                ("BBD rack system", "bbd_is_rack_system"),
                ("BBD t0 (mm)", "bbd_t0_mm"),
                ("BBD double-pass", "bbd_double_pass"),
                ("BBD reverse", "bbd_reverse"),
                ("SHOT-602 port", "shot602_port"),
                ("SHOT-602 baud", "shot602_baud"),
                ("Pump SHOT port", "pump_port"),
                ("Pump SHOT baud", "pump_baud"),
                ("Pulses/mm", "pulses_per_mm"),
                ("X axis", "x_axis"), ("Z axis", "z_axis"),
                ("Stage fast speed (pulse/s)", "stage_speed_fast"),
                ("Lock-in addr", "lockin_addr"),
                ("Lock-in model", "lockin_model"),
            ]:
                row = ttk.Frame(p); row.pack(fill="x", pady=1)
                ttk.Label(row, text=lbl, width=18).pack(side="left")
                v = tk.StringVar(value=str(self.config_data.get(key, "")))
                ttk.Entry(row, textvariable=v, width=28).pack(side="left")
                self.cfg_vars[key] = v
            _ds = ttk.Label(p, foreground="#555", text="t0 / direction: park on the overlap → 't0 = here' → reverse=True → Apply  (hover)")
            _ds.pack(anchor="w", pady=(6, 2))
            Tip(_ds, "Delay sign: with reverse=False the app's ps is just (position − t0)/0.1499, so a LARGER "
                     "position reads as a LARGER 'ps' — backwards from the physics. Park the delay line on the "
                     "pump–probe overlap, press the button below, set reverse=True, then Apply: after that 'ps' "
                     "is the real delay from t0. (Apply pushes the calibration straight to the delay line, so no "
                     "reconnect is needed.)")
            ttk.Button(p, text="t0 = the delay line's current position",
                       command=self._set_t0_here).pack(anchor="w", pady=(0, 2))
            ttk.Button(p, text="Apply to config", command=self._apply_settings).pack(anchor="w", pady=(6, 2))
            ttk.Button(p, text="Detect hardware (scan GPIB / serial / Kinesis → identify system)",
                       command=self._detect_hardware).pack(anchor="w", pady=(0, 4))

            info = ttk.LabelFrame(tab, text="Connected instruments")
            info.pack(fill="both", expand=True, padx=8, pady=8)
            self.idn_text = tk.Text(info, height=10, wrap="word")
            self.idn_text.pack(fill="both", expand=True)
            self.idn_text.insert("1.0", "Not connected.\n")
            self.idn_text.config(state="disabled")

        # -------------------------------------------------- config helpers
        def _set_t0_here(self):
            """Calibrate time-zero: whatever position the delay line is parked at becomes 0 ps.
            Park it on the pump-probe overlap (the ultrafast spike) first, then press this."""
            if not self._require_conn():
                return
            try:
                mm = float(self.hub.read_position_mm())
            except Exception as e:
                self._set_status(f"Can't read the delay line: {type(e).__name__}: {e}"); return
            self.cfg_vars["bbd_t0_mm"].set(f"{mm:.4f}")
            self._set_status(f"t0 = {mm:.4f} mm ({mm * 1000:,.0f} um). "
                             f"Set 'BBD reverse' to True, press 'Apply to config', then reconnect.")

        def _apply_settings(self):
            for key, v in self.cfg_vars.items():
                raw = v.get().strip()
                cur = self.config_data.get(key)
                try:
                    if isinstance(cur, bool):
                        self.config_data[key] = raw.lower() in ("1", "true", "yes", "on")
                    elif isinstance(cur, int):
                        self.config_data[key] = int(float(raw))
                    elif isinstance(cur, float):
                        self.config_data[key] = float(raw)
                    else:
                        self.config_data[key] = raw
                except ValueError:
                    self.config_data[key] = raw
            # t0 / double-pass / reverse changed here must reach the OPEN delay line too,
            # otherwise "Apply" silently leaves the motor converting the old way.
            self.hub.sync_bbd_calibration(self.config_data)
            self._set_status("Config updated.")
            self._update_map_estimate()   # speed change may affect the time estimate
            self._update_ts_estimate()    # t0/reverse move the ps preview

        def _detect_hardware(self):
            """Full read-only bus scan (GPIB *IDN? + serial ports + Kinesis) → identify
            the system. Best run while disconnected so it doesn't share held sessions."""
            if self.worker and self.worker.is_alive():
                messagebox.showwarning("Busy", "Stop the running scan first."); return
            self._apply_settings()
            self._hw_busy = True
            self._set_status("Detecting hardware on all buses…")
            cfg = copy.deepcopy(self.config_data)

            def work():
                try:
                    det = am.detect_hardware(cfg)
                    self.out_q.put({"kind": "hardware", "det": det})
                except Exception as e:
                    self.out_q.put({"kind": "status", "text": f"Detect failed: {e}"})
                finally:
                    self._hw_busy = False
            threading.Thread(target=work, daemon=True).start()

        def _show_hardware(self, det):
            lines = [f"Detected system:  {det.get('system', '?')}", "", "GPIB:"]
            for g in det.get("gpib", []):
                lines.append(f"   {g['addr']}   {g.get('idn', '') or '(no *IDN? response)'}")
            if not det.get("gpib"):
                lines.append(f"   (none — {det.get('gpib_error', 'no instruments')})")
            lines.append("Serial ports:")
            for s in det.get("serial", []):
                lines.append(f"   {s['port']}  {s.get('desc', '')}  [{s.get('hwid', '')}]")
            lines.append("Thorlabs / Kinesis:")
            for k in det.get("kinesis", []):
                lines.append(f"   {k}")
            self._set_idn("\n".join(lines))
            self.system_var.set(f"System: {det.get('system', '?')}")
            self._set_status("Hardware detection done — see Settings / Devices tab.")

        def _pull(self, vars_dict):
            """Merge Entry values (parsed to the CONFIG dtype) into config_data."""
            for key, var in vars_dict.items():
                if key == "repetition":
                    continue
                cur = self.config_data.get(key)
                raw = var.get().strip()
                try:
                    if isinstance(cur, int) and not isinstance(cur, bool):
                        self.config_data[key] = int(float(raw))
                    else:
                        self.config_data[key] = float(raw)
                except ValueError:
                    raise ValueError(f"Bad value for {key!r}: {raw!r}")

        # -------------------------------------------------- connect / disconnect
        def _swap_delay_line(self):
            """Close the old delay line, open the selected one; nothing else is touched."""
            self._connecting = True
            self._hw_busy = True                 # live monitor off while the delay line changes
            cfg = copy.deepcopy(self.config_data)
            name = cfg.get("stage_name", "?")

            def work():
                try:
                    st = self.hub.reconnect_bbd(cfg)
                except Exception as e:
                    st = f"FAILED: {type(e).__name__}: {e}"
                pos = ""
                if st == "ok":
                    try:
                        pos = f"{self.hub.read_position_mm():.4f} mm"
                    except Exception as e:
                        pos = f"position read failed: {e}"
                usb = _list_delay_lines()        # the open is done -> safe to enumerate
                if usb is not None and st == "ok":
                    usb = sorted(set(usb) | {str(cfg.get("bbd_serial"))})
                self._log(f"swap delay line -> {name}: {st} {pos}")
                self.out_q.put({"kind": "bbd_swapped", "status": st, "pos": pos, "name": name})
                self.out_q.put({"kind": "usb_stages", "serials": usb})
            threading.Thread(target=work, daemon=True).start()

        def _toggle_conn(self):
            if self.worker and self.worker.is_alive():
                return
            if self.hub.connected:
                self._disconnect()
            else:
                self._connect()

        def _sync_conn_ui(self):
            """One button, derived from state — no scattered enable/disable bookkeeping."""
            running = bool(self.worker and self.worker.is_alive())
            if running:
                text, state, style = "Disconnect", "disabled", "TButton"
            elif self._connecting:
                text, state, style = "Connecting…", "disabled", "TButton"
            elif self.hub.connected:
                text, state, style = "Disconnect", "normal", "TButton"
            else:
                text, state, style = "Connect", "normal", "Accent.TButton"
            if self.btn_connect.cget("text") != text:
                self.btn_connect.config(text=text)
            self.btn_connect.config(state=state, style=style)
            for key, chip in getattr(self, "chips", {}).items():
                st = str(self._conn_status.get(key, "n/a")) if self.hub.connected else ""
                color = "#d97706" if self._connecting else (
                    "#16a34a" if st == "ok" else "#dc2626" if st.startswith("FAILED") else "#9aa0a6")
                if chip.cget("fg") != color:
                    chip.config(fg=color)

        def _chip_detail(self, key):
            st = str(self._conn_status.get(key, "")) if self.hub.connected else ""
            names = {"bbd": "Main delay line", "lockin": "Lock-in", "stage": "XZ stage", "pump": "Pump delay"}
            if not st or st == "n/a":
                messagebox.showinfo(names[key], "Not connected." if not self.hub.connected else "Not requested.")
            elif st == "ok":
                messagebox.showinfo(names[key], "Connected.")
            else:
                messagebox.showwarning(names[key], self._plain_error(st) + "\n\n" + self._conn_hint(key)
                                       + "\n\n(details: " + st[:160] + ")")

        @staticmethod
        def _plain_error(st):
            t = str(st)
            if "Device Not Opened" in t:
                return "already open in another program (LabVIEW? another window?)"
            if "Device Not Found" in t:
                return "not found on USB (unplugged or powered off?)"
            if "Access is denied" in t or "PermissionError" in t:
                return "port in use by another program"
            if "could not open port" in t or "FileNotFoundError" in t:
                return "COM port not found (wrong port?)"
            if "RSRC_NFOUND" in t or "TMO" in t or "Timeout" in t:
                return "no answer (power? cable? address?)"
            t = t.replace("FAILED:", "").strip()
            return t[:90] + ("…" if len(t) > 90 else "")

        def _conn_hint(self, key):
            c = self.config_data
            return {
                "bbd": "Close LabVIEW Testbench (or the other window), check USB / power, then Connect again.",
                "lockin": "Check power and the GPIB cable; stop LabVIEW reading it.",
                "stage": f"Port {c.get('shot602_port')}: close the program using it, or fix the port in Settings.",
                "pump": f"Port {c.get('pump_port')}: fix it in Settings (Detect hardware lists the ports).",
            }.get(key, "")

        def _connect(self):
            self._apply_settings()
            self._connecting = True
            mode = MODE_LABELS[self.mode_var.get()]
            need_stage = bool(self.stage_var.get())
            need_pump = bool(self.pump_var.get())
            try:   # the pump controller is opened with the panel's pulses/mm
                self.config_data["dl2_pulses_per_mm"] = float(self.dl2_vars["dl2_pulses_per_mm"].get())
            except (ValueError, KeyError, AttributeError):
                pass
            self._set_status(f"Connecting ({mode}, stage={need_stage}) …")
            self.btn_connect.config(state="disabled")
            self._hw_busy = True   # hold off the monitor while connect reads the devices

            def work():
                try:
                    status = self.hub.connect(self.config_data, mode, need_stage, need_pump)
                except Exception as e:
                    self.out_q.put({"kind": "connect_error", "text": f"{type(e).__name__}: {e}"})
                    return
                lines = []
                if status["bbd"] == "ok":
                    try:
                        lines.append(f"Main delay line ({self.config_data.get('stage_name', '?').split(' (')[0]}):   OK  —  {self.hub.read_position_mm():.4f} mm "
                                     f"({self.hub.read_position_ps():.2f} ps)")
                    except Exception as e:
                        lines.append(f"BBD delay:   OK  (pos read failed: {e})")
                elif status["bbd"].startswith("FAILED"):
                    lines.append(f"Main delay line ({self.config_data.get('stage_name', '?').split(' (')[0]}):   ⚠  {status['bbd']}"
                                 f"   — is serial {self.config_data.get('bbd_serial')} plugged in (USB + power)?")
                lockin_idn = ""
                if status["lockin"] == "ok":
                    try:
                        lockin_idn = self.hub.lockin.identify().strip()
                        lines.append(f"Lock-in:     OK  —  {lockin_idn}")
                    except Exception as e:
                        lines.append(f"Lock-in:     OK  (idn failed: {e})")
                    try:
                        tc = self.hub.lockin.get_time_constant_ms()
                        if tc is not None:
                            self.out_q.put({"kind": "tc", "ms": tc})
                    except Exception:
                        pass
                    try:
                        sv = self.hub.lockin.get_sensitivity_vrms()
                        if sv is not None:
                            self.out_q.put({"kind": "sens", "vrms": sv})
                    except Exception:
                        pass
                elif status["lockin"].startswith("FAILED"):
                    lines.append(f"Lock-in:     ⚠  {status['lockin']}")
                if status["stage"] == "ok":
                    try:
                        xp, zp = self.hub.stage.get_position()
                        lines.append(f"SHOT-602 XZ: OK  —  x={xp} z={zp} pulses")
                    except Exception as e:
                        lines.append(f"SHOT-602 XZ: OK  (read failed: {e})")
                elif status["stage"].startswith("FAILED"):
                    lines.append(f"SHOT-602 XZ: ⚠  {status['stage']}")
                elif not need_stage:
                    lines.append("SHOT-602 XZ: (not requested — tick 'XZ' to include)")
                if status.get("pump") == "ok":
                    try:
                        xp, zp = self.hub.pump.get_position()
                        lines.append(f"Pump SHOT:   OK  —  axis1={xp} axis2={zp} pulses  ({self.config_data.get('pump_port')})")
                    except Exception as e:
                        lines.append(f"Pump SHOT:   OK  (read failed: {e})")
                elif str(status.get("pump", "")).startswith("FAILED"):
                    lines.append(f"Pump SHOT:   ⚠  {status['pump']}   — port {self.config_data.get('pump_port')} right? (Detect hardware lists ports)")
                any_fail = any(str(v).startswith("FAILED") for v in status.values())
                if any_fail:
                    lines += ["", "⚠ Some devices did not connect — click the red light for the reason."]
                usb = _list_delay_lines()           # opens are done -> safe to enumerate now
                if usb is not None and status.get("bbd") == "ok":
                    usb = sorted(set(usb) | {str(self.config_data.get("bbd_serial"))})
                self.out_q.put({"kind": "usb_stages", "serials": usb})
                sysname = am.identify_system({"gpib": [{"idn": lockin_idn}], "serial": [], "kinesis": []})
                self._log("connect: " + " | ".join(lines))
                self.out_q.put({"kind": "connected", "idn": "\n".join(lines),
                                "any_fail": any_fail, "system": sysname, "status": status,
                                "fail_lines": [ln for ln in lines if "⚠" in ln]})

            threading.Thread(target=work, daemon=True).start()

        def _disconnect(self):
            if self.worker and self.worker.is_alive():
                messagebox.showwarning("Busy", "Stop the running scan first.")
                return
            self.hub.close()
            self.btn_connect.config(state="normal")
            self.btn_disconnect.config(state="disabled")
            self.pos_var.set("Main delay: — um")
            self.val_var.set("—")
            self._set_idn("Not connected.")
            self._set_status("Disconnected (BBD released).")

        def _require_conn(self):
            if not self.hub.connected:
                messagebox.showinfo("Not connected", "Click Connect first.")
                return False
            if self.hub.bbd_faulted:
                messagebox.showerror(
                    "BBD state unknown",
                    "The previous delay-line command failed or timed out.\n"
                    "Wait until physical motion has stopped, then Disconnect and Connect again."
                )
                return False
            if self.worker and self.worker.is_alive():
                messagebox.showwarning("Busy", "A scan is already running.")
                return False
            return True

        # -------------------------------------------------- stage control
        def _bbd_go_ps(self, key):
            if not self._require_conn():
                return
            try:
                ps = self._ts_pos_to_ps(float(self.ts_vars[key].get()))
            except ValueError:
                return
            self._run_bg(lambda: self.hub.move_bbd_to_ps(ps), "BBD move")

        def _bbd_go_extreme(self, kind):
            """Move the delay line to the delay where the last scan's signal is max/min."""
            if not self._require_conn():
                return
            if self.worker and self.worker.is_alive():
                self._set_status("Scan running — can't move the delay line now."); return
            if self._ts_t is None or self._ts_y is None:
                messagebox.showinfo("No scan", "Run a timescan first, then move to its peak/trough."); return
            col = {"X": 0, "Y": 1, "R": 2, "theta": 3}.get(self.ts_signal.get(), 0)
            sig = self._ts_y[:, col]
            if not np.isfinite(sig).any():
                return
            idx = int(np.nanargmax(sig) if kind == "max" else np.nanargmin(sig))
            ps = float(self._ts_t[idx])
            self._log(f"_bbd_go_extreme({kind}) idx={idx} ps={ps:.1f} -> _run_bg")
            self._run_bg(lambda: self.hub.move_bbd_to_ps(ps), f"BBD → {kind} signal @ {ps:.1f} ps")

        # ---- mapping delay <-> actual position (um) -------------------------
        def _ps_to_um(self, ps):
            """The delay the app moves in, expressed in ACTUAL stage micrometres."""
            return ps_to_pos_um(ps, float(self.config_data.get("bbd_t0_mm", 0.0)),
                                self._ts_mm_per_ps())

        def _delay_out_of_range(self, ps):
            """The stage can only reach 0..bbd_travel_mm, so most delays are simply unreachable.
            Returns the offending position in um, or None when the delay is fine."""
            um = self._ps_to_um(ps)
            hi = float(self.config_data.get("bbd_travel_mm", 570.0)) * 1000.0
            return um if (um < 0 or um > hi) else None

        def _um_to_ps(self, um):
            return pos_um_to_ps(um, float(self.config_data.get("bbd_t0_mm", 0.0)),
                                self._ts_mm_per_ps())

        def _update_map_delay_um(self):
            """ps box changed -> update the linked um box (guarded so it doesn't ping-pong)."""
            if getattr(self, "_delay_sync", False):
                return
            self._delay_sync = True
            try:
                ps = float(self.map_vars["delay_label_ps"].get())
                um = self._ps_to_um(ps)
                self.map_delay_um.set(f"{um:,.0f}")
                self.map_delay_um_lbl.config(
                    foreground=("red" if self._delay_out_of_range(ps) is not None else ""))
            except (ValueError, TypeError):
                self.map_delay_um.set("")
                self.map_delay_um_lbl.config(foreground="")
            finally:
                self._delay_sync = False

        def _sync_delay_from_um(self):
            """um box edited -> update the linked ps box (guarded)."""
            if getattr(self, "_delay_sync", False):
                return
            self._delay_sync = True
            try:
                um = float(self.map_delay_um.get().replace(",", "").replace("um", "").strip())
                self.map_vars["delay_label_ps"].set(f"{self._um_to_ps(um):.1f}")
                self.map_delay_um_lbl.config(
                    foreground=("red" if (um < 0 or um > float(self.config_data.get("bbd_travel_mm", 570.0)) * 1000.0) else ""))
            except (ValueError, TypeError):
                pass
            finally:
                self._delay_sync = False

        def _map_delay_go(self):
            """Park the delay line at whatever the mapping Delay field says, right now."""
            if not self._require_conn():
                return
            if self.worker and self.worker.is_alive():
                self._set_status("Scan running — can't move the delay line now."); return
            try:
                ps = float(self.map_vars["delay_label_ps"].get())
            except (ValueError, TypeError):
                messagebox.showerror("Bad input", "Delay ps must be a number."); return
            um = self._ps_to_um(ps)
            if self._delay_out_of_range(ps) is not None:
                hi = float(self.config_data.get("bbd_travel_mm", 570.0))
                messagebox.showerror(
                    "Delay out of range",
                    f"{ps:g} ps sits at {um:,.0f} um, and the delay line only reaches "
                    f"0 .. {hi*1000:,.0f} um.\n\n"
                    f"Reachable delays: {self._um_to_ps(hi * 1000):.0f} .. "
                    f"{self._um_to_ps(0.0):.0f} ps  (t0 = {self.config_data.get('bbd_t0_mm', 0):.3f} mm).")
                return
            self._log(f"_map_delay_go ps={ps:.1f} um={um:.0f}")
            self._run_bg(lambda: self.hub.move_bbd_to_ps(ps), f"BBD → {ps:.1f} ps ({um:,.0f} um)")

        def _map_delay_from_current(self):
            """Fill the mapping Delay from where the delay line actually IS — no maths, no sign traps."""
            if not self._require_conn():
                return
            try:
                ps = float(self.hub.read_position_ps())
            except Exception as e:
                self._set_status(f"Can't read the delay line: {type(e).__name__}: {e}"); return
            self.map_vars["delay_label_ps"].set(f"{ps:.1f}")
            self._set_status(f"Mapping delay = the delay line's current position ({self._ps_to_um(ps):,.0f} um)")

        # ---- pick the delay slice by CLICKING the timescan -------------------
        def _on_ts_click(self, event):
            """Click the timescan curve -> that delay becomes the mapping's slice.

            The plot's x is the delay measured from the scan start; the mapping needs the
            app's delay axis (_ts_t). Picking converts between them, so the user never has to.
            """
            if event.inaxes is not self.ts_ax or event.button != 1 or event.xdata is None:
                return
            if self.ts_view.get() != "TDS" or self._ts_t is None or len(self._ts_t) == 0:
                return
            tb = getattr(self.ts_canvas, "toolbar", None)
            if tb is not None and getattr(tb, "mode", ""):
                return                       # zoom/pan is armed — don't hijack the drag
            xdisp = self._ts_xdisp()
            idx = int(np.nanargmin(np.abs(xdisp - float(event.xdata))))
            ps = float(self._ts_t[idx])
            self._ts_pick_ps = ps
            self.map_vars["delay_label_ps"].set(f"{ps:.1f}")
            col = {"X": 0, "Y": 1, "R": 2, "theta": 3}.get(self.ts_signal.get(), 0)
            yv = float(self._ts_y[idx, col]) if self._ts_y is not None else np.nan
            self.ts_pick.set_data([xdisp[idx]], [yv])
            self.ts_canvas.draw_idle()
            self._set_status(f"Slice picked: {xdisp[idx]:.1f} ps on the plot = {self._ps_to_um(ps):,.0f} um"
                             f"  ->  sent to the mapping tab's Delay")

        def _ts_pick_to_3d(self):
            """Append the picked slice to the 3D-delays list — build up "which few slices to map"."""
            if self._ts_pick_ps is None:
                messagebox.showinfo("No slice", "Click a point on the timescan curve first."); return
            cur = [s for s in self.map3d_delays_var.get().replace(",", " ").split() if s]
            cur.append(f"{self._ts_pick_ps:.1f}")
            self.map3d_delays_var.set(", ".join(cur))
            self.map3d_on.set(True)
            self._set_status(f"3D delays: {self.map3d_delays_var.get()}")

        def _ts_t0_from_scan(self):
            """Re-calibrate time zero from the timescan on screen: t0 = the half-rise of the
            ultrafast step (pump and probe overlapping). t0 is a property of the OPTICS, not of
            the scan, but it MOVES whenever the optics do — realignment, or simply inserting a
            filter (1 mm of fused silica shifts it ~1.6 ps). So re-run this after any change.
            """
            if self._ts_t is None or self._ts_y is None:
                messagebox.showinfo("No scan", "Run a timescan through t0 first."); return
            col = {"X": 0, "Y": 1, "R": 2, "theta": 3}.get(self.ts_signal.get(), 0)
            y = self._ts_y[:, col]
            um = self._ps_to_um(self._ts_t)
            o = np.argsort(-um)                      # descending position = increasing delay
            um, y = um[o], y[o]
            ok = np.isfinite(y)
            um, y = um[ok], y[ok]
            if len(y) < 12:
                messagebox.showinfo("Not enough data", "Need a finished timescan."); return
            base, pk = float(np.median(y[:8])), int(np.argmax(np.abs(y)))
            if abs(y[pk] - base) < 5 * (np.std(y[:8]) + 1e-15):
                messagebox.showwarning("No step found",
                    "No ultrafast step stands out above the noise. The scan must cross t0."); return
            half = base + 0.5 * (y[pk] - base)
            i = pk
            while i > 0 and abs(y[i]) > abs(half):
                i -= 1
            t0_um = float(np.interp(half, [y[i], y[i + 1]], [um[i], um[i + 1]]))
            old_mm = float(self.config_data.get("bbd_t0_mm", 0.0))
            shift_ps = (t0_um / 1000.0 - old_mm) / abs(self._ts_mm_per_ps())
            if not messagebox.askyesno("Set time zero",
                    f"Ultrafast step (half-rise) is at {t0_um:,.0f} um.\n\n"
                    f"Current t0: {old_mm * 1000:,.0f} um\n"
                    f"New t0:     {t0_um:,.0f} um   ({shift_ps:+.1f} ps)\n\n"
                    f"Set this as 0 ps? (Reconnect afterwards so the delay line picks it up.)"):
                return
            self.config_data["bbd_t0_mm"] = t0_um / 1000.0
            self.cfg_vars["bbd_t0_mm"].set(f"{t0_um / 1000.0:.4f}")
            self._update_map_delay_um()
            self._redraw_timescan()
            self._set_status(f"t0 = {t0_um:,.0f} um ({shift_ps:+.1f} ps). Reconnect to apply it to the delay line.")

        def _ts_pick_clear(self):
            self._ts_pick_ps = None
            self.map3d_delays_var.set("")
            self.ts_pick.set_data([], [])
            self.ts_canvas.draw_idle()
            self._set_status("Picked slices cleared.")

        def _bbd_home(self):
            if not self._require_conn():
                return
            if not messagebox.askyesno("Home", "Home the delay line? This runs the stage to its reference."):
                return
            self._run_bg(lambda: self.hub.home_bbd(), "BBD home")

        def _bbd_jog(self, sign):
            if not self._require_conn():
                return
            try:
                um = float(self.ts_jog_um.get())
            except ValueError:
                return
            cur = self.hub.read_position_mm()
            self._run_bg(lambda: self.hub.move_bbd_to_mm(cur + sign * um / 1000.0), "BBD jog")

        def _bbd_move_target(self):
            if not self._require_conn():
                return
            try:
                um = float(self.ts_target_um.get())
            except ValueError:
                return
            self._run_bg(lambda: self.hub.move_bbd_to_mm(um / 1000.0), "BBD move-to")

        def _xz_move(self, axis, var):
            if not self._require_conn():
                return
            if self._mapping_block_reason() and not messagebox.askyesno(
                    "XZ stage", "The XZ stage belongs to the Stage A system (ID0), but another delay line is "
                    "selected.\n\nMove the XZ stage anyway?"):
                return
            if not self.hub.has_stage:
                messagebox.showinfo("No stage", "Connect with 'XZ stage' checked.")
                return
            try:
                mm = float(var.get())
            except ValueError:
                return
            self._run_bg(lambda: self.hub.stage.move_abs(axis, mm), "XZ move")

        def _run_bg(self, fn, label):
            self._set_status(f"{label} …")
            self._hw_busy = True   # pause the live monitor so it doesn't talk to the
            #                        BBD at the same time (that desyncs the APT stream)

            def work():
                try:
                    self._log(f"_run_bg '{label}' calling fn()")
                    fn()
                    self._log(f"_run_bg '{label}' fn() returned")
                    self.out_q.put({"kind": "status", "text": f"{label} done."})
                except Exception as e:
                    self._log(f"_run_bg '{label}' EXCEPTION {e}")
                    self.out_q.put({"kind": "status", "text": f"{label} failed: {e}"})
                finally:
                    self._hw_busy = False
                    self.out_q.put({"kind": "refresh_pos"})
            threading.Thread(target=work, daemon=True).start()

        def _have_data(self, scan):
            return (self._map_arr is not None) if scan == "mapping" else (self._ts_t is not None)

        def _write_ts_simple_csv(self, path):
            """Focused 2-column CSV: delay (exactly the plotted x) + selected signal."""
            sig = self.ts_signal.get()
            col = {"X": 0, "Y": 1, "R": 2, "theta": 3}[sig]
            # Take the axis from _ts_xdisp so the file and the plot can never disagree.
            # This used to be a bare abs(t - t[0]), which folded a pre-t0 baseline onto
            # the positive side and silently turned negative delays into a ramp.
            delay = self._ts_xdisp()
            y = self._ts_y[:, col]
            t = self._ts_t
            with open(path, "w", newline="") as f:
                f.write(f"delay_ps,{sig}\n")
                for i in range(len(t)):
                    f.write(f"{delay[i]:.4f},{y[i]:.6e}\n")

        def _write_map_simple_csv(self, path):
            """Focused CSV: the selected-signal intensity grid (rows=X mm, cols=Z mm)."""
            sig = self.map_signal.get()
            col = {"X": 0, "Y": 1, "R": 2, "theta": 3}[sig]
            grid = self._map_arr[:, :, col]
            x, z = self._map_x, self._map_z
            with open(path, "w", newline="") as f:
                f.write(f"# {sig} intensity  |  rows=X(mm), cols=Z(mm)  |  shape {len(x)}x{len(z)}\n")
                f.write("X\\Z," + ",".join(f"{zz:.4f}" for zz in z) + "\n")
                for i in range(len(x)):
                    f.write(f"{x[i]:.4f}," + ",".join(f"{grid[i, j]:.6e}" for j in range(len(z))) + "\n")

        def _save_to_chosen(self, scan):
            """Save-As dialog (user NAMES the file) → focused CSV: timescan = delay +
            amplitude; mapping = intensity grid. Returns True if saved. The full
            multi-channel data is always auto-saved to moke_data separately."""
            stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            default = (f"timescan_{stamp}.csv" if scan != "mapping"
                       else f"mapping_{self.map_signal.get()}_{stamp}.csv")
            path = filedialog.asksaveasfilename(
                title="Save data as…", defaultextension=".csv", initialfile=default,
                filetypes=[("CSV", "*.csv"), ("All files", "*.*")])
            if not path:
                return False
            try:
                if scan == "mapping":
                    self._write_map_simple_csv(path)
                else:
                    self._write_ts_simple_csv(path)
                self._set_status(f"Saved: {path}")
                return True
            except Exception as e:
                messagebox.showerror("Save failed", str(e))
                return False

        def _save_as(self):
            """Manual 'Save last scan as…' — saves the data for the ACTIVE tab."""
            scan = "mapping" if "Mapping" in self._nb.tab(self._nb.select(), "text") else "timescan"
            if not self._have_data(scan):
                messagebox.showinfo("No data", "No data yet — run (or start) a scan first."); return
            self._save_to_chosen(scan)

        def _save_current_tab(self):
            """Save the data of whichever tab is showing — usable any time, not only in the
            one-shot dialog that pops after a scan."""
            scan = "mapping" if "Mapping" in self._nb.tab(self._nb.select(), "text") else "timescan"
            if not self._have_data(scan):
                messagebox.showinfo("No data", f"No {scan} data in memory yet."); return
            self._save_to_chosen(scan)

        def _prompt_save(self, scan):
            """Auto-pop after a scan finishes: name + save the focused data
            (time + amplitude / intensity grid) wherever the user wants. The FULL
            multi-channel data is always auto-saved to moke_data regardless."""
            if scan not in ("timescan", "mapping"):
                return   # 3D stack is auto-saved to moke_data (too big for the quick CSV)
            if not self._have_data(scan):
                return
            if messagebox.askyesno("Save data",
                                    "Scan finished. Save the data (time + amplitude)? "
                                    "You'll name the file & pick the location.\n"
                                    "(The full data is also auto-saved in moke_data.)"):
                self._save_to_chosen(scan)

        def _lockin_pending(self):
            """Which lock-in boxes have been typed into but never pushed to the instrument."""
            pend = []
            if self._tc_dev_ms is not None:
                try:
                    if abs(float(self.tc_var.get()) - self._tc_dev_ms) > 1e-9:
                        pend.append(f"TC: box {self.tc_var.get()} ms, instrument {self._tc_dev_ms:g} ms")
                except (ValueError, TypeError):
                    pass
            if self._sens_dev is not None and self.sens_var.get() != self._sens_dev:
                pend.append(f"Sens: box {self.sens_var.get()}, instrument {self._sens_dev}")
            return pend

        def _check_lockin_applied(self):
            self.lockin_warn.set("← not applied! press Set" if self._lockin_pending() else "")

        def _lockin_gate(self):
            """Refuse to start a scan that would silently run with the OLD time constant.
            (This is the "why didn't the TC change? I never pressed Set" trap — it wastes a whole map.)"""
            pend = self._lockin_pending()
            if not pend:
                return True
            return messagebox.askyesno(
                "Lock-in settings not applied",
                "These boxes were changed but never sent to the lock-in:\n\n  "
                + "\n  ".join(pend)
                + "\n\nThe scan would run with the INSTRUMENT's values, not the boxes'.\n\n"
                  "Press Set first, then start.\n\nStart anyway?")

        def _set_tc(self):
            """Write the lock-in time constant (ms) to the instrument. Valid in the
            full-Python mode (LabVIEW closed, Python owns the lock-in)."""
            if self.worker and self.worker.is_alive():
                messagebox.showwarning("Busy", "A scan is running — stop it before changing "
                                       "the time constant (writing to the lock-in mid-scan collides "
                                       "with the scan's reads)."); return
            if not self.hub.connected:
                messagebox.showinfo("Not connected", "Connect first."); return
            try:
                tc = float(self.tc_var.get())
            except ValueError:
                return

            def apply_tc():
                self.hub.lockin.set_time_constant_ms(tc)
                try:    # read it back — the box must reflect the INSTRUMENT, not what was typed
                    self.out_q.put({"kind": "tc", "ms": float(self.hub.lockin.get_time_constant_ms())})
                except Exception:
                    pass
            self._run_bg(apply_tc, f"Set lock-in time constant = {tc:g} ms")

        def _set_sensitivity(self):
            """Write the lock-in voltage sensitivity (full-scale range) to the instrument."""
            if self.worker and self.worker.is_alive():
                messagebox.showwarning("Busy", "A scan is running — stop it before changing "
                                       "the sensitivity (writing to the lock-in mid-scan collides "
                                       "with the scan's reads)."); return
            if not self.hub.connected:
                messagebox.showinfo("Not connected", "Connect first."); return
            lbl = self.sens_var.get(); vrms = SENS_TO_V.get(lbl)
            if vrms is None:
                return

            def apply_sens():
                self.hub.lockin.set_sensitivity_vrms(vrms)
                try:
                    v = self.hub.lockin.get_sensitivity_vrms()
                    if v:
                        self.out_q.put({"kind": "sens", "vrms": float(v)})
                except Exception:
                    pass
            self._run_bg(apply_sens, f"Set lock-in sensitivity = {lbl}")

        def _refresh_position(self):
            if not self.hub.connected:
                return
            self.out_q.put({"kind": "refresh_pos"})

        def _refresh_xz(self):
            if not (self.hub.connected and self.hub.has_stage):
                return
            try:
                x_mm, z_mm = self.hub.stage_position_mm(self.config_data["pulses_per_mm"])
                self.map_xz_pos.set(f"XZ: x={x_mm:.3f}  z={z_mm:.3f} mm")
            except Exception as e:
                self.map_xz_pos.set(f"XZ: (read failed: {e})")

        # -------------------------------------------------- start scans
        def _ts_plan_or_error(self):
            """Validate the Scan boxes -> (t0_mm, start_ps, end_ps, step_ps), or None
            after saying what is wrong. Applies the calibration as a side effect.

            Shared by Set and Start so the two can never disagree about where the scan
            begins or whether it fits on the stage."""
            try:
                self._pull(self.ts_vars)
            except ValueError as e:
                messagebox.showerror("Bad input", str(e)); return None
            # µm(position) range -> ps, anchoring delay 0 to the Start box ("move the delay line
            # to the Start position and call that 0 ps"). No-op when the unit is already ps.
            t0_mm, sp, ep, stp = plan_timescan(
                self.config_data, self.ts_unit.get(),
                self.config_data["time_start_ps"], self.config_data["time_end_ps"],
                self.config_data["time_step_ps"])
            if not stp:
                messagebox.showerror("Bad input", "Step must not be 0."); return None
            self.config_data["bbd_t0_mm"] = t0_mm
            # the delay line converts ps->mm itself; hand it the same t0 AND the same
            # sign, or it sweeps the opposite way (the 2026-07-27 '549390 -> 600000' bug)
            self.hub.sync_bbd_calibration(self.config_data)
            off = timescan_out_of_travel(self.config_data, sp, ep)
            if off is not None:
                hi_um = float(self.config_data.get("bbd_travel_mm", 570.0)) * 1000.0
                messagebox.showerror(
                    "Scan leaves the stage travel",
                    f"This sweep asks the delay line for {off[0]:,.0f} .. {off[1]:,.0f} um, "
                    f"but it only reaches 0 .. {hi_um:,.0f} um.\n\n"
                    f"Nothing has moved. Fix the Start/End boxes (or 'BBD travel' in "
                    f"Settings if the stage really is longer).")
                return None
            return t0_mm, sp, ep, stp

        def _ts_set(self):
            """Apply the Scan boxes AND park the delay line on Start, in one press.

            Requested: a Set button for the scan fields, so the hardware follows them.
            Typing alone never moves anything, so before this the boxes and the hardware
            could disagree with no way to tell. Set makes them agree on demand: after it
            finishes the readout sits at 0.00 ps, which is also where Start would begin."""
            if not self._require_conn():
                return
            if self.worker and self.worker.is_alive():
                self._set_status("Scan running — can't move the delay line now."); return
            plan = self._ts_plan_or_error()
            if plan is None:
                return
            t0_mm, sp, _ep, _stp = plan
            um = ps_to_pos_um(sp, t0_mm, mm_per_ps_for(self.config_data))
            self._log(f"Set: park on Scan Start {um:,.0f} um ({sp:.2f} ps)")
            self._run_bg(lambda: self.hub.move_bbd_to_ps(sp), f"Set → Scan Start ({um:,.0f} um)")

        def _start_timescan(self):
            if not self._require_conn():
                return
            if self.hub.bbd is None or self.hub.lockin is None:
                messagebox.showerror("Devices not ready",
                    "Timescan needs the delay line + lock-in connected.\nSee the Settings / Devices tab."); return
            if not self._lockin_gate():
                return
            try:
                self._pull(self.ts_vars)
            except ValueError as e:
                messagebox.showerror("Bad input", str(e)); return
            plan = self._ts_plan_or_error()
            if plan is None:
                return
            t0_mm, sp, ep, stp = plan
            self.config_data["time_start_ps"] = sp
            self.config_data["time_end_ps"] = ep
            self.config_data["time_step_ps"] = stp
            self.config_data["signal"] = self.ts_signal.get()
            self.config_data["dl2_enabled"] = False
            if self.dl2_on.get():
                try:
                    for k, v in self.dl2_vars.items():
                        x = float(v.get().strip())
                        self.config_data[k] = int(x) if k in ("dl2_axis", "dl2_pulses_per_mm") else x
                except ValueError as e:
                    messagebox.showerror("Bad input", f"Pump delay: {e}"); return
                c = self.config_data
                if not self.hub.has_pump:
                    messagebox.showerror("Pump delay", "The pump delay is an axis of the SHOT controller.\n"
                                         "tick 'Pump' and Connect first."); return
                if c["dl2_axis"] not in (1, 2) or c["dl2_pulses_per_mm"] <= 0:
                    messagebox.showerror("Pump delay", "Axis must be 1 or 2 and Pulses/mm > 0."); return
                if sp == ep and (c["dl2_start_um"] == c["dl2_end_um"] or c["dl2_step_um"] <= 0):
                    messagebox.showerror("Pump delay", "The main delay line is fixed (Start = End), so the pump "
                                         "delay must have Start ≠ End and Step > 0."); return
                err = self._pump_limit_error([c["dl2_start_um"], c["dl2_end_um"]])
                if err:
                    messagebox.showerror("Pump delay", err); return
                if abs(c["dl2_end_um"] - c["dl2_start_um"]) > 300000:
                    messagebox.showerror("Pump delay", "Pump delay range > 300 mm — check the numbers."); return
                c["dl2_enabled"] = True
            self._log(f"timescan plan: unit={self.ts_unit.get()} t0={t0_mm:.4f} mm  "
                      f"{sp:.2f} -> {ep:.2f} ps step {stp:.4f}  "
                      f"({ps_to_pos_um(sp, t0_mm, mm_per_ps_for(self.config_data)):,.0f} -> "
                      f"{ps_to_pos_um(ep, t0_mm, mm_per_ps_for(self.config_data)):,.0f} um)")
            try:
                rep = max(1, int(float(self.ts_vars["repetition"].get())))
            except ValueError:
                rep = 1
            self._save_settings()   # persist params on every run (crash insurance)
            after_map = {"Pre-scan pos": "prescan", "Stay": "stay",
                         "Max Position": "max", "Min Position": "min"}
            after_scan = after_map.get(self.ts_after.get(), "prescan")
            self.stop_event.clear()
            self.pause_event.clear()
            self._scan_t0 = time.time()
            self._ts_t, self._ts_y = None, None
            self._ts_pick_ps = None; self.ts_pick.set_data([], [])   # the old pick is stale now
            self.ts_start.config(state="disabled"); self.ts_stop.config(state="normal")
            self.ts_pause.config(state="normal", text="Pause")
            self._lock_ui(True)
            cfg = copy.deepcopy(self.config_data)

            def work():
                timescan_loop(cfg, self.hub, self.out_q, self.stop_event,
                              repetition=rep, after_scan=after_scan,
                              pause_event=self.pause_event, trace=self._log)
            self.worker = threading.Thread(target=work, daemon=True)
            self.worker.start()

        def _mapping_block_reason(self):
            """2D mapping drives the XZ stage of the Stage A system (ID0). With another delay line
            selected, the SHOT controller may be wired to the pump delay instead."""
            if str(self.config_data.get("bbd_serial")) != "10000000":
                return ("2D mapping belongs to the Stage A system (ID0): it moves the XZ stage and "
                        "that system's delay line.\n\nSelected now: " + str(self.config_data.get("stage_name")) +
                        "\n\nSwitch 'Delay line' to ID0 Stage A first.")
            return None

        def _start_mapping(self):
            why = self._mapping_block_reason()
            if why:
                messagebox.showerror("2D mapping", why); return
            if not self._require_conn():
                return
            if not self.hub.has_stage:
                messagebox.showinfo("No stage", "Connect with 'XZ stage' checked for mapping.")
                return
            if not self._lockin_gate():
                return
            if self.hub.lockin is None or self.hub.bbd is None:
                messagebox.showerror("Devices not ready",
                    "Mapping needs the lock-in + delay line connected.\nSee the Settings / Devices tab."); return
            try:
                for k in ("x_step", "z_step", "delay_label_ps", "num_averages", "settling_time"):
                    self._set_cfg(self.map_vars[k], k)
                xs, xe, xstep, zs, ze, zstep = self._map_bounds()
                self.config_data.update(x_start=xs, x_end=xe, x_step=xstep,
                                        z_start=zs, z_end=ze, z_step=zstep)
            except (ValueError, ZeroDivisionError) as e:
                messagebox.showerror("Bad input", str(e)); return
            self.config_data["signal"] = self.map_signal.get()
            self.config_data["map_after_scan"] = "start" if self.map_after.get() == "Return to start" else "stay"
            delays = None
            if self.map3d_on.get():
                try:
                    delays = [float(x) for x in self.map3d_delays_var.get().replace(",", " ").split()]
                except ValueError:
                    messagebox.showerror("Bad input", "3D delays must be numbers, e.g. 0, 100, 500"); return
                if not delays:
                    messagebox.showerror("Bad input", "Enter at least one delay for 3D."); return
            self._save_settings()   # persist params on every run (crash insurance)
            self.stop_event.clear()
            self.pause_event.clear()
            self._scan_t0 = time.time()
            self.map_start.config(state="disabled"); self.map_stop.config(state="normal")
            self.map_pause.config(state="normal", text="Pause")
            self._lock_ui(True)
            cfg = copy.deepcopy(self.config_data)

            if delays is not None:
                def work():
                    mapping3d_loop(cfg, self.hub, self.out_q, self.stop_event, delays, pause_event=self.pause_event)
            else:
                def work():
                    mapping_loop(cfg, self.hub, self.out_q, self.stop_event, pause_event=self.pause_event)
            self.worker = threading.Thread(target=work, daemon=True)
            self.worker.start()

        def _stop_scan(self):
            self.stop_event.set()
            self.pause_event.clear()   # unblock a paused scan so it can stop
            self._set_status("Stopping after current point …")

        def _toggle_pause(self):
            if self.pause_event.is_set():
                self.pause_event.clear(); self._set_pause_btn("Pause"); self._set_status("Resumed.")
            else:
                self.pause_event.set(); self._set_pause_btn("Resume")
                self._set_status("Paused — finishes the current point, then holds.")

        def _set_pause_btn(self, text):
            self.ts_pause.config(text=text); self.map_pause.config(text=text)

        def _lock_ui(self, running):
            self.btn_connect.config(state="disabled" if running else ("disabled" if self.hub.connected else "normal"))
            self.btn_disconnect.config(state="disabled" if running else ("normal" if self.hub.connected else "disabled"))
            # lock-in TC write must not happen mid-scan (GPIB collision) — grey it out
            self.btn_set_tc.config(state="disabled" if running else "normal")
            self.btn_set_sens.config(state="disabled" if running else "normal")
            # moving the delay line mid-scan would wreck the scan — same rule as click-to-move
            self.map_delay_go.config(state="disabled" if running else "normal")
            self.map_delay_now.config(state="disabled" if running else "normal")
            self.ts_set_btn.config(state="disabled" if running else "normal")

        # -------------------------------------------------- queue poll
        def _poll_queue(self):
            # Drain at most N messages per cycle, then hand control back to Tk so it can
            # repaint + pump Windows messages. Draining the WHOLE queue in one go on a fast
            # XZ scan starved the event loop -> the window froze ("not responding") until Stop.
            n = 0
            try:
                while n < 50:
                    self._handle(self.out_q.get_nowait())
                    n += 1
            except queue.Empty:
                pass
            try:
                self._sync_conn_ui()
            except Exception:
                pass
            self.after(30 if n >= 50 else 120, self._poll_queue)   # poll faster when backlogged

        def _handle(self, m):
            kind = m["kind"]
            if kind in ("connected", "connect_error"):
                self._connecting = False
            if kind == "connected":
                self._hw_busy = False
                self._conn_status = m.get("status", {}) or {}
                bad = [k for k, v in self._conn_status.items() if str(v).startswith("FAILED")]
                if bad:
                    _lbl = {"bbd": "Delay (" + str(self.config_data.get("stage_name", "?")).split(" (")[0] + ")",
                            "lockin": "Lock-in", "stage": "XZ stage", "pump": "Pump delay"}
                    txt = "\n\n".join(f"• {_lbl.get(k, k)}: {self._plain_error(self._conn_status[k])}\n"
                                       f"   {self._conn_hint(k)}" for k in bad)
                    self.after(50, lambda t=txt: messagebox.showwarning("Some devices did not connect", t))
                self._set_idn(m["idn"])
                if m.get("system"):
                    self.system_var.set(f"System: {m['system']}")
                if self.hub.connected:
                    self.btn_connect.config(state="disabled")
                    self.btn_disconnect.config(state="normal")
                    if m.get("any_fail"):
                        self._set_status("Connected with warnings — see Settings / Devices tab.")
                    else:
                        self._set_status(f"Connected ({self.mode_var.get()}).")
                    self.out_q.put({"kind": "refresh_pos"})
                else:
                    self.btn_connect.config(state="normal")
                    self._set_status("Connect failed — see Settings / Devices tab.")
            elif kind == "connect_error":
                self._hw_busy = False
                self.btn_connect.config(state="normal")
                self._set_status("Connect failed.")
                messagebox.showerror("Connect failed", m["text"])
            elif kind == "status":
                self._set_status(m["text"])
            elif kind == "sens":
                self._sens_dev = _nearest_sens_label(m["vrms"])   # what the LOCK-IN actually holds
                self.sens_var.set(self._sens_dev)
                self._check_lockin_applied()
            elif kind == "tc":
                self._tc_dev_ms = float(m["ms"])                  # what the LOCK-IN actually holds
                self.tc_var.set(f"{m['ms']:g}")
                self._check_lockin_applied()
            elif kind == "hardware":
                self._show_hardware(m["det"])
                self._scan_usb_bg()
            elif kind == "bbd_swapped":
                self._connecting = False
                self._hw_busy = False
                self._conn_status["bbd"] = m["status"]
                if m["status"] == "ok":
                    self._set_status(f"Delay line → {m['name']}: connected ({m['pos']}). Lock-in / XZ / pump unchanged.")
                    self.out_q.put({"kind": "refresh_pos"})
                else:
                    self._set_status(f"Delay line → {m['name']}: FAILED — click the red 'Delay' light.")
                    _t = self._plain_error(m["status"]) + "\n\n" + self._conn_hint("bbd")
                    self.after(50, lambda: messagebox.showwarning("Delay line did not connect", _t))
            elif kind == "usb_stages":
                self._usb_serials = m["serials"]
                self._show_usb_state()
            elif kind == "refresh_pos":
                self._do_refresh_pos()
            elif kind == "begin_timescan":
                self._ts_t = np.array(m["t_ps"], float)
                self._ts_y = np.full((len(self._ts_t), 4), np.nan)
                self.ts_prog.config(maximum=len(self._ts_t), value=0)
                self._redraw_timescan()
            elif kind == "point_timescan":
                i = m["i"]
                self._ts_y[i] = [m["X"], m["Y"], m["R"], m["theta"]]
                self.ts_prog.config(value=i + 1)
                self._update_value(m)
                if m.get("pos_mm") is not None:
                    self._last_pos_mm = m["pos_mm"]
                    self._relabel_position()
                self._update_eta(self.ts_time_var, m["i"] + 1, len(self._ts_t) * m["reps"]
                                 if m["reps"] > 1 else len(self._ts_t),
                                 done_offset=(m["rep"] - 1) * len(self._ts_t))
                self._redraw_timescan()
            elif kind == "begin_mapping":
                self._map_x = np.array(m["x_mm"], float)
                self._map_z = np.array(m["z_mm"], float)
                self._map_arr = np.full((len(self._map_x), len(self._map_z), 4), np.nan)
                self._map_delay = float(m["delay_ps"])
                self.map_prog.config(maximum=len(self._map_x) * len(self._map_z), value=0)
                self._init_map_image(m["delay_ps"])
            elif kind == "point_mapping":
                self._map_arr[m["i_x"], m["i_z"]] = [m["X"], m["Y"], m["R"], m["theta"]]
                self.map_prog.config(value=m["count"])
                self._update_value(m)
                self.map_xz_pos.set(f"XZ scanning: x={m['x_mm']:.3f}  z={m['z_mm']:.3f} mm")
                if getattr(self, "map_cursor", None) is not None:
                    self.map_cursor.set_data([m["x_mm"]], [m["z_mm"]])
                self._update_eta(self.map_time_var, m["count"], m["total"])
                self._redraw_mapping()
            elif kind == "begin_map3d":
                self._map3d_x = np.array(m["x_mm"], float)
                self._map3d_z = np.array(m["z_mm"], float)
                self._map3d_delays = list(m["delays"])
                self._map3d = np.full((len(self._map3d_delays), len(self._map3d_x),
                                       len(self._map3d_z), 4), np.nan)
                self.map_prog.config(maximum=len(self._map3d_delays) * len(self._map3d_x) * len(self._map3d_z), value=0)
                self._set_map3d_view_options()
                self._map_x, self._map_z = self._map3d_x, self._map3d_z
                self._map_arr = self._map3d[0]
                self._map_delay = self._map3d_delays[0]
                self._map3d_shown_di = 0
                self._init_map_image(self._map3d_delays[0])
            elif kind == "point_map3d":
                di = m["di"]
                self._map3d[di, m["i_x"], m["i_z"]] = [m["X"], m["Y"], m["R"], m["theta"]]
                self.map_prog.config(value=m["count"])
                self._update_value(m)
                self.map_xz_pos.set(f"3D delay {m['delay']:g}ps  x={m['x_mm']:.3f} z={m['z_mm']:.3f} mm")
                self._map_arr = self._map3d[di]
                if getattr(self, "_map3d_shown_di", None) != di:   # new delay slice
                    self._map3d_shown_di = di
                    self.map3d_view.set(f"{m['delay']:g}")
                    self._map_delay = m["delay"]
                    self._init_map_image(m["delay"])
                if self.map_cursor is not None:
                    self.map_cursor.set_data([m["x_mm"]], [m["z_mm"]])
                self._update_eta(self.map_time_var, m["count"], m["total"])
                self._redraw_mapping()
            elif kind == "done":
                self._scan_finished(m)
            elif kind == "error":
                self._scan_finished(m)
                messagebox.showerror("Scan error", m["text"])

        def _scan_finished(self, m):
            self._redraw_mapping(force=True)   # the live redraw is throttled — make the last one land
            self.ts_start.config(state="normal"); self.ts_stop.config(state="disabled")
            self.map_start.config(state="normal"); self.map_stop.config(state="disabled")
            self.ts_pause.config(state="disabled", text="Pause")
            self.map_pause.config(state="disabled", text="Pause")
            self.pause_event.clear()
            self._lock_ui(False)
            if m.get("kind") == "error":
                self._set_status(f"Scan error: {m['text']}")
                return
            tag = "stopped" if m.get("stopped") else "done"
            where = m.get("dir") or "(not saved)"
            if m.get("scan") == "mapping" and m.get("shape"):
                size = f" [{m['shape'][0]}×{m['shape'][1]} = {m['shape'][0]*m['shape'][1]} pts]"
            elif m.get("scan") == "timescan" and m.get("npts"):
                size = f" [{m['npts']} pts]"
            else:
                size = ""
            self._set_status(f"Scan {tag}.{size}  Saved: {where}")
            self._prompt_save(m.get("scan"))   # auto-pop a save dialog for a user-chosen location

        # -------------------------------------------------- live monitor
        def _poll_monitor(self):
            if (self.monitor_on.get() and self.hub.connected and not self._hw_busy
                    and not (self.worker and self.worker.is_alive())):
                self.out_q.put({"kind": "refresh_pos"})
            # refresh rate tracks the lock-in time constant (no point reading faster
            # than the lock-in updates); clamped to 150-1000 ms.
            try:
                tc = float(self.tc_var.get())
            except (ValueError, AttributeError):
                tc = 300.0
            self.after(int(min(1000.0, max(150.0, tc))), self._poll_monitor)

        def _do_refresh_pos(self):
            # A refresh already queued before Start must not race the new scan's
            # first BBD command or lock-in read.
            if (not self.hub.connected or self._hw_busy
                    or (self.worker and self.worker.is_alive())):
                return
            try:
                self._last_pos_mm = self.hub.read_position_mm()
                self._relabel_position()
            except Exception:
                pass
            r = self.hub.read_lockin()
            if r is not None:
                self._update_value({"X": r["X"], "Y": r["Y"], "R": r["R"], "theta": r["theta"]})

        def _update_value(self, m):
            sig = self.val_sig_var.get()
            v = m.get(sig)
            if v is None:
                return
            self.val_var.set(f"{v: .5e}" if sig != "theta" else f"{v: .2f}°")

        # -------------------------------------------------- plotting
        def _auto_phase(self, X, Y):
            """Best lock-in phase φ (rad) to rotate the recorded X,Y so the signal lands in X'.
            = the weighted principal axis of the X-Y scatter (the signal dominates the variance).
            Removes the need to physically re-tune the lock-in phase each measurement."""
            m = np.isfinite(X) & np.isfinite(Y)
            if m.sum() < 4:
                return 0.0
            Xr, Yr = X[m], Y[m]
            w = np.hypot(Xr, Yr)                       # weight toward the signal (large |R|) region
            sxx = float(np.sum(w * Xr * Xr)); syy = float(np.sum(w * Yr * Yr))
            sxy = float(np.sum(w * Xr * Yr))
            phi = 0.5 * np.arctan2(2 * sxy, sxx - syy)
            Xp = Xr * np.cos(phi) + Yr * np.sin(phi)   # sign convention: biggest feature positive
            if Xp[int(np.argmax(np.abs(Xp)))] < 0:
                phi += np.pi
            return phi

        def _redraw_timescan(self):
            if self._ts_t is None:
                return
            COL = {"X": 0, "Y": 1, "R": 2, "theta": 3}
            sig = self.ts_signal.get()
            fft = self.ts_view.get() == "FFT"
            # which channels to draw: FFT shows only the primary channel; TDS overlays the checked ones
            chans = [sig] if fft else [c for c in ("X", "Y", "R", "theta", "auto")
                                       if self.ts_show.get(c) and self.ts_show[c].get()]
            if not chans:
                chans = [sig]
            any_mask = None
            for ch, line in self.ts_lines.items():
                if ch not in chans:
                    line.set_data([], []); continue
                if ch == "auto":     # software lock-in phase: rotate X,Y so the full signal lands in X'
                    Xc, Yc = self._ts_y[:, 0], self._ts_y[:, 1]
                    self._last_phi = self._auto_phase(Xc, Yc)
                    y = Xc * np.cos(self._last_phi) + Yc * np.sin(self._last_phi)
                else:
                    y = self._ts_y[:, COL[ch]]
                mask = ~np.isnan(y)
                any_mask = mask if any_mask is None else (any_mask | mask)
                if fft and mask.sum() >= 4:
                    t = self._ts_t[mask]; yv = y[mask] - np.nanmean(y[mask])
                    dt = abs(np.mean(np.diff(t))) if len(t) > 1 else 1.0
                    line.set_data(np.fft.rfftfreq(len(yv), d=dt), np.abs(np.fft.rfft(yv)))
                else:
                    line.set_data(self._ts_xdisp()[mask], y[mask])
            if fft:
                self.ts_ax.set_xlabel("Frequency (THz)"); self.ts_ax.set_ylabel(f"|FFT {sig}|")
            else:
                self.ts_ax.set_xlabel("Delay (ps)")
                self.ts_ax.set_ylabel("Amplitude " + ("/".join(chans) if len(chans) > 1 else chans[0]) + " (V)")
            # legend only when overlaying more than one
            leg = self.ts_ax.get_legend()
            if len(chans) > 1 and not fft:
                self.ts_ax.legend(loc="upper right", fontsize=8, framealpha=0.6)
            elif leg is not None:
                leg.remove()
            self.ts_ax.relim(); self.ts_ax.autoscale_view()
            if not fft:
                # 0 pinned to the LEFT edge; the right edge follows the data so the trace
                # still unrolls gradually as points come in (as before).
                # Only the left changed — it used to be min-pad, and with no data
                # or a single point min == max == 0, which put 0 in the middle.
                hi = 0.0
                if any_mask is not None and any_mask.any():
                    hi = float(self._ts_xdisp()[any_mask].max())
                self.ts_ax.set_xlim(0.0, hi + max(hi * 0.02, 0.5))
            self.ts_canvas.draw_idle()

        def _reset_ts_axis(self):
            """With no scan loaded, keep 0 at the left instead of matplotlib's default
            empty-axes range (-0.05 .. 0.05), which draws 0 in the middle."""
            if not hasattr(self, "ts_ax") or getattr(self, "ts_view", None) is None:
                return
            if self.ts_view.get() == "FFT" or self._ts_t is not None:
                return
            self.ts_ax.set_xlim(0.0, 1.0)
            self.ts_canvas.draw_idle()

        def _ts_xdisp(self):
            """The timescan's x axis, in ps: how far the scan has run, from its own start.

            Same rule as the position readout — starts at 0, only grows (it can never be
            negative). One unconditional rule, so the axis can't depend on
            a calibration flag and drift away from what the readout says."""
            return np.abs(self._ts_t - float(self._ts_t[0]))

        def _init_map_image(self, delay_ps):
            self._cbar_synced = False   # new map -> rebuild the colourbar once on the first redraw
            self.map_ax.clear()
            self.map_ax.set_facecolor("black")
            # Mapping convention : horizontal = X, vertical = Z, colour = intensity.
            # data is [i_x, i_z]; imshow rows->y, cols->x, so transpose to put X on x-axis.
            arr = self._map_signal_arr()
            extent = [self._map_x.min(), self._map_x.max(), self._map_z.min(), self._map_z.max()]
            self.map_im = self.map_ax.imshow(arr.T, origin="lower", aspect="auto",
                                             extent=extent, cmap=self.map_cmap.get(),
                                             interpolation="nearest")
            self.map_ax.set_xlabel("X (mm)"); self.map_ax.set_ylabel("Z (mm)")
            self.map_ax.set_title(f"XZ map @ delay = {delay_ps} ps")
            if self.map_flip_x.get():
                self.map_ax.invert_xaxis()
            if self.map_flip_z.get():
                self.map_ax.invert_yaxis()
            # live "currently scanning" pixel marker (X horizontal, Z vertical)
            self.map_cursor, = self.map_ax.plot([], [], marker="+", ms=13, mec="lime",
                                                mew=1.6, ls="none")
            if self.map_cbar is None:
                self.map_cbar = self.map_fig.colorbar(self.map_im, ax=self.map_ax)
            else:
                self.map_cbar.update_normal(self.map_im)
            self.map_canvas.draw_idle()

        def _map_signal_arr(self):
            col = {"X": 0, "Y": 1, "R": 2, "theta": 3}[self.map_signal.get()]
            return self._map_arr[:, :, col]

        def _change_cmap(self):
            if self.map_im is not None:
                self.map_im.set_cmap(self.map_cmap.get())
                self.map_canvas.draw_idle()

        def _reset_ts_view(self):
            """Undo any zoom/pan on the timescan plot — refit to the data."""
            self.ts_ax.relim(); self.ts_ax.autoscale(enable=True)
            self._redraw_timescan()
            self.ts_canvas.draw_idle()

        def _goto_xz_then_timescan(self):
            """Park the stage on the X,Z in the boxes (click the map to fill them), then hand
            over to the Timescan tab — 'interesting pixel on the map -> its full delay trace'."""
            if not self._require_conn():
                return
            if not self.hub.has_stage:
                messagebox.showinfo("No stage", "Connect with 'XZ stage' checked first."); return
            if self.worker and self.worker.is_alive():
                messagebox.showwarning("Busy", "A scan is running — can't move the stage now."); return
            try:
                x_mm, z_mm = float(self.map_x_to.get()), float(self.map_z_to.get())
            except (ValueError, TypeError):
                messagebox.showerror("Bad input", "X and Z must be numbers (mm)."); return
            xa, za = self.config_data["x_axis"], self.config_data["z_axis"]
            self._run_bg(lambda: (self.hub.stage.move_abs(xa, x_mm), self.hub.stage.move_abs(za, z_mm)),
                         f"Move stage to X={x_mm:.3f} Z={z_mm:.3f}")
            self._nb.select(0)                      # Timescan tab — press Start when the move lands
            self._set_status(f"Stage → X={x_mm:.3f} Z={z_mm:.3f} mm. "
                             f"When the move finishes, press Start for the timescan at this point.")

        def _on_map_click(self, event):
            """Click-to-move: send the XZ stage to the clicked (X, Z). Only when the
            'Click map → move' toggle is on, NOT scanning, stage connected, toolbar idle."""
            if not self.map_click_move.get():
                return
            if event.inaxes is not self.map_ax or event.xdata is None or event.ydata is None:
                return
            if getattr(self.map_toolbar, "mode", ""):   # pan/zoom is active — ignore
                return
            if self.worker and self.worker.is_alive():
                messagebox.showwarning("Busy", "A scan is running — can't move the stage now."); return
            if not (self.hub.connected and self.hub.has_stage):
                messagebox.showinfo("No stage", "Connect with 'XZ stage' checked first."); return
            x_mm, z_mm = float(event.xdata), float(event.ydata)   # X horizontal, Z vertical
            self.map_x_to.set(f"{x_mm:.3f}")   # mirror the click into the X,Z boxes, so the
            self.map_z_to.set(f"{z_mm:.3f}")   # point can be re-used (e.g. "Timescan there")
            xa, za = self.config_data["x_axis"], self.config_data["z_axis"]
            self._run_bg(lambda: (self.hub.stage.move_abs(xa, x_mm), self.hub.stage.move_abs(za, z_mm)),
                         f"Move stage to X={x_mm:.3f} Z={z_mm:.3f}")

        def _enable_roi(self):
            if self._map_arr is None:
                messagebox.showinfo("No map", "Run a map first, then drag a box to pick a sub-area."); return
            self._set_status("Drag a rectangle on the map to set the scan area…")
            if self._roi_selector is None:
                self._roi_selector = RectangleSelector(
                    self.map_ax, self._on_roi_select, useblit=True, button=[1], interactive=False)
            self._roi_selector.set_active(True)

        def _on_roi_select(self, eclick, erelease):
            if None in (eclick.xdata, erelease.xdata, eclick.ydata, erelease.ydata):
                return
            x0, x1 = sorted((float(eclick.xdata), float(erelease.xdata)))   # X horizontal
            z0, z1 = sorted((float(eclick.ydata), float(erelease.ydata)))   # Z vertical
            self.map_mode.set("Start/End")
            self.map_vars["x_start"].set(f"{x0:.3f}"); self.map_vars["x_end"].set(f"{x1:.3f}")
            self.map_vars["z_start"].set(f"{z0:.3f}"); self.map_vars["z_end"].set(f"{z1:.3f}")
            self._update_map_estimate()
            self._set_status(f"Scan area → X {x0:.3f}–{x1:.3f}, Z {z0:.3f}–{z1:.3f} mm. "
                             "Set a finer step and press Start.")
            if self._roi_selector is not None:
                self._roi_selector.set_active(False)

        def _set_map3d_view_options(self):
            menu = self.map3d_view_menu["menu"]
            menu.delete(0, "end")
            for d in self._map3d_delays:
                lbl = f"{d:g}"
                menu.add_command(label=lbl, command=lambda v=lbl: (self.map3d_view.set(v), self._on_map3d_view()))
            if self._map3d_delays:
                self.map3d_view.set(f"{self._map3d_delays[0]:g}")

        def _map3d_view_index(self):
            try:
                target = float(self.map3d_view.get())
                for i, d in enumerate(self._map3d_delays):
                    if abs(d - target) < 1e-6:
                        return i
            except (ValueError, AttributeError):
                pass
            return 0

        def _on_map3d_view(self):
            """Show the XZ map for the delay picked in the 'View delay' dropdown."""
            if self._map3d is None:
                return
            di = self._map3d_view_index()
            self._map_arr = self._map3d[di]
            self._map3d_shown_di = di
            self._map_delay = self._map3d_delays[di]
            self._init_map_image(self._map3d_delays[di])
            self._redraw_mapping()

        def _flip_changed(self):
            """Re-apply the display flip (X/Z) after a toggle."""
            if self._map_arr is not None and self.map_im is not None:
                self._init_map_image(self._map_delay)
                self._redraw_mapping()

        def _reset_map_view(self):
            """Undo any zoom/pan on the map — refit to the full XZ extent (respecting flips)."""
            if self._map_x is not None and self._map_z is not None:
                self.map_ax.set_xlim(self._map_x.min(), self._map_x.max())
                self.map_ax.set_ylim(self._map_z.min(), self._map_z.max())
                if self.map_flip_x.get():
                    self.map_ax.invert_xaxis()
                if self.map_flip_z.get():
                    self.map_ax.invert_yaxis()
            self._redraw_mapping()
            self.map_canvas.draw_idle()

        def _clim_from_data(self):
            arr = self._map_signal_arr() if self._map_arr is not None else None
            if arr is not None and np.isfinite(arr).any():
                self.map_clim_lo.set(f"{np.nanmin(arr):.3e}")
                self.map_clim_hi.set(f"{np.nanmax(arr):.3e}")
                self.map_clim_lock.set(True)
                self._redraw_mapping()

        def _redraw_mapping(self, force=False):
            if self._map_arr is None or self.map_im is None:
                return
            # Throttle the live redraw — redrawing on every point flickered and ate CPU.
            # The scan's final redraw passes force=True, so the finished map always lands.
            # Live redraws are throttled to ~1 s (a big 36x31 map redrawn every 0.3 s starved the
            # UI). The scan's final redraw passes force=True so the finished map is always full.
            now = time.time()
            if not force and (now - self._map_last_draw) < 1.0:
                return
            self._map_last_draw = now

            arr = self._map_signal_arr()
            self.map_im.set_data(arr.T)   # X on horizontal, Z on vertical
            if self.map_clim_lock.get():
                try:
                    self.map_im.set_clim(float(self.map_clim_lo.get()), float(self.map_clim_hi.get()))
                except (ValueError, TypeError):
                    pass
            elif np.isfinite(arr).any():
                # ROBUST auto-scale: 2-98 percentile, not min/max. A couple of noisy outlier
                # pixels used to grab both extremes of the colourmap and wash the structure out;
                # clipping to percentiles lets the bulk of the signal use the full colour range.
                lo, hi = (float(v) for v in np.nanpercentile(arr, [2, 98]))
                # Early in a scan only a few near-identical pixels exist -> lo~=hi -> a ~1e-12
                # span that matplotlib renders as an ugly "1e-12 + offset" bar. Floor the span to
                # something sensible (5% of the value, or a tiny absolute) so the bar stays readable.
                c = 0.5 * (lo + hi)
                floor = max(abs(c) * 0.05, 1e-9)
                if (hi - lo) < floor:
                    lo, hi = c - floor, c + floor
                self.map_im.set_clim(lo, hi)
            # Rebuilding the whole COLOURBAR (update_normal) every frame is expensive and was a big
            # part of the freeze. set_clim already keeps the bar's scale in sync, so only do the full
            # rebuild ONCE per map (fixes the born-on-all-NaN ±0.1 stale bar) and on the final redraw.
            if force or not getattr(self, "_cbar_synced", False):
                try:
                    self.map_cbar.update_normal(self.map_im)
                    self._cbar_synced = True
                except Exception:
                    pass
            self.map_cbar.set_label(f"{self.map_signal.get()}")
            self.map_canvas.draw_idle()

        # -------------------------------------------------- misc UI
        def _map_bounds(self):
            """Effective (x_start,x_end,x_step,z_start,z_end,z_step) for either mode."""
            xstep = float(self.map_vars["x_step"].get())
            zstep = float(self.map_vars["z_step"].get())
            if self.map_mode.get() == "Center+Size":
                xc = float(self.map_cs["x_center"].get()); xsz = float(self.map_cs["x_size"].get())
                zc = float(self.map_cs["z_center"].get()); zsz = float(self.map_cs["z_size"].get())
                xs, xe = xc - xsz / 2.0, xc + xsz / 2.0
                zs, ze = zc - zsz / 2.0, zc + zsz / 2.0
            else:
                xs = float(self.map_vars["x_start"].get()); xe = float(self.map_vars["x_end"].get())
                zs = float(self.map_vars["z_start"].get()); ze = float(self.map_vars["z_end"].get())
            return xs, xe, xstep, zs, ze, zstep

        def _set_cfg(self, var, key):
            cur = self.config_data.get(key)
            raw = var.get().strip()
            try:
                if isinstance(cur, int) and not isinstance(cur, bool):
                    self.config_data[key] = int(float(raw))
                else:
                    self.config_data[key] = float(raw)
            except ValueError:
                raise ValueError(f"Bad value for {key!r}: {raw!r}")

        def _map_mode_switch(self):
            """Show only the active definition mode's fields (Start/End vs Center+Size)."""
            self.map_se_frame.pack_forget()
            self.map_cs_frame.pack_forget()
            (self.map_cs_frame if self.map_mode.get() == "Center+Size"
             else self.map_se_frame).pack(fill="x")
            self._update_map_estimate()

        def _update_map_estimate(self):
            """Live readout of the mapping plan: number of pixels, pixel size
            (= step = spatial resolution), field of view (= range), and an
            a-priori scan-time estimate from the XZ stage speed/accel + per-point
            settle + averaging. Lets you see a 4x4 is seconds while 500x500 is
            hours, and pick a resolution accordingly — before starting."""
            try:
                xs, xe, xstep, zs, ze, zstep = self._map_bounds()
                nx = len(np.arange(xs, xe + xstep / 2, xstep))
                nz = len(np.arange(zs, ze + zstep / 2, zstep))
                n = nx * nz
                settle = float(self.map_vars["settling_time"].get())
                navg = max(1, int(float(self.map_vars["num_averages"].get())))
                ppmm = float(self.config_data.get("pulses_per_mm", 500)) or 500.0
                v = float(self.config_data.get("stage_speed_fast", 5000)) / ppmm   # mm/s
                v = v if v > 0 else 10.0
                t_accel, t_read = 0.2, 0.08   # SHOT-602 accel ramp; per lock-in read
                # snake raster: (n-nx) short Z steps + ~nx longer X row-changes
                move = (n - nx) * (t_accel + zstep / v) + nx * (t_accel + xstep / v)
                dwell = n * (settle + navg * t_read)
                secs = move + dwell
                self.map_est.set(
                    f"Resolution {nx}×{nz} = {n} pts · pixel {xstep:g}×{zstep:g} mm · "
                    f"FOV {xe-xs:g}×{ze-zs:g} mm\n"
                    f"est. scan time ≈ {self._fmt_long(secs)}   "
                    f"(@ {v:.0f} mm/s, settle {settle:g}s, avg {navg})")
            except (ValueError, ZeroDivisionError):
                self.map_est.set("Resolution: —")

        # ---- timescan range unit conversion (µm position <-> ps) + estimate ----
        def _anchor_t0_to_start(self):
            """In µm mode the Start box IS delay 0 — apply it the moment it is typed.

            If the Start is 549390 µm, that position is 0 ps. This used to happen only when
            Start was pressed, so until then every ps in the window was quoted against
            the PREVIOUS scan's zero — 549390 um reading "-26.95 ps" off a stale
            t0 = 545.35 mm. Nothing moves here; only the conversion is re-based."""
            if self.ts_unit.get() == "ps":
                return
            if self.worker and self.worker.is_alive():
                return          # re-basing mid-scan would bend the sweep under way
            try:
                t0_mm = float(self.ts_vars["time_start_ps"].get()) / 1000.0
            except (ValueError, TypeError):
                return          # half-typed number; wait for a parseable one
            if t0_mm == float(self.config_data.get("bbd_t0_mm", 0.0)):
                return
            self.config_data["bbd_t0_mm"] = t0_mm
            self.hub.sync_bbd_calibration(self.config_data)
            if "bbd_t0_mm" in getattr(self, "cfg_vars", {}):
                self.cfg_vars["bbd_t0_mm"].set(f"{t0_mm:.4f}")   # Settings tab shows the truth
            self._relabel_position()
            self._rebase_map_delay()      # keep Mapping on its spot, re-label its ps

        def _rebase_map_delay(self):
            """t0 moved -> hold the Mapping delay on the same PHYSICAL spot and re-label
            its ps, not the reverse.

            Every timescan re-zeros t0 to its own Start, so a bare "1118 ps" means a
            different place after the next scan. The µm twin is the ground truth: a delay
            picked off a curve is a position on the sample, not an abstract number."""
            if not hasattr(self, "map_delay_um"):
                return                      # mapping tab not built yet
            try:
                float(self.map_delay_um.get().replace(",", "").replace("um", "").strip())
            except (ValueError, TypeError):
                self._update_map_delay_um()  # no µm yet -> derive it from the ps box
                return
            self._sync_delay_from_um()

        def _scan_start_mm(self):
            """The stage position the Scan Start box points at, in mm (either unit)."""
            try:
                v = float(self.ts_vars["time_start_ps"].get())
            except (ValueError, TypeError, KeyError):
                return None
            if self.ts_unit.get() == "ps":
                return ps_to_pos_um(v, float(self.config_data.get("bbd_t0_mm", 0.0)),
                                    self._ts_mm_per_ps()) / 1000.0
            return v / 1000.0

        def _relabel_position(self):
            """Re-render the position readout from the last known mm — no hardware I/O,
            so it can run on every keystroke in the Start box.

            The ps is the DISTANCE from the Scan Start box to where the delay line is
            standing. Two requirements, and this satisfies both at once:
              "the readout follows whatever is typed"  -> the zero is the Start box, so retyping
                  Start re-reads immediately even though nothing moved;
              "it can never be negative"                 -> it is a distance, so it cannot be.
            On Start the line parks on the box and it reads 0.00, then counts up as the
            sweep runs. Just the number, no explanatory tail."""
            if self._last_pos_mm is None or not hasattr(self, "pos_var"):
                return
            mm = self._last_pos_mm
            zero = self._scan_start_mm()
            if zero is None:
                zero = mm               # half-typed Start box: hold at 0 rather than lie
            ps = abs(mm - zero) / abs(self._ts_mm_per_ps())
            self.pos_var.set(f"Main delay: {mm*1000:.1f} um  ({ps:.2f} ps)")

        def _ts_mm_per_ps(self):
            return mm_per_ps_for(self.config_data)

        def _ts_pos_to_ps(self, v):
            """Absolute range endpoint -> ps. µm(position)->ps via t0/double-pass, else identity."""
            if self.ts_unit.get() == "µm":
                return pos_um_to_ps(v, float(self.config_data.get("bbd_t0_mm", 0.0)),
                                    self._ts_mm_per_ps())
            return v

        def _ts_step_to_ps(self, v):
            """Step delta -> ps (no t0 offset)."""
            if self.ts_unit.get() == "µm":
                return step_um_to_ps(v, self._ts_mm_per_ps())
            return v

        def _check_ts_travel(self):
            """Show this delay line's travel; turn Start/End red the moment the range leaves it."""
            if not hasattr(self, "ts_travel_var") or not hasattr(self, "ts_entries"):
                return
            cfg = dict(self.config_data)
            travel_um = float(cfg.get("bbd_travel_mm", 570.0)) * 1000.0
            short = str(cfg.get("stage_name", "")).split(" (")[0]
            off = None
            try:
                s = float(self.ts_vars["time_start_ps"].get()); e = float(self.ts_vars["time_end_ps"].get())
                st = float(self.ts_vars["time_step_ps"].get()) or 1.0
                t0, sp, ep, _ = plan_timescan(cfg, self.ts_unit.get(), s, e, st)
                cfg["bbd_t0_mm"] = t0
                off = timescan_out_of_travel(cfg, sp, ep)
            except (ValueError, KeyError, TypeError):
                pass
            for k in ("time_start_ps", "time_end_ps"):
                if k in self.ts_entries:
                    self.ts_entries[k].configure(style="Bad.TEntry" if off else "TEntry")
            if off:
                self.ts_travel_var.set(f"⚠ outside travel: {short} reaches 0 – {travel_um:,.0f} µm")
                self.ts_travel_lbl.configure(foreground="#b91c1c")
            else:
                self.ts_travel_var.set(f"Travel 0 – {travel_um:,.0f} µm  ({short})")
                self.ts_travel_lbl.configure(foreground="#555")

        def _ts_here(self, key):
            """Put the delay line's current position into Start or End (µm, or ps in ps mode)."""
            if not self.hub.connected or self.hub.bbd is None:
                messagebox.showinfo("Delay line", "Connect first — 'here' is read from the delay line."); return
            try:
                um = float(self.hub.read_position_mm()) * 1000.0
            except Exception as e:
                self._set_status(f"Can't read the delay line: {e}"); return
            if self.ts_unit.get() == "ps":
                val = pos_um_to_ps(um, float(self.config_data.get("bbd_t0_mm", 0.0)), mm_per_ps_for(self.config_data))
                self.ts_vars[key].set(f"{val:.2f}")
            else:
                self.ts_vars[key].set(f"{um:.0f}")
            self._set_status(f"{'Start' if key == 'time_start_ps' else 'End'} = here ({um:,.0f} µm).")

        def _update_ts_estimate(self):
            try:
                s = float(self.ts_vars["time_start_ps"].get())
                e = float(self.ts_vars["time_end_ps"].get())
                st = float(self.ts_vars["time_step_ps"].get())
                navg = max(1, int(float(self.ts_vars["num_averages"].get())))
                settle = float(self.ts_vars["bbd_settle_s"].get())
                reps = max(1, int(float(self.ts_vars["repetition"].get())))
                # preview the sweep Start will actually run — same anchoring, same signs.
                # (It used to convert with whatever t0 was left over from the last scan,
                # so the µm mode advertised e.g. "ps -3665–0" for a run that does 0–3665.)
                _t0, sp, ep, stp = plan_timescan(self.config_data, self.ts_unit.get(), s, e, st)
                n = int(round(abs(ep - sp) / stp)) + 1 if stp else 0
                # per-point time, calibrated to a real run 2026-07-03 (~0.47 s/pt
                # at settle 0.2, avg 3): settle + reads + BBD move.
                per = settle + navg * 0.06 + 0.10
                secs = n * per * reps
                note = f" · ps {min(sp,ep):.0f}–{max(sp,ep):.0f}" if self.ts_unit.get() == "µm" else ""
                self.ts_est.set(f"{n} pts{(' ×'+str(reps)) if reps > 1 else ''}{note}\n"
                                f"est. scan time ≈ {self._fmt_long(secs)}")
            except (ValueError, ZeroDivisionError):
                self.ts_est.set("—")
            self._reset_ts_axis()   # no scan loaded -> keep the axis showing 0 .. this sweep

        def _update_eta(self, var, done, total, done_offset=0):
            if self._scan_t0 is None:
                return
            elapsed = time.time() - self._scan_t0
            done_all = done_offset + done
            rate = done_all / elapsed if elapsed > 0 else 0
            togo = (total - done_all) / rate if rate > 0 else 0
            var.set(f"Total {self._fmt(elapsed)}   /   To go {self._fmt(togo)}   ({done_all}/{total})")

        @staticmethod
        def _fmt(sec):
            sec = int(sec)
            return f"{sec//60}m {sec%60}s" if sec >= 60 else f"{sec}s"

        @staticmethod
        def _fmt_long(sec):
            sec = int(round(sec))
            h, rem = divmod(sec, 3600)
            m, s = divmod(rem, 60)
            if h:
                return f"{h}h {m}m"
            if m:
                return f"{m}m {s}s"
            return f"{s}s"

        @staticmethod
        def _style_labview(fig, ax):
            """Black plot area + yellow grid, echoing the LabVIEW Waveform/Intensity Graph."""
            fig.patch.set_facecolor("#ececec")
            ax.set_facecolor("black")
            ax.grid(True, which="major", color="#d4d400", alpha=0.55, linewidth=0.7)
            ax.grid(True, which="minor", color="#7a7a00", alpha=0.35, linewidth=0.4)
            ax.minorticks_on()
            for sp in ax.spines.values():
                sp.set_color("#606060")
            ax.tick_params(colors="#202020", which="both")
            ax.title.set_color("#202020")
            ax.xaxis.label.set_color("#202020")
            ax.yaxis.label.set_color("#202020")

        def _set_status(self, text):
            self.status_var.set(text)

        def _set_idn(self, text):
            self.idn_text.config(state="normal")
            self.idn_text.delete("1.0", "end")
            self.idn_text.insert("1.0", text + "\n")
            self.idn_text.config(state="disabled")

        # -------------------------------------------------- settings persistence
        def _here(self):
            return os.path.dirname(os.path.abspath(__file__))

        def _log(self, msg):
            """Append a flushed breadcrumb to moke_gui.log — so if the GUI HANGS (no
            exception), the last line pinpoints exactly where it froze.

            MOKE_LOG_DIR redirects it: the test scripts set that so a test run can't
            interleave with the live GUI's log and make a real session unreadable."""
            try:
                d = os.environ.get("MOKE_LOG_DIR") or self._here()
                with open(os.path.join(d, "moke_gui.log"), "a", encoding="utf-8") as f:
                    f.write(f"[{datetime.now().strftime('%H:%M:%S')}] {msg}\n"); f.flush()
            except Exception:
                pass

        def _apply_config_file(self):
            """If moke_gui_config.json sits next to the script, its keys override the
            built-in device defaults (COM ports, GPIB addr, serials...). Lets another
            lab re-point the app without editing code."""
            try:
                p = os.path.join(self._here(), "moke_gui_config.json")
                if os.path.exists(p):
                    with open(p) as f:
                        for k, v in json.load(f).items():
                            if k in self.config_data:
                                self.config_data[k] = v
            except Exception:
                pass

        def _log_exc(self, exc, val, tb):
            """Write any GUI-callback traceback to moke_gui.log (for no-console runs)."""
            import traceback
            try:
                with open(os.path.join(self._here(), "moke_gui.log"), "a", encoding="utf-8") as f:
                    f.write("\n=== GUI callback error ===\n"
                            + "".join(traceback.format_exception(exc, val, tb)))
            except Exception:
                pass
            try:
                self._set_status(f"Error (see moke_gui.log): {val}")
            except Exception:
                pass

        # -------------------------------------------------- delay-line profiles
        def _init_stages(self):
            """Restore saved profiles, else build DEFAULT_STAGES with profile 0 = current settings."""
            saved = self._saved_stages
            if isinstance(saved, list) and saved and all(isinstance(p, dict) and "name" in p for p in saved):
                self.stages = saved
            else:
                self.stages = copy.deepcopy(DEFAULT_STAGES)
                for k in STAGE_KEYS:             # profile 0 inherits the existing calibration
                    if k in self.config_data:
                        self.stages[0][k] = copy.deepcopy(self.config_data[k])
            # which stage was active, by SERIAL (its saved name may be an old one)
            act_serial = next((str(p.get("bbd_serial")) for p in self.stages
                               if p.get("name") == self._saved_active), None)
            for p in self.stages:                # profiles saved with older names: rename by serial
                p["name"] = STAGE_NAMES.get(str(p.get("bbd_serial", "")), p["name"])
            order = list(STAGE_NAMES)            # keep LabVIEW ID order 0, 1, 2
            self.stages.sort(key=lambda p: order.index(str(p["bbd_serial"])) if str(p["bbd_serial"]) in order else 99)
            for p in self.stages:                # older/partial profiles: fill missing keys
                for k in STAGE_KEYS:
                    p.setdefault(k, copy.deepcopy(self.config_data.get(k)))
                p.setdefault("calibrated", False)
            names = [p["name"] for p in self.stages]
            # the active profile must match what config_data already holds (same serial)
            want = self._saved_active if self._saved_active in names else STAGE_NAMES.get(act_serial)
            if want is None:
                cur = str(self.config_data.get("bbd_serial", ""))
                want = next((p["name"] for p in self.stages if str(p["bbd_serial"]) == cur), names[0])
            self.active_stage = want
            self._load_stage_into_config(self._active_profile())
            self.stage_combo["values"] = names
            self.stage_sel_var.set(want)

        def _active_profile(self):
            return next((p for p in self.stages if p["name"] == self.active_stage), None)

        def _store_active_stage(self):
            """Copy the live calibration (config_data) back into the active profile."""
            p = self._active_profile()
            if p is None:
                return
            old_t0 = p.get("bbd_t0_mm")
            for k in STAGE_KEYS:
                if k in self.config_data:
                    p[k] = copy.deepcopy(self.config_data[k])
            if old_t0 is not None and abs(float(p["bbd_t0_mm"]) - float(old_t0)) > 1e-9:
                p["calibrated"] = True       # t0 was set on this stage
            if hasattr(self, "ts_vars") and hasattr(self, "ts_unit"):   # each stage keeps its own scan range
                p["scan"] = {k: self.ts_vars[k].get() for k in ("time_start_ps", "time_end_ps", "time_step_ps")}
                p["scan"]["unit"] = self.ts_unit.get()
            if hasattr(self, "tc_var") and hasattr(self, "sens_var"):   # each system keeps its lock-in settings
                p["lockin"] = {"tc": self.tc_var.get(), "sens": self.sens_var.get()}

        def _load_stage_into_config(self, p):
            if p is None:
                return
            for k in STAGE_KEYS:
                if k in p and p[k] is not None:
                    self.config_data[k] = copy.deepcopy(p[k])
                    if k in getattr(self, "cfg_vars", {}):
                        self.cfg_vars[k].set(str(p[k]))
            self.config_data["stage_name"] = p["name"]
            short = p["name"].split(" (")[0]
            for fr, title in ((getattr(self, "scan_frame", None), "Scan — main delay line: "),
                              (getattr(self, "main_ctrl_frame", None), "Main delay line control: ")):
                if fr is not None:
                    fr.config(text=title + short)
            if hasattr(self, "cfg_stage_lbl"):
                self.cfg_stage_lbl.set(f"Fields below → {p['name']}  ·  serial {p.get('bbd_serial')}")
            self._show_usb_state()
            if hasattr(self, "_check_ts_travel"):
                self._check_ts_travel()

        def _restore_scan_range(self, p):
            sc = (p or {}).get("scan")
            if not sc:
                return False
            try:
                if sc.get("unit"):
                    self.ts_unit.set(sc["unit"])
                for k in ("time_start_ps", "time_end_ps", "time_step_ps"):
                    if k in sc:
                        self.ts_vars[k].set(sc[k])
                return True
            except Exception:
                return False

        # ---- which delay lines are actually on USB right now (read-only enumeration) ----
        def _scan_usb_bg(self):
            if self._connecting or (self.worker and self.worker.is_alive()):
                return                          # never enumerate while devices are being opened
            if not hasattr(self, "_usb_lock"):
                self._usb_lock = threading.Lock()
            if not self._usb_lock.acquire(blocking=False):
                return                          # a scan is already running

            def work():
                try:
                    serials = _list_delay_lines()
                    if serials is not None and self.hub.bbd is not None:
                        serials = sorted(set(serials) | {str(self.config_data.get("bbd_serial"))})
                    self.out_q.put({"kind": "usb_stages", "serials": serials})
                finally:
                    self._usb_lock.release()
            threading.Thread(target=work, daemon=True).start()

        def _show_usb_state(self):
            if not hasattr(self, "stage_usb_var"):
                return
            serials = getattr(self, "_usb_serials", None)
            p = self._active_profile() if getattr(self, "stages", None) else None
            if serials is None or p is None:
                self.stage_usb_var.set(""); return
            on = [q["name"].split(" ·")[0] for q in self.stages if str(q["bbd_serial"]) in serials]
            if str(p["bbd_serial"]) in serials:
                self.stage_usb_var.set("✓ USB")
                self.stage_usb_lbl.config(foreground="#060")
            else:
                self.stage_usb_var.set("✗ not found (unplugged / in use)" + (f" · free: {', '.join(on)}" if on else ""))
                self.stage_usb_lbl.config(foreground="#a00")

        # ---- pump delay travel limits (blank = not set: nothing is checked) ----
        def _pump_limits(self):
            try:
                lo = self.pump_lim_vars["pump_min_um"].get().strip()
                hi = self.pump_lim_vars["pump_max_um"].get().strip()
                if not lo or not hi:
                    return None
                lo, hi = float(lo), float(hi)
                return (min(lo, hi), max(lo, hi))
            except (ValueError, AttributeError, KeyError):
                return None

        def _pump_limit_error(self, values):
            lim = self._pump_limits()
            if lim is None:
                return None
            bad = [v for v in values if v < lim[0] - 1e-6 or v > lim[1] + 1e-6]
            if bad:
                return (f"{bad[0]:,.0f} µm is outside the pump delay limits "
                        f"{lim[0]:,.0f} … {lim[1]:,.0f} µm. Nothing has moved.")
            return None

        def _pump_lim_refresh(self):
            self.pump_lim_note.set("" if self._pump_limits() else "not set — no travel check")

        # ---- pump delay manual control ----
        def _pump_cfg(self):
            c = dict(self.config_data)
            c["dl2_axis"] = int(float(self.dl2_vars["dl2_axis"].get()))
            c["dl2_pulses_per_mm"] = float(self.dl2_vars["dl2_pulses_per_mm"].get())
            if c["dl2_axis"] not in (1, 2) or c["dl2_pulses_per_mm"] <= 0:
                raise ValueError("Axis must be 1 or 2 and Pulses/mm > 0.")
            return c

        def _pump_ready(self, quiet=False):
            if not self.hub.connected or (self.worker and self.worker.is_alive()):
                if not quiet:
                    self._require_conn()
                return None
            if not self.hub.has_pump:
                if not quiet:
                    messagebox.showinfo("Pump delay", "Tick 'Pump' in the top bar and Connect — the pump delay has its own SHOT-602.")
                return None
            try:
                return self._pump_cfg()
            except ValueError as e:
                if not quiet:
                    messagebox.showerror("Pump delay", str(e))
                return None

        def _pump_read(self, quiet=False):
            c = self._pump_ready(quiet=quiet)
            if c is None or self._hw_busy:
                return
            try:
                self.pump_pos_var.set(f"Pump delay position: {_dl2_read_um(self.hub, c):,.1f} µm")
            except Exception as e:
                self.pump_pos_var.set(f"Pump delay position: read failed ({e})")

        def _pump_read_when_idle(self, tries=300):
            if self._hw_busy and tries > 0:
                self.after(200, lambda: self._pump_read_when_idle(tries - 1)); return
            self._pump_read(quiet=True)

        def _pump_move(self, um, label):
            c = self._pump_ready()
            if c is None:
                return
            err = self._pump_limit_error([um])
            if err:
                messagebox.showerror("Pump delay", err); return
            self._run_bg(lambda: _dl2_move_um(self.hub, c, um), label)
            self.after(300, self._pump_read_when_idle)

        def _pump_go(self, key):
            try:
                um = float(self.dl2_vars[key].get())
            except ValueError:
                return
            self._pump_move(um, f"Pump delay → {um:,.0f} µm")

        def _pump_jog(self, sign):
            c = self._pump_ready()
            if c is None:
                return
            try:
                step = float(self.pump_jog_um.get())
                cur = _dl2_read_um(self.hub, c)
            except Exception as e:
                self._set_status(f"Pump delay jog: {e}"); return
            self._pump_move(cur + sign * step, "Pump delay jog")

        def _pump_move_target(self):
            try:
                um = float(self.pump_target_um.get())
            except ValueError:
                return
            self._pump_move(um, f"Pump delay → {um:,.0f} µm")

        def _on_stage_selected(self, event=None):
            name = self.stage_sel_var.get()
            if not name or name == self.active_stage:
                return
            if (self.worker and self.worker.is_alive()) or self._hw_busy:
                messagebox.showwarning("Busy", "Stop the running scan / move first, then switch the delay line.")
                self.stage_sel_var.set(self.active_stage)
                return
            try:
                self._apply_settings()       # keep un-applied edits of the CURRENT stage
            except Exception:
                pass
            self._store_active_stage()
            was_connected = self.hub.connected   # if so: swap only the delay line below
            self.active_stage = name
            p = self._active_profile()
            self._load_stage_into_config(p)
            try:
                self._update_ts_estimate(); self._update_map_estimate()
            except Exception:
                pass
            if self._restore_scan_range(p):
                name_msg = " Its last scan range is back in 'Scan'."
            else:
                name_msg = " First use: set Start/End inside its travel (or 'Start = here')."
            lk = p.get("lockin") or {}
            if lk.get("tc"):
                self.tc_var.set(lk["tc"])
            if lk.get("sens"):
                self.sens_var.set(lk["sens"])
            if lk:
                name_msg += " Its TC/Sens are back — press Set to apply them."
            self._check_ts_travel()
            msg = f"Delay line → {name}. Press Connect.{name_msg}"
            if was_connected:
                msg = f"Delay line → {name}: switching (lock-in / XZ / pump stay connected)…{name_msg}"
                self._swap_delay_line()
            self._usb_serials = None          # stale until the rescan below answers
            self.stage_usb_var.set("checking USB…"); self.stage_usb_lbl.config(foreground="#555")
            if not was_connected:
                self._scan_usb_bg()
            if not p.get("calibrated", False):
                msg += "  ⚠ t0 / double-pass / reverse not calibrated for this stage yet."
            self._set_status(msg)
            self._save_settings()

        def _settings_path(self):
            return os.environ.get("MOKE_SETTINGS") or \
                os.path.join(os.path.dirname(os.path.abspath(__file__)), "moke_gui_settings.json")

        def _settings_vars(self):
            d = {}
            for k, v in self.ts_vars.items():   d[f"ts.{k}"] = v
            for k, v in self.map_vars.items():  d[f"map.{k}"] = v
            for k, v in self.map_cs.items():    d[f"mapcs.{k}"] = v
            for k, v in self.cfg_vars.items():  d[f"cfg.{k}"] = v
            for k, v in self.ts_show.items():   d[f"ts.show_{k}"] = v
            for k, v in getattr(self, "dl2_vars", {}).items(): d[f"ts.{k}"] = v
            for k, v in getattr(self, "pump_lim_vars", {}).items(): d[f"ts.{k}"] = v
            if hasattr(self, "dl2_on"): d["ts.dl2_on"] = self.dl2_on
            d.update({"ts.unit": self.ts_unit, "ts.signal": self.ts_signal,
                      "ts.view": self.ts_view, "ts.after": self.ts_after,
                      "map.mode": self.map_mode, "map.signal": self.map_signal,
                      "map.cmap": self.map_cmap, "map.after": self.map_after,
                      "map.3d_on": self.map3d_on, "map.3d_delays": self.map3d_delays_var,
                      "map.click_move": self.map_click_move, "map.clim_lock": self.map_clim_lock,
                      "map.clim_lo": self.map_clim_lo, "map.clim_hi": self.map_clim_hi,
                      "map.flip_x": self.map_flip_x, "map.flip_z": self.map_flip_z,
                      "topbar.mode": self.mode_var,
                      "topbar.stage": self.stage_var, "topbar.pump": self.pump_var, "topbar.monitor": self.monitor_on,
                      "topbar.tc": self.tc_var, "topbar.sens": self.sens_var})
            return d

        def _save_settings(self):
            """Persist all parameters so the next launch restores them."""
            try:
                data = {k: v.get() for k, v in self._settings_vars().items()}
                data["_version"] = SETTINGS_VERSION
                if getattr(self, "stages", None):
                    self._store_active_stage()
                    data["stages"] = self.stages
                    data["active_stage"] = self.active_stage
                with open(self._settings_path(), "w") as f:
                    json.dump(data, f, indent=2)
            except Exception:
                pass

        def _load_settings(self):
            try:
                with open(self._settings_path()) as f:
                    data = json.load(f)
            except Exception:
                return
            data = self._migrate_settings(data)
            self._saved_stages = data.pop("stages", None)
            self._saved_active = data.pop("active_stage", None)
            vs = self._settings_vars()
            for k, val in data.items():
                if k in vs:
                    try:
                        vs[k].set(val)
                    except Exception:
                        pass

        def _migrate_settings(self, data):
            """A saved settings file OVERRIDES the code defaults (it is applied on load), so an old
            file silently resurrects an old calibration. Version it and migrate instead.

            v1 -> v2 (2026-07-13): the delay axis used to be a bare position proxy — bbd_t0_mm = 0,
            no reverse, so a LARGER position read as a LARGER "ps", backwards from the physics.
            t0 is now the measured pump-probe overlap. Drop v1's stale delay calibration (so the
            new measured default wins) and re-base any delay it stored onto the same position.
            """
            if int(data.get("_version", 1)) >= SETTINGS_VERSION:
                return data
            # v2/v3 (2026-07-16): drop the pump-probe-overlap t0 + reverse experiment. Delay is
            # again ps = position/149.9 (always positive; the timescan plot measures from its start).
            # Removing the stale cfg keys lets the new code defaults (t0=0, reverse=False) win.
            for key in ("cfg.bbd_t0_mm", "cfg.bbd_reverse"):
                data.pop(key, None)
            self.after(400, lambda: self._set_status(
                "Delay convention reset: ps = position/149.9 (no negatives). "
                "The timescan x-axis is measured from its own start."))
            return data

        def _on_close(self):
            if self.worker and self.worker.is_alive():
                if not messagebox.askyesno("Quit", "A scan is running. Stop and quit?"):
                    return
                self.stop_event.set()
                self.worker.join(timeout=5)
            self._save_settings()
            self.hub.close()
            self.destroy()

    app = MokeApp()
    if test_hook is not None:
        try:
            test_hook(app)
        finally:
            app.destroy()
        return
    app.mainloop()


# ============================================================
if __name__ == "__main__":
    if "--selftest" in sys.argv[1:]:
        _selftest()
    elif "--smoke" in sys.argv[1:]:
        run_app(smoke=True)
    else:
        # capture a startup crash to the log too, so a no-console (pythonw) launch is debuggable
        try:
            run_app()
        except Exception:
            import traceback
            try:
                with open(os.path.join(os.path.dirname(os.path.abspath(__file__)), "moke_gui.log"), "a", encoding="utf-8") as f:
                    f.write("\n=== STARTUP CRASH ===\n" + traceback.format_exc())
            except Exception:
                pass
            raise
