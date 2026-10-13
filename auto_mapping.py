"""
Automated TR-MOKE acquisition (full Python control)
====================================================
Hardware (all driven directly from this script — LabVIEW no longer required):
  - Thorlabs BBD30X benchtop brushless RACK controller (delay line, Thorlabs
    serial 10000000, DDS-family stage). Driven via pylablib over the FTDI D2XX
    driver, addressed BY SERIAL NUMBER with is_rack_system=True. No Kinesis .NET
    needed. See ThorlabsDelay docstring for the 3 connection gotchas.
  - NF LI5645 lock-in amplifier (GPIB 12, 1 mHz – 250 kHz)
  - Sigma Koki SHOT-602 (2-axis stage controller, RS232C @ 9600 8N1 CR+LF)
    Mac:   /dev/cu.usbserial-FTEA2UHG (FTDI USB-RS232)
    LabPC: COMx (assign after plugging FTDI adapter)
    Stages: SGSP20-20

Coexistence: LabVIEW Testbench.vi must be CLOSED before running — it holds
the BBD COM and LI5645 GPIB. After this script connects, those resources are
owned by Python until the script exits.

Modes:
  scan      -> probe all instruments, print what's reachable
  timescan  -> sweep BBD delay at the current (fixed) XZ position, read lock-in
  run       -> full XZ map at one BBD delay (BBD moved by Python before scan)
  plot      -> load saved .npz and visualize
"""

import numpy as np
import time
import os
import sys
from datetime import datetime

# pyvisa and pyserial are heavy / lab-only; loaded lazily by classes that need them.
def _pyvisa():
    import pyvisa as _pv
    return _pv

def _pyserial():
    import serial as _s
    return _s


# ============================================================
# Configuration — fill in after checking devices
# ============================================================
CONFIG = {
    # --- SHOT-602 (RS232 9600 8N1 CR+LF, verified 2026-04-30 on Mac) ---
    "shot602_port":  "/dev/cu.usbserial-FTEA2UHG",  # Mac default; lab PC override to "COM4" etc.
    "shot602_baud":  9600,

    # --- LI5645 lock-in (GPIB) ---
    "lockin_addr":   "GPIB0::12::INSTR",
    "lockin_model":  "LI5645",   # also supported: SR830/SR860/7265

    # --- SHOT-602 axis assignment ---
    "x_axis": 1,
    "z_axis": 2,

    # --- Scan ranges (mm) ---
    "x_start": 6.0,
    "x_end":   12.0,
    "x_step":  1.0,

    "z_start": 3.0,
    "z_end":   8.0,
    "z_step":  1.0,

    # --- Thorlabs BBD delay line (BBD30X RACK controller via pylablib + FTDI D2XX) ---
    # Connect by THORLABS SERIAL (D2XX), NOT a COM port; rack system; manual scale
    # because pylablib doesn't recognize the BBD30X model. (resolved lab PC 2026-06-02)
    "bbd_serial":         "10000000",  # Thorlabs serial of OUR delay line (per list_kinesis_devices)
    "bbd_is_rack_system": True,         # BBD30X is a multi-bay rack -> required, else connect fails
    "bbd_counts_per_mm":  20000,        # DDS220/300/600/MLS203 family = 20000 counts/mm (pylablib table)
    "bbd_channel":        1,            # nchannels=1
    "bbd_port":           None,         # optional COM override; normally unused (we connect by serial)
    "bbd_t0_mm":          545.350,      # delay 0 = the last timescan's START position (auto-set on
    #                                     half-rise of the ultrafast step in timescan_20260713_180709
    #                                     (baseline -1e-6 -> peak 4.8e-4 at 539,375 um). Good to ~+-7 ps
    #                                     (the scan step was 13 ps); refine with a fine scan near t0.
    "bbd_travel_mm":      800.0,        # usable stage travel (the stage has been seen at 710 mm);
    #                                     delays landing outside 0..this are unreachable
    "bbd_double_pass":    True,         # True: 1 ps optical = 0.14989 mm stage; False: 0.29979 mm
    "bbd_reverse":        True,         # physical delay: larger position = smaller delay (delay 0 at t0)
    #                                     (the physical case: t0 = the pump-probe overlap position,
    #                                      and moving away from it shortens the probe path). Set
    #                                      bbd_t0_mm to the overlap position when you enable this.
    "bbd_settle_s":       0.2,          # extra settle after BBD move before lock-in read

    # --- timescan range (used by `timescan`; `run` uses delay_label_ps) ---
    "time_start_ps": 0.0,
    "time_end_ps":   50.0,
    "time_step_ps":  1.0,
    "delay_label_ps": 0.0,            # for `run` (XZ map): single delay BBD is parked at

    # --- SHOT-602 stage pulse conversion ---
    # SGSP20-20: 1 pulse = 2 um (full step) → 500 pulses/mm. Verify against actual stage motion.
    "pulses_per_mm": 500,

    # --- Lock-in / scan timing ---
    "settling_time":      0.5,   # seconds to wait after move before reading
    "num_averages":       3,
    "time_constant_ms":   100,
    "wait_before_scan_ms": 300,

    # --- SHOT-602 speed (pulses/sec) ---
    "stage_speed_slow":  500,
    "stage_speed_fast": 5000,

    # --- Output ---
    "output_dir": None,          # auto-generated if None
}


# ============================================================
# SHOT-602 Controller (RS232 serial, 9600 8N1 CR+LF)
# ============================================================
class SHOT602:
    """Sigma Koki SHOT-602 two-axis stage controller.
    Command format (verified on Mac 2026-04-30):
      A:{axis}{+/-}P{pulses}  -> queue absolute target (in pulses)
      A:W{s1}P{p1}{s2}P{p2}   -> queue both axes
      G:                      -> execute queued move
      !:                      -> status (B=busy, R=ready)
      Q:                      -> "<pos1>,<pos2>,<ack1>,<ack2>,<R/B>"
    """

    TERM = "\r\n"

    def __init__(self, port, baud=9600, pulses_per_mm=500, timeout=2.0):
        serial = _pyserial()
        self.ser = serial.Serial(
            port=port, baudrate=baud,
            bytesize=8, parity="N", stopbits=1,
            timeout=timeout,
        )
        # SHOT-602 only answers when RTS is asserted (it uses RTS as a line-enable).
        # macOS pyserial asserts RTS on open by default, so this was invisible on the
        # Mac; Windows pyserial leaves RTS low, so Q:/!: returned nothing on the lab PC
        # until RTS was driven high. DTR is don't-care but set for a fully-asserted line.
        # (root-caused on lab PC COM5, 2026-06-02)
        self.ser.rts = True
        self.ser.dtr = True
        time.sleep(0.2)
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()
        self.ppmm = pulses_per_mm

    def _write(self, cmd):
        self.ser.write((cmd + self.TERM).encode("ascii"))

    def _query(self, cmd, wait=0.15):
        self.ser.reset_input_buffer()
        self._write(cmd)
        time.sleep(wait)
        return self.ser.read_all().decode("ascii", errors="replace").strip()

    def mm_to_pulse(self, mm):
        return int(round(mm * self.ppmm))

    def set_speed(self, axis, start_speed=500, max_speed=5000, accel_time=200):
        """D:{axis}S{start}F{max}R{accel_ms}."""
        self._write(f"D:{axis}S{start_speed}F{max_speed}R{accel_time}")
        time.sleep(0.1)

    def move_abs(self, axis, position_mm):
        """Move axis to an absolute position (mm), implemented as a relative M: move.

        The SHOT-602's absolute command A: was inferred from the LabVIEW VI binary
        but does NOT actually move this unit (verified on lab PC COM5 2026-06-02:
        `A:1+P500;G:` left the stage at 0). Only the relative M: command moves the
        stage. So we read the controller's current pulse count and command the delta.
        The controller's position counter is the source of truth for absolute coords.
        """
        target = self.mm_to_pulse(position_mm)
        cur_pair = self.get_position()
        cur = cur_pair[0] if axis == 1 else cur_pair[1]
        delta = target - cur
        if delta == 0:
            return
        sign = "+" if delta > 0 else "-"
        self._write(f"M:{axis}{sign}P{abs(delta)}")
        self._write("G:")
        self._wait_ready()

    def move_abs_both(self, x_mm, z_mm, x_axis=1, z_axis=2):
        """Move both axes to absolute (x_mm, z_mm).

        Done as two sequential single-axis relative M: moves (the verified path).
        The simultaneous A:W form was unverified and uses the broken A: opcode;
        sequential moves are slightly slower per point but reliable for raster scans.
        """
        self.move_abs(x_axis, x_mm)
        self.move_abs(z_axis, z_mm)

    def _wait_ready(self, timeout=30.0, poll_interval=0.1):
        t0 = time.time()
        while time.time() - t0 < timeout:
            status = self._query("!:", wait=0.1)
            if "R" in status:
                return
            time.sleep(poll_interval)
        raise TimeoutError("SHOT-602 did not return ready within timeout")

    def get_position(self):
        """Returns (x_pulses, z_pulses) by parsing Q: response."""
        resp = self._query("Q:")
        parts = [p.strip() for p in resp.split(",")]
        if len(parts) < 2:
            raise RuntimeError(f"Unexpected Q: response: {resp!r}")
        return int(parts[0]), int(parts[1])

    def close(self):
        try:
            self.ser.close()
        except Exception:
            pass


# ============================================================
# Thorlabs Delay Line Controller (USB via Kinesis .NET)
# ============================================================
class ThorlabsDelay:
    """Thorlabs BBD30X benchtop brushless delay-line controller (DDS stage).

    Connection — all three points root-caused on the lab PC 2026-06-02:
      1. This is a BBD30X RACK controller. Address it by its THORLABS SERIAL NUMBER
         ("10000000") over the FTDI D2XX driver — NOT by a VCP COM port. The COM
         ports' FTDI serials (e.g. 2310052041) do NOT match the Thorlabs serial;
         opening a COM port as the BBD only reads floating-line noise.
         Use `pylablib.devices.Thorlabs.list_kinesis_devices()` to enumerate.
      2. Must pass is_rack_system=True (multi-bay rack). Otherwise REQ_HW_INFO is
         sent to the single-unit dest 0x50, the device never replies, and pylablib
         raises ThorlabsBackendError on connect.
      3. pylablib does NOT recognize the "BBD30X" model, so its model gate refuses
         to apply the DDS stage scale and falls back to internal units (scale 1,1,1).
         We bypass that by passing an explicit scale TUPLE (the first branch of
         pylablib's _calculate_scale returns a tuple verbatim, skipping the gate).
         DDS220/300/600/MLS203 = 20000 counts/mm; BBD vel/accel time_conv = 102.4e-6.

    NOTE: live position read 540.55 mm > DDS220's 220 mm travel, so the physical
    stage is likely a DDS600 (scale is identical across the DDS family either way).

    Time/position model:
      double-pass: 1 ps optical delay = c/2 × 1 ps = 0.14989 mm stage motion
      single-pass: 1 ps optical delay = c   × 1 ps = 0.29979 mm stage motion
      stage_mm   = t0_mm + ps × mm_per_ps
    """

    C_MM_PER_PS = 0.299792458   # speed of light in mm/ps
    BBD_TIME_CONV = 102.4e-6    # APT velocity/accel time base for BBD controllers

    def __init__(self, serial_number="10000000", counts_per_mm=20000,
                 is_rack_system=True, channel=1, t0_mm=0.0, double_pass=True,
                 reverse=False, port=None):
        self.serial = str(serial_number)
        self.counts_per_mm = float(counts_per_mm)
        self.is_rack_system = is_rack_system
        self.channel = channel
        self.t0_mm = float(t0_mm)
        self.mm_per_ps = self.C_MM_PER_PS / (2.0 if double_pass else 1.0)
        if reverse:                     # delay grows as the position shrinks -> negative slope
            self.mm_per_ps = -self.mm_per_ps
        self.port = port           # optional COM override; normally None (connect by serial)
        self.motor = None
        self._connect()

    def _scale_tuple(self):
        """(position, velocity, acceleration) scale in mm-based units.
        Passed as a tuple so pylablib returns it verbatim and skips its
        model-recognition gate (which doesn't know BBD30X). With this,
        get_position()/move_to() work directly in mm."""
        ssc = self.counts_per_mm
        tc = self.BBD_TIME_CONV
        return (ssc, ssc * tc * 2**16, ssc * tc**2 * 2**16)

    def _connect(self):
        from pylablib.devices import Thorlabs
        conn = self.port if self.port else self.serial   # COM override or Thorlabs serial (D2XX)
        self.motor = Thorlabs.KinesisMotor(
            conn, scale=self._scale_tuple(),
            default_channel=self.channel, is_rack_system=self.is_rack_system,
        )
        info = self.motor.get_device_info()
        print(f"BBD connected: serial={info.serial_no}, model={info.model_no}, "
              f"pos={self.motor.get_position():.4f} mm")

    def ps_to_mm(self, time_ps):
        return self.t0_mm + time_ps * self.mm_per_ps

    def mm_to_ps(self, mm):
        return (mm - self.t0_mm) / self.mm_per_ps

    def move_to_mm(self, mm, wait=True, timeout=60.0):
        self.motor.move_to(float(mm))
        if wait:
            self.motor.wait_move(timeout=timeout)

    def move_to_ps(self, time_ps, wait=True, timeout=60.0):
        self.move_to_mm(self.ps_to_mm(time_ps), wait=wait, timeout=timeout)

    def is_moving(self):
        return bool(self.motor.is_moving())

    def stop(self, immediate=False, timeout=30.0):
        """Stop an in-progress move and wait until the controller reports idle."""
        self.motor.stop(immediate=bool(immediate), sync=True, timeout=timeout)

    def get_position_mm(self):
        return float(self.motor.get_position())

    def get_position_ps(self):
        return self.mm_to_ps(self.get_position_mm())

    def home(self, timeout=120.0):
        self.motor.home()
        self.motor.wait_for_home(timeout=timeout)

    def close(self):
        try:
            if self.motor:
                self.motor.close()
        except Exception:
            pass


# ============================================================
# Lock-in Amplifier (GPIB)
# ============================================================
class LockInAmp:
    """Lock-in amplifier reader.
    Supports NF LI5645 (primary), SR830, SR860, EG&G 7265.

    LI5645 (NF SCPI — verified on hardware 2026-06-05):
      FETC?            -> live values of the data items currently configured on the
                          instrument (this rig outputs X,Y; LabVIEW shows read X/read Y).
                          SR830-style `DATA? 1,2,3,4` is WRONG on the LI5645 (times out).
      FILT:TCON?       -> time constant
      FREQ?            -> reference frequency
    """

    def __init__(self, rm, address, model="LI5645"):
        self.inst = rm.open_resource(address)
        self.inst.timeout = 5000
        self.inst.read_termination = "\n"
        self.inst.write_termination = "\n"
        self.model = model

    def identify(self):
        return self.inst.query("*IDN?")

    def _fetch_li5645(self):
        """FETCh? returns the live values of the data items currently configured
        on the LI5645 (set in LabVIEW). This rig outputs [X, Y] (verified 2026-06-05).
        Read-only: we never change the DATA selection, so the other experiment that
        shares this lock-in keeps its config. R/theta are derived from X,Y."""
        vals = [float(t) for t in self.inst.query("FETC?").strip().split(",")]
        x = vals[0]
        y = vals[1] if len(vals) > 1 else 0.0
        return x, y

    def read_r(self):
        """Read signal magnitude R."""
        if self.model == "LI5645":
            x, y = self._fetch_li5645()
            return float(np.hypot(x, y))
        elif self.model == "SR830":
            return float(self.inst.query("OUTP? 3"))
        elif self.model == "SR860":
            return float(self.inst.query("OUTP? 2"))
        elif self.model == "7265":
            return float(self.inst.query("MAG."))

    def read_xy(self):
        """Read X and Y simultaneously."""
        if self.model == "LI5645":
            return self._fetch_li5645()
        elif self.model == "SR830":
            resp = self.inst.query("SNAP? 1,2")
            x, y = resp.split(",")
            return float(x), float(y)
        elif self.model == "SR860":
            resp = self.inst.query("SNAP? 0,1")
            x, y = resp.split(",")
            return float(x), float(y)
        elif self.model == "7265":
            x = float(self.inst.query("X."))
            y = float(self.inst.query("Y."))
            return x, y

    def read_all(self):
        """Read X, Y, R, theta simultaneously."""
        if self.model == "LI5645":
            x, y = self._fetch_li5645()
            return {"X": x, "Y": y,
                    "R": float(np.hypot(x, y)),
                    "theta": float(np.degrees(np.arctan2(y, x)))}
        elif self.model == "SR830":
            resp = self.inst.query("SNAP? 1,2,3,4")
            vals = [float(v) for v in resp.split(",")]
            return {"X": vals[0], "Y": vals[1], "R": vals[2], "theta": vals[3]}
        elif self.model == "SR860":
            resp = self.inst.query("SNAP? 0,1,2,3")
            vals = [float(v) for v in resp.split(",")]
            return {"X": vals[0], "Y": vals[1], "R": vals[2], "theta": vals[3]}
        elif self.model == "7265":
            x = float(self.inst.query("X."))
            y = float(self.inst.query("Y."))
            r = float(self.inst.query("MAG."))
            t = float(self.inst.query("PHA."))
            return {"X": x, "Y": y, "R": r, "theta": t}

    def set_time_constant_ms(self, tc_ms):
        """Set time constant in milliseconds. (Writes to the instrument — only in
        the full-Python mode where LabVIEW is closed and Python owns the lock-in.)"""
        if self.model == "LI5645":
            self.inst.write(f"FILT:TCON {tc_ms*1e-3}")
        elif self.model in ("SR830", "SR860"):
            # SR830 OFLT codes: 4=1ms 5=3ms 6=10ms 7=30ms 8=100ms 9=300ms 10=1s
            tc_codes = {1: 4, 3: 5, 10: 6, 30: 7, 100: 8, 300: 9, 1000: 10}
            code = tc_codes.get(int(tc_ms), 8)  # default 100 ms
            self.inst.write(f"OFLT {code}")

    def get_time_constant_ms(self):
        """Read the current time constant in ms (read-only). None if unknown."""
        try:
            if self.model == "LI5645":
                return float(self.inst.query("FILT:TCON?")) * 1000.0
        except Exception:
            return None
        return None

    def set_sensitivity_vrms(self, vrms):
        """Set the voltage sensitivity (full-scale range) in Vrms. LI5645 offers
        10 nV..1 V (VOLT:AC:RANG). Only writes in full-Python mode (LabVIEW closed)."""
        if self.model == "LI5645":
            self.inst.write(f"VOLT:AC:RANG {vrms:g}")

    def get_sensitivity_vrms(self):
        """Read the current voltage sensitivity in Vrms (read-only). None if unknown."""
        try:
            if self.model == "LI5645":
                return float(self.inst.query("VOLT:AC:RANG?"))
        except Exception:
            return None
        return None

    def close(self):
        self.inst.close()


# ============================================================
# Mock devices (for offline pipeline validation: --mock flag)
# ============================================================
class MockBBD:
    """Mimics ThorlabsDelay; pretends to move and reports position."""
    C_MM_PER_PS = 0.299792458

    def __init__(self, t0_mm=0.0, double_pass=True, reverse=False, **_):
        self.t0_mm = float(t0_mm)
        self.mm_per_ps = self.C_MM_PER_PS / (2.0 if double_pass else 1.0)
        if reverse:
            self.mm_per_ps = -self.mm_per_ps
        self._pos_mm = self.t0_mm
        print("BBD (mock) connected")

    def ps_to_mm(self, ps):  return self.t0_mm + ps * self.mm_per_ps
    def mm_to_ps(self, mm):  return (mm - self.t0_mm) / self.mm_per_ps
    def move_to_mm(self, mm, **_):
        time.sleep(0.01)
        self._pos_mm = float(mm)
    def move_to_ps(self, ps, **_): self.move_to_mm(self.ps_to_mm(ps))
    def is_moving(self):           return False
    def stop(self, **_):           pass
    def get_position_mm(self):     return self._pos_mm
    def get_position_ps(self):     return self.mm_to_ps(self._pos_mm)
    def home(self, **_):           self._pos_mm = self.t0_mm
    def close(self):               pass


class MockLockIn:
    """Synthetic lock-in signal for offline tests.

    - timescan (no stage): damped oscillation vs delay t (ps).
    - mapping (stage present): a spatially-structured pattern vs (x, z) — a
      centred symmetric Gaussian blob with fine fringes — so a mock XZ map looks
      like a real map instead of a flat field. Uses the stage position each call.
    """
    def __init__(self, bbd=None, stage=None, pulses_per_mm=500.0):
        self._bbd = bbd
        self._stage = stage
        self._ppmm = float(pulses_per_mm) or 500.0
        self._rng = np.random.default_rng(0)

    def identify(self): return "MOCK,LockIn,0,Sim"
    def set_time_constant(self, **_): pass
    def set_time_constant_ms(self, tc_ms): self._tc_ms = float(tc_ms)
    def get_time_constant_ms(self): return getattr(self, "_tc_ms", 100.0)
    def set_sensitivity_vrms(self, vrms): self._sens = float(vrms)
    def get_sensitivity_vrms(self): return getattr(self, "_sens", 1.0)
    def _signal(self):
        t = self._bbd.get_position_ps() if self._bbd else 0.0
        if self._stage is not None:
            # spatial test pattern from the current stage (x, z) in mm
            xp, zp = self._stage.get_position()
            x, z = xp / self._ppmm, zp / self._ppmm
            r2 = (x - 9.0) ** 2 + (z - 5.5) ** 2      # blob centred near default FOV
            mag = 5e-5 * np.exp(-r2 / 6.0) * (0.6 + 0.4 * np.cos(2 * np.pi * x / 1.5)
                                              * np.cos(2 * np.pi * z / 1.5))
            mag *= np.exp(-max(t, 0.0) / 200.0)        # mild delay dependence
            ph = 0.3
            xo = mag * np.cos(ph) + 1e-6 * self._rng.standard_normal()
            yo = mag * np.sin(ph) + 1e-6 * self._rng.standard_normal()
            return xo, yo
        # damped oscillation: 100 mV peak, 80 ps decay, 0.5 THz
        env = np.exp(-max(t, 0.0) / 80.0)
        osc = np.cos(2 * np.pi * 0.5 * max(t, 0.0))
        x = 0.1 * env * osc + 1e-4 * self._rng.standard_normal()
        y = 0.1 * env * np.sin(2 * np.pi * 0.5 * max(t, 0.0)) + 1e-4 * self._rng.standard_normal()
        return x, y
    def read_xy(self):  return self._signal()
    def read_r(self):
        x, y = self._signal()
        return float(np.hypot(x, y))
    def read_all(self):
        x, y = self._signal()
        r = float(np.hypot(x, y))
        th = float(np.degrees(np.arctan2(y, x)))
        return {"X": x, "Y": y, "R": r, "theta": th}
    def close(self): pass


class MockSHOT602:
    def __init__(self, *a, **k):
        self._x, self._z = 0, 0
        print("SHOT-602 (mock) connected")
    def set_speed(self, *a, **k): pass
    def move_abs_both(self, x_mm, z_mm, *a, **k):
        time.sleep(0.005)
        self._x, self._z = x_mm, z_mm
    def move_abs(self, axis, mm):
        time.sleep(0.005)
        if axis == 1: self._x = mm
        else:         self._z = mm
    def get_position(self): return (int(self._x * 500), int(self._z * 500))
    def close(self): pass


def _make_devices(config, need_stage=False):
    """Factory: returns (bbd, lockin, stage_or_None, rm_or_None).

    Three modes:
      mock        -> all three simulated; no hardware touched at all.
      mock_lockin -> REAL BBD + REAL stage, but a simulated lock-in (MockLockIn).
                     Exercises the full move->settle->read->save orchestration with
                     real motion + a synthetic signal — for probing the pipeline when
                     the real lock-in is unavailable (e.g. busy on another experiment).
      (neither)   -> all real.
    """
    if config.get("mock"):
        bbd = MockBBD(t0_mm=config["bbd_t0_mm"], double_pass=config["bbd_double_pass"],
                      reverse=config.get("bbd_reverse", False))
        stage = MockSHOT602() if need_stage else None
        lockin = MockLockIn(bbd=bbd, stage=stage,
                            pulses_per_mm=config.get("pulses_per_mm", 500))
        return bbd, lockin, stage, None

    bbd = ThorlabsDelay(
        serial_number=config["bbd_serial"], counts_per_mm=config["bbd_counts_per_mm"],
        is_rack_system=config["bbd_is_rack_system"], channel=config["bbd_channel"],
        t0_mm=config["bbd_t0_mm"], double_pass=config["bbd_double_pass"],
        reverse=config.get("bbd_reverse", False),   # MUST match the mock path above;
        port=config["bbd_port"],                    # forgetting it ran the real rig reverse=False
    )
    stage = (
        SHOT602(config["shot602_port"], config["shot602_baud"], config["pulses_per_mm"])
        if need_stage else None
    )
    if config.get("mock_lockin"):
        lockin = MockLockIn(bbd=bbd, stage=stage,   # real motion, synthetic signal
                            pulses_per_mm=config.get("pulses_per_mm", 500))
        rm = None
    else:
        rm = _pyvisa().ResourceManager()
        lockin = LockInAmp(rm, config["lockin_addr"], config["lockin_model"])
    return bbd, lockin, stage, rm


# ============================================================
# Hardware detection + system identification (read-only)
# ============================================================
# Registry mapping detected-hardware signatures -> a named measurement system.
# Add an entry per system; each `match(det)` inspects the detection dict.
def _has_idn(det, needle):
    return any(needle.lower() in (g.get("idn", "") or "").lower() for g in det.get("gpib", []))

def _has_kinesis(det, serial):
    return any(str(serial) in k for k in det.get("kinesis", []))

KNOWN_SYSTEMS = [
    # name, match(det) -> bool
    ("Stage A (delay-scan + XZ mapping)",
     lambda det: _has_idn(det, "LI5645")),
    ("Field-MOKE (Keithley source-meter)",
     lambda det: _has_idn(det, "KEITHLEY") or _has_idn(det, "MODEL 6")),
]


def identify_system(det):
    """Return the name of the first KNOWN_SYSTEMS entry whose signature matches."""
    for name, match in KNOWN_SYSTEMS:
        try:
            if match(det):
                return name
        except Exception:
            pass
    return "Unknown / generic"


def detect_hardware(config=None, timeout_ms=1500):
    """Read-only enumeration of connected instruments across GPIB / serial / Kinesis.
    Returns {gpib:[{addr,idn}], serial:[{port,desc,hwid}], kinesis:[...], system:name}.
    Safe to call any time Python owns the buses (LabVIEW closed)."""
    det = {"gpib": [], "serial": [], "kinesis": []}
    try:
        rm = _pyvisa().ResourceManager()
        for r in rm.list_resources():
            idn = ""
            try:
                inst = rm.open_resource(r)
                inst.timeout = timeout_ms
                idn = inst.query("*IDN?").strip()
                inst.close()
            except Exception:
                pass
            det["gpib"].append({"addr": r, "idn": idn})
        rm.close()
    except Exception as e:
        det["gpib_error"] = str(e)
    try:
        from serial.tools import list_ports
        for p in list_ports.comports():
            det["serial"].append({"port": p.device, "desc": p.description or "", "hwid": p.hwid or ""})
    except Exception as e:
        det["serial_error"] = str(e)
    try:
        from pylablib.devices import Thorlabs
        det["kinesis"] = [str(d) for d in Thorlabs.list_kinesis_devices()]
    except Exception as e:
        det["kinesis_error"] = str(e)
    det["system"] = identify_system(det)
    return det


# ============================================================
# Utility: scan all connected devices
# ============================================================
def scan_devices():
    """Scan GPIB (for LI5645) + serial ports (for SHOT-602). Run this first."""
    # GPIB scan
    print("=== GPIB devices ===")
    try:
        rm = _pyvisa().ResourceManager()
        resources = rm.list_resources()
        print(f"  {len(resources)} found")
        for r in resources:
            print(f"  {r}", end="")
            try:
                inst = rm.open_resource(r)
                inst.timeout = 2000
                idn = inst.query("*IDN?").strip()
                print(f"  ->  {idn}")
                inst.close()
            except Exception:
                print("  ->  (no *IDN? response)")
        rm.close()
    except Exception as e:
        print(f"  (pyvisa unavailable: {e})")

    # Serial port scan
    print("\n=== Serial ports (candidates for SHOT-602) ===")
    try:
        from serial.tools import list_ports
        ports = list(list_ports.comports())
        if not ports:
            print("  (none found)")
        for p in ports:
            print(f"  {p.device}  {p.description}  [{p.hwid}]")
    except Exception as e:
        print(f"  (pyserial list_ports failed: {e})")

    # Quick SHOT-602 probe on the configured port
    port = CONFIG["shot602_port"]
    print(f"\n=== Probe SHOT-602 at {port} @ {CONFIG['shot602_baud']} baud ===")
    try:
        s = SHOT602(port, CONFIG["shot602_baud"], CONFIG["pulses_per_mm"])
        resp = s._query("Q:")
        s.close()
        if resp:
            print(f"  Q: -> {resp!r}")
        else:
            print("  (no response — check cable / power / DIP switches)")
    except Exception as e:
        msg = str(e).encode(sys.stdout.encoding or "utf-8", errors="replace").decode(sys.stdout.encoding or "utf-8", errors="replace")
        print(f"  ({type(e).__name__}: {msg})")

    # Quick BBD probe via pylablib (APT-over-COM)
    print(f"\n=== Probe Thorlabs BBD (port={CONFIG['bbd_port']}, SN={CONFIG['bbd_serial']}) ===")
    try:
        bbd = ThorlabsDelay(
            serial_number=CONFIG["bbd_serial"],
            counts_per_mm=CONFIG["bbd_counts_per_mm"],
            is_rack_system=CONFIG["bbd_is_rack_system"],
            channel=CONFIG["bbd_channel"],
            t0_mm=CONFIG["bbd_t0_mm"],
            double_pass=CONFIG["bbd_double_pass"],
            reverse=CONFIG.get("bbd_reverse", False),
            port=CONFIG["bbd_port"],
        )
        pos_mm = bbd.get_position_mm()
        pos_ps = bbd.get_position_ps()
        print(f"  position: {pos_mm:.4f} mm  ({pos_ps:.3f} ps)")
        bbd.close()
    except Exception as e:
        msg = str(e).encode(sys.stdout.encoding or "utf-8", errors="replace").decode(sys.stdout.encoding or "utf-8", errors="replace")
        print(f"  ({type(e).__name__}: {msg})")


# ============================================================
# Time scan (BBD only — no SHOT-602 / fixed XZ)
# ============================================================
def run_timescan(config=None):
    """Sweep BBD delay at the current XZ position; record lock-in vs delay.

    Use this when no XZ stages are installed / connected (or for a calibration
    run at a single spot). LabVIEW Testbench.vi must be CLOSED first so that
    Python can take over BBD COM and LI5645 GPIB.
    """
    if config is None:
        config = CONFIG

    t_ps = np.arange(
        config["time_start_ps"],
        config["time_end_ps"] + config["time_step_ps"] / 2,
        config["time_step_ps"],
    )
    n_t = len(t_ps)
    print(f"Time scan: {n_t} points  {t_ps[0]:.2f} -> {t_ps[-1]:.2f} ps  "
          f"step {config['time_step_ps']} ps")

    if config["output_dir"] is None:
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        config["output_dir"] = f"timescan_{timestamp}"
    os.makedirs(config["output_dir"], exist_ok=True)

    bbd, lockin, _, rm = _make_devices(config, need_stage=False)
    print(f"Lock-in: {lockin.identify().strip()}")
    print(f"BBD start position: {bbd.get_position_mm():.4f} mm "
          f"({bbd.get_position_ps():.3f} ps)")

    data = np.zeros((n_t, 4))   # X, Y, R, theta per delay
    t_start = time.time()

    try:
        for i, ps in enumerate(t_ps):
            bbd.move_to_ps(float(ps))
            time.sleep(config["bbd_settle_s"])

            readings = []
            for _ in range(config["num_averages"]):
                readings.append(lockin.read_all())
                time.sleep(0.05)
            avg = {
                k: float(np.mean([r[k] for r in readings]))
                for k in ("X", "Y", "R", "theta")
            }
            data[i] = [avg["X"], avg["Y"], avg["R"], avg["theta"]]

            elapsed = time.time() - t_start
            rate = (i + 1) / elapsed if elapsed > 0 else 0
            eta = (n_t - i - 1) / rate if rate > 0 else 0
            print(f"  [{i+1}/{n_t}] t={ps:7.2f} ps  R={avg['R']:.3e}  "
                  f"theta={avg['theta']:7.2f}  ETA:{eta/60:.1f}min", end="\r")

    except KeyboardInterrupt:
        print("\n\n*** Time scan interrupted by user ***")
    finally:
        out = os.path.join(config["output_dir"], "timescan.npz")
        np.savez(
            out,
            t_ps=t_ps,
            data=data,
            t0_mm=config["bbd_t0_mm"],
            double_pass=config["bbd_double_pass"],
        reverse=config.get("bbd_reverse", False),
            config=str(config),
        )
        with open(os.path.join(config["output_dir"], "timescan.dat"), "w") as f:
            f.write("t(ps)\tX(V)\tY(V)\tR(V)\ttheta(deg)\n")
            for i, ps in enumerate(t_ps):
                f.write(f"{ps:.4f}\t{data[i,0]:.6e}\t{data[i,1]:.6e}\t"
                        f"{data[i,2]:.6e}\t{data[i,3]:.4f}\n")
        print(f"\nSaved {out}")
        bbd.close()
        lockin.close()
        if rm is not None: rm.close()

    return t_ps, data


# ============================================================
# Main Scan Routine
# ============================================================
def run_mapping(config=None):
    if config is None:
        config = CONFIG

    x_positions = np.arange(config["x_start"], config["x_end"] + config["x_step"] / 2, config["x_step"])
    z_positions = np.arange(config["z_start"], config["z_end"] + config["z_step"] / 2, config["z_step"])

    n_x = len(x_positions)
    n_z = len(z_positions)
    total_points = n_x * n_z
    delay_ps = config["delay_label_ps"]

    print(f"Scan plan: {n_x} X x {n_z} Z = {total_points} measurements at delay={delay_ps}ps (BBD set in LabVIEW)")
    print(f"  X: {x_positions[0]:.2f} -> {x_positions[-1]:.2f} mm ({n_x} pts, step {config['x_step']} mm)")
    print(f"  Z: {z_positions[0]:.2f} -> {z_positions[-1]:.2f} mm ({n_z} pts, step {config['z_step']} mm)")

    if config["output_dir"] is None:
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        config["output_dir"] = f"mapping_{timestamp}_t{delay_ps}ps"
    os.makedirs(config["output_dir"], exist_ok=True)

    # data: [x, z, {X, Y, R, theta}]
    data = np.zeros((n_x, n_z, 4))

    bbd, lockin, stage, rm = _make_devices(config, need_stage=True)
    print(f"Parking BBD at {delay_ps} ps...")
    bbd.move_to_ps(float(delay_ps))
    time.sleep(config["bbd_settle_s"])
    print(f"  BBD now at {bbd.get_position_mm():.4f} mm ({bbd.get_position_ps():.3f} ps)")

    stage.set_speed(config["x_axis"], 500, config["stage_speed_fast"], 200)
    stage.set_speed(config["z_axis"], 500, config["stage_speed_fast"], 200)

    print(f"\nStarting scan at {datetime.now().strftime('%H:%M:%S')}...")
    count = 0
    t_start = time.time()

    try:
        for i_x, x_mm in enumerate(x_positions):
            # Snake pattern: alternate Z direction for efficiency
            if i_x % 2 == 0:
                z_iter = list(enumerate(z_positions))
            else:
                z_iter = list(reversed(list(enumerate(z_positions))))

            for i_z, z_mm in z_iter:
                stage.move_abs_both(x_mm, z_mm, config["x_axis"], config["z_axis"])
                time.sleep(config["settling_time"])

                readings = []
                for _ in range(config["num_averages"]):
                    readings.append(lockin.read_all())
                    time.sleep(0.05)

                avg = {
                    "X":     float(np.mean([r["X"] for r in readings])),
                    "Y":     float(np.mean([r["Y"] for r in readings])),
                    "R":     float(np.mean([r["R"] for r in readings])),
                    "theta": float(np.mean([r["theta"] for r in readings])),
                }
                data[i_x, i_z] = [avg["X"], avg["Y"], avg["R"], avg["theta"]]

                count += 1
                elapsed = time.time() - t_start
                rate = count / elapsed if elapsed > 0 else 0
                eta = (total_points - count) / rate if rate > 0 else 0
                print(f"  [{count}/{total_points}] x={x_mm:.2f} z={z_mm:.2f}"
                      f"  R={avg['R']:.3e}  ETA:{eta/60:.1f}min", end="\r")

        # incremental save also at end
        _save_slice(config["output_dir"], delay_ps, x_positions, z_positions, data)

    except KeyboardInterrupt:
        print("\n\n*** Scan interrupted by user ***")
    finally:
        np.savez(
            os.path.join(config["output_dir"], "mapping_full.npz"),
            data=data,
            x_positions=x_positions,
            z_positions=z_positions,
            delay_label_ps=delay_ps,
            config=str(config),
        )
        print(f"\nData saved to {config['output_dir']}/")

        stage.close()
        bbd.close()
        lockin.close()
        if rm is not None: rm.close()

    return data, x_positions, z_positions, delay_ps


def _save_slice(output_dir, delay_ps, x_positions, z_positions, slice_data):
    """Save the XZ map at fixed delay as tab-separated text file."""
    fname = os.path.join(output_dir, f"map_t{delay_ps:06.0f}ps.dat")
    with open(fname, "w") as f:
        f.write(f"# Delay: {delay_ps} ps (set in LabVIEW)\n")
        f.write("X(mm)\tZ(mm)\tSignal_X(V)\tSignal_Y(V)\tR(V)\tTheta(deg)\n")
        for i_x, x in enumerate(x_positions):
            for i_z, z in enumerate(z_positions):
                vals = slice_data[i_x, i_z]
                f.write(f"{x:.2f}\t{z:.2f}\t{vals[0]:.6e}\t{vals[1]:.6e}\t{vals[2]:.6e}\t{vals[3]:.4f}\n")


# ============================================================
# Visualization (run on Mac after data collection)
# ============================================================
def plot_mapping(npz_path):
    """Load saved data and plot XZ heatmap or timescan, dispatching by file content."""
    import matplotlib.pyplot as plt

    d = np.load(npz_path, allow_pickle=True)
    keys = set(d.files)

    if "t_ps" in keys:
        t_ps = d["t_ps"]
        data = d["data"]
        fig, axes = plt.subplots(2, 1, figsize=(7, 6), sharex=True)
        axes[0].plot(t_ps, data[:, 2])
        axes[0].set_ylabel("R (V)")
        axes[0].grid(True, alpha=0.3)
        axes[1].plot(t_ps, data[:, 0], label="X")
        axes[1].plot(t_ps, data[:, 1], label="Y")
        axes[1].set_xlabel("Delay (ps)")
        axes[1].set_ylabel("Signal (V)")
        axes[1].legend()
        axes[1].grid(True, alpha=0.3)
        fig.suptitle(os.path.basename(npz_path))
    else:
        data = d["data"]
        x_pos = d["x_positions"]
        z_pos = d["z_positions"]
        delay_ps = d["delay_label_ps"]
        fig, ax = plt.subplots(figsize=(6, 5))
        im = ax.pcolormesh(z_pos, x_pos, data[:, :, 2], shading="auto", cmap="RdBu_r")
        ax.set_title(f"XZ map at delay = {delay_ps} ps")
        ax.set_xlabel("Z (mm)")
        ax.set_ylabel("X (mm)")
        fig.colorbar(im, ax=ax, label="R (V)")

    plt.tight_layout()
    out = npz_path.replace(".npz", "_overview.png")
    plt.savefig(out, dpi=200)
    plt.show()
    print(f"Saved {out}")


# ============================================================
# Entry points
# ============================================================
if __name__ == "__main__":
    args = sys.argv[1:]
    for i, a in enumerate(list(args)):
        if a.startswith("--port="):
            CONFIG["shot602_port"] = a.split("=", 1)[1]
        elif a == "--port" and i + 1 < len(args):
            CONFIG["shot602_port"] = args[i + 1]
    if "--mock-lockin" in args:
        CONFIG["mock_lockin"] = True
    if "--mock" in args:
        CONFIG["mock"] = True
    args = [a for a in args
            if not a.startswith("--port")
            and a != CONFIG["shot602_port"]
            and a not in ("--mock", "--mock-lockin")]

    if args and args[0] == "scan":
        scan_devices()

    elif args and args[0] == "timescan":
        run_timescan()

    elif args and args[0] == "run":
        run_mapping()

    elif args and args[0] == "plot":
        npz_file = args[1] if len(args) > 1 else "mapping_full.npz"
        plot_mapping(npz_file)

    else:
        print("Usage:")
        print("  python auto_mapping.py scan [--port COMx]              # Probe GPIB + serial + SHOT-602 + BBD")
        print("  python auto_mapping.py timescan [--mock]               # Sweep BBD delay at fixed XZ, read lock-in")
        print("  python auto_mapping.py run  [--mock] [--port COMx]     # Full XZ map at one BBD delay")
        print("  python auto_mapping.py plot [file]                     # Plot saved .npz")
        print()
        print("  --mock        : all-synthetic devices (no hardware) — offline pipeline tests")
        print("  --mock-lockin : REAL BBD + REAL stage + simulated lock-in — probe motion pipeline when lock-in is unavailable")
