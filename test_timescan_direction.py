"""The 2026-07-27 'the timescan runs the wrong way' bug.

Reported: Scan Start = 549390 um, End = 0 um -> the delay line walked UP
(549390 -> 600000 ...) instead of down, and a negative Step (-1000) changed
nothing. The saved config of timescan_20260727_131927_partial proves the GUI
side was right (bbd_reverse=True, t0=549.39 mm, ps 0 -> +3665) -- so the only
way up is the delay-line OBJECT using the opposite slope, i.e.
DeviceHub.connect building the real ThorlabsDelay without reverse=.

ASCII only in the prints: the lab PC console is cp1252 and dies on CJK.
"""
import os, sys, shutil, inspect, queue, threading

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np
import auto_mapping as am
import moke_gui

FAILED = []


def check(name, cond, extra=""):
    print(f"  {'PASS' if cond else 'FAIL'}  {name}  {extra}")
    if not cond:
        FAILED.append(name)


# ---------------------------------------------------------------- fakes
class FakeBBD:
    """Same constructor contract as am.ThorlabsDelay, no hardware, records moves."""

    C_MM_PER_PS = 0.299792458

    def __init__(self, serial_number="x", counts_per_mm=20000, is_rack_system=True,
                 channel=1, t0_mm=0.0, double_pass=True, reverse=False, port=None):
        self.t0_mm = float(t0_mm)
        self.mm_per_ps = self.C_MM_PER_PS / (2.0 if double_pass else 1.0)
        if reverse:
            self.mm_per_ps = -self.mm_per_ps
        self._pos_mm = self.t0_mm
        self.visited_um = []

    def ps_to_mm(self, ps):    return self.t0_mm + ps * self.mm_per_ps
    def mm_to_ps(self, mm):    return (mm - self.t0_mm) / self.mm_per_ps
    def get_position_mm(self): return self._pos_mm
    def get_position_ps(self): return self.mm_to_ps(self._pos_mm)

    def move_to_mm(self, mm, wait=True, timeout=60.0):
        self._pos_mm = float(mm)
        self.visited_um.append(self._pos_mm * 1000.0)

    def move_to_ps(self, ps, wait=True, timeout=60.0):
        self.move_to_mm(self.ps_to_mm(ps))

    def is_moving(self): return False
    def close(self):     pass


class FakeLockIn:
    def read_all(self): return {"X": 1e-6, "Y": 0.0, "R": 1e-6, "theta": 0.0}
    def close(self):    pass


def real_hub(cfg):
    """A DeviceHub opened through the REAL-hardware branch with the delay line
    swapped for FakeBBD. The lock-in/stage opens fail here (no GPIB); connect()
    reports that per device instead of raising, which is what we want."""
    hub = moke_gui.DeviceHub()
    real_cls = am.ThorlabsDelay
    am.ThorlabsDelay = FakeBBD
    try:
        hub.connect(cfg, "real", need_stage=False)
    finally:
        am.ThorlabsDelay = real_cls
    hub.lockin = FakeLockIn()
    return hub


OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "moke_data", "_test_direction")
cfg = dict(am.CONFIG)
cfg.update(bbd_t0_mm=549.390, bbd_double_pass=True, bbd_reverse=True,
           bbd_settle_s=0.0, num_averages=1, bbd_travel_mm=800.0, data_dir=OUT)

# ---------------------------------------------------------------- the bug
print("=== 1. the real delay line is built with the configured reverse ===")
src = inspect.getsource(moke_gui.DeviceHub.connect)
real_call = src.split("am.ThorlabsDelay(")[1].split(")")[0]
check("DeviceHub.connect passes reverse= to the real ThorlabsDelay",
      "reverse" in real_call,
      "" if "reverse" in real_call else "reverse NOT in the real BBD constructor call")

hub = real_hub(cfg)
check("its slope has the sign the GUI converts um<->ps with",
      np.sign(hub.bbd.mm_per_ps) == np.sign(moke_gui.mm_per_ps_for(cfg)),
      f"bbd {hub.bbd.mm_per_ps:+.6f} mm/ps  vs  gui {moke_gui.mm_per_ps_for(cfg):+.6f} mm/ps")

print("=== 2. Typical range: Start 549390 um -> End 0 um, Step 1000 um ===")
t0_mm, sp, ep, stp = moke_gui.plan_timescan(cfg, "um", 549390.0, 0.0, 1000.0)
check("delay 0 is anchored to the scan START", abs(t0_mm - 549.390) < 1e-9, f"t0 = {t0_mm} mm")
check("the scan starts at 0 ps", abs(sp) < 1e-9, f"{sp:.4f} ps")
check("position 0 um is the LONGEST delay (+3665 ps), not a negative one",
      3660 < ep < 3670, f"{ep:.1f} ps")
check("step is 6.671 ps per 1000 um", abs(stp - 6.6713) < 1e-3, f"{stp:.4f} ps")

print("=== 3. end to end: the stage walks DOWN 549390 -> 0 um ===")
shutil.rmtree(OUT, ignore_errors=True)
scan = dict(cfg)
scan.update(bbd_t0_mm=t0_mm, time_start_ps=sp, time_end_ps=ep, time_step_ps=stp)
hub = real_hub(scan)
t_ps, data = moke_gui.timescan_loop(scan, hub, queue.Queue(), threading.Event(), after_scan="stay")
um = hub.bbd.visited_um
check("the scan ran to completion", t_ps is not None and len(t_ps) == 550,
      f"{0 if t_ps is None else len(t_ps)} points")
check("first point is the Start box (549390 um)", abs(um[0] - 549390) < 1, f"{um[0]:,.0f} um")
# 549390 / 1000 is not a whole number of steps, so the sweep stops one short of
# the End box rather than taking a ragged final step. It must never overshoot it.
check("last point is within one step of the End box (0 um), and never past it",
      -1 < um[-1] < 1000 + 1, f"{um[-1]:,.0f} um")
check("every step moves DOWN (the reported bug: it went UP)",
      all(b < a for a, b in zip(um, um[1:])),
      f"{um[0]:,.0f} -> {um[1]:,.0f} -> {um[2]:,.0f} um")
check("the stage never leaves its travel", max(um) <= 800000 and min(um) >= -1,
      f"{min(um):,.0f} .. {max(um):,.0f} um")

print("=== 4. a NEGATIVE Step (-1000) scans the same way, not a different one ===")
_, sp2, ep2, stp2 = moke_gui.plan_timescan(cfg, "um", 549390.0, 0.0, -1000.0)
check("Step -1000 gives the same sweep as Step 1000",
      abs(sp2 - sp) < 1e-9 and abs(ep2 - ep) < 1e-9 and abs(stp2 - stp) < 1e-9,
      f"{sp2:.1f} -> {ep2:.1f} step {stp2:.4f} ps")

print("=== 5. the saved CSV position column matches where the stage really went ===")
outdirs = sorted(d for d in os.listdir(moke_gui._data_root(scan)) if d.startswith("timescan_"))
rows = open(os.path.join(moke_gui._data_root(scan), outdirs[-1], "timescan.csv")).read().splitlines()
first, last = rows[2].split(","), rows[-1].split(",")
check("CSV row 1 position = 549390 um", abs(float(first[2]) - 549390) < 1, f"{first[2]} um")
check("CSV last position = where the stage actually ended",
      abs(float(last[2]) - um[-1]) < 1, f"csv {last[2]} um  vs  stage {um[-1]:,.0f} um")
check("the CSV position column counts DOWN like the stage did",
      float(rows[3].split(",")[2]) < float(rows[2].split(",")[2]),
      f"{rows[2].split(',')[2]} -> {rows[3].split(',')[2]} um")

print("=== 6. a sweep that leaves the stage travel is refused, not driven into the limit ===")
bad = moke_gui.timescan_out_of_travel(scan, 0.0, 6000.0)     # -350 mm: off the far end
check("6000 ps (off the stage) is flagged", bad is not None, str(bad))
check("the real sweep 0 -> 3665 ps is inside travel",
      moke_gui.timescan_out_of_travel(scan, sp, ep) is None,
      str(moke_gui.timescan_out_of_travel(scan, sp, ep)))

shutil.rmtree(OUT, ignore_errors=True)
print("\n" + ("ALL PASSED" if not FAILED else f"FAILED: {FAILED}"))
sys.exit(1 if FAILED else 0)
