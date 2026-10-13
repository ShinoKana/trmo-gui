"""Mock test of delay line 2 (SHOT axis) swept with the BBD — three ways of scanning.
No hardware. Run: python test_dual_delay.py"""
import os, sys, copy, queue, threading, tempfile, shutil, json
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import numpy as np
import auto_mapping as am
import moke_gui as mg

TMP = tempfile.mkdtemp(prefix="dualtest_")
os.environ["MOKE_LOG_DIR"] = TMP   # never write into the live GUI log
fails = []
def check(cond, msg):
    print(("PASS " if cond else "FAIL ") + msg)
    if not cond: fails.append(msg)

def make_hub():
    cfg = copy.deepcopy(am.CONFIG)
    cfg.update(mock=True, data_dir=TMP, bbd_t0_mm=100.0, bbd_double_pass=True, bbd_reverse=False,
               bbd_settle_s=0.0, num_averages=1, pulses_per_mm=500, signal="X",
               dl2_axis=2, dl2_pulses_per_mm=1000, dl2_settle_s=0.0)
    hub = mg.DeviceHub(); hub.connect(cfg, "mock", False, True)
    moves = []                                   # record every delay-line-2 command
    orig = hub.pump.move_abs
    def spy(axis, mm):
        moves.append((axis, mm)); orig(axis, mm)
    hub.pump.move_abs = spy
    hub.pump.move_abs(2, 7.0)                   # delay line 2 starts somewhere (7.0 mm-equiv)
    moves.clear()
    return cfg, hub, moves

def run(cfg, hub):
    q = queue.Queue(); stop = threading.Event()
    t, data = mg.timescan_loop(cfg, hub, q, stop, repetition=1, after_scan="stay")
    msgs = []
    while not q.empty(): msgs.append(q.get())
    return t, data, msgs

ratio = 1000 / 500   # dl2 pulses/mm over the SHOT driver's pulses/mm

# 1) delay line 1 scans, delay line 2 parked (Start = End)
cfg, hub, moves = make_hub()
cfg.update(time_start_ps=0.0, time_end_ps=10.0, time_step_ps=1.0,
           dl2_enabled=True, dl2_start_um=2000.0, dl2_end_um=2000.0, dl2_step_um=100.0)
t, data, msgs = run(cfg, hub)
check(t is not None and len(t) == 11, "mode 1: delay line 1 scans 11 points")
scan_moves = moves[:-1]
check(len(scan_moves) == 1 and abs(scan_moves[0][1] - 2.0 * ratio) < 1e-9 and scan_moves[0][0] == 2,
      f"mode 1: delay line 2 moved once, to 2000 um on axis 2 (moves={scan_moves})")
check(moves[-1] == (2, 7.0), "mode 1: delay line 2 restored to where it started")

# 2) both move together
cfg, hub, moves = make_hub()
cfg.update(time_start_ps=0.0, time_end_ps=10.0, time_step_ps=1.0,
           dl2_enabled=True, dl2_start_um=1000.0, dl2_end_um=11000.0, dl2_step_um=100.0)
t, data, msgs = run(cfg, hub)
um = [m[1] / ratio * 1000 for m in moves[:-1]]
check(len(um) == 11 and np.allclose(um, np.linspace(1000, 11000, 11)), "mode 2: delay line 2 steps 1000->11000 um with delay line 1 (11 points)")
done = [m for m in msgs if m.get("kind") == "done"][-1]
z = np.load(os.path.join(done["dir"], "timescan.npz"))
check("dl2_um" in z and np.allclose(z["dl2_um"], np.linspace(1000, 11000, 11)), "mode 2: npz stores delay line 2 positions")
rows = open(os.path.join(done["dir"], "timescan.csv")).read().splitlines()
check(rows[1].endswith(",dl2_um") and rows[-1].endswith(",11000.00"), "mode 2: CSV has a dl2_um column")

# 3) delay line 1 fixed, only delay line 2 scans (its step sets the points)
cfg, hub, moves = make_hub()
cfg.update(time_start_ps=5.0, time_end_ps=5.0, time_step_ps=1.0,
           dl2_enabled=True, dl2_start_um=0.0, dl2_end_um=1500.0, dl2_step_um=300.0)
t, data, msgs = run(cfg, hub)
check(t is not None and len(t) == 6 and np.allclose(t, 5.0), "mode 3: 6 points, delay line 1 held at 5 ps")
begin = [m for m in msgs if m.get("kind") == "begin_timescan"][0]
check(np.allclose(begin["t_ps"], np.arange(6) * 300 / 1000 / (0.299792458 / 2)), "mode 3: plot axis = delay line 2's delay (ps) from its start")
done = [m for m in msgs if m.get("kind") == "done"][-1]
rows = open(os.path.join(done["dir"], "timescan.csv")).read().splitlines()
pos = {r.split(",")[1] for r in rows[2:]}
check(len(pos) == 1, f"mode 3: CSV position_mm = delay line 1's real (constant) position {pos}")

# 4) delay line 2 off -> identical to the old single-delay behaviour (no extra column / key)
cfg, hub, moves = make_hub()
cfg.update(time_start_ps=0.0, time_end_ps=3.0, time_step_ps=1.0, dl2_enabled=False)
t, data, msgs = run(cfg, hub)
done = [m for m in msgs if m.get("kind") == "done"][-1]
rows = open(os.path.join(done["dir"], "timescan.csv")).read().splitlines()
z = np.load(os.path.join(done["dir"], "timescan.npz"))
check(moves == [] and rows[1] == "t_ps,position_mm,position_um,X_V,Y_V,R_V,theta_deg" and "dl2_um" not in z,
      "off: no delay-line-2 moves, CSV/npz format unchanged")

# 5) profiles saved by the first deployment (old names) are renamed by serial, calibration kept
SET = os.path.join(TMP, "settings.json")
base = json.load(open(os.path.join(HERE, "test_fixture_settings.json")))
base["stages"] = [{"name": "Stage A  (DDS600, 10000000)", "bbd_serial": "10000000", "bbd_t0_mm": 549.39, "calibrated": True},
                  {"name": "New / middle  (DDS220, 10000002)", "bbd_serial": "10000002", "bbd_t0_mm": 1.5, "calibrated": True},
                  {"name": "THz TDS  (DDS300, 10000001)", "bbd_serial": "10000001", "bbd_t0_mm": 0.0, "calibrated": False}]
base["active_stage"] = "New / middle  (DDS220, 10000002)"
json.dump(base, open(SET, "w"))
def hook(app):
    app._settings_path = lambda: SET; app.withdraw()
    app._load_settings(); app._apply_settings(); app._init_stages()
    names = [p["name"] for p in app.stages]
    check(names == [mg.STAGE_NAMES[k] for k in ("10000000", "10000001", "10000002")], "old names renamed + LabVIEW ID order 0/1/2")
    check(app.config_data["bbd_serial"] == "10000002" and abs(app.config_data["bbd_t0_mm"] - 1.5) < 1e-9,
          "active stage kept by serial, its calibration (t0 1.5) kept")
    app.dl2_on.set(True); app.dl2_vars["dl2_end_um"].set("4321"); app._save_settings()
    saved = json.load(open(SET))
    check(saved.get("ts.dl2_on") is True and saved.get("ts.dl2_end_um") == "4321", "delay line 2 panel is saved")
mg.run_app(test_hook=hook)

# 6) pump delay manual control + per-stage scan range + USB hint (GUI, mock hardware)
SET2 = os.path.join(TMP, "settings2.json")
_b2 = json.load(open(os.path.join(HERE, "test_fixture_settings.json"), encoding="utf-8"))
for _k in ("stages", "active_stage"): _b2.pop(_k, None)
_b2["cfg.bbd_serial"] = "10000000"
json.dump(_b2, open(SET2, "w", encoding="utf-8"))
import time as _t
def wait_idle(app, s=10):
    t0 = _t.time()
    while app._hw_busy and _t.time() - t0 < s: _t.sleep(0.02)
def hook2(app):
    app._settings_path = lambda: SET2; app.withdraw()
    app._load_settings(); app._apply_settings(); app._init_stages()
    cfg = dict(app.config_data); cfg["mock"] = True
    app.hub.connect(cfg, "mock", False, True)
    app.dl2_vars["dl2_axis"].set("2"); app.dl2_vars["dl2_pulses_per_mm"].set("1000")
    app.pump_target_um.set("2500"); app._pump_move_target(); wait_idle(app); app._pump_read()
    check("2,500.0" in app.pump_pos_var.get(), f"pump: Move to 2500 um -> reads back {app.pump_pos_var.get()}")
    app.pump_jog_um.set("10"); app._pump_jog(+1); wait_idle(app); app._pump_read()
    check("2,510.0" in app.pump_pos_var.get(), f"pump: jog +10 um -> {app.pump_pos_var.get()}")
    app.dl2_vars["dl2_start_um"].set("100"); app._pump_go("dl2_start_um"); wait_idle(app); app._pump_read()
    check("100.0" in app.pump_pos_var.get(), f"pump: -> Pump Start (100 um) -> {app.pump_pos_var.get()}")
    app.hub.close()          # disconnected: switching is synchronous (connected -> async swap, tested in 7)
    names = [p["name"] for p in app.stages]
    # each stage keeps its own scan range
    app.ts_vars["time_start_ps"].set("549390"); app.ts_vars["time_end_ps"].set("100000")
    app.stage_sel_var.set(names[1]); app._on_stage_selected()
    app.ts_vars["time_start_ps"].set("12000"); app.ts_vars["time_end_ps"].set("2000")
    app.stage_sel_var.set(names[0]); app._on_stage_selected()
    check(app.ts_vars["time_start_ps"].get() == "549390" and app.ts_vars["time_end_ps"].get() == "100000",
          "scan range: ID0 gets its own 549390 -> 100000 back")
    app.stage_sel_var.set(names[1]); app._on_stage_selected()
    check(app.ts_vars["time_start_ps"].get() == "12000", "scan range: ID1 gets its own 12000 back")
    check("ID1" in app.cfg_stage_lbl.get(), "Settings tab says which delay line the fields belong to")
    # USB hint: ID1 and ID2 plugged in, ID0 not
    app._handle({"kind": "usb_stages", "serials": ["10000001", "10000002"]})
    check(app.stage_usb_var.get().startswith("\u2713"), f"USB hint for ID1: {app.stage_usb_var.get()!r}")
    app.stage_sel_var.set(names[0]); app._on_stage_selected()
    check(app.stage_usb_var.get().startswith("checking"), "switching marks the USB state stale (rescan)")
    app._handle({"kind": "usb_stages", "serials": ["10000001", "10000002"]})   # the rescan's answer
    check("not found" in app.stage_usb_var.get() and "ID1" in app.stage_usb_var.get(), f"USB hint for ID0: {app.stage_usb_var.get()!r}")
    app._handle({"kind": "usb_stages", "serials": ["10000000", "10000002"]})   # cables changed: ID0 back
    check(app.stage_usb_var.get().startswith("\u2713"), "USB hint follows a cable change (ID0 now on USB)")
    # names follow the selected stage; mapping is only for ID0
    check("ID0" in app.scan_frame.cget("text") and "ID0" in app.main_ctrl_frame.cget("text"), "frame titles name the selected delay line")
    check(app._mapping_block_reason() is None, "mapping allowed on ID0")
    app.stage_sel_var.set(names[1]); app._on_stage_selected()
    check(app._mapping_block_reason() is not None, "mapping blocked when ID1 is selected")
    # pump travel limits
    check(app._pump_limit_error([99999.0]) is None and "not set" in app.pump_lim_note.get(), "pump limits blank -> no check, says 'not set'")
    app.pump_lim_vars["pump_min_um"].set("0"); app.pump_lim_vars["pump_max_um"].set("5000")
    check(app._pump_limit_error([6000.0]) is not None and app._pump_limit_error([0.0, 5000.0]) is None and app.pump_lim_note.get() == "",
          "pump limits 0..5000: 6000 refused, 0 and 5000 allowed")
    app._save_settings(); saved = json.load(open(SET2))
    check(saved.get("ts.pump_max_um") == "5000", "pump limits are saved")
mg.run_app(test_hook=hook2)

# 7) real mode (fake hardware): switching swaps ONLY the delay line
class FakeBBD:
    opened, closed = [], []
    def __init__(self, serial_number=None, **k):
        self.sn = str(serial_number); self.t0_mm = 0.0; self.mm_per_ps = 0.15; FakeBBD.opened.append(self.sn)
    def close(self): FakeBBD.closed.append(self.sn)
    def get_position_mm(self): return 12.5
    def get_position_ps(self): return 0.0
SET3 = os.path.join(TMP, "settings3.json")
json.dump(_b2, open(SET3, "w", encoding="utf-8"))
def hook3(app):
    app._settings_path = lambda: SET3; app.withdraw()
    app._load_settings(); app._apply_settings(); app._init_stages()
    real = mg.am.ThorlabsDelay; mg.am.ThorlabsDelay = FakeBBD
    try:
        lock_obj, xz_obj, pump_obj = object(), object(), object()
        app.hub.mode = "real"; app.hub.bbd = FakeBBD("10000000"); FakeBBD.opened.clear()
        app.hub.lockin, app.hub.stage, app.hub.pump = lock_obj, xz_obj, pump_obj
        names = [p["name"] for p in app.stages]
        app.stage_sel_var.set(names[1]); app._on_stage_selected()
        import time as _t, queue as _q
        t0 = _t.time(); got = None
        while _t.time() - t0 < 10 and got is None:
            try:
                m = app.out_q.get(timeout=0.2)
            except _q.Empty:
                continue
            app._handle(m)
            if m.get("kind") == "bbd_swapped": got = m
        check(got is not None and got["status"] == "ok", f"swap reported ok: {got}")
        check(FakeBBD.closed[-1:] == ["10000000"] and FakeBBD.opened == ["10000001"], f"old ID0 closed, ID1 opened ({FakeBBD.closed}, {FakeBBD.opened})")
        check(app.hub.lockin is lock_obj and app.hub.stage is xz_obj and app.hub.pump is pump_obj, "lock-in / XZ / pump untouched")
        check(app._conn_status.get("bbd") == "ok" and not app._connecting and not app._hw_busy, "lights + busy flags restored")
    finally:
        mg.am.ThorlabsDelay = real
        app.hub.lockin = app.hub.stage = app.hub.pump = None
mg.run_app(test_hook=hook3)

print("\nRESULT:", "ALL PASS" if not fails else f"{len(fails)} FAIL")
shutil.rmtree(TMP, ignore_errors=True)
sys.exit(1 if fails else 0)
