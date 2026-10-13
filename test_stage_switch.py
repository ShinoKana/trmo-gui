"""Headless test of the delay-line profiles (Mock only, no hardware). Run: python test_stage_switch.py
Uses a COPY of the settings file in a temp dir so the real moke_gui_settings.json is never touched."""
import os, sys, json, shutil, tempfile
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import numpy as np
import moke_gui as mg

TMP = tempfile.mkdtemp(prefix="stagetest_")
SETTINGS = os.path.join(TMP, "moke_gui_settings.json")
_base = json.load(open(os.path.join(HERE, "test_fixture_settings.json"), encoding="utf-8"))
for _k in ("stages", "active_stage"): _base.pop(_k, None)      # start as a first run, whatever the live file holds
_base["cfg.bbd_serial"] = "10000000"; _base["cfg.bbd_t0_mm"] = "549.39"
_base["cfg.bbd_reverse"] = "True"; _base["cfg.bbd_double_pass"] = "True"
json.dump(_base, open(SETTINGS, "w", encoding="utf-8"))
os.environ["MOKE_LOG_DIR"] = TMP   # never write into the live GUI log
fails = []
def check(cond, msg):
    print(("PASS " if cond else "FAIL ") + msg)
    if not cond: fails.append(msg)

def hook_factory(fn):
    def hook(app):
        app._settings_path = lambda: SETTINGS        # never write the real settings file
        app.withdraw()
        fn(app)
    return hook

NAMES = [p["name"] for p in mg.DEFAULT_STAGES]

def run1(app):
    # settings were loaded from the default path before the hook; reload from the temp copy
    app._load_settings(); app._apply_settings(); app._init_stages()
    check([p["name"] for p in app.stages] == NAMES, "three default profiles built")
    check(app.active_stage == NAMES[0], "active = Stage A (matches saved serial 10000000)")
    check(abs(app.config_data["bbd_t0_mm"] - 549.39) < 1e-6, f"Stage A keeps existing t0 549.39 (got {app.config_data['bbd_t0_mm']})")
    check(app.stages[0]["bbd_reverse"] is True and app.stages[0]["bbd_double_pass"] is True, "Stage A keeps reverse/double-pass")
    # switch to the new middle stage
    app.stage_sel_var.set(NAMES[1]); app._on_stage_selected()
    c = app.config_data
    check(c["bbd_serial"] == "10000001" and c["bbd_travel_mm"] == 300.0 and c["bbd_t0_mm"] == 0.0, "switched config to ID1 DDS300 / 10000001")
    check(app.cfg_vars["bbd_serial"].get() == "10000001", "Settings tab shows the new serial")
    check("not calibrated" in app.status_var.get() if hasattr(app, "status_var") else True, "warns: stage not calibrated")
    check(c.get("stage_name") == NAMES[1], "stage_name recorded in config")
    # calibrate the middle stage, then switch away and back.
    # ps mode: t0 is an independent calibration (in um mode the Start box IS t0, see below)
    app.ts_unit.set("ps")
    app.cfg_vars["bbd_t0_mm"].set("12.3456"); app._apply_settings()
    app.stage_sel_var.set(NAMES[0]); app._on_stage_selected()
    check(abs(app.config_data["bbd_t0_mm"] - 549.39) < 1e-6, "back on Stage A: its t0 untouched")
    app.stage_sel_var.set(NAMES[1]); app._on_stage_selected()
    check(abs(app.config_data["bbd_t0_mm"] - 12.3456) < 1e-9, "middle stage remembers its own t0")
    check(app._active_profile()["calibrated"] is True, "middle stage marked calibrated after t0 set")
    # um mode (rule: the Start box is 0 ps): switching restores that stage's Start, so t0 = its Start
    app.ts_unit.set("µm"); app.ts_vars["time_start_ps"].set("200000"); app.ts_vars["time_end_ps"].set("150000")
    app.stage_sel_var.set(NAMES[0]); app._on_stage_selected()
    app.stage_sel_var.set(NAMES[1]); app._on_stage_selected()
    check(app.ts_vars["time_start_ps"].get() == "200000" and abs(app.config_data["bbd_t0_mm"] - 200.0) < 1e-9,
          "um mode: switching back restores Start 200000 and t0 = Start (200 mm)")
    app.ts_unit.set("ps"); app.cfg_vars["bbd_t0_mm"].set("12.3456"); app._apply_settings()
    # switching while connected releases the controller (mock connect)
    app.hub.connect(app.config_data, "mock", False)
    check(app.hub.connected, "mock connect ok on middle stage")
    old_bbd, old_lockin = app.hub.bbd, app.hub.lockin
    app.stage_sel_var.set(NAMES[2]); app._on_stage_selected()
    import time as _t, queue as _q
    t0 = _t.time(); seen = False
    while _t.time() - t0 < 10 and not seen:          # the swap runs in a thread; drain its messages
        try:
            m = app.out_q.get(timeout=0.2)
        except _q.Empty:
            continue
        app._handle(m); seen = seen or m.get("kind") == "bbd_swapped"
    check(seen and app.hub.connected and app.hub.bbd is not old_bbd and app.hub.lockin is old_lockin,
          "switching while connected swaps only the delay line (lock-in kept)")
    check(app.config_data["bbd_serial"] == "10000002" and app.config_data["bbd_travel_mm"] == 220.0, "switched to ID2 DDS220 / 10000002")
    # travel guard follows the active stage
    over = mg.timescan_out_of_travel(app.config_data, 0, 1600)   # 1600 ps = 240 mm > 220 mm
    ok300 = mg.timescan_out_of_travel(dict(app.config_data, bbd_travel_mm=300.0), 0, 1600)
    check(bool(over) and ok300 is None, "out-of-travel check uses this stage's 220 mm")
    app._save_settings()

def run2(app):
    app._load_settings(); app._apply_settings(); app._init_stages()
    check(app.active_stage == NAMES[2], "restart restores the last active stage")
    p1 = next(p for p in app.stages if p["name"] == NAMES[1])
    check(abs(p1["bbd_t0_mm"] - 12.3456) < 1e-9, "restart keeps per-stage t0")
    check(app.config_data["bbd_serial"] == "10000002", "config after restart = active stage")
    # data file records which delay line was used
    cfg = dict(app.config_data); cfg["data_dir"] = TMP
    out = mg._save_timescan(cfg, np.array([0.0, 1.0]), np.zeros((2, 4)))
    head = open(os.path.join(out, "timescan.csv")).read().splitlines()[0]
    check("10000002" in head and "DDS220" in head, f"CSV header names the delay line: {head[:80]}")

mg.run_app(test_hook=hook_factory(run1))
mg.run_app(test_hook=hook_factory(run2))
saved = json.load(open(SETTINGS))
check(len(saved.get("stages", [])) == 3 and saved.get("active_stage") == NAMES[2], "settings JSON holds stages + active_stage")
print("\nRESULT:", "ALL PASS" if not fails else f"{len(fails)} FAIL")
shutil.rmtree(TMP, ignore_errors=True)
sys.exit(1 if fails else 0)
