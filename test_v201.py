"""v2.0.1 checks (mock only): a delay line remembers only a scan range that fits its own travel and was
set on it. Reproduces a settings file seen in use: ID1 (300 mm) lost its 71,100 um Start to a
549,000 typed before switching to ID0, and ID2 (220 mm) kept ID0's 549,390 -> 100,000 from its first use.
Also: a "Device Not Opened" connect line must not tell people to check USB / power. ASCII prints only."""
import os, sys, json, tempfile
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
TMP = tempfile.mkdtemp(prefix="v201test_")
os.environ["MOKE_LOG_DIR"] = TMP
os.environ["MOKE_SETTINGS"] = os.path.join(TMP, "settings.json")
base = json.load(open(os.path.join(HERE, "test_fixture_settings.json"), encoding="utf-8"))
base["cfg.bbd_serial"] = "10000000"
json.dump(base, open(os.environ["MOKE_SETTINGS"], "w", encoding="utf-8"))
import moke_gui as mg

fails = []
def check(cond, msg):
    print(("PASS " if cond else "FAIL ") + msg)
    if not cond: fails.append(msg)

def scan(app, s, e, st):
    app.ts_vars["time_start_ps"].set(s); app.ts_vars["time_end_ps"].set(e); app.ts_vars["time_step_ps"].set(st)

def pick(app, name):
    app.stage_sel_var.set(name); app._on_stage_selected()

def prof(app, i):
    return app.stages[i]

def hook(app):
    app.withdraw()
    n = [p["name"] for p in app.stages]
    app.ts_unit.set("µm")
    scan(app, "549390", "100000", "1000")                       # ID0

    # first use of ID2: ID0's numbers are carried over (red). Leaving untouched must not save them.
    # Re-writing the same Start (as loading the settings on launch does) re-anchors t0 = 549.39 mm,
    # which is how ID2 got t0 549.39 / calibrated in that file.
    pick(app, n[2]); app.ts_vars["time_start_ps"].set("549390"); pick(app, n[0])
    p2 = prof(app, 2)
    check("scan" not in p2, f"ID2: carried-over ID0 range not saved as its own: {p2.get('scan')}")
    check(float(p2["bbd_t0_mm"]) == 0.0 and not p2["calibrated"],
          f"ID2: t0 / calibrated untouched: t0={p2['bbd_t0_mm']} calibrated={p2['calibrated']}")
    pick(app, n[2])
    check("First use" in app.status_var.get(), "ID2: still 'first use' next time")
    pick(app, n[0])

    # ID1 gets its own range (inside 300 mm), then someone types an ID0 Start before switching
    pick(app, n[1]); scan(app, "71100", "75000", "2")
    pick(app, n[0]); pick(app, n[1])
    check(app.ts_vars["time_start_ps"].get() == "71100", "ID1: its range comes back")
    app.ts_vars["time_start_ps"].set("549000")                  # typed while ID1 is still selected
    check(str(app.ts_entries["time_start_ps"].cget("style")) == "Bad.TEntry", "ID1: 549000 is red")
    pick(app, n[0])
    p1 = prof(app, 1)
    check(p1.get("scan", {}).get("time_start_ps") == "71100",
          f"ID1: out-of-travel Start not saved over 71100: {p1.get('scan')}")
    check(abs(float(p1["bbd_t0_mm"]) - 71.1) < 1e-9, f"ID1: t0 stays 71.1 mm: {p1['bbd_t0_mm']}")
    check(app.ts_vars["time_start_ps"].get() == "549390", "ID0: its own range is back")
    pick(app, n[1])
    check(app.ts_vars["time_start_ps"].get() == "71100" and app.ts_vars["time_step_ps"].get() == "2",
          "ID1: 71100 / step 2 come back after the switch")

    # the settings file written while ID1 shows a red range keeps the good one
    app.ts_vars["time_start_ps"].set("549000")
    app._save_settings()
    saved = json.load(open(os.environ["MOKE_SETTINGS"], encoding="utf-8"))
    s1 = [p for p in saved["stages"] if p["bbd_serial"] == "10000001"][0]
    check(s1["scan"]["time_start_ps"] == "71100" and abs(s1["bbd_t0_mm"] - 71.1) < 1e-9,
          f"settings file: ID1 keeps 71100 / 71.1 mm: {s1['scan']} t0={s1['bbd_t0_mm']}")
    app.ts_vars["time_start_ps"].set("71100")

    # a range that fits and was typed on the stage is still saved (v2.0 behaviour)
    pick(app, n[2]); scan(app, "150000", "50000", "1000"); pick(app, n[0])
    check(prof(app, 2).get("scan", {}).get("time_start_ps") == "150000", "ID2: an in-travel range typed there is saved")

    # connect line wording for a delay line held by another program
    line = app._bbd_fail_line("FAILED: ThorlabsBackendError: backend exception: D2XXException(3) ('Device Not Opened')")
    check("another program" in line and "USB + power" not in line, f"'Device Not Opened' -> in use elsewhere: {line[-70:]!r}")
    line = app._bbd_fail_line("FAILED: ThorlabsBackendError: backend exception: D2XXException(2) ('Device Not Found')")
    check("USB + power" in line, "other failures still point at USB / power")

mg.run_app(test_hook=hook)
print("\nRESULT:", "ALL PASS" if not fails else f"{len(fails)} FAIL")
sys.exit(1 if fails else 0)
