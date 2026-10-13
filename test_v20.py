"""v2.0 checks (mock only): travel shown + out-of-range highlight, per-stage TC/Sens,
'Start = here', version in the title, first-use hint. ASCII prints only."""
import os, sys, json, tempfile
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
TMP = tempfile.mkdtemp(prefix="v20test_")
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

def hook(app):
    app.withdraw()
    check(("v" + mg.__version__) in app.title(), f"title shows the version: {app.title()!r}")
    names = [p["name"] for p in app.stages]
    check(app.active_stage == names[0], "starts on ID0")
    app.ts_unit.set("µm")
    app.ts_vars["time_start_ps"].set("549390"); app.ts_vars["time_end_ps"].set("100000"); app.ts_vars["time_step_ps"].set("1000")
    check(str(app.ts_entries["time_start_ps"].cget("style")) != "Bad.TEntry", "ID0 (600 mm class): 549390 -> 100000 is fine")
    app.tc_var.set("200")
    # ID2 (220 mm) first use: the ID0 range is carried over and must turn red at once
    app.stage_sel_var.set(names[2]); app._on_stage_selected()
    check(str(app.ts_entries["time_start_ps"].cget("style")) == "Bad.TEntry" and app.ts_travel_var.get().startswith("⚠"),
          f"ID2: out-of-travel range is red: {app.ts_travel_var.get()!r}")
    check("First use" in app.status_var.get(), "first-use hint shown")
    app.ts_vars["time_start_ps"].set("150000"); app.ts_vars["time_end_ps"].set("50000")
    check(str(app.ts_entries["time_start_ps"].cget("style")) == "TEntry" and "220,000" in app.ts_travel_var.get(),
          f"ID2: in-range numbers clear the red, travel shown: {app.ts_travel_var.get()!r}")
    app.tc_var.set("1000")
    # back to ID0: its range AND its TC come back
    app.stage_sel_var.set(names[0]); app._on_stage_selected()
    check(app.ts_vars["time_start_ps"].get() == "549390" and app.tc_var.get() == "200", "ID0: range and TC 200 restored")
    app.stage_sel_var.set(names[2]); app._on_stage_selected()
    check(app.ts_vars["time_start_ps"].get() == "150000" and app.tc_var.get() == "1000", "ID2: its own range and TC 1000 restored")
    # Start = here (mock delay line)
    cfg = dict(app.config_data); cfg["mock"] = True
    app.hub.connect(cfg, "mock", False)
    app.hub.bbd.move_to_mm(123.456)
    app._ts_here("time_start_ps")
    check(app.ts_vars["time_start_ps"].get() == "123456", f"Start = here -> {app.ts_vars['time_start_ps'].get()}")
    app.hub.close()

mg.run_app(test_hook=hook)
print("\nRESULT:", "ALL PASS" if not fails else f"{len(fails)} FAIL")
sys.exit(1 if fails else 0)
