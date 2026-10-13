"""show-channel defaults + persistence. ASCII prints only (lab PC console is cp1252)."""
import os, sys, json, tempfile, shutil
os.environ["MOKE_MOCK"] = "1"
os.environ.setdefault("MOKE_LOG_DIR", tempfile.mkdtemp(prefix="moke_test_log_"))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import moke_gui

FAILED = []
def check(n, c, e=""):
    print(f"  {'PASS' if c else 'FAIL'}  {n}  {e}")
    if not c: FAILED.append(n)

SETTINGS = os.environ.get("MOKE_SETTINGS") or \
    os.path.join(os.path.dirname(os.path.abspath(__file__)), "moke_gui_settings.json")
backup = SETTINGS + ".showtest.bak"
if os.path.exists(SETTINGS):
    shutil.copy2(SETTINGS, backup)
    os.remove(SETTINGS)                 # a virgin install: no saved state

def fresh(app):
    app.withdraw()
    on = [c for c, v in app.ts_show.items() if v.get()]
    check("a fresh launch shows X only (R no longer rides along)", on == ["X"], str(on))
    app.ts_show["R"].set(True)          # user ticks R ...
    app.ts_show["theta"].set(True)      # ... and theta
    app._save_settings()

moke_gui.run_app(test_hook=fresh)

saved = json.load(open(SETTINGS))
check("the tick state is written to settings",
      saved.get("ts.show_R") is True and saved.get("ts.show_Y") is False,
      f"R={saved.get('ts.show_R')} Y={saved.get('ts.show_Y')} theta={saved.get('ts.show_theta')}")

def relaunch(app):
    app.withdraw()
    on = sorted(c for c, v in app.ts_show.items() if v.get())
    check("the next launch restores exactly what was ticked", on == ["R", "X", "theta"], str(on))

moke_gui.run_app(test_hook=relaunch)

if os.path.exists(backup):
    shutil.move(backup, SETTINGS)       # put the real settings back
elif os.path.exists(SETTINGS) and os.environ.get("MOKE_SETTINGS"):
    os.remove(SETTINGS)                 # the temp file this test created
print("\n" + ("ALL PASSED" if not FAILED else f"FAILED: {FAILED}"))
sys.exit(1 if FAILED else 0)
