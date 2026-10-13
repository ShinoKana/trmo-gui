"""Headless tests for the traps that cost real measurement time (2026-07-13 lab session).
ASCII only in the prints: the lab PC console is cp1252 and dies on CJK."""
import os, sys, types
os.environ["MOKE_MOCK"] = "1"
import tempfile
# keep test breadcrumbs out of the live GUI's moke_gui.log
os.environ.setdefault("MOKE_LOG_DIR", tempfile.mkdtemp(prefix="moke_test_log_"))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np
import moke_gui

FAILED = []
def check(name, cond, extra=""):
    print(f"  {'PASS' if cond else 'FAIL'}  {name}  {extra}")
    if not cond:
        FAILED.append(name)

def tests(app):
    app.withdraw()

    print("=== 1. the 'typed a new TC but never pressed Set' trap (a whole map on the wrong TC) ===")
    app._tc_dev_ms = 500.0; app._sens_dev = "10 mV"
    app.tc_var.set("100")                    # typed 100 ms, never pressed Set
    check("unapplied TC is detected", len(app._lockin_pending()) == 1, app._lockin_pending())
    check("red 'not applied' warning shows", app.lockin_warn.get() != "", ascii(app.lockin_warn.get()))
    app._tc_dev_ms = 100.0                   # as if Set wrote it and read it back
    app._check_lockin_applied()
    check("warning clears once applied", app._lockin_pending() == [] and app.lockin_warn.get() == "")
    app.sens_var.set("1 mV")                 # sensitivity changed, not Set
    check("unapplied SENS is detected too", len(app._lockin_pending()) == 1, app._lockin_pending())
    app._sens_dev = "1 mV"; app._check_lockin_applied()
    check("both clean -> no warning", app.lockin_warn.get() == "")

    print("=== 2. delay <-> position: ps = position/149.9, always positive ===")
    app.config_data["bbd_t0_mm"] = 0.0; app.config_data["bbd_reverse"] = False
    check("0 ps is position 0", abs(app._ps_to_um(0)) < 1, f"{app._ps_to_um(0):,.0f} um")
    check("a bigger delay is a bigger position (no reverse)", app._ps_to_um(3000) > app._ps_to_um(100),
          f"3000 ps -> {app._ps_to_um(3000):,.0f} um  >  100 ps -> {app._ps_to_um(100):,.0f} um")
    check("2500 ps -> 374,741 um (position/149.9, positive)", abs(app._ps_to_um(2500) - 374741) < 5,
          f"{app._ps_to_um(2500):,.0f} um")
    app.map_vars["delay_label_ps"].set("2500")
    check("the um label tracks the Delay box live", "374,74" in app.map_delay_um.get(),
          app.map_delay_um.get())

    print("=== 3. click the timescan -> that slice lands in Mapping ===")
    # a scan from a large position downward: internal ps 3789 -> 0 (ps = position/149.9)
    app._ts_t = np.linspace(3789.0, 0.0, 21)
    app._ts_y = np.zeros((21, 4)); app._ts_y[:, 0] = np.arange(21)
    app.ts_signal.set("X"); app.ts_view.set("TDS")
    # plot x-axis = |ts_t - ts_t[0]| (delay from scan start); click at plot-x 1000
    app._on_ts_click(types.SimpleNamespace(inaxes=app.ts_ax, button=1, xdata=1000.0))
    picked = float(app.map_vars["delay_label_ps"].get())
    xdisp = np.abs(app._ts_t - app._ts_t[0])
    expect = float(app._ts_t[int(np.argmin(np.abs(xdisp - 1000.0)))])
    check("plot-x 1000 ps maps back to the right internal ps (from scan start)",
          abs(picked - expect) < 0.5, f"picked {picked:.1f} ps")
    app.map3d_delays_var.set("")
    app._ts_pick_to_3d(); app._ts_pick_to_3d()
    check("'+ add to 3D delays' accumulates slices", app.map3d_delays_var.get().count(",") == 1,
          app.map3d_delays_var.get())
    check("adding a slice arms 3D mode", app.map3d_on.get())
    app._ts_pick_clear()
    check("clear empties the list", app.map3d_delays_var.get() == "" and app._ts_pick_ps is None)

    print("=== 4. mapping default signal (must be X, not R) ===")
    check("default map signal is X", app.map_signal.get() == "X", app.map_signal.get())

moke_gui.run_app(test_hook=tests)
print("\n" + ("ALL PASSED" if not FAILED else f"FAILED: {FAILED}"))
sys.exit(1 if FAILED else 0)
