"""The delay readout reads HOW FAR THIS SCAN HAS RUN :

  "the readout starts at 0 ps and only grows as the scan runs; the plot too"
  "it can never be negative"

So it is a distance from the scan's own start, not a coordinate against a
calibration zero. At rest -> 0.00 whatever Start says; press Start, the line
parks on Start, and from there it only counts up. Same rule for the plot x axis.
ASCII only in the prints: the lab PC console is cp1252 and dies on CJK.
"""
import os, sys
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
    app.config_data["bbd_reverse"] = True
    app.config_data["bbd_t0_mm"] = 545.350      # the stale zero from an earlier scan
    app._last_pos_mm = 549.390                  # the delay line is parked at the Start box

    print("=== every Start reads 0 ps; only the END ps changes ===")
    for start_um, want_end in [(525000, 3502.4), (545350, 3638.2), (553550, 3692.9)]:
        app.ts_unit.set("µm")
        app.ts_vars["time_start_ps"].set(str(start_um))
        app.ts_vars["time_end_ps"].set("0")
        app.ts_vars["time_step_ps"].set("1000")
        _t0, sp, ep, _stp = moke_gui.plan_timescan(
            app.config_data, "um", float(start_um), 0.0, 1000.0)
        check(f"Start {start_um:,} um -> 0 ps, end +{want_end:.0f} ps",
              abs(sp) < 1e-9 and abs(ep - want_end) < 0.5, f"{sp:.1f} .. {ep:.1f} ps")

    print("=== the readout changes with the Start box, and is never negative ===")
    app.ts_unit.set("\u00b5m")
    app._last_pos_mm = 549.390                  # the line stands still through all of this
    for start, want in (("549390", "0.00 ps"), ("559390", "66.71 ps"),
                        ("539390", "66.71 ps"), ("500000", "329.49 ps")):
        app.ts_vars["time_start_ps"].set(start)
        check(f"Start={start} -> {want}", f"({want})" in app.pos_var.get(),
              ascii(app.pos_var.get()))
        check(f"Start={start} has no minus sign", "-" not in app.pos_var.get(),
              ascii(app.pos_var.get()))

    print("=== on the Start position it is 0, and the sweep counts UP from there ===")
    app.ts_vars["time_start_ps"].set("549390")
    for mm, want in ((549.390, "0.00 ps"), (548.390, "6.67 ps"), (0.0, "3665.14 ps")):
        app._last_pos_mm = mm
        app._relabel_position()
        check(f"{mm*1000:,.0f} um -> {want}", want in app.pos_var.get(), ascii(app.pos_var.get()))
    app._last_pos_mm = 553.550                  # jogged the WRONG way, past the Start
    app._relabel_position()
    check("even past the Start it cannot go negative", "-" not in app.pos_var.get(),
          ascii(app.pos_var.get()))

    print("=== typing must never command the delay line ===")
    moved = []
    for name in ("move_bbd_to_mm", "move_bbd_to_ps", "home_bbd"):
        setattr(app.hub, name, (lambda n: (lambda *a, **k: moved.append(n)))(name))
    app._last_pos_mm = 549.390
    for start in ("5", "54", "549", "5493", "549390", "559390", "0", ""):
        app.ts_vars["time_start_ps"].set(start)
    check("no move/home was issued while typing", moved == [], str(moved))

    print("=== the estimate tracks the boxes live ===")
    app.ts_vars["time_start_ps"].set("549390")
    app.ts_vars["time_end_ps"].set("0")
    app.ts_vars["time_step_ps"].set("1000")
    est_1000 = app.ts_est.get()
    check("Step 1000 um -> 550 pts, ps 0-3665", "550 pts" in est_1000 and "0\u20133665" in est_1000,
          ascii(est_1000))
    app.ts_vars["time_step_ps"].set("500")
    est_500 = app.ts_est.get()
    check("halving the step doubles the points, live", "1100 pts" in est_500, ascii(est_500))
    app.ts_vars["time_end_ps"].set("400000")
    check("changing End re-reads the ps span live", "0\u20133665" not in app.ts_est.get(),
          ascii(app.ts_est.get()))
    app.ts_vars["time_end_ps"].set("0"); app.ts_vars["time_step_ps"].set("1000")

    print("=== Example: line at 549390, Set 538390 -> that spot becomes 0 ps ===")
    for name in ("move_bbd_to_mm", "move_bbd_to_ps", "home_bbd"):
        app.hub.__dict__.pop(name, None)      # drop the stubs the previous block installed
    app.hub.connect(app.config_data, "mock", need_stage=False)   # a real mock delay line
    app._run_bg = lambda fn, label: fn()                          # run the move inline
    moved_to = []
    real_move = app.hub.move_bbd_to_ps
    app.hub.move_bbd_to_ps = lambda ps, **k: (moved_to.append(ps), real_move(ps, **k))[0]
    app.ts_unit.set("\u00b5m")
    app._last_pos_mm = 549.390
    app.ts_vars["time_start_ps"].set("538390")
    app.ts_vars["time_end_ps"].set("0")
    app.ts_vars["time_step_ps"].set("1000")
    app._ts_set()
    check("Set commands one move, to 0 ps", len(moved_to) == 1 and abs(moved_to[0]) < 1e-9,
          str(moved_to))
    check("the delay line is physically at 538390 um",
          abs(app.hub.read_position_mm() - 538.390) < 1e-6,
          f"{app.hub.read_position_mm()*1000:,.1f} um")
    app._last_pos_mm = app.hub.read_position_mm()
    app._relabel_position()
    check("and the readout there is 0.00 ps",
          "538390.0 um  (0.00 ps)" in app.pos_var.get(), ascii(app.pos_var.get()))
    _t0, sp, ep, _st = moke_gui.plan_timescan(app.config_data, "um", 538390.0, 0.0, 1000.0)
    check("the sweep then runs 0 -> +3591.8 ps (set by the End box)",
          abs(sp) < 1e-9 and abs(ep - 3591.75) < 0.5, f"{sp:.1f} .. {ep:.1f} ps")
    end_um = moke_gui.ps_to_pos_um(ep, _t0, moke_gui.mm_per_ps_for(app.config_data))
    check("ps up corresponds to um down, ending on the End box", abs(end_um) < 1,
          f"{end_um:,.0f} um at the last point")

    print("=== the plot x axis obeys the same rule ===")
    app._ts_t = np.array([0.0, 6.67, 13.34, 3665.14])
    x = app._ts_xdisp()
    check("plot x starts at 0 and only grows",
          abs(x[0]) < 1e-9 and all(b > a for a, b in zip(x, x[1:])), str(x))
    app._ts_t = np.array([-100.0, -50.0, 0.0, 50.0])   # a ps-mode sweep crossing t0
    x = app._ts_xdisp()
    check("even a sweep crossing t0 plots from 0 upward", x.min() >= 0 and abs(x[0]) < 1e-9, str(x))

    # render for real and read the axis back: 0 must sit at the LEFT edge
    app._ts_t = np.linspace(0.0, 3591.75, 60)
    app._ts_y = np.zeros((60, 4)); app._ts_y[:, 0] = np.linspace(1e-6, 2e-6, 60)
    app.ts_signal.set("X"); app.ts_view.set("TDS")
    app._redraw_timescan()
    lo, hi = app.ts_ax.get_xlim()
    check("the Delay/Amplitude plot runs left-to-right with 0 exactly at the left edge",
          lo == 0.0 and hi > 3500, f"xlim = {lo:.1f} .. {hi:.1f}")

    # the cases that used to put 0 in the MIDDLE: no data, and a single acquired point
    app._ts_t = None
    app.ts_vars["time_start_ps"].set("549390")
    app.ts_vars["time_end_ps"].set("0")
    app.ts_vars["time_step_ps"].set("1000")
    app._update_ts_estimate()
    lo, hi = app.ts_ax.get_xlim()
    check("with no scan loaded 0 is at the left, not centred",
          lo == 0.0, f"xlim = {lo:.1f} .. {hi:.1f}")
    app._ts_t = np.linspace(0.0, 3665.14, 550)
    app._ts_y = np.full((550, 4), np.nan)
    app._ts_y[0] = [1e-6, 0.0, 1e-6, 0.0]          # only the FIRST point has arrived
    app._redraw_timescan()
    lo, hi = app.ts_ax.get_xlim()
    check("after the first point 0 is at the left AND the axis is still narrow",
          lo == 0.0 and hi < 10, f"xlim = {lo:.1f} .. {hi:.1f}")
    app._ts_y[:200] = [1e-6, 0.0, 1e-6, 0.0]       # 200 points in
    app._redraw_timescan()
    lo2, hi2 = app.ts_ax.get_xlim()
    check("the axis unrolls gradually as points arrive, 0 still at the left",
          lo2 == 0.0 and hi < hi2 < 1500, f"xlim = {lo2:.1f} .. {hi2:.1f}")
    app._ts_y[:] = [1e-6, 0.0, 1e-6, 0.0]          # the whole sweep
    app._redraw_timescan()
    lo3, hi3 = app.ts_ax.get_xlim()
    check("and reaches the full span only at the end",
          lo3 == 0.0 and hi3 > 3600, f"xlim = {lo3:.1f} .. {hi3:.1f}")

    # the LabVIEW look must survive all of this
    check("the yellow grid is still on", app.ts_ax.xaxis._major_tick_kw is not None
          and any(l.get_visible() for l in app.ts_ax.get_xgridlines()),
          f"gridline colour = {app.ts_ax.get_xgridlines()[0].get_color()}")
    check("the plot area is still black", app.ts_ax.get_facecolor()[:3] == (0.0, 0.0, 0.0),
          str(app.ts_ax.get_facecolor()))
    check("no negative delay is plotted", app._ts_xdisp().min() >= 0,
          f"min x = {app._ts_xdisp().min():.3f}")

    print("=== Mapping keeps its PHYSICAL spot when t0 re-bases ===")
    app.ts_vars["time_start_ps"].set("549390")
    app.map_vars["delay_label_ps"].set("1118")
    um_before = app.map_delay_um.get()
    ps_before = float(app.map_vars["delay_label_ps"].get())
    app.ts_vars["time_start_ps"].set("553550")     # next scan starts 4160 um higher
    check("the um (the spot on the sample) is unchanged",
          app.map_delay_um.get() == um_before, f"{um_before} -> {app.map_delay_um.get()}")
    check("the ps label is re-based instead (+27.7 ps for +4160 um)",
          abs(float(app.map_vars["delay_label_ps"].get()) - (ps_before + 27.75)) < 0.3,
          f"{ps_before} -> {app.map_vars['delay_label_ps'].get()} ps")

    print("=== ps mode must NOT hijack t0 ===")
    app.config_data["bbd_t0_mm"] = 545.350
    app.ts_unit.set("ps")
    app.ts_vars["time_start_ps"].set("0")
    check("typing in ps mode leaves the calibration alone",
          abs(app.config_data["bbd_t0_mm"] - 545.350) < 1e-9,
          f"t0 = {app.config_data['bbd_t0_mm']} mm")

    print("=== a half-typed number must not corrupt the calibration ===")
    app.ts_unit.set("\u00b5m")
    app.ts_vars["time_start_ps"].set("")
    check("an empty box is ignored", abs(app.config_data["bbd_t0_mm"] - 545.350) < 1e-9,
          f"t0 = {app.config_data['bbd_t0_mm']} mm")
    app.ts_vars["time_start_ps"].set("-")
    check("a lone minus sign is ignored", abs(app.config_data["bbd_t0_mm"] - 545.350) < 1e-9,
          f"t0 = {app.config_data['bbd_t0_mm']} mm")


moke_gui.run_app(test_hook=tests)
print("\n" + ("ALL PASSED" if not FAILED else f"FAILED: {FAILED}"))
sys.exit(1 if FAILED else 0)
