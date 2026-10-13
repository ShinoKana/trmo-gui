"""The real ThorlabsDelay must honour bbd_reverse — mock passed it, the real path forgot it,
so on hardware a LARGER um wrongly read as a LARGER ps (the '467402 um = -500 ps' bug)."""
import sys, os, inspect
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import auto_mapping as am

fails = []
def check(n, c, e=""):
    print(f"  {'PASS' if c else 'FAIL'}  {n}  {e}")
    if not c: fails.append(n)

# The bug is structural: does _make_devices pass `reverse` to BOTH the mock and the
# real ThorlabsDelay? Inspect the source of _make_devices for the real-BBD call.
src = inspect.getsource(am._make_devices)
real_call = src.split("ThorlabsDelay(")[1].split(")")[0]
check("real ThorlabsDelay(...) is passed reverse=", "reverse" in real_call,
      "reverse NOT in the real BBD constructor call" if "reverse" not in real_call else "")

# And prove the convention once it IS passed: with reverse=True + measured t0,
# a LARGER position must give a SMALLER ps (467402.8 um -> +500, NOT -500).
b = am.ThorlabsDelay.__new__(am.ThorlabsDelay)   # bypass hardware connect
b.t0_mm = 542.351; b.mm_per_ps = -(0.299792458/2)  # reverse=True
ps = b.mm_to_ps(467.4028)
check("467402.8 um -> +500 ps (not -500) under reverse=True", abs(ps - 500) < 1,
      f"got {ps:.1f} ps")
check("a LARGER um gives a SMALLER ps", b.mm_to_ps(500.0) < b.mm_to_ps(100.0),
      f"500mm->{b.mm_to_ps(500):.0f}ps  <  100mm->{b.mm_to_ps(100):.0f}ps")

print("\n" + ("ALL PASSED" if not fails else f"FAILED: {fails}"))
sys.exit(1 if fails else 0)
