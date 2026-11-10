#!/usr/bin/env python3
"""
SPIRE Test B summary: one thesis-ready workbook (and figures) from a set of
bench sessions plus the power log recorded alongside them.

Inputs per session folder: manifest.csv, session.json, bench.log,
imu_camera.log, imu_platform.log and, if present, c###_imu.csv (servo
response, criteria B-3 and B-4). Without c###_imu.csv, B-3 and B-4 are
estimated from the six frame samples per cycle in manifest.csv and marked
"frames (proxy)".

Sheets:
  README     what each sheet holds, criteria, known limitations
  Sessions   configuration and health per session
  Cycles     one row per cycle (the data every other sheet is computed from)
  Bins       per rotation-speed bin, UNCORRECTED vs CORRECTED (formulas)
  Criteria   B-1..B-4 verdicts (formulas) and Mann-Whitney tests (script)
  Selection  gyro3 burst rule vs the old gz rule
  Static     STATIC baseline
  Power      supply per session window (formulas over PowerRaw)
  PowerRaw   the power log

Usage (PC / WSL, needs openpyxl; matplotlib only for --figures):
  python3 src/testb_summary.py DIR [DIR ...] --power power_YYYYMMDD_HHMMSS.csv \\
      -o testB_summary.xlsx --figures figs/
"""

import argparse
import csv
import glob
import json
import math
import os
import re
import sys
from datetime import datetime, timedelta, timezone
from statistics import median

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from bench_report import (BINS, bin_label, bin_of, fnum, mann_whitney,  # noqa: E402
                          quartiles)
import servo_response  # noqa: E402

try:
    import openpyxl
except ImportError:
    sys.exit("openpyxl missing: python3 -m pip install --user 'openpyxl>=3.1'")
try:
    from openpyxl import Workbook
    from openpyxl.comments import Comment
    from openpyxl.styles import Alignment, Font, PatternFill
    from openpyxl.utils import get_column_letter
    from openpyxl.worksheet.formula import ArrayFormula
except ImportError:
    sys.exit(f"openpyxl {openpyxl.__version__} is too old (needs >= 3.1 for array "
             "formulas): python3 -m pip install --user 'openpyxl>=3.1'")

# Criteria constants (test plan, Test B); written to the Criteria sheet as inputs
WINNER_LIMIT_DPS = 3.4      # 1 px yaw blur at the frame corner at 1/150 s
B1_SHARE = 0.80
B2_MAX_ATT = 0.25
B3_LO, B3_HI = 0.9, 1.1
NO_RESPONSE_GAIN = 0.15
N_MIN = 10
ALPHA = 0.05
STATIC_MAX_SCORE_PX = 1.0   # STATIC valid if every frame's 3-axis score <= this
PLAT_MIN_DPS = 5.0          # frames slower than this are not used for ratios
CMD_MIN_DPS = 10.0          # servo_response.MIN_CMD_RATE
SAT_DEG = 134.0
SHUNT_OHM_DEFAULT = 0.01    # ASSUMPTION, to be read off the UPS board

TRUE = ("1", "True", "true")
LOG_TS = re.compile(r"^(\d{4}-\d\d-\d\d \d\d:\d\d:\d\d\.\d+)")
CYCLE_START = re.compile(r"Cycle (\d+): ")
CYCLE_END = re.compile(r"\[c(\d{3}) ")
GAIN_RE = re.compile(r"Exposure locked: requested (\d+) us, sensor (\d+) us, gain ([\d.]+)")
IMU_RE = re.compile(r"avg rate: ([\d.]+) Hz.*I2C errors: (\d+)")


# ---------------------------------------------------------------------------
# Loading
# ---------------------------------------------------------------------------

def parse_utc(s):
    return datetime.fromisoformat(s).astimezone(timezone.utc)


def read_log(path):
    """[(naive local datetime, line)] for lines that start with a timestamp."""
    out = []
    if not os.path.exists(path):
        return out
    with open(path, errors="replace") as fh:
        for line in fh:
            m = LOG_TS.match(line)
            if m:
                out.append((datetime.strptime(m.group(1), "%Y-%m-%d %H:%M:%S.%f"), line))
    return out


def imu_stats(path):
    if not os.path.exists(path):
        return None, None
    rate = err = None
    with open(path, errors="replace") as fh:
        for line in fh:
            m = IMU_RE.search(line)
            if m:
                rate, err = float(m.group(1)), int(m.group(2))
    return rate, err


def frames_proxy(frames):
    """Compensation ratio and servo gain from the burst frames alone.

    comp = -(cam - plat) / plat per frame with |plat| >= 5 deg/s (same
    definition as servo_response); gain = (cam - plat) / servo command rate
    between consecutive frames with |command rate| >= 10 deg/s."""
    comps, gains = [], []
    for f in frames:
        c, p = fnum(f["cam_gz"]), fnum(f["plat_gz"])
        if c is not None and p is not None and abs(p) >= PLAT_MIN_DPS:
            comps.append(-(c - p) / p)
    for a, b in zip(frames, frames[1:]):
        sa, sb = fnum(a["servo_cmd_deg"]), fnum(b["servo_cmd_deg"])
        ta, tb = fnum(a["sensor_timestamp_ns"]), fnum(b["sensor_timestamp_ns"])
        if None in (sa, sb, ta, tb) or tb <= ta:
            continue
        cmd = (sb - sa) / ((tb - ta) * 1e-9)
        acts = [fnum(f["cam_gz"]) - fnum(f["plat_gz"]) for f in (a, b)
                if fnum(f["cam_gz"]) is not None and fnum(f["plat_gz"]) is not None]
        if len(acts) == 2 and abs(cmd) >= CMD_MIN_DPS:
            gains.append((sum(acts) / 2) / cmd)
    return (median(comps) if comps else None, median(gains) if gains else None)


def load_session(path):
    name = os.path.basename(os.path.normpath(path))
    with open(os.path.join(path, "session.json")) as fh:
        meta = json.load(fh)
    with open(os.path.join(path, "manifest.csv"), newline="") as fh:
        rows = list(csv.DictReader(fh))
    fp, bp = meta.get("flight_params", {}), meta.get("bench_params", {})
    started = parse_utc(meta["started_utc"])

    log = read_log(os.path.join(path, "bench.log"))
    # bench.log is in Pi local time; offset to UTC from the session start
    offset = timedelta(0)
    if log:
        raw = (log[0][0] - started.replace(tzinfo=None)).total_seconds()
        offset = timedelta(minutes=15 * round(raw / 900))
    exp_req = exp_sensor = again = None
    stale = []
    for t, line in log:
        m = GAIN_RE.search(line)
        if m:
            exp_req, exp_sensor, again = int(m.group(1)), int(m.group(2)), float(m.group(3))
        if "platform data stale" in line:
            stale.append(t - offset)
    cam_rate, cam_err = imu_stats(os.path.join(path, "imu_camera.log"))
    plat_rate, plat_err = imu_stats(os.path.join(path, "imu_platform.log"))

    by_cycle = {}
    for r in rows:
        by_cycle.setdefault(int(r["cycle_id"]), []).append(r)
    for fr in by_cycle.values():
        fr.sort(key=lambda r: int(r["frame_index"]))

    mode = meta.get("mode", "")
    cycles = []
    for cid, fr in sorted(by_cycle.items()):
        r0 = fr[0]
        status = r0["effective_status"]
        speed = fnum(r0["rotation_speed_dps"]) or 0.0
        win = next((f for f in fr if f["winner"] in TRUE), None)
        gzp = next((f for f in fr if f.get("winner_gz") in TRUE), None)
        scores = [fnum(f["score_px"]) for f in fr if fnum(f["score_px"]) is not None]
        plat = [abs(fnum(f["plat_gz"])) for f in fr if fnum(f["plat_gz"]) is not None]
        atts = [fnum(f["attenuation"]) for f in fr if fnum(f["attenuation"]) is not None]
        servo = [fnum(f["servo_cmd_deg"]) for f in fr if fnum(f["servo_cmd_deg"]) is not None]
        logged_valid = r0["cycle_valid"] in TRUE
        valid, reason = logged_valid, ""
        if mode == "STATIC":
            # Logged rule used |cam_gz| through the 16 mm linear model; a gyro
            # bias residual of ~1 deg/s alone exceeds it. Re-judged on the
            # 3-axis exposure score with the bench lens.
            valid = bool(scores) and len(scores) == len(fr) and max(scores) <= STATIC_MAX_SCORE_PX
            reason = "" if valid else f"max score {max(scores or [0]):.2f} px > {STATIC_MAX_SCORE_PX}"
        elif not logged_valid:
            reason = ("CALM: pre-burst rate below min_rotation" if status == "CALM"
                      else f"rotation {speed:.1f} < {fp.get('min_rotation', 5.0)} deg/s")

        t_burst = parse_utc(r0["timestamp_utc"])
        ts = [fnum(f["sensor_timestamp_ns"]) for f in fr if fnum(f["sensor_timestamp_ns"])]
        span = (max(ts) - min(ts)) * 1e-9 if len(ts) > 1 else 1.3
        w0 = t_burst.replace(tzinfo=None) - timedelta(seconds=0.2)
        w1 = t_burst.replace(tzinfo=None) + timedelta(seconds=span + 0.1)
        n_stale = sum(w0 <= t <= w1 for t in stale)

        def g(f, k, absval=False):
            v = fnum(f[k]) if f else None
            return abs(v) if absval and v is not None else v

        comp = gain = None
        src = ""
        if mode == "CORRECTED" and status == "ACTIVE":
            imu = os.path.join(path, f"c{cid:03d}_imu.csv")
            res = servo_response.analyse_cycle(imu) if os.path.exists(imu) else None
            if res:
                comp, gain, src = res["comp"], res["gain"], "c###_imu.csv"
            else:
                comp, gain = frames_proxy(fr)
                src = "frames (proxy)"

        late = [abs(fnum(f["plat_gz"])) for f in fr[1:] if fnum(f["plat_gz"]) is not None]
        p0 = abs(fnum(r0["plat_gz"]) or 0.0)
        win_plat = g(win, "plat_gz", True)
        cycles.append({
            "session": name, "cycle": cid, "mode": mode, "status": status,
            "valid_logged": int(logged_valid), "valid": int(valid), "reason": reason,
            "rot_dps": speed, "bin": bin_label(bin_of(speed)),
            "burst_plat_med": median(plat) if plat else None,
            "plat_speedup": (median(late) / p0 if late and p0 >= 1.0 else None),
            "win_abs_plat": win_plat,
            "bin_win": bin_label(bin_of(win_plat)) if win_plat is not None else "",
            "win_frame": int(win["frame_index"]) if win else None,
            "gz_frame": int(gzp["frame_index"]) if gzp else None,
            "agree": int(win is not None and gzp is not None and win is gzp),
            "win_abs_gz": g(win, "cam_gz", True), "win_score": g(win, "score_px"),
            "win_tilt": g(win, "blur_tilt_px"), "win_yaw": g(win, "blur_yaw_px"),
            "gz_abs_gz": g(gzp, "cam_gz", True), "gz_score": g(gzp, "score_px"),
            "burst_min_score": min(scores) if scores else None,
            "burst_cam_gz_med": median([fnum(f["cam_gz"]) for f in fr
                                        if fnum(f["cam_gz"]) is not None] or [0]),
            "att_med": median(atts) if atts and mode != "STATIC" else None,
            "comp": comp, "servo_gain": gain, "resp_src": src,
            "sat_share": (sum(abs(s) >= SAT_DEG for s in servo) / len(servo)
                          if servo and mode == "CORRECTED" else None),
            "pressure_hpa": g(win or r0, "pressure_hpa"),
            "baro_alt_m": g(win or r0, "baro_alt_m"),
            "pid_stale_in_burst": n_stale,
            "burst_utc": t_burst.replace(tzinfo=None),
        })

    last = max(c["burst_utc"] for c in cycles) + timedelta(seconds=2)
    press = [fnum(r["pressure_hpa"]) for r in rows if fnum(r["pressure_hpa"]) is not None]
    sess = {
        "session": name, "mode": mode, "start_utc": started.replace(tzinfo=None),
        "end_utc": last, "exp_req_us": exp_req or bp.get("exposure_us"),
        "exp_sensor_us": exp_sensor or median([int(r["exposure_time_us"]) for r in rows]),
        "analogue_gain": again, "rate_gain": fp.get("rate_gain"),
        "deadband": fp.get("gyro_deadband"), "min_rotation": fp.get("min_rotation"),
        "select": bp.get("select"), "lens_mm": bp.get("lens_mm"),
        "score_radius": bp.get("score_radius"), "cam_rate": cam_rate,
        "cam_err": cam_err, "plat_rate": plat_rate, "plat_err": plat_err,
        "pid_stale": len(stale), "baro_hpa": median(press) if press else None,
        "baro_ok_share": (sum(r["baro_ok"] in TRUE for r in rows) / len(rows)) if rows else None,
        "imu_csv": len(glob.glob(os.path.join(path, "c*_imu.csv"))),
    }
    return sess, cycles


def load_power(path):
    out = []
    with open(path, newline="") as fh:
        for r in csv.DictReader(fh):
            out.append({
                "utc": parse_utc(r["utc"]).replace(tzinfo=None),
                "ext5v_v": fnum(r["ext5v_v"]), "throttled": r["throttled_hex"],
                "uv_now": fnum(r["uv_now"]), "thr_now": fnum(r["thr_now"]),
                "soc_temp_c": fnum(r["soc_temp_c"]), "pack_v": fnum(r["pack_v"]),
                "shunt_mv": fnum(r["shunt_mv"])})
    return out


# ---------------------------------------------------------------------------
# Workbook helpers
# ---------------------------------------------------------------------------

FONT = "Arial"
HDR = Font(name=FONT, bold=True, color="FFFFFF")
HDR_FILL = PatternFill("solid", fgColor="1F3864")
INPUT = Font(name=FONT, color="0000FF")
ASSUME = PatternFill("solid", fgColor="FFFF00")
BODY = Font(name=FONT)
BOLD = Font(name=FONT, bold=True)
WRAP = Alignment(wrap_text=True, vertical="top")


def header(ws, row, names, widths=None):
    for i, n in enumerate(names, 1):
        c = ws.cell(row=row, column=i, value=n)
        c.font, c.fill = HDR, HDR_FILL
        c.alignment = Alignment(wrap_text=True, vertical="center")
    if widths:
        for i, w in enumerate(widths, 1):
            ws.column_dimensions[get_column_letter(i)].width = w
    ws.freeze_panes = ws.cell(row=row + 1, column=1)


def put(ws, row, col, value, fmt=None, font=BODY):
    c = ws.cell(row=row, column=col, value=value)
    c.font = font
    if fmt:
        c.number_format = fmt
    return c


def finish_fonts(wb):
    for ws in wb.worksheets:
        for row in ws.iter_rows():
            for c in row:
                if c.font is None or c.font.name != FONT:
                    f = c.font
                    c.font = Font(name=FONT, bold=f.bold, color=f.color, italic=f.italic)


# Cycles sheet columns: key, header, width, number format
CYC = [
    ("session", "session", 24, None), ("cycle", "cycle", 6, "0"),
    ("mode", "mode", 13, None), ("status", "status", 9, None),
    ("valid_logged", "valid (logged)", 8, "0"), ("valid", "valid (used)", 8, "0"),
    ("reason", "invalid reason", 34, None),
    ("rot_dps", "pre-burst platform rate [deg/s]", 11, "0.0"),
    ("bin", "bin [deg/s]", 8, None),
    ("burst_plat_med", "burst median |plat_gz| [deg/s]", 11, "0.0"),
    ("plat_speedup", "platform rate f1-f5 / f0", 9, "0.00"),
    ("win_abs_plat", "platform |gz| at winner [deg/s]", 10, "0.0"),
    ("bin_win", "bin by winner platform rate", 8, None),
    ("win_frame", "winner frame (gyro3)", 8, "0"),
    ("gz_frame", "gz-rule frame", 8, "0"),
    ("agree", "same frame", 7, "0"),
    ("win_abs_gz", "winner |cam_gz| [deg/s]", 10, "0.00"),
    ("win_score", "winner score [px]", 9, "0.00"),
    ("win_tilt", "winner tilt blur [px]", 9, "0.00"),
    ("win_yaw", "winner yaw blur @0.25R [px]", 9, "0.00"),
    ("gz_abs_gz", "gz-rule |cam_gz| [deg/s]", 10, "0.00"),
    ("gz_score", "gz-rule score [px]", 9, "0.00"),
    ("burst_min_score", "burst min score [px]", 9, "0.00"),
    ("burst_cam_gz_med", "burst median cam_gz [deg/s]", 10, "0.00"),
    ("att_med", "median |cam|/|plat|", 9, "0.00"),
    ("comp", "compensation ratio", 10, "0.00"),
    ("servo_gain", "servo gain act/cmd", 10, "0.00"),
    ("resp_src", "B-3/B-4 source", 14, None),
    ("sat_share", "servo saturated share", 9, "0%"),
    ("pressure_hpa", "pressure [hPa]", 9, "0.00"),
    ("baro_alt_m", "ISA altitude [m]", 9, "0"),
    ("pid_stale_in_burst", "PID stale events in burst", 8, "0"),
    ("burst_utc", "burst start UTC", 20, "yyyy-mm-dd hh:mm:ss"),
]
COL = {k: get_column_letter(i + 1) for i, (k, *_r) in enumerate(CYC)}


def rng(key, n):
    return f"Cycles!${COL[key]}$2:${COL[key]}${n + 1}"


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    ap = argparse.ArgumentParser(description="SPIRE Test B summary workbook")
    ap.add_argument("sessions", nargs="+")
    ap.add_argument("--power", help="power_log.py CSV covering the sessions")
    ap.add_argument("--shunt-ohm", type=float, default=SHUNT_OHM_DEFAULT,
                    help="UPS INA219 shunt (default 0.01, verify on the board)")
    ap.add_argument("-o", "--output", default="testB_summary.xlsx")
    ap.add_argument("--figures", help="Folder for thesis figures (PNG + PDF)")
    args = ap.parse_args()

    sessions, cycles = [], []
    for p in sorted(args.sessions):
        s, cs = load_session(p)
        sessions.append(s)
        cycles += cs
        print(f"{s['session']}: {s['mode']}, {len(cs)} cycles, "
              f"{sum(c['valid'] for c in cs)} valid")
    n = len(cycles)
    power = load_power(args.power) if args.power else []

    wb = Workbook()
    readme = wb.active
    readme.title = "README"

    # ---- Cycles -----------------------------------------------------------
    ws = wb.create_sheet("Cycles")
    header(ws, 1, [h for _, h, _, _ in CYC], [w for *_x, w, _ in CYC])
    for i, c in enumerate(cycles, 2):
        for j, (k, _h, _w, f) in enumerate(CYC, 1):
            put(ws, i, j, c[k], f)
    ws.auto_filter.ref = f"A1:{get_column_letter(len(CYC))}{n + 1}"

    # ---- Criteria inputs (referenced by Bins and Criteria) -----------------
    cr = wb.create_sheet("Criteria")
    cr.column_dimensions["A"].width = 46
    for col in "BCDEFG":
        cr.column_dimensions[col].width = 16
    put(cr, 1, 1, "Criterion inputs (test plan, Test B)", font=BOLD)
    inputs = [
        ("B-1 winner rate limit [deg/s]", WINNER_LIMIT_DPS,
         "1 px yaw blur at the frame corner (R = 2534 px) at the flight exposure 1/150 s. "
         "At the bench exposure 20 ms the same rate gives ~3 px."),
        ("B-1 required share", B1_SHARE, None),
        ("B-2 max median attenuation", B2_MAX_ATT, None),
        ("B-3 compensation ratio min", B3_LO, None),
        ("B-3 compensation ratio max", B3_HI, None),
        ("B-4 NO RESPONSE gain below", NO_RESPONSE_GAIN, None),
        ("minimum valid cycles per bin", N_MIN, None),
        ("Mann-Whitney alpha", ALPHA, None),
        ("STATIC max score [px]", STATIC_MAX_SCORE_PX,
         "Re-judged rule for STATIC validity, applied by the script (Cycles col F)."),
    ]
    P = {}
    for i, (label, v, note) in enumerate(inputs, 2):
        put(cr, i, 1, label)
        c = put(cr, i, 2, v, "0.00", INPUT)
        if note:
            c.comment = Comment(note, "testb_summary")
        P[label] = f"Criteria!$B${i}"
    lim, share_req = P["B-1 winner rate limit [deg/s]"], P["B-1 required share"]
    nmin = P["minimum valid cycles per bin"]

    # ---- Bins ---------------------------------------------------------------
    bs = wb.create_sheet("Bins")
    bhead = ["mode", "status", "bin [deg/s]", "cycles", "valid",
             "winner |cam_gz| median", "Q1", "Q3", "share <= limit",
             "winner score median [px]", "score Q1", "score Q3",
             "tilt median [px]", "yaw median [px]",
             "median |cam|/|plat|", "compensation ratio median", "servo gain median",
             "gz-rule |cam_gz| median", "gz-rule share <= limit", "saturated share mean"]
    header(bs, 1, bhead, [13, 9, 9, 7, 7] + [11] * 15)
    groups = [("UNCORRECTED", "HELD"), ("CORRECTED", "ACTIVE")]
    labels = [bin_label(b) for b in BINS]
    bin_row = {}
    r = 2
    M, S, V, B = rng("mode", n), rng("status", n), rng("valid", n), rng("bin", n)
    for mode, status in groups:
        for lab in labels:
            bin_row[(mode, lab)] = r
            put(bs, r, 1, mode)
            put(bs, r, 2, status)
            put(bs, r, 3, lab)
            cond = f'{M},$A{r},{S},$B{r},{B},$C{r}'
            put(bs, r, 4, f"=COUNTIFS({cond})", "0")
            put(bs, r, 5, f"=COUNTIFS({cond},{V},1)", "0")
            mask = f'({M}=$A{r})*({S}=$B{r})*({B}=$C{r})*({V}=1)'

            def arr(col, key, fn="MEDIAN", fmt="0.00"):
                ref = f"{get_column_letter(col)}{r}"
                vr = rng(key, n)
                f = f'=IFERROR({fn}(IF({mask}*({vr}<>""),{vr})),"")'
                bs[ref] = ArrayFormula(ref, f)
                bs[ref].number_format = fmt

            arr(6, "win_abs_gz")
            for col, k in ((7, 1), (8, 3)):
                ref = f"{get_column_letter(col)}{r}"
                vr = rng("win_abs_gz", n)
                bs[ref] = ArrayFormula(ref, f'=IFERROR(QUARTILE(IF({mask}*({vr}<>""),{vr}),{k}),"")')
                bs[ref].number_format = "0.00"
            put(bs, r, 9, f'=IF(E{r}=0,"",COUNTIFS({cond},{V},1,{rng("win_abs_gz", n)},"<="&{lim})/E{r})', "0%")
            arr(10, "win_score")
            for col, k in ((11, 1), (12, 3)):
                ref = f"{get_column_letter(col)}{r}"
                vr = rng("win_score", n)
                bs[ref] = ArrayFormula(ref, f'=IFERROR(QUARTILE(IF({mask}*({vr}<>""),{vr}),{k}),"")')
                bs[ref].number_format = "0.00"
            arr(13, "win_tilt")
            arr(14, "win_yaw")
            arr(15, "att_med")
            arr(16, "comp")
            arr(17, "servo_gain")
            arr(18, "gz_abs_gz")
            put(bs, r, 19, f'=IF(E{r}=0,"",COUNTIFS({cond},{V},1,{rng("gz_abs_gz", n)},"<="&{lim})/E{r})', "0%")
            arr(20, "sat_share", "AVERAGE", "0%")
            r += 1

    # ---- Criteria verdicts --------------------------------------------------
    row = len(inputs) + 3
    put(cr, row, 1, "Verdicts (CORRECTED / ACTIVE, valid cycles)", font=BOLD)
    row += 1
    for j, h in enumerate(["criterion", "value", "n", "verdict", "UNCORRECTED value"], 1):
        c = put(cr, row, j, h, font=HDR)
        c.fill = HDR_FILL
    row += 1

    def verdict_row(label, value_f, n_f, ok_f, unc_f=None, fmt="0.00"):
        nonlocal row
        put(cr, row, 1, label)
        put(cr, row, 2, value_f, fmt)
        put(cr, row, 3, n_f, "0")
        put(cr, row, 4, f'=IF(C{row}<{nmin},"not enough data",IF({ok_f},"PASS","FAIL"))')
        if unc_f:
            put(cr, row, 5, unc_f, fmt)
        row += 1

    for lab in ("10-20", "20-40"):
        br, ur = bin_row[("CORRECTED", lab)], bin_row[("UNCORRECTED", lab)]
        verdict_row(f"B-1 {lab} deg/s: share of gyro3 winners <= limit",
                    f"=Bins!I{br}", f"=Bins!E{br}", f"B{row}>={share_req}",
                    f"=Bins!I{ur}", "0%")
    for lab in ("10-20", "20-40"):
        br = bin_row[("CORRECTED", lab)]
        verdict_row(f"B-1 (reference) {lab} deg/s: share of gz-rule frames <= limit",
                    f"=Bins!S{br}", f"=Bins!E{br}", f"B{row}>={share_req}", None, "0%")
    W = rng("bin_win", n)
    for lab in ("10-20", "20-40"):
        cnd = f'{rng("mode", n)},"CORRECTED",{rng("status", n)},"ACTIVE",{V},1,{W},"{lab}"'
        verdict_row(f"B-1 (sensitivity) {lab} deg/s, binned by platform rate at the winner exposure",
                    f'=IF(C{row}=0,"",COUNTIFS({cnd},{rng("win_abs_gz", n)},"<="&{lim})/C{row})',
                    f"=COUNTIFS({cnd})", f"B{row}>={share_req}", None, "0%")
    for lab in ("5-10", "10-20", "20-40"):
        br, ur = bin_row[("CORRECTED", lab)], bin_row[("UNCORRECTED", lab)]
        verdict_row(f"B-2 {lab} deg/s: median attenuation |cam|/|plat|",
                    f"=Bins!O{br}", f"=Bins!E{br}",
                    f"B{row}<={P['B-2 max median attenuation']}", f"=Bins!O{ur}")
    act = f'({rng("mode", n)}="CORRECTED")*({rng("status", n)}="ACTIVE")*({rng("valid", n)}=1)'
    for key, label, ok in (
            ("comp", "B-3 compensation ratio, median over cycles",
             f"AND(B{{r}}>={P['B-3 compensation ratio min']},B{{r}}<={P['B-3 compensation ratio max']})"),
            ("servo_gain", "B-4 servo gain act/cmd, median over cycles", None)):
        vr = rng(key, n)
        ref = f"B{row}"
        put(cr, row, 1, label)
        cr[ref] = ArrayFormula(ref, f'=IFERROR(MEDIAN(IF({act}*({vr}<>""),{vr})),"")')
        cr[ref].number_format = "0.00"
        put(cr, row, 3, f'=SUMPRODUCT({act}*({vr}<>""))', "0")
        if ok:
            put(cr, row, 4, f'=IF(C{row}<{nmin},"not enough data",IF({ok.format(r=row)},"PASS","FAIL"))')
        row += 1
    vr = rng("servo_gain", n)
    put(cr, row, 1, "B-4 NO RESPONSE cycles (gain < limit)")
    put(cr, row, 2, f'=SUMPRODUCT({act}*({vr}<>"")*({vr}<{P["B-4 NO RESPONSE gain below"]}))', "0")
    put(cr, row, 3, f'=SUMPRODUCT({act}*({vr}<>""))', "0")
    put(cr, row, 4, f'=IF(C{row}<{nmin},"not enough data",IF(B{row}=0,"PASS","FAIL"))')
    row += 1
    src = sorted({c["resp_src"] for c in cycles if c["resp_src"]})
    c = put(cr, row, 1, f"B-3/B-4 source: {', '.join(src) or '-'}")
    c.font = Font(name=FONT, italic=True)
    row += 2

    # Mann-Whitney (computed here; no spreadsheet equivalent)
    put(cr, row, 1, "Mann-Whitney U, two-sided, CORRECTED/ACTIVE vs UNCORRECTED, valid cycles "
                    "(computed by testb_summary.py)", font=BOLD)
    row += 1
    mw_head = ["measure / bin", "CORRECTED median", "UNCORRECTED median",
               "n CORR", "n UNC", "U", "p", "significant"]
    for j, h in enumerate(mw_head, 1):
        cc = put(cr, row, j, h, font=HDR)
        cc.fill = HDR_FILL
    cr.column_dimensions["H"].width = 12
    row += 1
    mw_results = []
    for key, name in (("win_abs_gz", "winner |cam_gz| [deg/s]"), ("win_score", "winner score [px]")):
        for lab in labels[1:]:
            a = [c[key] for c in cycles if c["mode"] == "CORRECTED" and c["status"] == "ACTIVE"
                 and c["valid"] and c["bin"] == lab and c[key] is not None]
            b = [c[key] for c in cycles if c["mode"] == "UNCORRECTED"
                 and c["valid"] and c["bin"] == lab and c[key] is not None]
            if not a or not b:
                continue
            U, p = mann_whitney(a, b)
            vals = [f"{name} {lab}", median(a), median(b), len(a), len(b), U, p,
                    ("n < 5" if p is None else ("yes" if p < ALPHA else "no"))]
            mw_results.append(vals)
            for j, v in enumerate(vals, 1):
                put(cr, row, j, v, "0.00" if j in (2, 3) else ("0.0E+00" if j == 7 else None))
            row += 1

    # ---- Sessions -----------------------------------------------------------
    ss = wb.create_sheet("Sessions", 1)
    sh = ["session", "mode", "start UTC", "end UTC", "cycles", "valid (used)",
          "ACTIVE", "CALM", "exposure requested [us]", "exposure sensor [us]",
          "analogue gain", "rate_gain", "deadband [deg/s]", "min_rotation [deg/s]",
          "burst select", "lens [mm]", "score radius [R]",
          "camera IMU rate [Hz]", "camera IMU I2C errors", "platform IMU rate [Hz]",
          "platform IMU I2C errors", "PID stale events", "pressure median [hPa]",
          "baro_ok share", "c###_imu.csv files"]
    header(ss, 1, sh, [24, 13, 19, 19] + [10] * 21)
    for i, s in enumerate(sessions, 2):
        q = f'{rng("session", n)},$A{i}'
        vals = [s["session"], s["mode"], s["start_utc"], s["end_utc"],
                f"=COUNTIFS({q})", f"=COUNTIFS({q},{rng('valid', n)},1)",
                f'=COUNTIFS({q},{rng("status", n)},"ACTIVE")',
                f'=COUNTIFS({q},{rng("status", n)},"CALM")',
                s["exp_req_us"], s["exp_sensor_us"], s["analogue_gain"], s["rate_gain"],
                s["deadband"], s["min_rotation"], s["select"], s["lens_mm"],
                s["score_radius"], s["cam_rate"], s["cam_err"], s["plat_rate"],
                s["plat_err"], s["pid_stale"], s["baro_hpa"], s["baro_ok_share"], s["imu_csv"]]
        fmts = {3: "yyyy-mm-dd hh:mm:ss", 4: "yyyy-mm-dd hh:mm:ss", 11: "0.00",
                18: "0.0", 20: "0.0", 23: "0.00", 24: "0%"}
        for j, v in enumerate(vals, 1):
            put(ss, i, j, v, fmts.get(j))

    # ---- Selection ----------------------------------------------------------
    sel = wb.create_sheet("Selection")
    header(sel, 1, ["mode", "status", "valid cycles", "same frame (gyro3 = gz rule)",
                    "agreement", "median score gyro3 winner [px]",
                    "median score gz-rule frame [px]", "median |cam_gz| gyro3 winner",
                    "median |cam_gz| gz-rule frame", "cycles where gz-rule score > 1.5x gyro3"],
           [13, 9, 9, 12, 10, 13, 13, 13, 13, 14])
    for i, (mode, status) in enumerate(groups + [("CORRECTED", "CALM"), ("STATIC", "HELD")], 2):
        put(sel, i, 1, mode)
        put(sel, i, 2, status)
        cond = f"{M},$A{i},{S},$B{i}"
        validc = "" if status == "CALM" else f",{V},1"
        put(sel, i, 3, f"=COUNTIFS({cond}{validc})", "0")
        put(sel, i, 4, f'=COUNTIFS({cond}{validc},{rng("agree", n)},1)', "0")
        put(sel, i, 5, f'=IF(C{i}=0,"",D{i}/C{i})', "0%")
        mask = f"({M}=$A{i})*({S}=$B{i})" + ("" if status == "CALM" else f"*({V}=1)")
        for col, key in ((6, "win_score"), (7, "gz_score"), (8, "win_abs_gz"), (9, "gz_abs_gz")):
            ref = f"{get_column_letter(col)}{i}"
            vr = rng(key, n)
            sel[ref] = ArrayFormula(ref, f'=IFERROR(MEDIAN(IF({mask}*({vr}<>""),{vr})),"")')
            sel[ref].number_format = "0.00"
        put(sel, i, 10, f'=SUMPRODUCT({mask}*({rng("gz_score", n)}>1.5*{rng("win_score", n)}))', "0")
    put(sel, 7, 1, "CALM rows include all CALM cycles (they are never valid). "
                   "STATIC uses the re-judged validity.", font=Font(name=FONT, italic=True))

    # ---- Static -------------------------------------------------------------
    st = wb.create_sheet("Static")
    header(st, 1, ["quantity", "value", "note"], [44, 12, 70])
    smask = f'({M}="STATIC")'
    srows = [
        ("cycles", f'=COUNTIFS({M},"STATIC")', "0", ""),
        ("valid by logged rule (16 mm linear model, |cam_gz|)",
         f'=COUNTIFS({M},"STATIC",{rng("valid_logged", n)},1)', "0",
         "Logged rule: |cam_gz| x t x f(16 mm) <= 1 px, i.e. |cam_gz| <= 0.28 deg/s at 20 ms."),
        ("valid by score rule (all frames score <= limit)",
         f'=COUNTIFS({M},"STATIC",{V},1)', "0", "Score: 3-axis integrated angle, 6 mm bench lens."),
        ("median pre-burst platform rate [deg/s]", "rot_dps", "0.00", "Platform IMU, 1 s before the burst."),
        ("median burst cam_gz [deg/s]", "burst_cam_gz_med", "0.00",
         "Signed. Non-zero while the platform is still: residual camera-gyro bias."),
        ("median winner score [px]", "win_score", "0.00", ""),
        ("max burst min score [px]", "burst_min_score", "0.00", ""),
        ("median winner tilt blur [px]", "win_tilt", "0.00", ""),
    ]
    for i, (q, f, fmt, note) in enumerate(srows, 2):
        put(st, i, 1, q)
        if f in COL:
            vr = rng(f, n)
            fn = "MAX" if q.startswith("max") else "MEDIAN"
            st[f"B{i}"] = ArrayFormula(f"B{i}", f'=IFERROR({fn}(IF({smask}*({vr}<>""),{vr})),"")')
            st[f"B{i}"].number_format = fmt
        else:
            put(st, i, 2, f, fmt)
        put(st, i, 3, note).alignment = WRAP

    # ---- PowerRaw + Power ---------------------------------------------------
    if power:
        pr = wb.create_sheet("PowerRaw")
        pcols = ["utc", "ext5v_v", "throttled", "uv_now", "thr_now", "soc_temp_c",
                 "pack_v", "shunt_mv"]
        header(pr, 1, ["UTC", "EXT5V [V]", "throttled", "under-voltage now",
                       "throttled now", "SoC temp [C]", "pack [V]", "INA219 shunt [mV]"],
               [22, 10, 10, 10, 10, 10, 10, 12])
        for i, rr in enumerate(power, 2):
            for j, k in enumerate(pcols, 1):
                pr.cell(row=i, column=j, value=rr[k])
            pr.cell(row=i, column=1).number_format = "yyyy-mm-dd hh:mm:ss.000"
        m = len(power) + 1
        T, E5 = f"PowerRaw!$A$2:$A${m}", f"PowerRaw!$B$2:$B${m}"
        TH, TE = f"PowerRaw!$C$2:$C${m}", f"PowerRaw!$F$2:$F${m}"
        PV, SH = f"PowerRaw!$G$2:$G${m}", f"PowerRaw!$H$2:$H${m}"

        pw = wb.create_sheet("Power")
        put(pw, 1, 1, "UPS INA219 shunt [ohm]", font=BOLD)
        c = put(pw, 1, 2, args.shunt_ohm, "0.000", INPUT)
        c.fill = ASSUME
        c.comment = Comment("ASSUMPTION: read the shunt resistor marking on the UPS board "
                            "(R010 = 0.010 ohm). Current columns scale as 1/R.", "testb_summary")
        R = "$B$1"
        put(pw, 2, 1, "Current = -shunt_mV / 1000 / R (discharge positive). "
                      "Pack side of the UPS only; the servo rail is not measured.",
            font=Font(name=FONT, italic=True))
        heads = ["window", "start UTC", "end UTC", "covered by log", "samples",
                 "EXT5V min [V]", "EXT5V mean [V]", "EXT5V max [V]",
                 "samples throttled/UV flag != 0x0", "SoC temp max [C]",
                 "pack max [V]", "pack min [V]", "pack current mean [A]",
                 "pack current peak [A]", "pack power mean [W]"]
        header(pw, 4, heads, [26, 19, 19, 10, 8, 9, 9, 9, 12, 9, 9, 9, 10, 10, 10])
        wins = [("whole log", power[0]["utc"], power[-1]["utc"])]
        wins += [(s["session"], s["start_utc"], s["end_utc"]) for s in sessions]
        for i, (lab, t0, t1) in enumerate(wins, 5):
            put(pw, i, 1, lab)
            put(pw, i, 2, t0, "yyyy-mm-dd hh:mm:ss")
            put(pw, i, 3, t1, "yyyy-mm-dd hh:mm:ss")
            w = f'{T},">="&$B{i},{T},"<="&$C{i}'
            put(pw, i, 4, f'=IF(AND($B{i}>=$B$5,$C{i}<=$C$5),"yes","NO")')
            put(pw, i, 5, f"=COUNTIFS({w})", "0")
            put(pw, i, 6, f'=IF(E{i}=0,"",_xlfn.MINIFS({E5},{w}))', "0.000")
            put(pw, i, 7, f'=IF(E{i}=0,"",AVERAGEIFS({E5},{w}))', "0.000")
            put(pw, i, 8, f'=IF(E{i}=0,"",_xlfn.MAXIFS({E5},{w}))', "0.000")
            put(pw, i, 9, f'=COUNTIFS({w},{TH},"<>0x0")', "0")
            put(pw, i, 10, f'=IF(E{i}=0,"",_xlfn.MAXIFS({TE},{w}))', "0.0")
            put(pw, i, 11, f'=IF(E{i}=0,"",_xlfn.MAXIFS({PV},{w}))', "0.000")
            put(pw, i, 12, f'=IF(E{i}=0,"",_xlfn.MINIFS({PV},{w}))', "0.000")
            put(pw, i, 13, f'=IF(E{i}=0,"",-AVERAGEIFS({SH},{w})/1000/{R})', "0.000")
            put(pw, i, 14, f'=IF(E{i}=0,"",-_xlfn.MINIFS({SH},{w})/1000/{R})', "0.000")
            put(pw, i, 15, f'=IF(E{i}=0,"",M{i}*AVERAGEIFS({PV},{w}))', "0.00")

    # ---- README -------------------------------------------------------------
    readme.column_dimensions["A"].width = 120
    text = [
        ("SPIRE Test B summary", BOLD),
        (f"Generated {datetime.now(timezone.utc):%Y-%m-%d %H:%M} UTC by testb_summary.py from: "
         + ", ".join(s["session"] for s in sessions)
         + (f"; power log {os.path.basename(args.power)}" if args.power else ""), None),
        ("", None),
        ("Sheets", BOLD),
        ("Sessions: configuration and health per session (exposure, gain, IMU rates and I2C "
         "errors, PID stale events, barometer).", None),
        ("Cycles: one row per cycle. Every other sheet is computed from it with formulas; "
         "filter it to inspect single cycles.", None),
        ("Bins: valid cycles by pre-burst platform rate, UNCORRECTED (servo held) vs "
         "CORRECTED/ACTIVE (servo on).", None),
        ("Criteria: inputs in blue (editable), verdicts B-1..B-4, Mann-Whitney U tests.", None),
        ("Selection: gyro3 burst rule (3-axis exposure score) vs the earlier gz rule.", None),
        ("Static: baseline with the servo held and the platform still.", None),
        ("Power / PowerRaw: supply during each session window (UTC).", None),
        ("", None),
        ("Definitions", BOLD),
        ("Rotation bins use the platform rate measured during the 1 s before the burst; "
         "the same value decides CALM/ACTIVE in CORRECTED mode.", None),
        ("With the servo active the bench platform speeds up during the burst (Cycles: "
         "'platform rate f1-f5 / f0', median ~1.65 CORRECTED vs ~0.98 UNCORRECTED), so the "
         "pre-burst bin understates the rate the servo works against. Criteria rows marked "
         "'sensitivity' bin by the platform rate at the winner exposure instead.", None),
        ("Winner = burst frame chosen by the gyro3 rule: min over frames of tilt blur "
         "(f_px x tilt angle range) + yaw blur at 0.25 R (r x yaw angle range), angles "
         "integrated from the camera gyro over the exposure.", None),
        ("|cam_gz| = camera yaw rate at the frame's exposure (single sample nearest the "
         "exposure). Attenuation = |cam_gz| / |plat_gz| per frame, frames with "
         "|plat_gz| >= 5 deg/s.", None),
        ("Compensation ratio = -(cam_gz - plat_gz) / plat_gz (1.0 = servo cancels the "
         "platform rate exactly). Servo gain = (cam_gz - plat_gz) / commanded servo rate.", None),
        ("", None),
        ("Known limitations", BOLD),
        ("Bench lens 6 mm, flight lens 16 mm: tilt blur in px scales by 16/6; yaw blur at a "
         "given radius does not depend on the lens.", None),
        ("Bench exposure 20 ms (mains flicker lock). The B-1 rate limit 3.4 deg/s is defined "
         "for 1/150 s; at 20 ms it corresponds to ~3 px corner blur.", None),
        ("STATIC validity re-judged: the logged rule applied the 16 mm linear model to the "
         "single-sample |cam_gz|; a residual camera-gyro bias of ~0.7 deg/s alone exceeds its "
         "1 px limit. The score rule (column F) is used instead; column E keeps the logged value.", None),
        ("B-3 and B-4 from 'frames (proxy)' use the six frame samples per cycle, not the full "
         "IMU trace. Final values need the c###_imu.csv files in the session folders.", None),
        ("Barometer: MPRLS reports a memory integrity fault (status 0x44); values are logged with "
         "baro_ok = false and are not a calibrated altitude.", None),
        ("bench.log timestamps are Pi local time (UTC+1); all times in this workbook are UTC.", None),
        ("Power: pack current requires the UPS shunt value (Power!B1, yellow = assumption).", None),
    ]
    for i, (t, f) in enumerate(text, 1):
        c = readme.cell(row=i, column=1, value=t)
        c.font = f or BODY
        c.alignment = WRAP

    finish_fonts(wb)
    wb.save(args.output)
    print(f"Written {args.output}")

    if args.figures:
        figures(cycles, args.figures)


# ---------------------------------------------------------------------------
# Figures
# ---------------------------------------------------------------------------

def figures(cycles, out):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    os.makedirs(out, exist_ok=True)
    plt.rcParams.update({"font.family": "DejaVu Sans", "font.size": 9,
                         "axes.spines.top": False, "axes.spines.right": False,
                         "axes.edgecolor": "#52514e", "axes.labelcolor": "#0b0b0b",
                         "xtick.color": "#52514e", "ytick.color": "#52514e"})
    sets = [("UNCORRECTED", "HELD", "Servo held (UNCORRECTED)", "#eb6834", "o"),
            ("CORRECTED", "ACTIVE", "Servo active (CORRECTED)", "#2a78d6", "^")]

    def sel(mode, status):
        return [c for c in cycles if c["mode"] == mode and c["status"] == status and c["valid"]]

    # 1: winner |cam_gz| vs platform rate
    fig, ax = plt.subplots(figsize=(6.3, 3.6))
    for mode, status, lab, col, mk in sets:
        cs = [c for c in sel(mode, status) if c["win_abs_gz"] is not None]
        ax.scatter([c["rot_dps"] for c in cs], [c["win_abs_gz"] for c in cs], s=22,
                   marker=mk, color=col, edgecolor="white", linewidth=0.6,
                   label=f"{lab}, n={len(cs)}", zorder=3)
    ax.axhline(WINNER_LIMIT_DPS, color="#52514e", linewidth=1, linestyle="--", zorder=2)
    ax.text(0.99, WINNER_LIMIT_DPS + 1.5, f"B-1 limit {WINNER_LIMIT_DPS} deg/s",
            color="#52514e", fontsize=8, ha="right", transform=ax.get_yaxis_transform())
    ax.set_xlabel("Platform yaw rate before burst [deg/s]")
    ax.set_ylabel("Winner frame camera yaw rate [deg/s]")
    ax.grid(True, color="#e6e5e1", linewidth=0.6, zorder=0)
    ax.legend(frameon=False, loc="upper left")
    fig.tight_layout()
    for ext in ("png", "pdf"):
        fig.savefig(os.path.join(out, f"testB_winner_rate.{ext}"), dpi=200)
    plt.close(fig)

    # 2: winner score per bin (box plots)
    labs = [bin_label(b) for b in BINS[1:]]
    fig, ax = plt.subplots(figsize=(6.3, 3.6))
    width = 0.36
    handles = []
    for k, (mode, status, lab, col, _mk) in enumerate(sets):
        data, pos = [], []
        for i, b in enumerate(labs):
            v = [c["win_score"] for c in sel(mode, status)
                 if c["bin"] == b and c["win_score"] is not None]
            if v:
                data.append(v)
                pos.append(i + (k - 0.5) * width * 1.1)
        bp = ax.boxplot(data, positions=pos, widths=width, patch_artist=True,
                        showfliers=True, medianprops={"color": "#0b0b0b", "linewidth": 1.5},
                        flierprops={"marker": ".", "markersize": 4, "markerfacecolor": col,
                                    "markeredgecolor": col},
                        whiskerprops={"color": "#52514e"}, capprops={"color": "#52514e"})
        for p in bp["boxes"]:
            p.set_facecolor(col)
            p.set_edgecolor("white")
            p.set_alpha(0.9)
        handles.append(plt.Rectangle((0, 0), 1, 1, color=col, label=lab))
    counts = [[sum(1 for c in sel(m, st) if c["bin"] == b and c["win_score"] is not None)
               for m, st, *_r in sets] for b in labs]
    ax.set_xticks(range(len(labs)))
    ax.set_xticklabels([f"{b}\nn={u}/{c}" for b, (u, c) in zip(labs, counts)])
    ax.set_xlabel("Platform yaw rate bin [deg/s]")
    ax.set_ylabel("Winner exposure blur score [px]")
    ax.grid(True, axis="y", color="#e6e5e1", linewidth=0.6)
    ax.set_axisbelow(True)
    ax.legend(handles=handles, frameon=False, loc="upper left")
    fig.tight_layout()
    for ext in ("png", "pdf"):
        fig.savefig(os.path.join(out, f"testB_score_bins.{ext}"), dpi=200)
    plt.close(fig)
    print(f"Figures written to {out}")


if __name__ == "__main__":
    main()