#!/usr/bin/env python3
"""
SPIRE bench report (Test B): pools bench sessions, bins cycles by measured
rotation speed and evaluates criteria B-1 to B-4 of the test plan.

Per cycle it uses the session's manifest.csv (one row per burst frame) and,
for CORRECTED cycles, the servo response from c###_imu.csv (servo_response.py).
Sessions are grouped by configuration (rate_gain, gyro_deadband, exposure);
groups are never pooled together, so a parameter change shows as a separate
block in the report.

Criteria (test plan, Test B):
  B-1  10-20 and 20-40 deg/s: >= 80% of CORRECTED winners |cam_gz| <= 3.4 deg/s
  B-2  5-40 deg/s: CORRECTED median attenuation <= 0.25
  B-3  servo gain 0.9-1.1 (median over CORRECTED cycles)
  B-4  zero NO RESPONSE cycles (servo gain < 0.15)
  Mann-Whitney U, two-sided, alpha 0.05: CORRECTED vs UNCORRECTED winner rates

Usage:
  python3 src/bench_report.py SESSION_DIR [SESSION_DIR ...]
  python3 src/bench_report.py --since 20261028          # all sessions from that UTC date
Writes <stamp>_bins.csv, <stamp>_cycles.csv and <stamp>_report.md to
/mnt/spire_data/reports (or -o).
"""

import argparse
import csv
import json
import math
import os
import sys
from datetime import datetime, timezone
from statistics import median

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import storage  # noqa: E402
import servo_response  # noqa: E402

BINS = [(0, 5), (5, 10), (10, 20), (20, 40), (40, 70), (70, math.inf)]
WINNER_LIMIT_DPS = 3.4       # 1 px corner blur at 1/150 s (test plan, Test B)
B1_SHARE = 0.80
B2_MAX_ATT = 0.25
B3_GAIN = (0.9, 1.1)
NO_RESPONSE_GAIN = 0.15
ATT_MIN_PLAT_DPS = 5.0
SAT_DEG = 134.0
ALPHA = 0.05


# ---------------------------------------------------------------------------
# Small statistics helpers (no scipy needed on the Pi)
# ---------------------------------------------------------------------------

def quartiles(v):
    """(q1, median, q3) with linear interpolation; None for an empty list."""
    if not v:
        return None
    s = sorted(v)

    def q(p):
        k = (len(s) - 1) * p
        f = math.floor(k)
        c = min(f + 1, len(s) - 1)
        return s[f] + (s[c] - s[f]) * (k - f)
    return q(0.25), q(0.5), q(0.75)


def mann_whitney(a, b):
    """Two-sided Mann-Whitney U, normal approximation with tie and
    continuity correction. Returns (U, p) or (None, None) if n is too small."""
    n1, n2 = len(a), len(b)
    if n1 < 5 or n2 < 5:
        return None, None
    pooled = sorted([(x, 0) for x in a] + [(x, 1) for x in b])
    ranks = [0.0] * len(pooled)
    ties = []
    i = 0
    while i < len(pooled):
        j = i
        while j + 1 < len(pooled) and pooled[j + 1][0] == pooled[i][0]:
            j += 1
        r = (i + j) / 2 + 1
        for k in range(i, j + 1):
            ranks[k] = r
        if j > i:
            ties.append(j - i + 1)
        i = j + 1
    r1 = sum(r for r, (_, g) in zip(ranks, pooled) if g == 0)
    u1 = r1 - n1 * (n1 + 1) / 2
    u = min(u1, n1 * n2 - u1)
    n = n1 + n2
    tie_term = sum(t ** 3 - t for t in ties) / (n * (n - 1))
    sigma = math.sqrt(n1 * n2 / 12 * ((n + 1) - tie_term))
    if sigma == 0:
        return u, 1.0
    z = (abs(u1 - n1 * n2 / 2) - 0.5) / sigma
    p = math.erfc(max(z, 0) / math.sqrt(2))
    return u, min(1.0, p)


# ---------------------------------------------------------------------------
# Loading
# ---------------------------------------------------------------------------

def fnum(x):
    try:
        return float(x)
    except (TypeError, ValueError):
        return None


def bin_of(speed):
    for lo, hi in BINS:
        if lo <= speed < hi:
            return (lo, hi)
    return BINS[-1]


def bin_label(b):
    lo, hi = b
    return f"{lo:g}+" if hi == math.inf else f"{lo:g}-{hi:g}"


def load_session(path):
    """List of cycle dicts for one session folder."""
    meta = {}
    sj = os.path.join(path, "session.json")
    if os.path.exists(sj):
        with open(sj) as fh:
            meta = json.load(fh)
    fp = meta.get("flight_params", {})
    bp = meta.get("bench_params", {})
    config = (f"gain={fp.get('rate_gain', '?')} deadband={fp.get('gyro_deadband', '?')} "
              f"exposure={bp.get('exposure_us', '?')}us")

    rows = []
    mpath = os.path.join(path, "manifest.csv")
    if not os.path.exists(mpath):
        return []
    with open(mpath, newline="") as fh:
        rows = list(csv.DictReader(fh))

    cycles = {}
    for r in rows:
        cid = int(r["cycle_id"])
        c = cycles.setdefault(cid, {
            "session": os.path.basename(path), "cycle": cid, "config": config,
            "mode": r.get("mode", ""), "status": r.get("effective_status", ""),
            "valid": r.get("cycle_valid", "") in ("1", "True", "true"),
            "speed": fnum(r.get("rotation_speed_dps")) or 0.0,
            "frames": []})
        c["frames"].append(r)

    out = []
    for cid, c in sorted(cycles.items()):
        fr = c["frames"]
        win = [f for f in fr if f.get("winner") in ("1", "True", "true")]
        w = win[0] if win else None
        atts = [fnum(f.get("attenuation")) for f in fr
                if fnum(f.get("attenuation")) is not None
                and fnum(f.get("plat_gz")) is not None
                and abs(fnum(f.get("plat_gz"))) >= ATT_MIN_PLAT_DPS]
        sat = [f for f in fr if fnum(f.get("servo_cmd_deg")) is not None]
        c.update({
            "bin": bin_of(c["speed"]),
            "winner_dps": abs(fnum(w.get("cam_gz"))) if w and fnum(w.get("cam_gz")) is not None else None,
            "winner_blur_px": fnum(w.get("blur_px_rot_corner")) if w else None,
            "attenuation": median(atts) if atts else None,
            "sat_share": (sum(abs(fnum(f["servo_cmd_deg"])) >= SAT_DEG for f in sat) / len(sat)
                          if sat else None),
            "gain": None,
        })
        if c["mode"] == "CORRECTED" and c["status"] == "ACTIVE":
            imu = os.path.join(path, f"c{cid:03d}_imu.csv")
            if os.path.exists(imu):
                r = servo_response.analyse_cycle(imu)
                c["gain"] = r["gain"] if r else None
        del c["frames"]
        out.append(c)
    return out


# ---------------------------------------------------------------------------
# Report
# ---------------------------------------------------------------------------

def fmt(v, f=".2f"):
    return "-" if v is None else format(v, f)


def summarize(cycles):
    """Rows per (config, mode, status, bin) plus criteria per config."""
    groups = {}
    for c in cycles:
        key = (c["config"], c["mode"], c["status"], c["bin"])
        groups.setdefault(key, []).append(c)

    rows = []
    for key in sorted(groups, key=lambda k: (k[0], k[1], k[2], k[3][0])):
        cs = groups[key]
        valid = [c for c in cs if c["valid"]]
        wins = [c["winner_dps"] for c in valid if c["winner_dps"] is not None]
        atts = [c["attenuation"] for c in valid if c["attenuation"] is not None]
        gains = [c["gain"] for c in valid if c["gain"] is not None]
        sats = [c["sat_share"] for c in valid if c["sat_share"] is not None]
        blur = [c["winner_blur_px"] for c in valid if c["winner_blur_px"] is not None]
        wq = quartiles(wins)
        rows.append({
            "config": key[0], "mode": key[1], "status": key[2], "bin": bin_label(key[3]),
            "bin_lo": key[3][0], "n_total": len(cs), "n_valid": len(valid),
            "winner_med": wq[1] if wq else None,
            "winner_q1": wq[0] if wq else None, "winner_q3": wq[2] if wq else None,
            "winner_ok_share": (sum(x <= WINNER_LIMIT_DPS for x in wins) / len(wins)
                                if wins else None),
            "att_med": median(atts) if atts else None,
            "gain_med": median(gains) if gains else None,
            "no_response": sum(g < NO_RESPONSE_GAIN for g in gains),
            "sat_share": sum(sats) / len(sats) if sats else None,
            "blur_med_px": median(blur) if blur else None,
            "_wins": wins,
        })
    return rows


def criteria(rows, cycles):
    """Verdict lines per configuration."""
    lines = []
    for cfg in sorted({r["config"] for r in rows}):
        rs = [r for r in rows if r["config"] == cfg]
        cor = {r["bin"]: r for r in rs if r["mode"] == "CORRECTED" and r["status"] == "ACTIVE"}
        unc = {r["bin"]: r for r in rs if r["mode"] == "UNCORRECTED"}
        lines.append(f"### Configuration: {cfg}")
        lines.append("")

        def verdict(ok, enough):
            return "not enough data" if not enough else ("PASS" if ok else "FAIL")

        for b in ("10-20", "20-40"):
            r = cor.get(b)
            n = r["n_valid"] if r else 0
            share = r["winner_ok_share"] if r else None
            lines.append(f"- B-1 {b} deg/s: {fmt(share, '.0%') if share is not None else '-'} "
                         f"of winners <= {WINNER_LIMIT_DPS} deg/s (n={n}, need >= 10): "
                         f"{verdict(share is not None and share >= B1_SHARE, n >= 10)}")
        for b in ("5-10", "10-20", "20-40"):
            r = cor.get(b)
            n = r["n_valid"] if r else 0
            att = r["att_med"] if r else None
            u = unc.get(b)
            lines.append(f"- B-2 {b} deg/s: median attenuation {fmt(att)} "
                         f"(UNCORRECTED {fmt(u['att_med'] if u else None)}), n={n}: "
                         f"{verdict(att is not None and att <= B2_MAX_ATT, n >= 10)}")
        cg = [c["gain"] for c in cycles if c["config"] == cfg and c["valid"]
              and c["mode"] == "CORRECTED" and c["gain"] is not None]
        g = median(cg) if cg else None
        lines.append(f"- B-3 servo gain: median {fmt(g)} over {len(cg)} cycles: "
                     f"{verdict(g is not None and B3_GAIN[0] <= g <= B3_GAIN[1], len(cg) >= 10)}")
        nr = sum(x < NO_RESPONSE_GAIN for x in cg)
        lines.append(f"- B-4 NO RESPONSE cycles: {nr}: "
                     f"{verdict(nr == 0, len(cg) >= 10)}")
        for b in ("5-10", "10-20", "20-40", "40-70"):
            r, u = cor.get(b), unc.get(b)
            if not r or not u:
                continue
            U, p = mann_whitney(r["_wins"], u["_wins"])
            if U is None:
                lines.append(f"- Mann-Whitney {b} deg/s: n too small "
                             f"({len(r['_wins'])} vs {len(u['_wins'])}, need >= 5 each)")
            else:
                lines.append(f"- Mann-Whitney {b} deg/s: CORRECTED median "
                             f"{fmt(r['winner_med'])} vs UNCORRECTED {fmt(u['winner_med'])} deg/s, "
                             f"U={U:.0f}, p={p:.3g} "
                             f"({'significant' if p < ALPHA else 'not significant'} at {ALPHA})")
        lines.append("")
    return lines


def main():
    ap = argparse.ArgumentParser(description="SPIRE bench report (Test B)")
    ap.add_argument("sessions", nargs="*", help="Session folders")
    ap.add_argument("--since", help="Use every session from this UTC date (YYYYMMDD)")
    ap.add_argument("-o", "--output", default=None,
                    help="Report folder (default: /mnt/spire_data/reports)")
    ap.add_argument("--allow-local", action="store_true",
                    help="Write to ./data/reports if the USB drive is missing")
    args = ap.parse_args()

    sessions = list(args.sessions)
    if args.since:
        sessions += [s for s in storage.find_sessions("bench")
                     if os.path.basename(s)[8:16] >= args.since]
    if not sessions:
        sys.exit("No sessions given (list folders or use --since YYYYMMDD)")

    cycles = []
    for s in sessions:
        cs = load_session(s)
        print(f"{os.path.basename(s)}: {len(cs)} cycles")
        cycles += cs
    if not cycles:
        sys.exit("No cycles found")

    rows = summarize(cycles)
    out_dir = args.output or storage.data_dir("reports", allow_local=args.allow_local)
    os.makedirs(out_dir, exist_ok=True)
    stamp = datetime.now(timezone.utc).strftime("%Y%m%d_%H%M%S")

    bin_fields = ["config", "mode", "status", "bin", "n_total", "n_valid",
                  "winner_med", "winner_q1", "winner_q3", "winner_ok_share",
                  "att_med", "gain_med", "no_response", "sat_share", "blur_med_px"]
    with open(os.path.join(out_dir, f"{stamp}_bins.csv"), "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=bin_fields, extrasaction="ignore")
        w.writeheader()
        w.writerows(rows)
    cyc_fields = ["session", "cycle", "config", "mode", "status", "valid", "speed",
                  "winner_dps", "winner_blur_px", "attenuation", "sat_share", "gain"]
    with open(os.path.join(out_dir, f"{stamp}_cycles.csv"), "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=cyc_fields, extrasaction="ignore")
        w.writeheader()
        for c in cycles:
            w.writerow(dict(c, bin=bin_label(c["bin"])))

    md = [f"# Bench report {stamp} UTC", "",
          f"Sessions: {', '.join(os.path.basename(s) for s in sessions)}", "",
          "| config | mode | status | bin deg/s | n valid/total | winner median (IQR) deg/s "
          "| winners <= 3.4 | attenuation | servo gain | sat. share |",
          "| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |"]
    for r in rows:
        iqr = (f"{fmt(r['winner_med'])} ({fmt(r['winner_q1'])}-{fmt(r['winner_q3'])})"
               if r["winner_med"] is not None else "-")
        md.append(f"| {r['config']} | {r['mode']} | {r['status']} | {r['bin']} | "
                  f"{r['n_valid']}/{r['n_total']} | {iqr} | "
                  f"{fmt(r['winner_ok_share'], '.0%') if r['winner_ok_share'] is not None else '-'} | "
                  f"{fmt(r['att_med'])} | {fmt(r['gain_med'])} | "
                  f"{fmt(r['sat_share'], '.0%') if r['sat_share'] is not None else '-'} |")
    md += ["", "## Criteria", ""] + criteria(rows, cycles)
    report = "\n".join(md)
    with open(os.path.join(out_dir, f"{stamp}_report.md"), "w") as fh:
        fh.write(report + "\n")
    print()
    print(report)
    print(f"Written to {out_dir}/{stamp}_*")


if __name__ == "__main__":
    main()
