"""Parse the 8 Nov A0 sweeps from raw_logs.txt, write CSVs and an SVG chart.

Standard library only. Run from anywhere:
    python3 make_chart.py
Outputs (next to this file):
    s1_sweep.csv, s2_sweep.csv   every frame of each sweep
    s2_summary.csv               S2 frames grouped by angle (nearest 0.5 deg)
    a0_sweep_s2.svg              chart: A0, A1, A3 against gyro angle (S2)
"""

import csv
import os
import re

HERE = os.path.dirname(os.path.abspath(__file__))
LINE = re.compile(
    r"(?P<time>[\d:.]+) -> A0:(?P<a0>\d+)\s+A1:(?P<a1>\d+)\s+A2:(?P<a2>\d+)\s+A3:(?P<a3>\d+)"
    r"\s+hold (?P<hold>-?[\d.]+) gyro (?P<gyro>-?[\d.]+)\s+->\s*(?P<dec>.*)"
)

# S2 is the clean sweep: only frames after the reset at 14:54:33
S2_FIRST = "14:54:33.965"
FIRE_NOW = 96000        # FIRE_MIN_US in main_see_fire today
FIRE_NEW = 110000       # proposed fire level
TOP = (2.0, 8.4)        # saturated top of A0 in S2 (deg)


def read_sections():
    sections, name = {}, None
    with open(os.path.join(HERE, "raw_logs.txt")) as f:
        for raw in f:
            if raw.startswith("## S1 "):
                name = "S1"
            elif raw.startswith("## S2 "):
                name = "S2"
            elif re.match(r"## R\d", raw):      # next non-sweep run ("## Reset ..." is a note)
                name = None
            m = LINE.search(raw)
            if name and m:
                sections.setdefault(name, []).append(m.groupdict())
    return sections


def to_rows(frames):
    rows = []
    for fr in frames:
        rows.append({
            "time": fr["time"], "gyro": float(fr["gyro"]), "hold": float(fr["hold"]),
            "A0": int(fr["a0"]), "A1": int(fr["a1"]), "A2": int(fr["a2"]), "A3": int(fr["a3"]),
            "decision": fr["dec"].strip(),
        })
    return rows


def write_csv(path, rows, fields):
    with open(path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=fields)
        w.writeheader()
        w.writerows(rows)


def summarise(rows):
    groups = {}
    for r in rows:
        key = round(r["gyro"] * 2) / 2
        groups.setdefault(key, []).append(r)
    out = []
    for key in sorted(groups):
        g = groups[key]
        n = len(g)
        rec = {"angle": key, "n": n}
        for s in ("A0", "A1", "A2", "A3"):
            vals = [r[s] for r in g]
            rec[s + "_mean"] = round(sum(vals) / n)
            rec[s + "_min"] = min(vals)
            rec[s + "_max"] = max(vals)
        rec["A3_minus_A1"] = rec["A3_mean"] - rec["A1_mean"]
        out.append(rec)
    return out


# ---------------------------------------------------------------- chart
W, H = 880, 520
ML, MR, MT, MB = 76, 120, 92, 64
X0, X1 = -7.0, 17.0
Y0, Y1 = 50000, 122000
SERIES = [  # key, label, css var
    ("A0", "A0 front (with tube)", "--s1"),
    ("A1", "A1 right", "--s2"),
    ("A3", "A3 left", "--s3"),
]


def sx(x):
    return ML + (x - X0) / (X1 - X0) * (W - ML - MR)


def sy(y):
    return MT + (Y1 - y) / (Y1 - Y0) * (H - MT - MB)


def fmt(v):
    return f"{v:,}"


def svg_chart(rows, summary):
    p = []
    p.append(f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {W} {H}" '
             f'width="{W}" height="{H}" font-family="system-ui, -apple-system, Segoe UI, sans-serif" '
             f'role="img" aria-labelledby="t d">')
    p.append("""<style>
  svg { --bg:#fcfcfb; --ink:#0b0b0b; --ink2:#52514e; --ink3:#8a8984; --grid:#e8e7e3;
        --band:#2a78d6; --s1:#2a78d6; --s2:#eb6834; --s3:#1baf7a; }
  @media (prefers-color-scheme: dark) {
    svg { --bg:#1a1a19; --ink:#f0efec; --ink2:#c3c2b7; --ink3:#8f8e86; --grid:#2e2e2c;
          --band:#3987e5; --s1:#3987e5; --s2:#d95926; --s3:#199e70; }
  }
  .bg { fill: var(--bg); }
  .title { fill: var(--ink); font-size: 18px; font-weight: 600; }
  .sub { fill: var(--ink2); font-size: 13px; }
  .tick { fill: var(--ink2); font-size: 12px; }
  .axis-label { fill: var(--ink2); font-size: 12px; }
  .grid { stroke: var(--grid); stroke-width: 1; }
  .zero { stroke: var(--ink3); stroke-width: 1; }
  .ref { stroke: var(--ink2); stroke-width: 1.5; stroke-dasharray: 5 4; fill: none; }
  .ref-label { fill: var(--ink2); font-size: 11px; }
  .band { fill: var(--band); opacity: 0.08; }
  .band-label { fill: var(--ink2); font-size: 11px; }
  .lab { font-size: 12px; fill: var(--ink); font-weight: 600; }
  .note { fill: var(--ink2); font-size: 11px; }
  .dot { opacity: 0.35; }
  .pt { stroke: var(--bg); stroke-width: 2; }
  .pt:hover { stroke: var(--ink); }
</style>""")
    p.append('<title id="t">A0, A1 and A3 readings against body angle (sweep S2, 8 Nov 2026)</title>')
    p.append('<desc id="d">A0 is saturated at about 119,500 us from +2.0 to +8.4 degrees and falls about 6,000 us '
             'per degree on both sides. A1 rises and A3 falls as the body turns anticlockwise.</desc>')
    p.append(f'<rect class="bg" x="0" y="0" width="{W}" height="{H}"/>')
    p.append(f'<text class="title" x="{ML}" y="32">A0 has a 6.4°-wide flat top with even slopes on both sides</text>')
    p.append(f'<text class="sub" x="{ML}" y="54">Beacon 1 m in front · robot turned by hand on the table · '
             f'sweep S2, 8 Nov 2026 · dots = single frames, line = mean per 0.5°</text>')

    # legend, one row under the subtitle
    lx = ML
    for key, label, var in SERIES:
        p.append(f'<line x1="{lx}" x2="{lx + 18}" y1="76" y2="76" stroke="var({var})" stroke-width="2"/>')
        p.append(f'<circle cx="{lx + 9}" cy="76" r="4" fill="var({var})"/>')
        p.append(f'<text class="tick" x="{lx + 26}" y="80">{label}</text>')
        lx += 26 + 8 * len(label) + 24

    # saturated band, label at its foot (the top is where A0's flat line runs)
    bx0, bx1 = sx(TOP[0]), sx(TOP[1])
    p.append(f'<rect class="band" x="{bx0:.1f}" y="{MT}" width="{bx1 - bx0:.1f}" height="{H - MT - MB}"/>')
    p.append(f'<text class="band-label" x="{(bx0 + bx1) / 2:.1f}" y="{H - MB - 22}" text-anchor="middle">'
             f'A0 saturated</text>')
    p.append(f'<text class="band-label" x="{(bx0 + bx1) / 2:.1f}" y="{H - MB - 8}" text-anchor="middle">'
             f'+2.0° to +8.4° · centre ≈ +5.2°</text>')

    # grid + y ticks
    for y in range(50000, 122001, 10000):
        yy = sy(y)
        p.append(f'<line class="grid" x1="{ML}" x2="{W - MR}" y1="{yy:.1f}" y2="{yy:.1f}"/>')
        p.append(f'<text class="tick" x="{ML - 8}" y="{yy + 4:.1f}" text-anchor="end">{y // 1000}k</text>')
    p.append(f'<text class="axis-label" x="16" y="{(MT + H - MB) / 2:.1f}" '
             f'transform="rotate(-90 16 {(MT + H - MB) / 2:.1f})" text-anchor="middle">LOW-pulse width (µs)</text>')

    # x ticks
    for x in range(-6, 17, 2):
        xx = sx(x)
        p.append(f'<line class="{"zero" if x == 0 else "grid"}" x1="{xx:.1f}" x2="{xx:.1f}" '
                 f'y1="{MT}" y2="{H - MB}"/>')
        p.append(f'<text class="tick" x="{xx:.1f}" y="{H - MB + 18}" text-anchor="middle">{x:+d}°</text>'
                 if x else f'<text class="tick" x="{xx:.1f}" y="{H - MB + 18}" text-anchor="middle">0°</text>')
    p.append(f'<text class="axis-label" x="{(ML + W - MR) / 2:.1f}" y="{H - 18}" text-anchor="middle">'
             f'gyro angle (° , + = anticlockwise from above; 0 = start position)</text>')

    # fire level reference lines
    # label above or below the line, whichever side is clear of the data at the left end
    for level, text, dy in ((FIRE_NOW, "fire level today 96k", 14), (FIRE_NEW, "proposed 110k", -5)):
        yy = sy(level)
        p.append(f'<line class="ref" x1="{ML}" x2="{W - MR}" y1="{yy:.1f}" y2="{yy:.1f}"/>')
        p.append(f'<text class="ref-label" x="{sx(-6.8):.1f}" y="{yy + dy:.1f}">{text}</text>')

    # raw frames (faint) then mean lines with markers
    for key, label, var in SERIES:
        for r in rows:
            p.append(f'<circle class="dot" cx="{sx(r["gyro"]):.1f}" cy="{sy(r[key]):.1f}" r="2.5" '
                     f'fill="var({var})"/>')
    for key, label, var in SERIES:
        pts = [(sx(s["angle"]), sy(s[key + "_mean"])) for s in summary]
        path = " ".join(f'{"M" if i == 0 else "L"}{x:.1f},{y:.1f}' for i, (x, y) in enumerate(pts))
        p.append(f'<path d="{path}" fill="none" stroke="var({var})" stroke-width="2" '
                 f'stroke-linejoin="round" stroke-linecap="round"/>')
        for s, (x, y) in zip(summary, pts):
            tip = (f'{label} at {s["angle"]:+.1f}°: mean {fmt(s[key + "_mean"])} µs '
                   f'(min {fmt(s[key + "_min"])}, max {fmt(s[key + "_max"])}, {s["n"]} frames)')
            p.append(f'<circle class="pt" cx="{x:.1f}" cy="{y:.1f}" r="4.5" fill="var({var})">'
                     f'<title>{tip}</title></circle>')

    # direct labels at the right end of each line
    last = summary[-1]
    ends = sorted(((sy(last[k + "_mean"]), lab, var) for k, lab, var in SERIES), key=lambda t: t[0])
    prev = -1e9
    for y, lab, var in ends:
        y = max(y, prev + 16)
        prev = y
        p.append(f'<circle cx="{W - MR + 10}" cy="{y:.1f}" r="4.5" fill="var({var})"/>')
        p.append(f'<text class="lab" x="{W - MR + 19}" y="{y + 4:.1f}">{lab.split(" (")[0]}</text>')

    p.append("</svg>")
    return "\n".join(p)


def main():
    sec = read_sections()
    s1 = to_rows(sec["S1"])
    s2_all = to_rows(sec["S2"])
    start = next(i for i, r in enumerate(s2_all) if r["time"] == S2_FIRST)
    s2 = s2_all[start:]

    fields = ["time", "gyro", "hold", "A0", "A1", "A2", "A3", "decision"]
    write_csv(os.path.join(HERE, "s1_sweep.csv"), s1, fields)
    write_csv(os.path.join(HERE, "s2_sweep.csv"), s2, fields)
    summary = summarise(s2)
    write_csv(os.path.join(HERE, "s2_summary.csv"), summary, list(summary[0].keys()))
    with open(os.path.join(HERE, "a0_sweep_s2.svg"), "w") as f:
        f.write(svg_chart(s2, summary))
    print(f"S1 {len(s1)} frames, S2 {len(s2)} frames, {len(summary)} angle groups")
    for s in summary:
        print(f'{s["angle"]:+5.1f}  n={s["n"]:2d}  A0={s["A0_mean"]:7,}  A1={s["A1_mean"]:7,}  '
              f'A3={s["A3_mean"]:7,}  A3-A1={s["A3_minus_A1"]:+7,}')


if __name__ == "__main__":
    main()
