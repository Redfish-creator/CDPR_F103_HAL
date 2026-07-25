#!/usr/bin/env python3
"""Fit an empirical CDPR encoder/rope model from PATROL SEG serial logs.

The firmware reports a command-space point (soft) and, when available, a
fresh visual laser point.  This tool deliberately uses the laser point as the
model input and keeps soft only as metadata.  It rejects communication
failures, incomplete encoder reads, and stale vision samples.

Example:
    python tools/fit_rope_model.py --log run.txt \
        --out data/rope_model.json --header data/rope_model_generated.h

The generated model predicts encoder-equivalent targets.  A real free-rope
length still requires a separately measured effective spool radius and
winding-layer model; encoder counts are not a tension measurement.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import re
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence, Tuple


MOTORS = 4
DEFAULT_SITE = Path(__file__).resolve().parents[1] / "data" / "site_parameters.csv"


@dataclass
class Segment:
    sequence: int
    name: str
    step: int
    segment: int
    fine: int
    status: str
    complete: int
    bal: int
    x_soft: float
    y_soft: float
    x_next: float
    y_next: float
    d_l: List[float]
    cmd: List[float]
    gain: List[float]
    enc_after: List[Optional[int]]
    enc_mask: int
    laser_x: Optional[float]
    laser_y: Optional[float]
    laser_age_ms: Optional[int]
    quality: str
    baseline: List[Optional[int]]


FLOAT = r"[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?"
INT = r"[-+]?\d+"
SEG_RE = re.compile(
    rf"PATROL SEG seq=(?P<seq>\d+)\s+name=(?P<name>\S+)\s+"
    rf"step=(?P<step>\d+)/(?P<total>\d+)\s+seg=(?P<seg>\d+)/(?P<segtotal>\d+)\s+"
    rf"fine=(?P<fine>\d+)\s+status=(?P<status>\S+)\s+complete=(?P<complete>\d+)\s+"
    rf"move_ms=(?P<move>\d+)\s+bal=(?P<bal>\d+)\s+"
    rf"(?:release<={FLOAT}\s+)?"
    rf"from=\((?P<x0>{FLOAT}),(?P<y0>{FLOAT})\)\s+"
    rf"to=\((?P<x1>{FLOAT}),(?P<y1>{FLOAT})\)\s+"
    rf"dL=\[(?P<dl>[^\]]+)\]\s+cmd=\[(?P<cmd>[^\]]+)\]\s+"
    rf"gain=\[(?P<gain>[^\]]+)\]"
)
DATA_RE = re.compile(
    rf"PATROL SEG DATA seq=(?P<seq>\d+).*?"
    rf"enc_after=\[(?P<enc>[^\]]+)\]\s+mask=0x(?P<mask>[0-9A-Fa-f]+).*?"
    rf"laser=\((?P<lx>{FLOAT}),(?P<ly>{FLOAT}),(?P<lage>\d+)ms,cnt=(?P<lcnt>\d+)\)"
    rf".*?quality=(?P<quality>\S+)"
)
BASE_RE = re.compile(
    r"PATROL SEG BASE enc=\[(?P<enc>[^\]]+)\]\s+mask=0x(?P<mask>[0-9A-Fa-f]+)"
)


def numbers(text: str, count: int, cast=float) -> List:
    values = []
    for token in text.split(","):
        token = token.strip()
        if token.upper() in {"NA", "N/A", ""}:
            values.append(None)
        else:
            values.append(cast(token))
    if len(values) != count:
        raise ValueError(f"expected {count} values, got {len(values)}: {text!r}")
    return values


def parse_site(path: Path) -> Dict[str, float]:
    values: Dict[str, float] = {}
    with path.open("r", encoding="utf-8-sig", newline="") as handle:
        for row in csv.DictReader(handle):
            value = row.get("value", "")
            try:
                values[f"{row.get('item', '')}.{row.get('axis_or_motor', '')}"] = float(value)
            except (TypeError, ValueError):
                continue
    required = {
        "motor_anchor.M1_x",
        "motor_anchor.M1_y",
        "motor_anchor.M2_x",
        "motor_anchor.M2_y",
        "motor_anchor.M3_x",
        "motor_anchor.M3_y",
        "motor_anchor.M4_x",
        "motor_anchor.M4_y",
        "height.dz",
        "gondola.half_length_x",
        "gondola.half_width_y",
    }
    missing = sorted(required - values.keys())
    if missing:
        raise ValueError(f"site file is missing confirmed geometry values: {missing}")
    return values


def geometry_lengths(x: float, y: float, site: Dict[str, float]) -> List[float]:
    out = []
    gx = site["gondola.half_length_x"]
    gy = site["gondola.half_width_y"]
    dz = site["height.dz"]
    for motor in range(1, MOTORS + 1):
        px = site[f"motor_anchor.M{motor}_x"]
        py = site[f"motor_anchor.M{motor}_y"]
        ax = -gx if motor in (1, 4) else gx
        ay = -gy if motor in (1, 3) else gy
        ex = px - (x + ax)
        ey = py - (y + ay)
        out.append(math.sqrt(ex * ex + ey * ey + dz * dz))
    return out


def parse_log(path: Path) -> List[Segment]:
    pending: Dict[int, Segment] = {}
    segments: List[Segment] = []
    baseline: List[Optional[int]] = [None] * MOTORS
    with path.open("r", encoding="utf-8", errors="replace") as handle:
        for raw in handle:
            line = raw.strip()
            base_match = BASE_RE.search(line)
            if base_match:
                raw_base = numbers(base_match.group("enc"), MOTORS, int)
                mask = int(base_match.group("mask"), 16)
                baseline = [
                    raw_base[i] if (mask & (1 << i)) else None
                    for i in range(MOTORS)
                ]
                continue

            seg_match = SEG_RE.search(line)
            if seg_match:
                g = seg_match.groupdict()
                seq = int(g["seq"])
                sample = Segment(
                    sequence=seq,
                    name=g["name"],
                    step=int(g["step"]),
                    segment=int(g["seg"]),
                    fine=int(g["fine"]),
                    status=g["status"],
                    complete=int(g["complete"]),
                    bal=int(g["bal"]),
                    x_soft=float(g["x0"]),
                    y_soft=float(g["y0"]),
                    x_next=float(g["x1"]),
                    y_next=float(g["y1"]),
                    d_l=numbers(g["dl"], MOTORS),
                    cmd=numbers(g["cmd"], MOTORS),
                    gain=numbers(g["gain"], MOTORS),
                    enc_after=[None] * MOTORS,
                    enc_mask=0,
                    laser_x=None,
                    laser_y=None,
                    laser_age_ms=None,
                    quality="MISSING_DATA",
                    baseline=list(baseline),
                )
                pending[seq] = sample
                segments.append(sample)
                continue

            data_match = DATA_RE.search(line)
            if data_match:
                g = data_match.groupdict()
                seq = int(g["seq"])
                sample = pending.get(seq)
                if sample is None:
                    continue
                raw_enc = numbers(g["enc"], MOTORS, int)
                mask = int(g["mask"], 16)
                sample.enc_after = [
                    raw_enc[i] if (mask & (1 << i)) else None
                    for i in range(MOTORS)
                ]
                sample.enc_mask = mask
                sample.laser_x = float(g["lx"])
                sample.laser_y = float(g["ly"])
                sample.laser_age_ms = int(g["lage"])
                sample.quality = g["quality"].rstrip("\r")

    return segments


def solve_linear(matrix: Sequence[Sequence[float]], rhs: Sequence[float]) -> List[float]:
    """Small Gaussian-elimination solver with partial pivoting."""
    n = len(rhs)
    a = [list(row) + [rhs[i]] for i, row in enumerate(matrix)]
    for col in range(n):
        pivot = max(range(col, n), key=lambda row: abs(a[row][col]))
        if abs(a[pivot][col]) < 1e-12:
            raise ValueError("singular fit matrix")
        a[col], a[pivot] = a[pivot], a[col]
        scale = a[col][col]
        for j in range(col, n + 1):
            a[col][j] /= scale
        for row in range(n):
            if row == col:
                continue
            factor = a[row][col]
            if factor == 0.0:
                continue
            for j in range(col, n + 1):
                a[row][j] -= factor * a[col][j]
    return [a[i][n] for i in range(n)]


def fit_ridge(features: Sequence[Sequence[float]],
              values: Sequence[float],
              ridge: float) -> Tuple[List[float], float, float]:
    width = len(features[0])
    gram = [[0.0] * width for _ in range(width)]
    vec = [0.0] * width
    for row, value in zip(features, values):
        for i in range(width):
            vec[i] += row[i] * value
            for j in range(width):
                gram[i][j] += row[i] * row[j]
    for i in range(1, width):  # do not penalize the intercept
        gram[i][i] += ridge
    coeff = solve_linear(gram, vec)
    residuals = [
        sum(c * x for c, x in zip(coeff, row)) - value
        for row, value in zip(features, values)
    ]
    rmse = math.sqrt(sum(e * e for e in residuals) / len(residuals))
    max_abs = max(abs(e) for e in residuals)
    return coeff, rmse, max_abs


def feature_row(x: float, y: float, geom_delta: float, degree: int) -> List[float]:
    # Normalization keeps the normal equations well-conditioned while the
    # output coefficients remain directly usable by the predictor.
    u = x / 22.0
    v = y / 22.0
    g = geom_delta / 30.0
    row = [1.0, g, u, v]
    if degree >= 2:
        row.extend((u * u, u * v, v * v))
    return row


def fit_samples(samples: Iterable[Segment], site: Dict[str, float],
                degree: int, ridge: float,
                max_vision_age: int) -> Tuple[dict, List[Segment]]:
    usable: List[Segment] = []
    home = geometry_lengths(0.0, 0.0, site)
    for sample in samples:
        if sample.status != "OK" or sample.complete != 1:
            continue
        if sample.quality != "GOOD":
            continue
        if sample.enc_mask != 0x0F or any(v is None for v in sample.enc_after):
            continue
        if any(v is None for v in sample.baseline):
            continue
        if sample.laser_x is None or sample.laser_age_ms is None:
            continue
        if sample.laser_age_ms > max_vision_age:
            continue
        usable.append(sample)

    if not usable:
        raise ValueError("no complete GOOD samples survived the quality filters")

    # A first field run may contain fewer than seven independent points (or
    # all points may lie on one path).  Fall back to the linear model instead
    # of manufacturing an ill-conditioned quadratic fit.
    requested_degree = degree
    if len(usable) < (7 if degree >= 2 else 4):
        degree = 1

    def fit_all_motors(fit_degree: int) -> List[dict]:
        fitted = []
        for motor in range(MOTORS):
            rows = []
            values = []
            for sample in usable:
                geom = geometry_lengths(sample.laser_x, sample.laser_y, site)
                geom_delta = geom[motor] - home[motor]
                rows.append(feature_row(sample.laser_x, sample.laser_y,
                                         geom_delta, fit_degree))
                base = sample.baseline[motor]
                raw = sample.enc_after[motor]
                values.append(float(raw - base))
            coeff, rmse, max_abs = fit_ridge(rows, values, ridge)
            fitted.append({
                "motor": motor + 1,
                "coefficients": coeff,
                "rmse_counts": rmse,
                "max_abs_error_counts": max_abs,
                "samples": len(values),
            })
        return fitted

    try:
        motors = fit_all_motors(degree)
    except ValueError:
        if degree != 2:
            raise
        # Keep every motor on the same feature order. A per-motor fallback
        # would create incompatible coefficient lengths in one model file.
        degree = 1
        motors = fit_all_motors(degree)

    model = {
        "model_type": "geometry_plus_residual_encoder_counts",
        "units": {"x_y": "cm", "encoder": "0.1deg"},
        "feature_order": (
            ["intercept", "geometry_delta_normalized", "x_normalized",
             "y_normalized"]
            + (["x2_normalized", "xy_normalized", "y2_normalized"]
               if degree >= 2 else [])
        ),
        "requested_degree": requested_degree,
        "degree": degree,
        "ridge": ridge,
        "max_vision_age_ms": max_vision_age,
        "site_file": str(DEFAULT_SITE.name),
        "home_geometry_cm": home,
        "sample_count": len(usable),
        "motors": motors,
        "warning": (
            "Encoder-equivalent targets only. Do not call this free-rope "
            "length or tension until spool radius/winding and slack labels "
            "are independently measured."
        ),
    }
    return model, usable


def predict(model: dict, x: float, y: float, site: Dict[str, float]) -> List[float]:
    home = model["home_geometry_cm"]
    degree = int(model["degree"])
    geom = geometry_lengths(x, y, site)
    out = []
    for motor in model["motors"]:
        i = int(motor["motor"]) - 1
        row = feature_row(x, y, geom[i] - home[i], degree)
        out.append(sum(a * b for a, b in zip(motor["coefficients"], row)))
    return out


def write_c_header(path: Path, model: dict) -> None:
    feature_count = len(model["feature_order"])
    motors = model["motors"]
    lines = [
        "/* Generated by tools/fit_rope_model.py; review sample quality before use. */",
        "#ifndef CDPR_ROPE_MODEL_GENERATED_H",
        "#define CDPR_ROPE_MODEL_GENERATED_H",
        "#include <math.h>",
        "",
        f"#define CDPR_ROPE_MODEL_DEGREE {int(model['degree'])}U",
        f"#define CDPR_ROPE_MODEL_FEATURES {feature_count}U",
        f"#define CDPR_ROPE_MODEL_SAMPLES {int(model['sample_count'])}U",
        "",
        "static const float cdpr_rope_model_coeff[4][CDPR_ROPE_MODEL_FEATURES] = {",
    ]
    for motor in motors:
        coeff = ", ".join(f"{float(v):.9g}f" for v in motor["coefficients"])
        lines.append(f"    {{{coeff}}},")
    lines.extend([
        "};",
        "",
        "/* Caller supplies geometry_delta_cm[4] from the confirmed kinematics. */",
        "static inline void cdpr_rope_model_predict(float x_cm, float y_cm,",
        "                                             const float geometry_delta_cm[4],",
        "                                             float enc_target[4])",
        "{",
        "    const float u = x_cm / 22.0f;",
        "    const float v = y_cm / 22.0f;",
        "    for (unsigned int i = 0U; i < 4U; ++i) {",
        "        float f[CDPR_ROPE_MODEL_FEATURES] = {1.0f,",
        "            geometry_delta_cm[i] / 30.0f, u, v};",
    ])
    if int(model["degree"]) >= 2:
        lines.extend([
            "        f[4] = u * u;",
            "        f[5] = u * v;",
            "        f[6] = v * v;",
        ])
    lines.extend([
        "        enc_target[i] = 0.0f;",
        "        for (unsigned int j = 0U; j < CDPR_ROPE_MODEL_FEATURES; ++j)",
        "            enc_target[i] += cdpr_rope_model_coeff[i][j] * f[j];",
        "    }",
        "}",
        "",
        "#endif",
        "",
    ])
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(lines), encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--log", type=Path, required=True)
    parser.add_argument("--site", type=Path, default=DEFAULT_SITE)
    parser.add_argument("--out", type=Path, default=Path("data/rope_model.json"))
    parser.add_argument("--header", type=Path)
    parser.add_argument("--degree", type=int, choices=(1, 2), default=2)
    parser.add_argument("--ridge", type=float, default=1e-5)
    parser.add_argument("--max-vision-age-ms", type=int, default=500)
    parser.add_argument("--predict", nargs=2, type=float, metavar=("X_CM", "Y_CM"))
    args = parser.parse_args()

    site = parse_site(args.site)
    segments = parse_log(args.log)
    model, usable = fit_samples(
        segments, site, args.degree, args.ridge, args.max_vision_age_ms
    )
    model["site_file"] = str(args.site)
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(model, indent=2) + "\n", encoding="utf-8")
    if args.header:
        write_c_header(args.header, model)

    rejected = len(segments) - len(usable)
    print(f"segments_seen={len(segments)} usable={len(usable)} rejected={rejected}")
    for motor in model["motors"]:
        print(
            f"M{motor['motor']}: samples={motor['samples']} "
            f"rmse={motor['rmse_counts']:.3f} 0.1deg "
            f"max_abs={motor['max_abs_error_counts']:.3f}"
        )
    if args.predict:
        values = predict(model, args.predict[0], args.predict[1], site)
        print(
            "predict encoder-equivalent ["
            + ", ".join(f"{value:.3f}" for value in values)
            + "] 0.1deg"
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
