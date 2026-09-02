#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""balance_tune.py -- reproducible balance experiments for the MuJoCo twin.

P2-T4 evidence + tuning harness (2026-09-02). Runs fully headless (plain
Python + the mujoco wheel, no GUI, no network) and prints one BALANCE_JSON
line with the machine-readable facts after each suite:

    python scripts/blender_humanoid/balance_tune.py --suite stand
    python scripts/blender_humanoid/balance_tune.py --suite gait
    python scripts/blender_humanoid/balance_tune.py --suite all
    python scripts/blender_humanoid/balance_tune.py --suite gait \\
        --scales 0.4,0.5,0.6 --tempos 0.7,0.8 --duration 6

Suites
------
stand  : quiet standing (balance on), upper-body motions under balance
         (idle/wave/nod/look must stay upright), and the perturbation
         envelope sweep -- open loop vs closed loop, four push directions x
         a force ladder. A trial FALLS when root up_z drops below 0.7; a
         RECOVER ends the trial still above 0.95.
gait   : walk trials across a (gait_scale x gait_tempo) grid, open loop
         (scale=tempo=1, balance off) as the baseline. Reports per-trial
         fall time / survived duration / end up_z.

The JSON facts are the honest capability boundary of the closed-loop balance
controller (joint-level proportional control; see balance_controller.py).
"""

import argparse
import json
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)
import physics_adapter as pa  # noqa: E402

TICK = pa.TICK
FALL_UP_Z = 0.7          # trial failure threshold (robot tipping over)
STAND_UP_Z = 0.95        # recovered threshold at trial end
RECOVER_S = 4.0          # recovery window after a perturbation


def up_z(ap):
    return float(ap.data.xmat[ap.model.body("root").id].reshape(3, 3)[2][1])


def fresh(balance_on=True, settle=1.0):
    ap = pa.PhysicsAdapter()
    ap.reset(settle_seconds=settle)
    ap.balance.enabled = balance_on
    return ap


def suite_stand():
    facts = {"quiet": {}, "motions": {}, "pushes": []}

    # -- quiet standing, balance on -----------------------------------------
    ap = fresh()
    umin = 1.0
    for _ in range(int(6.0 / TICK)):
        ap.drive_once()
        umin = min(umin, up_z(ap))
    facts["quiet"] = {"seconds": 6.0, "up_end": round(up_z(ap), 4),
                      "up_min": round(umin, 4),
                      "stand": up_z(ap) > STAND_UP_Z and umin > FALL_UP_Z}
    print("QUIET 6s balance-on: up_end=%.4f up_min=%.4f -> %s"
          % (facts["quiet"]["up_end"], facts["quiet"]["up_min"],
             "STAND" if facts["quiet"]["stand"] else "FALL"))

    # -- upper-body motions under balance -------------------------------------
    for name in ("idle", "wave", "nod", "look"):
        ap = fresh()
        ap.start_motion(name, 3.0)
        umin = 1.0
        for _ in range(int(4.0 / TICK)):
            ap.drive_once()
            umin = min(umin, up_z(ap))
            if umin < FALL_UP_Z:
                break
        ok = umin > FALL_UP_Z and up_z(ap) > STAND_UP_Z
        facts["motions"][name] = {"up_end": round(up_z(ap), 4),
                                  "up_min": round(umin, 4), "stand": ok}
        print("MOTION %-5s balance-on: up_end=%.4f up_min=%.4f -> %s"
              % (name, up_z(ap), umin, "STAND" if ok else "FALL"))

    # -- perturbation envelope: open loop vs closed loop ---------------------
    ladder = {"+X": [(f, 0, 0) for f in (100, 150, 200)],
              "-X": [(-f, 0, 0) for f in (100, 150, 200)],
              "-Y": [(0, -f, 0) for f in (100, 150, 200)],
              "+Y": [(0, f, 0) for f in (60, 80, 100)]}
    for direction, pushes in ladder.items():
        for force in pushes:
            row = {"direction": direction,
                   "force_N": list(force), "duration_s": 0.1}
            for mode in ("open", "closed"):
                ap = fresh(balance_on=(mode == "closed"))
                ap.perturb(force, duration=0.1)
                umin = 1.0
                t_end = None
                for i in range(int(RECOVER_S / TICK)):
                    ap.drive_once()
                    u = up_z(ap)
                    umin = min(umin, u)
                    if u < FALL_UP_Z and t_end is None:
                        t_end = i * TICK
                result = ("fall" if (t_end is not None or
                                     up_z(ap) <= STAND_UP_Z) else "recover")
                row[mode] = {"result": result,
                             "fall_time_s": (round(t_end, 3)
                                             if t_end is not None else None),
                             "up_end": round(up_z(ap), 4)}
            facts["pushes"].append(row)
            print("PUSH %-3s %s N 0.1s: open=%s closed=%s"
                  % (direction, force, row["open"]["result"],
                     row["closed"]["result"]))
    return facts


def gait_trial(scale, tempo, duration, balance_on=True, motion="walk"):
    ap = fresh(balance_on=balance_on)
    ap.start_motion(motion, duration)
    # override AFTER start_motion (which loads the per-gait tuned defaults)
    ap.balance.gait_scale = scale
    ap.balance.gait_tempo = tempo
    umin = 1.0
    fall_t = None
    for i in range(int(duration / TICK)):
        ap.drive_once()
        u = up_z(ap)
        umin = min(umin, u)
        if u < FALL_UP_Z and fall_t is None:
            fall_t = i * TICK
    return {"gait_scale": scale, "gait_tempo": tempo,
            "motion": motion, "balance": balance_on,
            "duration_s": duration,
            "fall_time_s": round(fall_t, 3) if fall_t is not None else None,
            "survived_s": round(fall_t if fall_t is not None else duration, 3),
            "up_end": round(up_z(ap), 4), "up_min": round(umin, 4)}


def gait_default_trial(duration, motion="walk", balance_on=True):
    """Play a gait using the per-gait tuned envelope that start_motion loads
    (no override) -- this is what the server does out of the box."""
    ap = fresh(balance_on=balance_on)
    ap.start_motion(motion, duration)
    umin = 1.0
    fall_t = None
    for i in range(int(duration / TICK)):
        ap.drive_once()
        u = up_z(ap)
        umin = min(umin, u)
        if u < FALL_UP_Z and fall_t is None:
            fall_t = i * TICK
    return {"motion": motion, "balance": balance_on,
            "gait_scale": round(ap.balance.gait_scale, 3),
            "gait_tempo": round(ap.balance.gait_tempo, 3),
            "duration_s": duration,
            "fall_time_s": round(fall_t, 3) if fall_t is not None else None,
            "survived_s": round(fall_t if fall_t is not None else duration, 3),
            "up_end": round(up_z(ap), 4), "up_min": round(umin, 4)}


def suite_gait(scales, tempos, duration):
    facts = {"gait_duration_s": duration, "baseline": {}, "defaults": [],
             "trials": []}
    # open-loop baseline: faithful contract gait, no balance
    facts["baseline"] = gait_trial(1.0, 1.0, duration, balance_on=False)
    b = facts["baseline"]
    print("GAIT open-loop  scale=1.00 tempo=1.00: fall_t=%s up_end=%.3f"
          % (b["fall_time_s"], b["up_end"]))
    # tuned out-of-the-box gaits (per-gait envelope loaded by start_motion)
    for motion in ("walk", "run"):
        d = gait_default_trial(duration, motion)
        facts["defaults"].append(d)
        print("GAIT tuned %-4s scale=%.2f tempo=%.2f: fall_t=%s up_end=%.3f "
              "up_min=%.3f" % (motion, d["gait_scale"], d["gait_tempo"],
                               d["fall_time_s"], d["up_end"], d["up_min"]))
    for s in scales:
        for t in tempos:
            r = gait_trial(s, t, duration, balance_on=True)
            facts["trials"].append(r)
            print("GAIT closed-loop scale=%.2f tempo=%.2f: fall_t=%s "
                  "up_end=%.3f up_min=%.3f"
                  % (s, t, r["fall_time_s"], r["up_end"], r["up_min"]))
    return facts


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--suite", choices=("stand", "gait", "all"),
                   default="all")
    p.add_argument("--duration", type=float, default=6.0,
                   help="gait trial duration in seconds")
    p.add_argument("--scales", default="0.4,0.5,0.6,0.7",
                   help="comma-separated gait_scale ladder")
    p.add_argument("--tempos", default="0.6,0.8,1.0",
                   help="comma-separated gait_tempo ladder")
    args = p.parse_args(argv)

    facts = {}
    if args.suite in ("stand", "all"):
        facts["stand"] = suite_stand()
    if args.suite in ("gait", "all"):
        scales = [float(v) for v in args.scales.split(",") if v.strip()]
        tempos = [float(v) for v in args.tempos.split(",") if v.strip()]
        facts["gait"] = suite_gait(scales, tempos, args.duration)
    print("BALANCE_JSON " + json.dumps(facts))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
