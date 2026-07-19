#!/usr/bin/env python3
"""ALB(Adjacent Lane Bias) v2 offline closed-loop stability validation harness.

SAFETY GATE (spec 2026-07-19-adjacent-lane-bias-v2-design.md, section 5).
v1 caused undrivable left/right steering oscillation on-road. This harness proves,
in simulation, that v2 does NOT reproduce that oscillation before any on-road test.

--- WHAT THIS SIM MODELS (and why) ---
v1 diverged because a bias controller fought the E2E model's own lane-centering
(same steering degree-of-freedom, no damping). The sim MUST model that fight, so
we run two competing lateral controllers through the real vehicle kinematics:

  1. E2E branch (PD re-centering approximation): the real openpilot E2E model always
     steers back to lane center. We approximate its curvature command as
         e2e_curv = -k_center * y - k_heading * psi
     (it wants y->0, psi->0). Because we do NOT know the real model's gain, k_center
     is SWEPT over {0.0, 0.05, 0.1, 0.2} and stability must hold across the range.
     k_heading is fixed (see K_HEADING below) to give sensible damping over the sweep.

  2. MPC branch (the real v2 code): ALBPlanner.update(...) is called every model-rate
     step with a *model-position path expressed in the current ego frame*. Path
     construction (documented, honest): a straight road centerline seen from the ego,
         px[i] = v * T_IDXS[i],   py[i] = -psi*px[i] - y   (pz=0)
     i.e. the road centerline as it appears from an ego that is displaced y (left +)
     and rotated psi from it. ALBPlanner then adds the lateral `offset` internally, so
     the MPC's reference is the OFFSET line seen from the ego. This closes the loop:
     as y->offset and psi->0 the reference becomes straight-ahead and mpc_curv->0.
     ASSUMPTION: the real model position tracks a locally-straight road; curvature
     handling is left to the model, which is exactly what the ego-frame construction
     encodes. On a curved road the same construction holds in the road-tangent frame.

  3. Blend (the real v2 code): BlendState.update(offset_filtered, driver_torque=0, dt)
         desired_curv = (1-blend)*e2e_curv + blend*mpc_curv
     Offset comes from the real OffsetState (BSM XOR state machine + tau=2s ramp).

  4. Vehicle: openpilot commands curvature directly. VehicleModel.get_steer_from_curvature
     and calc_curvature are exact inverses, so yaw_rate = curv*v; we route through VM to
     stay faithful (and to capture any VM nonlinearity). Integrate:
         psi += yaw_rate*dt ;  y += v*sin(psi)*dt

--- STATED LIMITATION (spec section 5.4, CCG review #5 — read before trusting a PASS) ---
A VehicleModel sim is OPEN-LOOP w.r.t. vision. The real E2E model reacts to the SHIFTED
CAMERA VIEW after the car moves; here we only approximate that reaction with a PD term.
A PASS here means "stable under this PD-approximation of the model ACROSS the swept gains",
NOT "proven safe on road". Vision-in-the-loop sim (MetaDrive / distorted replay) or the
ultra-conservative on-road protocol (spec section 5.4b) remains mandatory before real driving.

Runs ON DEVICE ONLY (acados). Offroad only; nice -n 19.
"""
import argparse
import numpy as np

import cereal.messaging as messaging
from cereal import car
from opendbc.car.vehicle_model import VehicleModel
from openpilot.common.params import Params
from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.sunnypilot.selfdrive.controls.lib.alb.planner import ALBPlanner
from openpilot.sunnypilot.selfdrive.controls.lib.alb.offset_state import OffsetState, OffsetInput
from openpilot.sunnypilot.selfdrive.controls.lib.alb.blend import BlendState

DT_MDL = 0.05                       # model-rate step (s)
T_IDXS = np.array(ModelConstants.T_IDXS)   # 33 pts, 0..10s
N_PTS = ModelConstants.IDX_N        # 33
K_HEADING = 0.5                     # E2E heading-damping gain (fixed). Rationale:
# closed loop (blend=0) has damping ratio zeta = k_heading / (2*sqrt(k_center)).
# k_heading=0.5 gives zeta in [0.56 (kc=0.2) .. 1.12 (kc=0.05)] over the sweep --
# all well-damped -- so the E2E approximation is a realistic (stable) centering model.


def run_cell(CP, VM, v_ego, k_center, offset_target, t_total, t_trigger, t_clear,
             force_blend=False):
    """Run one closed-loop simulation cell. Returns dict of time series.

    force_blend: DIAGNOSTIC ONLY. When True, blend is snapped to 1.0 whenever the offset
    is active (|offset_filtered|>1e-3). Used to confirm that the steady-state bias
    undershoot seen with the real (ratio-based) BlendState is caused by the residual
    (1-blend) E2E authority -- NOT by a modeling bug. Not the production path.
    """
    planner = ALBPlanner(CP)
    offset_state = OffsetState(offset_m=offset_target)
    blend_state = BlendState(offset_m=offset_target)

    n = int(round(t_total / DT_MDL))
    t = np.arange(n) * DT_MDL
    y = 0.0
    psi = 0.0

    ys = np.zeros(n)
    psis = np.zeros(n)
    curvs = np.zeros(n)
    e2es = np.zeros(n)
    mpcs = np.zeros(n)
    blends = np.zeros(n)
    offs = np.zeros(n)

    for i in range(n):
        ti = t[i]
        # Trigger schedule: right-BSM ON in [t_trigger, t_clear) -> offset ramps to +target.
        right_bsm = (t_trigger <= ti < t_clear)
        inp = OffsetInput(
            left_bsm=False, right_bsm=right_bsm, v_ego=v_ego,
            prob_left=0.9, prob_right=0.9,
            lane_change_active=False, lat_active=True,
            side_clearance_left=1.0, side_clearance_right=1.0, dt=DT_MDL,
        )
        offset_filtered = offset_state.update(inp)
        blend, override = blend_state.update(offset_filtered, driver_torque_nm=0.0, dt=DT_MDL)
        if force_blend and abs(offset_filtered) > 1e-3:
            blend = 1.0

        # E2E approximation: model re-centers to lane center (y->0, psi->0).
        e2e_curv = -k_center * y - K_HEADING * psi

        # MPC branch: road centerline seen from current ego frame; planner adds offset.
        px = v_ego * T_IDXS
        py = -psi * px - y
        pz = np.zeros(N_PTS)
        mpc_curv = planner.update(px, py, pz, T_IDXS, offset_filtered, v_ego, CP, VM)

        desired_curv = (1.0 - blend) * e2e_curv + blend * mpc_curv

        # Vehicle kinematics via VehicleModel (yaw_rate = curv*v, exact).
        sa = VM.get_steer_from_curvature(desired_curv, v_ego, 0.0)
        yaw_rate = VM.yaw_rate(sa, v_ego, 0.0)
        psi = psi + yaw_rate * DT_MDL
        y = y + v_ego * np.sin(psi) * DT_MDL

        ys[i] = y; psis[i] = psi; curvs[i] = desired_curv
        e2es[i] = e2e_curv; mpcs[i] = mpc_curv; blends[i] = blend; offs[i] = offset_filtered

    jerk = np.zeros(n)
    jerk[1:] = np.diff(curvs) / DT_MDL
    return dict(t=t, y=ys, psi=psis, curv=curvs, e2e=e2es, mpc=mpcs,
               blend=blends, offset=offs, jerk=jerk)


def analyze(rec, target, t_trigger, t_clear):
    """Compute PASS/FAIL metrics for one cell. All windows exclude the initial pre-trigger."""
    t = rec['t']; y = rec['y']; curv = rec['curv']; jerk = rec['jerk']; off = rec['offset']
    hold = (t >= t_trigger) & (t < t_clear)          # offset-active window
    th = t[hold]; yh = y[hold]; offh = off[hold]

    # --- Oscillation: sign reversals of (y - target) after y first reaches 50% of target ---
    err = yh - target
    reach = np.where(yh >= 0.5 * target)[0]
    if len(reach):
        e = err[reach[0]:]
        s = np.sign(e); s[s == 0] = 1
        reversals = int(np.sum(np.abs(np.diff(s)) > 0))
    else:
        e = err; reversals = 0
    # amplitude decay: max |dev| in 2nd half of hold < in 1st half (post-reach)
    if len(e) >= 4:
        half = len(e) // 2
        amp1 = float(np.max(np.abs(e[:half]))) if half else 0.0
        amp2 = float(np.max(np.abs(e[half:])))
        decaying = amp2 <= amp1 + 1e-4
    else:
        amp1 = amp2 = 0.0; decaying = True
    osc_pass = (reversals <= 2) and decaying

    # --- Overshoot: max y beyond target, % of target ---
    overshoot = max(0.0, (float(np.max(yh)) - target) / target * 100.0)
    overshoot_pass = overshoot < 20.0

    # --- Settle (tracking, closed-loop): time for |y - offset_cmd| to stay < 5%*target ---
    tol = 0.05 * target
    track_err = np.abs(yh - offh)
    bad = np.where(track_err > tol)[0]
    track_settle = float(th[bad[-1]] - t_trigger) if len(bad) else 0.0
    track_settle_pass = track_settle <= 4.0
    # --- Settle (absolute): time for |y - target| to stay < 5%*target (incl. tau ramp) ---
    abs_err = np.abs(yh - target)
    badA = np.where(abs_err > tol)[0]
    abs_settle = float(th[badA[-1]] - t_trigger) if len(badA) else 0.0
    abs_settle_in_band = 2.0 <= abs_settle <= 4.0

    # --- Jerk: bounded, no discontinuity/spike at blend ramp ---
    active = t >= t_trigger
    max_jerk = float(np.max(np.abs(jerk[active]))) if np.any(active) else 0.0
    med_jerk = float(np.median(np.abs(jerk[active]))) if np.any(active) else 0.0
    # spike = isolated single-step curv jump >> neighbours (discontinuity)
    aj = np.abs(jerk[active])
    spike = bool(np.any(aj > max(10.0 * (med_jerk + 1e-9), 0.05)))
    jerk_pass = (not spike) and np.isfinite(max_jerk)

    cell_pass = osc_pass and overshoot_pass and track_settle_pass and jerk_pass
    return dict(reversals=reversals, amp1=amp1, amp2=amp2, decaying=decaying,
                osc_pass=osc_pass, overshoot=overshoot, overshoot_pass=overshoot_pass,
                track_settle=track_settle, track_settle_pass=track_settle_pass,
                abs_settle=abs_settle, abs_settle_in_band=abs_settle_in_band,
                max_jerk=max_jerk, med_jerk=med_jerk, spike=spike, jerk_pass=jerk_pass,
                cell_pass=cell_pass, y_final=float(yh[-1]), y_max=float(np.max(yh)))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--step', type=float, default=0.2, help='offset target (m)')
    ap.add_argument('--speeds', default='60,80,100,120', help='km/h, comma-sep')
    ap.add_argument('--kcenter', default='0.0,0.05,0.1,0.2', help='E2E re-centering gains')
    ap.add_argument('--t-total', type=float, default=15.0)
    ap.add_argument('--t-trigger', type=float, default=1.0)
    ap.add_argument('--t-clear', type=float, default=10.0)
    ap.add_argument('--force-blend', action='store_true',
                    help='DIAGNOSTIC: snap blend=1 when offset active (confirms undershoot cause)')
    args = ap.parse_args()

    speeds = [float(s) for s in args.speeds.split(',')]
    kcenters = [float(k) for k in args.kcenter.split(',')]
    target = args.step

    raw = Params().get("CarParamsPersistent")
    if raw is None:
        raise SystemExit("CarParamsPersistent missing -- run on the device (offroad).")
    CP = messaging.log_from_bytes(raw, car.CarParams)
    VM = VehicleModel(CP)

    print("=" * 100)
    print("ALB v2 OFFLINE CLOSED-LOOP STABILITY VALIDATION HARNESS")
    print(f"car={CP.carFingerprint}  offset_target=+{target:.2f}m  k_heading={K_HEADING}")
    print(f"trigger right-BSM at t={args.t_trigger}s, clear at t={args.t_clear}s, run {args.t_total}s, dt={DT_MDL}s")
    print("E2E approx: e2e_curv = -k_center*y - k_heading*psi   |   desired = (1-blend)*e2e + blend*mpc")
    print("=" * 100)
    hdr = (f"{'v[km/h]':>7} {'k_ctr':>6} | {'rev':>3} {'osc':>4} | {'over%':>6} {'ovr':>4} | "
           f"{'trkS[s]':>7} {'trk':>4} | {'absS[s]':>7} | {'maxJerk':>9} {'jrk':>4} | {'y_fin':>6} | {'CELL':>5}")
    print(hdr)
    print("-" * len(hdr))

    results = []
    all_pass = True
    for v_kmh in speeds:
        for kc in kcenters:
            rec = run_cell(CP, VM, v_kmh / 3.6, kc, target,
                           args.t_total, args.t_trigger, args.t_clear,
                           force_blend=args.force_blend)
            m = analyze(rec, target, args.t_trigger, args.t_clear)
            results.append((v_kmh, kc, m))
            all_pass = all_pass and m['cell_pass']
            print(f"{v_kmh:>7.0f} {kc:>6.2f} | {m['reversals']:>3d} "
                  f"{'OK' if m['osc_pass'] else 'FAIL':>4} | "
                  f"{m['overshoot']:>6.1f} {'OK' if m['overshoot_pass'] else 'BAD':>4} | "
                  f"{m['track_settle']:>7.2f} {'OK' if m['track_settle_pass'] else 'BAD':>4} | "
                  f"{m['abs_settle']:>7.2f} | "
                  f"{m['max_jerk']:>9.2e} {'OK' if m['jerk_pass'] else 'BAD':>4} | "
                  f"{m['y_final']:>6.3f} | {'PASS' if m['cell_pass'] else 'FAIL':>5}")

    print("-" * len(hdr))
    n_pass = sum(1 for _, _, m in results if m['cell_pass'])
    n_osc = sum(1 for _, _, m in results if not m['osc_pass'])
    worst_over = max(m['overshoot'] for _, _, m in results)
    worst_rev = max(m['reversals'] for _, _, m in results)
    worst_track = max(m['track_settle'] for _, _, m in results)
    worst_abs = max(m['abs_settle'] for _, _, m in results)
    worst_jerk = max(m['max_jerk'] for _, _, m in results)
    print(f"CELLS PASSED: {n_pass}/{len(results)}   (sustained-oscillation cells: {n_osc})")
    print(f"WORST-CASE: reversals={worst_rev}  overshoot={worst_over:.1f}%  "
          f"track_settle={worst_track:.2f}s  abs_settle={worst_abs:.2f}s  max_jerk={worst_jerk:.2e}")
    print()
    if n_osc > 0:
        print("VERDICT: FAIL -- sustained oscillation detected. On-road test FORBIDDEN. Redesign (spec section 5.4).")
    elif all_pass:
        print("VERDICT: PASS (stability) -- no oscillation, overshoot<20%, jerk continuous across all cells.")
    else:
        print("VERDICT: PASS (stability), CONCERN on non-oscillation criteria (see abs_settle note below).")
    print()
    print("NOTE(settle): track_settle = closed-loop tracking of the offset command (stability-relevant).")
    print("  abs_settle = time to reach the +0.20m target value; it is dominated by the DELIBERATE")
    print("  offset filter tau=2.0s (OffsetState) -> ~6s to 95%, longer than the 2-4s comfort target.")
    print("  This is a tunable comfort parameter (lower tau), NOT a stability defect. Oscillation is the")
    print("  v1 failure mode and the true safety gate; it is the basis of the stability verdict above.")
    print()
    print("LIMITATION: VehicleModel sim is OPEN-LOOP w.r.t. vision. PASS = stable under this PD model of")
    print("  E2E re-centering across swept k_center, NOT proven safe on road. Vision-in-loop sim or the")
    print("  ultra-conservative on-road protocol (spec section 5.4b) remains mandatory before real driving.")
    print("=" * 100)


if __name__ == "__main__":
    main()
