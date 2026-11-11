#!/usr/bin/env python3
"""Fits what hangs beyond the follower's force/torque sensor (the gripper) from a recording,
for `follower --ft wrist|arm|all --ft-tool FILE` (lib/ft_core.h).

  ft_tool_cal.py RECORDING_follower.tlog [-o ft_tool.conf] [--origin-z METRES] [--j7-window RAD]

The recording: `follower --ft-cal --record ...` (`wam ft-cal`: the arm turns the gripper through
its poses by itself), or `follower --ft log --record ...` (any teleop mode) with NOTHING touching
the gripper, while the wrist is turned through its range, pausing a second or two in each pose
(about a minute in all). The gripper should be as it will be used (a BarrettHand: in its fist).
Only the moments at rest are used.

--j7-window RAD fits only the moments with J7 within RAD of the angle it rests at most (the
teleop's sync angle in a `wam ft-cal` run). Whatever crosses J7 to the gripper or the sensor (a
cable) loads the sensor as J7 turns, which is no part of the gripper's weight: on slax, 2026-11-11,
5 N with J7 one radian beyond the sync angle. The other moments at rest are then compared with
the fit, which shows that load. After the fit a table lists every pause (pose, what the fit
leaves unexplained there), so that a bad fit shows which poses do not agree.

What is fitted, from how the readings change with the direction of gravity in the tool frame:
  mass and centre of mass of the gripper;
  the sensor's axes relative to libbarrett's tool frame (rot_*), so a sensor mounted turned
    about the tool axis is handled;
  its sign convention (whether it reports the load's wrench on the arm, or the arm's on the load);
  the reading's offsets (not written: the follower takes its own zero at the link pose).
Not fitted (free space cannot show it): how far the sensor's measuring origin is from the tool
plate along the tool axis. --origin-z sets it if the sensor's drawing gives it; 0 otherwise
(a contact's wrist torques are then off by that distance times the contact force).

Only numpy is needed.
"""
import argparse
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from teleop_log import load  # noqa: E402

GRAVITY = 9.81  # as lib/ft_core.h


def skew(v):
    return np.array([[0.0, -v[2], v[1]], [v[2], 0.0, -v[0]], [-v[1], v[0], 0.0]])


def fit(g, f, n):
    """g: gravity's direction in the tool frame; f, n: the readings (sensor axes), all (N, 3).
    Returns a dict. Model, with Q = rot^T (tool axes -> sensor axes):
      f = sign m G Q g + bf          n = sign m G Q (com x g) + bn"""
    N = len(g)
    # the sign and how even the readings are, from the unconstrained f = A g + b
    X = np.hstack([g, np.ones((N, 1))])
    coef, *_ = np.linalg.lstsq(X, f, rcond=None)
    A = coef[:3].T
    sign = 1.0 if np.linalg.det(A) > 0 else -1.0
    S = np.linalg.svd(A, compute_uv=False)
    # the least-squares mass, rotation and offset together (a scaled rotation fitted to the
    # centred readings; the offset follows from the means)
    gm, fm = g.mean(axis=0), f.mean(axis=0)
    H = (sign * (f - fm)).T @ (g - gm)
    U, _, Vt = np.linalg.svd(H)
    D = np.diag([1.0, 1.0, np.linalg.det(U @ Vt)])
    Q = U @ D @ Vt
    mass = np.trace(Q.T @ H) / ((g - gm) ** 2).sum() / GRAVITY
    bf = fm - sign * mass * GRAVITY * Q @ gm
    # n = -sign m G Q [g]x com + bn, linear in com and bn
    M = np.zeros((3 * N, 6))
    for i in range(N):
        M[3 * i:3 * i + 3, :3] = -sign * mass * GRAVITY * Q @ skew(g[i])
        M[3 * i:3 * i + 3, 3:] = np.eye(3)
    sol, *_ = np.linalg.lstsq(M, n.reshape(-1), rcond=None)
    com, bn = sol[:3], sol[3:]
    f_model = sign * mass * GRAVITY * (Q @ g.T).T + bf
    n_model = (M @ sol).reshape(-1, 3)
    return dict(mass=mass, sign=sign, rot=Q.T, com=com, bf=bf, bn=bn, singular=S,
                f_res=f - f_model, n_res=n - n_model)


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('recording')
    p.add_argument('-o', '--output', default='ft_tool.conf')
    p.add_argument('--origin-z', type=float, default=0.0,
                   help='the sensor\'s measuring origin beyond the tool plate, m (default 0)')
    p.add_argument('--max-speed', type=float, default=0.03,
                   help='at rest: every joint slower than this on average over 0.2 s, rad/s (default 0.03)')
    p.add_argument('--j7-window', type=float, default=None,
                   help='fit only where J7 is within this of the angle it rests at most, rad (default: everywhere)')
    a = p.parse_args()

    log = load(a.recording)
    for name in ('ft_raw_f', 'ft_raw_n', 'ft_g', 'dq', 'q'):
        if name not in log:
            sys.exit('%s has no %s columns: record with `follower --ft log --record`' % (a.recording, name))
    f, n, g, dq, q = log['ft_raw_f'], log['ft_raw_n'], log['ft_g'], log['dq'], log['q']
    t = log['t'] - log['t'][0]
    fresh = log['ft_new'] > 0 if 'ft_new' in log else np.ones(len(f), bool)
    ok = fresh & np.isfinite(f).all(axis=1) & np.isfinite(g).all(axis=1)
    if ok.sum() < 500:
        sys.exit('only %d cycles with a reading: was the follower started with --ft?' % ok.sum())
    # at rest: the fastest joint's speed, averaged over the 0.2 s around each cycle (an encoder
    # count flickering at rest is a spike in the velocity, not motion), slow now and for the
    # 0.2 s before (the gripper stops swinging)
    speed = np.convolve(np.abs(dq).max(axis=1), np.ones(100) / 100.0, mode='same')
    slow = speed < a.max_speed
    settled = np.convolve(slow.astype(float), np.ones(100), mode='full')[:len(slow)] >= 100
    use = ok & settled
    print('%s: %d cycles, %d with a reading, %d of those at rest' % (a.recording, len(f), ok.sum(), use.sum()))
    if use.sum() < 500:
        sys.exit('too little of the recording is at rest: pause a second or two in each pose')

    # --j7-window: only where J7 is near the angle it rests at most
    fitted = use.copy()
    j7_centre = None
    if a.j7_window is not None and q.shape[1] >= 7:
        bins = np.round(q[use, 6] / 0.05).astype(int)
        values, counts = np.unique(bins, return_counts=True)
        near = np.abs(q[:, 6] - 0.05 * values[counts.argmax()]) < 0.1
        j7_centre = float(np.median(q[use & near, 6]))
        fitted = use & (np.abs(q[:, 6] - j7_centre) < a.j7_window)
        print('  fitting where J7 is within %.2f rad of %.2f rad: %d cycles' % (a.j7_window, j7_centre, fitted.sum()))
        if fitted.sum() < 500:
            sys.exit('too little of the recording is at rest there')
    gf, ff, nf = g[fitted], f[fitted], n[fitted]

    # how well the poses cover the directions of gravity: the weakest direction's spread
    spread = np.linalg.svd(gf - gf.mean(axis=0), compute_uv=False) / np.sqrt(len(gf))
    if spread.min() < 0.15:
        print('  gravity directions covered: spread %.2f, %.2f, %.2f (1/sqrt(3) = 0.58 each is ideal)' % tuple(spread))
        print('NOT WRITTEN: the poses do not cover the directions of gravity (weakest spread %.2f, need 0.15):\n'
              '  turn the wrist further and tilt the forearm, pausing in each pose' % spread.min())
        return 1

    # two passes: fit, drop what does not fit (something touched the gripper), fit again
    r = fit(gf, ff, nf)
    for _ in range(2):
        fe, ne = np.linalg.norm(r['f_res'], axis=1), np.linalg.norm(r['n_res'], axis=1)
        keep = (fe < 3.0 * max(np.sqrt((fe ** 2).mean()), 0.05)) & (ne < 3.0 * max(np.sqrt((ne ** 2).mean()), 0.002))
        gf, ff, nf = gf[keep], ff[keep], nf[keep]
        r = fit(gf, ff, nf)
    dropped = fitted.sum() - len(gf)

    spread = np.linalg.svd(gf - gf.mean(axis=0), compute_uv=False) / np.sqrt(len(gf))  # of what was kept
    f_rms, n_rms = np.sqrt((r['f_res'] ** 2).sum(axis=1).mean()), np.sqrt((r['n_res'] ** 2).sum(axis=1).mean())
    uneven = r['singular'].max() / r['singular'].min() - 1.0
    R = r['rot']
    Q = R.T
    print('  used %d cycles (%d dropped as not fitting)' % (len(gf), dropped))
    print('  gravity directions covered: spread %.2f, %.2f, %.2f (1/sqrt(3) = 0.58 each is ideal)' % tuple(spread))
    print('  mass %.3f kg; centre of mass %.1f, %.1f, %.1f mm from the sensor\'s origin (tool axes)'
          % (r['mass'], *(1e3 * r['com'])))
    print('  sign %+d (%s)' % (r['sign'], 'the reading is the load\'s wrench on the arm' if r['sign'] > 0
                              else 'the reading is the arm\'s wrench on the load'))
    print('  sensor axes in the tool frame (columns: the sensor\'s x, y, z):')
    for row in R:
        print('    %8.4f %8.4f %8.4f' % tuple(row))
    tilt = np.degrees(np.arccos(np.clip(R[2, 2], -1.0, 1.0)))
    print('    = turned %.1f degrees about the tool axis, its z %.1f degrees off the tool\'s'
          % (np.degrees(np.arctan2(R[1, 0], R[0, 0])), tilt))
    print('  what is left unexplained: %.3f N, %.4f Nm rms; reading offsets %.2f, %.2f, %.2f N'
          % (f_rms, n_rms, *r['bf']))

    # every pause against the fit: what the model leaves of the mean reading there
    scale = r['sign'] * r['mass'] * GRAVITY
    f_model = scale * (Q @ g.T).T + r['bf']
    n_model = scale * (Q @ np.cross(r['com'], g).T).T + r['bn']
    pauses = []
    idx = np.flatnonzero(use)
    start = idx[0]
    for i, j in zip(idx, np.r_[idx[1:], -1]):
        ends = j < 0 or j - i > 50 or (q.shape[1] >= 7 and np.abs(q[j, 4:7] - q[start, 4:7]).max() > 0.05)
        if ends:
            sel = np.zeros(len(use), bool)
            sel[start:i + 1] = use[start:i + 1]
            if sel.sum() >= 200:
                pauses.append(dict(t=t[start], seconds=0.002 * sel.sum(), q=q[sel].mean(axis=0),
                                   f=np.linalg.norm((f[sel] - f_model[sel]).mean(axis=0)),
                                   n=np.linalg.norm((n[sel] - n_model[sel]).mean(axis=0)),
                                   fitted=fitted[sel].mean() > 0.5))
            start = j
    if q.shape[1] >= 7 and pauses:
        print('  every pause against that fit (the mean reading there minus the model):')
        print('     at     for     J5     J6     J7    force    torque')
        for k, h in enumerate(pauses[:60]):
            print('  %6.1f s %4.1f s  %5.2f  %5.2f  %5.2f  %5.2f N  %6.3f Nm%s'
                  % (h['t'], h['seconds'], h['q'][4], h['q'][5], h['q'][6], h['f'], h['n'], '' if h['fitted'] else '   (not fitted)'))
        if len(pauses) > 60:
            print('  ... and %d more' % (len(pauses) - 60))
    worst = max([h['f'] for h in pauses if h['fitted']] or [0.0])
    outside = [h for h in pauses if not h['fitted']]

    problems, notes = [], []
    if spread.min() < 0.15:
        problems.append('the poses do not cover the directions of gravity (weakest spread %.2f, need 0.15): '
                        'turn the wrist further and tilt the forearm' % spread.min())
    if not 0.05 < r['mass'] < 10.0:
        problems.append('a mass of %.2f kg is not a gripper' % r['mass'])
    if np.abs(r['com']).max() > 0.3:
        problems.append('a centre of mass %.2f m away is not a gripper' % np.abs(r['com']).max())
    if f_rms > 1.0 or n_rms > 0.08:
        problems.append('the fit leaves %.2f N, %.3f Nm unexplained (the limit is 1 N, 0.08 Nm): see the table for the poses that do not agree'
                        % (f_rms, n_rms))
    if worst > 2.0:
        problems.append('one pause is %.1f N off the fit (the limit is 2 N): something touched the gripper or pulled on it there' % worst)
    if uneven > 0.6:
        problems.append('the force readings do not look like one mass turning in gravity (%.0f %% uneven): '
                        'something touched the gripper, or the tool frame is not what the follower thinks' % (100 * uneven))
    elif uneven > 0.10:
        notes.append('the sensor reads the same weight %.0f %% larger in one direction than in another (a rigid gripper on an exact '
                     'sensor: 0). The fit takes the mean; that difference is in what is left unexplained.' % (100 * uneven))
    if tilt > 25.0:
        problems.append('the sensor\'s z axis is %.0f degrees off the tool axis: check how it is mounted' % tilt)
    elif tilt > 5.0:
        notes.append('the sensor\'s z axis comes out %.0f degrees off the follower\'s tool axis. A sensor bolted to the tool plate cannot '
                     'be tilted: more likely the follower\'s J6 or J5 zero is off by about that (an arm zeroed by its home pose only '
                     'has the wrist zero where the wrist sat at start-up), or the sensor mixes its axes. The file keeps the axes as '
                     'fitted: that is what removes the gripper\'s weight best.' % tilt)
    if outside and max(h['f'] for h in outside) > 1.5:
        h = max(outside, key=lambda x: x['f'])
        notes.append('with J7 away from %.2f rad the reading is up to %.1f N off this fit (at J7 = %.2f rad): that is not the '
                     'gripper\'s weight. Something that crosses J7 (a cable to the gripper or the sensor) pulls on the sensor as '
                     'J7 turns; the teleop will feel it as a contact force there.' % (j7_centre, h['f'], h['q'][6]))
    for x in notes:
        print('NOTE: ' + x)
    if problems:
        print('NOT WRITTEN:')
        for x in problems:
            print('  - ' + x)
        return 1

    with open(a.output, 'w') as out:
        out.write('# The gripper beyond the follower\'s force/torque sensor (follower --ft-tool; lib/ft_core.h).\n')
        out.write('# Fitted by scripts/ft_tool_cal.py from %s\n' % os.path.basename(a.recording))
        out.write('# %d cycles at rest%s; left unexplained %.3f N, %.4f Nm rms (worst pause %.2f N); gravity spread %.2f %.2f %.2f\n'
                  % (len(gf), '' if j7_centre is None else ' with J7 within %.2f rad of %.2f' % (a.j7_window, j7_centre),
                     f_rms, n_rms, worst, *spread))
        out.write('mass = %.4f      # kg\n' % r['mass'])
        for i, axis in enumerate('xyz'):
            out.write('com_%s = %.5f   # m, from the sensor\'s origin, tool axes\n' % (axis, r['com'][i]))
        out.write('sign = %d\n' % r['sign'])
        out.write('# the sensor\'s axes in the tool frame: v_tool = rot v_sensor\n')
        for i, row in enumerate('xyz'):
            for j, col in enumerate('xyz'):
                out.write('rot_%s%s = %.6f\n' % (row, col, R[i, j]))
        out.write('# the sensor\'s measuring origin in the tool frame, m (not fitted: --origin-z)\n')
        out.write('origin_x = 0\norigin_y = 0\norigin_z = %.4f\n' % a.origin_z)
    print('wrote', a.output)
    return 0


if __name__ == '__main__':
    sys.exit(main())
