#!/usr/bin/env python3
"""Fits what hangs beyond the follower's force/torque sensor (the gripper) from a recording,
for `follower --ft wrist|arm|all --ft-tool FILE` (lib/ft_core.h).

  ft_tool_cal.py RECORDING_follower.tlog [-o ft_tool.conf] [--origin-z METRES]

The recording: `follower --ft log --record ...` (any teleop mode), with NOTHING touching the
gripper, while the wrist is turned through its range and the arm moved between a few different
poses, pausing a second or two in each (about a minute in all). The gripper should be as it
will be used (a BarrettHand: in its fist). Only the moments at rest are used.

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
    X = np.hstack([g, np.ones((N, 1))])
    coef, *_ = np.linalg.lstsq(X, f, rcond=None)
    A, bf = coef[:3].T, coef[3]
    sign = 1.0 if np.linalg.det(A) > 0 else -1.0
    U, S, Vt = np.linalg.svd(sign * A)
    D = np.diag([1.0, 1.0, np.linalg.det(U @ Vt)])
    Q = U @ D @ Vt  # the rotation nearest to sign A / (m G)
    mass = np.trace(Q.T @ (sign * A)) / (3.0 * GRAVITY)
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
    a = p.parse_args()

    log = load(a.recording)
    for name in ('ft_raw_f', 'ft_raw_n', 'ft_g', 'dq'):
        if name not in log:
            sys.exit('%s has no %s columns: record with `follower --ft log --record`' % (a.recording, name))
    f, n, g, dq = log['ft_raw_f'], log['ft_raw_n'], log['ft_g'], log['dq']
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
    g, f, n = g[use], f[use], n[use]

    # how well the poses cover the directions of gravity: the weakest direction's spread
    spread = np.linalg.svd(g - g.mean(axis=0), compute_uv=False) / np.sqrt(len(g))
    if spread.min() < 0.15:
        print('  gravity directions covered: spread %.2f, %.2f, %.2f (1/sqrt(3) = 0.58 each is ideal)' % tuple(spread))
        print('NOT WRITTEN: the poses do not cover the directions of gravity (weakest spread %.2f, need 0.15):\n'
              '  turn the wrist further and tilt the forearm, pausing in each pose' % spread.min())
        return 1

    # two passes: fit, drop what does not fit (something touched the gripper), fit again
    r = fit(g, f, n)
    for _ in range(2):
        fe, ne = np.linalg.norm(r['f_res'], axis=1), np.linalg.norm(r['n_res'], axis=1)
        keep = (fe < 3.0 * max(np.sqrt((fe ** 2).mean()), 0.05)) & (ne < 3.0 * max(np.sqrt((ne ** 2).mean()), 0.002))
        g, f, n = g[keep], f[keep], n[keep]
        r = fit(g, f, n)
    dropped = use.sum() - len(g)

    spread = np.linalg.svd(g - g.mean(axis=0), compute_uv=False) / np.sqrt(len(g))  # of what was kept
    f_rms, n_rms = np.sqrt((r['f_res'] ** 2).sum(axis=1).mean()), np.sqrt((r['n_res'] ** 2).sum(axis=1).mean())
    uneven = r['singular'].max() / r['singular'].min() - 1.0
    R = r['rot']
    print('  used %d cycles (%d dropped as not fitting)' % (len(g), dropped))
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

    problems = []
    if spread.min() < 0.15:
        problems.append('the poses do not cover the directions of gravity (weakest spread %.2f, need 0.15): '
                        'turn the wrist further and tilt the forearm' % spread.min())
    if uneven > 0.10:
        problems.append('the force readings do not look like one mass turning in gravity (%.0f %% uneven): '
                        'something touched the gripper, or the tool frame is not what the follower thinks' % (100 * uneven))
    if not 0.05 < r['mass'] < 10.0:
        problems.append('a mass of %.2f kg is not a gripper' % r['mass'])
    if np.abs(r['com']).max() > 0.3:
        problems.append('a centre of mass %.2f m away is not a gripper' % np.abs(r['com']).max())
    if f_rms > 1.0 or n_rms > 0.08:
        problems.append('the fit leaves %.2f N, %.3f Nm unexplained (expect a few tenths of a newton)' % (f_rms, n_rms))
    if tilt > 10.0:
        problems.append('the sensor\'s z axis is %.0f degrees off the tool axis: check how it is mounted' % tilt)
    if problems:
        print('NOT WRITTEN:')
        for q in problems:
            print('  - ' + q)
        return 1

    with open(a.output, 'w') as out:
        out.write('# The gripper beyond the follower\'s force/torque sensor (follower --ft-tool; lib/ft_core.h).\n')
        out.write('# Fitted by scripts/ft_tool_cal.py from %s\n' % os.path.basename(a.recording))
        out.write('# %d cycles at rest; left unexplained %.3f N, %.4f Nm rms; gravity spread %.2f %.2f %.2f\n'
                  % (len(g), f_rms, n_rms, *spread))
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
