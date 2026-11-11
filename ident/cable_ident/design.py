"""The next excitation, designed for teleop (python -m cable_ident design).

Each round adds the experiment that most reduces what the model does not yet
know, where the teleop needs it:
  * a Fourier trajectory (5 harmonics, 20 s period) whose coefficients
    maximise log det(F_now + F_new): F_now is the information of the data
    already fitted (excitation runs and self-driven teleop segments), F_new
    that of the new trajectory, both for the rigid-body parameters and the
    cable-friction terms (sequential D-optimal design);
  * constrained to the teleop's own workspace: every sample within --radius
    of a J1-J4 configuration the arm visited while teleoperated (the bags),
    so the arm only goes where the operator already drove it; joint speed
    and acceleration limited;
  * followed by reversal ladders: each joint in turn oscillates about the
    sync pose with amplitudes from 16 mrad down to 0.3 mrad (the friction
    elements' slip displacements are 0.2-6 mrad at J1), and slow sweeps with
    holds (the teleop's p95 joint speed is 0.04 rad/s);
starting and ending at rest at the sync pose. Output: t, q1-q4, dq1-dq4 at
100 Hz for dynamics_excitation --trajectory (which interpolates).
"""
import os

import numpy as np
import scipy.optimize as so
from scipy.spatial import cKDTree

from . import arms, data, features as F, model as M

SYNC = np.array([np.pi / 2, -np.pi / 2, -np.pi / 2, np.pi / 2])
RATE = 500.0
W0 = 2 * np.pi / 20.0
L_HARM = 5


def add_parser(sub):
    s = sub.add_parser('design', help='the next excitation trajectory')
    s.add_argument('--arm', required=True, choices=sorted(arms.ARMS))
    s.add_argument('--out', required=True, help='trajectory file (t q1-q4 dq1-dq4)')
    s.add_argument('--radius', type=float, default=0.12,
                   help='max distance (rad, J1-J4) from a configuration visited in teleop (default 0.12)')
    s.add_argument('--vmax', type=float, default=0.6, help='joint speed limit, rad/s (default 0.6)')
    s.add_argument('--amax', type=float, default=1.5, help='joint acceleration limit, rad/s^2 (default 1.5)')
    s.add_argument('--periods', type=int, default=2, help='Fourier periods of 20 s (default 2)')
    s.add_argument('--restarts', type=int, default=8)
    s.add_argument('--seed', type=int, default=0)
    s.add_argument('--no-ladders', action='store_true')


def teleop_configurations(arm, max_points=40000):
    """J1-J4 configurations of the active teleop (not at home), from the bags."""
    pts = []
    key = arm['topic'].strip('/').replace('/', '_')
    for b in arm['bags']:
        path = os.path.join(arms.BAGS, b + '.bag')
        if not os.path.exists(path):
            continue
        d = data.extract_bag(path)
        if key not in d:
            continue
        q = d[key][::25, 1:5]
        home = (q[:, 1] < -1.85) & (q[:, 3] > 2.75)
        pts.append(q[~home])
    P = np.vstack(pts)
    if len(P) > max_points:
        P = P[np.random.default_rng(0).choice(len(P), max_points, replace=False)]
    return P


def fourier(c, t, q0):
    """c: (4, 2L) [a_1..a_L, b_1..b_L]; q(0) = q0, dq(0) = sum a."""
    w = W0 * np.arange(1, L_HARM + 1)
    a, b = c[:, :L_HARM], c[:, L_HARM:]
    S, C = np.sin(np.outer(t, w)), np.cos(np.outer(t, w))
    q = q0[None, :] + (S / w) @ a.T - (C / w) @ b.T + ((1.0 / w) @ b.T)[None, :]
    dq = C @ a.T + S @ b.T
    ddq = (-S * w) @ a.T + (C * w) @ b.T
    return q, dq, ddq


def info(q, dq, ddq, T):
    """Information of a path for the rigid body and the friction (Coulomb
    proxies in motor and joint space, viscous): G^T G per sample, scaled."""
    W = M.W_rb(q, dq, ddq)
    x_dot = dq @ M.TM.T
    fr = M.motor_to_joint(np.stack([np.tanh(x_dot / 0.3), x_dot], axis=2))
    G = np.concatenate([W, fr], axis=2).reshape(-1, W.shape[2] + fr.shape[2])
    return G.T @ G * T


def current_information(arm_name):
    """F_now of the data the fit already uses (rigid body + friction proxies)."""
    from .pipeline import load
    arm, exc, segs = load(arm_name, quiet=True)
    Fsum = 0.0
    for s in exc + segs:
        sl = s.crop
        Fsum = Fsum + info(s.q[sl][::5], s.dq[sl][::5], s.ddq[sl][::5], 5 * s.T)
    return Fsum


def ladder(j, q0, T, amps=(16e-3, 6e-3, 2e-3, 0.8e-3, 0.3e-3), cycles=3, freq=0.4, span=0.15):
    """Joint j oscillates about q0 with decreasing amplitudes (smooth start and
    stop of each block), then a slow sweep +-span rad at 0.05 rad/s with 2 s
    holds at each end."""
    segs = []
    for A in amps:
        n = int(cycles / freq / T)
        t = np.arange(n) * T
        env = np.minimum(1.0, np.minimum(t, t[-1] - t) / (0.5 / freq))  # ramp in/out over half a period
        s = A * env * np.sin(2 * np.pi * freq * t)
        segs.append(s)
        segs.append(np.zeros(int(0.5 / T)))
    v = 0.05
    ramp = np.arange(0, span, v * T)
    hold = np.zeros(int(2.0 / T))
    segs += [ramp, span + hold, span - ramp, hold, -ramp, -span + hold, -span + ramp, hold]
    s = np.concatenate(segs)
    q = np.repeat(q0[None, :], len(s), axis=0)
    q[:, j] += s
    return q


def smooth_join(qa, qb, T, vmax):
    """A minimum-jerk move from qa to qb."""
    d = np.abs(qb - qa).max()
    if d < 1e-9:
        return np.zeros((0, 4))
    dur = max(1.0, 1.875 * d / vmax)
    s = np.linspace(0, 1, int(dur / T))
    h = 10 * s ** 3 - 15 * s ** 4 + 6 * s ** 5
    return qa[None, :] + h[:, None] * (qb - qa)[None, :]


def main(a):
    arm = arms.ARMS[a.arm]
    rng = np.random.default_rng(a.seed)
    P = teleop_configurations(arm)
    tree = cKDTree(P)
    print('%s: %d teleop configurations; J1-J4 range %s .. %s' % (a.arm, len(P), P.min(0).round(2), P.max(0).round(2)))
    print('information of the data fitted so far ...', flush=True)
    F_now = current_information(a.arm)
    ld_now = np.linalg.slogdet(F_now + 1e-9 * np.eye(len(F_now)))[1]
    T = 1.0 / RATE
    tc = np.arange(0, 20.0, 0.1)  # coarse grid for the optimisation
    q0 = SYNC.copy()
    if tree.query(q0)[0] > a.radius:
        q0 = P[tree.query(SYNC)[1]]
        print('sync pose outside the teleop set; centring on its nearest visited configuration', q0.round(3))

    def penalties(q, dq, ddq):
        dist = tree.query(q)[0]
        p = np.sum(np.maximum(dist - a.radius, 0) ** 2) * 1e4
        p += np.sum(np.maximum(np.abs(dq) - a.vmax, 0) ** 2) * 1e3
        p += np.sum(np.maximum(np.abs(ddq) - a.amax, 0) ** 2) * 1e2
        return p

    def objective(x):
        c = x.reshape(4, 2 * L_HARM)
        q, dq, ddq = fourier(c, tc, q0)
        Fn = info(q, dq, ddq, 0.1) * a.periods
        ld = np.linalg.slogdet(F_now + Fn)[1]
        return -(ld - ld_now) + penalties(q, dq, ddq)

    def feasible_scale(c):
        """The largest s <= 1 with q0 + s (q(t) - q0) inside every limit."""
        lo, hi = 0.0, 1.0
        q, dq, ddq = fourier(c, tc, q0)
        if penalties(q, dq, ddq) == 0.0:
            return 1.0
        for _ in range(30):
            mid = 0.5 * (lo + hi)
            qs = q0[None, :] + mid * (q - q0[None, :])
            if penalties(qs, mid * dq, mid * ddq) == 0.0:
                lo = mid
            else:
                hi = mid
        return lo

    best = None
    for r in range(a.restarts):
        x0 = rng.normal(0, 0.08, 4 * 2 * L_HARM)
        res = so.minimize(objective, x0, method='L-BFGS-B', options=dict(maxiter=200))
        c = res.x.reshape(4, -1)
        sc = feasible_scale(c)
        c = sc * c
        q, dq, ddq = fourier(c, tc, q0)
        gain = np.linalg.slogdet(F_now + info(q, dq, ddq, 0.1) * a.periods)[1] - ld_now
        print('  restart %2d: scaled by %.2f to fit the limits; information gain %.2f nats' % (r, sc, gain), flush=True)
        if best is None or gain > best[0]:
            best = (gain, c.reshape(-1))
    c = best[1].reshape(4, -1)
    t = np.arange(0, 20.0 * a.periods, T)
    q, dq, ddq = fourier(c, t, q0)
    env = np.minimum(1.0, np.minimum(t, t[-1] - t) / 2.0)[:, None]  # 2 s fade in and out
    q = q0[None, :] + env * (q - q0[None, :])
    parts = [np.repeat(q0[None, :], int(2.0 / T), axis=0), q, np.repeat(q[-1:], int(1.0 / T), axis=0)]
    if not a.no_ladders:
        for j in range(4):
            span = 0.15
            while True:
                lq = ladder(j, q0, T, span=span)
                if tree.query(lq[::50])[0].max() <= a.radius or span < 0.02:
                    break
                span *= 0.8
            print('  ladder J%d: sweep +-%.3f rad' % (j + 1, span))
            parts += [smooth_join(parts[-1][-1], lq[0], T, a.vmax), lq]
        parts.append(smooth_join(parts[-1][-1], q0, T, a.vmax))
        parts.append(np.repeat(q0[None, :], int(2.0 / T), axis=0))
    Q = np.vstack([p for p in parts if len(p)])
    dQ = np.gradient(Q, T, axis=0)
    tt = np.arange(len(Q)) * T
    dist = tree.query(Q[::10])[0]
    print('trajectory: %.0f s; max |dq| %s rad/s; max distance from the teleop set %.3f rad; information gain %.2f nats'
          % (tt[-1], np.abs(dQ).max(0).round(2), dist.max(), best[0]))
    keep = slice(None, None, 5)  # stored at 100 Hz; dynamics_excitation interpolates
    np.savetxt(a.out, np.column_stack([tt, Q, dQ])[keep], fmt='%.7f',
               header='t q1 q2 q3 q4 dq1 dq2 dq3 dq4 (rad, rad/s) -- cable_ident design --arm %s' % a.arm)
    print('wrote', a.out)
