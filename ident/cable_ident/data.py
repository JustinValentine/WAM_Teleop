"""Training and test data: excitation logs and the teleop bags' self-driven
segments, each with the controller's own signals (velocity, acceleration
chain) so the fit is the model the controller runs.

Excitation logs (dynamics_excitation): t, q1-q4, tau1-tau4 at 250 Hz (the
4-joint format), or with --full-log: t, q1-qN, dq1-dqN, tau1-tauN at 500 Hz.
"""
import os

import numpy as np
from scipy.ndimage import maximum_filter1d, minimum_filter1d

from . import config, model as M, signals

CACHE = os.environ.get('CABLE_IDENT_CACHE', os.path.join(os.path.expanduser('~'), '.cache', 'cable_ident'))


class Segment:
    """Arm signals over a window, the cropped part being what is fitted or
    scored (the rest only warms up the friction elements and filters).
    tau is the commanded torque minus the wrist's gravity relative to the
    identification pose, i.e. what the J1-J4 model must predict."""

    def __init__(self, name, T, q, dq, tau, crop, gravity=None, wrist_first=4, kind=''):
        self.name, self.T, self.kind = name, T, kind
        self.qfull = q
        self.q = q[:, :4]
        self.dq = dq[:, :4]
        self.ddq = signals.controller_acceleration(self.dq, T)
        self.tau_cmd = tau[:, :4]
        self.crop = crop
        if gravity is not None and q.shape[1] > 4:
            self.dg = gravity.wrist_correction(q, wrist_first)
            self.load = gravity.torque(q)[:, :4]
        else:
            self.dg = np.zeros_like(self.q)
            self.load = None
        self.tau = self.tau_cmd - self.dg

    @property
    def seconds(self):
        return (self.crop.stop - self.crop.start) * self.T


def excitation(path, arm, gravity=None):
    d = np.loadtxt(path)
    T = float(np.median(np.diff(d[:, 0])))
    ncol = d.shape[1] - 1
    if ncol == 8:  # 4-joint format: velocities as libbarrett computes them
        q, tau = d[:, 1:5], d[:, 5:9]
        dq = signals.libbarrett_velocity(q, T)
        qfull = np.hstack([q, np.zeros((len(q), arm['dof'] - 4))])  # the wrist was held at 0
        crop = slice(int(12.0 / T), len(q) - int(4.7 / T))  # as Feas-Ident: settled start, decay excluded
    else:
        n = ncol // 3
        qfull, dq, tau = d[:, 1:1 + n], d[:, 1 + n:1 + 2 * n], d[:, 1 + 2 * n:1 + 3 * n]
        crop = slice(int(2.0 / T), len(qfull) - int(1.2 / T))
        q = qfull
    return Segment(os.path.basename(path), T, qfull, dq, tau, crop, gravity, kind='excitation')


# --- teleop bags -----------------------------------------------------------

def extract_bag(path):
    """Joint states of both arms: stamp, q, dq, tau (cached as .npz)."""
    os.makedirs(CACHE, exist_ok=True)
    key = os.path.join(CACHE, os.path.basename(path) + '.e1.npz')
    if os.path.exists(key):
        return dict(np.load(key))
    from rosbags.rosbag1 import Reader
    from rosbags.typesys import Stores, get_typestore
    ts = get_typestore(Stores.ROS1_NOETIC)
    out = {}
    with Reader(path) as r:
        conns = [c for c in r.connections if c.topic in ('/leader/joint_states', '/follower/joint_states')]
        bufs = {c.topic: [] for c in conns}
        for c, t, raw in r.messages(connections=conns):
            m = ts.deserialize_ros1(raw, c.msgtype)
            stamp = m.header.stamp.sec + 1e-9 * m.header.stamp.nanosec
            bufs[c.topic].append(np.concatenate([[stamp], m.position, m.velocity, m.effort]))
    for k, v in bufs.items():
        out[k.strip('/').replace('/', '_')] = np.array(v)
    np.savez_compressed(key, **out)
    return out


def resample(A, n, rate=500.0):
    t = A[:, 0]
    keep = np.concatenate([[True], np.diff(t) > 0])
    A, t = A[keep], t[keep]
    tu = np.arange(t[0], t[-1], 1.0 / rate)
    out = np.empty((len(tu), A.shape[1] - 1))
    for c in range(1, A.shape[1]):
        out[:, c - 1] = np.interp(tu, t, A[:, c])
    return tu - tu[0], out[:, :n], out[:, n:2 * n], out[:, 2 * n:3 * n]


def _still(t, q, window=0.3, tol=0.002):
    w = max(2, int(window / np.median(np.diff(t))))
    span = maximum_filter1d(q[:, :4], w, axis=0) - minimum_filter1d(q[:, :4], w, axis=0)
    return span.max(axis=1) < tol


SYNC = [np.array([1.5708, -1.5708, -1.5708, 1.5708]), np.array([0.215, -1.55, -1.545, 2.015])]
HOME = np.array([0.0, -2.0, 0.0, 2.95])


def self_driven(t, q, min_disp=0.3, rest_time=2.0, pre=0.5):
    """The arm's own moves (moveTo the sync pose or home, no operator, no
    contact): from a rest, a smooth move (each joint's velocity changes sign at
    most once) ending at the sync pose or home and followed by a rest.
    Returns [(i0, i_arrive, i_end, kind)]."""
    rate = 1.0 / np.median(np.diff(t))
    still = _still(t, q)
    segs, i, n = [], 0, len(t)
    while i < n - 1:
        while i < n - 1 and still[i]:
            i += 1
        start = i
        while i < n - 1 and not still[i]:
            i += 1
        arrive = i
        while i < n - 1 and still[i]:
            i += 1
        end = i
        qa = q[arrive, :4]
        if np.abs(qa - q[start, :4]).max() <= min_disp or (end - arrive) / rate < rest_time:
            continue
        v = np.diff(q[start:arrive, :4], axis=0) * rate
        v = np.array([np.convolve(v[:, j], np.ones(25) / 25, mode='same') for j in range(4)]).T
        flips = max(int(np.sum(np.diff(np.sign(v[np.abs(v[:, j]) > 0.02, j])) != 0)) for j in range(4))
        if flips > 1:
            continue
        if any(np.abs(qa - s).max() < 0.02 for s in SYNC):
            kind = 'sync'
        elif np.abs(qa[[1, 3]] - HOME[[1, 3]]).max() < 0.1 and abs(qa[0]) < 0.1:
            kind = 'home'
        else:
            continue
        segs.append((max(0, start - int(pre * rate)), arrive, max(arrive, end - int(0.5 * rate)), kind))
    return segs


def bag_segments(bagname, arm, gravity, history=20.0, leader_hold=3.0):
    """Self-driven segments of one arm in one bag, as Segments (500 Hz)."""
    from .arms import BAGS
    path = os.path.join(BAGS, bagname + '.bag')
    d = extract_bag(path)
    key = arm['topic'].strip('/').replace('/', '_')
    if key not in d:
        return []
    t, q, dq, tau = resample(d[key], arm['dof'])
    T = 1.0 / 500.0
    out = []
    for (a, arrive, end, kind) in self_driven(t, q):
        if arm['robot'] == 'wam8g':  # the operator may take the leader's handle while it waits
            end = min(end, arrive + int(leader_hold / T))
        w0 = max(0, a - int(history / T))
        sl = slice(w0, end)
        out.append(Segment('%s@%.0fs' % (bagname, t[a]), T, q[sl], dq[sl], tau[sl],
                           slice(a - w0, end - w0), gravity, kind=kind))
    return out


def gravity_of(arm):
    cfg = config.read(arm['config'])
    return config.Gravity(cfg[arm['group']])
