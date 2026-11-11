"""The controller's regressor (src/lib/regressor_W.h, compiled for ctypes),
the cable transmission, and the friction elements."""
import ctypes
import os
import subprocess

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
LIB = os.path.normpath(os.path.join(HERE, '..', '..', 'src', 'lib'))
REGRESSOR_H = os.path.join(LIB, 'regressor_W.h')
_SO = os.path.join(HERE, '_regressor.so')

# J1-J4 block of j2mp (wam8g.conf and the P2's wam7w.conf agree): motor angle = TM q.
# Motors 2 and 3 drive J2 and J3 through the cable differential.
TM = np.array([[-42.0, 0.0, 0.0, 0.0],
               [0.0, 28.25, -16.8155, 0.0],
               [0.0, -28.25, -16.8155, 0.0],
               [0.0, 0.0, 0.0, -18.0]])

# regressor_W.h columns: viscous and Coulomb friction (modelled here instead) and the rest
FC_COLS = [2, 11, 20, 29]
FV_COLS = [1, 10, 19, 28]
RB_COLS = [c for c in range(30) if c not in FC_COLS + FV_COLS]

# Jenkins elements: slip displacements. Motor side: at least five encoder
# counts (2 pi / 4096 rad each) so a joint toggling a count at rest does not
# flip an element; joint side (J2, J3): the same divided by the J2 ratio.
COUNT = 2 * np.pi / 4096
MOTOR_DELTAS = (7.7e-3, 1.6e-2, 3.2e-2, 6.4e-2, 0.13, 0.26)
JOINT_DELTAS = tuple(d / 28.25 for d in MOTOR_DELTAS)


def _lib():
    if not os.path.exists(_SO) or os.path.getmtime(_SO) < os.path.getmtime(REGRESSOR_H):
        src = os.path.join(HERE, '_regressor.cpp')
        with open(src, 'w') as f:
            f.write('#include "%s"\n' % REGRESSOR_H)
            f.write('extern "C" void batch_W(int n, const double* q, const double* dq, const double* ddq, double* W) {\n'
                    '  for (int k = 0; k < n; ++k) calculate_W(q + 4*k, dq + 4*k, ddq + 4*k, W + 120*k);\n}\n')
        subprocess.check_call(['g++', '-O2', '-shared', '-fPIC', '-I/usr/include/eigen3', '-o', _SO, src])
    return ctypes.CDLL(_SO)


_L = None


def W30(q, dq, ddq):
    """regressor_W.h, all 30 columns, (n, 4, 30)."""
    global _L
    if _L is None:
        _L = _lib()
    q, dq, ddq = (np.ascontiguousarray(np.atleast_2d(x), dtype=np.float64) for x in (q, dq, ddq))
    n = q.shape[0]
    out = np.zeros((n, 4, 30))
    P = ctypes.POINTER(ctypes.c_double)
    _L.batch_W(n, q.ctypes.data_as(P), dq.ctypes.data_as(P), ddq.ctypes.data_as(P), out.ctypes.data_as(P))
    return out


def W_rb(q, dq, ddq):
    return W30(q, dq, ddq)[:, :, RB_COLS]


def jenkins(x, deltas):
    """Jenkins (Maxwell-slip) elements on displacement x (n, m): spring
    deflection / delta, in [-1, 1], (n, m, K). Driven by increments, as the
    controller runs them (zero state at the first sample)."""
    x = np.asarray(x, dtype=float)
    n, m = x.shape
    d = np.asarray(deltas, dtype=float)[None, :]
    st = np.zeros((m, d.shape[1]))
    out = np.zeros((n, m, d.shape[1]))
    dx = np.vstack([np.zeros((1, m)), np.diff(x, axis=0)])
    for i in range(n):
        st = np.clip(st + dx[i][:, None], -d, d)
        out[i] = st
    return out / d


def motor_to_joint(F):
    """Motor-space columns (n, 4 motors, k) -> joint torques (n, 4, 4k)."""
    n, m, k = F.shape
    out = np.zeros((n, 4, m * k))
    for mm in range(m):
        out[:, :, mm * k:(mm + 1) * k] = TM[mm][None, :, None] * F[:, mm, None, :]
    return out
