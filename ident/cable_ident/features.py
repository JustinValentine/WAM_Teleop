"""Regression columns: the model's terms as the controller computes them
(libbarrett velocity, the controller's acceleration chain, Jenkins elements
driven by encoder increments), filtered by the same causal filter as the
torque. The external-torque estimate of a parameter vector theta on a segment
is  y - G theta  (zero in free motion)."""
import numpy as np

from . import model as M
from .signals import Filt


class Spec:
    """friction: 'cable'          motor Jenkins + J2/J3 joint Jenkins + motor viscous (proposed)
                 'motor_jenkins'  motor Jenkins + motor viscous
                 'joint_jenkins'  joint Jenkins + joint viscous
                 'joint_coulomb'  tanh(dq / 0.01) Coulomb + joint viscous (Sousa & Cortesao)
                 'viscous'        joint viscous only (the deployed models)
       inertia:  share of the controller's acceleration in the model (1, 0.25, 0)
       rotor, offset: motor rotor inertia / constant torque per joint"""

    def __init__(self, friction='cable', inertia=1.0, rotor=True, offset=True,
                 motor_deltas=M.MOTOR_DELTAS, joint_deltas=M.JOINT_DELTAS):
        self.friction, self.inertia, self.rotor, self.offset = friction, inertia, rotor, offset
        self.motor_deltas, self.joint_deltas = tuple(motor_deltas), tuple(joint_deltas)

    def __repr__(self):
        return 'Spec(%s, inertia=%g, rotor=%s, offset=%s)' % (self.friction, self.inertia, self.rotor, self.offset)


def labels(spec):
    L = ['rb%d' % c for c in M.RB_COLS]
    f = spec.friction
    if f in ('cable', 'motor_jenkins'):
        L += ['ms_m%d_%d' % (m + 1, i) for m in range(4) for i in range(len(spec.motor_deltas))]
    if f == 'cable':
        L += ['ms_j%d_%d' % (j + 1, i) for j in (1, 2) for i in range(len(spec.joint_deltas))]
    if f == 'joint_jenkins':
        L += ['ms_j%d_%d' % (j + 1, i) for j in range(4) for i in range(len(spec.joint_deltas))]
    if f == 'joint_coulomb':
        L += ['fc_j%d' % (j + 1) for j in range(4)]
    if f in ('cable', 'motor_jenkins'):
        L += ['fv_m%d' % (m + 1) for m in range(4)]
    else:
        L += ['fv_j%d' % (j + 1) for j in range(4)]
    if spec.rotor:
        L += ['Ia_m%d' % (m + 1) for m in range(4)]
    if spec.offset:
        L += ['off_j%d' % (j + 1) for j in range(4)]
    return L


def columns(seg, spec, filt=Filt(), wrist=True):
    """(G, y) cropped: G (n, 4, p), y (n, 4)."""
    q, dq = seg.q, seg.dq
    ddq = spec.inertia * seg.ddq
    n = len(q)
    x, dx = q @ M.TM.T, dq @ M.TM.T
    blocks = [M.W_rb(q, dq, ddq)]
    f = spec.friction
    if f in ('cable', 'motor_jenkins'):
        blocks.append(M.motor_to_joint(M.jenkins(x, spec.motor_deltas)))
    if f == 'cable':
        phi = M.jenkins(q[:, 1:3], spec.joint_deltas)
        Fj = np.zeros((n, 4, 2 * phi.shape[2]))
        k = phi.shape[2]
        Fj[:, 1, :k] = phi[:, 0]
        Fj[:, 2, k:] = phi[:, 1]
        blocks.append(Fj)
    if f == 'joint_jenkins':
        phi = M.jenkins(q, spec.joint_deltas)
        k = phi.shape[2]
        Fj = np.zeros((n, 4, 4 * k))
        for j in range(4):
            Fj[:, j, j * k:(j + 1) * k] = phi[:, j]
        blocks.append(Fj)
    if f == 'joint_coulomb':
        blocks.append(np.eye(4)[None] * np.tanh(dq / 0.01)[:, None, :])
    if f in ('cable', 'motor_jenkins'):
        blocks.append(M.motor_to_joint(dx[:, :, None]))
    else:
        blocks.append(np.eye(4)[None] * dq[:, None, :])
    if spec.rotor:
        blocks.append(M.motor_to_joint((ddq @ M.TM.T)[:, :, None]))
    if spec.offset:
        blocks.append(np.broadcast_to(np.eye(4), (n, 4, 4)))
    G = np.concatenate(blocks, axis=2)
    tau = seg.tau if wrist else seg.tau_cmd
    Gf = filt(G.reshape(n, -1), seg.T).reshape(G.shape)
    yf = filt(tau, seg.T)
    return Gf[seg.crop], yf[seg.crop]


def rms(e):
    return np.sqrt((e ** 2).mean(axis=0))


def predict(seg, spec, theta):
    """The model torque over the whole window, unfiltered (as the controller
    computes it, sample by sample)."""
    ident_filter = lambda x, T: x  # noqa: E731
    G, _ = columns(_Uncropped(seg), spec, filt=ident_filter)
    return G @ theta


class _Uncropped:
    def __init__(self, seg):
        self.__dict__.update(seg.__dict__)
        self.crop = slice(0, len(seg.q))
