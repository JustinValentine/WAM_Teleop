"""One convex program for the whole model:

    minimise   sum_j w_j ||G_j theta - y_j||^2 / N
             + sum_i lam_i ( -log det J_i + tr(J0_i^-1 J_i) )      prior (optional)
             + s sum (w_k - w_k+1)^2                                smoothness of the
                                                                    element forces
    s.t.       J_i(delta_i) >= eps I    pseudo-inertia of links 1-4 (physical consistency)
               element forces, viscous coefficients, rotor inertias >= 0  (passive friction)

theta = [beta_rb = K delta, friction..., rotors, offsets]: the rigid-body base
parameters are the 22 regrouped combinations (regressor_W.h) of the 40
standard ones delta. J_i is the 4x4 pseudo-inertia (Wensing, Kim & Slotine,
RA-L 2017); the log-det term is the Bregman divergence to a prior (Lee,
Wensing & Park, T-RO 2020). A Jenkins element with a fixed slip displacement
is linear in its slip force, so the presliding hysteresis joins the convex
program (Prandtl-Ishlinskii / Maxwell-slip with fixed thresholds)."""
import os

import cvxpy as cp
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
K_RB = np.load(os.path.join(HERE, 'K_rb.npy'))   # (22, 40): Feas-Ident-WAM4's sympybotics model
DELTA_CAD = np.load(os.path.join(HERE, 'delta_cad_wam7_links1to4.npy'))  # WAM7 CAD, links 1-4
N_RB = 22


def pseudo_inertia(d):
    """d = [Lxx, Lxy, Lxz, Lyy, Lyz, Lzz, lx, ly, lz, m] (L about the link
    frame origin, l = m c). numpy or cvxpy."""
    Lxx, Lxy, Lxz, Lyy, Lyz, Lzz, lx, ly, lz, m = [d[i] for i in range(10)]
    h = 0.5 * (Lxx + Lyy + Lzz)
    rows = [[h - Lxx, -Lxy, -Lxz, lx], [-Lxy, h - Lyy, -Lyz, ly], [-Lxz, -Lyz, h - Lzz, lz], [lx, ly, lz, m]]
    if isinstance(d, np.ndarray):
        return np.array(rows, dtype=float)
    return cp.bmat(rows)


def nonneg(label):
    return label.startswith(('ms_', 'fc_', 'fv_', 'Ia_'))


def compress(blocks, joint_weights=None):
    """Stack (G, y) blocks and QR-compress: ||G th - y||^2 = ||R th - c||^2 + sse0."""
    w = np.ones(4) if joint_weights is None else np.asarray(joint_weights, dtype=float)
    Gs = [(G * np.sqrt(w)[None, :, None]).reshape(-1, G.shape[2]) for G, _ in blocks]
    ys = [(y * np.sqrt(w)[None, :]).reshape(-1) for _, y in blocks]
    G, y = np.concatenate(Gs), np.concatenate(ys)
    Q, R = np.linalg.qr(G)
    c = Q.T @ y
    return R, c, float(y @ y - c @ c), G.shape[0]


def identify(blocks, labels, joint_weights=None, prior=DELTA_CAD, prior_lam=(1e-3, 1e-3, 1e-3, 0.0),
             smooth=1e-2, eps=1e-5, drop=('Ia_m1',), constrained=True, solver='CLARABEL'):
    """blocks: [(G, y)] from features.columns; labels: features.labels(spec).
    drop: columns pinned to 0 (motor 1's rotor is J1's link inertia in the
    base parameters). constrained=False: ordinary least squares (ablation)."""
    R, c, sse0, nrows = compress(blocks, joint_weights)
    p = len(labels)
    if not constrained:
        keep = [i for i, l in enumerate(labels) if l not in drop]
        th = np.zeros(p)
        th[keep] = np.linalg.lstsq(R[:, keep], c, rcond=None)[0]
        return dict(theta=th, delta=None, status='ols', labels=list(labels))
    delta = cp.Variable(40)
    rest = cp.Variable(p - N_RB)
    pin = np.diag([0.0 if labels[N_RB + k] in drop else 1.0 for k in range(p - N_RB)])
    theta = cp.hstack([K_RB @ delta, pin @ rest])
    obj = cp.sum_squares(R @ theta - c) / nrows
    cons, Js = [], []
    for i in range(4):
        J = pseudo_inertia(delta[10 * i:10 * i + 10])
        Js.append(J)
        cons.append(J >> eps * np.eye(4))
    cons += [rest[k] >= 0 for k, l in enumerate(labels[N_RB:]) if nonneg(l)]
    if prior is not None:
        for i, lam in enumerate(prior_lam):
            if lam > 0:
                J0 = pseudo_inertia(prior[10 * i:10 * i + 10])
                e, V = np.linalg.eigh(J0)
                J0 = V @ np.diag(np.maximum(e, 1e-4)) @ V.T
                obj = obj + lam * (-cp.log_det(Js[i]) + cp.trace(np.linalg.inv(J0) @ Js[i]))
    if smooth:
        groups = {}
        for k, l in enumerate(labels[N_RB:]):
            if l.startswith('ms_'):
                groups.setdefault(l.rsplit('_', 1)[0], []).append(k)
        for ks in groups.values():
            for a, b in zip(ks[:-1], ks[1:]):
                obj = obj + smooth * cp.square(rest[a] - rest[b])
    prob = cp.Problem(cp.Minimize(obj), cons)
    for s in (solver, 'SCS'):
        try:
            prob.solve(solver=s, **({'max_iters': 200000, 'eps': 1e-9} if s == 'SCS' else {}))
        except cp.error.SolverError:
            continue
        if prob.status in ('optimal', 'optimal_inaccurate'):
            break
    if prob.status not in ('optimal', 'optimal_inaccurate'):
        raise RuntimeError('identification: ' + str(prob.status))
    th = np.concatenate([K_RB @ delta.value, pin @ rest.value])
    th[N_RB:][[nonneg(l) for l in labels[N_RB:]]] = np.maximum(th[N_RB:][[nonneg(l) for l in labels[N_RB:]]], 0.0)
    d = delta.value
    return dict(theta=th, delta=d, status=prob.status, labels=list(labels),
                min_eig=[float(np.linalg.eigvalsh(pseudo_inertia(d[10 * i:10 * i + 10])).min()) for i in range(4)],
                masses=[float(d[10 * i + 9]) for i in range(4)])
