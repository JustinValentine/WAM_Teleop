"""Rate independence of the cable drive's friction: after each reversal of a
joint, the friction torque (commanded torque minus the fitted model without
its friction elements) against the displacement since the reversal and
against the time since it, for slow and for fast reversals. Rate-independent
hysteresis (a Jenkins / Prandtl-Ishlinskii operator) collapses in
displacement and not in time."""
import numpy as np

from . import features as F, ident

DISP_BINS = [0, 0.25, 0.5, 1, 2, 4, 8, 16, 32]   # joint mrad
TIME_BINS = [0, 0.02, 0.05, 0.1, 0.2, 0.4, 0.8, 1.6]  # s


def reversals(seg, j, min_len=50):
    q = seg.q[seg.crop, j]
    v = np.convolve(np.gradient(q, seg.T), np.ones(5) / 5, mode='same')
    s = np.sign(v)
    idx = np.where(s[1:] * s[:-1] < 0)[0] + 1
    acc = np.abs(np.gradient(v, seg.T))
    out = []
    for k, i0 in enumerate(idx):
        i1 = idx[k + 1] if k + 1 < len(idx) else len(q)
        if i1 - i0 >= min_len:
            out.append((i0, i1, acc[i0], np.sign(v[min(i0 + 10, i1 - 1)])))
    return out


def binned(rows, key, bins):
    line = []
    for a, b in zip(bins[:-1], bins[1:]):
        v = np.concatenate([f[(k >= a) & (k < b)] for k, f in rows]) if rows else np.array([])
        line.append('%7.2f' % v.mean() if len(v) > 5 else '%7s' % '-')
    return ''.join(line)


def report(seg):
    spec = F.Spec('cable')
    G, y = F.columns(seg, spec)
    labels = F.labels(spec)
    res = ident.identify([(G, y)], labels)
    th = res['theta'].copy()
    fric = [i for i, l in enumerate(labels) if l.startswith('ms_')]
    th_nf = th.copy()
    th_nf[fric] = 0.0
    meas = y - G @ th_nf          # friction the model's other terms leave unexplained
    model = G[:, :, fric] @ th[fric]
    q = seg.q[seg.crop]
    print('friction after reversals (joint Nm, signed along the new motion), %s' % seg.name)
    for j in (0, 3):
        rev = reversals(seg, j)
        if not rev:
            continue
        med = np.median([r[2] for r in rev])
        print('J%d: %d reversals, median |acc| at reversal %.2f rad/s^2' % (j + 1, len(rev), med))
        print('  displacement (mrad) ' + ''.join('%7s' % ('<%g' % b) for b in DISP_BINS[1:]))
        for name, sel in (('slow', lambda r: r[2] <= med), ('fast', lambda r: r[2] > med)):
            rows_m = [(np.abs(q[i0:i1, j] - q[i0, j]) * 1e3, sg * meas[i0:i1, j]) for i0, i1, acc, sg in rev if sel((i0, i1, acc))]
            rows_p = [(np.abs(q[i0:i1, j] - q[i0, j]) * 1e3, sg * model[i0:i1, j]) for i0, i1, acc, sg in rev if sel((i0, i1, acc))]
            print('  %s measured        %s' % (name, binned(rows_m, None, DISP_BINS)))
            print('  %s model           %s' % (name, binned(rows_p, None, DISP_BINS)))
        print('  time (s)            ' + ''.join('%7s' % ('<%g' % b) for b in TIME_BINS[1:]))
        for name, sel in (('slow', lambda r: r[2] <= med), ('fast', lambda r: r[2] > med)):
            rows_t = [(np.arange(i1 - i0) * seg.T, sg * meas[i0:i1, j]) for i0, i1, acc, sg in rev if sel((i0, i1, acc))]
            print('  %s measured        %s' % (name, binned(rows_t, None, TIME_BINS)))
