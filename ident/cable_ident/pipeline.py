"""Data loading, fitting and leave-one-session-out scoring shared by the commands."""
import os

import numpy as np

from . import arms, data, features as F, ident


def load(arm_name, bags=True, quiet=False):
    arm = arms.ARMS[arm_name]
    grav = data.gravity_of(arm)
    exc = [data.excitation(p, arm, grav) for p in arm['excitation']]
    segs = []
    if bags:
        for b in arm['bags']:
            if not os.path.exists(os.path.join(arms.BAGS, b + '.bag')):
                continue
            segs += data.bag_segments(b, arm, grav)
    if not quiet:
        print('%s: %d excitation run(s) (%.0f s), %d self-driven segments (%.0f s) in %d sessions' % (
            arm_name, len(exc), sum(s.seconds for s in exc), len(segs), sum(s.seconds for s in segs),
            len(set(s.name.split('@')[0] for s in segs))))
    return arm, exc, segs


def session(seg):
    return seg.name.split('@')[0]


def fit_blocks(spec, exc, segs, exc_weight, wrist=True):
    blocks = []
    nb = sum(len(F.columns(s, spec, wrist=wrist)[1]) for s in segs) if segs else 0
    for s in exc:
        G, y = F.columns(s, spec, wrist=wrist)
        w = 1.0
        if segs and exc_weight is not None:
            # the excitation's total weight = exc_weight x the bags' total
            w = np.sqrt(exc_weight * nb / len(y))
        blocks.append((G * w, y * w))
    for s in segs:
        blocks.append(F.columns(s, spec, wrist=wrist))
    return blocks


def score(theta, spec, segs, wrist=True):
    e = [y - G @ theta for G, y in (F.columns(s, spec, wrist=wrist) for s in segs)]
    e = np.vstack(e)
    return F.rms(e), np.abs(e.mean(axis=0))


def run_cv(spec, exc, segs, exc_weight=1.0, use_bags=True, wrist=True, **kw):
    sessions = sorted(set(session(s) for s in segs))
    errs, exc_err = [], []
    for held in sessions:
        train = [s for s in segs if session(s) != held] if use_bags else []
        res = ident.identify(fit_blocks(spec, exc, train, exc_weight, wrist), F.labels(spec), **kw)
        for s in segs:
            if session(s) == held:
                G, y = F.columns(s, spec, wrist=wrist)
                errs.append(y - G @ res['theta'])
        G, y = F.columns(exc[0], spec, wrist=wrist)
        exc_err.append(F.rms(y - G @ res['theta']))
    e = np.vstack(errs)
    return F.rms(e), np.abs(e.mean(axis=0)), np.mean(exc_err, axis=0)
