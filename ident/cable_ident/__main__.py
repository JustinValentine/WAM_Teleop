"""python -m cable_ident {fit,cv,ablate,hysteresis,design,testvectors} --arm {leader,follower}

  fit         fit on the excitation logs and the bags' self-driven segments;
              write src/lib/<header> and a JSON record
  cv          leave-one-session-out: fit without one teleop session, test
              on its self-driven segments, for every session
  ablate      the cv for the model variants a reviewer asks for
  hysteresis  friction after reversals vs displacement and vs time
              (rate independence of the cable drive's friction)
  design      the next excitation (cable_ident.design)
  testvectors test/data for the controller's C++ tests (run after fit)
"""
import argparse
import os
import sys
import warnings

import numpy as np

from . import arms, data, export, features as F, ident, model as M
from .pipeline import fit_blocks, load, run_cv, score, session

warnings.filterwarnings('ignore', module='cvxpy')


def fmt(v):
    return '[' + ' '.join('%5.2f' % x for x in v) + ']'


def cmd_fit(a):
    arm, exc, segs = load(a.arm, bags=not a.no_bags)
    spec = F.Spec('cable')
    res = ident.identify(fit_blocks(spec, exc, segs, a.exc_weight), F.labels(spec))
    th = res['theta']
    print('physically consistent: smallest pseudo-inertia eigenvalue per link', np.round(res['min_eig'], 5))
    print('link masses (kg)', np.round(res['masses'], 2))
    ex = [F.rms(y - G @ th) for G, y in (F.columns(s, spec) for s in exc)]
    print('excitation residual (Nm, 10 Hz):', ' '.join(fmt(e) for e in ex))
    if segs:
        r, m = score(th, spec, segs)
        print('self-driven segments (in-sample) rms', fmt(r), '|mean|', fmt(m))
    note = ('Data: %s; %d self-driven teleop segments (%.0f s) from %s.\n'
            'Model: %s; physically consistent rigid body, passive cable friction (fit on 10 Hz-filtered\n'
            'torques with the controller\'s velocity and acceleration chain).' % (
                ', '.join(os.path.basename(p) for p in arm['excitation']), len(segs), sum(s.seconds for s in segs),
                ', '.join(sorted(set(session(s) for s in segs))) or 'none', spec))
    out = os.path.join(a.out_dir, arm['header'])
    with open(out, 'w') as f:
        f.write(export.header(a.arm, spec, th, F.labels(spec), arm['namespace'], note))
    export.record(os.path.splitext(out)[0] + '.json', arm=a.arm, spec=repr(spec), labels=F.labels(spec), theta=th,
                  delta=res['delta'], min_eig=res['min_eig'], excitation=arm['excitation'],
                  segments=[s.name for s in segs])
    print('wrote', out)


def cmd_cv(a):
    arm, exc, segs = load(a.arm)
    spec = F.Spec('cable')
    for use_bags in (False, True):
        r, m, ex = run_cv(spec, exc, segs, a.exc_weight, use_bags)
        print('train on %-22s held-out sessions rms %s |mean| %s ; excitation %s' % (
            'excitation + other sessions' if use_bags else 'excitation only', fmt(r), fmt(m), fmt(ex)))


ABLATIONS = [
    # (label, spec, fit kwargs, use bags, wrist correction)
    ('viscous only, inertia x0.25, OLS (prior WAM work)', F.Spec('viscous', 0.25, False, False), dict(constrained=False), False, False),
    ('Coulomb tanh, LMI (Sousa & Cortesao)', F.Spec('joint_coulomb', 1.0, False, False), dict(prior=None, smooth=0), False, False),
    ('joint-space Jenkins, LMI', F.Spec('joint_jenkins', 1.0, True, True), {}, False, True),
    ('motor-space Jenkins, LMI', F.Spec('motor_jenkins', 1.0, True, True), {}, False, True),
    ('cable (motor + J2/J3 Jenkins), LMI', F.Spec('cable', 1.0, True, True), {}, False, True),
    ('cable, OLS (no LMI, no sign constraints)', F.Spec('cable', 1.0, True, True), dict(constrained=False), False, True),
    ('cable, no wrist-gravity correction', F.Spec('cable', 1.0, True, True), {}, False, False),
    ('cable, no inertia in the model', F.Spec('cable', 0.0, True, True), {}, False, True),
    ('cable + teleop-region data (proposed)', F.Spec('cable', 1.0, True, True), {}, True, True),
]


def cmd_ablate(a):
    arm, exc, segs = load(a.arm)
    print('leave-one-session-out on the self-driven segments (Nm rms, J1-J4, 10 Hz):')
    for label, spec, kw, use_bags, wrist in ABLATIONS:
        try:
            r, m, ex = run_cv(spec, exc, segs, a.exc_weight, use_bags, wrist, **kw)
            print('  %-52s %s |mean| %s  exc %s' % (label, fmt(r), fmt(m), fmt(ex)), flush=True)
        except Exception as e:  # report and go on
            print('  %-52s failed: %s' % (label, e))


def cmd_testvectors(a):
    """test/data for test_cable_drive_model and test_arm_gravity, from the
    headers' .json records (run after fit)."""
    import json
    out_dir = os.path.join(arms.PKG, 'test', 'data')
    os.makedirs(out_dir, exist_ok=True)
    rng = np.random.default_rng(1)
    for name, arm in sorted(arms.ARMS.items()):
        rec = json.load(open(os.path.join(arms.PKG, 'src', 'lib', arm['header'].replace('.h', '.json'))))
        spec = F.Spec('cable')
        grav = data.gravity_of(arm)
        seg = data.excitation(arm['excitation'][0], arm, grav)
        n = 3000
        seg.q, seg.dq, seg.ddq = seg.q[:n], seg.dq[:n], seg.ddq[:n]
        seg.tau, seg.tau_cmd = seg.tau[:n], seg.tau_cmd[:n]
        pred = F.predict(seg, spec, np.array(rec['theta']))
        np.savetxt(os.path.join(out_dir, 'cable_model_%s.txt' % name), np.column_stack([seg.q, seg.dq, seg.ddq, pred]),
                   fmt='%.12g', header='q1-4 dq1-4 ddq1-4 model1-4 (python -m cable_ident testvectors; %s)' % arm['header'])
        q = rng.uniform(-2.5, 2.5, size=(200, arm['dof']))
        np.savetxt(os.path.join(out_dir, 'arm_gravity_%s.txt' % name),
                   np.column_stack([q, grav.torque(q), grav.wrist_correction(q, 4)]), fmt='%.12g',
                   header='%s: q1-q%d, gravity torque, wrist correction J1-J4 (python -m cable_ident testvectors)' % (
                       name, arm['dof']))
        print('wrote test vectors for', name)


def cmd_hysteresis(a):
    from .hysteresis import report
    arm, exc, segs = load(a.arm, bags=False)
    report(exc[0])


def main(argv=None):
    p = argparse.ArgumentParser(prog='cable_ident', description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = p.add_subparsers(dest='cmd', required=True)
    for name in ('fit', 'cv', 'ablate', 'hysteresis'):
        s = sub.add_parser(name)
        s.add_argument('--arm', required=True, choices=sorted(arms.ARMS))
        s.add_argument('--exc-weight', type=float, default=1.0,
                       help="the excitation's total weight relative to the teleop segments' (default 1)")
        if name == 'fit':
            s.add_argument('--no-bags', action='store_true', help='excitation only')
            s.add_argument('--out-dir', default=os.path.join(arms.PKG, 'src', 'lib'))
    from . import design
    design.add_parser(sub)
    sub.add_parser('testvectors', help='test/data for the C++ tests (after fit)')
    a = p.parse_args(argv)
    if a.cmd == 'testvectors':
        return cmd_testvectors(a)
    if a.cmd == 'design':
        return design.main(a)
    return {'fit': cmd_fit, 'cv': cmd_cv, 'ablate': cmd_ablate, 'hysteresis': cmd_hysteresis}[a.cmd](a)


if __name__ == '__main__':
    sys.exit(main())
