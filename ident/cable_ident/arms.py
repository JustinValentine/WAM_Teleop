"""The two arms: configuration, logs, and which teleop recordings share each
arm's current calibration (zerocal/gravitycal), so they can train or test."""
import os

REPO = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', '..', '..', '..'))
PKG = os.path.join(REPO, 'wam_ws', 'src', 'wam_teleop')
EXC = os.path.join(REPO, 'excitation')
BAGS = os.path.join(REPO, 'teleop_logs')

ARMS = {
    # leader: the CANopen wam8g (passive J5, Alberta wrist J6-J8)
    'leader': dict(
        robot='wam8g',
        config=os.path.join(REPO, 'config', 'wam8g', 'wam8g.conf'), group='wam8g', dof=8,
        topic='/leader/joint_states',
        excitation=[os.path.join(EXC, 'wam8g_rbtlog_cpp_4dof_20260925.dat')],
        # zerocal 2026-09-25 14:21 (the excitation above); again 2026-09-28 12:52 (small offsets)
        bags=['teleop_20260926_181348', 'teleop_20260929_144721', 'teleop_20260929_151957',
              'teleop_20260929_160843', 'teleop_20260930_125552', 'teleop_20260930_152304'],
        header='cable_params_wam8g.h', namespace='cable_params_wam8g'),
    # follower: the P2 WAM "zeus" with its BarrettHand
    'follower': dict(
        robot='zeus',
        config=os.path.join(PKG, 'config', 'follower-zeus-hand', 'wam7w.conf'), group='wam7w', dof=7,
        topic='/follower/joint_states',
        excitation=[os.path.join(EXC, 'zeus_rbtlog_cpp_4dof.dat')],
        # zerocal/gravitycal 2026-09-24 11:07/11:18, before the excitation and every bag below
        bags=['test1_u1', 'test4_u4', 'test5_u4_nowrist', 'test6_24_g020', 'test13_nofilter', 'test14_notch35',
              'teleop_20260926_181348', 'teleop_20260929_144721', 'teleop_20260929_151957',
              'teleop_20260929_160843', 'teleop_20260930_125552', 'teleop_20260930_152304'],
        header='cable_params_zeus_hand.h', namespace='cable_params_zeus_hand'),
    # slax: the legacy WAM on odin with the Barrett F/T sensor and the gripper of 2026-11-12
    # (zerocal 2026-11-11 19:16, gravitycal 2026-11-12 07:37; `wam cal excite` runs 1-3, wrist held at 0).
    # SLAX_RUNS picks other runs of that day (e.g. SLAX_RUNS=1,2 to score on run 3).
    'slax': dict(
        robot='slax',
        config=os.path.join(PKG, 'config', 'follower-slax-skid-ft', 'wam7w.conf'), group='wam7w', dof=7,
        topic='/follower/joint_states',
        excitation=[os.path.join(EXC, 'slax_20261112_%s.dat' % r) for r in os.environ.get('SLAX_RUNS', '1,2,3').split(',')],
        bags=[],
        header='cable_params_slax_20261112.h', namespace='cable_params_slax_20261112'),
}
