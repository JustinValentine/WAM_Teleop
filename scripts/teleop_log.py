#!/usr/bin/env python3
"""Reads the teleop's own recordings (leader_wam8g / follower --record; lib/teleop_recorder.h).

  teleop_log.py FILE [FILE ...]        timing, link and recording summary of each file
  teleop_log.py FILE --npz OUT.npz     one array per signal (q, dq, tau_cmd, ext, ...), plus t
  teleop_log.py FILE --csv OUT.csv     every column as text

A file is three text lines (format, column names, the program's command line) and then
little-endian float64 rows, one per control cycle. Columns named x_1..x_n are one signal x.
tau_cmd = tau_grav + tau_pid + tau_ff is the torque the arm was commanded that cycle.
Only numpy is needed; no ROS.

In Python:  from teleop_log import load;  log = load('teleop_..._leader.tlog');  log['q'], log['t']
"""
import argparse
import sys

import numpy as np

STATES = {0: 'not linked yet', 1: 'linked', 2: 'unlinked'}
MODES = {0: 'full', 1: '4C off', 2: 'P-P'}


def read(path):
    """(column names, rows as an (N, columns) float64 array, the note line)."""
    with open(path, 'rb') as f:
        first = f.readline().decode().split()
        if len(first) != 3 or first[0] != 'TELEOPLOG' or first[1] != '1':
            raise ValueError('%s is not a teleop recording' % path)
        width = int(first[2])
        names = f.readline().decode().rstrip('\n').split(',')
        note = f.readline().decode().rstrip('\n')
        if len(names) != width:
            raise ValueError('%s: %d column names for %d columns' % (path, len(names), width))
        raw = np.frombuffer(f.read(), dtype='<f8')
    rows = raw[:len(raw) // width * width].reshape(-1, width)  # a torn last row is dropped
    return names, rows, note


def load(path):
    """A dict: one (N, n) array per signal x_1..x_n, one (N,) array per single column, plus
    'tau_cmd', 'columns', 'note'."""
    names, rows, note = read(path)
    out, groups = {}, {}
    for i, name in enumerate(names):
        base, _, index = name.rpartition('_')
        if base and index.isdigit():
            groups.setdefault(base, []).append(i)
        else:
            out[name] = rows[:, i]
    for base, idx in groups.items():
        out[base] = rows[:, idx]
    if all(k in out for k in ('tau_grav', 'tau_pid', 'tau_ff')):
        out['tau_cmd'] = out['tau_grav'] + out['tau_pid'] + out['tau_ff']
    out['columns'], out['note'] = names, note
    return out


def percentiles(x, scale=1.0):
    p = np.percentile(x, [50, 99, 99.9, 100]) * scale
    return 'median %.3f, 99%% %.3f, 99.9%% %.3f, worst %.3f' % tuple(p)


def summary(path):
    log = load(path)
    n = len(log['t'])
    print('%s\n  %s' % (path, log['note']))
    if n < 2:
        print('  %d rows' % n)
        return
    t = log['t']
    print('  %d rows, %.1f s; recorder dropped %d rows' % (n, t[-1] - t[0], int(log['rec_dropped'][-1])))
    cycles = np.diff(log['cycle'])
    if np.any(cycles != 1):
        print('  cycles missing from the file: %d (dropped rows)' % int((cycles - 1).sum()))
    dt = log['dt'][1:] * 1e3
    print('  control period (ms): %s' % percentiles(dt))
    print('    cycles later than 2.5 ms: %d (%.3f %%); later than 4 ms: %d' % (
        (dt > 2.5).sum(), 100.0 * (dt > 2.5).mean(), (dt > 4.0).sum()))
    alive = log['link_alive'] > 0
    if alive.any():
        age = log['rx_age'][alive]
        if np.isfinite(age).any():
            print('  age of the other arm\'s sample when used (ms): %s' % percentiles(age[np.isfinite(age)], 1e3))
        else:
            print('  the other arm\'s packets are not timed (another computer, or a program without the new link)')
        new = log['rx_new'][alive] > 0
        gaps, run = [], 0
        for v in new:
            run = 0 if v else run + 1
            if run:
                gaps.append(run)
        print('  cycles without a new packet: %.2f %%, longest run %d; packets lost on the way: %d' % (
            100.0 * (~new).mean(), max(gaps) if gaps else 0, int(log['rx_lost'][-1])))
    else:
        print('  no packets from the other arm')
    state = log['state'].astype(int)
    print('  ' + ', '.join('%s %.1f s' % (STATES.get(s, s), (state == s).sum() * 0.002) for s in sorted(set(state))))
    mode = log['mode'].astype(int)
    if len(set(mode)) > 1:
        print('  modes: ' + ', '.join('%s %.1f s' % (MODES.get(m, m), (mode == m).sum() * 0.002) for m in sorted(set(mode))))
    if (log['send_ok'] == 0).any():
        print('  packets the kernel refused to send: %d' % int((log['send_ok'] == 0).sum()))
    linked = state == 1
    if linked.any() and 'ext' in log:
        print('  while linked, rms (J1-J4): own estimate %s Nm; other arm\'s as used %s Nm' % (
            np.round(np.sqrt((log['ext'][linked] ** 2).mean(axis=0)), 2),
            np.round(np.sqrt((log['ext_used'][linked] ** 2).mean(axis=0)), 2)))
    force_torque(log, linked)


def rms(x):
    return np.sqrt((x ** 2).mean(axis=0))


def force_torque(log, linked):
    """The follower's force/torque sensor (lib/ft_core.h), in either arm's recording."""
    if 'ft_new' in log and np.isfinite(log['ft_new']).any():  # the follower, started with --ft
        new, use = log['ft_new'] > 0, log['ft_use']
        gaps, run = [0], 0
        for v in new:
            run = 0 if v else run + 1
            gaps.append(run)
        print('  force/torque sensor: a reading in %.1f %% of the cycles, longest gap %d cycles; reading it took the '
              'loop at most %.0f us' % (100.0 * new.mean(), max(gaps), 1e6 * np.nanmax(log['ft_poll'])))
        print('    zeros taken: %d; in use %.1f s of %.1f s' % (int(np.nanmax(log['ft_zeros'])), (use > 0.99).sum() * 0.002,
                                                              len(use) * 0.002))
        flags = log['tx_flags'].astype(int)
        print('    sent to the leader as measured: J1-J4 %.1f s, wrist %.1f s' % ((flags & 2 > 0).sum() * 0.002,
                                                                             (flags & 4 > 0).sum() * 0.002))
        used = use > 0.99
        if used.any():
            f, n = np.linalg.norm(log['ft_f'][used], axis=1), np.linalg.norm(log['ft_n'][used], axis=1)
            print('    from the surroundings while in use: force median %.2f N, 95%% %.2f N; torque median %.3f Nm, '
                  '95%% %.3f Nm' % (np.median(f), np.percentile(f, 95), np.median(n), np.percentile(n, 95)))
            print('    as joint torques, rms: %s Nm' % np.round(rms(log['ft_tau'][used]), 3))
    if 'wrist_use' in log:  # the leader
        use = log['wrist_use']
        if (use > 0).any() or (log['wrist_deadband'] > 0).any():
            on = linked & (use > 0.99)
            print('  wrist: the follower\'s measured torques in use %.1f s; position deadband up to %.1f degrees, '
                  'open %.1f s' % (on.sum() * 0.002, np.degrees(log['wrist_deadband'].max()),
                                   (log['wrist_deadband'] > 0).sum() * 0.002))
            if on.any():
                print('    wrist force torque on J6-J8 while in use, rms %s Nm, largest %s Nm' % (
                    np.round(rms(log['wrist_ff'][on]), 3), np.round(np.abs(log['wrist_ff'][on]).max(axis=0), 3)))
        if linked.any() and 'tau_pid' in log and np.isfinite(log['tau_pid'][:, 5:8]).any():
            pid = log['tau_pid'][linked][:, 5:8]
            print('    position coupling\'s torque on J6-J8 while linked: median |.| %s Nm, 90%% %s Nm' % (
                np.round(np.median(np.abs(pid), axis=0), 3), np.round(np.percentile(np.abs(pid), 90, axis=0), 3)))


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('files', nargs='+')
    p.add_argument('--npz', help='write the first file\'s signals to this .npz')
    p.add_argument('--csv', help='write the first file\'s columns to this .csv')
    a = p.parse_args()
    if a.npz:
        log = load(a.files[0])
        np.savez_compressed(a.npz, **{k: v for k, v in log.items() if isinstance(v, np.ndarray)},
                            columns=np.array(log['columns']), note=np.array(log['note']))
        print('wrote', a.npz)
    if a.csv:
        names, rows, _ = read(a.files[0])
        np.savetxt(a.csv, rows, delimiter=',', header=','.join(names), comments='', fmt='%.10g')
        print('wrote', a.csv)
    if not a.npz and not a.csv:
        for f in a.files:
            summary(f)
    return 0


if __name__ == '__main__':
    sys.exit(main())
