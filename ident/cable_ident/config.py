"""A small libconfig reader (enough for libbarrett's arm files: groups, lists,
arrays, numbers, strings, booleans, @include, # // /* */ comments) and the
calibrated gravity model it describes (cdlbt/calgrav.c)."""
import os
import re

import numpy as np

_TOKEN = re.compile(r'''
    (?P<ws>\s+) |
    (?P<comment>\#[^\n]*|//[^\n]*|/\*.*?\*/) |
    (?P<include>@include\s+"[^"]*") |
    (?P<str>"(?:[^"\\]|\\.)*") |
    (?P<num>[-+]?(?:0[xX][0-9a-fA-F]+|(?:\d+\.\d*|\.\d+|\d+)(?:[eE][-+]?\d+)?)L?) |
    (?P<bool>\b(?:true|false|TRUE|FALSE|True|False)\b) |
    (?P<name>[A-Za-z*][-A-Za-z0-9_*]*) |
    (?P<punct>[{}()\[\]=:;,])
''', re.X | re.S)


def _tokens(path):
    text = open(path).read()
    base = os.path.dirname(os.path.abspath(path))
    pos = 0
    while pos < len(text):
        m = _TOKEN.match(text, pos)
        if not m:
            raise ValueError('%s: cannot parse near %r' % (path, text[pos:pos + 40]))
        pos = m.end()
        kind = m.lastgroup
        if kind in ('ws', 'comment'):
            continue
        if kind == 'include':
            inc = re.search(r'"([^"]*)"', m.group()).group(1)
            for t in _tokens(os.path.join(base, inc)):
                yield t
            continue
        yield kind, m.group()


class _Parser:
    def __init__(self, path):
        self.t = list(_tokens(path))
        self.i = 0

    def peek(self):
        return self.t[self.i] if self.i < len(self.t) else (None, None)

    def take(self, value=None):
        tok = self.peek()
        if value is not None and tok[1] != value:
            raise ValueError('expected %r, got %r' % (value, tok[1]))
        self.i += 1
        return tok

    def settings(self, end=None):
        out = {}
        while self.peek()[1] is not None and self.peek()[1] != end:
            kind, name = self.take()
            if kind != 'name':
                raise ValueError('expected a setting name, got %r' % name)
            self.take()  # = or :
            out[name] = self.value()
            if self.peek()[1] in (';', ','):
                self.take()
        return out

    def value(self):
        kind, v = self.peek()
        if v == '{':
            self.take('{')
            g = self.settings('}')
            self.take('}')
            return g
        if v in ('(', '['):
            close = ')' if v == '(' else ']'
            self.take()
            items = []
            while self.peek()[1] != close:
                items.append(self.value())
                if self.peek()[1] == ',':
                    self.take()
            self.take(close)
            return items
        self.take()
        if kind == 'num':
            v = v.rstrip('L')
            return int(v, 0) if re.fullmatch(r'[-+]?(0[xX][0-9a-fA-F]+|\d+)', v) else float(v)
        if kind == 'str':
            return v[1:-1]
        if kind == 'bool':
            return v.lower() == 'true'
        raise ValueError('unexpected %r' % v)


def read(path):
    return _Parser(path).settings()


class Gravity:
    """libbarrett's calibrated gravity model of one arm (kinematics.moving and
    gravity_compensation.mus of its configuration group)."""
    G_WORLD = np.array([0.0, 0.0, -9.805])

    def __init__(self, arm_group):
        moving = arm_group['kinematics']['moving']
        self.alpha = np.array([float(l['alpha_pi']) * np.pi for l in moving])
        self.theta = np.array([float(l.get('theta_pi', 0.0)) * np.pi for l in moving])
        self.mu = np.array(arm_group['gravity_compensation']['mus'], dtype=float)
        self.dof = len(moving)
        assert self.mu.shape == (self.dof, 3)

    @staticmethod
    def _rot(theta, alpha):
        ct, st, ca, sa = np.cos(theta), np.sin(theta), np.cos(alpha), np.sin(alpha)
        R = np.zeros(theta.shape + (3, 3))
        R[..., 0, 0] = ct; R[..., 1, 0] = st
        R[..., 0, 1] = -st * ca; R[..., 1, 1] = ct * ca; R[..., 2, 1] = sa
        R[..., 0, 2] = st * sa; R[..., 1, 2] = -ct * sa; R[..., 2, 2] = ca
        return R

    def torque(self, q):
        q = np.atleast_2d(q)
        n = q.shape[0]
        Rp = [self._rot(q[:, j] + self.theta[j], np.full(n, self.alpha[j])) for j in range(self.dof)]
        Rw, R = [], np.broadcast_to(np.eye(3), (n, 3, 3))
        for j in range(self.dof):
            R = R @ Rp[j]
            Rw.append(R)
        tau = np.zeros((n, self.dof))
        t_next = None
        for j in range(self.dof - 1, -1, -1):
            g = np.einsum('nij,i->nj', Rw[j], self.G_WORLD)
            t = np.cross(g, self.mu[j][None, :])
            if t_next is not None:
                t = t + np.einsum('nij,nj->ni', Rp[j + 1], t_next)
            tau[:, j] = np.einsum('nij,nj->ni', Rp[j], t)[:, 2]
            t_next = t
        return tau

    def wrist_correction(self, q, first=4):
        """g(q) - g(q with joints first.. at 0), J1-J4."""
        q = np.atleast_2d(q)
        q0 = q.copy()
        q0[:, first:] = 0.0
        return (self.torque(q) - self.torque(q0))[:, :4]
