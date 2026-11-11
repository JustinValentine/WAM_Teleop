"""The controller's causal signals, discretised exactly as libbarrett does
(math::FirstOrderFilter: backward Euler), and the evaluation filter."""
import numpy as np
import scipy.signal as ss


def first_order(x, a, b, c, T):
    """libbarrett FirstOrderFilter, H(s) = (a s + b) / (s + c)."""
    den = 1.0 + c * T
    c1, c2, c3 = 1.0 / den, (a + b * T) / den, a / den
    # y[n] = c1 y[n-1] + c2 x[n] - c3 x[n-1]
    x = np.asarray(x, dtype=float)
    xp = np.vstack([x[:1], x[:-1]])
    return ss.lfilter([1.0], [1.0, -c1], c2 * x - c3 * xp, axis=0)


def libbarrett_velocity(q, T, omega=180.0):
    """Backward difference, then joint_velocity_filter (low-pass 180 rad/s)."""
    dq = np.vstack([np.zeros((1, q.shape[1])), np.diff(q, axis=0) / T])
    return first_order(dq, 0.0, omega, omega, T)


def controller_acceleration(dq, T):
    """leader_wam8g.cpp / follower.cpp: hp1 = 25 s / (s + 25) on jv, then a
    50 rad/s low-pass (jaFilter)."""
    return first_order(first_order(dq, 25.0, 0.0, 25.0, T), 0.0, 50.0, 50.0, T)


class Filt:
    """Evaluation / fitting filter: causal Butterworth on both sides of
    tau = Phi theta (it commutes with the linear model), so torque noise above
    the force channel's band does not drive the fit."""

    def __init__(self, fc=10.0, order=2):
        self.fc, self.order = fc, order

    def __call__(self, x, T):
        b, a = ss.butter(self.order, self.fc * 2 * T)
        return ss.lfilter(b, a, x, axis=0)
