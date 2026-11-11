"""Identification of the WAM's J1-J4 dynamics with cable-drive friction.

The model (src/lib/cable_drive_model.h runs it in the controller):

    tau = W(q, dq, ddq) beta                rigid body, physically consistent
        + TM^T diag(I) TM ddq               motor rotors
        + TM^T f(TM q) + f_J2,J3(q)         Jenkins elements of the cable drive
        + TM^T diag(v) TM dq                viscous, at the motors
        + offset

fitted in one convex program (ident.py): pseudo-inertia LMIs on the four
links, nonnegative element forces (a passive friction model), an optional
log-det prior toward CAD. See ../README.md.
"""
