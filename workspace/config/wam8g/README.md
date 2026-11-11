# wam8g configuration (from opus, 2026-09-23)

Copied unchanged from opus's `~/.barrett` (the calibrated WAM8: zerocal home
and zero angles, the 15:38 gravitycal fit, `gravity_calibrated = true`).
Programs select it with `BARRETT_CONFIG_FILE=/home/user/wam_ros/config/wam8g/default.conf`
(the demos' `run_demo.py --robot canopen-wam8 --config config/wam8g/default.conf`
does this).

Before connecting the arm to thor: `default.conf` still says `bus.port = 1`
(`can1`, as on opus). Set it to the number of the CAN interface the WAM's
PCAN-USB gets on thor. thor's udev rules name the adapters by hub port:
1-6.4.1 -> can10, 1-6.4.2 -> can20; other ports get the kernel's canN.
