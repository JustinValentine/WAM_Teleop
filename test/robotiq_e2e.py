#!/usr/bin/env python3
"""The Robotiq gripper's teleop end to end, without a gripper or a controller:
joystick messages -> robotiq_teleop (the real node, started by the real launch
file) -> libgripper's RobotiqGripper -> Modbus RTU on a serial device ->
the simulated 2F-140 of robotiq_sim.py behind a pseudo-terminal.

Needs a workspace built with a libgripper that has the Robotiq classes (its
branch robotiq) and the joy package (the launch file's joy_node; it is given
a joystick device that does not exist, finds nothing and publishes nothing,
and this script plays the controller on /nav_controller_teleop/joy).

    roscore &
    python3 test/robotiq_e2e.py         (about two minutes)

or test/robotiq_e2e.sh, which starts roscore itself. What it goes through:
the start, where the first activation waits for L2 + circle; the stick, both
ways and at half deflection;
closing on an object and opening again; L2 + cross; the controller going
silent; a restart by the service and by L2 + circle; a fault and a gripper
that lost its power; the serial device disappearing and coming back, with the
gripper still activated and with it in reset; the line to the gripper cut;
answers with a bad checksum; an answer that comes too late and puts the
library's reads out of step; fault 0x09; the node killed and started again by
roslaunch while the gripper holds something; auto_activate and auto_restart. It prints the node's lines as they come ([pane]) and
"N checks, M failed" at the end; the exit status is 0 only if none failed.
"""

import os
import signal
import subprocess
import sys
import threading
import time

import rospy
from sensor_msgs.msg import JointState, Joy
from std_srvs.srv import Trigger

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from robotiq_sim import SerialGripper, width_of, with_crc  # noqa: E402

LINK = "/tmp/robotiq_e2e_port"
DEADMAN, CROSS, CIRCLE = 5, 0, 1   # test/robotiq_e2e.yaml, which the launch file is given below
LEAD = 5.0                         # mm, the same file (stick 50 mm/s, device speed 0.3, force 0.5)
CONFIG = os.path.join(os.path.dirname(os.path.abspath(__file__)), "robotiq_e2e.yaml")
COUNT = 140.0 / 255.0              # mm in one position count

checks = failures = 0


def check(ok, what):
    global checks, failures
    checks += 1
    failures += 0 if ok else 1
    print("  %s %s" % ("ok  " if ok else "FAIL", what), flush=True)


def wait_for(condition, timeout):
    """Seconds until condition() holds, or None after timeout."""
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout:
        if condition():
            return time.monotonic() - t0
        time.sleep(0.005)
    return None


class Pane:
    """What roslaunch and the node print: the gripper pane."""

    def __init__(self, args):
        self.lines = []
        self.lock = threading.Lock()
        self.process = subprocess.Popen(args, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, bufsize=1)
        threading.Thread(target=self._read, daemon=True).start()

    def _read(self):
        for line in self.process.stdout:
            line = line.rstrip()
            if "Gripper:" in line:
                print("       [pane] " + line, flush=True)
            with self.lock:
                self.lines.append(line)

    def mark(self):
        with self.lock:
            return len(self.lines)

    def count(self, text, since=0):
        with self.lock:
            return sum(1 for line in self.lines[since:] if text in line)

    def has(self, text, since=0):
        return self.count(text, since) > 0

    def wait(self, text, since=0, timeout=10.0):
        return wait_for(lambda: self.has(text, since), timeout) is not None

    def stop(self):
        self.process.send_signal(signal.SIGINT)
        try:
            self.process.wait(timeout=20)
        except subprocess.TimeoutExpired:
            self.process.kill()


class Controller:
    """The navigation controller as joy_node publishes it: 3 axes, 12 buttons,
    every 20 ms while it is on."""

    def __init__(self):
        self.pub = rospy.Publisher("/nav_controller_teleop/joy", Joy, queue_size=1)
        self.on = True
        self.l2 = self.cross = self.circle = False
        self.stick = 0.0
        threading.Thread(target=self._run, daemon=True).start()

    def _run(self):
        while not rospy.is_shutdown():
            if self.on:
                msg = Joy()
                msg.header.stamp = rospy.Time.now()
                msg.axes = [0.0, self.stick, 0.0]
                msg.buttons = [0] * 12
                msg.buttons[DEADMAN] = int(self.l2)
                msg.buttons[CROSS] = int(self.cross)
                msg.buttons[CIRCLE] = int(self.circle)
                self.pub.publish(msg)
            time.sleep(0.02)

    def hold(self, stick):
        self.l2, self.stick, self.cross, self.circle = True, stick, False, False

    def let_go(self):
        self.l2, self.stick, self.cross, self.circle = False, 0.0, False, False


def main():
    rospy.init_node("robotiq_e2e", disable_signals=True)
    sim = SerialGripper(LINK)
    sim.start()
    pane = Pane(["roslaunch", "wam_teleop", "robotiq_nav_controller.launch", "config:=" + CONFIG, "port:=" + LINK, "joy_dev:=/dev/no_joystick"])
    pad = Controller()
    opening = []
    rospy.Subscriber("/robotiq_teleop/joint_states", JointState, lambda m: opening.append(m.position[0]))
    restart = rospy.ServiceProxy("/robotiq_teleop/restart", Trigger)
    status = rospy.ServiceProxy("/robotiq_teleop/status", Trigger)
    open_mm = width_of(13)  # the simulated gripper's software limit at the open end

    def snap():
        return sim.snapshot()

    def speed_over(seconds):
        w0 = snap()["width"]
        t0 = time.monotonic()
        time.sleep(seconds)
        return (w0 - snap()["width"]) / (time.monotonic() - t0)

    def still_for(seconds):
        w0 = snap()["width"]
        time.sleep(seconds)
        return abs(snap()["width"] - w0) < 1e-6

    def restart_by_service():
        try:
            return restart()
        except rospy.ServiceException as e:
            return type("Failed", (), {"success": False, "message": "the call failed: %s" % e})()

    def to_ready(since, timeout=12.0):
        return pane.wait("ready at", since, timeout)

    try:
        print("1. The start: connecting; the first activation waits to be asked for", flush=True)
        ok = pane.wait("not activated yet", 0, 20.0) and pane.wait("navigation controller connected", 0, 5.0)
        pad.hold(1.0)
        time.sleep(2.0)  # the stick does nothing yet
        pad.let_go()
        s = snap()
        check(ok and s["activations"] == 0 and s["resets"] == 0 and s["writes"] == 0,
              "the node opens the port, finds the gripper not activated and sends it nothing, the stick held or not")
        check(pane.has("L2 + circle") and pane.has("wam gripper restart") and pane.has("once its fingers have room"),
              "the pane says how to activate it, and that its fingers will move")
        rospy.wait_for_service("/robotiq_teleop/status", 5.0)
        r = status()
        check(not r.success and "NOT USABLE: not activated" in r.message, "~status: " + r.message.split(";")[0])
        pad.l2 = pad.circle = True
        ok = pane.wait("RESTART (L2 + circle held", 0, 3.0) and pane.wait("restarted in", 0, 12.0)
        pad.let_go()
        s = snap()
        check(ok and s["activations"] == 1 and s["sta"] == 3, "L2 + circle held for a second: activated, once (%d)" % s["activations"])
        check(abs(s["width"] - open_mm) < 1e-6 and s["position_requests"] == 1,
              "the fingers are open (%.1f mm); the only position request so far is the library's own after its activation" % s["width"])
        time.sleep(0.3)
        check(bool(opening) and abs(opening[-1] - open_mm / 1000.0) < 1e-4, "~joint_states carries the opening in m (%.4f)" % (opening[-1] if opening else -1))
        r = status()
        check(r.success and "ready" in r.message, "~status: " + r.message)

        print("2. The stick", flush=True)
        pad.hold(1.0)
        time.sleep(0.4)
        v = speed_over(1.0)
        check(45.0 < v < 55.0, "L2 + full stick up closes at %.1f mm/s (50 set)" % v)
        s = snap()
        check(s["speed"] == 77 and s["force"] == 128, "the requests carry speed 0.3 and force 0.5 (%d, %d of 255)" % (s["speed"], s["force"]))
        pad.let_go()
        w0 = snap()["width"]
        time.sleep(0.3)
        writes = snap()["writes"]
        check(w0 - snap()["width"] < 4.0 and still_for(1.0) and snap()["writes"] == writes,
              "L2 released: the opening stays (%.1f mm further, then still), nothing more is sent" % (w0 - snap()["width"]))
        pad.hold(-(0.05 + 0.5 * 0.95))
        time.sleep(0.4)
        v = -speed_over(1.0)
        check(21.0 < v < 29.0, "half stick down opens at %.1f mm/s" % v)
        pad.l2 = False  # the stick stays down
        time.sleep(0.3)
        writes = snap()["writes"]
        check(still_for(1.0) and snap()["writes"] == writes, "the stick without L2: nothing")
        pad.let_go()

        print("3. Closing on an object", flush=True)
        sim.set(object=60.0)
        mark = pane.mark()
        pad.hold(1.0)
        wait_for(lambda: snap()["obj"] == 2, 4.0)
        time.sleep(1.0)  # the stick still held
        s = snap()
        past = s["width"] - s["requested_mm"]
        check(s["obj"] == 2 and abs(s["width"] - 60.0) < 1e-6, "the fingers stop on the object (%.1f mm), contact while closing" % s["width"])
        check(0.0 < past <= LEAD + COUNT and s["pressing"], "the request stays %.1f mm beyond the contact: the grip keeps its force" % past)
        check(pane.count("closing stopped by contact", mark) == 1, "the pane reports the contact, once")
        pad.let_go()
        time.sleep(1.0)
        check(snap()["request"] == s["request"] and snap()["pressing"], "L2 released: the request stays where it is")
        pad.hold(-1.0)
        t = wait_for(lambda: snap()["width"] > 60.0, 2.0)
        check(t is not None and t < 0.25, "stick down: the fingers open after %s ms" % ("%.0f" % (t * 1000) if t is not None else "more than 2000"))
        time.sleep(0.5)
        pad.hold(1.0)
        wait_for(lambda: snap()["obj"] == 2, 4.0)
        time.sleep(0.5)
        pad.let_go()
        time.sleep(0.3)
        mark = pane.mark()
        sim.set(object=None)
        time.sleep(1.0)
        s = snap()
        check(0.0 < 60.0 - s["width"] <= LEAD + COUNT and s["obj"] == 3,
              "the object taken away: the fingers follow by %.1f mm, to the request, and stop" % (60.0 - s["width"]))
        check(pane.has("no contact any more", mark), "the pane says the contact is lost")

        print("4. L2 + cross", flush=True)
        mark = pane.mark()
        before = snap()["position_requests"]
        pad.l2 = pad.cross = True
        time.sleep(1.5)  # held
        pad.let_go()
        wait_for(lambda: abs(snap()["width"] - open_mm) < 1e-6, 3.0)
        s = snap()
        check(abs(s["width"] - open_mm) < 1e-6 and s["position_requests"] == before + 1 and pane.count("opening fully", mark) == 1,
              "held for 1.5 s: fully open (%.1f mm) with one request" % s["width"])

        print("5. The controller goes silent", flush=True)
        mark = pane.mark()
        pad.hold(1.0)
        time.sleep(0.8)
        pad.on = False  # the stick was held up when the messages stopped
        t0 = time.monotonic()
        wait_for(lambda: pane.has("no joystick message", mark), 2.0)
        said = time.monotonic() - t0
        time.sleep(0.3)
        check(0.4 < said < 0.8 and still_for(1.0), "the fingers stop, %.2f s after the last message (0.5 s set)" % said)
        pad.let_go()
        pad.on = True
        check(pane.wait("navigation controller connected", mark, 3.0), "the controller back: the pane says so")

        print("6. A restart from another terminal (the service), holding an object", flush=True)
        sim.set(object=60.0)
        pad.hold(1.0)
        wait_for(lambda: snap()["obj"] == 2, 4.0)
        pad.let_go()
        time.sleep(0.3)
        before = snap()
        mark = pane.mark()
        t0 = time.monotonic()
        r = restart_by_service()
        took = time.monotonic() - t0
        s = snap()
        check(r.success and "restarted in" in r.message, "~restart answers after %.1f s: %s" % (took, r.message))
        check(s["activations"] == before["activations"] + 1 and s["resets"] > before["resets"] and s["port_opened"] == before["port_opened"] + 1,
              "the port was closed and opened, the gripper reset and activated once")
        check(pane.has("RESTART (the restart service", mark) and pane.has("whatever they hold falls", mark),
              "the pane says who asked, and that what is held falls")
        check(abs(s["width"] - open_mm) < 1e-6 and s["position_requests"] == before["position_requests"] + 1,
              "the fingers are open (the object was let go); nothing was asked of the gripper but the library's own request")
        writes = s["writes"]
        check(still_for(1.0) and snap()["writes"] == writes, "... and nothing is, until the stick moves")
        r = status()
        check(r.success and "2 restarts (the last one worked)" in r.message, "~status counts the restarts")
        sim.set(object=None)

        print("7. L2 + circle", flush=True)
        before = snap()["activations"]
        pad.l2 = pad.circle = True
        time.sleep(0.6)
        pad.let_go()
        time.sleep(1.0)
        check(snap()["activations"] == before, "held for 0.6 s: no restart")
        mark = pane.mark()
        pad.l2 = pad.circle = True
        ok = pane.wait("RESTART (L2 + circle held", mark, 2.0)
        ok = to_ready(mark) and ok
        time.sleep(1.0)  # still held
        pad.let_go()
        check(ok and snap()["activations"] == before + 1, "held on: one restart, and ready again")

        print("8. A fault", flush=True)
        pad.hold(1.0)
        time.sleep(0.5)
        mark = pane.mark()
        sim.call("set_fault", 0x0E)
        ok = pane.wait("fault 0x0E", mark, 2.0)
        writes = snap()["writes"]
        time.sleep(1.5)  # the stick still held
        check(ok and pane.has("overcurrent", mark) and pane.has("wam gripper restart", mark), "overcurrent (0x0E): the pane has the code, its meaning and how to restart")
        check(snap()["writes"] == writes, "nothing is sent to the gripper meanwhile, the stick held or not")
        r = status()
        check(not r.success and "NOT USABLE" in r.message and "0x0E" in r.message, "~status: " + r.message.split(";")[0])
        pad.let_go()
        r = restart_by_service()
        check(r.success and snap()["fault"] == 0, "a restart clears it")

        print("9. The gripper loses its power and comes back", flush=True)
        pad.hold(1.0)
        time.sleep(0.5)
        mark = pane.mark()
        before = snap()["activations"]
        sim.muted = True  # off for 0.3 s
        sim.call("power_cycle")
        time.sleep(0.3)
        sim.muted = False
        ok = pane.wait("not activated", mark, 2.0)
        time.sleep(1.5)  # the stick still held
        s = snap()
        check(ok and not s["act"] and s["fault"] == 0 and s["activations"] == before,
              "off for 0.3 s and back in reset: the pane says not activated; no request reached it (one would have set fault 0x07, "
              "or started an activation)")
        pad.let_go()
        mark = pane.mark()
        pad.l2 = pad.circle = True
        ok = to_ready(mark)
        pad.let_go()
        check(ok and snap()["sta"] == 3, "L2 + circle: activated and ready again")

        print("10. The serial device disappears and comes back", flush=True)
        pad.hold(1.0)
        time.sleep(1.0)
        pad.let_go()
        time.sleep(0.3)
        before = snap()
        mark = pane.mark()
        sim.unplug()
        ok = pane.wait("NOT ANSWERING", mark, 3.0) and pane.wait("serial port does not open", mark, 3.0)
        check(ok and pane.has("Its last status", mark), "the device gone: not answering, the port does not open, with the last status")
        time.sleep(3.0)
        n = pane.count("serial:", mark)
        check(1 <= n <= 8, "the library's serial errors reach the pane, thinned out (%d lines in 4 s)" % n)
        r = status()
        check(not r.success and "NOT ANSWERING" in r.message, "the node goes on: ~status says " + r.message.split(";")[0])
        r = restart_by_service()
        check(not r.success and "serial port does not open" in r.message, "a restart now fails and says why: " + r.message)
        sim.plug()
        ok = pane.wait("answers again and is still activated", mark, 4.0)
        s = snap()
        check(ok and s["activations"] == before["activations"] and abs(s["width"] - before["width"]) < 1e-6,
              "the device back: ready again without a restart, nothing moved")
        pad.hold(1.0)
        time.sleep(0.3)
        v = speed_over(0.6)
        pad.let_go()
        check(v > 40.0, "... and the stick works (%.0f mm/s)" % v)

        sim.unplug()
        sim.call("power_cycle")
        mark = pane.mark()
        pane.wait("NOT ANSWERING", mark, 3.0)
        sim.plug()
        ok = pane.wait("not activated", mark, 4.0)
        check(ok and snap()["activations"] == before["activations"], "back with the gripper in reset: the pane says so and waits")
        r = restart_by_service()
        check(r.success, "... and a restart makes it ready")

        print("11. The line to the gripper cut, broken answers, a late answer, fault 0x09", flush=True)
        before = snap()
        mark = pane.mark()
        sim.muted = True
        ok = pane.wait("NOT ANSWERING", mark, 3.0) and pane.wait("serial port is open", mark, 3.0)
        time.sleep(2.0)
        check(ok and pane.has("Timed out waiting for Modbus response", mark),
              "no answers on an open port: not answering, with the library's time-outs in the pane")
        sim.muted = False
        check(pane.wait("answers again and is still activated", mark, 4.0) and snap()["activations"] == before["activations"],
              "the answers back: ready, no restart")

        mark = pane.mark()
        sim.garble = 5
        pad.hold(-1.0)
        time.sleep(1.0)
        pad.let_go()
        check(sim.garble == 0 and not pane.has("NOT ANSWERING", mark) and status().success,
              "five answers with a bad checksum: lost, and the teleop carries on")

        pad.hold(1.0)
        time.sleep(1.0)
        pad.let_go()
        time.sleep(0.3)
        before = snap()
        mark = pane.mark()
        sim.inject(with_crc(bytes.fromhex("090306f90000000d00")))  # a whole status answer nobody is waiting for
        time.sleep(0.5)
        check(status().success and not pane.has("out of step", mark), "an answer too late for its request: nothing shows while only the status is read")
        pad.hold(-1.0)
        ok = pane.wait("out of step", mark, 3.0) and pane.wait("open again and the gripper is still activated", mark, 3.0)
        time.sleep(0.3)
        v = -speed_over(0.6)
        pad.let_go()
        s = snap()
        check(ok and s["port_opened"] == before["port_opened"] + 1 and s["activations"] == before["activations"],
              "the first position requests after it get no proper answer: the node closes and opens the port, no restart")
        check(v > 40.0, "... and the stick works again (%.0f mm/s)" % v)

        sim.set(comm_fault_clears=0.5)
        mark = pane.mark()
        sim.muted = True
        time.sleep(1.6)
        sim.muted = False
        ok = pane.wait("fault 0x09", mark, 4.0) and pane.wait("no fault any more", mark, 4.0)
        check(ok and pane.has("needs no restart", mark) and snap()["activations"] == before["activations"],
              "fault 0x09 after the silence: waited out, ready again without a restart")
        sim.set(comm_fault_clears=None)

        print("12. The node dies while the gripper holds something", flush=True)
        sim.set(object=60.0)
        pad.hold(1.0)
        wait_for(lambda: snap()["obj"] == 2, 5.0)
        pad.let_go()
        time.sleep(0.3)
        before = snap()
        mark = pane.mark()
        subprocess.run(["pkill", "-KILL", "-x", "robotiq_teleop"])
        ok = pane.wait("connected: the gripper is activated and ready", mark, 8.0)
        s = snap()
        check(ok and s["activations"] == before["activations"] and s["pressing"] and abs(s["width"] - 60.0) < 1e-6,
              "roslaunch starts it again; it takes the gripper as it is: no activation, the object still held")
        pad.hold(-1.0)
        t = wait_for(lambda: snap()["width"] > 60.0, 2.0)
        pad.let_go()
        check(t is not None, "... and the stick opens it")
        sim.set(object=None)

        print("13. auto_activate and auto_restart", flush=True)
        rospy.set_param("/robotiq_teleop/auto_activate", True)
        rospy.set_param("/robotiq_teleop/auto_restart", True)
        before = snap()["activations"]
        mark = pane.mark()
        sim.call("power_cycle")  # the next node finds it in reset
        subprocess.run(["pkill", "-KILL", "-x", "robotiq_teleop"])
        ok = pane.wait("RESTART (activation at the start, auto_activate)", mark, 8.0) and to_ready(mark)
        check(ok and snap()["activations"] == before + 1, "with auto_activate set, a node that starts on a gripper in reset activates it without asking")
        mark = pane.mark()
        sim.call("set_fault", 0x0E)
        ok = pane.wait("RESTART (automatic, auto_restart)", mark, 3.0) and to_ready(mark)
        check(ok and snap()["activations"] == before + 2 and snap()["fault"] == 0, "with auto_restart set, a fault restarts the gripper without asking")
    finally:
        pad.let_go()
        pane.stop()
        sim.stop()
    check(pane.has("stopped. Nothing was sent to the gripper at the end"), "at the end the node leaves the gripper as it is")
    print("%d checks, %d failed" % (checks, failures), flush=True)
    return 0 if failures == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
