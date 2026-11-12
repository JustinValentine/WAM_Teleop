#!/usr/bin/env python3
"""A Robotiq 2F-140 made up for tests: a pseudo-terminal that answers Modbus RTU
the way the gripper does, so that libgripper and robotiq_teleop run against it
unchanged, on a computer with no gripper.

The registers are laid out as the 2F-85 & 2F-140 instruction manual gives them
(4.3, 4.4, 4.7) and as libgripper's branch robotiq (2ddb4c7) packs them:

    0x03E8  action request (rACT bit 0, rGTO bit 3, rATR bit 4) | reserved
    0x03E9  reserved | position request (0 open ... 255 closed)
    0x03EA  speed | force
    0x07D0  gripper status (gACT bit 0, gGTO bit 3, gSTA bits 4-5, gOBJ bits 6-7) | reserved
    0x07D1  fault status | position request echo
    0x07D2  position | motor current

    read  (function 3 or 4):  id fn addr_hi addr_lo n_hi n_lo crc_lo crc_hi
          answer:             id fn 2n data... crc_lo crc_hi
    write (function 16):      id 10 addr_hi addr_lo n_hi n_lo 2n data... crc_lo crc_hi
          answer:             id 10 addr_hi addr_lo n_hi n_lo crc_lo crc_hi

`robotiq_sim.py --selftest` runs the manual's own Modbus RTU example (4.7.5)
through it and compares the answers byte for byte.

What the model does, and where that is only a guess about the real gripper:
  - A position request moves the fingers at 30 to 250 mm/s (speed 0 to 255),
    on the straight line 0 = 140 mm, 255 = 0 mm that libgripper uses. A new
    request in mid-move takes effect at once, speed included (guess).
  - The fingers stop at the "software limit" short of count 0, as in the
    manual's example (count 13, reported as "at requested position"), and
    short of count 255 where they touch each other (count 230 and "contact
    while closing": guesses).
  - An object (--object MM, or `object MM` typed here) stops them while
    closing: gOBJ 2 once they stand, 3 if they reach the request. With the
    request left beyond the object they follow it when it is taken away
    (the manual's re-grasp).
  - Activation (rACT 0 -> 1) closes them fully, then opens them fully, and only
    then reports gSTA 3; it lets go of an object on the way (the manual only
    says the gripper moves; which way first is a guess).
  - rACT = 0 resets: activation gone, faults cleared.
  - After `power` (off and on) the gripper is in reset and needs rACT cleared
    and set, as the manual says; a request with rACT set before that gets
    fault 0x07.
  - Major faults (`fault 14`) stop the fingers until a reset. With
    `commfault S`, a second without a request sets fault 0x09, which clears
    S seconds after requests come again (that it clears by itself is a guess).
  - `unplug` takes the serial device away (the pseudo-terminal is closed and
    its link removed); `plug` brings it back. `mute` leaves the device and
    cuts the line behind it: the gripper hears and answers nothing, and goes
    on to its last request. `garble N` breaks the checksum of the next N
    answers. `late` puts a status answer on the line that no request is
    waiting for: libgripper then pairs every request with the answer to the
    one before, until the port is closed (which empties the line, here as on
    a real serial port) and opened again.

Alone:   robotiq_sim.py [--link /tmp/robotiq_sim] [--object 60]
         then start the node with port:=/tmp/robotiq_sim and type commands
         here (`help` lists them).
In a test: test/robotiq_e2e.py imports SerialGripper.
"""

import argparse
import os
import select
import sys
import termios
import threading
import time
import tty

STROKE = 140.0  # mm at count 0, as libgripper's kRobotiqMaxWidthMm
COMMAND_BASE = 0x03E8
STATUS_BASE = 0x07D0


def crc16(data):
    """Modbus RTU's CRC; sent low byte first."""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return crc


def with_crc(frame):
    crc = crc16(frame)
    return bytes(frame) + bytes([crc & 0xFF, crc >> 8])


def width_of(count):
    return (1.0 - count / 255.0) * STROKE


def count_of(width):
    return int(round((1.0 - max(0.0, min(STROKE, width)) / STROKE) * 255.0))


class Gripper:
    """The gripper behind the serial line: registers and fingers."""

    def __init__(self):
        self.open_count = 13      # the software limit at the open end (the manual's example)
        self.closed_count = 230   # where the fingers touch each other
        self.activation_speed = 150.0  # mm/s through the travel in an activation
        self.object = None        # mm: something this wide between the fingers
        self.comm_fault_clears = None  # s; None: no fault 0x09 at all
        self.width = width_of(self.open_count)
        self.command = bytearray(6)
        self.act = self.gto = False
        self.must_clear = False   # after a power cycle
        self.request = 0
        self.speed = self.force = 0
        self.sta = self.obj = self.fault = 0
        self.activating = 0       # 1 closing, 2 opening
        self.activations = 0      # activation routines started
        self.resets = 0           # rACT cleared while it was set
        self.position_requests = []  # every position request taken with rGTO set, in order
        self.silent_for = 0.0     # s since the last request
        self.talking_for = 0.0    # s of requests since the last silence
        self.log = lambda text: None

    # ---- the fingers ----

    def limits(self):
        return width_of(self.closed_count), width_of(self.open_count)

    def advance(self, dt):
        lowest, highest = self.limits()
        self.silent_for += dt
        if self.silent_for >= 1.0:
            self.talking_for = 0.0
            if self.comm_fault_clears is not None and self.act and self.fault == 0:
                self.fault = 0x09
                self.log("fault 0x09: no request for a second")
        if self.activating == 1:
            stop = self.object if self.object is not None and self.object <= self.width else lowest
            self.width = max(stop, self.width - self.activation_speed * dt)
            if self.width <= stop:
                self.activating = 2
        elif self.activating == 2:
            self.width = min(highest, self.width + self.activation_speed * dt)
            if self.width >= highest:
                self.activating = 0
                self.sta = 3
                self.obj = 0
                self.log("activation completed")
        elif self.act and self.sta == 3 and self.gto and self.fault in (0, 0x09):
            goal = width_of(self.request)
            v = 30.0 + self.speed / 255.0 * 220.0
            if goal < self.width - 1e-9:
                stop = max(goal, lowest)
                if self.object is not None and self.object <= self.width + 1e-9:
                    stop = max(stop, self.object)
                self.width = max(stop, self.width - v * dt)
                if self.width > stop:
                    new = 0
                else:
                    new = 2 if stop > goal else 3
                if new == 2 and self.obj != 2:
                    self.log("contact while closing at %.1f mm" % self.width)
                self.obj = new
            elif goal > self.width + 1e-9:
                stop = min(goal, highest)
                self.width = min(stop, self.width + v * dt)
                # at the software limit the manual's example reports "at requested position"
                self.obj = 0 if self.width < stop else 3
            elif self.obj == 0:
                self.obj = 3

    def position(self):
        return count_of(self.width)

    def pressing(self):
        """Stopped on something with the request beyond it: the grip has its force."""
        return self.act and self.gto and self.obj == 2 and width_of(self.request) < self.width

    # ---- the registers ----

    def status_bytes(self):
        status = (1 if self.act else 0) | ((1 if self.gto else 0) << 3) | (self.sta << 4) | (self.obj << 6)
        moving = self.activating or (self.gto and self.obj == 0 and self.sta == 3)
        return bytes([status, 0, self.fault, self.request, self.position(), 0x20 if moving else 0])

    def heard(self):
        """A request arrived, whatever it was."""
        self.silent_for = 0.0

    def tick_talking(self, dt):
        self.talking_for += dt
        if self.fault == 0x09 and self.comm_fault_clears is not None and self.talking_for >= self.comm_fault_clears:
            self.fault = 0
            self.log("fault 0x09 cleared: requests again")

    def write_command(self, offset, data):
        for i, byte in enumerate(data):
            if 0 <= offset + i < 6:
                self.command[offset + i] = byte
        act = bool(self.command[0] & 0x01)
        gto = bool(self.command[0] & 0x08)
        if not act:
            if self.act or self.fault:
                self.resets += 1
                self.log("reset (rACT 0)")
            self.act = self.gto = False
            self.sta = self.obj = self.fault = 0
            self.activating = 0
            self.must_clear = False
            return
        if not self.act:
            if self.must_clear:
                self.fault = 0x07
                self.log("rACT set after a power cycle without clearing it first: fault 0x07")
                return
            self.act = True
            self.sta = 1
            self.obj = 0
            self.activating = 1
            self.activations += 1
            self.gto = False
            self.log("activation started (the fingers close, then open)")
        if self.activating:
            return  # the manual: an action waits until the activation is completed
        if self.fault not in (0, 0x09):
            return
        self.gto = gto
        self.request, self.speed, self.force = self.command[3], self.command[4], self.command[5]
        if gto:
            self.position_requests.append(self.request)
            self.obj = 0 if abs(width_of(self.request) - self.width) > 1e-9 else self.obj

    def power_cycle(self):
        self.act = self.gto = False
        self.sta = self.obj = self.fault = 0
        self.activating = 0
        self.must_clear = True
        self.log("power off and on: in reset")

    def set_fault(self, code):
        self.fault = code
        self.log("fault 0x%02X" % code)

    # ---- Modbus ----

    def answer(self, frame, slave=9):
        """The answer to one complete request frame, or None (not for this
        gripper, a bad checksum, a function it does not have: the manual says
        it sends no exception responses)."""
        if len(frame) < 8 or frame[0] != slave:
            return None
        if crc16(frame[:-2]) != (frame[-2] | frame[-1] << 8):
            return None
        fn = frame[1]
        addr = frame[2] << 8 | frame[3]
        n = frame[4] << 8 | frame[5]
        self.heard()
        if fn in (0x03, 0x04):
            if STATUS_BASE <= addr and addr + n <= STATUS_BASE + 3:
                data = self.status_bytes()[2 * (addr - STATUS_BASE):2 * (addr - STATUS_BASE + n)]
            elif COMMAND_BASE <= addr and addr + n <= COMMAND_BASE + 3:
                data = bytes(self.command[2 * (addr - COMMAND_BASE):2 * (addr - COMMAND_BASE + n)])
            else:
                return None
            return with_crc(bytes([slave, fn, len(data)]) + data)
        if fn == 0x10:
            count = frame[6]
            if count != 2 * n or len(frame) != 9 + count or not (COMMAND_BASE <= addr and addr + n <= COMMAND_BASE + 3):
                return None
            self.write_command(2 * (addr - COMMAND_BASE), frame[7:7 + count])
            return with_crc(frame[:6])
        return None


def frame_length(buf):
    """How long the request at the start of buf is, 0 if that is not known
    yet, -1 if it is no request this gripper has."""
    if len(buf) < 2:
        return 0
    if buf[1] in (0x03, 0x04):
        return 8
    if buf[1] == 0x10:
        return 9 + buf[6] if len(buf) >= 7 else 0
    return -1


class SerialGripper:
    """The gripper on a pseudo-terminal, served from its own thread. `link` is
    a path the serial device is linked at (the node's port)."""

    def __init__(self, link, slave=9, verbose=False):
        self.link = link
        self.slave = slave
        self.gripper = Gripper()
        if verbose:
            self.gripper.log = lambda text: print("  [sim] " + text, flush=True)
        self.lock = threading.RLock()
        self.master = None
        self.muted = False
        self.garble = 0          # answers still to break
        self.reply_delay = 0.002
        self.requests = 0        # complete requests for this gripper
        self.writes = 0
        self.port_opened = 0     # times a program opened the device
        self.device = None       # the pseudo-terminal's own name
        self.running = False
        self.thread = None
        self._buf = bytearray()
        self._buf_at = 0.0
        self._open = False

    def start(self):
        self.plug()
        self.running = True
        self.thread = threading.Thread(target=self._serve, daemon=True)
        self.thread.start()

    def stop(self):
        self.running = False
        if self.thread:
            self.thread.join()
        self.unplug()

    def plug(self):
        """The serial device appears: a new pseudo-terminal behind the link."""
        with self.lock:
            if self.master is not None:
                return
            master, slave = os.openpty()
            tty.setraw(slave)
            self.device = os.ttyname(slave)
            os.close(slave)
            try:
                os.unlink(self.link)
            except FileNotFoundError:
                pass
            os.symlink(self.device, self.link)
            self.master = master
            self._buf = bytearray()
            self._open = False

    def unplug(self):
        """The serial device goes away under whoever has it open."""
        with self.lock:
            if self.master is None:
                return
            try:
                os.unlink(self.link)
            except FileNotFoundError:
                pass
            os.close(self.master)
            self.master = None

    def _serve(self):
        last = time.monotonic()
        while self.running:
            with self.lock:
                master = self.master
            readable = False
            if master is None:
                time.sleep(0.002)
            else:
                poller = select.poll()
                poller.register(master, select.POLLIN)
                events = poller.poll(2)
                readable = any(e & select.POLLIN for _, e in events)
                hung_up = any(e & select.POLLHUP for _, e in events)
                if hung_up and not readable:
                    time.sleep(0.002)  # nobody has the device open
                with self.lock:
                    # A device nobody has open reports a hang-up.
                    if self._open and hung_up:
                        self._open = False
                        self._empty_line()
                    elif not self._open and not hung_up and self.master == master:
                        self._open = True
                        self.port_opened += 1
            now = time.monotonic()
            with self.lock:
                dt, last = now - last, now
                self.gripper.advance(dt)  # muted too: the line is cut, the gripper has its power
                self.gripper.tick_talking(dt)
                if readable and self.master == master:
                    self._read(master, now)
                elif self._buf and now - self._buf_at > 0.05:
                    self._buf = bytearray()  # half a frame and then nothing: dropped

    def _empty_line(self):
        # A serial port drops what is still on its line when the program
        # closes it. A pseudo-terminal keeps it for the next program that
        # opens it, as long as this end stays open; so it is emptied here.
        try:
            fd = os.open(self.device, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
            termios.tcflush(fd, termios.TCIOFLUSH)
            os.close(fd)
        except OSError:
            pass
        self._buf = bytearray()

    def inject(self, data):
        """Bytes on the line that answer nothing: an answer that came too late
        for the request it belonged to."""
        with self.lock:
            if self.master is not None:
                os.write(self.master, bytes(data))

    def _read(self, master, now):
        try:
            data = os.read(master, 256)
        except OSError:
            return
        if not data:
            return
        self._buf += data
        self._buf_at = now
        while True:
            need = frame_length(self._buf)
            if need < 0:
                self._buf = bytearray()
                return
            if need == 0 or len(self._buf) < need:
                return
            frame, self._buf = bytes(self._buf[:need]), self._buf[need:]
            if self.muted:
                continue
            reply = self.gripper.answer(frame, self.slave)
            if reply is None:
                continue
            self.requests += 1
            self.writes += 1 if frame[1] == 0x10 else 0
            if self.garble > 0:
                self.garble -= 1
                reply = reply[:-1] + bytes([reply[-1] ^ 0xFF])
            time.sleep(self.reply_delay)
            try:
                os.write(master, reply)
            except OSError:
                return

    # ---- for a test: everything under the lock ----

    def get(self, name):
        with self.lock:
            value = getattr(self.gripper, name)
            return value() if callable(value) else value

    def set(self, **values):
        with self.lock:
            for name, value in values.items():
                setattr(self.gripper, name, value)

    def call(self, name, *args):
        with self.lock:
            return getattr(self.gripper, name)(*args)

    def snapshot(self):
        with self.lock:
            g = self.gripper
            return dict(width=g.width, position=g.position(), request=g.request, requested_mm=width_of(g.request),
                        act=g.act, gto=g.gto, sta=g.sta, obj=g.obj, fault=g.fault, activations=g.activations,
                        resets=g.resets, pressing=g.pressing(), speed=g.speed, force=g.force,
                        requests=self.requests, writes=self.writes, port_opened=self.port_opened,
                        position_requests=len(g.position_requests))


def selftest():
    """The manual's Modbus RTU example (4.7.5), frame by frame."""
    checks = failures = 0

    def check(ok, what):
        nonlocal checks, failures
        checks += 1
        failures += 0 if ok else 1
        print("  %s %s" % ("ok  " if ok else "FAIL", what))

    def frame(text):
        return bytes(int(b, 16) for b in text.split())

    def run(g, seconds):
        for _ in range(int(round(seconds / 0.005))):
            g.advance(0.005)

    for text in ("09 04 07 D0 00 02 70 0E", "09 04 04 E0 00 00 00 45 84", "09 10 03 E9 00 02 04 60 E6 3C C8 EC 7C",
                 "09 10 03 E9 00 02 91 30", "09 04 06 39 00 00 FF 0E 0A B6 6D", "09 04 06 39 00 00 00 BB 10 71 06"):
        f = frame(text)
        check(crc16(f[:-2]) == (f[-2] | f[-1] << 8), "the checksum of the manual's frame " + text)

    g = Gripper()
    g.object = width_of(189) + 0.1  # the manual's object: the fingers stop at position 189
    ack = frame("09 10 03 E8 00 03 01 30")
    check(g.answer(frame("09 10 03 E8 00 03 06 00 00 00 00 00 00 73 30")) == ack, "step 1, clear rACT: the manual's answer")
    check(g.answer(frame("09 10 03 E8 00 03 06 01 00 00 00 00 00 72 E1")) == ack, "step 1, set rACT: the manual's answer")
    run(g, 0.2)
    check(g.answer(frame("09 04 07 D0 00 01 30 0F")) == frame("09 04 02 11 00 54 A1"),
          "step 2, status during the activation: 09 04 02 11 00 54 A1")
    run(g, 5.0)
    check(g.answer(frame("09 04 07 D0 00 01 30 0F")) == frame("09 04 02 31 00 4D 61"),
          "step 2, status after it: 09 04 02 31 00 4D 61")
    check(g.answer(frame("09 10 03 E8 00 03 06 09 00 00 FF FF FF 42 29")) == ack, "step 4, close at full speed and force: the manual's answer")
    run(g, 0.1)
    moving = g.answer(frame("09 04 07 D0 00 03 B1 CE"))
    check(moving[:7] == frame("09 04 06 39 00 00 FF"), "step 5, status on the way: 39 00 00 FF, then the position and current of that moment")
    run(g, 2.0)
    check(g.answer(frame("09 04 07 D0 00 03 B1 CE")) == frame("09 04 06 B9 00 00 FF BD 00 5C 9A"),
          "step 5, stopped on the object: 09 04 06 B9 00 00 FF BD 00 5C 9A")
    check(g.answer(frame("09 10 03 E8 00 03 06 09 00 00 00 FF FF 72 19")) == ack, "step 7, open: the manual's answer")
    run(g, 2.0)
    check(g.answer(frame("09 04 07 D0 00 03 B1 CE")) == frame("09 04 06 F9 00 00 00 0D 00 17 AA"),
          "step 8, open, at the software limit: 09 04 06 F9 00 00 00 0D 00 17 AA")
    # the read libgripper sends: function 3, three registers
    reply = g.answer(with_crc(frame("09 03 07 D0 00 03")))
    check(reply == with_crc(frame("09 03 06 F9 00 00 00 0D 00")), "the same status read with function 3, as libgripper asks")
    check(g.answer(with_crc(frame("08 03 07 D0 00 03"))) is None and g.answer(frame("09 04 07 D0 00 03 B1 CF")) is None
          and g.answer(with_crc(frame("09 2B 07 D0 00 03"))) is None,
          "another slave's request, a bad checksum, an unknown function: no answer")
    print("%d checks, %d failed" % (checks, failures))
    return failures == 0


HELP = """  object MM | noobject     something MM wide between the fingers / take it away
  fault CODE              a fault, e.g. fault 14 (0x0E overcurrent); a reset clears it
  power                   the gripper off and on: in reset
  commfault S | nocommfault   fault 0x09 after a second without requests, clearing S s after they resume
  unplug | plug           the serial device away / back
  mute | unmute           the device stays, the line to the gripper is cut
  garble N                break the checksum of the next N answers
  late                    an answer too late for its request: from then on every answer is one behind
  status                  where it is
  quit"""


def main():
    parser = argparse.ArgumentParser(description="A simulated Robotiq 2F-140 on a pseudo-terminal (Modbus RTU).")
    parser.add_argument("--link", default="/tmp/robotiq_sim", help="where the serial device is linked: the node's port")
    parser.add_argument("--object", type=float, help="mm: something this wide between the fingers")
    parser.add_argument("--selftest", action="store_true", help="check the frames against the manual's example and exit")
    args = parser.parse_args()
    if args.selftest:
        sys.exit(0 if selftest() else 1)

    sim = SerialGripper(args.link, verbose=True)
    sim.set(object=args.object)
    sim.start()
    print("Simulated Robotiq 2F-140 on %s (-> %s). `help` lists the commands." % (args.link, os.path.realpath(args.link)), flush=True)
    try:
        for line in sys.stdin:
            words = line.split()
            if not words:
                continue
            cmd, arg = words[0], (words[1] if len(words) > 1 else None)
            if cmd == "quit":
                break
            elif cmd == "object" and arg:
                sim.set(object=float(arg))
            elif cmd == "noobject":
                sim.set(object=None)
            elif cmd == "fault" and arg:
                sim.call("set_fault", int(arg, 0))
            elif cmd == "power":
                sim.call("power_cycle")
            elif cmd == "commfault" and arg:
                sim.set(comm_fault_clears=float(arg))
            elif cmd == "nocommfault":
                sim.set(comm_fault_clears=None)
            elif cmd == "unplug":
                sim.unplug()
            elif cmd == "plug":
                sim.plug()
            elif cmd in ("mute", "unmute"):
                sim.muted = cmd == "mute"
            elif cmd == "garble" and arg:
                sim.garble = int(arg)
            elif cmd == "late":
                sim.inject(with_crc(bytes([sim.slave, 0x03, 6]) + sim.call("status_bytes")))
            elif cmd == "status":
                print("  " + ", ".join("%s %s" % (k, ("%.1f" % v) if isinstance(v, float) else v)
                                       for k, v in sim.snapshot().items()), flush=True)
            else:
                print(HELP, flush=True)
    except KeyboardInterrupt:
        pass
    sim.stop()


if __name__ == "__main__":
    main()
