"""Fake PreMaid AI on a serial line (used by the bridge test and the browser test): answers 0x01 voltage / servo queries,
applies 0x18 poses, records every packet it receives. Framing: [LEN][CMD]...[XOR of all previous bytes]."""
import os, threading, time, select

PIDS = list(range(1, 0x1D)) + [0x1E, 0x1F, 0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27]


def xor(b):
    x = 0
    for v in b:
        x ^= v
    return x & 255


def pkt(*body):
    b = list(body)
    return bytes(b + [xor(b)])


class FakeRobot:
    def __init__(self, fd, pids, volts=11.1, dead=()):
        self.fd, self.pids, self.volts, self.dead = fd, set(pids), volts, set(dead)
        self.enc = {p: 7500 for p in pids}; self.cmd = {p: 7500 for p in pids}
        self.rx = []; self.run = True; self.buf = bytearray()
        self.th = threading.Thread(target=self.loop, daemon=True); self.th.start()

    def send(self, b):
        os.write(self.fd, b)

    def servo_reply(self, pid):
        valid = pid in self.pids and pid not in self.dead
        e = self.enc.get(pid, 0) if valid else 0; c = self.cmd.get(pid, 0) if valid else 0
        body = [pid, 0x80 if valid else 0, e & 255, e >> 8, 0, 0, c & 255, c >> 8, 0xFF, 0xFF, 0, 0, 100 if valid else 0, 60 if valid else 0]
        return pkt(18, 1, 0, *body)

    def handle(self, p):
        self.rx.append(bytes(p)); cmd = p[1]
        if cmd == 0x01 and p[3] == 2:
            raw = int(self.volts * 216 + 0.5); self.send(pkt(6, 1, 0, raw & 255, raw >> 8))
        elif cmd == 0x01 and p[3] == 5:
            self.send(self.servo_reply(p[4]))
        elif cmd == 0x18:
            for i in range(4, len(p) - 1, 3):
                pid, v = p[i], p[i + 1] | (p[i + 2] << 8)
                if pid in self.pids:
                    self.cmd[pid] = v
                    if v:
                        self.enc[pid] = v
            self.send(pkt(4, 0x18, 0))
        elif cmd == 0x1F:
            self.send(pkt(4, 0x1F, 0))

    def loop(self):
        while self.run:
            r, _, _ = select.select([self.fd], [], [], 0.05)
            if not r:
                continue
            try:
                self.buf += os.read(self.fd, 256)
            except OSError:
                return
            while self.buf:
                L = self.buf[0]
                if L < 4:
                    del self.buf[0]; continue
                if len(self.buf) < L:
                    break
                p = bytes(self.buf[:L]); del self.buf[:L]
                if xor(p[:-1]) == p[-1]:
                    self.handle(p)

    def stop(self):
        self.run = False
