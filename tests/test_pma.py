#!/usr/bin/env python3
"""Unit tests for the .pma parser / loop expansion / 0x19 handling.

    python -m unittest discover -s tests -v
"""
import os
import sys
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'pma'))
import make_index as mi       # noqa: E402


def packet(cmd, payload):
    """[LEN][CMD][payload...][XOR] with LEN including the LEN and XOR bytes."""
    body = [2 + len(payload), cmd] + list(payload)
    x = 0
    for b in body:
        x ^= b
    return body + [x]


def frame(pid, raw, spd=10):
    return packet(0x18, [0x00, spd, pid, raw & 0xFF, (raw >> 8) & 0xFF])


def loop_start(iterations, ident=0):
    return packet(0x02, [0x00, 0x06, ident, iterations, 0x00])


def loop_end(ident=0):
    return packet(0x07, [0x02, 0x06, ident, 0x01, 0x00])


def parse(*packets, expand=True):
    return mi.parse_section(bytes(sum(packets, [])), expand=expand)


PID = 0x02
IDX = mi.PID2IDX[PID]


class PmaTests(unittest.TestCase):
    def test_loop_expands_from_previous_keyframe(self):
        p = parse(frame(PID, 8000), frame(PID, 8100), loop_start(3), frame(PID, 8200), loop_end(), frame(PID, 8300))
        self.assertEqual(p['bad'], 0)
        self.assertEqual(len(p['sourceFrames']), 4)
        self.assertEqual([f['raw'][IDX] for f in p['frames']], [8000, 8100, 8200, 8100, 8200, 8100, 8200, 8300])
        self.assertEqual(len(p['loops']), 1)
        self.assertEqual(p['loops'][0]['iterations'], 3)

    def test_loop_once_keeps_source_frames(self):
        p = parse(frame(PID, 8000), frame(PID, 8100), loop_start(3), frame(PID, 8200), loop_end(), expand=False)
        self.assertEqual(len(p['frames']), 3)
        self.assertEqual(len(p['sourceFrames']), 3)
        self.assertEqual(p['loops'][0]['iterations'], 3)

    def test_nested_loops(self):
        p = parse(
            frame(PID, 8000), frame(PID, 8100),
            loop_start(2),
            frame(PID, 8200),
            loop_start(3),
            frame(PID, 8300),
            loop_end(),
            frame(PID, 8400),
            loop_end(),
        )
        self.assertEqual(len(p['sourceFrames']), 5)
        self.assertEqual(len(p['frames']), 17)
        self.assertEqual([f['raw'][IDX] for f in p['frames']],
                         [8000, 8100, 8200, 8300, 8200, 8300, 8200, 8300, 8400,
                          8100, 8200, 8300, 8200, 8300, 8200, 8300, 8400])
        self.assertEqual(max(lp['depth'] for lp in p['loops']), 2)

    def test_019_speed_and_stretch_apply_to_following_frames(self):
        p = parse(
            frame(PID, 8000),
            packet(0x19, [0x00, PID, 30]),
            frame(PID, 8100),
            packet(0x19, [0x10, PID, 15]),
            frame(PID, 8200),
        )
        self.assertEqual([f['speed'][IDX] for f in p['frames']], [0, 30, 30])
        self.assertEqual([f['stretch'][IDX] for f in p['frames']], [60, 60, 15])
        self.assertIsNone(mi.range_of(p['sourceFrames'], 'speed', ignore_zero=True))
        self.assertEqual(mi.range_of(p['sourceFrames'], 'stretch'), (15, 60))

    def test_duration_is_max_spd_and_travel(self):
        p = parse(frame(PID, 7500), frame(PID, 7501))
        times, durations, total = mi.timeline(p['frames'])
        self.assertEqual(times, [0.0, 150.0])
        self.assertAlmostEqual(durations[0], 150.0)
        self.assertAlmostEqual(durations[1], 150.0)
        self.assertAlmostEqual(total, 300.0)

        p = parse(frame(PID, 7500), frame(PID, 7700, spd=1))
        _, durations, total = mi.timeline(p['frames'])
        self.assertAlmostEqual(durations[1], 200 / mi.PULSE_PER_MS)   # travel dominates
        self.assertAlmostEqual(total, 15 + 200 / mi.PULSE_PER_MS)

    def test_bad_checksum_is_counted_but_still_parsed(self):
        bad = frame(PID, 8000)
        bad[-1] ^= 0xFF
        p = parse(bad)
        self.assertEqual(p['bad'], 1)
        self.assertEqual(p['frames'][0]['raw'][IDX], 8000)

    def test_bpm_estimate_is_in_range(self):
        p = parse(*([frame(PID, 8000 + i, spd=8) for i in range(20)]))
        bpm = mi.estimate_bpm(p['sourceFrames'])
        self.assertGreaterEqual(bpm, 90)
        self.assertLessEqual(bpm, 200)


if __name__ == '__main__':
    unittest.main()
