import asyncio, os, pty, sys, tty, unittest
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__))); sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'tools'))
try:
    import aiohttp, serial
    from aiohttp import web
    from aiohttp.test_utils import TestServer, TestClient
    import ws_bridge
except ImportError:                                  # pip install aiohttp pyserial
    aiohttp = None
from fake_robot import FakeRobot, pkt, PIDS


@unittest.skipIf(aiohttp is None, 'aiohttp / pyserial not installed')
class BridgeTest(unittest.TestCase):
    def test_roundtrip(self):
        master, slave = pty.openpty(); tty.setraw(master); tty.setraw(slave)
        robot = FakeRobot(master, PIDS, volts=11.1, dead=[0x1E])
        ser = serial.Serial(os.ttyname(slave), 115200, timeout=0.05)

        async def run():
            server = TestServer(ws_bridge.make_app(ser, None)); client = TestClient(server); await client.start_server()
            try:
                ws = await client.ws_connect('/ws')
                async def read(n):
                    buf = b''
                    while len(buf) < n:
                        m = await asyncio.wait_for(ws.receive(), 3); buf += m.data
                    return buf
                await ws.send_bytes(bytes.fromhex('07 01 00 02 00 02 06'))
                v = await read(6); self.assertEqual(v[:3], bytes([6, 1, 0])); self.assertAlmostEqual((v[3] | v[4] << 8) / 216, 11.1, 2)
                await ws.send_bytes(pkt(7, 1, 0, 5, 0x0B, 14))
                r = await read(18); self.assertEqual(r[3], 0x0B); self.assertEqual(r[4], 0x80); self.assertEqual(r[5] | r[6] << 8, 7500)
                await ws.send_bytes(pkt(8, 0x18, 0, 3, 0x0B, 0x34, 0x21))     # pid 0x0B -> 0x2134
                self.assertEqual(await read(4), pkt(4, 0x18, 0))
                self.assertEqual(robot.cmd[0x0B], 0x2134)
                await ws.send_bytes(pkt(7, 1, 0, 5, 0x1E, 14)); d = await read(18); self.assertEqual(d[4], 0)   # unwired servo: flag 0
                await ws.close()
            finally:
                await client.close()
        try:
            asyncio.run(run())
        finally:
            robot.stop(); ser.close()


if __name__ == '__main__':
    unittest.main()
