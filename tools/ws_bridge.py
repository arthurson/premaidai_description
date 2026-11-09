#!/usr/bin/env python3
"""WebSocket <-> serial bridge for the PreMaid AI web simulator (Raspberry Pi, PC, anything with pyserial).

Safari / iOS have no Web Serial. Run this next to the robot, open the simulator over http from it and use the
"WS" row of the Robot tab. Binary WebSocket frames go to the serial port unchanged, bytes read from the port are
sent back to every connected page as binary frames. The repository is also served over http, so one command is enough:

    pip install aiohttp pyserial
    python tools/ws_bridge.py /dev/ttyUSB0 --host 0.0.0.0        # then open http://<pi>:8765/web/premaid-sim.html

There is NO authentication: anyone who can reach the port can move the robot. The default host is 127.0.0.1;
use --host 0.0.0.0 only on a network you trust. An https page can only open wss:// (or ws://localhost); serve this
over http (as above) or put a TLS proxy in front of it.
"""
import argparse, asyncio, os, sys, threading

import serial
from aiohttp import web, WSMsgType


def make_app(ser, static_root=None):
    clients = set()
    wlock = threading.Lock()
    state = {'running': True}

    async def broadcast(data):
        for ws in list(clients):
            try:
                await ws.send_bytes(data)
            except Exception:
                clients.discard(ws)

    def reader(loop):
        while state['running']:
            try:
                data = ser.read(ser.in_waiting or 1)      # returns as soon as the first byte arrives
            except Exception as e:                        # port unplugged
                print('serial read failed:', e, file=sys.stderr); break
            if data:
                asyncio.run_coroutine_threadsafe(broadcast(data), loop)

    async def ws_handler(request):
        ws = web.WebSocketResponse(heartbeat=20)
        await ws.prepare(request)
        clients.add(ws)
        try:
            async for msg in ws:
                if msg.type == WSMsgType.BINARY:
                    with wlock:
                        ser.write(msg.data)
        finally:
            clients.discard(ws)
        return ws

    async def on_start(app):
        t = threading.Thread(target=reader, args=(asyncio.get_running_loop(),), daemon=True)
        t.start(); app['reader'] = t

    async def on_stop(app):
        state['running'] = False

    app = web.Application()
    app.router.add_get('/ws', ws_handler)
    if static_root:
        app.router.add_static('/', static_root, show_index=True)
    app.on_startup.append(on_start); app.on_cleanup.append(on_stop)
    return app


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('serial', help='serial port, e.g. /dev/ttyUSB0 or COM5')
    ap.add_argument('--baud', type=int, default=115200)
    ap.add_argument('--host', default='127.0.0.1')
    ap.add_argument('--port', type=int, default=8765)
    ap.add_argument('--no-static', action='store_true', help='do not serve the repository over http')
    a = ap.parse_args()
    ser = serial.Serial(a.serial, a.baud, timeout=0.05)
    root = None if a.no_static else os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    print('serial %s @ %d  ->  ws://%s:%d/ws%s' % (a.serial, a.baud, a.host, a.port, '' if a.no_static else '   page: http://%s:%d/web/premaid-sim.html' % (a.host, a.port)))
    web.run_app(make_app(ser, root), host=a.host, port=a.port, print=None)


if __name__ == '__main__':
    main()
