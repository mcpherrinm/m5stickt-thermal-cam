#!/usr/bin/env python3
"""Log the camera's battery once a minute until it stops answering.

Connects to the camera's WebSocket, reads the header of the first message
(see README: battery % at offset 2, battery mV at 4, flags at 1), and
disconnects. Appends a CSV row each minute; stops after the camera has been
unreachable for 20 minutes in a row. Minutes count from the first row in the
file, so a restarted logger continues the same timeline. Standard library only.

    python3 tools/battery_log.py 192.168.0.211 battery-discharge.csv
    python3 tools/battery_log.py 192.168.0.211 battery-charge.csv charge

In "charge" mode it also stops once charging has finished (5 readings in a
row on external power but not charging).
"""
import base64, csv, os, socket, struct, sys, time
from datetime import datetime

HOST, OUT = sys.argv[1], sys.argv[2]
CHARGE_MODE = len(sys.argv) > 3 and sys.argv[3] == 'charge'
INTERVAL_S, GIVE_UP_AFTER, CHARGED_AFTER = 60, 20, 5


def read_exact(sock, n):
    buf = b''
    while len(buf) < n:
        chunk = sock.recv(n - len(buf))
        if not chunk:
            raise ConnectionError('connection closed')
        buf += chunk
    return buf


def read_header():
    """Returns (pct, mv, flags) from the first WebSocket message."""
    with socket.create_connection((HOST, 80), timeout=10) as s:
        key = base64.b64encode(os.urandom(16)).decode()
        s.sendall((f'GET /ws HTTP/1.1\r\nHost: {HOST}\r\nUpgrade: websocket\r\nConnection: Upgrade\r\n'
                   f'Sec-WebSocket-Key: {key}\r\nSec-WebSocket-Version: 13\r\n\r\n').encode())
        resp = b''
        while b'\r\n\r\n' not in resp:
            resp += read_exact(s, 1)
        if b' 101 ' not in resp.split(b'\r\n', 1)[0]:
            raise ConnectionError('no WebSocket upgrade')
        b0, b1 = read_exact(s, 2)
        n = b1 & 0x7F
        if n == 126:
            read_exact(s, 2)
        elif n == 127:
            read_exact(s, 8)
        head = read_exact(s, 20)
        flags, pct, _, mv = struct.unpack_from('<BBBH', head, 1)
        return pct, mv, flags


new = not os.path.exists(OUT) or os.path.getsize(OUT) == 0
start, failures, charged = time.time(), 0, 0
if not new:  # Continue the existing timeline.
    with open(OUT) as f:
        first = next(csv.DictReader(f), None)
    if first:
        start = datetime.fromisoformat(first['time']).timestamp()
with open(OUT, 'a', newline='') as f:
    w = csv.writer(f)
    if new:
        w.writerow(['time', 'minutes', 'battery_mv', 'battery_pct', 'charging', 'external_power', 'error'])
    while failures < GIVE_UP_AFTER and not (CHARGE_MODE and charged >= CHARGED_AFTER):
        t = time.time()
        row = [datetime.now().isoformat(timespec='seconds'), round((t - start) / 60, 1)]
        try:
            pct, mv, flags = read_header()
            row += [mv, pct, int(bool(flags & 2)), int(bool(flags & 4)), '']
            failures = 0
            charged = charged + 1 if flags & 4 and not flags & 2 else 0
        except Exception as e:
            row += ['', '', '', '', f'{type(e).__name__}: {e}']
            failures += 1
        w.writerow(row)
        f.flush()
        time.sleep(max(1, INTERVAL_S - (time.time() - t)))
    why = (f'charged: not charging for {CHARGED_AFTER} readings' if failures < GIVE_UP_AFTER
           else f'unreachable for {GIVE_UP_AFTER} minutes')
    w.writerow([datetime.now().isoformat(timespec='seconds'), round((time.time() - start) / 60, 1),
                '', '', '', '', f'stopped: {why}'])
