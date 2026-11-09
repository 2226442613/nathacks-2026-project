"""Run on laptop: python3 pc_test.py <ESP32_IP>.
Both laptop and ESP32 must be on the same reachable Wi-Fi network.
Type commands manually or use demo ground/wall.
"""
import socket
import sys
import threading
import time

HOST = sys.argv[1] if len(sys.argv) > 1 else input('ESP32 IP: ').strip()
PORT = 3333

with socket.create_connection((HOST, PORT), timeout=5) as sock:
    sock.settimeout(None)
    def receive():
        try:
            for line in sock.makefile('r', encoding='utf-8'):
                print('ESP32:', line.rstrip())
        except OSError:
            pass
    threading.Thread(target=receive, daemon=True).start()
    def send(cmd):
        sock.sendall((cmd + '\n').encode('utf-8'))
        print('PC:', cmd)
    print('Commands: TARGET,GROUND,BLOOD | TARGET,WALL,OTHER | MOVE,F,100 | ARRIVED | STOP | STATUS')
    print('Demo: demo-ground / demo-wall. Motors require repeated MOVE commands every <450ms.')
    try:
        while True:
            command = input('> ').strip()
            if command in ('exit', 'quit'): break
            if command in ('demo-ground', 'demo-wall'):
                surface, fluid = ('GROUND', 'BLOOD') if command == 'demo-ground' else ('WALL', 'OTHER')
                send(f'TARGET,{surface},{fluid}')
                # Deliberately do NOT move in demos: testing in-place only.
                time.sleep(0.3)
                send('ARRIVED')
            elif command:
                send(command)
    except (KeyboardInterrupt, EOFError):
        pass
    finally:
        try: send('STOP')
        except OSError: pass
