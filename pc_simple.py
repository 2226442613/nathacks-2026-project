"""Direct TCP console, no heartbeat/reset/arming. Run: python3 pc_simple.py ESP32_IP"""
import socket
import sys
import threading


def main():
    if len(sys.argv) != 2:
        print('Usage: python3 pc_simple.py ESP32_IP')
        return
    with socket.create_connection((sys.argv[1], 3333), timeout=5) as sock:
        sock.settimeout(None)
        disconnected = threading.Event()

        def receive():
            try:
                with sock.makefile('r', encoding='ascii') as stream:
                    for line in stream:
                        print('\nESP32:', line.rstrip(), flush=True)
            except OSError as exc:
                print('\nConnection error:', exc, flush=True)
            finally:
                disconnected.set()
                print('\nDisconnected; no automatic reconnect or replay.', flush=True)

        threading.Thread(target=receive, daemon=True).start()
        print('Connected. No heartbeat/reset/arming required.')
        print('Commands: PING | STATUS | MOVE,F,100 | MOVE,R,1500 | STOP | quit')
        print('Boot/STOP returns servos to 180 degrees. Start with wheels lifted.')
        try:
            while not disconnected.is_set():
                command = input('> ').strip().upper()
                if command in {'QUIT', 'EXIT'}:
                    break
                if not command:
                    continue
                if len(command) > 120 or any(ord(c) < 32 or ord(c) > 126 for c in command):
                    print('Use a single ASCII command, max 120 characters.')
                    continue
                sock.sendall((command + '\n').encode('ascii'))
        except (KeyboardInterrupt, EOFError, OSError):
            pass
        finally:
            try:
                sock.sendall(b'STOP\n')
                sock.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass


if __name__ == '__main__':
    main()
