"""Wire lines logged only once, after successful sendall / complete ASCII reception."""
from datetime import datetime
import sys
import threading


class WireLogger:
    def __init__(self,path=None,stream=None):
        self.stream=sys.stdout if stream is None else stream
        self.file=open(path,'x',encoding='utf-8') if path else None
        self.lock=threading.Lock()

    def emit(self,label,line):
        # All TX/RX lines are already ASCII framed. Other messages must not contain secrets.
        stamp=datetime.now().strftime('%H:%M:%S.%f')[:-3]
        text=f'[{stamp}] [{label}] {line}\n'
        with self.lock:
            self.stream.write(text);self.stream.flush()
            if self.file:self.file.write(text);self.file.flush()

    def close(self):
        with self.lock:
            if self.file:self.file.close();self.file=None
