"""Duration-based MOVE contract; fixed motor power is never a wire parameter."""
import re

MAX_LINE = 120
FIXED_PWM = 170
PWM_FREQUENCY_HZ = 20000
MAX_MOVE_MS = 10000
CAPS = 'CAPS,NATHACK_V1_1,MOVE_DURATION_MS,FIXED_PWM_170,STOP,ESTOP,RESET_ESTOP,HEARTBEAT,SAFETY,TARGET,ARRIVED'
REQUIRED_CAPABILITIES = frozenset(CAPS.split(',')[1:])
SIMPLE_CAPS = 'CAPS,NATHACK_V2_SIMPLE,MOVE_DURATION_MS,FIXED_PWM_170,STOP,TARGET,ARRIVED'
STATES = {'IDLE','APPROACH','SPRAY_LIFT','SPRAYING','SPRAY_LOWER','GROUND_FORWARD','GROUND_BACKWARD','WALL_WIPE','FINISHED'}


class ProtocolError(ValueError):
    pass


class LineFramer:
    def __init__(self):
        self.buffer = bytearray()

    def feed(self, chunk, on_line=None):
        lines = []
        for value in chunk:
            if value == 10:
                raw = bytes(self.buffer);self.buffer.clear()
                if raw.endswith(b'\r'):raw=raw[:-1]
                if not raw or any(v<32 or v>126 for v in raw):
                    raise ProtocolError('Invalid ASCII line encoding')
                line=raw.decode('ascii')
                lines.append(line)
                if on_line:on_line(line)
            else:
                if value != 13 and (value<32 or value>126):
                    raise ProtocolError('Invalid ASCII line encoding')
                self.buffer.append(value)
                if len(self.buffer)>MAX_LINE+(1 if self.buffer[-1]==13 else 0):
                    raise ProtocolError('Line exceeds 120 bytes')
        return lines


def validate_command(line):
    if not isinstance(line,str) or not line or len(line)>MAX_LINE or any(ord(c)<32 or ord(c)>126 for c in line):
        raise ProtocolError('Use one ASCII command without newline, maximum 120 bytes')
    if line in {'PING','STATUS','STOP','ABORT','CAPS','SAFETY','HEARTBEAT','ESTOP','RESET_ESTOP','ARRIVED'}:
        return line
    if re.fullmatch(r'MOVE,[FBLR],[0-9]+',line):
        duration = line.split(',')[2]
        if len(duration) <= 5 and 1 <= int(duration) <= MAX_MOVE_MS:return line
        raise ProtocolError('MOVE duration must be a positive integer, 1..10000 milliseconds (at fixed PWM170)')
    if re.fullmatch(r'TARGET,(GROUND|WALL),(BLOOD|OTHER)',line):return line
    raise ProtocolError('Unsupported command or invalid parameter/unit')


def expected_reply(command,line):
    if line.startswith('ERR,'):return True
    verb=command.split(',')[0]
    if verb=='PING':return line=='PONG'
    if verb=='STATUS':return line.startswith('STATUS,') and line[7:] in STATES
    if verb=='CAPS':return line.startswith('CAPS,')
    if verb=='SAFETY':return line.startswith('SAFETY,')
    return line==('OK,STOP' if verb=='ABORT' else 'OK,'+verb)


def parse_safety(line):
    match=re.fullmatch(r'SAFETY,ESTOP=([01]),WATCHDOG_MS=1000,WATCHDOG=(OK|EXPIRED)',line or '')
    if not match:raise ProtocolError('Invalid or unsupported controller safety report')
    return {'estop':match[1]=='1','watchdog_expired':match[2]=='EXPIRED'}
