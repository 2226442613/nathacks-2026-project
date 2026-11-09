"""Standalone communication CLI; imports no camera/vision/OpenCV code."""
import argparse
import time

from .client import RobotClient,CommunicationError
from .logging import WireLogger
from .simulator import RobotSimulator


def add_communication_commands(commands):
    sim=commands.add_parser('robot-simulator',help='Local-only inspected ESP32 text protocol simulator')
    sim.add_argument('--host',default='127.0.0.1');sim.add_argument('--port',type=int,default=3333)
    sim.add_argument('--log-file');sim.set_defaults(headless=True)
    sim.add_argument('--firmware-profile',choices=['safe_v1_1','simple'],default='safe_v1_1')
    client=commands.add_parser('robot-client',help='Plain-text ESP32 test client; defaults to local simulation')
    client.add_argument('--mode',choices=['simulation','physical'],default='simulation')
    client.add_argument('--firmware-profile',choices=['safe_v1_1','simple'],default='safe_v1_1')
    client.add_argument('--host');client.add_argument('--port',type=int,default=3333)
    client.add_argument('--command',dest='wire_commands',action='append',help='Repeatable ASCII command; MOVE,F,1000 means 1000 ms at fixed PWM170')
    client.add_argument('--listen',type=float,default=.5,help='Seconds to display incoming telemetry after commands')
    client.add_argument('--timeout',type=float,default=.8)
    client.add_argument('--heartbeat-interval',type=float,default=.2)
    client.add_argument('--log-file');client.add_argument('--demo',action='store_true',help='Simulation-only movement and cleaning protocol exercise')
    client.add_argument('--arm',action='store_true');client.add_argument('--authorize-motion',action='store_true')
    client.add_argument('--validated-max-pwm',type=int)
    client.add_argument('--validated-max-duration-ms',type=int,help='Separate operator-validated single MOVE limit, 1..9999 ms')
    client.add_argument('--wait-move',action='store_true',help='Wait for DONE,MOVE while heartbeats continue; no arrival/settling claim')
    client.set_defaults(headless=True)


def run_communication_command(args):
    logger=WireLogger(args.log_file)
    try:
        if args.command=='robot-simulator':
            simulator=RobotSimulator(args.host,args.port,logger,firmware_profile=args.firmware_profile)
            try:simulator.serve_forever()
            finally:simulator.close()
            return
        if args.listen<0:raise ValueError('--listen must be nonnegative')
        if args.demo and args.mode!='simulation':raise ValueError('--demo is simulation only')
        # argparse's repeatable --command must not overwrite the subcommand destination.
        commands=args.wire_commands or ['PING','STATUS']
        with RobotClient(args.host,args.port,mode=args.mode,timeout=args.timeout,
                         heartbeat_interval=args.heartbeat_interval,logger=logger,firmware_profile=args.firmware_profile) as client:
            if args.arm:client.arm_physical(validated_max_pwm=args.validated_max_pwm,
                validated_max_duration_ms=args.validated_max_duration_ms,authorized=args.authorize_motion)
            if args.demo:commands=['MOVE,F,80','STATUS','STOP','TARGET,WALL,OTHER','ARRIVED','STATUS','ESTOP','SAFETY','RESET_ESTOP','STATUS']
            if args.demo and args.firmware_profile=='simple':commands=['MOVE,F,80','STATUS','STOP','TARGET,WALL,OTHER','ARRIVED','STATUS','STOP']
            for line in commands:
                result=client.send_command(line)
                if result.firmware_accepted is False:logger.emit('WARN','Controller rejected command; no execution assumed')
                if line.startswith('MOVE,') and result.firmware_accepted and (args.wait_move or args.demo):
                    result=client.wait_for_move(result)
                    logger.emit('INFO','MOVE result='+result.execution_state+'; not arrival or camera settling')
                    if not result.execution_completed:raise CommunicationError('MOVE '+result.execution_state)
                if args.demo and line=='ARRIVED':time.sleep(2.8)  # Simulated wall lift, pump, then sponge sweep.
                if line in {'STOP','ESTOP'}:time.sleep(.03)  # Allow asynchronous safety replies; no command replay.
            deadline=time.monotonic()+args.listen
            while time.monotonic()<deadline:client.receive_telemetry(min(.1,max(0,deadline-time.monotonic())))
    finally:logger.close()


def main(argv=None):
    parser=argparse.ArgumentParser(description='Hospital Guardian Module 07; no OpenCV dependency')
    commands=parser.add_subparsers(dest='command',required=True)
    add_communication_commands(commands)
    args=parser.parse_args(argv)
    try:run_communication_command(args);return 0
    except KeyboardInterrupt:return 0
    except (OSError,ValueError,PermissionError,CommunicationError) as exc:
        print(f'[ERROR] {exc}',flush=True);return 1
