"""Local-only Module 07 transcript, never a physical ESP32 endpoint."""
import argparse
import io
from pathlib import Path
import time

from robot_comm import RobotClient,RobotSimulator,WireLogger


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--output',default='data/module07_demo')
    args=parser.parse_args()
    output=Path(args.output)
    output.mkdir(parents=True,exist_ok=False)
    server_log=WireLogger(output/'simulator.log',stream=io.StringIO())
    client_log=WireLogger(output/'client.log')
    try:
        with RobotSimulator(port=0,logger=server_log) as simulator:
            with RobotClient(port=simulator.port,logger=client_log) as client:
                for command in ['PING','STATUS','MOVE,F,80','STATUS','STOP','TARGET,WALL,OTHER','ARRIVED']:
                    client.send_command(command)
                time.sleep(2.8)
                client.request_status()
                client.emergency_stop()
                client.send_command('SAFETY')
                client.reset_estop()
                client.request_status()
        (output/'README.txt').write_text('SOFTWARE SIMULATION ONLY. TCP loopback on an ephemeral port.\n'
           'client.log and simulator.log contain exact ASCII wire lines.\n'
           'TX is transport acceptance, OK is parser/command acceptance, neither proves physical execution.\n'
           'The simulated wall sequence exercises lift/pump/sponge, then ESTOP aborts it.\n'
           'No hardware connection, navigation, plan execution or firmware upload occurred.\n')
    finally:
        client_log.close();server_log.close()


if __name__=='__main__':main()
