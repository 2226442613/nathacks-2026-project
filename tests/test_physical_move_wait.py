"""Real-time physical client semantics with an injected in-memory socket boundary."""
from queue import Queue,Empty
import socket
import threading
import time
from types import SimpleNamespace

import pytest

from robot_comm import CommunicationError,RobotClient,LineFramer
from robot_comm.protocol import SIMPLE_CAPS
from test_robot_communication import quiet,standard_reply,wait_until


class Endpoint:
    """No listening port or network: feed bytes through the production reader."""
    def __init__(self,response):
        self.response=response;self.lines=[];self.port=3333;self.timeout=.05
        self.incoming=Queue();self.framer=LineFramer();self.closed=False
        self.conn=SimpleNamespace(sendall=self.incoming.put)
    def sendall(self,data):
        if self.closed:raise OSError('closed test transport')
        for line in self.framer.feed(data):
            self.lines.append(line);reply=self.response(line)
            if reply:self.incoming.put(reply)
    def recv(self,size):
        try:return self.incoming.get(timeout=self.timeout)
        except Empty:raise socket.timeout()
    def settimeout(self,value):self.timeout=value
    def shutdown(self,*args):self.close()
    def close(self):self.closed=True;self.incoming.put(b'')
    def __enter__(self):
        self.patch=pytest.MonkeyPatch()
        def connect(address,timeout):
            assert address==('127.0.0.1',self.port)
            return self
        self.patch.setattr(socket,'create_connection',connect)
        return self
    def __exit__(self,*args):self.close();self.patch.undo()


def simple_reply(line):
    if line=='CAPS':return (SIMPLE_CAPS+'\n').encode('ascii')
    return standard_reply(line)


def physical_client(endpoint,**options):
    # Exercise physical-mode code without ever addressing a robot or camera.
    return RobotClient('127.0.0.1',endpoint.port,mode='physical',
        firmware_profile='simple',logger=quiet(),**options)


def start_wait(client,result,check=None):
    outcomes=[];errors=[];finished=threading.Event()
    def run():
        try:outcomes.append(client.wait_for_move(result,check=check))
        except BaseException as exc:errors.append(exc)
        finally:finished.set()
    worker=threading.Thread(target=run)
    worker.start()
    return worker,finished,outcomes,errors


def test_physical_move_waits_real_duration_and_done_while_monitoring():
    duration_ms=200;callbacks=[];done_sent=threading.Event();timers=[]
    def response(line):
        if line.startswith('MOVE,'):
            def complete():
                try:endpoint.conn.sendall(b'DONE,MOVE\n');done_sent.set()
                except OSError:pass
            timer=threading.Timer(duration_ms/1000+.04,complete)
            timer.start();timers.append(timer)
            return b'OK,MOVE\n'
        return simple_reply(line)
    with Endpoint(response) as endpoint:
        with physical_client(endpoint,timeout=.5) as client:
            started=time.monotonic();accepted=client.send_command(f'MOVE,F,{duration_ms}')
            assert accepted.firmware_accepted and accepted.execution_state=='accepted'
            assert not accepted.execution_completed and not done_sent.is_set()
            worker,finished,outcomes,errors=start_wait(client,accepted,lambda:callbacks.append(time.monotonic()))
            try:
                assert not finished.wait(.08)
                assert finished.wait(2)
            finally:worker.join(2)
            assert not errors and len(outcomes)==1 and outcomes[0].execution_completed
            assert outcomes[0].execution_state=='completed' and done_sent.is_set()
            assert time.monotonic()-started>=duration_ms/1000
            assert len(callbacks)>=2 and endpoint.lines.count(f'MOVE,F,{duration_ms}')==1
            assert '[RX] OK,MOVE' in client.logger.stream.getvalue()
            assert '[RX] DONE,MOVE' in client.logger.stream.getvalue()
    for timer in timers:timer.join(1)


def test_physical_elapsed_duration_without_done_does_not_complete():
    with Endpoint(simple_reply) as endpoint:
        with physical_client(endpoint,timeout=.5) as client:
            accepted=client.send_command('MOVE,R,60')
            worker,finished,outcomes,errors=start_wait(client,accepted)
            try:
                assert not finished.wait(.15)  # More than the requested motor duration.
                assert client.get_move_result(accepted).execution_state=='accepted'
                endpoint.conn.sendall(b'DONE,MOVE\n')
                assert finished.wait(1)
            finally:worker.join(2)
            assert not errors and outcomes[0].execution_state=='completed'
            assert endpoint.lines.count('MOVE,R,60')==1


def test_physical_premature_done_faults_without_replay():
    def response(line):
        return b'OK,MOVE\nDONE,MOVE\n' if line.startswith('MOVE,') else simple_reply(line)
    with Endpoint(response) as endpoint:
        with physical_client(endpoint,timeout=.2) as client:
            try:
                accepted=client.send_command('MOVE,F,500')
                outcome=client.wait_for_move(accepted)
                assert outcome.execution_state=='unknown' and not outcome.execution_completed
            except CommunicationError:pass
            wait_until(lambda:not client.is_connected())
            assert client.estop_latched and not client.armed
            assert client.get_move_result().execution_state=='unknown'
            with pytest.raises(CommunicationError):client.send_command('MOVE,F,500')
            assert endpoint.lines.count('MOVE,F,500')==1


def test_physical_missing_done_is_unknown_locked_and_never_replayed():
    with Endpoint(simple_reply) as endpoint:
        with physical_client(endpoint,timeout=.1) as client:
            accepted=client.send_command('MOVE,B,50')
            outcome=client.wait_for_move(accepted)
            assert accepted.execution_state=='accepted' and accepted.firmware_accepted
            assert outcome.execution_state=='unknown' and not outcome.execution_completed
            assert not client.is_connected() and client.estop_latched
            with pytest.raises(CommunicationError):client.send_command('MOVE,B,50')
            assert endpoint.lines.count('MOVE,B,50')==1


def test_physical_early_acknowledged_stop_aborts_without_waiting_for_done():
    with Endpoint(simple_reply) as endpoint:
        with physical_client(endpoint,timeout=.3) as client:
            accepted=client.send_command('MOVE,L,1000')
            worker,finished,outcomes,errors=start_wait(client,accepted)
            try:
                stopped=client.stop_and_wait(accepted)
                assert finished.wait(1)
            finally:worker.join(2)
            assert not errors and stopped.execution_state=='aborted'
            assert outcomes[0].execution_state=='aborted' and not outcomes[0].execution_completed
            assert 'DONE,MOVE' not in client.telemetry
            assert endpoint.lines.count('MOVE,L,1000')==1 and endpoint.lines.count('STOP')==1
