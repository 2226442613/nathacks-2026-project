"""Actual shared vision overlays and MJPEG; synthetic camera, no device/socket IO."""
import socket
import threading
from unittest.mock import Mock

import cv2
import numpy as np
import pytest

import main
from dashboard.app import create_app
from dashboard.live import LiveMonitor
from dashboard.service import DashboardService
from test_dashboard import FakeLive,service,client
from test_live_vision import live_inputs

pytestmark=pytest.mark.filterwarnings('ignore:Legacy floor-only calibration:UserWarning')


def test_test_vision_pixels_are_exact_dashboard_jpeg_input_and_mjpeg_payload(live_inputs,monkeypatch,capsys):
    import vision.live_test as vision
    import dashboard.live as live
    frame,args=live_inputs;options=main.build_parser().parse_args(args)
    monkeypatch.setattr(vision.time,'monotonic',lambda:100.)
    monkeypatch.setattr(vision.time,'time',lambda:1794189600.)
    monkeypatch.setattr(socket,'create_connection',Mock(side_effect=AssertionError('No ESP32/network access')))
    cameras=[]
    def camera(*args):
        capture=Mock();capture.__enter__=Mock(return_value=capture);capture.__exit__=Mock(return_value=False)
        capture.read.side_effect=lambda:frame.copy();cameras.append(capture);return capture
    manager=Mock(side_effect=camera)
    monkeypatch.setattr(vision,'CameraManager',manager);monkeypatch.setattr(live,'CameraManager',manager)
    desktop=[]
    monkeypatch.setattr(cv2,'namedWindow',Mock())
    monkeypatch.setattr(cv2,'imshow',lambda name,image:desktop.append(image.copy()))
    monkeypatch.setattr(cv2,'waitKey',Mock(side_effect=[-1]*4+[ord('q')]))
    monkeypatch.setattr(cv2,'getWindowProperty',Mock(return_value=1))
    monkeypatch.setattr(cv2,'destroyAllWindows',Mock())
    assert main.main(args)==0
    service=DashboardService(options.calibration,live_factory=FakeLive);service.set_mode('LIVE')
    monitor=LiveMonitor(service,background=options.background,robot_settings=options.robot_settings,marker_plane=options.marker_plane)
    annotated=[];jpegs=[];publish=service.live_frame
    def receive(image,events,**kwargs):
        annotated.append(image.copy());publish(image,events,**kwargs);jpegs.append(service.jpeg)
        if len(annotated)==5:monitor.stop_event.set()
    monkeypatch.setattr(service,'live_frame',receive)
    try:
        monitor.run()
        assert len(desktop)==len(annotated)==5
        assert all(np.array_equal(a,b) for a,b in zip(desktop,annotated))
        assert not np.array_equal(annotated[-1],frame)
        assert manager.call_count==2  # One for the standalone run; one for the Dashboard run.
        assert all(c.read.call_count==5 and c.__exit__.call_count==1 for c in cameras)
        for image,jpeg in zip(desktop,jpegs):
            assert cv2.imencode('.jpg',image,[cv2.IMWRITE_JPEG_QUALITY,82])[1].tobytes()==jpeg
        # Publish the last synthetic output again; no new detection or camera capture.
        publish(annotated[-1],[])
        app=create_app(service)
        for _ in range(3):  # Tabs and refresh all subscribe to the same JPEG.
            with app.test_client() as browser:
                response=browser.get('/api/stream.mjpg',buffered=False)
                try:
                    assert response.status_code==200
                    assert response.content_type=='multipart/x-mixed-replace; boundary=frame'
                    assert response.headers['Cache-Control']=='no-store'
                    chunk=next(response.response)
                    assert chunk.startswith(b'--frame\r\nContent-Type: image/jpeg\r\n')
                    assert chunk.split(b'\r\n\r\n',1)[1]==jpegs[-1]+b'\r\n'
                    assert browser.get('/api/frame.jpg').data==jpegs[-1]
                    assert browser.get('/api/state').get_json()['non_actuating']
                finally:response.close()
        assert manager.call_count==2
        assert len(service.snapshot()['tasks'])==2  # Confirmed red and non-red targets retained.
    finally:service.close()


def test_stream_delivers_new_shared_frame_and_releases_only_its_subscription(service,client):
    service.set_mode('LIVE')
    service.live_frame(np.zeros((720,1280,3),np.uint8),[])
    first=service.jpeg
    response=client.get('/api/stream.mjpg',buffered=False)
    assert first in next(response.response)
    service.live_frame(np.full((720,1280,3),255,np.uint8),[])
    assert service.jpeg!=first and service.jpeg in next(response.response)
    response.close()
    assert service.live.started and not service.live.stopped
    # A refreshed browser starts at the latest frame, without restarting capture.
    refreshed=client.get('/api/stream.mjpg',buffered=False)
    assert service.jpeg in next(refreshed.response)
    refreshed.close()


@pytest.mark.parametrize('ending',['camera_loss','demo','shutdown'])
def test_waiting_stream_exits_on_disconnect_mode_change_or_shutdown(service,ending):
    service.set_mode('LIVE');service.live_frame(np.zeros((720,1280,3),np.uint8),[])
    stream=service.frame_stream();next(stream)
    entered=threading.Event();result=[]
    def wait_for_next():
        entered.set();result.append(next(stream,None))
    thread=threading.Thread(target=wait_for_next,daemon=True);thread.start();assert entered.wait(1)
    if ending=='camera_loss':service.live_unavailable('Synthetic disconnect')
    elif ending=='demo':service.set_mode('DEMO')
    else:service.close()
    thread.join(1)
    assert not thread.is_alive() and result==[None]


def test_disconnected_camera_is_released_before_explicit_retry(live_inputs,monkeypatch):
    import dashboard.live as live
    _,args=live_inputs;options=main.build_parser().parse_args(args)
    service=DashboardService(options.calibration,live_factory=FakeLive);service.set_mode('LIVE')
    capture=Mock();capture.__enter__=Mock(return_value=capture);capture.__exit__=Mock(return_value=False)
    capture.read.side_effect=RuntimeError('Synthetic camera disconnected')
    manager=Mock(return_value=capture);monkeypatch.setattr(live,'CameraManager',manager)
    try:
        monitor=LiveMonitor(service,background=options.background,robot_settings=options.robot_settings,marker_plane=options.marker_plane)
        monitor.run()
        capture.__exit__.assert_called_once()
        assert service.monitor['status']=='UNAVAILABLE' and service.jpeg is None
        assert 'Synthetic camera disconnected' in service.monitor['message']
        assert create_app(service).test_client().get('/api/stream.mjpg').status_code==503
        assert manager.call_count==1  # No reconnect loop or second capture after failure.
    finally:service.close()


def test_live_stream_unavailable_in_demo_and_default_resolution_is_720p(client):
    assert client.get('/api/stream.mjpg').status_code==503
    args=main.build_parser().parse_args(['dashboard','--mode','live','--camera','0'])
    assert (args.width,args.height)==(1280,720)
    assert args.robot_settings=='config/robot_config.json' and args.marker_plane=='config/robot_marker_plane.json'


def test_one_real_worker_for_tabs_then_mode_change_releases_and_can_reopen(live_inputs,monkeypatch):
    import dashboard.live as live
    frame,args=live_inputs;options=main.build_parser().parse_args(args)
    capture=Mock();capture.__enter__=Mock(return_value=capture);capture.__exit__=Mock(return_value=False)
    capture.read.side_effect=lambda:frame.copy()
    manager=Mock(return_value=capture);monkeypatch.setattr(live,'CameraManager',manager)
    service=DashboardService(options.calibration,live_options=dict(background=options.background,
        robot_settings=options.robot_settings,marker_plane=options.marker_plane))
    ready=threading.Event();publish=service.live_frame
    def receive(*args,**kwargs):publish(*args,**kwargs);ready.set()
    monkeypatch.setattr(service,'live_frame',receive)
    try:
        service.set_mode('LIVE');worker=service.live
        assert ready.wait(2)
        browser=create_app(service).test_client()
        for _ in range(3):
            service.set_mode('LIVE')
            response=browser.get('/api/stream.mjpg',buffered=False)
            assert next(response.response).startswith(b'--frame\r\n');response.close()
        assert manager.call_count==1 and worker.thread.is_alive()
        service.set_mode('DEMO')
        assert not worker.thread.is_alive() and capture.__exit__.call_count==1
        ready.clear();service.set_mode('LIVE');assert ready.wait(2)
        assert manager.call_count==2
    finally:service.close()
    assert capture.__exit__.call_count==2
