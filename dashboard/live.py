"""One camera producer sharing the actual test-vision processor and overlays."""
import threading
import time
import logging
from types import SimpleNamespace

from vision.camera import CameraManager
from vision.live_test import LiveVision
from vision.localization_runtime import TargetEventStore


class LiveMonitor:
    def __init__(self,service,*,camera=0,width=1280,height=720,background='data/iphone_reference.png',
                 settings='config/spill_config.json',localization_settings='config/localization_config.json',
                 robot_settings='config/robot_config.json',marker_plane='config/robot_marker_plane.json'):
        self.service=service;self.camera=camera;self.width=width;self.height=height
        self.background=background;self.settings=settings;self.localization_settings=localization_settings
        self.robot_settings=robot_settings;self.marker_plane=marker_plane
        self.stop_event=threading.Event();self.thread=None

    def start(self):
        if self.thread and self.thread.is_alive():return
        self.stop_event.clear()
        self.thread=threading.Thread(target=self.run,name='dashboard-camera',daemon=True);self.thread.start()

    def stop(self):
        self.stop_event.set()
        if self.thread and self.thread is not threading.current_thread():self.thread.join(3)
        if self.thread and self.thread.is_alive():raise RuntimeError('Camera is still stopping; retry after it releases')

    def run(self):
        try:
            view=LiveVision(SimpleNamespace(calibration=self.service.calibration_path,
                robot_settings=self.robot_settings,marker_plane=self.marker_plane,background=self.background,
                settings=self.settings,localization_settings=self.localization_settings,width=self.width,height=self.height),
                calibration_id=self.service.calibration_id)
            events=TargetEventStore()  # Memory only; never append to experimental target files.
            with CameraManager(self.camera,view.width,view.height,30) as capture:
                while not self.stop_event.is_set():
                    start=time.monotonic();frame=capture.read()
                    if frame is None:raise RuntimeError('Camera returned no frame; retry monitoring after reconnecting')
                    image,summary=view.process(frame)
                    self.service.live_frame(image,events.update(view.records),paused=view.paused,diagnostics=summary['diagnostics'])
                    self.stop_event.wait(max(0,1/30-(time.monotonic()-start)))
        except Exception as exc:
            logging.getLogger(__name__).warning('Dashboard live monitoring unavailable: %s',exc)
            self.service.live_unavailable(str(exc))
        finally:
            if self.stop_event.is_set():self.service.live_unavailable('Live monitoring stopped')
