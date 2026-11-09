"""Non-blocking framing diagnostics and explicit R/Y baseline capture."""
from dataclasses import dataclass, field
import logging
import textwrap

import cv2

logger = logging.getLogger(__name__)


@dataclass
class FramingSession:
    capture_count: int = 15
    state: str = 'NORMAL'
    override_active: bool = False
    calibration_may_be_invalid: bool = False
    confirming_registration: bool = False
    reason: str = ''
    frames: list = field(default_factory=list)
    events: list = field(default_factory=list)
    _last_key: int = -1
    _return_state: str = 'NORMAL'

    def mismatch(self, reason: str) -> None:
        # Camera framing is operator-managed; only explicit R/Y starts capture.
        changed = self.reason != reason or not self.calibration_may_be_invalid
        self.calibration_may_be_invalid = True
        self.reason = reason
        if changed:
            self.events.append('FRAMING_WARNING_NONBLOCKING')
            logger.debug('Non-blocking camera framing warning: %s', reason)

    def key(self, raw: int) -> str | None:
        key = raw & 0xff
        if key in (27, ord('q'), ord('Q')):
            logger.info('Operator chose safe quit')
            return 'quit'
        if key == 255:
            self._last_key = -1
            return None
        key = ord(chr(key).lower())
        if key == self._last_key:
            return None
        self._last_key = key
        if self.state == 'BASELINE_REREGISTERING':
            return None  # Repeated keys cannot restart capture or write again.
        if self.confirming_registration:
            if key == ord('y'):
                self.confirming_registration = False
                self.state = 'BASELINE_REREGISTERING'
                self.frames.clear()
                self.events.append('BASELINE_REGISTRATION_CONFIRMED')
                logger.warning('Operator explicitly accepted ALL visible objects as legal; capturing median baseline')
                return 'confirm_registration'
            if key == ord('n'):
                self.confirming_registration = False
                self.state = self._return_state
                self.events.append('BASELINE_REGISTRATION_CANCELLED')
                return 'cancel'
            return None
        if key == ord('c') and self.state == 'WAITING_FOR_OPERATOR':
            self.override_active = True
            self.calibration_may_be_invalid = True
            self.state = 'OVERRIDE_ACTIVE'
            self.events.append('FRAMING_OVERRIDE_ACCEPTED')
            logger.warning('FRAMING OVERRIDE ACTIVE: explicit operator C; baseline/calibration unchanged; physical coordinates NOT VERIFIED')
            return 'override'
        if key == ord('r'):
            self._return_state = self.state
            self.state = 'WAITING_FOR_OPERATOR'
            self.confirming_registration = True
            self.events.append('BASELINE_REGISTRATION_REQUESTED')
            logger.warning('Re-registration requested: ALL visible objects will become legal; explicit Y required before any write')
            return 'register'
        return None

    def registered(self) -> None:
        self.frames.clear()
        self.override_active = False
        self.state = 'NORMAL'
        # A new baseline does not validate the original physical calibration.
        self.calibration_may_be_invalid = True
        self.reason = 'New baseline registered; physical calibration remains unverified'
        self.events.append('BASELINE_REREGISTERED')
        logger.info('New legal baseline persisted; stale tracks cleared; physical estimates remain UNVERIFIED.')

    def registration_failed(self, reason: str) -> None:
        self.frames.clear()
        self.confirming_registration = False
        self.state = 'NORMAL'
        self.reason = f'Baseline registration failed: {reason}'
        self.events.append('BASELINE_REGISTRATION_FAILED')

    def diagnostics(self) -> dict:
        events, self.events = self.events, []
        return {'state': self.state, 'override_active': self.override_active,
                'calibration_may_be_invalid': self.calibration_may_be_invalid,
                'confirming_baseline_registration': self.confirming_registration,
                'baseline_capture_frames': len(self.frames), 'baseline_capture_required': self.capture_count,
                'reason': self.reason, 'events': events}

    def draw(self, image, *, debug=False) -> int:
        lines = [f'STATE {self.state}'] if debug else []
        if debug and self.calibration_may_be_invalid:
            lines.append('CALIBRATION MAY BE INVALID - physical coordinates are not verified.')
        if self.confirming_registration:
            lines += ['ALL currently visible objects, including test targets, will become LEGAL.',
                      'Remove active targets unless you deliberately accept them as legal.',
                      'Y: confirm capture and replace baseline | N: cancel | Q: quit']
        elif debug and self.state == 'WAITING_FOR_OPERATOR':
            lines += ['Camera framing differs from the registered baseline. How would you like to proceed?',
                      'C: Continue anyway | R: Re-register current scene | Q: Quit safely', self.reason]
        elif self.state == 'BASELINE_REREGISTERING':
            lines += [f'Capturing legal baseline: {len(self.frames)}/{self.capture_count}',
                      'Keep the intended legal scene still. Q: quit without saving.']
        elif debug and self.override_active:
            lines += ['WARNING: FRAMING OVERRIDE ACTIVE - old baseline retained.',
                      'Diagnostic/demo only; detections and coordinates may be inaccurate.', 'R: Re-register | Q: Quit']
        width = max(18, int(image.shape[1]/6.5))
        wrapped = [part for line in lines for part in textwrap.wrap(line, width=width)]
        bottom = 93 if debug else 58
        for index, line in enumerate(wrapped):
            y = (110 if debug else 76)+index*17
            if y >= image.shape[0]-3:
                break
            cv2.putText(image,line,(8,y),0,.43,(0,0,0),3)
            cv2.putText(image,line,(8,y),0,.43,(0,180,255),1)
            bottom = y
        return bottom+12
