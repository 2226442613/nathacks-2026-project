"""Camera capture and ground-plane coordinates for Hospital Guardian."""
from .camera import CameraError, CameraManager
from .calibration import (CalibrationError, GroundCalibration, WallCalibration,
                          DualPlaneCalibration, SurfaceType, SurfaceCoordinates,
                          RobotReachGeometry, load_calibration, CalibrationQualityThresholds,
                          PhysicalValidationPoint, load_validation_points)
from .spill_detection import (BackgroundReference, DetectorConfig, DetectionError, DetectionResult,
                              SpillCategory, SpillDetection, SpillDetector, render_detections)
from .spill_tracking import SpillTracker, TrackerConfig, load_spill_settings
from .exclusion import exclusion_mask_from_polygons, load_exclusion_polygons
from .frame_geometry import GeometryCheck
from .localization import LocalizationConfig, LocalizationResult, CameraRegistration, SurfaceLocalizer
from .localization_runtime import TargetEventStore, LocalizationRegistration

__all__ = ["CameraError", "CameraManager", "CalibrationError", "GroundCalibration",
           "WallCalibration", "DualPlaneCalibration", "SurfaceType", "SurfaceCoordinates",
           "RobotReachGeometry", "load_calibration", "CalibrationQualityThresholds",
           "PhysicalValidationPoint", "load_validation_points", "BackgroundReference", "DetectorConfig",
           "DetectionError", "DetectionResult", "SpillCategory", "SpillDetection", "SpillDetector",
           "render_detections", "SpillTracker", "TrackerConfig", "load_spill_settings",
           "exclusion_mask_from_polygons", "load_exclusion_polygons", "GeometryCheck",
           "LocalizationConfig","LocalizationResult","CameraRegistration","SurfaceLocalizer","TargetEventStore","LocalizationRegistration"]

from .robot_tracking import RobotConfig, MarkerObservation, RobotMarkerDetector, RobotPose, RobotTracker, robot_pose_is_current
from .robot_plane import MarkerPlaneMapping
__all__ += ['RobotConfig','MarkerObservation','RobotMarkerDetector','RobotPose','RobotTracker','MarkerPlaneMapping','robot_pose_is_current']

from .cleaning_config import CleaningHardware
from .cleaning_planner import CleaningPlanner, CleaningPlan, ApproachPose, CoverageSegment
__all__ += ['CleaningHardware','CleaningPlanner','CleaningPlan','ApproachPose','CoverageSegment']
