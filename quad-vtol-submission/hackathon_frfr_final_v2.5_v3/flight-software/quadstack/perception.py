"""Optional, bounded image-only fiducial processing. No camera or flight-control I/O."""
from dataclasses import dataclass
import math
from .core import number, Rejected


@dataclass(frozen=True)
class TargetObservation:
    timestamp: float
    marker_id: int
    angle_x: float
    angle_y: float
    side_pixels: float
    valid: bool
    source: str = "opencv-image"


def detect_target(gray, *, timestamp, now, fx, fy, cx, cy, expected_id=7):
    """Undistorted grayscale frame, caller-supplied calibrated intrinsics.

    Angular centre only: no uncalibrated metric pose, pad-clear inference, confidence
    probability, or autonomous precision landing output. Capture time is preserved.
    Intended cap: 640x480 frames; caller schedules <=5 Hz on a Pi after profiling.
    """
    import cv2
    import numpy as np
    for x in (timestamp,now,fx,fy,cx,cy):
        number(x)
    if fx <= 0 or fy <= 0 or not 0 <= now-timestamp <= .5:
        raise Rejected("invalid intrinsics or stale image")
    if not isinstance(gray,np.ndarray) or gray.dtype!=np.uint8 or gray.ndim!=2 or not 16<=gray.shape[0]<=480 or not 16<=gray.shape[1]<=640:
        raise Rejected("expected bounded grayscale image")
    dictionary=cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    detector=cv2.aruco.ArucoDetector(dictionary,cv2.aruco.DetectorParameters())
    corners,ids,_=detector.detectMarkers(gray)
    if ids is None:
        return None
    found=[c.reshape(4,2) for c,i in zip(corners,ids.flatten()) if i==expected_id]
    if len(found)!=1:
        return None
    c=found[0]
    lengths=[float(np.linalg.norm(c[(i+1)%4]-c[i])) for i in range(4)]
    if min(lengths)<12 or max(lengths)/min(lengths)>2:
        return None
    u,v=c.mean(axis=0)
    return TargetObservation(timestamp,expected_id,math.atan2(float(u)-cx,fx),math.atan2(float(v)-cy,fy),sum(lengths)/4,True)
