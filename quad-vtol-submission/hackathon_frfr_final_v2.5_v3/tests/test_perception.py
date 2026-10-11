import cv2
import numpy as np
import pytest
from quadstack.perception import detect_target
from quadstack.core import Rejected


def frame(marker=7):
    image=np.full((240,320),255,dtype=np.uint8)
    image[60:180,100:220]=cv2.aruco.generateImageMarker(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50),marker,120)
    return image


def detect(image,**kw):
    params=dict(timestamp=10.,now=10.1,fx=250.,fy=250.,cx=159.5,cy=119.5)
    params.update(kw)
    return detect_target(image,**params)


def test_real_aruco_detection_on_synthetic_frame():
    result=detect(frame())
    assert result.marker_id==7 and result.valid
    assert abs(result.angle_x)<.005 and abs(result.angle_y)<.005
    assert 117<=result.side_pixels<=121
    assert result.timestamp==10.


def test_wrong_missing_stale_and_oversized_targets():
    assert detect(frame(8)) is None
    assert detect(np.full((240,320),255,dtype=np.uint8)) is None
    with pytest.raises(Rejected):
        detect(frame(),now=11.)
    with pytest.raises(Rejected):
        detect(np.zeros((1080,1920),dtype=np.uint8))
    with pytest.raises(Rejected):
        detect(frame(),fx=0.)
