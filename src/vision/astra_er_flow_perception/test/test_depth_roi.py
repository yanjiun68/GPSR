import numpy as np
from astra_er_flow_perception.depth_roi import DepthROI


def test_uint16_depth_median():
    depth = np.full((100, 100), 1500, dtype=np.uint16)
    est = DepthROI(depth_scale=0.001).estimate(depth, (10, 10, 40, 40))
    assert est is not None
    assert abs(est["depth_m"] - 1.5) < 1e-6


def test_invalid_depth():
    depth = np.zeros((50, 50), dtype=np.uint16)
    est = DepthROI(depth_scale=0.001).estimate(depth, (0, 0, 30, 30))
    assert est is None
