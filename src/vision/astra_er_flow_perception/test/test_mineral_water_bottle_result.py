def valid_bottle_result(result):
    roi = result.get("roi_xywh")
    depth = result.get("depth_m")

    return (
        result.get("target") == "mineral water bottle"
        and result.get("found") is True
        and result.get("tracking") is True
        and isinstance(roi, list)
        and len(roi) == 4
        and roi[2] >= 10
        and roi[3] >= 20
        and isinstance(depth, (int, float))
        and 0.20 <= float(depth) <= 6.00
    )


def test_expected_mineral_water_bottle_result():
    result = {
        "target": "mineral water bottle",
        "found": True,
        "tracking": True,
        "source": "tracker",
        "roi_xywh": [240, 110, 95, 260],
        "center_uv": [288, 240],
        "depth_m": 1.35,
    }
    assert valid_bottle_result(result)


def test_invalid_missing_depth():
    result = {
        "target": "mineral water bottle",
        "found": True,
        "tracking": True,
        "roi_xywh": [240, 110, 95, 260],
        "depth_m": None,
    }
    assert not valid_bottle_result(result)
