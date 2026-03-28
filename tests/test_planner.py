# -*- coding: utf-8 -*-
"""Tests for planner.py navigation math and waypoint reading."""

import math
import os
import sys
import tempfile

# Mock ROS/hardware dependencies so planner can be imported without them
sys.modules.setdefault('rospy', __import__('unittest.mock', fromlist=['MagicMock']).MagicMock())
sys.modules.setdefault('RPi', __import__('unittest.mock', fromlist=['MagicMock']).MagicMock())
sys.modules.setdefault('RPi.GPIO', __import__('unittest.mock', fromlist=['MagicMock']).MagicMock())
sys.modules.setdefault('clever', __import__('unittest.mock', fromlist=['MagicMock']).MagicMock())
sys.modules.setdefault('clever.srv', __import__('unittest.mock', fromlist=['MagicMock']).MagicMock())
sys.modules.setdefault('mavros_msgs', __import__('unittest.mock', fromlist=['MagicMock']).MagicMock())
sys.modules.setdefault('mavros_msgs.srv', __import__('unittest.mock', fromlist=['MagicMock']).MagicMock())
sys.modules.setdefault('std_srvs', __import__('unittest.mock', fromlist=['MagicMock']).MagicMock())
sys.modules.setdefault('std_srvs.srv', __import__('unittest.mock', fromlist=['MagicMock']).MagicMock())
sys.modules.setdefault('cv2', __import__('unittest.mock', fromlist=['MagicMock']).MagicMock())
sys.modules.setdefault('numpy', __import__('unittest.mock', fromlist=['MagicMock']).MagicMock())

# Add repo root to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

import pytest
from planner import nav4, read_waypoints


class TestNav4:
    """nav4 converts (current, target) positions to a bearing in degrees."""

    def test_due_east(self):
        # Target directly to the right (+x): bearing 0°
        assert nav4(0, 0, 1, 0) == pytest.approx(0, abs=1e-9)

    def test_due_north(self):
        # Target directly up (+y): bearing 90°
        assert nav4(0, 0, 0, 1) == pytest.approx(90, abs=1e-9)

    def test_due_west(self):
        # Target directly to the left (-x): bearing 180°
        assert nav4(0, 0, -1, 0) == pytest.approx(180, abs=1e-9)

    def test_due_south(self):
        # Target directly down (-y): bearing -90°
        assert nav4(0, 0, 0, -1) == pytest.approx(-90, abs=1e-9)

    def test_northeast_diagonal(self):
        # 45° northeast — equal +x and +y components
        angle = nav4(0, 0, 1, 1)
        assert angle == pytest.approx(45.0, abs=1e-6)

    def test_southeast_diagonal(self):
        # -45° (southeast)
        angle = nav4(0, 0, 1, -1)
        assert angle == pytest.approx(-45.0, abs=1e-6)

    def test_northwest_diagonal(self):
        # 135° (northwest)
        angle = nav4(0, 0, -1, 1)
        assert angle == pytest.approx(135.0, abs=1e-6)

    def test_southwest_diagonal(self):
        # -135° (southwest)
        angle = nav4(0, 0, -1, -1)
        assert angle == pytest.approx(-135.0, abs=1e-6)

    def test_non_origin_source(self):
        # Same relative direction from a non-origin position
        assert nav4(2, 3, 3, 3) == pytest.approx(nav4(0, 0, 1, 0), abs=1e-9)

    def test_result_in_range(self):
        # Result must stay in [-180, 180]
        for dx in range(-5, 6):
            for dy in range(-5, 6):
                if dx == 0 and dy == 0:
                    continue
                angle = nav4(0, 0, dx, dy)
                assert -180 <= angle <= 180


class TestReadWaypoints:
    """read_waypoints parses the points file into a list of strings."""

    def test_reads_all_lines(self, tmp_path):
        points_file = tmp_path / "points.txt"
        points_file.write_text("1.5 2.0 1.5 0\n4.0 1.5 1.0 1\n")
        waypoints = read_waypoints(str(points_file))
        assert len(waypoints) == 2

    def test_parses_coordinates(self, tmp_path):
        points_file = tmp_path / "points.txt"
        points_file.write_text("4 1.5 1 1\n")
        waypoints = read_waypoints(str(points_file))
        x, y, z, state = map(float, waypoints[0].split())
        assert x == 4.0
        assert y == 1.5
        assert z == 1.0
        assert int(state) == 1

    def test_ignores_blank_lines(self, tmp_path):
        points_file = tmp_path / "points.txt"
        points_file.write_text("1 2 3 0\n\n2 3 4 1\n\n")
        waypoints = read_waypoints(str(points_file))
        assert len(waypoints) == 2

    def test_independent_calls(self, tmp_path):
        """Calling read_waypoints twice returns independent lists."""
        points_file = tmp_path / "points.txt"
        points_file.write_text("1 2 3 0\n")
        first = read_waypoints(str(points_file))
        second = read_waypoints(str(points_file))
        assert first == second
        assert first is not second
