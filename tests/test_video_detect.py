# -*- coding: utf-8 -*-
"""Tests for video_detect.py distance estimation."""

import os
import sys
from unittest.mock import MagicMock, patch

# Mock hardware dependencies
sys.modules.setdefault('cv2', MagicMock())
sys.modules.setdefault('numpy', MagicMock())

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

import pytest


class TestReadDistance:
    """read_distance maps contour bounding-box width to a distance value."""

    def _make_contour(self, x, y, w, h):
        """Build a minimal fake contour list that cv2.boundingRect will accept."""
        import cv2
        fake_contour = MagicMock()
        # cv2.contourArea and cv2.boundingRect are already mocked globally;
        # we configure cv2.boundingRect to return the desired bounding box
        cv2.contourArea.return_value = 1000
        cv2.boundingRect.return_value = (x, y, w, h)
        return [fake_contour]

    def test_wide_contour_means_close(self):
        """A contour near the max width (~230 px) should give a distance near 0."""
        from video_detect import read_distance
        import cv2
        cv2.boundingRect.return_value = (100, 50, 220, 100)
        cv2.contourArea.return_value = 1000
        frame = MagicMock()
        contours = [MagicMock()]
        result = read_distance(contours, frame)
        # 220 px width close to 230 -> distance close to 0
        assert result < 20

    def test_narrow_contour_means_far(self):
        """A contour near the min width (~46 px) should give a distance near 100."""
        from video_detect import read_distance
        import cv2
        cv2.boundingRect.return_value = (100, 50, 50, 100)
        cv2.contourArea.return_value = 500
        frame = MagicMock()
        contours = [MagicMock()]
        result = read_distance(contours, frame)
        # 50 px width close to 46 -> distance close to 100
        assert result > 80

    def test_rectangle_drawn_on_frame(self):
        """read_distance should draw a bounding rectangle on the frame."""
        from video_detect import read_distance
        import cv2
        cv2.boundingRect.return_value = (10, 20, 100, 80)
        cv2.contourArea.return_value = 800
        cv2.rectangle.reset_mock()
        frame = MagicMock()
        contours = [MagicMock()]
        read_distance(contours, frame)
        cv2.rectangle.assert_called_once()
