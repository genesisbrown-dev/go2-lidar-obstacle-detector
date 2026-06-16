"""
tests/test_classifier.py
------------------------
Unit tests for the _classify() safety logic in LidarDetectorNode.

Run with:
    pytest tests/
or within ROS2:
    colcon test --packages-select go2_lidar_detector

No ROS2 middleware is needed for these tests because we test the
pure-Python classification function in isolation.
"""

import sys
import os

# ── Make the package importable without installing it ────────
# This lets pytest find go2_lidar_detector when run from the
# project root without `colcon build`.
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from go2_lidar_detector.lidar_detector import LidarDetectorNode


class TestClassify:
    """Tests for the static _classify() method."""

    # ── SAFE zone ────────────────────────────────────────────
    def test_safe_far(self):
        """Obstacle clearly far away → SAFE."""
        result = LidarDetectorNode._classify(5.0)
        assert "SAFE" in result

    def test_safe_boundary(self):
        """Obstacle exactly at 2.0 m → SAFE (exclusive upper threshold)."""
        result = LidarDetectorNode._classify(2.001)
        assert "SAFE" in result

    # ── CAUTION zone ─────────────────────────────────────────
    def test_caution_middle(self):
        """Obstacle at 1.5 m → CAUTION."""
        result = LidarDetectorNode._classify(1.5)
        assert "CAUTION" in result

    def test_caution_lower_boundary(self):
        """Obstacle exactly at 1.0 m → CAUTION (inclusive lower bound)."""
        result = LidarDetectorNode._classify(1.0)
        assert "CAUTION" in result

    def test_caution_upper_boundary(self):
        """Obstacle exactly at 2.0 m → CAUTION (inclusive upper bound)."""
        result = LidarDetectorNode._classify(2.0)
        assert "CAUTION" in result

    # ── STOP zone ────────────────────────────────────────────
    def test_stop_close(self):
        """Obstacle very close → STOP."""
        result = LidarDetectorNode._classify(0.3)
        assert "STOP" in result

    def test_stop_boundary(self):
        """Obstacle just below 1.0 m → STOP."""
        result = LidarDetectorNode._classify(0.999)
        assert "STOP" in result

    def test_stop_zero(self):
        """Edge case: distance = 0 (sensor touching object) → STOP."""
        result = LidarDetectorNode._classify(0.0)
        assert "STOP" in result
