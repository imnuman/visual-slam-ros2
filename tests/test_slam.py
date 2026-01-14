"""
Tests for Visual SLAM ROS2.
Run with: pytest tests/ -v
"""

import pytest
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).parent.parent))


class TestConfiguration:
    """Test configuration."""

    def test_requirements_exists(self):
        assert (Path(__file__).parent.parent / "requirements.txt").exists()

    def test_config_directory_exists(self):
        assert (Path(__file__).parent.parent / "config").exists()


class TestSLAM:
    """Test SLAM components."""

    def test_src_directory_exists(self):
        assert (Path(__file__).parent.parent / "src").exists()


class TestEvaluation:
    """Test evaluation logs."""

    def test_logs_directory_exists(self):
        assert (Path(__file__).parent.parent / "logs").exists()


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
