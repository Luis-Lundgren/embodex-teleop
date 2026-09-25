"""
Test CLI entrypoints to ensure no naming collisions between telegrip and embodex-teleop.
"""

import shutil
import subprocess
import sys


def test_cli_entrypoints_distinct():
    """Verify telegrip and embodex-teleop CLIs are both present and display distinct help text."""
    # Test embodex-teleop CLI
    result_embodex = subprocess.run(
        [sys.executable, "-m", "embodex.cli", "--help"],
        capture_output=True,
        text=True,
        check=True,
    )
    assert "Embodex Teleoperation Backend" in result_embodex.stdout
    assert "--digital-twin" in result_embodex.stdout

    # Test telegrip CLI
    result_telegrip = subprocess.run(
        [sys.executable, "-m", "telegrip.main", "--help"],
        capture_output=True,
        text=True,
        check=True,
    )
    assert "Unified SO100 Robot Teleoperation System" in result_telegrip.stdout
    assert "--urdf" in result_telegrip.stdout
