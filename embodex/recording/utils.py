"""
Utilities for session recording and file management.
"""

from pathlib import Path


def get_next_session_dir(root_dir: Path, prefix: str = "test_session") -> Path:
    """
    Find the next available session directory with an incremented number.
    e.g. test_session_00, test_session_01, etc.
    """
    if not root_dir.exists():
        root_dir.mkdir(parents=True, exist_ok=True)

    i = 0
    while True:
        session_dir = root_dir / f"{prefix}_{i:02d}"
        if not session_dir.exists():
            return session_dir
        i += 1
