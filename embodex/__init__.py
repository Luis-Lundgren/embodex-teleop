"""
Embodex Teleoperation & Motion Data Platform.
Provides APIs, dataset recording, challenge task simulation, and WebXR integration
for robotic teleoperation, building on core TeleGrip.
"""

import sys
from pathlib import Path

__version__ = "0.1.0"

# Automatically ensure vendor/telegrip is accessible if present
_vendor_dir = Path(__file__).resolve().parent.parent / "vendor" / "telegrip"
if _vendor_dir.is_dir() and str(_vendor_dir) not in sys.path:
    sys.path.insert(0, str(_vendor_dir))
