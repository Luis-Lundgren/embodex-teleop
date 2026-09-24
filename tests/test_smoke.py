"""
Smoke test for Embodex Teleop Backend.
Verifies that the server, digital-twin simulator, and API endpoints start and pass health check
without physical hardware or headsets attached.
"""

import asyncio
import json
import urllib.request

from telegrip.config import TelegripConfig
from embodex.cli import EmbodexTeleopSystem


async def _run_smoke_cycle():
    port = 8998
    config = TelegripConfig()
    config.enable_robot = False
    config.enable_pybullet_gui = False
    config.enable_vr = False
    config.enable_keyboard = False
    config.port = port
    config.host_ip = "127.0.0.1"
    config.log_level = "error"

    system = EmbodexTeleopSystem(
        config=config,
        record_dir="/tmp/embodex_test_records",
        record_vr_only=True,
    )

    # Start system in background task
    system_task = asyncio.create_task(system.start())

    def _query(endpoint: str):
        with urllib.request.urlopen(f"http://127.0.0.1:{port}{endpoint}", timeout=1.5) as resp:
            return resp.status, json.loads(resp.read().decode("utf-8"))

    # Wait for server to bind
    for _ in range(25):
        await asyncio.sleep(0.2)
        try:
            status, data = await asyncio.to_thread(_query, "/health")
            if status == 200 and data.get("status") == "ok":
                break
        except Exception:
            continue
    else:
        await system.stop()
        system_task.cancel()
        raise AssertionError("Server did not respond to /health in time")

    # Verify /api/status
    status, status_data = await asyncio.to_thread(_query, "/api/status")
    assert status == 200
    assert "running" in status_data
    assert status_data["running"] is True

    # Clean shutdown
    await system.stop()
    await asyncio.sleep(0.1)
    system_task.cancel()
    try:
        await system_task
    except (asyncio.CancelledError, Exception):
        pass


def test_backend_health_and_status():
    """Run full simulation-mode smoke test."""
    asyncio.run(_run_smoke_cycle())


if __name__ == "__main__":
    test_backend_health_and_status()
    print("✅ Smoke test passed!")
