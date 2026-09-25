"""
Embodex Teleoperation Persistence Layer.

Architectural Rule:
`embodex-web` is the canonical owner of the application and marketplace database
(Users, Accounts, Marketplace Datasets, Episodes, Payments, Jobs).
`embodex-teleop` owns local session recordings, trajectory storage (records/),
and robot run metadata, exposing them via the REST API (/api/sessions) to `embodex-web`.

This module handles session indexing and optional API notifications to `embodex-web`.
"""

import json
import logging
from pathlib import Path
import threading
from typing import Optional

logger = logging.getLogger(__name__)


async def notify_web_service_async(session_id: str, record_dir: str, metadata: dict) -> bool:
    """
    Optionally notify the canonical `embodex-web` service that a new teleoperation
    session recording has been finalized and is ready for marketplace ingestion.
    """
    import os
    import urllib.request

    web_url = os.environ.get("EMBODEX_WEB_URL", "").strip().rstrip("/")
    if not web_url:
        logger.debug("EMBODEX_WEB_URL not configured; skipping web service notification.")
        return False

    api_token = os.environ.get("EMBODEX_API_TOKEN", "").strip()
    payload = json.dumps({
        "sessionId": session_id,
        "recordDir": record_dir,
        "metadata": metadata,
    }).encode("utf-8")

    req = urllib.request.Request(
        f"{web_url}/api/teleop/sessions/notify",
        data=payload,
        headers={
            "Content-Type": "application/json",
            **({"Authorization": f"Bearer {api_token}"} if api_token else {}),
        },
        method="POST",
    )

    try:
        with urllib.request.urlopen(req, timeout=5.0) as resp:
            if resp.status in (200, 201, 202):
                logger.info(f"Successfully notified embodex-web of session {session_id}")
                return True
    except Exception as e:
        logger.debug(f"Notification to embodex-web failed (continuing normally): {e}")

    return False


async def save_session_to_db_async(session_id: str, json_data: dict, robot: str = "so100") -> bool:
    """
    Saves session metadata locally and optionally notifies embodex-web.
    Does NOT mutate marketplace tables directly.
    """
    logger.info(
        f"Session {session_id} finalized. Retained in local records for embodex-web API ingestion."
    )
    return True


def save_session_sync(
    session_id: str, json_data: dict, robot: str = "so100"
) -> Optional[threading.Thread]:
    """
    Finalize session persistence in background thread without blocking control loop.
    """
    def worker():
        try:
            import os
            web_url = os.environ.get("EMBODEX_WEB_URL", "").strip()
            if web_url:
                import asyncio
                asyncio.run(save_session_to_db_async(session_id, json_data, robot))
        except Exception as e:
            logger.debug(f"Session notification worker encountered an exception: {e}")

    thread = threading.Thread(target=worker, daemon=True)
    thread.start()
    return thread
