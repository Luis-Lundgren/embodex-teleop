"""
Embodex Database Persistence Layer.
Handles asynchronous database synchronization of teleoperation datasets and episodes
to PostgreSQL via Prisma.
"""

import asyncio
import json
import logging
import threading
from typing import Optional

logger = logging.getLogger(__name__)


async def save_session_to_db_async(session_id: str, json_data: dict, robot: str = "so100") -> bool:
    """
    Asynchronous logic to persist a recorded session and its episodes to the database.
    """
    try:
        from prisma import Prisma, Json
    except ImportError:
        logger.debug("Prisma client not installed; skipping database persistence.")
        return False

    db = Prisma()
    try:
        await db.connect()
        logger.info(f"Connected to DB for session {session_id}")

        # Check if dataset already exists for this session_id (sourceId)
        dataset = await db.dataset.find_first(
            where={
                "sourceId": session_id
            }
        )

        if not dataset:
            logger.info(f"Creating new Dataset for session {session_id}")
            dataset = await db.dataset.create(
                data={
                    "title": f"Teleop Session: {session_id}",
                    "description": f"Recorded trajectory using {robot} robot.",
                    "sourceType": "teleop",
                    "sourceId": session_id,
                    "status": "COMPLETED",
                }
            )
        else:
            logger.info(f"Found existing Dataset for session {session_id}")

        # Create Episode(s)
        episodes_list = json_data.get("episodes", [])
        if not episodes_list:
            logger.warning("No episodes found in JSON data")
            return True

        for ep_data in episodes_list:
            timestamps = ep_data.get("timestamps", [])
            duration = float(timestamps[-1] - timestamps[0]) if timestamps else 0.0
            frame_count = len(timestamps)
            episode_id = ep_data.get("id", "unknown")

            await db.episode.create(
                data={
                    "dataset": {
                        "connect": {
                            "id": dataset.id
                        }
                    },
                    "data": Json(json_data),
                    "duration": duration,
                    "frameCount": frame_count,
                    "filePath": f"{session_id}/{episode_id}.json"
                }
            )
            logger.info(f"Saved Episode {episode_id} to DB for session {session_id}")
        return True

    except Exception as e:
        logger.error(f"Failed to save session {session_id} to DB: {e}", exc_info=True)
        return False
    finally:
        if db.is_connected():
            await db.disconnect()


def save_session_sync(session_id: str, json_data: dict, robot: str = "so100") -> Optional[threading.Thread]:
    """
    Starts a background daemon thread to save the session to the database.
    Prevents blocking the main teleoperation control loop or recorder flush.
    """
    def worker():
        try:
            asyncio.run(save_session_to_db_async(session_id, json_data, robot))
        except RuntimeError:
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)
            loop.run_until_complete(save_session_to_db_async(session_id, json_data, robot))
            loop.close()
        except Exception as e:
            logger.error(f"Error in DB save worker thread: {e}", exc_info=True)

    t = threading.Thread(target=worker, daemon=True)
    t.start()
    return t
