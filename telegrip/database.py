import asyncio
import logging
import json
import threading
from prisma import Prisma
from datetime import datetime

logger = logging.getLogger(__name__)

async def _save_session_to_db_async(session_id: str, json_data: dict, robot: str = "so100"):
    """
    Async logic to save session to DB.
    """
    db = Prisma()
    try:
        await db.connect()
        logger.info(f"Connected to DB for session {session_id}")
        
        # Check if dataset exists for this session_id (sourceId)
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
                    # ownerId is unknown here unless passed in.
                }
            )
        else:
            logger.info(f"Found existing Dataset for session {session_id}")

        # Create Episode
        episodes_list = json_data.get("episodes", [])
        if not episodes_list:
            logger.warning("No episodes found in JSON data")
            return

        for ep_data in episodes_list:
            # Calculate duration
            timestamps = ep_data.get("timestamps", [])
            duration = 0.0
            if timestamps:
                duration = float(timestamps[-1] - timestamps[0])
            
            frame_count = len(timestamps)
            
            # Create episode
            # Create episode
            episode_id = ep_data.get('id', 'unknown')
            
            # Prisma Python Client requires 'Json' fields to be passed as serializable objects (dicts/lists)
            # or wrapped if it's a specific type. 
            # The error also mentioned `data.dataset` is required, so we should use the relation connect.
            
            # Use prisma.Json wrapper for JSON field
            from prisma import Json
            
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

    except Exception as e:
        logger.error(f"Failed to save session to DB: {e}", exc_info=True)
    finally:
        if db.is_connected():
            await db.disconnect()

def save_session_sync(session_id: str, json_data: dict, robot: str = "so100"):
    """
    Starts a thread to save the session to the database asynchronously.
    This prevents blocking the main control loop or recorder flush.
    """
    def worker():
        try:
            asyncio.run(_save_session_to_db_async(session_id, json_data, robot))
        except RuntimeError:
            # Fallback if somehow there's loop conflict, create new loop
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)
            loop.run_until_complete(_save_session_to_db_async(session_id, json_data, robot))
            loop.close()
        except Exception as e:
            logger.error(f"Error in DB save worker thread: {e}", exc_info=True)

    t = threading.Thread(target=worker, daemon=True)
    t.start()
    return t
