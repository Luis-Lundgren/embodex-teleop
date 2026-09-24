# Deploying Telegrip on Local Coolify

This guide explains how to host the Telegrip application and its PostgreSQL database on your local Coolify instance running at `http://localhost:8000/`.

## Prerequisites
- A running Coolify instance (e.g., accessible at `http://localhost:8000/`).
- Docker installed on the host machine where Coolify is running.

---

## Deployment Steps

### Step 1: Create a Project in Coolify
1. Open your Coolify dashboard at `http://localhost:8000/`.
2. Go to **Projects** and click **Add New Project**.
3. Provide a name, e.g., `Telegrip Unified Teleoperation`.

### Step 2: Add a Docker Compose Resource
1. Inside the project environment, click **+ Add Resource** -> select **Docker Compose**.
2. Select the destination server (usually `localhost`).
3. You have two options for the source:
   - **Git Repository**: Point it to this repository's Git URL (`https://github.com/Luis-Lundgren/telegrip-embodex.git`) and branch. Coolify will read the `docker-compose.yml` from the repository root.
   - **Raw Docker Compose**: Copy the contents of [docker-compose.yml](file:///home/user0/TELEOP/embodex-telegrip/docker-compose.yml) and paste it directly into the Compose box in Coolify.

### Step 3: Configure Environment Variables
Coolify will parse the environment variables from the compose file. You can configure them or leave the defaults:
- `POSTGRES_USER`: Database user (default: `telegrip`)
- `POSTGRES_PASSWORD`: Database password (default: `telegrip_password`)
- `POSTGRES_DB`: Database name (default: `telegrip`)
- `PORT`: App external port mapping (default: `8000`)

*Note: The database credentials will be dynamically injected into the database connection variables (`DATABASE_URL` and `DIRECT_URL`) for the application container.*

### Step 4: Configure Domain and SSL
In the Coolify dashboard under the `app` service settings:
1. Locate the **Domains** field.
2. Enter the domain or local URL you want to use (e.g., `http://telegrip.local` or `http://localhost:8000`).
3. Coolify will automatically configure its reverse proxy (Traefik/Caddy) to forward traffic to the application.
4. **WebSocket Support:** Coolify's reverse proxy supports WebSockets out-of-the-box. The frontend dynamically resolves the websocket endpoint using the browser's current host (e.g. `ws://telegrip.local/ws` or `wss://telegrip.local/ws`), ensuring seamless connection without configuration.

### Step 5: Deploy the Application
1. Click **Deploy**.
2. Coolify will start the database container, wait for it to be healthy, build the backend application image, run migrations via the custom entrypoint, and launch the server.
3. Check the **Deployment Logs** in Coolify to ensure the backend starts correctly:
   - You should see `Database is ready!` followed by `Running database migrations via prisma db push...`.
   - Then, the FastAPI server logs: `Server running on 0.0.0.0:8000`.

---

## Customizing Command Line Arguments
By default, the container starts with:
`telegrip --no-robot --digital-twin --record`

If you want to modify the command line parameters (e.g. disable PyBullet simulation, change log level, etc.), you can edit the **Command** field of the `app` service in Coolify:
- Example command to disable PyBullet visualization (headless): `telegrip --no-robot --no-viz --record`
- Example command to stream VR inputs to ROS2: `telegrip --no-robot --ros2 --record`
