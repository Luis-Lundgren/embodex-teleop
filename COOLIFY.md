# Deploying Embodex Teleop on Coolify

This guide explains how to deploy the `embodex-teleop` service and its database on a Coolify instance with production-grade security configuration.

---

## Prerequisites
- A running Coolify instance.
- Docker installed on the host machine.
- An instance of `embodex-web` deployed or running to broker user teleoperation access.

---

## Deployment Steps

### Step 1: Create a Project in Coolify
1. Open your Coolify dashboard.
2. Navigate to **Projects** and click **Add New Project**.
3. Set a name, e.g., `Embodex Teleop Backend`.

### Step 2: Add a Docker Compose Resource
1. Inside the project environment, click **+ Add Resource** -> select **Docker Compose**.
2. Select your destination server (e.g., `localhost`).
3. Choose your configuration source:
   - **Git Repository**: Point to `https://github.com/Luis-Lundgren/embodex-teleop.git` on branch `main`. Coolify will automatically read `docker-compose.yml`.
   - **Raw Docker Compose**: Copy the contents of `docker-compose.yml` into the Compose editor in Coolify.

---

## Step 3: Configure Environment Variables

For secured deployments, configure the following environment variables in the Coolify **Environment Variables** section:

### 1. Security Configuration
| Variable | Description | Example Value |
| :--- | :--- | :--- |
| `EMBODEX_SECURITY_MODE` | Security mode: `hardware` (enforces tokens/tickets), `simulation`, or `local` | `hardware` |
| `EMBODEX_API_TOKEN` | Permanent service Bearer token matching `embodex-web` | *(Generate via `openssl rand -hex 32`)* |
| `EMBODEX_WS_TICKET_SECRET` | Shared secret to verify short-lived tickets matching `embodex-web` | *(Generate via `openssl rand -hex 32`)* |
| `EMBODEX_ALLOWED_ORIGINS` | Comma-separated allowed web origins for CORS | `https://your-personal-instance.example.com` |

> [!IMPORTANT]
> **Secret Generation**: Generate separate, cryptographically secure values for the service token and ticket secret:
> ```bash
> openssl rand -hex 32   # Value for EMBODEX_API_TOKEN
> openssl rand -hex 32   # Value for EMBODEX_WS_TICKET_SECRET
> ```
> **Do not reuse the same secret** for `EMBODEX_API_TOKEN` and `EMBODEX_WS_TICKET_SECRET`.
>
> The **same `EMBODEX_WS_TICKET_SECRET`** must be configured on both `embodex-web` and `embodex-teleop`, because `embodex-web` signs short-lived tickets and `embodex-teleop` verifies them.
>
> Likewise, `EMBODEX_API_TOKEN` on `embodex-web` must match `EMBODEX_API_TOKEN` on `embodex-teleop`.

### 2. General Service Settings
- `PORT`: App external port mapping (default: `8500`)
- `POSTGRES_USER`: Database user (default: `embodex`)
- `POSTGRES_PASSWORD`: Strong database password
- `POSTGRES_DB`: Database name (default: `embodex`)

---

## Step 4: Configure Domain and SSL
In the Coolify dashboard under the `app` service settings:
1. Locate the **Domains** field.
2. Enter the domain or subdomain for the teleop service (e.g. `https://teleop.your-personal-instance.example.com`).
3. Coolify automatically configures SSL/TLS and reverse-proxies both HTTP and WebSocket traffic (`/ws`).

---

## Step 5: Deploy the Application
1. Click **Deploy**.
2. Coolify will build the backend application image, initialize the database container, and launch the server.
3. Check the **Deployment Logs** to confirm startup:
   - FastAPI server logs: `Embodex server listening on 0.0.0.0:8000`.
   - Security status confirms `security_mode="hardware"` (or configured mode).

---

## Step 6: Verify Health & Security
Once deployed, test health and auth endpoints:
```bash
# Public health check
curl https://teleop.your-personal-instance.example.com/health

# Verify unauthenticated control calls are rejected (401)
curl -X POST https://teleop.your-personal-instance.example.com/api/robot \
  -H "Content-Type: application/json" \
  -d '{"action":"connect"}'

# Verify authorized control calls succeed
curl -X POST https://teleop.your-personal-instance.example.com/api/robot \
  -H "Content-Type: application/json" \
  -H "Authorization: Bearer <YOUR_EMBODEX_API_TOKEN>" \
  -d '{"action":"connect"}'
```
