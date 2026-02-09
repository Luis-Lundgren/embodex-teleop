# Telegrip Dockerized Backend

This directory contains the Telegrip python backend, dockerized for easy deployment.

## Features
- Headless execution with PyBullet digital twin.
- Automatic self-signed SSL certificate generation.
- HTTPS API and WebSocket support.
- Recording enabled by default.

## Prerequisites
- Docker installed on your system.

## Building the Image
To build the Docker image, run the following command in this directory:

```bash
docker build -t telegrip-backend .
```

## Running the Container
To run the container with the default configuration (`--no-robot --digital-twin --record`):

```bash
docker run -p 8443:8443 -p 8442:8442 telegrip-backend
```

- **HTTPS API/UI:** Accessible at `https://localhost:8443`
- **WebSocket:** Accessible at `wss://localhost:8442`

### Persistent Recordings
If you want to keep the recordings after the container stops, mount a local volume:

```bash
docker run -p 8443:8443 -p 8442:8442 -v $(pwd)/records:/app/records telegrip-backend
```

### Custom Commands
You can override the default command if needed:

```bash
docker run -p 8443:8443 -p 8442:8442 telegrip-backend telegrip --no-robot --digital-twin --log-level info
```

## GitHub Actions
You can use the provided Dockerfile in a GitHub Actions workflow to automatically build and push to GHCR (GitHub Container Registry).
