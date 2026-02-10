FROM python:3.10-slim

# Install system dependencies
# build-essential and git are often needed for pip installing certain packages
# libgl1 and libglib2.0-0 are for pybullet/opencv headless if needed
# openssl is used by the application to generate self-signed certificates
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    libgl1 \
    libglib2.0-0 \
    git \
    openssl \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /app

# Copy requirement files first for better caching
COPY requirements.txt .
# Install dependencies
RUN pip install --no-cache-dir -r requirements.txt

# Copy the application source and assets
COPY telegrip ./telegrip
COPY pyproject.toml .

# Install the package itself in editable mode so it can find its own files relative to project root
RUN pip install --no-cache-dir -e .

COPY URDF ./URDF
COPY web-ui ./web-ui
COPY config.yaml .
COPY reference_poses.json .

# Create directory for recordings
RUN mkdir -p records

# Expose port (Railway will override this with PORT environment variable)
ENV PORT=8000
EXPOSE 8000

# Ensure python output is streamed directly to terminal
ENV PYTHONUNBUFFERED=1

# Default command as requested: no physical robot, digital twin enabled, recording active
CMD ["telegrip", "--no-robot", "--digital-twin", "--record"]
