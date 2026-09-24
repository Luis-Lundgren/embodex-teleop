FROM python:3.10-slim

# Install system dependencies
# build-essential and git are needed for compiling certain packages
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
COPY requirements-base.txt requirements.txt ./
RUN pip install --default-timeout=1000 --no-cache-dir -r requirements.txt

# Copy vendor/telegrip and install it
COPY vendor/telegrip ./vendor/telegrip
RUN pip install --no-cache-dir -e ./vendor/telegrip

# Copy the embodex application source and config
COPY embodex ./embodex
COPY pyproject.toml .
COPY config.yaml .
COPY reference_poses.json .
COPY web-ui ./web-ui

# Symlink URDF to vendor/telegrip/URDF for backward compatibility
RUN ln -s vendor/telegrip/URDF URDF

# Install embodex-teleop package in editable mode
RUN pip install --no-cache-dir -e .

# Copy prisma schema and generate client if schema exists
COPY prisma ./prisma
RUN if [ -f "./prisma/schema.prisma" ]; then prisma generate --schema=./prisma/schema.prisma; fi

# Create directory for recordings
RUN mkdir -p records

# Copy entrypoint script and set executable permissions
COPY entrypoint.sh /app/entrypoint.sh
RUN chmod +x /app/entrypoint.sh

# Expose port (default 8500, configurable via PORT env var)
ENV PORT=8500
EXPOSE 8500

# Ensure python output is streamed directly to terminal
ENV PYTHONUNBUFFERED=1

# Set the entrypoint script to run on container startup
ENTRYPOINT ["/app/entrypoint.sh"]

# Default command: simulation mode, digital twin active, auto-record enabled
CMD ["embodex-teleop", "--no-robot", "--digital-twin", "--record"]
