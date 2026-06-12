#!/bin/sh
set -e

# Run prisma db push if DIRECT_URL is set
if [ -n "$DIRECT_URL" ]; then
  echo "Database URL is configured. Waiting for database connection..."
  python -c "
import socket, sys, urllib.parse, os, time
url_str = os.environ.get('DIRECT_URL')
if url_str:
    try:
        if '://' in url_str:
            url_str_parsed = url_str.split('://')[1]
        else:
            url_str_parsed = url_str
        
        if '@' in url_str_parsed:
            credentials_and_host = url_str_parsed.split('@')[1]
        else:
            credentials_and_host = url_str_parsed
            
        if '/' in credentials_and_host:
            host_and_port = credentials_and_host.split('/')[0]
        else:
            host_and_port = credentials_and_host
            
        if ':' in host_and_port:
            host, port = host_and_port.split(':')
            port = int(port)
        else:
            host = host_and_port
            port = 5432
            
        print(f'Waiting for database at {host}:{port}...')
        for i in range(30):
            try:
                with socket.create_connection((host, port), timeout=2):
                    print('Database is ready!')
                    sys.exit(0)
            except OSError:
                print(f'Attempt {i+1}/30 failed, retrying...')
                time.sleep(1)
        print('Database connection timed out')
        sys.exit(1)
    except Exception as e:
        print(f'Error parsing DIRECT_URL or connecting: {e}')
        sys.exit(1)
"
  echo "Running database migrations via prisma db push..."
  prisma db push --schema=./prisma/schema.prisma
fi

# Execute the CMD passed to docker run
exec "$@"
