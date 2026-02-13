set dotenv-load
set export

docker_compose := "docker-compose -f docker/docker-compose.yml"

# List available commands
default:
    @just --list

# Start Isaac Sim container (Standby Mode)
up:
    @echo "Enabling Xhost for container access..."
    @xhost +local:root || true
    @echo "Using DISPLAY=$DISPLAY"
    {{docker_compose}} up -d
    @echo "Container started in standby mode. Run 'just gui' or 'just sim' to start applications."

# Stop Isaac Sim container
down:
    {{docker_compose}} down

# Launch Isaac Sim GUI (Standard)
gui:
    @echo "Starting Isaac Sim GUI (Forcing X11)..."
    docker exec -it -e WAYLAND_DISPLAY="" isaac-sim /isaac-sim/isaac-sim.sh

# Launch Isaac Sim GUI with Bridge & Robot Preset loaded
gui-bridge:
    @echo "Installing pyzmq in container (if needed)..."
    docker exec isaac-sim /isaac-sim/python.sh -m pip install pyzmq
    @echo "Starting Isaac Sim GUI with Bridge..."
    docker exec -it \
        -e WAYLAND_DISPLAY="" \
        -e HEADLESS=False \
        -e LD_LIBRARY_PATH=/run/opengl-driver/lib \
        -e NVIDIA_VISIBLE_DEVICES=all \
        -e NVIDIA_DRIVER_CAPABILITIES=all \
        isaac-sim /isaac-sim/python.sh /isaac-sim/scripts_local/bridge/sim_server.py

# Launch Sim Server (WebRTC Streaming) - Optimized for Isaac Sim 5.0.0
webrtc:
    @echo "Installing pyzmq in container (if needed)..."
    docker exec isaac-sim /isaac-sim/python.sh -m pip install pyzmq
    @echo "Starting Sim Server (WebRTC Mode)..."
    @echo "----------------------------------------------------------------------------"
    @echo "NOTICE: Use 'Isaac Sim WebRTC Streaming Client' (AppImage/DMG) to connect."
    @echo "Server Address: 127.0.0.1 (Local) or your machine's Public IP."
    @echo "Required Ports: UDP 47998, TCP 49100"
    @echo "----------------------------------------------------------------------------"
    docker exec -it \
        -e HEADLESS=True \
        -e ENABLE_WEBRTC=True \
        -e WAYLAND_DISPLAY="" \
        -e LD_LIBRARY_PATH=/run/opengl-driver/lib \
        -e NVIDIA_VISIBLE_DEVICES=all \
        -e NVIDIA_DRIVER_CAPABILITIES=all \
        isaac-sim /isaac-sim/python.sh /isaac-sim/scripts_local/bridge/sim_server.py \
        --/app/livestream/enabled=true \
        --/app/livestream/type=webrtc \
        --/app/livestream/port=49100 \
        --/app/window/width=1280 \
        --/app/window/height=720 \
        --/app/window/hideUi=0 \
        --/exts/omni.kit.livestream.webrtc/enabled=true \
        --/exts/omni.services.transport.server.http/port=8211

# Launch Full Isaac Sim with WebRTC Streaming (Standard Full App)

stream-full:
    @echo "Starting Isaac Sim Full App (WebRTC Mode)..."
    @echo "NOTICE: Use 'Isaac Sim WebRTC Streaming Client' (AppImage/DMG) to connect."
    docker exec -it \
        -e WAYLAND_DISPLAY="" \
        isaac-sim /isaac-sim/isaac-sim.streaming.sh

# Show container logs
logs:
    {{docker_compose}} logs -f

# Enter container shell
shell:
    docker exec -it isaac-sim bash

# Launch Host Driver (Client)
bridge port="/dev/ttyACM0":
    @echo "Starting Host Driver (Leader)..."
    python3 scripts/bridge/host_driver.py --port {{port}}

# Launch Host Driver (Mock Mode)
bridge-mock:
    @echo "Starting Host Driver (Mock Mode)..."
    python3 scripts/bridge/host_driver.py --mock

# Check environment status
check:
    @echo "Checking Environment..."
    @echo "DISPLAY: $DISPLAY"
    @echo "NVIDIA Driver:"
    @nvidia-smi --query-gpu=driver_version --format=csv,noheader || echo "nvidia-smi failed"
    @echo "Docker Container Status:"
    @docker ps --format "table {{{{.Names}}}}\t{{{{.Status}}}}"
    @echo "Xhost Status:"
    @xhost

# Run linter
lint:
    ruff check .
    shellcheck scripts/*.sh

# Run formatter
format:
    ruff format .

# Run tests
test:
    pytest scripts/tests
