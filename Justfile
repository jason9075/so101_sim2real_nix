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
    @echo "Container started in standby mode. Run 'just gui' to start applications."

# Stop Isaac Sim container
down:
    {{docker_compose}} down

# Launch Isaac Sim GUI (Standard)
gui:
    @echo "Starting Isaac Sim GUI (Forcing X11)..."
    docker exec -it -e WAYLAND_DISPLAY="" isaac-sim /isaac-sim/isaac-sim.sh

# Launch Isaac Sim GUI with Bridge (scene A)
# - real2sim: SUB 5555 -> control sim
# - sim2real: PUB 5556 (30Hz) from sim state
gui-bridge:
    @echo "Installing pyzmq in container (if needed)..."
    docker exec isaac-sim /isaac-sim/python.sh -m pip install pyzmq
    @echo "Starting Isaac Sim GUI with Bridge (Physics)..."
    docker exec -it \
        -e WAYLAND_DISPLAY="" \
        -e HEADLESS=False \
        -e REAL2SIM_SUB_ENABLED=True \
        -e SIM2REAL_PUB_ENABLED=True \
        -e SIM2REAL_PUB_RATE_HZ=30 \
        -e SIM2REAL_PUB_BIND=tcp://*:5556 \
        -e LD_LIBRARY_PATH=/run/opengl-driver/lib \
        -e NVIDIA_VISIBLE_DEVICES=all \
        -e NVIDIA_DRIVER_CAPABILITIES=all \
        isaac-sim /isaac-sim/python.sh /isaac-sim/scripts_local/bridge/sim_server_physics.py

# Launch Isaac Sim GUI with Bridge (Direct: set_joint_positions)
gui-bridge-direct:
    @echo "Installing pyzmq in container (if needed)..."
    docker exec isaac-sim /isaac-sim/python.sh -m pip install pyzmq
    @echo "Starting Isaac Sim GUI with Bridge (Direct)..."
    docker exec -it \
        -e WAYLAND_DISPLAY="" \
        -e HEADLESS=False \
        -e REAL2SIM_SUB_ENABLED=True \
        -e SIM2REAL_PUB_ENABLED=True \
        -e SIM2REAL_PUB_RATE_HZ=30 \
        -e SIM2REAL_PUB_BIND=tcp://*:5556 \
        -e LD_LIBRARY_PATH=/run/opengl-driver/lib \
        -e NVIDIA_VISIBLE_DEVICES=all \
        -e NVIDIA_DRIVER_CAPABILITIES=all \
        isaac-sim /isaac-sim/python.sh /isaac-sim/scripts_local/bridge/sim_server_direct.py

# Launch Isaac Sim GUI with desk stage loaded (scene B)
# - real2sim: SUB 5555 -> control sim
# - sim2real: PUB 5556 (30Hz) from sim state
gui-desk stage="/isaac-sim/assets/sim2real.usd" robot="/World/so101_follower":
    @echo "Installing pyzmq in container (if needed)..."
    docker exec isaac-sim /isaac-sim/python.sh -m pip install pyzmq
    @echo "Starting Isaac Sim GUI Desk (Physics)..."
    docker exec -it \
        -e WAYLAND_DISPLAY="" \
        -e HEADLESS=False \
        -e REAL2SIM_SUB_ENABLED=True \
        -e SIM2REAL_PUB_ENABLED=True \
        -e SIM2REAL_PUB_RATE_HZ=30 \
        -e SIM2REAL_PUB_BIND=tcp://*:5556 \
        -e ISAAC_STAGE_PATH={{stage}} \
        -e ROBOT_PRIM_PATH={{robot}} \
        -e LD_LIBRARY_PATH=/run/opengl-driver/lib \
        -e NVIDIA_VISIBLE_DEVICES=all \
        -e NVIDIA_DRIVER_CAPABILITIES=all \
        isaac-sim /isaac-sim/python.sh /isaac-sim/scripts_local/bridge/sim_server_physics.py

# Launch Sim Server (WebRTC Streaming) - Optimized for Isaac Sim 5.0.0 (Default: Physics)
webrtc:
    @echo "Installing pyzmq in container (if needed)..."
    docker exec isaac-sim /isaac-sim/python.sh -m pip install pyzmq
    @echo "Starting Sim Server (WebRTC Mode, Physics)..."
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
        isaac-sim /isaac-sim/python.sh /isaac-sim/scripts_local/bridge/sim_server_physics.py \
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

# Host real2sim (Leader -> Sim)
# Usage: `just real2sim` (use profile.json) or `just real2sim /dev/ttyACM1`
real2sim port="__PROFILE__":
    @echo "Starting real2sim (Leader -> Sim)..."
    python3 scripts/bridge/host_driver.py --port {{port}}

real2sim-mock:
    @echo "Starting real2sim (Mock Mode)..."
    python3 scripts/bridge/host_driver.py --mock

# Host sim2real (Sim -> Follower)
# Usage: `just sim2real` (use profile.json) or `just sim2real /dev/ttyACM0`
sim2real port="__PROFILE__":
    @echo "Starting sim2real (Sim -> Follower)..."
    python3 scripts/bridge/follower_driver.py --port {{port}} --enable --timeout-ms 300 --soft-stop-ms 300

sim2real-dry port="__PROFILE__":
    @echo "Starting sim2real (Dry Run)..."
    python3 scripts/bridge/follower_driver.py --port {{port}} --timeout-ms 300 --soft-stop-ms 300

# Host real2sim2real (Leader -> Sim -> Follower)
# Usage: `just real2sim2real` (use profile.json) or `just real2sim2real /dev/ttyACM1 /dev/ttyACM0`
real2sim2real leader="__PROFILE__" follower="__PROFILE__":
    @echo "Starting real2sim + sim2real..."
    python3 scripts/bridge/host_driver.py --port {{leader}} & \
    python3 scripts/bridge/follower_driver.py --port {{follower}} --enable --timeout-ms 300 --soft-stop-ms 300 & \
    wait

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
