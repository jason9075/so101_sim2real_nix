set dotenv-load

docker_compose := "docker-compose -f docker/docker-compose.yml"

# List available commands
default:
    @just --list

# Start Isaac Sim container (Standby Mode)
up:
    @echo "Enabling Xhost for container access..."
    @xhost +local:root || true
    {{docker_compose}} up -d
    @echo "Container started in standby mode. Run 'just gui' or 'just sim' to start applications."

# Stop Isaac Sim container
down:
    {{docker_compose}} down

# Launch Isaac Sim GUI (Standard)
gui:
    @echo "Starting Isaac Sim GUI..."
    docker exec -it isaac-sim /isaac-sim/isaac-sim.sh

# Launch Sim Server (Bridge Mode)
sim:
    @echo "Installing pyzmq in container (if needed)..."
    docker exec isaac-sim /isaac-sim/python.sh -m pip install pyzmq
    @echo "Starting Sim Server..."
    docker exec -it isaac-sim /isaac-sim/python.sh /isaac-sim/scripts_local/bridge/sim_server.py

# Launch Host Driver (Client)
bridge port="/dev/ttyACM1":
    @echo "Starting Host Driver (Leader)..."
    python3 scripts/bridge/host_driver.py --port {{port}}

# Launch Host Driver (Mock Mode)
bridge-mock:
    @echo "Starting Host Driver (Mock Mode)..."
    python3 scripts/bridge/host_driver.py --mock

# Show container logs
logs:
    {{docker_compose}} logs -f

# Enter container shell
shell:
    docker exec -it isaac-sim bash
