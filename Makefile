.PHONY: up down logs shell clean help

# Load environment variables
ifneq (,$(wildcard .env))
    include .env
    export
endif

DOCKER_COMPOSE = docker-compose -f docker/docker-compose.yml

help:
	@echo "Available targets:"
	@echo "  up      - Start Isaac Sim container (Standby Mode)"
	@echo "  down    - Stop Isaac Sim container"
	@echo "  gui     - Launch Isaac Sim GUI (Standard)"
	@echo "  sim     - Launch Sim Server (Bridge Mode)"
	@echo "  bridge  - Launch Host Driver (Client)"
	@echo "  logs    - Show container logs"
	@echo "  shell   - Enter container shell"

up:
	@echo "Enabling Xhost for container access..."
	@xhost +local:root || true
	$(DOCKER_COMPOSE) up -d
	@echo "Container started in standby mode. Run 'make gui' or 'make sim' to start applications."

down:
	$(DOCKER_COMPOSE) down

gui:
	@echo "Starting Isaac Sim GUI..."
	docker exec -it isaac-sim /isaac-sim/isaac-sim.sh

sim:
	@echo "Installing pyzmq in container (if needed)..."
	docker exec isaac-sim /isaac-sim/python.sh -m pip install pyzmq
	@echo "Starting Sim Server..."
	docker exec -it isaac-sim /isaac-sim/python.sh /isaac-sim/scripts_local/bridge/sim_server.py

bridge:
	@echo "Starting Host Driver (Mock Mode)..."
	python3 scripts/bridge/host_driver.py --mock

