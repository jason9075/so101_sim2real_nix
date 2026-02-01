.PHONY: up down logs shell clean help

# Load environment variables
ifneq (,$(wildcard .env))
    include .env
    export
endif

DOCKER_COMPOSE = docker-compose -f docker/docker-compose.yml

help:
	@echo "Available targets:"
	@echo "  up      - Start Isaac Sim container"
	@echo "  down    - Stop Isaac Sim container"
	@echo "  logs    - Show container logs"
	@echo "  shell   - Enter container shell"
	@echo "  clean   - Remove data and logs"

up:
	@echo "Enabling Xhost for container access..."
	@xhost +local:root || true
	$(DOCKER_COMPOSE) up -d
	@echo "Isaac Sim is starting. Use 'make logs' to monitor progress."

down:
	$(DOCKER_COMPOSE) down

logs:
	$(DOCKER_COMPOSE) logs -f

shell:
	docker exec -it isaac-sim /bin/bash

clean:
	rm -rf data/* logs/*
