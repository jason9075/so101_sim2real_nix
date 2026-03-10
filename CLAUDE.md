# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

SO-101 Sim-to-Real bridge for NixOS + NVIDIA Isaac Sim. Enables bidirectional communication between a real SO-101 robotic arm and its simulated counterpart in Isaac Sim.

## Architecture

```
┌─────────────────────┐     ZMQ 5555      ┌─────────────────────────┐
│  Host (NixOS)       │ ──────────────►   │  Container (Isaac Sim)  │
│  - leader_driver    │                   │  - sim_server_physics   │
│  - follower_driver  │ ◄──────────────   │  - sim_server_direct    │
└─────────────────────┘     ZMQ 5556      └─────────────────────────┘
```

**Data flows:**
- **real2sim** (Leader → Isaac Sim): Host reads leader arm via Serial, publishes joint angles to ZMQ port 5555
- **sim2real** (Isaac Sim → Follower): Isaac Sim publishes joint positions on ZMQ port 5556, Host writes to follower arm

**Key files:**
- `scripts/bridge/host_driver.py` - Reads leader arm (Feetech servos), publishes to ZMQ
- `scripts/bridge/follower_driver.py` - Subscribes from ZMQ, writes to follower arm with safety gates
- `scripts/bridge/sim_server_common.py` - Isaac Sim runtime: stage loading, robot initialization, ZMQ pub/sub loop
- `scripts/bridge/sim_server_physics.py` - Physics mode: uses articulation controller for PD control
- `scripts/bridge/calibration.py` - Loads joint calibration (homing offset, range limits)
- `scripts/bridge/profile_loader.py` - Single source of truth for serial ports and calibration paths

**Configuration:**
- Profile: `~/.cache/huggingface/lerobot/profile.json` (serial ports, calibration paths)
- Calibration: `~/.cache/huggingface/lerobot/calibration/{teleoperators,robots}/*.json`

## Development Commands

```bash
nix develop                    # Enter dev environment
just up                        # Start Isaac Sim container (standby mode)
just down                      # Stop container
just logs                      # View container logs
just shell                     # Enter container terminal

# GUI modes
just gui-bridge                # Isaac Sim GUI with bridge (scene A)
just gui-desk                  # Isaac Sim GUI with desk scene (sim2real.usd)
just webrtc                    # WebRTC headless streaming

# Host-side bridge
just real2sim                  # Leader → Sim (reads profile.json)
just real2sim /dev/ttyACM1     # Leader → Sim (explicit port)
just real2sim-mock             # Leader → Sim (mock data)
just sim2real                  # Sim → Follower (with torque enable)
just sim2real-dry              # Sim → Follower (dry run, no servo writes)

# Quality
just lint                      # Run ruff + shellcheck
just format                    # Run ruff format
just test                      # Run pytest
pytest scripts/tests/test_foo.py::test_bar  # Single test
```

## Code Conventions

- **Nix-First**: All dependencies in `flake.nix`. Never use `pip install` or `apt`.
- **Type Hints**: Required on all Python functions.
- **Logging**: Use `logging` module, not `print()`.
- **Language**: Comments/docs in Traditional Chinese (Taiwan). Technical terms in English.
- **Git**: Conventional Commits (feat, fix, docs, style, refactor, test, chore).

## Environment Variables (Isaac Sim scripts)

- `HEADLESS` - Run without GUI
- `ENABLE_WEBRTC` - Enable WebRTC streaming
- `ISAAC_STAGE_PATH` - Custom USD stage path
- `ROBOT_PRIM_PATH` - Robot articulation prim path
- `REAL2SIM_SUB_ENABLED` / `SIM2REAL_PUB_ENABLED` - Enable ZMQ bridges
- `SIM2REAL_PUB_RATE_HZ` - Publish rate (default 30)
- `SO101_KP` / `SO101_KD` - Joint PD gains (default 100000/1000)

## Safety Notes

- `sim2real` requires calibration; missing calibration will raise RuntimeError
- Safety gate: `--max-start-delta-raw` (default 300) blocks torque enable if sim target differs too much from follower pose
- Timeout/soft-stop: follower torque disables after command timeout
