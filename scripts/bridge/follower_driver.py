#!/usr/bin/env python3
"""SO-101 Follower Driver (Sim-to-Real Bridge).

此程式負責 sim2real：從 Isaac Sim 端（ZMQ PUB）接收關節角度（radians），
並寫入 Feetech STS servo 的 goal position。

安全策略：
- 若超過 timeout 沒收到新命令，會進入 soft-stop（維持最後目標），
  soft-stop 結束後 torque disable。
- 可設定每 step 最大角度變化量，避免突跳。
"""

from __future__ import annotations

import argparse
import json
import logging
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import numpy as np
import serial
import zmq

# Allow running as a script (not `python -m`).
REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
  sys.path.insert(0, str(REPO_ROOT))

from scripts.bridge.calibration import JOINT_NAMES, load_calibration
from scripts.bridge.profile_loader import (
  DEFAULT_PROFILE_PATH,
  PROFILE_PORT_SENTINEL,
  find_profile,
)


logging.basicConfig(level=logging.INFO, format='[%(levelname)s] %(asctime)s - %(message)s')
logger = logging.getLogger('FollowerDriver')


HEADER = b'\xFF\xFF'
INSTR_READ = 0x02
INSTR_WRITE = 0x03
ADDR_GOAL_POSITION = 42
ADDR_PRESENT_POSITION = 56
ADDR_TORQUE_ENABLE = 40
POS_RESOLUTION = 4096.0
SERVO_IDS = [1, 2, 3, 4, 5, 6]


def calculate_checksum(payload: list[int]) -> int:
  """Checksum = ~(ID + Length + Instr + Params...) & 0xFF"""

  total = sum(payload)
  return (~total) & 0xFF


def make_packet(servo_id: int, instruction: int, params: list[int]) -> bytes:
  length = len(params) + 2  # Instr + Checksum
  payload = [servo_id, length, instruction, *params]
  checksum = calculate_checksum(payload)
  return HEADER + bytes(payload) + bytes([checksum])


def make_read_packet(servo_id: int, address: int, read_len: int) -> bytes:
  return make_packet(servo_id, INSTR_READ, [address, read_len])


def make_write_packet(servo_id: int, address: int, params: list[int]) -> bytes:
  return make_packet(servo_id, INSTR_WRITE, [address, *params])


def make_torque_packet(servo_id: int, enable: bool) -> bytes:
  return make_write_packet(servo_id, ADDR_TORQUE_ENABLE, [1 if enable else 0])


def make_goal_position_packet(servo_id: int, raw_position: int) -> bytes:
  raw_position &= 0xFFF
  low = raw_position & 0xFF
  high = (raw_position >> 8) & 0xFF
  return make_write_packet(servo_id, ADDR_GOAL_POSITION, [low, high])


def radians_to_raw(radians: float, center_raw: int) -> int:
  raw = int(round(radians * (POS_RESOLUTION / (2 * np.pi)) + center_raw))
  return max(0, min(4095, raw))


@dataclass
class DriverConfig:
  profile: str
  port: str
  baud: int
  zmq_addr: str
  timeout_ms: int
  soft_stop_ms: int
  max_delta_rad: float
  max_start_delta_raw: int
  enable: bool


class FeetechFollower:
  def __init__(self, port: str, baud: int):
    self.port = port
    self.baud = baud
    self.ser: Optional[serial.Serial] = None

  def read_present_position_raw(self, servo_id: int) -> Optional[int]:
    if not self.ser:
      return None

    self.ser.reset_input_buffer()
    self.ser.write(make_read_packet(servo_id, ADDR_PRESENT_POSITION, 2))

    try:
      response = self.ser.read(8)
      if len(response) != 8:
        return None

      if response[0] != 0xFF or response[1] != 0xFF:
        return None

      if response[2] != servo_id:
        return None

      resp_payload = list(response[2:7])
      calc_sum = calculate_checksum(resp_payload)
      if calc_sum != response[7]:
        return None

      low = response[5]
      high = response[6]
      raw_pos = ((high << 8) | low) & 0xFFF
      return int(raw_pos)

    except Exception:
      return None

  def read_all_present_positions_raw(self) -> list[int]:
    positions: list[int] = []
    for servo_id in SERVO_IDS:
      raw = self.read_present_position_raw(servo_id)
      if raw is None:
        # Fallback to calibrated center if read fails.
        raw = 2048
      positions.append(raw)
      time.sleep(0.001)

    return positions

  def connect(self) -> None:
    self.ser = serial.Serial(self.port, self.baud, timeout=0.05, write_timeout=0.05)
    logger.info(f'Connected to {self.port} at {self.baud} baud')

  def close(self) -> None:
    if self.ser and self.ser.is_open:
      self.ser.close()

  def torque_enable(self, enable: bool) -> None:
    if not self.ser:
      return

    for servo_id in SERVO_IDS:
      self.ser.write(make_torque_packet(servo_id, enable))
      time.sleep(0.001)

  def write_goal_positions(self, raw_positions: list[int]) -> None:
    if not self.ser:
      return

    if len(raw_positions) != len(SERVO_IDS):
      raise ValueError('expected 6 joint positions')

    for servo_id, raw in zip(SERVO_IDS, raw_positions, strict=True):
      self.ser.write(make_goal_position_packet(servo_id, raw))
      time.sleep(0.001)


def receive_latest(socket: zmq.Socket) -> Optional[list[float]]:
  latest_msg: Optional[str] = None

  while True:
    try:
      latest_msg = socket.recv_string(flags=zmq.NOBLOCK)
    except zmq.Again:
      break

  if not latest_msg:
    return None

  data = json.loads(latest_msg)
  joints = data.get('joints')
  if not isinstance(joints, list):
    return None

  return [float(v) for v in joints]


def clamp_raw_targets(last_raw: list[int], desired_raw: list[int], max_delta_raw: int) -> list[int]:
  targets: list[int] = []
  for prev, desired in zip(last_raw, desired_raw, strict=True):
    delta = desired - prev
    if delta > max_delta_raw:
      desired = prev + max_delta_raw
    elif delta < -max_delta_raw:
      desired = prev - max_delta_raw
    targets.append(max(0, min(4095, desired)))
  return targets


def main() -> None:
  parser = argparse.ArgumentParser(description='SO-101 Follower Driver (sim2real)')
  parser.add_argument(
    '--profile',
    type=str,
    default=DEFAULT_PROFILE_PATH,
    help='LeRobot profile.json path',
  )
  parser.add_argument(
    '--port',
    type=str,
    default=PROFILE_PORT_SENTINEL,
    help="Serial port path ('__PROFILE__' = use profile.json)",
  )
  parser.add_argument('--baud', type=int, default=1000000)
  parser.add_argument('--zmq-addr', type=str, default='tcp://127.0.0.1:5556')
  parser.add_argument('--timeout-ms', type=int, default=300)
  parser.add_argument('--soft-stop-ms', type=int, default=300)
  parser.add_argument('--max-delta-rad', type=float, default=0.02)
  parser.add_argument(
    '--max-start-delta-raw',
    type=int,
    default=300,
    help='Safety gate: require initial target within this delta (raw units) before torque enable',
  )
  parser.add_argument('--enable', action='store_true', help='Actually write commands to servos')
  args = parser.parse_args()

  cfg = DriverConfig(
    profile=args.profile,
    port=args.port,
    baud=args.baud,
    zmq_addr=args.zmq_addr,
    timeout_ms=args.timeout_ms,
    soft_stop_ms=args.soft_stop_ms,
    max_delta_rad=args.max_delta_rad,
    max_start_delta_raw=int(args.max_start_delta_raw),
    enable=bool(args.enable),
  )

  context = zmq.Context()
  socket = context.socket(zmq.SUB)
  socket.setsockopt(zmq.LINGER, 0)

  try:
    socket.setsockopt(zmq.RCVHWM, 1)
    socket.setsockopt(zmq.CONFLATE, 1)
  except Exception:
    pass

  socket.connect(cfg.zmq_addr)
  socket.setsockopt_string(zmq.SUBSCRIBE, '')
  logger.info(f'Connected to Sim publisher at {cfg.zmq_addr}')

  prof = find_profile(
    'so101_follower',
    profile_path=cfg.profile,
    fallback_port='/dev/ttyACM0',
  )

  follower_port = cfg.port
  if follower_port == PROFILE_PORT_SENTINEL:
    follower_port = ''

  follower_port = follower_port or prof.port
  if not prof.calib_path:
    raise RuntimeError(
      'Follower calibration path missing in profile.json; run calibration first.'
    )

  follower_calib = load_calibration(prof.calib_path)
  logger.info(f'Follower port: {follower_port}')
  logger.info(f'Follower calib: {prof.calib_path}')

  follower = FeetechFollower(follower_port, cfg.baud)
  if cfg.enable:
    follower.connect()
    follower.torque_enable(False)
  else:
    logger.warning('Running in dry-run mode (no servo writes). Use --enable to actuate.')

  max_delta_raw = max(1, int(round(cfg.max_delta_rad * (POS_RESOLUTION / (2 * np.pi)))))

  torque_enabled = False
  deadman_active = False
  last_cmd_time = time.monotonic()
  stopping_since: Optional[float] = None

  # Seed with actual follower positions if possible (torque is disabled).
  last_raw = [follower_calib[name].center_raw for name in JOINT_NAMES]
  if cfg.enable:
    present = follower.read_all_present_positions_raw()
    if len(present) == 6:
      last_raw = present

  poller = zmq.Poller()
  poller.register(socket, zmq.POLLIN)

  try:
    while True:
      now = time.monotonic()
      events = dict(poller.poll(timeout=10))

      if socket in events:
        joints = receive_latest(socket)
        if joints is None or len(joints) < 6:
          continue

        desired_raw: list[int] = []
        for joint_name, radians in zip(JOINT_NAMES, joints[:6], strict=True):
          calib = follower_calib[joint_name]
          raw = radians_to_raw(radians, calib.center_raw)

          # Clamp to follower safety range.
          raw = max(calib.range_min, min(calib.range_max, raw))
          desired_raw.append(raw)

        if cfg.enable and not torque_enabled:
          # Safety gate: require initial target close to current follower pose.
          # Re-read current follower pose (torque disabled) to avoid stale seed.
          present = follower.read_all_present_positions_raw()
          if len(present) == 6:
            last_raw = present

          max_delta = max(abs(d - p) for d, p in zip(desired_raw, last_raw, strict=True))
          if max_delta > cfg.max_start_delta_raw:
            logger.error(
              'Refuse to enable torque: sim target too far from follower pose. '
              f'max_delta_raw={max_delta} limit={cfg.max_start_delta_raw}'
            )
            logger.error(
              '請先在 Isaac Sim 將手臂移到接近真實 follower 的姿態，或調大 --max-start-delta-raw。'
            )
            continue

          follower.torque_enable(True)
          torque_enabled = True

        targets = clamp_raw_targets(last_raw, desired_raw, max_delta_raw)

        last_raw = targets
        last_cmd_time = now
        stopping_since = None
        deadman_active = False

        if cfg.enable:
          follower.write_goal_positions(targets)
        else:
          logger.info(f'DRY targets={targets}')

        continue

      # No new commands.
      if deadman_active:
        continue

      since_ms = (now - last_cmd_time) * 1000.0
      if since_ms <= cfg.timeout_ms:
        continue

      if stopping_since is None:
        stopping_since = now
        logger.warning('No commands received, entering soft-stop...')

      # Soft-stop: keep holding last target for a short duration.
      if cfg.enable and torque_enabled:
        follower.write_goal_positions(last_raw)

      if (now - stopping_since) * 1000.0 >= cfg.soft_stop_ms:
        if cfg.enable and torque_enabled:
          follower.torque_enable(False)
          torque_enabled = False

        deadman_active = True
        stopping_since = None
        logger.warning('Torque disabled (deadman).')
        # Keep waiting for next command.

  except KeyboardInterrupt:
    logger.info('Stopping...')
  finally:
    try:
      if cfg.enable and torque_enabled:
        follower.torque_enable(False)
    finally:
      follower.close()
      socket.close()
      context.term()


if __name__ == '__main__':
  main()
