#!/usr/bin/env python3
"""Isaac Sim ZMQ bridge (common runtime).

此模組提供共用的 Isaac Sim 啟動、stage/robot 載入與 ZMQ 主迴圈。
控制策略（Direct/Physics）應在獨立腳本中實作，並注入 apply function。
"""

from __future__ import annotations

import json
import logging
import os
import time
from typing import Any, Callable, Optional, Sequence

ApplyJointTargetsFn = Callable[[Any, Sequence[float]], None]
ConfigureRobotFn = Callable[[Any], None]

import numpy as np
import zmq


logging.basicConfig(level=logging.INFO)
logger = logging.getLogger('SimServer')


def fix_robot_base(stage: Any, robot_prim_path: str) -> None:
  """嘗試用 PhysicsFixedJoint 固定機器人底座。

  注意：若 base link 已經是 static/kinematic，則不應再嘗試建立 FixedJoint，
  否則可能導致 PhysX 初始化失敗。
  """

  from pxr import Gf, Usd, UsdPhysics

  prim = stage.GetPrimAtPath(robot_prim_path)
  if not prim.IsValid():
    logger.error(f'Robot prim path not found in stage: {robot_prim_path}')
    return

  base_link_prim = None
  candidates = [prim] + list(prim.GetChildren())
  for candidate in candidates:
    if candidate.HasAPI(UsdPhysics.RigidBodyAPI):
      base_link_prim = candidate
      break

  if not base_link_prim:
    for candidate in Usd.PrimRange(prim):
      if candidate.HasAPI(UsdPhysics.RigidBodyAPI):
        base_link_prim = candidate
        break

  if not base_link_prim:
    logger.warning(f'Could not find a RigidBody to fix in {robot_prim_path}')
    return

  rigid_body = UsdPhysics.RigidBodyAPI.Get(stage, base_link_prim.GetPath())
  if rigid_body:
    kinematic_attr = rigid_body.GetKinematicEnabledAttr()
    if kinematic_attr and bool(kinematic_attr.Get()):
      logger.info(
        'Base link is kinematic; skip FixedJoint to avoid PhysX errors. '
        f'prim={base_link_prim.GetPath()}',
      )
      return

    enabled_attr = rigid_body.GetRigidBodyEnabledAttr()
    if enabled_attr and not bool(enabled_attr.Get()):
      logger.info(
        'Base link rigid body disabled/static; skip FixedJoint. '
        f'prim={base_link_prim.GetPath()}',
      )
      return

  logger.info(f'Fixing robot base via FixedJoint on: {base_link_prim.GetPath()}')

  joint_path = f'{base_link_prim.GetPath()}/root_joint'
  if stage.GetPrimAtPath(joint_path).IsValid():
    return

  fixed_joint = UsdPhysics.FixedJoint.Define(stage, joint_path)
  fixed_joint.CreateBody1Rel().SetTargets([base_link_prim.GetPath()])

  # Ensure transform is identity.
  # fixed_joint.CreateLocalPos0Attr().Set(Gf.Vec3f(0, 0, 0))
  # fixed_joint.CreateLocalRot0Attr().Set(Gf.Quatf(1, 0, 0, 0))
  # fixed_joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0, 0, 0))
  # fixed_joint.CreateLocalRot1Attr().Set(Gf.Quatf(1, 0, 0, 0))


def find_first_articulation_root(stage: Any) -> Optional[str]:
  """回傳最可能的 Articulation root prim path。

  有些資產會把 `ArticulationRootAPI` 套在 rigid body（例如 `/.../base`）。
  此時回傳 parent Xform 通常更符合 `Articulation(prim_path=...)` 的預期。
  """

  from pxr import Usd, UsdPhysics

  pseudo_root = stage.GetPseudoRoot()
  pseudo_root_path = pseudo_root.GetPath()

  for prim in Usd.PrimRange(pseudo_root):
    if not prim.HasAPI(UsdPhysics.ArticulationRootAPI):
      continue

    parent = prim.GetParent()
    if parent.IsValid() and parent.GetPath() != pseudo_root_path:
      return str(parent.GetPath())

    return str(prim.GetPath())

  return None


def open_stage(kit: Any, stage_path: str) -> bool:
  """開啟 stage（open_stage 為 async，會短暫 pump updates）。"""

  import omni.usd

  logger.info(f'Opening stage: {stage_path}')
  usd_context = omni.usd.get_context()
  usd_context.open_stage(stage_path)

  for _ in range(120):
    kit.update()
    stage = usd_context.get_stage()
    if stage is not None:
      return True

  return False


def configure_joint_drives(robot: Any) -> None:
  """設定關節 stiffness/damping，讓 PD 控制更穩定。"""

  k_p = float(os.getenv('SO101_KP', '100000.0'))
  k_d = float(os.getenv('SO101_KD', '1000.0'))

  try:
    controller = robot.get_articulation_controller()
    controller.set_gains(kps=np.full(robot.num_dof, k_p), kds=np.full(robot.num_dof, k_d))
    logger.info(f'Set joint gains for {robot.name}: kP={k_p}, kD={k_d}')
  except Exception as exc:
    logger.warning(f'Failed to set joint gains: {exc}')


def setup_zmq_subscriber(address: str = 'tcp://127.0.0.1:5555') -> zmq.Socket:
  context = zmq.Context.instance()
  socket = context.socket(zmq.SUB)
  socket.setsockopt(zmq.LINGER, 0)

  # Keep only latest messages.
  try:
    socket.setsockopt(zmq.RCVHWM, 1)
    socket.setsockopt(zmq.CONFLATE, 1)
  except Exception:
    pass

  socket.connect(address)
  socket.setsockopt_string(zmq.SUBSCRIBE, '')
  logger.info('Connected to ZMQ Publisher')
  return socket


def setup_zmq_publisher(bind_address: str = 'tcp://*:5556') -> zmq.Socket:
  context = zmq.Context.instance()
  socket = context.socket(zmq.PUB)
  socket.setsockopt(zmq.LINGER, 0)

  try:
    socket.setsockopt(zmq.SNDHWM, 1)
  except Exception:
    pass

  socket.bind(bind_address)
  logger.info(f'ZMQ publisher bound to {bind_address}')
  return socket


def publish_positions(socket: zmq.Socket, positions: Sequence[float], *, seq: int) -> None:
  payload = {
    'timestamp': time.time(),
    'seq': seq,
    'joints': list(map(float, positions)),
  }
  socket.send_string(json.dumps(payload))


def receive_latest_joints(socket: zmq.Socket) -> Optional[list[float]]:
  latest_msg: Optional[str] = None

  while True:
    try:
      latest_msg = socket.recv_string(flags=zmq.NOBLOCK)
    except zmq.Again:
      break

  if not latest_msg:
    return None

  data = json.loads(latest_msg)
  joints = data.get('joints', [])
  if not isinstance(joints, list):
    return None

  return [float(v) for v in joints]


def create_simulation_app() -> Any:
  from isaacsim import SimulationApp

  headless = os.getenv('HEADLESS', 'False').lower() == 'true'
  enable_webrtc = os.getenv('ENABLE_WEBRTC', 'False').lower() == 'true'

  config: dict[str, Any] = {
    'headless': headless,
    'width': 1280,
    'height': 720,
    'experience': '/isaac-sim/apps/isaacsim.exp.full.kit',
  }

  if enable_webrtc:
    config['livestream'] = 'webrtc'
    logger.info('Enabling WebRTC Livestreaming with Full Editor (Isaac Sim 5.0.0+)')

  kit = SimulationApp(config)

  if enable_webrtc:
    from omni.isaac.core.utils.extensions import enable_extension

    enable_extension('omni.kit.livestream.webrtc')
    enable_extension('omni.services.transport.server.http')

  return kit


def resolve_robot_prim_path(
  kit: Any,
  robot_prim_path: str,
  get_current_stage_fn: Callable[[], Any],
) -> str:
  stage = get_current_stage_fn()

  for _ in range(240):
    if robot_prim_path:
      prim = stage.GetPrimAtPath(robot_prim_path)
      if prim.IsValid():
        return robot_prim_path
    else:
      detected = find_first_articulation_root(stage)
      if detected:
        return detected

    kit.update()
    stage = get_current_stage_fn()

  return robot_prim_path


def create_world_and_robot(kit: Any) -> tuple[Any, Any, bool]:
  """回傳 (world, robot, add_ground_plane)。"""

  from omni.isaac.core import World
  from omni.isaac.core.articulations import Articulation
  from omni.isaac.core.utils.stage import add_reference_to_stage, get_current_stage

  stage_path = os.getenv('ISAAC_STAGE_PATH', '').strip()
  robot_prim_path = os.getenv('ROBOT_PRIM_PATH', '').strip()

  if stage_path:
    if not os.path.exists(stage_path):
      raise FileNotFoundError(f'Stage not found at {stage_path}')

    if not open_stage(kit, stage_path):
      raise RuntimeError(f'Failed to open stage: {stage_path}')

  world = World()

  add_ground_plane = True
  robot: Any

  if stage_path:
    robot_prim_path = resolve_robot_prim_path(kit, robot_prim_path, get_current_stage)
    stage = get_current_stage()

    if robot_prim_path:
      prim = stage.GetPrimAtPath(robot_prim_path)
      if not prim.IsValid():
        logger.warning(
          f'ROBOT_PRIM_PATH not found: {robot_prim_path}. Falling back to auto-detect.',
        )
        robot_prim_path = ''

    if not robot_prim_path:
      robot_prim_path = find_first_articulation_root(stage) or ''

    if not robot_prim_path:
      raise RuntimeError('ROBOT_PRIM_PATH not set and no ArticulationRoot found in stage.')

    logger.info(f'Using robot prim path: {robot_prim_path}')
    fix_robot_base(stage, robot_prim_path)

    robot = Articulation(prim_path=robot_prim_path, name='so101')
    world.scene.add(robot)
    add_ground_plane = False
  else:
    asset_path = '/isaac-sim/assets/so101_follower.usd'

    if os.path.exists(asset_path):
      logger.info(f'Found custom asset at {asset_path}. Loading SO-101...')
      add_reference_to_stage(usd_path=asset_path, prim_path='/World/SO101')

      stage = get_current_stage()
      fix_robot_base(stage, '/World/SO101')

      robot = Articulation(prim_path='/World/SO101', name='so101')
      world.scene.add(robot)
    else:
      logger.warning(f'Asset not found at {asset_path}. Fallback to Franka.')
      from omni.isaac.franka import Franka

      robot = Franka(prim_path='/World/Franka', name='franka')
      world.scene.add(robot)

  return world, robot, add_ground_plane


def drain_socket(socket: zmq.Socket) -> None:
  while True:
    try:
      socket.recv_string(flags=zmq.NOBLOCK)
    except zmq.Again:
      return


def run_server(
  apply_joint_targets_fn: ApplyJointTargetsFn,
  configure_robot_fn: Optional[ConfigureRobotFn] = None,
) -> None:
  kit = create_simulation_app()

  import omni.timeline

  timeline = omni.timeline.get_timeline_interface()
  warmup_frames = int(os.getenv('SO101_WARMUP_FRAMES', '10'))

  real2sim_enabled = os.getenv('REAL2SIM_SUB_ENABLED', 'True').lower() == 'true'
  sim2real_enabled = os.getenv('SIM2REAL_PUB_ENABLED', 'True').lower() == 'true'
  sim2real_bind = os.getenv('SIM2REAL_PUB_BIND', 'tcp://*:5556')
  sim2real_rate_hz = int(os.getenv('SIM2REAL_PUB_RATE_HZ', '30'))
  sim2real_rate_hz = max(1, sim2real_rate_hz)

  try:
    sub_socket = setup_zmq_subscriber() if real2sim_enabled else None
    pub_socket = setup_zmq_publisher(sim2real_bind) if sim2real_enabled else None
    pub_seq = 0
    pub_interval = 1.0 / float(sim2real_rate_hz)
    next_pub_time = time.monotonic()

    world, robot, add_ground_plane = create_world_and_robot(kit)

    if add_ground_plane:
      world.scene.add_default_ground_plane()

    try:
      world.reset()
    except Exception as exc:
      logger.error(f'World Reset Failed: {exc}')
      return

    if configure_robot_fn is not None:
      configure_robot_fn(robot)

    try:
      logger.info(f'Robot Num DOF: {robot.num_dof}')
      logger.info(f'Robot DOF Names: {robot.dof_names}')
    except Exception as exc:
      logger.warning(f'Failed to read robot DOF info: {exc}')

    was_playing = False
    remaining_warmup_frames = 0
    reset_on_resume = False

    def is_stopped() -> bool:
      is_stopped_fn = getattr(timeline, 'is_stopped', None)
      if callable(is_stopped_fn):
        return bool(is_stopped_fn())
      return False

    while kit.is_running():
      is_playing = bool(timeline.is_playing())

      if not is_playing:
        # 停止/暫停時，不套用 teleop（避免 reset 後被舊 target 立刻覆寫）。
        if sub_socket is not None:
          drain_socket(sub_socket)
        kit.update()
        was_playing = False

        # Stop 會重置 simulation view；下次 resume 需要 world.reset().
        if is_stopped():
          reset_on_resume = True

        remaining_warmup_frames = warmup_frames
        continue

      if not was_playing:
        # Resume：清掉 pause 期間殘留封包，並讓 PhysX 有幾個 frame 穩定。
        if reset_on_resume:
          try:
            world.reset()
          except Exception as exc:
            logger.error(f'World Reset Failed: {exc}')
            return

          if configure_robot_fn is not None:
            configure_robot_fn(robot)

          reset_on_resume = False

        if sub_socket is not None:
          drain_socket(sub_socket)

        remaining_warmup_frames = warmup_frames
        next_pub_time = time.monotonic()
        was_playing = True

      world.step(render=True)

      if remaining_warmup_frames > 0:
        remaining_warmup_frames -= 1
        continue

      if sub_socket is not None:
        try:
          joints = receive_latest_joints(sub_socket)
          if joints is not None:
            apply_joint_targets_fn(robot, joints)
        except Exception as exc:
          logger.error(f'Error in apply: {exc}')

      if pub_socket is not None:
        now = time.monotonic()
        if now >= next_pub_time:
          next_pub_time = now + pub_interval

          try:
            positions = robot.get_joint_positions()
            pos_arr = np.asarray(positions)
            if pos_arr.ndim == 1 and pos_arr.size >= 6:
              pub_seq += 1
              publish_positions(pub_socket, pos_arr[:6].tolist(), seq=pub_seq)

          except Exception as exc:
            logger.error(f'Error in publish: {exc}')

  finally:
    kit.close()
