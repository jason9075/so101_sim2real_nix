#!/usr/bin/env python3
"""Isaac Sim ZMQ bridge (physics mode).

Physics 模式不會直接覆寫關節 state，而是透過 articulation controller 設定
position targets（PD drive），讓 PhysX 能正確處理接觸與碰撞。
"""

from __future__ import annotations

from typing import Any, Sequence

import numpy as np

import sim_server_common


def apply_joint_targets(robot: Any, joints: Sequence[float]) -> None:
  num_to_set = min(len(joints), robot.num_dof)
  if num_to_set <= 0:
    return

  # Keep extra DOF unchanged.
  target_positions = np.array(robot.get_joint_positions(), copy=True)
  target_positions[:num_to_set] = np.array(joints[:num_to_set])

  controller = robot.get_articulation_controller()

  # NOTE: Import inside function since this runs inside Isaac Sim container.
  from omni.isaac.core.utils.types import ArticulationAction

  controller.apply_action(ArticulationAction(joint_positions=target_positions))


def configure_robot(robot: Any) -> None:
  sim_server_common.configure_joint_drives(robot)


def main() -> None:
  sim_server_common.run_server(
    apply_joint_targets_fn=apply_joint_targets,
    configure_robot_fn=configure_robot,
  )


if __name__ == '__main__':
  main()
