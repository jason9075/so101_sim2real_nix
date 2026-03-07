#!/usr/bin/env python3
"""Isaac Sim ZMQ bridge (direct mode).

Direct 模式會在每個 step 直接呼叫 `set_joint_positions()` 改變關節狀態。
優點是反應快；缺點是容易造成碰撞穿透（因為等同於瞬移 state）。
"""

from __future__ import annotations

from typing import Any, Sequence

import numpy as np

import sim_server_common


def apply_joint_targets(robot: Any, joints: Sequence[float]) -> None:
  current_positions = robot.get_joint_positions()
  target_positions = np.array(current_positions, copy=True)

  num_to_set = min(len(joints), robot.num_dof)
  if num_to_set <= 0:
    return

  target_positions[:num_to_set] = np.array(joints[:num_to_set])
  robot.set_joint_positions(target_positions)


def main() -> None:
  sim_server_common.run_server(apply_joint_targets_fn=apply_joint_targets)


if __name__ == '__main__':
  main()
