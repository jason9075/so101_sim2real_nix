#!/usr/bin/env python3
"""LeRobot calibration loader.

支援兩種格式：
- 完整格式（Goro 產生）：包含 `homing_offset`/`range_min`/`range_max`。
- 簡化格式（只有 min/max/center）：會推導 `homing_offset = center - 2048`。

本模組負責把兩種格式統一成同一個 in-memory representation。
"""

from __future__ import annotations

import json
from dataclasses import dataclass
from typing import Any


JOINT_NAMES: list[str] = [
    'shoulder_pan',
    'shoulder_lift',
    'elbow_flex',
    'wrist_flex',
    'wrist_roll',
    'gripper',
]


@dataclass(frozen=True)
class JointCalibration:
    joint_name: str
    servo_id: int
    drive_mode: int
    homing_offset: int
    range_min: int
    range_max: int

    @property
    def center_raw(self) -> int:
        return 2048 + self.homing_offset


def _load_json(path: str) -> dict[str, Any]:
    with open(path, 'r', encoding='utf-8') as f:
        data = json.load(f)

    if not isinstance(data, dict):
        raise ValueError(f'Invalid calibration JSON at {path}')

    return data


def load_calibration(path: str) -> dict[str, JointCalibration]:
    data = _load_json(path)

    # Full format detection: has `homing_offset` key.
    any_full = False
    for name in JOINT_NAMES:
        entry = data.get(name)
        if isinstance(entry, dict) and 'homing_offset' in entry:
            any_full = True
            break

    joints: dict[str, JointCalibration] = {}

    if any_full:
        for name in JOINT_NAMES:
            entry = data.get(name)
            if not isinstance(entry, dict):
                raise ValueError(f'Missing joint calibration: {name}')

            joints[name] = JointCalibration(
                joint_name=name,
                servo_id=int(entry.get('id', JOINT_NAMES.index(name) + 1)),
                drive_mode=int(entry.get('drive_mode', 0)),
                homing_offset=int(entry['homing_offset']),
                range_min=int(entry['range_min']),
                range_max=int(entry['range_max']),
            )

        return joints

    # Simple format: {name: {min,max,center}}
    for name in JOINT_NAMES:
        entry = data.get(name)
        if not isinstance(entry, dict):
            raise ValueError(f'Missing joint calibration: {name}')

        center = int(entry['center'])
        joints[name] = JointCalibration(
            joint_name=name,
            servo_id=JOINT_NAMES.index(name) + 1,
            drive_mode=0,
            homing_offset=center - 2048,
            range_min=int(entry['min']),
            range_max=int(entry['max']),
        )

    return joints
