#!/usr/bin/env python3
"""LeRobot profile loader.

以 `~/.cache/huggingface/lerobot/profile.json` 為單一事實來源，避免手臂接錯
port / 套錯 calibration 造成暴衝。
"""

from __future__ import annotations

import json
import os
from dataclasses import dataclass
from typing import Any


DEFAULT_PROFILE_PATH = os.path.expanduser('~/.cache/huggingface/lerobot/profile.json')
PROFILE_PORT_SENTINEL = '__PROFILE__'


@dataclass(frozen=True)
class TtyProfile:
    arm_type: str
    port: str
    arm_id: str
    calib_path: str


def _parse_profile_entry(entry: dict[str, Any]) -> TtyProfile:
    return TtyProfile(
        arm_type=str(entry['type']),
        port=str(entry['port']),
        arm_id=str(entry['id']),
        calib_path=str(entry.get('calib_path', '')),
    )


def load_profiles(profile_path: str = DEFAULT_PROFILE_PATH) -> list[TtyProfile]:
    with open(profile_path, 'r', encoding='utf-8') as f:
        data = json.load(f)

    entries = data.get('tty_profiles', [])
    if not isinstance(entries, list):
        raise ValueError('Invalid profile.json: tty_profiles must be a list')

    return [_parse_profile_entry(e) for e in entries]


def find_profile(
    arm_type: str,
    *,
    profile_path: str = DEFAULT_PROFILE_PATH,
    fallback_port: str,
) -> TtyProfile:
    """Find a TTYProfile for the given arm type.

    If the profile is missing, returns a fallback profile with empty `calib_path`.
    """

    try:
        profiles = load_profiles(profile_path)
    except FileNotFoundError:
        return TtyProfile(arm_type=arm_type, port=fallback_port, arm_id='', calib_path='')

    for p in profiles:
        if p.arm_type == arm_type:
            return p

    return TtyProfile(arm_type=arm_type, port=fallback_port, arm_id='', calib_path='')
