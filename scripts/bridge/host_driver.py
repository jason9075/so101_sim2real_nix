#!/usr/bin/env python3
"""
LeIsaac Host Driver (Sim-to-Real Bridge)

Reads joint states from Feetech STS/SCS Servos (SO-101 Leader Arm)
and publishes them via ZeroMQ.
"""

import argparse
import json
import logging
import sys
import time
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

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='[%(levelname)s] %(asctime)s - %(message)s'
)
logger = logging.getLogger("HostDriver")

# Feetech Protocol Constants
HEADER = b'\xFF\xFF'
INSTR_READ = 0x02
ADDR_PRESENT_POSITION = 56
POS_RESOLUTION = 4096.0

class FeetechArm:
    def __init__(self, port: str, baudrate: int = 1000000):
        self.port = port
        self.baudrate = baudrate
        self.ser: Optional[serial.Serial] = None
        
    def connect(self):
        try:
            self.ser = serial.Serial(
                self.port, 
                self.baudrate, 
                timeout=0.05, # Fast timeout for 1Mbps
                write_timeout=0.05
            )
            logger.info(f"Connected to {self.port} at {self.baudrate} baud")
        except serial.SerialException as e:
            logger.error(f"Failed to connect to {self.port}: {e}")
            sys.exit(1)

    def calculate_checksum(self, payload: list[int]) -> int:
        """Checksum = ~(ID + Length + Instr + Params...) & 0xFF"""
        total = sum(payload)
        return (~total) & 0xFF

    def read_joint_raw(self, servo_id: int) -> Optional[int]:
        """Reads raw position (0-4095) of a single servo."""
        if not self.ser:
            return None

        # Packet: [FF, FF, ID, Len, Instr, P1, P2, Checksum]
        # Length = Params(2) + 2 = 4
        # Instr = 0x02 (Read)
        # P1 = Address (56)
        # P2 = Read Length (2 bytes)
        
        pkt_len = 4
        payload = [servo_id, pkt_len, INSTR_READ, ADDR_PRESENT_POSITION, 2]
        checksum = self.calculate_checksum(payload)
        
        packet = HEADER + bytes(payload) + bytes([checksum])
        
        # Clear buffer to avoid sync issues
        self.ser.reset_input_buffer()
        self.ser.write(packet)
        
        # Expected Response: [FF, FF, ID, Len, Err, Low, High, Checksum]
        # Total 8 bytes
        try:
            response = self.ser.read(8)
            if len(response) != 8:
                # logger.debug(f"ID {servo_id}: Timeout or incomplete packet")
                return None
                
            if response[0] != 0xFF or response[1] != 0xFF:
                # logger.debug(f"ID {servo_id}: Invalid header")
                return None
                
            if response[2] != servo_id:
                return None
                
            # Verify checksum
            # Payload for checksum is [ID, Len, Err, P1, P2]
            resp_payload = list(response[2:7])
            calc_sum = self.calculate_checksum(resp_payload)
            if calc_sum != response[7]:
                logger.warning(f"ID {servo_id}: Checksum error")
                return None
            
            # Parse Position (Little Endian)
            low = response[5]
            high = response[6]
            raw_pos = (high << 8) | low
            
            # Mask to 12-bit just in case
            raw_pos &= 0xFFF
            
            return int(raw_pos)
            
        except Exception as e:
            logger.error(f"Serial Error: {e}")
            return None

    def close(self):
        if self.ser and self.ser.is_open:
            self.ser.close()

def main():
    parser = argparse.ArgumentParser(description="SO-101 Host Driver (Feetech)")
    parser.add_argument(
        "--profile",
        type=str,
        default=DEFAULT_PROFILE_PATH,
        help="LeRobot profile.json path",
    )
    parser.add_argument(
        "--port",
        type=str,
        default=PROFILE_PORT_SENTINEL,
        help="Serial port path ('__PROFILE__' = use profile.json)",
    )
    parser.add_argument("--baud", type=int, default=1000000, help="Baudrate (Default 1M)")
    parser.add_argument("--zmq-port", type=int, default=5555, help="ZeroMQ PUB port")
    parser.add_argument("--mock", action="store_true", help="Use mock data")
    args = parser.parse_args()

    # Resolve leader port + calibration from profile.json
    leader_port = args.port
    if leader_port == PROFILE_PORT_SENTINEL:
        leader_port = ""
    leader_calib = None

    if not args.mock:
        prof = find_profile(
            "so101_leader",
            profile_path=args.profile,
            fallback_port="/dev/ttyACM1",
        )
        if not leader_port:
            leader_port = prof.port

        if prof.calib_path:
            leader_calib = load_calibration(prof.calib_path)
        else:
            raise RuntimeError(
                "Leader calibration path missing in profile.json; "
                "run calibration first to avoid uncontrolled motion."
            )

        logger.info(f"Leader port: {leader_port}")
        logger.info(f"Leader calib: {prof.calib_path}")

    # ZMQ Setup
    context = zmq.Context()
    socket = context.socket(zmq.PUB)
    socket.setsockopt(zmq.LINGER, 0)
    socket.bind(f"tcp://*:{args.zmq_port}")
    logger.info(f"ZeroMQ Publisher bound to port {args.zmq_port}")

    # Arm Setup
    arm = None
    if not args.mock:
        arm = FeetechArm(leader_port, args.baud)
        arm.connect()

    logger.info("Starting bridge loop... (Press Ctrl+C to stop)")
    
    # Store last smoothed positions to avoid jitter
    # 注意：若一開始用 0 初始化，重啟 bridge 時會造成關節從 0 漸進到真實角度，
    # 使 Sim 端看到一段怪異軌跡。這裡改成「首次讀到值就直接 seed」。
    last_smoothed_joints = [0.0] * 6
    has_seed = [False] * 6
    alpha = 0.3 # Smoothing factor (0.0 to 1.0, lower is smoother)
    
    try:
        while True:
            start_time = time.time()
            current_joints = []
            
            if args.mock:
                t = start_time
                current_joints = [
                    np.sin(t) * 0.5,
                    np.cos(t) * 0.5,
                    np.sin(t * 0.5) * 0.3,
                    0.0,
                    0.0,
                    0.0
                ]
                time.sleep(0.02)
            else:
                # Read ID 1 to 6
                temp_joints = []
                
                if arm:
                    if leader_calib is None:
                        time.sleep(1)
                        continue

                    for joint_index, joint_name in enumerate(JOINT_NAMES, start=1):
                        raw = arm.read_joint_raw(joint_index)
                        if raw is None:
                            # If read fails, use last SMOOTHED value
                            temp_joints.append(last_smoothed_joints[joint_index - 1])
                            continue

                        calib = leader_calib[joint_name]
                        center_raw = calib.center_raw
                        radians = (raw - center_raw) * (2 * np.pi / POS_RESOLUTION)

                        if not has_seed[joint_index - 1]:
                            smoothed_val = radians
                            has_seed[joint_index - 1] = True
                        else:
                            smoothed_val = (
                                alpha * radians
                                + (1 - alpha) * last_smoothed_joints[joint_index - 1]
                            )

                        temp_joints.append(smoothed_val)
                        last_smoothed_joints[joint_index - 1] = smoothed_val
                else:
                    # Should not reach here unless logic error
                    time.sleep(1)
                    continue

                current_joints = temp_joints
                
                # Small sleep to prevent busy loop if reading is too fast
                # But serial read timeout acts as delay usually
                time.sleep(0.005)

            # Publish Data
            payload = {
                "timestamp": start_time,
                "joints": current_joints
            }
            socket.send_string(json.dumps(payload))
            
            # Debug log every 1 second
            if int(start_time * 10) % 10 == 0 and int(start_time*100)%5==0: 
               # logger.debug(f"Joints: {[round(x,2) for x in current_joints]}")
               pass

    except KeyboardInterrupt:
        logger.info("Stopping...")
    finally:
        if arm:
            arm.close()
        socket.close()
        context.term()

if __name__ == "__main__":
    main()
