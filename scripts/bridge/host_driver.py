#!/usr/bin/env python3
"""
LeIsaac Host Driver (Sim-to-Real Bridge)

Reads joint states from Feetech STS/SCS Servos (SO-101 Leader Arm)
and publishes them via ZeroMQ.
"""

import sys
import time
import json
import logging
import argparse
import zmq
import serial
import struct
import numpy as np
from typing import Optional

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

    def read_joint(self, servo_id: int) -> Optional[float]:
        """
        Reads position of a single servo.
        Returns: Position in Radians or None if failed.
        """
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
            
            # Convert to Radians
            # Assuming 0-4096 maps to 0-2pi (approx)
            # Center is 2048
            # In Goro project: 2048 is Center (0 degrees)
            # Direction may vary per joint, but let's send raw radian mapping first.
            radians = (raw_pos - 2048) * (2 * np.pi / POS_RESOLUTION)
            return radians
            
        except Exception as e:
            logger.error(f"Serial Error: {e}")
            return None

    def close(self):
        if self.ser and self.ser.is_open:
            self.ser.close()

def main():
    parser = argparse.ArgumentParser(description="SO-101 Host Driver (Feetech)")
    parser.add_argument("--port", type=str, default="/dev/ttyACM0", help="Serial port path")
    parser.add_argument("--baud", type=int, default=1000000, help="Baudrate (Default 1M)")
    parser.add_argument("--zmq-port", type=int, default=5555, help="ZeroMQ PUB port")
    parser.add_argument("--mock", action="store_true", help="Use mock data")
    args = parser.parse_args()

    # ZMQ Setup
    context = zmq.Context()
    socket = context.socket(zmq.PUB)
    socket.bind(f"tcp://*:{args.zmq_port}")
    logger.info(f"ZeroMQ Publisher bound to port {args.zmq_port}")

    # Arm Setup
    arm = None
    if not args.mock:
        arm = FeetechArm(args.port, args.baud)
        arm.connect()

    logger.info("Starting bridge loop... (Press Ctrl+C to stop)")
    
    # Store last smoothed positions to avoid jitter
    last_smoothed_joints = [0.0] * 6
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
                    for i in range(1, 7):
                        val = arm.read_joint(i)
                        if val is None:
                            # If read fails, use last SMOOTHED value
                            temp_joints.append(last_smoothed_joints[i-1])
                        else:
                            # Apply Low-Pass Filter
                            smoothed_val = alpha * val + (1 - alpha) * last_smoothed_joints[i-1]
                            temp_joints.append(smoothed_val)
                            last_smoothed_joints[i-1] = smoothed_val
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
