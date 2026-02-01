# Server script is now intended to run inside Isaac Sim container
# It uses the built-in python environment of Isaac Sim
# But we need to install pyzmq there first.

import zmq
import json
import logging
import numpy as np
# from isaacsim import SimulationApp # Deferred import in main

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("SimServer")

def main():
    # 1. Start Isaac Sim
    from isaacsim import SimulationApp
    kit = SimulationApp({"headless": False})

    from omni.isaac.core import World
    from omni.isaac.core.articulations import Articulation
    from omni.isaac.core.utils.stage import add_reference_to_stage
    import carb
    
    # 2. Setup ZMQ Subscriber
    context = zmq.Context()
    socket = context.socket(zmq.SUB)
    # Connect to Host (Gateway IP in Docker default bridge is usually 172.17.0.1, 
    # but with network_mode: host, we use localhost)
    socket.connect("tcp://127.0.0.1:5555")
    socket.setsockopt_string(zmq.SUBSCRIBE, "")
    
    logger.info("Connected to ZMQ Publisher")

    # 3. Setup World & Robot
    world = World()
    
    # Use Franka for testing
    from omni.isaac.franka import Franka
    robot = Franka(prim_path="/World/Franka", name="franka")
    world.scene.add(robot)
    
    # Original SO-101 logic (commented out)
    # asset_path = "/isaac-sim/assets/so101.usd" 
    # add_reference_to_stage(usd_path=asset_path, prim_path="/World/SO101")
    # robot = Articulation(prim_path="/World/SO101", name="so101")
    
    world.scene.add_default_ground_plane()
    
    world.reset()

    # 4. Main Loop
    while kit.is_running():
        world.step(render=True)
        
        # Non-blocking ZMQ read
        try:
            # Read all pending messages, take the latest one
            latest_msg = None
            while True:
                try:
                    latest_msg = socket.recv_string(flags=zmq.NOBLOCK)
                except zmq.Again:
                    break
            
            if latest_msg:
                data = json.loads(latest_msg)
                joints = data.get("joints", [])
                
                # Apply to Franka (Franka has 9 joints including gripper, SO-101 likely has 6)
                # Padding joint array to match Franka's DOF if needed
                if robot:
                     # Basic 6-DOF mapping
                    if len(joints) >= 6:
                        # Franka expects 9 DOFs (7 arm + 2 gripper)
                        # Mapping first 6 joints directly for visual feedback
                        target_joints = np.zeros(9)
                        target_joints[0:6] = joints[0:6]
                        robot.set_joint_positions(target_joints)
                
                # Print debug info (throttle)
                if int(data["timestamp"] * 10) % 20 == 0:
                    print(f"Received Joints: {joints}")
                    
        except Exception as e:
            logger.error(f"Error in loop: {e}")

    kit.close()

if __name__ == "__main__":
    main()
