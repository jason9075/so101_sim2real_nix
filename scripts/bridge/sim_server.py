# Server script is now intended to run inside Isaac Sim container
# It uses the built-in python environment of Isaac Sim
# But we need to install pyzmq there first.

import zmq
import json
import os
import logging
import numpy as np
# from isaacsim import SimulationApp # Deferred import in main

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("SimServer")

def fix_robot_base(stage, robot_prim_path):
    """
    Fix the robot base to the world using a PhysicsFixedJoint.
    This prevents the robot from falling over without breaking the ArticulationView.
    """
    from pxr import UsdPhysics, Gf
    
    # 1. Find the base link (usually the first rigid body)
    base_link_prim = None
    prim = stage.GetPrimAtPath(robot_prim_path)
    
    # Simple search for base_link or the first RigidBody
    # Often named "base_link", "base", or the root itself
    candidates = [prim] + list(prim.GetChildren())
    for p in candidates:
        if p.HasAPI(UsdPhysics.RigidBodyAPI):
            base_link_prim = p
            break
            
    if not base_link_prim:
        # Deep search if not found at top level
        from pxr import Usd
        for p in Usd.PrimRange(prim):
             if p.HasAPI(UsdPhysics.RigidBodyAPI):
                base_link_prim = p
                break
    
    if not base_link_prim:
        logger.warning(f"Could not find a RigidBody to fix in {robot_prim_path}")
        return

    logger.info(f"Fixing robot base via FixedJoint on: {base_link_prim.GetPath()}")

    # 2. Create a Fixed Joint connecting World to Base Link
    joint_path = f"{base_link_prim.GetPath()}/root_joint"
    if not stage.GetPrimAtPath(joint_path).IsValid():
        fixed_joint = UsdPhysics.FixedJoint.Define(stage, joint_path)
        
        # Body 1 is the robot base
        fixed_joint.CreateBody1Rel().SetTargets([base_link_prim.GetPath()])
        
        # Body 0 is implicit World (empty target) or explict world
        # Leaving Body0 empty implies attaching to the static world frame at local pos (0,0,0)
        
        # Ensure transform is identity
        fixed_joint.CreateLocalPos0Attr().Set(Gf.Vec3f(0,0,0))
        fixed_joint.CreateLocalRot0Attr().Set(Gf.Quatf(1,0,0,0))
        fixed_joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0,0,0))
        fixed_joint.CreateLocalRot1Attr().Set(Gf.Quatf(1,0,0,0)) 

def configure_joint_drives(robot):
    """
    Set stiffness and damping for all joints to prevent collapsing
    """
    # High stiffness for position control
    kP = 100000.0 
    kD = 1000.0
    
    try:
        # Debug info for mapping
        logger.info(f"Robot DOF Names: {robot.dof_names}")
        logger.info(f"Robot Num DOF: {robot.num_dof}")

        # Set gains on the Articulation Controller (Runtime)
        robot.get_articulation_controller().set_gains(
            kps=np.full(robot.num_dof, kP),
            kds=np.full(robot.num_dof, kD)
        )
        logger.info(f"Set joint gains for {robot.name}: kP={kP}, kD={kD}")
        
        # [CRITICAL] Also enforce Drive API on USD prims directly
        # If the USD file has stiffness=0, the physics engine might ignore the controller initially.
        from pxr import UsdPhysics, Usd
        
        stage = robot.prim.GetStage()
        for joint_path in robot.dof_paths:
            joint_prim = stage.GetPrimAtPath(joint_path)
            if joint_prim.IsValid() and joint_prim.HasAPI(UsdPhysics.DriveAPI):
                # Usually "angular" for revolute joints
                drive = UsdPhysics.DriveAPI.Get(joint_prim, "angular")
                if not drive:
                    # Try linear for prismatic if any
                    drive = UsdPhysics.DriveAPI.Get(joint_prim, "linear")
                
                if drive:
                    drive.CreateStiffnessAttr(kP)
                    drive.CreateDampingAttr(kD)
                    logger.info(f"Enforced USD Drive API stiffness on {joint_path}")

    except Exception as e:
        logger.warning(f"Failed to set joint gains: {e}")

def main():
    # 1. Start Isaac Sim with WebRTC support if enabled
    from isaacsim import SimulationApp
    
    # Check environment variables for streaming configuration
    headless = os.getenv("HEADLESS", "False").lower() == "true"
    enable_webrtc = os.getenv("ENABLE_WEBRTC", "False").lower() == "true"
    
    config = {
        "headless": headless,
        "width": 1280,
        "height": 720,
        "experience": "/isaac-sim/apps/isaacsim.exp.full.kit", # Use full editor experience
    }
    
    if enable_webrtc:
        # In 5.0.0, "webrtc" string is often more reliable than integer 1
        config["livestream"] = "webrtc"
        logger.info("Enabling WebRTC Livestreaming with Full Editor (Isaac Sim 5.0.0+)")

    # Launch SimulationApp with explicit settings
    kit = SimulationApp(config)
    
    # Manually ensure the WebRTC extension is enabled if not already
    if enable_webrtc:
        from omni.isaac.core.utils.extensions import enable_extension
        enable_extension("omni.kit.livestream.webrtc")
        # Optional: Also enable the services transport for better compatibility
        enable_extension("omni.services.transport.server.http")

    from omni.isaac.core import World
    from omni.isaac.core.articulations import Articulation
    from omni.isaac.core.utils.stage import add_reference_to_stage, get_current_stage
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
    
    # Check for SO-101 Follower USD
    asset_path = "/isaac-sim/assets/so101_follower.usd"
    robot = None
    is_custom_robot = False
    
    if os.path.exists(asset_path):
        logger.info(f"Found custom asset at {asset_path}. Loading SO-101...")
        add_reference_to_stage(usd_path=asset_path, prim_path="/World/SO101")
        
        # [Corrected Logic] Fix Root Link Kinematics
        # We need to make sure the root link is fixed to the world
        stage = get_current_stage()
        fix_robot_base(stage, "/World/SO101")
        
        robot = Articulation(prim_path="/World/SO101", name="so101")
        world.scene.add(robot)
        is_custom_robot = True
    else:
        logger.warning(f"Asset not found at {asset_path}. Fallback to Franka.")
        from omni.isaac.franka import Franka
        robot = Franka(prim_path="/World/Franka", name="franka")
        world.scene.add(robot)
    
    world.scene.add_default_ground_plane()
    
    # Reset triggers initialization
    try:
        world.reset()
    except Exception as e:
        logger.error(f"World Reset Failed: {e}")
        return

    # Apply Controller Gains AFTER Reset (Runtime)
    if is_custom_robot:
        pass
        # configure_joint_drives(robot)
    
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
                
                if robot:
                    # Determine DOF based on robot type
                    num_dof = robot.num_dof
                    
                    if num_dof == 9: # Franka (Fallback)
                        if len(joints) >= 6:
                            target_joints = np.zeros(9)
                            target_joints[0:6] = joints[0:6]
                            robot.set_joint_positions(target_joints)
                    
                    elif num_dof == 6: # SO-101 (Likely)
                        if len(joints) >= 6:
                            robot.set_joint_positions(np.array(joints[:6]))
                            
                    else: # Unknown or partial match
                         if len(joints) <= num_dof:
                             robot.set_joint_positions(np.array(joints))
                         else:
                             robot.set_joint_positions(np.array(joints[:num_dof]))
                
                # Print debug info (throttle)
                if int(data["timestamp"] * 10) % 20 == 0:
                    print(f"Received Joints: {joints}")
                    
        except Exception as e:
            logger.error(f"Error in loop: {e}")

    kit.close()

if __name__ == "__main__":
    main()
