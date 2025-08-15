# IK Solver Troubleshooting Guide

## Issues Fixed

### 1. IK Service Not Available Error

**Problem**: The code was saying "IK service not available, waiting" because:
- MoveIt was not running or properly configured
- The service name `/compute_ik` was hardcoded and might not exist
- No fallback mechanism was provided

**Solutions Implemented**:

1. **Improved Service Discovery**: The code now tries multiple service names:
   - `/compute_ik`
   - `/plan_kinematic_path`
   - `/move_group/plan_kinematic_path`

2. **Better Error Handling**: Clear error messages when services are not available

3. **Alternative IK Solver**: Created `arm_ik_solver_ikpy.py` using the ikpy library as fallback

### 2. Non-blocking Service Calls

**Problem**: The original code used blocking calls that could freeze the node

**Solution**: Implemented async service calls with callback handling

## Available Nodes

### 1. `arm_ik_solver_kinematics.py` (MoveIt-based)
- Uses MoveIt's IK service
- Tries multiple service names
- Handles service unavailability gracefully

### 2. `arm_ik_solver_ikpy.py` (IKPy-based)
- Uses ikpy library for IK computation
- Works without MoveIt
- Creates a 5-DOF arm model

### 3. `service_checker.py` (Diagnostic tool)
- Lists all available ROS 2 services
- Identifies MoveIt-related services

## Usage Instructions

### Step 1: Check Available Services
```bash
# Build the package first
cd /home/zenhar/hor/RoboticArmSim_Dockerized/workspace
colcon build --packages-select arm_ik_solver

# Source the workspace
source install/setup.bash

# Check what services are available
ros2 run arm_ik_solver service_checker
```

### Step 2A: If MoveIt Services are Available
```bash
# Use the MoveIt-based solver
ros2 run arm_ik_solver kinematics
```

### Step 2B: If MoveIt Services are NOT Available
```bash
# Use the ikpy-based solver
ros2 run arm_ik_solver kinematics_ikpy
```

### Step 3: Test with Target Publisher
```bash
# In another terminal, publish target positions
ros2 run arm_ik_solver target
```

## Troubleshooting Steps

1. **First, check if ROS 2 is properly sourced**:
   ```bash
   echo $ROS_DISTRO
   # Should show your ROS 2 distribution (humble, iron, etc.)
   ```

2. **Check if MoveIt is running**:
   ```bash
   ros2 service list | grep -i moveit
   ```

3. **If MoveIt is not running, start your robot simulation first**:
   ```bash
   # Your robot launch command here
   ros2 launch your_robot_package your_launch_file.py
   ```

4. **Install ikpy if not available**:
   ```bash
   pip install ikpy
   ```

## Configuration Notes

- **URDF Path**: Update the URDF path in `arm_ik_solver_ikpy.py` if your URDF is in a different location
- **Frame Names**: Update `base_frame` and `end_effector_link` to match your robot's frame names
- **Move Group**: Update `move_group_name` to match your MoveIt configuration

## Next Steps

1. Run the service checker to see what's available
2. Use the appropriate IK solver based on your setup
3. If neither works, we may need to:
   - Fix your MoveIt configuration
   - Adjust the robot model parameters
   - Check ROS 2 environment setup
