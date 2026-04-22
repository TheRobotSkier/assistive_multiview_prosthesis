# Hardware Connectivity: Mia Hand + Wrist Motor

## Current Status
The Mia Hand and Wrist Dynamixel motor are now integrated into the ROS2 control framework:
- **Mia Hand**: Uses existing `mia_hand_driver` and `mia_hand_ros2_control` packages (RS-232 @ 115200 baud)
- **Wrist Motor**: Uses `dynamixel_hardware_interface` ROS2 control plugin (Protocol 2.0 @ 57600 baud)
- **Combined URDF**: Includes wrist rotation joint (revolute, +/-90°) between world and hand palm
- **Joints Controlled**: 5 total (thumb_fle, index_fle, mrl_fle, thumb_opp, wrist_rotation)

## Hardware Setup Steps

### 1. USB Device Configuration
```bash
# Identify devices (run when both are connected)
lsusb

# Example output:
# Bus 001 Device 005: ID 0403:6001 FTFT (Mia Hand adapter)
# Bus 001 Device 006: ID 10c4:ea60 CP210x (Dynamixel adapter)

# Create udev rules (replace IDs with your actual devices)
echo 'SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6001", SYMLINK+="mia_hand"' | sudo tee /etc/udev/rules.d/99-mia-hand.rules
echo 'SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", SYMLINK+="wrist_motor"' | sudo tee /etc/udev/rules.d/99-wrist-motor.rules

# Reload rules
sudo udevadm control --reload-rules && sudo udevadm trigger
```

### 2. WSL2 USB Passthrough (if applicable)
```bash
# On Windows (PowerShell as admin):
winget install usbipd
usbipd list  # Note bus IDs for both devices
usbipd bind --busid <BUSID_1>
usbipd bind --busid <BUSID_2>
usbipd attach --wsl --busid <BUSID_1>
usbipd attach --wsl --busid <BUSID_2>
```

### 3. Verify Device Access
```bash
# Inside container/WSL2:
ls -la /dev/mia_hand /dev/wrist_motor
# Should show symlinks to actual ttyUSB/ttyACM devices
```

### 4. Test Hardware Connection
```bash
# Test Mia Hand (existing service):
docker compose run --rm miahand_driver

# Test Wrist Motor (standalone test):
python3 test_dynamixel_motor.py --port /dev/wrist_motor --baud 57600 --motor-id 1

# Test Combined System:
docker compose run --rm miahand_hardware
# Then in another terminal:
ros2 topic pub /wrist_pos_ff_controller/commands std_msgs/msg/Float64MultiArray "{data: [0.0]}"
```

## Key Technical Details

### Mia Hand Interface
- **Serial**: `/dev/mia_hand` @ 115200 baud
- **Protocol**: ASCII commands (`@<14 chars>*\r`)
- **ROS2**: Services/actions via `mia_hand_driver`, ros2_control via `mia_hand_ros2_control`

### Wrist Motor Interface  
- **Serial**: `/dev/wrist_motor` @ 57600 baud
- **Protocol**: Dynamixel Protocol 2.0
- **Control Table**:
  - Torque Enable: Addr 64
  - Goal Position: Addr 116  
  - Goal Velocity: Addr 104
  - Present Position: Addr 132
  - Present Velocity: Addr 128
- **ROS2**: Uses `dynamixel_hardware_interface` plugin + custom messages/services

### Joint Configuration
- **Wrist Rotation**: Revolute joint, limits ±1.5708 rad (±90°)
- **Control**: Position, velocity, and trajectory controllers available
- **Topics**: 
  - Command: `/wrist_pos_ff_controller/commands` (Float64MultiArray)
  - State: `/joint_states` (includes wrist_rotation position)

## File Locations
- Combined URDF: `docker_ws/mia_hand_ros2_control/description/urdf/mia_hand_with_wrist_system_interface.urdf.xacro`
- Launch File: `docker_ws/mia_hand_ros2_control/launch/mia_hand_with_wrist_system_interface_launch.py`
- Controllers: `docker_ws/mia_hand_ros2_control/config/mia_hand_with_wrist_controllers.yaml`
- Hardware Service: `docker_ws/docker-deployment/docker-compose.yml` (miahand_hardware)
- Test Script: `test_dynamixel_motor.py` (standalone motor validation)

## Verification
- Build: `colcon build --packages-up-to dynamixel_sdk dynamixel_interfaces dynamixel_hardware_interface mia_hand_ros2_control`
- Mock Test: `USE_MOCK_HARDWARE=true` launch validates software stack
- Hardware Test: Connect devices and run `miahand_hardware` service