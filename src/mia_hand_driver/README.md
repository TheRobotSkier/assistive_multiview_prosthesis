# mia_hand_driver

This package implements a generic ROS2 driver node for Mia Hand devices.

The Mia Hand functionalities that can be achieved through this driver are listed below, together with the related ROS2 interfaces.

## Connecting to Mia Hand

Connection with Mia Hand is attempted automatically when launching the ROS2 driver. This is done by specifying the `/dev/tty` serial port to which the hand is connected, as `serial_port` parameter to the `ros2 launch` command.

**Example:** if Mia Hand is connected to `\dev\ttyUSB1` port, the ROS2 driver node should be launched through `ros2 launch mia_hand_driver mia_hand_driver_launch.py serial_port:=\dev\ttyUSB1`.

The `\dev\tty` device to which Mia Hand is connected can be known through `sudo dmesg | grep tty` terminal command.

If no `serial_port` parameter is specified, connection is attempted by default through `\dev\ttyUSB0` port.

Upon successful connection, a "Mia Hand connected." INFO message should be displayed on the terminal window.

Together with this, connection to Mia Hand and disconnection from it can be performed, respectively, through the `mia_hand/connect` and `mia_hand/disconnect` services.

**Note:** Before launching the Mia Hand ROS2 driver, it is recommended to set a low latency for Mia Hand serial port. This can be done through the terminal command `setserial <port_name> low_latency`. If `setserial` is not installed, this can be done through `sudo apt install setserial`.

## Motors control

Mia Hand motors (thumb, index and mrl) can be controlled by setting target motor speeds or target motor trajectories. Below, each of these ways is described.

### Speed control

Target motor speeds can be set through the proper services, named `mia_hand/motors/<motor_name>/set_speed`. Such services exploit the [mia_hand_msgs/srv/SetMotorSpeed.srv](https://bitbucket.org/prensiliasrl/mia_hand_ros2_pkgs/src/master/mia_hand_msgs/srv/SetMotorSpeed.srv) interface.

Together with setting a target speed, the services allow to set a maximum current that should be absorbed by the motor during the motion at the target speed. This can be exploited to indirectly control the grasping force, if the motor speeds are set for objects graspings. However, care should be taken when setting too low maximum currents, since this might impede the motor to reach the target speed.

Motor speeds are expressed in [rounds/96ms], and range between [-90, 90], where positive speeds correspond to finger closing, while negative speeds correspond to finger opening. Motor currents are expressed in percentage, and range in 0-80%.

**Example:** to set a target thumb motor speed of 50 rounds/96ms, with 70% maximum motor current, the command would be `ros2 service call <any_namespace_added>/mia_hand/motors/thumb/set_speed mia_hand_msgs/srv/SetMotorSpeed "{target_speed: 50, max_current_percent: 70}"`.

### Trajectory control

Target motor trajectories can be set through the proper services, named `mia_hand/motors/<motor_name>/set_trajectory`. Such services exploit the [mia_hand_msgs/srv/SetMotorTraj.srv](https://bitbucket.org/prensiliasrl/mia_hand_ros2_pkgs/src/master/mia_hand_msgs/srv/SetMotorTraj.srv) interface.

Motor trajectories are set by means of a target position and a target speed/force percentage. The speed/force percentage allows, at the same time, to control the speed at which the motor reaches the target position, if no object is grasped in between, and the grasping force, if an object is grasped before reaching the target position.

Motor positions are expressed in adimensional values, that ranges between [0, 255], for thumb and mrl motors, and between [-255, 255], for index motor (negative positions correspond to index flexed, and thumb abducted). Speed/force percentages, instead, range in 0-99%.

**Example:** to send index motor to position 150, at 30% speed/force, the command would be `ros2 service call <any_namespace_added>/mia_hand/motors/index/set_trajectory mia_hand_msgs/srv/SetMotorTraj "{target_position: 150, spe_for_percent: 30}"`.

Target motor trajectories can also be set by means of actions, named `mia_hand/motors/<motor_name>/trajectory`. Such actions exploit the [mia_hand_msgs/action/MotorTraj.action](https://bitbucket.org/prensiliasrl/mia_hand_ros2_pkgs/src/master/mia_hand_msgs/action/MotorTraj.action) interface.

The action goals are sent by means of a target position and a target speed/force percent, as for the services. However, differently from the services, current motor position and speed are published as feedback during trajectory execution, at a 2Hz rate. Trajectory actions are terminated when the target position is reached by the motor.

Following the same example above, the command for setting the same trajectory to the index motor through an action, visualizing the feedback, would be `ros2 action send_goal <any_namespace_added>/mia_hand/motors/index/trajectory mia_hand_msgs/action/MotorTraj "{target_position: 150, spe_for_percent: 30}" --feedback`.

## Joints control

Together with Mia Hand motors, also the Mia hand joints can be controlled, by setting target joint speeds and target joint trajectories. Each of these functionalities is described below.

### Speed control

As for Mia Hand motors, also Mia hand joint speeds can be set through the proper services, named `mia_hand/joints/<joint_name>/set_speed`. Such services exploit the [mia_hand_msgs/srv/SetJointSpeed.srv](https://bitbucket.org/prensiliasrl/mia_hand_ros2_pkgs/src/master/mia_hand_msgs/srv/SetJointSpeed.srv) interface.

Also here, the services allow to set a target joint speed (positive for finger closing, negative for finger opening) and a maximum current that should be absorbed by the motor during the motion. However, in this case, the speed is expressed in [rad/s] and has an accuracy of 0.1 rad/s. So, target joint speeds are truncated to the first decimal digit.

**Example:** to set a target mrl joint speed of 0.8 rad/s, with 80% maximum motor current, the command would be `ros2 service call <any_namespace_added>/mia_hand/joints/mrl/set_speed mia_hand_msgs/srv/SetJointSpeed "{target_speed: 0.8, max_current_percent: 80}"`.

### Trajectory control

As for Mia Hand motors, also Mia Joint joint trajectories can be set through services and actions.

The services are named `mia_hand/joints/<joint_name>/set_trajectory` and exploit the [mia_hand_msgs/srv/SetJointTraj.srv](https://bitbucket.org/prensiliasrl/mia_hand_ros2_pkgs/src/master/mia_hand_msgs/srv/SetJointTraj.srv) interface. The actions are named `mia_hand/joints/<joint_name>/trajectory` and exploit the [mia_hand_msgs/action/JointTraj.action](https://bitbucket.org/prensiliasrl/mia_hand_ros2_pkgs/src/master/mia_hand_msgs/action/JointTraj.action) interface.

Also here, the trajectories are set by means of a target position and a target speed/force percent. However, the target position (i.e., the target joint angle) is expressed in [rad] and has an accuracy od 0.01 rad. So, target joint angles are truncated to the second decimal digit. The position is expressed in [rad] also in the actions feedback, where current joint angle and current motor speed are published at a 2Hz rate.

**Example:** to send, through service, index joint to -1.25 rad at 50% speed/force, the command would be `ros2 service call <any_namespace_added>/mia_hand/joints/index/set_trajectory mia_hand_msgs/srv/SetJointTraj "{target_angle: -1.25, spe_for_percent: 50}"`. Through action, reading the feedback, the command would instead be `ros2 action send_goal <any_namespace_added>/mia_hand/joints/index/trajectory mia_hand_msgs/action/JointTraj "{target_angle: -1.25, spe_for_percent: 50}" --feedback`.

**Note:** The range of motion of Mia Hand joints might slightly vary among Mia Hand devices. Therefore, the joint maximum and minimum angles might not be exacly the same among Mia Hand devices.

## Grasps

Together with allowing to control single Mia Hand motors/joints, the ROS2 driver allows to control all the motors at the same time, to perform pre-configured grasps.

Eight pre-configured grasps can be performed: cylindrical, pinch, lateral, point-up, point-down, relax, spherical and tridigital. Despite being included among the grasps, the point-up, point-down and relax are not meant to be used for actually grasping objects, but for performing related gestures. However, as better explained in the nest section, the grasp names are purely indicative of their factory configuration, but can be freely re-configured to meet any user preference.

As for motors and joints control, also grasps can be executed through dedicated services or actions.

The services are named `mia_hand/grasps/<grasp_name>/execute` and exploit the [mia_hand_msgs/srv/ExecuteGrasp.srv](https://bitbucket.org/prensiliasrl/mia_hand_ros2_pkgs/src/master/mia_hand_msgs/srv/ExecuteGrasp.srv) interface. The actions are named `mia_hand/grasps/<grasp_name>/action` and exploit the [mia_hand_msgs/action/Grasp.action](https://bitbucket.org/prensiliasrl/mia_hand_ros2_pkgs/src/master/mia_hand_msgs/action/Grasp.action) interface.

The grasps are set by means of a closure percentage and a speed/force percentage. 0% closure means that all the motors are driven to their OPEN grasp positions, while 100% closure means that all the motors are driven to their CLOSE grasp positions. OPEN and CLOSE motor positions can be freely reconfigured, to customize the hand grasps (see the next section for how to configure them). Speed/force percentage parameter allows to tune, at the same time, the grasp speed (when an object is not grasped) and the grasp force (when an object is grasped), as for what happens for trajectory controls.

If grasps are performed through actions, the feedback, published at a 2Hz rate, is given in terms of current motor positions, current motor speeds and current force sensor outputs, both for normal and tangential sensors.

**Example:** to close, through service, a cylindrical grasp at 70%, with 50% speed/force, the command would be `ros2 service call <any_namespace_added>/mia_hand/grasps/cylindrical/execute mia_hand_msgs/srv/ExecuteGrasp "{close_percent: 70, spe_for_percent: 50}"`. To do that through action, by collecting also the feedback, the command would instead be `ros2 action send_goal <any_namespace_added>/mia_hand/grasps/cylindrical/action mia_hand_msgs/action/Grasp "{close_percent: 70, spe_for_percent: 50}" --feedback`.

## Grasps configuration

Mia Hand grasps are defined through OPEN and CLOSE reference motor positions and are freely re-configurable. Re-configuring a grasp, thus, is done by setting the related OPEN and CLOSE positions for each motor.

Cylindrical, pinch, lateral, point-up, point-down and relax grasps come up with a factory pre-configuration, that allows them to be directly performed even without re-configuration. Spherical and tridigital grasps, instead, come up as "empty" additional grasps, that do not trigger any hand motion if not re-configured.

The ROS2 driver allows to re-configure the grasp OPEN and CLOSE references through dedicated services, named `mia_hand/grasps/<grasp_name>/set_references`. These services exploit the [mia_hand_msgs/srv/SetGraspReferences.srv](https://bitbucket.org/prensiliasrl/mia_hand_ros2_pkgs/src/master/mia_hand_msgs/srv/SetGraspReferences.srv) interface.

OPEN and CLOSE motor positions are expressed in adimensional units, ranging in [0, 255] for thumb and mrl motors, and in [-255, 255] for index motor.

**Example:** to set the cylindrical grasp thumb OPEN and CLOSE references to, respectively, 20 and 110, the command would be `ros2 service call <any_namespace_added>/mia_hand/grasps/cylindrical/set_references mia_hand_msgs/srv/SetGraspReferences "{motor: 0, open_mpos: 20, close_mpos: 110}"`.

It is possible also to read the current grasp references through dedicated services, named `mia_hand/grasps/<grasp_name>/get_references`. These services exploit the [mia_hand_msgs/srv/GetGraspReferences.srv](https://bitbucket.org/prensiliasrl/mia_hand_ros2_pkgs/src/master/mia_hand_msgs/srv/GetGraspReferences.srv) interface.

Once re-configured any grasp references, the new settings are lost when the hand is switched-off, unless they are permanently stored inside hand internal memory. To do so, the service `mia_hand/store_current_settings` should be called.

## Motor positions calibration

During Mia Hand usage, some misalignments in motor positions might be observed.

As an example, a completely closed finger might not correspond to motor position 255.

These misaligments usually happen when Mia Hand is switched off while moving, and can affect the accuracy of the trajectory and grasp controls. When this happens, motor positions can be re-calibrated through the `mia_hand/motors/calibrate_positions` service.

A successful motor positions re-calibration is signaled through a green light on the hand back. After that, motor positions should be realigned in the interval [0, 255] (or [-255, 255], for index motor).

## Data reading

Mia Hand motor/joint positions, motor/joint speeds, motor currents and force sensor outputs can be read at any moment, through dedicated services. Moreover, those data can be streamed over dedicated topics.

The service names for reading the above data are:  

* `mia_hand/motors/get_positions` for motor positions.  
* `mia_hand/motors/get_speeds` for motor speeds.  
* `mia_hand/motors/get_currents` for motor currents.  
* `mia_hand/joints/get_positions` for joint positions.  
* `mia_hand/joint/get_speeds` for joint speeds.   
* `mia_hand/fingers/get_forces` for force sensor outputs.

The services for reading motor data exploit the [mia_hand_msgs/srv/GetMotordata.srv](https://bitbucket.org/prensiliasrl/mia_hand_ros2_pkgs/src/master/mia_hand_msgs/srv/GetMotorData.srv) interface. The services for reading joint data exploit the [mia_hand_msgs/srv/GetJointdata.srv](https://bitbucket.org/prensiliasrl/mia_hand_ros2_pkgs/src/master/mia_hand_msgs/srv/GetJointData.srv) interface. The service for reading force sensor outputs, instead, exploits the [mia_hand_msgs/srv/GetForcedata.srv](https://bitbucket.org/prensiliasrl/mia_hand_ros2_pkgs/src/master/mia_hand_msgs/srv/GetForceData.srv) interface.

To stream, instead, the above data over the dedicated topics, the related streams needs to be enabled through the following services:

* `mia_hand/data_streams/motors/positions/switch`, for motor positions.  
* `mia_hand/data_streams/motors/speeds/switch`, for motor speeds.  
* `mia_hand/data_streams/motors/currents/switch`, for motor currents.  
* `mia_hand/data_streams/joints/positions/switch`, for joint positions.  
* `mia_hand/data_streams/joints/speeds/switch`, for joint speeds.  
* `mia_hand/data_streams/fingers/forces/switch`, for force sensor outputs.

These services allow both to switch on and switch off the data streams. Once enabled the related streams, the data are published at a fixed rate over the following topics:

* `mia_hand/data_streams/motors/positions/data`, for motor positions.  
* `mia_hand/data_streams/motors/speeds/data`, for motor speeds.  
* `mia_hand/data_streams/motors/currents/data`, for motor currents.  
* `mia_hand/data_streams/joints/positions/data`, for joint positions.  
* `mia_hand/data_streams/joints/speeds/data`, for joint speeds.  
* `mia_hand/data_streams/fingers/forces/data`, for force sensor outputs.

The rate at which the data are published varies depending on the number of active data streams. Such rate is equal to 100 Hz / n_active_streams.

## Mia Hand stop

A stop service, named `mia_hand/stop` is also available. When called, this service stops all the Mia Hand motors and puts Mia Hand in a "stopped" state.

Under this state, any motion command sent to Mia Hand through the ROS2 driver is ignored, until the `mia_hand/play` service is called. This service re-establishes Mia Hand normal operation after a stop.
