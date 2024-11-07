
# SDV_CONTROL guide

### Getting started

**Note #1:** This module requires 2 launch files. The first one just needs to be launched once, as it also runs other modules besides the ***controllers***, like the ***localization*** one, which starts the Vectornav and takes some time to fix for gps. The second one is launched every time the car needs to do a route again.

**Note #2:** To change between simulation and real life, there is an argument in the launch files.

The *control_launch.py* runs the ***longitudinal control***, ***rviz***, ***tf2***, ***description***, ***localization***, and ***can*** modules.

The *guidance_launch.py* runs the ***waypoint handler*** and ***lateral control*** modules.

<br>

### What to run

```sh
# Remember to source workspace
source /workspace/install/setup.bash

# Longitudinal control launch file
ros2 launch sdv_control control_launch.py

# Lateral control launch file
ros2 launch sdv_control control_launch.py
```

<br>

### Relevant topics from nodes

| Node                | Topic                                 | Description                                                                   |
| ------------------- | ------------------------------------- | ----------------------------------------------------------------------------- |
| pid node            | **/sdc_control/control_signal/D**     | Control signal sent to the throttle accelerator pot                           |
| stanley controller  | **/sdc_control/control_signal/delta** | Required angle for the wheels to follow the desired path                      |
| stanley controller  | **/sdv/steering/setpoint**            | Delta converted to a setpoint for the encoder                                 |
| stanley controller  | **/sdv/steering/current_reference**   | Path msg that defines the current line (reference) for the Stanley controller |
| vectornav           | **/vectornav/velocity_body**          | Vehicle's velocity in body                                                    |
| lidar to odom       | **/odom_lidar**                       | Vehicle's NED position                                                        |