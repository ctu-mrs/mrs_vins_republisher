
# MRS VINS republisher

Transformes the VIO odometry to the reference frame expected by the [EstimationManager](https://github.com/ctu-mrs/mrs_uav_managers).

## Usage

Use one of the launch files in the `launch/` folder, for example:
```
ros2 launch mrs_vins_republisher vins_republisher.launch.py
```
or create your own by copying an existing launch file and modifying the transformations.

## External dependencies

ROS2, mrs_lib
