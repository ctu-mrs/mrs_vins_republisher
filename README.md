
# MRS VINS republisher

Transformes the VIO odometry to the reference frame expected by the [EstimationManager](https://github.com/ctu-mrs/mrs_uav_managers).

## Usage

Use one of the launch files in the `launch/` folder, for example:
```
ros2 launch mrs_vins_republisher vins_republisher.launch.py
```
or create your own by copying an existing launch file and modifying the transformations.

## How it works

When we have multiple sources of the localization (GPS, VIO, SLAM...), each source's node publishes it's own TF frame representing position of the drone in the world frame according to that particular localization source. Thus we should have TF tree with multiple root nodes. However, in ROS, we cannot have multiple roots. We solve this by appending the TF tree of the each localization source under the main TF tree.

If we run the OpenVINS VIO node, it publishes its own tf tree, which is separated from the "main" tf tree:

<center><img src="separated_tf_trees.png" alt="image" width="100%"></center>

The "main" TF tree has the `uav1/fcu` frame at it's root. The root of the TF tree outputted by OpenVINS is `uav1/imu`. Here is the detail of the TF tree produced by OpenVINS:

<center><img src="vio_tf_tree.png" alt="image" width="33%"></center>

We have to connect those two TF trees together. Here is he complete, connected TF tree:

<center><img src="combined_tf_tree.png" alt="image" width="50%"></center>

We added static TF frame `uav1/vins_body_front` under the `uav1/fcu`. This transform represents position of the camera relaive to the FCU frame, a.k.a. camera mounting position/orientation. Another static TF frame is `uav1/imu` (connected to the `uav1/vins_body_front`), which OpenVINS is using as it's UAV body frame. OpenVINS is publishing the transform between `uav1/imu` and `uav1/global`, which is the actual localization.

Node `mrs_vins_republisher` is publishing transform from `uav1/global` to `uav1/mrs_vins_world`. Thos two frames have identical position but different orientaton. TODO: Why those frames have different orientation? What that orientation means?

## External dependencies

ROS2, mrs_lib
