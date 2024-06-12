# odometry_transformer
A ROS package to transform odometry messages from source frame `S` to target frame `T` on a rigid body. For example this software transforms odometry from a tracking camera coordinate frame to a robot body frame.
```
Input: source odometry (T_IS, v_S, w_S), calibration source to target (T_ST)
Output: target odometry (T_IT, v_T, w_T)
```
![Problem description of transforming odometry on a rigid body.](https://user-images.githubusercontent.com/11293852/161266750-58eec6a4-a13b-447c-b990-9517a6e6c90d.png)

## Features
- Given known offset and rotation between odometry source and target frame, transform and publish odometry in target frame.
- Pass sensor calibration either by ROS parameter server or TF tree.
- Dynamically re-configure offset and orientation via [rqt_reconfigure](http://wiki.ros.org/rqt_reconfigure).
- Integration as ROS node, all parameters accessible as [roslaunch arguments](launch/odometry_transformer.launch).

## How to use.
The [odometry_transformer.launch](launch/odometry_transformer.launch) file shows how to startup the ROS node and pass all relevant parameters. The user can pass the calibration between the odometry source frame and odometry target frame either via parameter array or via [tf2_ros static_transform_publisher](http://wiki.ros.org/tf2_ros).
```
roslaunch odometry_transformer odometry_transformer.launch arg:=value
```

## Example
Convert the odometry (position, orientation, linear and angular velocity) measured by the [Realsense Tracking Camera T265](https://www.intelrealsense.com/tracking-camera-t265/) from the camera odometry frame to the robot base frame. For startup details see the [test_realsense_t265.launch](test/launch/test_realsense_t265.launch) file.
```
roslaunch odometry_transformer test_realsense_t265.launch
```
![Odometry transformation of Realsense Tracking Camera T265.](https://user-images.githubusercontent.com/11293852/161270225-cc7c778b-d4d2-406a-bf67-f643ed6784f6.gif)
