# C++ Example: ROS Bridge

This is an example that shows how to use Rerun's C++ API to log and visualize [ROS](https://www.ros.org/) messages. 

It works by subscribing to all topics with supported types, converting the messages, and logging the data to Rerun. It further allows to remap topic names to specific entity paths, specify additional timeless transforms, and pinhole parameters via an external config file. See the [launch](https://github.com/rerun-io/cpp-example-ros-bridge/tree/main/rerun_bridge/launch) directory for usage examples.

https://github.com/rerun-io/cpp-example-ros-bridge/assets/9785832/fcdf62ad-89f3-47c6-8996-9bc88a5bfb70

This example is built for ROS 1. For more ROS examples, also check out the [ROS 2 example](https://www.rerun.io/docs/howto/ros2-nav-turtlebot), and the [URDF data-loader](https://github.com/rerun-io/rerun-loader-python-example-urdf).

> NOTE: Currently only `geometry_msgs/{PoseStamped,TransformStamped}`, `nav_msgs/Odometry`,  `tf2_msgs/TFMessage`, and `sensor_msgs/{Image,CameraInfo,Imu}` are supported. However, extending to other messages should be straightforward.

## Compile and run using pixi
The easiest way to get started is to install [pixi](https://prefix.dev/docs/pixi/overview).

The pixi environment described in `pixi.toml` contains all required dependencies, including rosbags, and the Rerun viewer. To run the [drone example](https://fpv.ifi.uzh.ch/datasets/) use
```bash
pixi run drone_example
```
and to run the [Spot example](http://ptak.felk.cvut.cz/darpa-subt/qualification_videos/spot/) use
```bash
pixi run spot_example
```

## Compile and run using existing ROS environment
If you have an existing ROS workspace and would like to add the Rerun node to it, clone this repository into the workspace's `src` directory and build the workspace.
