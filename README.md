# C++ Example: ROS Logger

This is an example that shows how to use Rerun's C++ API to log and visualize [ROS](https://www.ros.org/) messages. 

It works by subscribing to all topics with supported types, converting the messages, and logging the data to Rerun. It further allows to remap topic names to specific entity paths, specify additional timeless transforms, and pinhole parameters via an external config file. See the [launch](./launch/) directory for usage examples.

https://github.com/rerun-io/cpp-example-ros-bridge/assets/9785832/797741a6-ab00-4334-8a49-10b84d581bf8

This example is build for ROS 1. For more ROS examples, also check out the [ROS 2 example](https://www.rerun.io/docs/howto/ros2-nav-turtlebot), and the [URDF data-loader](https://github.com/rerun-io/rerun-loader-python-example-urdf).

> NOTE: Currently only `geometry_msgs/PoseStamped`, `nav_msgs/Odometry`, `sensor_msgs/Image`, and `sensor_msgs/Imu` are supported. However, extending to other messages should be straightforward. This node does not support tf, URDF, and `camera_info` out-of-the-box yet.

## Compile and run using pixi
The easiest way to get started is to install [pixi](https://prefix.dev/docs/pixi/overview).

The pixi environment described in `pixi.toml` contains all of the dependencies, including a sample rosbag from [here](https://fpv.ifi.uzh.ch/datasets/), and the Rerun viewer. This allows you to run the example with a single command:
```bash
pixi run example
```

## Compile and run using existing ROS environment
If you have an existing ROS workspace and would like to add the Rerun node to it, clone this repository into the workspace's `src` directory and build the workspace.
