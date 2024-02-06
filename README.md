# C++ Example: ROS Logger

This is an example that shows how to use Rerun's C++ API to log and visualize ROS messages.

You can download a sample .bag file from ...

> NOTE: So far this is a minimal example only supporting IMU messages, poses, and images. However, extending to other messages should be straightforward.

## Compile and run using pixi
The easiest way to get started is to install [pixi](https://prefix.dev/docs/pixi/overview).

The pixi environment described in `pixi.toml` contains all of the dependencies, including a sample rosbag, and the Rerun viewer  allowing you to run the example with a single command:
```bash
pixi run example
```

## Compile and run using existing ROS environment
If you have an existing ROS workspace and would like to add the Rerun node to it, you only need the `rerun_logger` package from this repository.
```
```
