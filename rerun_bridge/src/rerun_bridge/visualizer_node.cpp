#include "visualizer_node.hpp"
#include "rerun_bridge/rerun_ros_interface.hpp"

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/master.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <yaml-cpp/yaml.h>

RerunLoggerNode::RerunLoggerNode() {
    _rec.spawn().exit_on_failure();

    // Read additional config from yaml file
    // NOTE We're not using the ROS parameter server for this, because roscpp doesn't support
    //   reading nested data structures.
    std::string yaml_path;
    if (_nh.getParam("yaml_path", yaml_path)) {
        ROS_INFO("Read yaml config at %s", yaml_path.c_str());
    }
    _read_yaml_config(yaml_path);
}

std::string RerunLoggerNode::_topic_to_entity_path(const std::string& topic) const {
    if (_topic_to_entity_path_map.find(topic) != _topic_to_entity_path_map.end()) {
        return _topic_to_entity_path_map.at(topic);
    } else {
        return topic;
    }
}

/// Convert a topic name to its immediate namespace.
/// E.g. "/parent/camera/camera_info" -> "/parent/camera"
std::string RerunLoggerNode::_topic_to_namespace(const std::string& topic) const {
    auto last_slash = topic.rfind('/');
    if (last_slash == std::string::npos) {
        return "";
    }
    return topic.substr(0, last_slash);
}

void RerunLoggerNode::_read_yaml_config(std::string yaml_path) {
    const YAML::Node config = YAML::LoadFile(yaml_path);

    // see https://www.rerun.io/docs/howto/ros2-nav-turtlebot#tf-to-rrtransform3d
    if (config["topic_to_entity_path"]) {
        _topic_to_entity_path_map =
            config["topic_to_entity_path"].as<std::map<std::string, std::string>>();

        for (auto const& [key, val] : _topic_to_entity_path_map) {
            ROS_INFO("Mapping topic %s to entity path %s", key.c_str(), val.c_str());
        }
    }
    if (config["extra_transform3ds"]) {
        for (const auto& extra_transform3d : config["extra_transform3ds"]) {
            const std::array<float, 3> translation = {
                extra_transform3d["transform"][3].as<float>(),
                extra_transform3d["transform"][7].as<float>(),
                extra_transform3d["transform"][11].as<float>()
            };
            // Rerun uses column-major order for Mat3x3
            const std::array<float, 9> mat3x3 = {
                extra_transform3d["transform"][0].as<float>(),
                extra_transform3d["transform"][4].as<float>(),
                extra_transform3d["transform"][8].as<float>(),
                extra_transform3d["transform"][1].as<float>(),
                extra_transform3d["transform"][5].as<float>(),
                extra_transform3d["transform"][9].as<float>(),
                extra_transform3d["transform"][2].as<float>(),
                extra_transform3d["transform"][6].as<float>(),
                extra_transform3d["transform"][10].as<float>()
            };
            _rec.log_timeless(
                extra_transform3d["entity_path"].as<std::string>(),
                rerun::Transform3D(
                    rerun::Vec3D(translation),
                    rerun::Mat3x3(mat3x3),
                    extra_transform3d["from_parent"].as<bool>()
                )
            );
        }
    }
    if (config["extra_pinholes"]) {
        for (const auto& extra_pinhole : config["extra_pinholes"]) {
            // Rerun uses column-major order for Mat3x3
            const std::array<float, 9> image_from_camera = {
                extra_pinhole["image_from_camera"][0].as<float>(),
                extra_pinhole["image_from_camera"][3].as<float>(),
                extra_pinhole["image_from_camera"][6].as<float>(),
                extra_pinhole["image_from_camera"][1].as<float>(),
                extra_pinhole["image_from_camera"][4].as<float>(),
                extra_pinhole["image_from_camera"][7].as<float>(),
                extra_pinhole["image_from_camera"][2].as<float>(),
                extra_pinhole["image_from_camera"][5].as<float>(),
                extra_pinhole["image_from_camera"][8].as<float>(),
            };
            _rec.log_timeless(
                extra_pinhole["entity_path"].as<std::string>(),
                rerun::Pinhole(image_from_camera)
                    .with_resolution(
                        extra_pinhole["width"].as<int>(),
                        extra_pinhole["height"].as<int>()
                    )
            );
        }
    }
}

void RerunLoggerNode::_create_subscribers() {
    ROS_INFO("Creating subscribers for new topics");
    // NOTE We are currently checking in each iteration if there are new topics.
    //   This is not efficient, but it's the easiest way to support new topics being added at runtime.
    //   If you have a lot of topics, you might want to optimize this.
    ros::master::V_TopicInfo topic_infos;
    ros::master::getTopics(topic_infos);
    for (const auto& topic_info : topic_infos) {
        if (_topic_to_subscriber.find(topic_info.name) == _topic_to_subscriber.end()) {
            auto entity_path = _topic_to_entity_path(topic_info.name);
            if (topic_info.datatype == "sensor_msgs/Image") {
                _topic_to_subscriber[topic_info.name] = _nh.subscribe<sensor_msgs::Image>(
                    topic_info.name,
                    100,
                    [&, entity_path](const sensor_msgs::Image::ConstPtr& msg) {
                        log_image(_rec, entity_path, msg);
                    }
                );
            } else if (topic_info.datatype == "sensor_msgs/Imu") {
                _topic_to_subscriber[topic_info.name] = _nh.subscribe<sensor_msgs::Imu>(
                    topic_info.name,
                    100,
                    [&, entity_path](const sensor_msgs::Imu::ConstPtr& msg) {
                        log_imu(_rec, entity_path, msg);
                    }
                );
            } else if (topic_info.datatype == "geometry_msgs/PoseStamped") {
                _topic_to_subscriber[topic_info.name] =
                    _nh.subscribe<geometry_msgs::PoseStamped>(
                        topic_info.name,
                        100,
                        [&, entity_path](const geometry_msgs::PoseStamped::ConstPtr& msg) {
                            log_pose_stamped(_rec, entity_path, msg);
                        }
                    );
            } else if (topic_info.datatype == "nav_msgs/Odometry") {
                _topic_to_subscriber[topic_info.name] = _nh.subscribe<nav_msgs::Odometry>(
                    topic_info.name,
                    100,
                    [&, entity_path](const nav_msgs::Odometry::ConstPtr& msg) {
                        log_odometry(_rec, entity_path, msg);
                    }
                );
            } else if (topic_info.datatype == "sensor_msgs/CameraInfo") {
                // If the camera_info topic has not been explicility mapped to an entity path,
                // we assume that the camera_info topic is a sibling of the image topic, and
                // hence use the namespace of the image topic as the entity path.
                if (_topic_to_entity_path_map.find(topic_info.name) ==
                    _topic_to_entity_path_map.end()) {
                    entity_path = _topic_to_namespace(topic_info.name);
                }
                _topic_to_subscriber[topic_info.name] = _nh.subscribe<sensor_msgs::CameraInfo>(
                    topic_info.name,
                    100,
                    [&, entity_path](const sensor_msgs::CameraInfo::ConstPtr& msg) {
                        log_camera_info(_rec, entity_path, msg);
                    }
                );
            }
        }
    }
}

void RerunLoggerNode::spin() {
    // check for new topics every 0.1 seconds
    ros::Timer timer = _nh.createTimer(
        ros::Duration(0.1),
        [&](const ros::TimerEvent&) {
            _create_subscribers();
        }
    );

    ros::MultiThreadedSpinner spinner(8); // Use 8 threads
    spinner.spin();
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "rerun_logger_node");
    RerunLoggerNode node;
    node.spin();
    return 0;
}
