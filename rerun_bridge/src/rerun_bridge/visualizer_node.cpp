#include "visualizer_node.hpp"
#include "rerun_bridge/rerun_ros_interface.hpp"

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/master.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <tf2_msgs/TFMessage.h>
#include <algorithm>

std::string parent_entity_path(const std::string& entity_path) {
    auto last_slash = entity_path.rfind('/');
    if (last_slash == std::string::npos) {
        return "";
    }
    return entity_path.substr(0, last_slash);
}

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

/// Convert a topic name to its entity path.
/// If the topic is explicitly mapped to an entity path, use that.
/// Otherwise, the topic name will be automatically converted to a flattened entity path like this:
///   "/one/two/three/four" -> "/topics/one-two-three/four"
std::string RerunLoggerNode::_resolve_entity_path(const std::string& topic) const {
    if (_topic_to_entity_path.find(topic) != _topic_to_entity_path.end()) {
        return _topic_to_entity_path.at(topic);
    } else {
        std::string flattened_topic = topic;
        auto last_slash =
            (std::find(flattened_topic.rbegin(), flattened_topic.rend(), '/') + 1).base();

        if (last_slash != flattened_topic.begin()) {
            // keep leading slash and last slash
            std::replace(flattened_topic.begin() + 1, last_slash, '/', '-');
        }

        return "/topics" + flattened_topic;
    }
}

void RerunLoggerNode::_read_yaml_config(std::string yaml_path) {
    const YAML::Node config = YAML::LoadFile(yaml_path);

    // see https://www.rerun.io/docs/howto/ros2-nav-turtlebot#tf-to-rrtransform3d
    if (config["topic_to_entity_path"]) {
        _topic_to_entity_path =
            config["topic_to_entity_path"].as<std::map<std::string, std::string>>();

        for (auto const& [key, val] : _topic_to_entity_path) {
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
    if (config["tf"] && config["tf"]["tree"]) {
        // set root frame, all messages with frame_id will be logged relative to this frame
        _root_frame = config["tf"]["tree"].begin()->first.as<std::string>();

        // recurse through the tree and add all transforms
        _add_tf_tree(config["tf"]["tree"], "");
    }
}

void RerunLoggerNode::_add_tf_tree(const YAML::Node& node, const std::string& parent) {
    for (const auto& child : node) {
        auto key = child.first.as<std::string>();
        auto value = child.second;
        const std::string entity_path = parent + "/" + key;
        _tf_frame_to_entity_path[key] = entity_path;
        ROS_INFO("Mapping tf frame %s to entity path %s", key.c_str(), entity_path.c_str());
        if (value.size() >= 1) {
            _add_tf_tree(value, entity_path);
        }
    }
}

void RerunLoggerNode::_create_subscribers() {
    ros::master::V_TopicInfo topic_infos;
    ros::master::getTopics(topic_infos);
    for (const auto& topic_info : topic_infos) {
        // already subscribed to this topic?
        if (_topic_to_subscriber.find(topic_info.name) != _topic_to_subscriber.end()) {
            continue;
        }

        if (topic_info.datatype == "sensor_msgs/Image") {
            _topic_to_subscriber[topic_info.name] = _create_image_subscriber(topic_info.name);
        } else if (topic_info.datatype == "sensor_msgs/Imu") {
            _topic_to_subscriber[topic_info.name] = _create_imu_subscriber(topic_info.name);
        } else if (topic_info.datatype == "geometry_msgs/PoseStamped") {
            _topic_to_subscriber[topic_info.name] =
                _create_pose_stamped_subscriber(topic_info.name);
        } else if (topic_info.datatype == "tf2_msgs/TFMessage") {
            _topic_to_subscriber[topic_info.name] = _create_tf_message_subscriber(topic_info.name);
        } else if (topic_info.datatype == "nav_msgs/Odometry") {
            _topic_to_subscriber[topic_info.name] = _create_odometry_subscriber(topic_info.name);
        } else if (topic_info.datatype == "sensor_msgs/CameraInfo") {
            _topic_to_subscriber[topic_info.name] = _create_camera_info_subscriber(topic_info.name);
        }
    }
}

void RerunLoggerNode::_print_tf_frames() const {
    ROS_INFO_STREAM(_tf_buffer.allFramesAsYAML() << std::endl);
}

ros::Subscriber RerunLoggerNode::_create_image_subscriber(const std::string& topic) {
    std::string entity_path = _resolve_entity_path(topic);

    return _nh.subscribe<sensor_msgs::Image>(
        topic,
        100,
        [&, entity_path](const sensor_msgs::Image::ConstPtr& msg) {
            if (!_root_frame.empty()) {
                try {
                    auto transform = _tf_buffer.lookupTransform(
                        _root_frame,
                        msg->header.frame_id,
                        msg->header.stamp,
                        ros::Duration(0.1)
                    );
                    log_transform(_rec, parent_entity_path(entity_path), transform);
                } catch (tf2::TransformException& ex) {
                    ROS_WARN("%s", ex.what());
                }
            }
            log_image(_rec, entity_path, msg);
        }
    );
}

ros::Subscriber RerunLoggerNode::_create_imu_subscriber(const std::string& topic) {
    std::string entity_path = _resolve_entity_path(topic);

    return _nh.subscribe<sensor_msgs::Imu>(
        topic,
        100,
        [&, entity_path](const sensor_msgs::Imu::ConstPtr& msg) { log_imu(_rec, entity_path, msg); }
    );
}

ros::Subscriber RerunLoggerNode::_create_pose_stamped_subscriber(const std::string& topic) {
    std::string entity_path = _resolve_entity_path(topic);

    return _nh.subscribe<geometry_msgs::PoseStamped>(
        topic,
        100,
        [&, entity_path](const geometry_msgs::PoseStamped::ConstPtr& msg) {
            log_pose_stamped(_rec, entity_path, msg);
        }
    );
}

ros::Subscriber RerunLoggerNode::_create_tf_message_subscriber(const std::string& topic) {
    std::string entity_path = _resolve_entity_path(topic);

    return _nh.subscribe<tf2_msgs::TFMessage>(
        topic,
        100,
        [&](const tf2_msgs::TFMessage::ConstPtr& msg) {
            log_tf_message(_rec, _tf_frame_to_entity_path, msg);
        }
    );
}

ros::Subscriber RerunLoggerNode::_create_odometry_subscriber(const std::string& topic) {
    std::string entity_path = _resolve_entity_path(topic);

    return _nh.subscribe<nav_msgs::Odometry>(
        topic,
        100,
        [&, entity_path](const nav_msgs::Odometry::ConstPtr& msg) {
            log_odometry(_rec, entity_path, msg);
        }
    );
}

ros::Subscriber RerunLoggerNode::_create_camera_info_subscriber(const std::string& topic) {
    std::string entity_path = _resolve_entity_path(topic);

    // If the camera_info topic has not been explicility mapped to an entity path,
    // we assume that the camera_info topic is a sibling of the image topic, and
    // hence use the parent as the entity path for the pinhole model.
    if (_topic_to_entity_path.find(topic) == _topic_to_entity_path.end()) {
        entity_path = parent_entity_path(entity_path);
    }

    return _nh.subscribe<sensor_msgs::CameraInfo>(
        topic,
        100,
        [&, entity_path](const sensor_msgs::CameraInfo::ConstPtr& msg) {
            log_camera_info(_rec, entity_path, msg);
        }
    );
}

void RerunLoggerNode::spin() {
    // check for new topics every 0.1 seconds
    ros::Timer timer =
        _nh.createTimer(ros::Duration(0.1), [&](const ros::TimerEvent&) { _create_subscribers(); });
    ros::Timer tf_timer =
        _nh.createTimer(ros::Duration(0.1), [&](const ros::TimerEvent&) { _print_tf_frames(); });

    ros::MultiThreadedSpinner spinner(8); // Use 8 threads
    spinner.spin();
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "rerun_logger_node");
    RerunLoggerNode node;
    node.spin();
    return 0;
}
