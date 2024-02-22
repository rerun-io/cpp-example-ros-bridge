#pragma once

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <tf2_msgs/TFMessage.h>

#include <rerun.hpp>

void log_imu(
    const rerun::RecordingStream& rec, const std::string& entity_path,
    const sensor_msgs::Imu::ConstPtr& msg
);

void log_image(
    const rerun::RecordingStream& rec, const std::string& entity_path,
    const sensor_msgs::Image::ConstPtr& msg
);

void log_pose_stamped(
    const rerun::RecordingStream& rec, const std::string& entity_path,
    const geometry_msgs::PoseStamped::ConstPtr& msg
);

void log_odometry(
    const rerun::RecordingStream& rec, const std::string& entity_path,
    const nav_msgs::Odometry::ConstPtr& msg
);

void log_camera_info(
    const rerun::RecordingStream& rec, const std::string& entity_path,
    const sensor_msgs::CameraInfo::ConstPtr& msg
);
void log_tf_message(
    const rerun::RecordingStream& rec,
    const std::map<std::string, std::string>& tf_frame_to_entity_path,
    const tf2_msgs::TFMessage::ConstPtr& msg
);
