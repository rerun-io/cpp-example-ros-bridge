#include "rerun_bridge/rerun_ros_interface.hpp"
#include "collection_adapters.hpp"

#include <cv_bridge/cv_bridge.h>
#include <rerun.hpp>


void log_imu(
    const rerun::RecordingStream& rec, const std::string& entity_path,
    const sensor_msgs::Imu::ConstPtr& msg
) {
    rec.set_time_seconds("timestamp", msg->header.stamp.toSec());

    rec.log(entity_path + "/x", rerun::Scalar(msg->linear_acceleration.x));
    rec.log(entity_path + "/y", rerun::Scalar(msg->linear_acceleration.y));
    rec.log(entity_path + "/z", rerun::Scalar(msg->linear_acceleration.z));
}

void log_image(
    const rerun::RecordingStream& rec, const std::string& entity_path,
    const sensor_msgs::Image::ConstPtr& msg
) {
    rec.set_time_seconds("timestamp", msg->header.stamp.toSec());

    cv::Mat img = cv_bridge::toCvCopy(msg, "rgb8")->image;
    rec.log(entity_path, rerun::Image(tensor_shape(img), rerun::TensorBuffer::u8(img)));
}

void log_pose_stamped(
    const rerun::RecordingStream& rec, const std::string& entity_path,
    const geometry_msgs::PoseStamped::ConstPtr& msg
) {
    rec.set_time_seconds("timestamp", msg->header.stamp.toSec());

    rec.log(
        entity_path,
        rerun::Transform3D(
            rerun::Vector3D(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z),
            rerun::Quaternion::from_wxyz(
                msg->pose.orientation.w,
                msg->pose.orientation.x,
                msg->pose.orientation.y,
                msg->pose.orientation.z
            )
        )
    );

    // this is a somewhat hacky way to get a trajectory visualization in Rerun
    // this should be be easier in the future, see https://github.com/rerun-io/rerun/issues/723
    std::string trajectory_entity_path = "/trajectories/" + entity_path;
    rec.log(
        trajectory_entity_path,
        rerun::Points3D(
            {{static_cast<float>(msg->pose.position.x),
              static_cast<float>(msg->pose.position.y),
              static_cast<float>(msg->pose.position.z)}}
        )
    );
}

void log_odometry(
    const rerun::RecordingStream& rec, const std::string& entity_path,
    const nav_msgs::Odometry::ConstPtr& msg
) {
    rec.set_time_seconds("timestamp", msg->header.stamp.toSec());

    rec.log(
        entity_path,
        rerun::Transform3D(
            rerun::Vector3D(
                msg->pose.pose.position.x,
                msg->pose.pose.position.y,
                msg->pose.pose.position.z
            ),
            rerun::Quaternion::from_wxyz(
                msg->pose.pose.orientation.w,
                msg->pose.pose.orientation.x,
                msg->pose.pose.orientation.y,
                msg->pose.pose.orientation.z
            )
        )
    );
}

void log_camera_info(
    const rerun::RecordingStream& rec, const std::string& entity_path,
    const sensor_msgs::CameraInfo::ConstPtr& msg
) {
    // Rerun uses column-major order for Mat3x3
    const std::array<float, 9> image_from_camera = {
        static_cast<float>(msg->K[0]),
        static_cast<float>(msg->K[3]),
        static_cast<float>(msg->K[6]),
        static_cast<float>(msg->K[1]),
        static_cast<float>(msg->K[4]),
        static_cast<float>(msg->K[7]),
        static_cast<float>(msg->K[2]),
        static_cast<float>(msg->K[5]),
        static_cast<float>(msg->K[8]),
    };
    rec.log(
        entity_path,
        rerun::Pinhole(image_from_camera).with_resolution(
            static_cast<int>(msg->width), static_cast<int>(msg->height)
        )
    );
}
