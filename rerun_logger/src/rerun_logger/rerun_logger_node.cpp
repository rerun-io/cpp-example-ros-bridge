#include <map>
#include <string>
#include <vector>

#include <cv_bridge/cv_bridge.h>
#include <ros/master.h>
#include <ros/ros.h>
#include <yaml-cpp/yaml.h>
#include <rerun.hpp>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>

// Adapters so we can borrow an OpenCV image easily into Rerun images without copying:
template <>
struct rerun::CollectionAdapter<uint8_t, cv::Mat> {
    /// Borrow for non-temporary.
    Collection<uint8_t> operator()(const cv::Mat& img) {
        assert(
            "OpenCV matrix was expected have bit depth CV_U8" && CV_MAT_DEPTH(img.type()) == CV_8U
        );

        return Collection<uint8_t>::borrow(img.data, img.total() * img.channels());
    }

    // Do a full copy for temporaries (otherwise the data might be deleted when the temporary is destroyed).
    Collection<uint8_t> operator()(cv::Mat&& img) {
        assert(
            "OpenCV matrix was expected have bit depth CV_U8" && CV_MAT_DEPTH(img.type()) == CV_8U
        );

        std::vector<uint8_t> img_vec(img.total() * img.channels());
        img_vec.assign(img.data, img.data + img.total() * img.channels());
        return Collection<uint8_t>::take_ownership(std::move(img_vec));
    }
};

rerun::Collection<rerun::TensorDimension> tensor_shape(const cv::Mat& img) {
    return {img.rows, img.cols, img.channels()};
};

void logImu(
    const rerun::RecordingStream& rec, const std::string& entity_path,
    const sensor_msgs::Imu::ConstPtr& msg
) {
    rec.set_time_seconds("timestamp", msg->header.stamp.toSec());

    rec.log(entity_path + "/x", rerun::Scalar(msg->linear_acceleration.x));
    rec.log(entity_path + "/y", rerun::Scalar(msg->linear_acceleration.y));
    rec.log(entity_path + "/z", rerun::Scalar(msg->linear_acceleration.z));
}

void logImage(
    const rerun::RecordingStream& rec, const std::string& entity_path,
    const sensor_msgs::Image::ConstPtr& msg
) {
    rec.set_time_seconds("timestamp", msg->header.stamp.toSec());

    cv::Mat img = cv_bridge::toCvCopy(msg, "rgb8")->image;
    rec.log(entity_path, rerun::Image(tensor_shape(img), rerun::TensorBuffer::u8(img)));
}

void logPose(
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

void logOdometry(
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

std::string getEntityPath(
    const std::map<std::string, std::string>& topic_to_entity_path, const std::string& topic
) {
    if (topic_to_entity_path.find(topic) != topic_to_entity_path.end()) {
        return topic_to_entity_path.at(topic);
    } else {
        return topic;
    }
}

int main(int argc, char** argv) {
    const rerun::RecordingStream rec("rerun_logger_node");
    rec.spawn().exit_on_failure();

    ros::init(argc, argv, "rerun_logger_node");
    ros::NodeHandle nh("~");
    ros::Rate loop_rate(1000);

    std::map<std::string, std::string> topic_to_entity_path;
    std::map<std::string, ros::Subscriber> topic_to_subscriber;

    // Read additional config from yaml file
    // NOTE We're not using the ROS parameter server for this, because roscpp doesn't support
    //   reading nested data structures.
    std::string yaml_path;
    if (nh.getParam("yaml_path", yaml_path)) {
        ROS_INFO("Read yaml config at %s", yaml_path.c_str());
        YAML::Node config = YAML::LoadFile(yaml_path);

        ROS_INFO_STREAM(config);

        // see https://www.rerun.io/docs/howto/ros2-nav-turtlebot#tf-to-rrtransform3d
        if (config["topic_to_entity_path"]) {
            topic_to_entity_path =
                config["topic_to_entity_path"].as<std::map<std::string, std::string>>();
            // print the map
            for (auto const& [key, val] : topic_to_entity_path) {
                ROS_INFO("Mapping topic %s to entity path %s", key.c_str(), val.c_str());
            }
        }
        if (config["extra_transform3ds"]) {
            for (const auto& extra_transform3d : config["extra_transform3ds"]) {
                const std::array<float, 3> translation = {
                    extra_transform3d["transform"][3].as<float>(),
                    extra_transform3d["transform"][7].as<float>(),
                    extra_transform3d["transform"][11].as<float>()};
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
                    extra_transform3d["transform"][10].as<float>()};
                rec.log_timeless(
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
                rec.log_timeless(
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

    while (ros::ok()) {
        ros::spinOnce();

        // NOTE We are currently checking in each iteration if there are new topics.
        //   This is not efficient, but it's the easiest way to support new topics being added at runtime.
        //   If you have a lot of topics, you might want to optimize this.
        ros::master::V_TopicInfo topic_infos;
        ros::master::getTopics(topic_infos);
        for (const auto& topic_info : topic_infos) {
            if (topic_to_subscriber.find(topic_info.name) == topic_to_subscriber.end()) {
                auto entity_path = getEntityPath(topic_to_entity_path, topic_info.name);
                if (topic_info.datatype == "sensor_msgs/Image") {
                    topic_to_subscriber[topic_info.name] = nh.subscribe<sensor_msgs::Image>(
                        topic_info.name,
                        100,
                        [&, entity_path](const sensor_msgs::Image::ConstPtr& msg) {
                            logImage(rec, entity_path, msg);
                        }
                    );
                } else if (topic_info.datatype == "sensor_msgs/Imu") {
                    topic_to_subscriber[topic_info.name] = nh.subscribe<sensor_msgs::Imu>(
                        topic_info.name,
                        1000,
                        [&, entity_path](const sensor_msgs::Imu::ConstPtr& msg) {
                            logImu(rec, entity_path, msg);
                        }
                    );
                } else if (topic_info.datatype == "geometry_msgs/PoseStamped") {
                    topic_to_subscriber[topic_info.name] = nh.subscribe<geometry_msgs::PoseStamped>(
                        topic_info.name,
                        1000,
                        [&, entity_path](const geometry_msgs::PoseStamped::ConstPtr& msg) {
                            logPose(rec, entity_path, msg);
                        }
                    );
                } else if (topic_info.datatype == "nav_msgs/Odometry") {
                    topic_to_subscriber[topic_info.name] = nh.subscribe<nav_msgs::Odometry>(
                        topic_info.name,
                        1000,
                        [&, entity_path](const nav_msgs::Odometry::ConstPtr& msg) {
                            logOdometry(rec, entity_path, msg);
                        }
                    );
                }
            }
        }

        loop_rate.sleep();
    }

    return 0;
}
