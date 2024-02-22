#pragma once

#include <map>
#include <string>

#include <rerun.hpp>
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <yaml-cpp/yaml.h>



class RerunLoggerNode {
  public:
    RerunLoggerNode();
    void spin();

  private:
    std::map<std::string, std::string> _topic_to_entity_path_map;
    std::map<std::string, ros::Subscriber> _topic_to_subscriber;
    std::map<std::string, std::string> _tf_frame_to_entity_path;

    void _read_yaml_config(std::string yaml_path);

    std::string _topic_to_entity_path(const std::string& topic) const;
    std::string _topic_to_namespace(const std::string& topic) const;


    void _add_tf_tree(const YAML::Node& node, const std::string& parent); 

    const rerun::RecordingStream _rec{"rerun_logger_node"};
    ros::NodeHandle _nh{"~"};
    /* tf2_ros::Buffer _tf_buffer; */
    /* tf2_ros::TransformListener _tf_listener{_tf_buffer}; */

    void _create_subscribers();
};
