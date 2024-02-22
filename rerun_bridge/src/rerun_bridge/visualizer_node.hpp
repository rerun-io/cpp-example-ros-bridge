#pragma once

#include <map>
#include <string>

#include <rerun.hpp>
#include <ros/ros.h>


class RerunLoggerNode {
  public:
    RerunLoggerNode();
    void spin();

  private:
    std::map<std::string, std::string> _topic_to_entity_path_map;
    std::map<std::string, ros::Subscriber> _topic_to_subscriber;

    void _read_yaml_config(std::string yaml_path);

    std::string _topic_to_entity_path(const std::string& topic) const;
    std::string _topic_to_namespace(const std::string& topic) const;

    const rerun::RecordingStream _rec{"rerun_logger_node"};
    ros::NodeHandle _nh{"~"};
    ros::Rate _loop_rate{1000};

    void _create_subscribers();
};
