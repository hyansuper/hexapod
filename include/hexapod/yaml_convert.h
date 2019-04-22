#ifndef __HEXAPOD_YAML_CONVERT__
#define __HEXAPOD_YAML_CONVERT__

#include "hexapod/ServoInfo.h"
#include <yaml-cpp/yaml.h>

namespace YAML {
template<>
struct convert<hexapod::ServoInfo> {
  static Node encode(const hexapod::ServoInfo& rhs) {
    Node node;
    node["angle_max"]=rhs.angle_max;
    node["angle_min"]=rhs.angle_min;
    node["angle_max_pwm"]=rhs.angle_max_pwm;
    node["angle_min_pwm"]=rhs.angle_min_pwm;
    node["board"]=rhs.board;
    node["channel"]=rhs.channel;
    return node;
  }

  // type and field checking is not implemented
  static bool decode(const Node& node, hexapod::ServoInfo& rhs) {
        
    rhs.angle_max = node["angle_max"].as<double>();
    rhs.angle_min = node["angle_min"].as<double>();
    rhs.angle_max_pwm = node["angle_max_pwm"].as<int>();
    rhs.angle_min_pwm = node["angle_min_pwm"].as<int>();
    rhs.board = node["board"].as<int>();
    rhs.channel = node["channel"].as<int>();
    return true;
  }
};
}

#endif