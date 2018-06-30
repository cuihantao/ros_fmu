// Copyright (c) 2018 - for information on the respective copyright owner
// see the NOTICE file and/or the repository https://github.com/boschresearch/ros_fmu.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <cassert>

#include <exception>
#include <map>
#include <string>

#include <ros/ros.h>
#include <std_msgs/Float64.h>

#include "ros_fmu/ROSFMUWrapper.h"


int main(int argc, char** argv) {
  ros::init(argc, argv, "ros_fmu_node");
  ros::NodeHandle n("~");

  std::string fmuPath;
  if (!n.getParam("fmu_path", fmuPath)) {
    ROS_ERROR("Parameter 'fmu_path' not specified!");
    throw std::runtime_error("Parameter 'fmu_path' not specified!");
  }

  ros_fmu::ROSFMUWrapper wrapper(fmuPath);
  for (const std::string name : wrapper.getParameterNames()) {
    ROS_DEBUG("FMU has parameter '%s'", name.c_str());
  }
  wrapper.initializeFromROSParameters(n);

  std::map<std::string, ros::Subscriber> subscribers;
  for (const std::string& name : wrapper.getInputVariableNames()) {
    std::string rosifiedName = ros_fmu::ROSFMUWrapper::rosifyName(name);
    ros::Subscriber subscriber =
        n.subscribe<std_msgs::Float64>(rosifiedName, 1000, [&wrapper, name](const std_msgs::Float64::ConstPtr& msg) {
          std::string myName = name;
          wrapper.setInputValue(myName, ros::Time::now(), msg->data);
        });
    subscribers[name] = subscriber;
  }

  std::map<std::string, ros::Publisher> publishers;
  for (const std::string& name : wrapper.getOutputVariableNames()) {
    std::string rosifiedName = ros_fmu::ROSFMUWrapper::rosifyName(name);
    publishers[name] = n.advertise<std_msgs::Float64>(rosifiedName, 1000);
  }

  ros::Timer timer = n.createTimer(ros::Duration(0.01), [&](const ros::TimerEvent& event) {
    wrapper.calcUntil(event.current_expected);
    for (const std::string& name : wrapper.getOutputVariableNames()) {
      std_msgs::Float64 msg;
      msg.data = wrapper.getOutputValue(name);
      publishers[name].publish(msg);
    }
  });

  wrapper.exitInitializationMode(ros::Time::now());

  ros::spin();

  return 0;
}
