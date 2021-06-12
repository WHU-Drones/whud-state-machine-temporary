/**
 * @file WhudBasicControl.cpp
 * @author LauZanMo (LauZanMo@whu.edu.cn)
 * @author Chen Junpeng (chenjunpeng@whu.edu.cn)
 * @brief whud_basic_control plugin
 * @version 1.0
 * @date 2021-06-04
 *
 * @copyright Copyright (c) 2021 WHU-Drones
 *
 */
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <stdlib.h>

#include "DataStructure.hpp"
#include "StateMachinePlugin.hpp"

using namespace std;

namespace whud_state_machine {
class WhudSwitch : public PluginBase {
public:

  WhudSwitch() : PluginBase();

  void OnInit(MavRosPublisher &mavros_pub) {
    PluginBase::OnInit(mavros_pub);
    bash_nh_.param<bool>("switch_state",switch_state_,false);
    finish_delay_time_=9999999;
    loop_frequency_=10;
  }

  bool SetTask(ros::V_string param) {
    PluginBase::SetTask(param);
  }


  virtual void TaskSpin() override {
      bash_nh_.param<bool>("switch_state",switch_state_,false);
      if(!switch_state_)
      {
          finish_delay_time_=9999999;
      }
      else
      {
          finish_delay_time_=0;
          task_status_ = TaskStatus::DONE;
      }
  }

  virtual void StopTask() override {}

private:
  bool switch_state_;
};

}  // namespace whud_state_machine

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(whud_state_machine::WhudSwitch,
                       whud_state_machine::PluginBase)
