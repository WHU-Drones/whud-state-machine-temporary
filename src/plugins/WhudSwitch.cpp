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
  }

  bool SetTask(ros::V_string param) {
    PluginBase::SetTask(param);
    bash_nh_.param<bool>("switch_state",switch_state_);
    if(param[0] >= 0 && switch_state_)
      SetFinishDelay(float time);
    else
      SetFinishDelay(0);
    
  }


  virtual void TaskSpin() override {
      bash_nh_.param<bool>("switch_state",switch_state_);
      task_status_ = TaskStatus::DONE;
      if(!switch_state_){
        SetFinishDelay(0);
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
