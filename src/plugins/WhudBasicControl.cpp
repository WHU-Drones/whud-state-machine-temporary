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
#include <std_msgs/Float64MultiArray.h>
#include <stdlib.h>

#include "DataStructure.hpp"
#include "StateMachinePlugin.hpp"

using namespace std;

namespace whud_state_machine {
class WhudBasicControl : public PluginBase {
public:
  /**
   * @brief Construct a new Whud Basic Control object
   *
   * @note In this function, private node handle and will be initialized
   */
  WhudBasicControl() : PluginBase(), nh_("~basic_control"){};

  enum Command { NONE = 0, TAKEOFF, LAND, HEIGHT_CONTROL, YAW_CONTROL };

  /**
   * @brief Plugin init
   * 
   * @note The mavros publisher will be initialized.
   * 
   * @param mavros_pub Mavros publisher passed by state machine
   */
  void OnInit(MavRosPublisher &mavros_pub) {
    mavros_pub_ = &mavros_pub;
  }
  /**
   * @brief Set task with given parameters
   * 
   * @note In this function, parameters passed by state machine will be parsed,
   * the result will be published to mavros to control basic operation.
   * 
   * @param param Parameter vector passed by state machine, plugins will set
   * task according to the paramter parse:
   * param[0]: A string denote the basic operation, it can be "takeoff", "land", 
   * "height_control" or "yaw_control".
   * param[1:]: The parameters needed for each basic operation. The parameters 
   * needed for each operation are:
   * takeoff:
   * -param[1]: z axis speed(takeoff speed)
   * -param[2]: takeoff height
   * land:
   * -param[1]: z axis speed(land speed)
   * height_control:
   * -param[1]: z axis speed
   * -param[2]: target height
   * yaw_control:
   * -param[1]: target angle
   * -param[2]: denote param[1] is absolute angle or relative angle, 
   * 0 for absolute and 1 for relative.
   * 
   * @retval true: Parameters are parsed correctly
   * @retval false: Parameters are parsed wrong
   */
  bool SetTask(ros::V_string param) {
    PluginBase::SetTask(param);
    if (param[0] == "takeoff") {
      takeoff_.data.clear();
      // set z axis speed
      takeoff_.data.push_back(atof(param[1].c_str()));

      // set height
      takeoff_.data.push_back(atof(param[2].c_str()));

      command_ = Command::TAKEOFF;

    } else if (param[0] == "land") {
      // set z axis speed
      land_.data = atof(param[1].c_str());

      command_ = Command::LAND;

    } else if (param[0] == "height_control") {
      height_control_.data.clear();
      // set z axis speed
      height_control_.data.push_back(atof(param[1].c_str()));

      // set target height(relatively)
      height_control_.data.push_back(atof(param[2].c_str()));

      command_ = Command::HEIGHT_CONTROL;

    } else if (param[0] == "yaw_control") {
      yaw_control_.data.clear();
      // set target angle
      yaw_control_.data.push_back(atof(param[1].c_str()));

      // 0: absolute, 1: relative
      yaw_control_.data.push_back(atof(param[2].c_str()));

      command_ = Command::YAW_CONTROL;

    } else {
      command_ = Command::NONE;
    }

    return true;
  }
  /**
   * @brief Spin task in regular frequency.
   * 
   * @note mavros_command denotes our target operation. 
   * mavros_result denotes the operation status, 0 for no 
   * operation now and 5 for operation is done. 
   */
  virtual void TaskSpin() override {
    if (mavros_pub_ != nullptr && control_flag_ == true) {
      int mavros_command, mavros_result;
      nh_.getParam("/mavros/whud_basic/ack_cmd_index", mavros_command);
      nh_.getParam("/mavros/whud_basic/ack_result", mavros_result);

      switch (command_) {
      case Command::TAKEOFF:
        if (mavros_command == 24 && mavros_result == 0)
          task_status_ = TaskStatus::DONE;
        else if (mavros_command != 24 && mavros_result != 5)
          mavros_pub_->takeoff_pub.publish(takeoff_);
        break;

      case Command::LAND:
        if (mavros_command == 23 && mavros_result == 0)
          task_status_ = TaskStatus::DONE;
        else if (mavros_command != 23 && mavros_result != 5)
          mavros_pub_->land_pub.publish(land_);
        break;

      case Command::HEIGHT_CONTROL:
        if (mavros_command == 113 && mavros_result == 0)
          task_status_ = TaskStatus::DONE;
        else if (mavros_command != 113 && mavros_result != 5)
          mavros_pub_->height_pub.publish(height_control_);
        break;

      case Command::YAW_CONTROL:
        if (mavros_command == 115 && mavros_result == 0)
          task_status_ = TaskStatus::DONE;
        else if (mavros_command != 115 && mavros_result != 5)
          mavros_pub_->yaw_pub.publish(yaw_control_);
        break;

      default:
        break;
      }
    }
  }

  virtual void StopTask() override {}

private:
  ros::NodeHandle nh_;
  Command command_;
  std_msgs::Float64MultiArray takeoff_;
  std_msgs::Float64 land_;
  std_msgs::Float64MultiArray height_control_;
  std_msgs::Float64MultiArray yaw_control_;
};

}  // namespace whud_state_machine

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(whud_state_machine::WhudBasicControl,
                       whud_state_machine::PluginBase)
