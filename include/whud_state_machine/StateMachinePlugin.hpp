/**
 * @file StateMachinePlugin.hpp
 * @author LauZanMo (LauZanMo@whu.edu.cn)
 * @brief State machine plugin base
 * @version 1.0
 * @date 2021-06-04
 *
 * @copyright Copyright (c) 2021 WHU-Drones
 *
 */
#pragma once

#include <ros/ros.h>

#include "DataStructure.hpp"

namespace whud_state_machine {

class PluginBase {
public:
  PluginBase() : base_nh_("~") {}
  PluginBase(const PluginBase&) = delete;
  ~PluginBase() {}
  /**
   * @brief Get the Task Status object
   * 
   * @return Present status of task to judge what to do next
   */
  inline TaskStatus GetTaskStatus() const {
    return task_status_;
  }
  /**
   * @brief Get the Interrupt Signal object
   * @return A bollean variable to judge whether interrupt signal is catched
   * @retval true Catch interrupt signal
   * @retval false No interrupt signal is catched yet
   */
  inline bool GetInterruptSignal() const {
    return interrupt_signal_;
  }
  /**
   * @brief Enable the moving control of plugin
   */
  inline void EnableControl() {
    control_flag_ = true;
  }
  /**
   * @brief Disable the moving control of plugin
   */
  inline void DisableControl() {
    control_flag_ = false;
  }
  /**
   * @brief Delay for a centain time when task is done or out of time
   * @return A boolean variable to judge if delay procedure is over
   * @retval true Delay is over
   * @retval false Delay is not over yet
   */
  inline bool Delay() {
    return (delay_counter_++ >= delay_time_ * loop_frequency_);
  }
  /**
   * @brief Get the parameters needed and initialze the mavros publishers in plugin
   * 
   * @param mavros_pub Mavros publishers to initialize the publishers in plugins 
   */
  virtual void OnInit(MavRosPublisher& mavros_pub) {
    base_nh_.param<int>("loop_frequency", loop_frequency_, 10);
    mavros_pub_ = &mavros_pub;
  }
  /**
   * @brief Set the Task object
   * 
   * @param param The parameter list which contains the name of parameters needed 
   * @return true 
   * @return false 
   */
  //TODO:why?
  virtual bool SetTask(ros::V_string param) {
    task_status_ = TaskStatus::RUN;
    interrupt_signal_ = false;
    delay_counter_ = 0;
    SetDelay(0);
  }

  virtual void TaskSpin() = 0;
  virtual void StopTask() = 0;

protected:
  ros::NodeHandle base_nh_;

  MavRosPublisher* mavros_pub_ = nullptr;
  TaskStatus task_status_;
  bool interrupt_signal_;
  bool control_flag_;

  int loop_frequency_;
  float delay_time_ = 0;
  int delay_counter_;

  inline void SetDelay(float time) {
    delay_time_ = time;
  }
};

}  // namespace whud_state_machine
