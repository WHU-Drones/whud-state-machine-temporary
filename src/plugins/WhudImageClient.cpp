/**
 * @file WhudImageClient.cpp
 * @author Liuzhihao (liuzhihao@whu.edu.cn)
 * @brief WhudImageClient plugin
 * @version 1.0
 * @date 2021-06-22
 * 
 * @copyright Copyright (c) 2021 WHU-Drones
 * 
 */
#include <ros/ros.h>
#include <whud_vision/ImageProcessAction.h>
#include <stdlib.h>

#include "DataStructure.hpp"
#include "StateMachinePlugin.hpp"

using namespace std;

namespace whud_state_machine {
class WhudImageClient : public PluginBase {
public:
  /**
   * @brief Construct a new Whud Image Control object
   *
   * @note In this function, private node handle and will be initialized
   */
  WhudImageControl() : PluginBase(), nh_("~whud_image_client"), image_client_("color_recognize"){};

  /**
   * @brief Destroy the Whud Image Client object
   */
  ~WhudImageClient() {}

  virtual void OnInit(MavRosPublisher &mavros_pub) override {
    PluginBase::OnInit(mavros_pub);
  }

  virtual bool SetTask(ros::V_string param) override {
        stop_judge_ = atof(param[0].c_str());
        SetFinishDelay(atof(param[1].c_str()));
        time_begin_ = current_expected.now().toSec();
        ROS_INFO(
          "Wait for image processing server and transform set up, please be sure they "
          "will be set up.");
        SetFinishDelay(atof(param[2].c_str()));
        image_client_.waitForServer();
        bool goal=true;
        image_client_.sendGoal(
            goal,
            std::bind(&WhudImageClient::DoneCb, this, std::placeholders::_1,
                    std::placeholders::_2),
            std::bind(&WhudImageClient::ActiveCb, this),
            std::bind(&WhudImageClient::FeedbackCb, this, std::placeholders::_1));
    return true;
  }


  virtual void TaskSpin() override {
      mavros_pub_->cmd_vel_pub.publish();
      if(vel.linear.x < 0.03 && vel.linear.y < 0.03){
        if(timer_.current_expected.now().toSec()-time_begin_>=stop_judge_)
      }
      else{
          timer_begin=timer_.current_expected.now().toSec();
      }
  }


  virtual void StopTask() override {
    image_client_.cancelGoal();
    std_msgs::Bool conversion;
    conversion.data = true;
    mavros_pub_->conversion_pub.publish(conversion);
  }

private:
  ros::NodeHandle nh_;
  geometry_msgs::Twist vel;
  ros::TimerEvent timer_;
  double time_begin_;
  double stop_judge_;
  actionlib::SimpleActionClient<whud_vision::ImageProcessingAction> image_client_;

  void ActiveCb() {}

  void FeedbackCb(const whud_vision::ImageProcessingFeedbackConstPtr &feedback) {
      vel.linear.x=feedback->speed_control[0];
      vel.linear.y=feedback->speed_control[1];
  }

  void DoneCb(const actionlib::SimpleClientGoalState &state,
              const whud_vision::ImageProcessingResultConstPtr &result) {
    if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
      task_status_ = TaskStatus::DONE;
      std_msgs::Bool conversion;
      conversion.data = true;
      mavros_pub_->conversion_pub.publish(conversion);
    }
  }

};

}  // namespace whud_state_machine

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(whud_state_machine::WhudImageClient,
                       whud_state_machine::PluginBase)