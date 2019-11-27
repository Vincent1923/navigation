/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Author: Eitan Marder-Eppstein
*********************************************************************/
#ifndef NAV_MOVE_BASE_ACTION_H_
#define NAV_MOVE_BASE_ACTION_H_

#include <vector>
#include <string>

#include <ros/ros.h>

#include <actionlib/server/simple_action_server.h>
#include <move_base_msgs/MoveBaseAction.h>

#include <nav_core/base_local_planner.h>
#include <nav_core/base_global_planner.h>
#include <nav_core/recovery_behavior.h>
#include <geometry_msgs/PoseStamped.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_msgs/GetPlan.h>

#include <pluginlib/class_loader.h>
#include <std_srvs/Empty.h>

#include <dynamic_reconfigure/server.h>
#include "move_base/MoveBaseConfig.h"

namespace move_base {
  //typedefs to help us out with the action server so that we don't hace to type so much
  // typedefs 来帮助我们使用 action server，这样我们就不必键入太多内容
  typedef actionlib::SimpleActionServer<move_base_msgs::MoveBaseAction> MoveBaseActionServer;

  // MoveBase 状态
  enum MoveBaseState {
    PLANNING,
    CONTROLLING,
    CLEARING
  };

  // Recovery 触发
  enum RecoveryTrigger
  {
    PLANNING_R,
    CONTROLLING_R,
    OSCILLATION_R
  };

  /**
   * @class MoveBase
   * @brief A class that uses the actionlib::ActionServer interface that moves the robot base to a goal location.
   *        使用 actionlib::ActionServer 接口的类，该接口将机器人移动到目标位置。
   */
  class MoveBase {
    public:
      /**
       * @brief  Constructor for the actions
       *         构造函数
       * @param name The name of the action
       * @param tf A reference to a TransformListener
       *           对 TransformListener 的引用
       */
      MoveBase(tf::TransformListener& tf);

      /**
       * @brief  Destructor - Cleans up
       *         析构函数 - 清理
       */
      virtual ~MoveBase();

      /**
       * @brief  Performs a control cycle
       *         执行控制周期
       * @param goal A reference to the goal to pursue
       *             目标点
       * @param global_plan A reference to the global plan being used
       *                    对正在使用的全局规划的引用
       * @return True if processing of the goal is done, false otherwise
       *         如果完成了目标，则为true，否则为false
       */
      bool executeCycle(geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& global_plan);

    private:
      /**
       * @brief  A service call that clears the costmaps of obstacles
       * @param req The service request 
       * @param resp The service response
       * @return True if the service call succeeds, false otherwise
       */
      bool clearCostmapsService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp);

      /**
       * @brief  A service call that can be made when the action is inactive that will return a plan
       *         当 action 处于非激活状态时可以进行的服务调用，该服务调用将返回一个规划
       *         这是 movebase 提供的一个服务，功能是调用相应的全局规划器来获得一条 path 返回给客户端
       * @param  req The goal request
       *             请求的目标点
       * @param  resp The plan request
       *              请求的规划
       * @return True if planning succeeded, false otherwise
       *         如果规划成功，则为true，否则为false
       */
      bool planService(nav_msgs::GetPlan::Request &req, nav_msgs::GetPlan::Response &resp);

      /**
       * @brief  Make a new global plan
       *         制定一个新的全局路径规划
       *         makePlan 作用是获取机器人的位姿作为起点，然后调用全局规划器的 makePlan 返回规划路径，存储在 plan
       * @param  goal The goal to plan to
       *              需要规划到达的目标点
       * @param  plan Will be filled in with the plan made by the planner
       *              全局规划器规划的全局路径，存储在 plan
       * @return  True if planning succeeds, false otherwise
       *          如果规划成功，则为true，否则为false
       */
      bool makePlan(const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan);

      /**
       * @brief  Load the recovery behaviors for the navigation stack from the parameter server
       * @param node The ros::NodeHandle to be used for loading parameters 
       * @return True if the recovery behaviors were loaded successfully, false otherwise
       */
      bool loadRecoveryBehaviors(ros::NodeHandle node);

      /**
       * @brief  Loads the default recovery behaviors for the navigation stack
       */
      void loadDefaultRecoveryBehaviors();

      /**
       * @brief  Clears obstacles within a window around the robot
       * @param size_x The x size of the window
       * @param size_y The y size of the window
       */
      void clearCostmapWindows(double size_x, double size_y);

      /**
       * @brief  Publishes a velocity command of zero to the base
       *         将零的速度命令发布给机器人
       *         向 cmd_vel 话题发布000的速度信息，机器人停止运动
       */
      void publishZeroVelocity();

      /**
       * @brief  Reset the state of the move_base action and send a zero velocity command to the base
       */
      void resetState();

      void goalCB(const geometry_msgs::PoseStamped::ConstPtr& goal);

      // 这是全局路径规划线程 planner_thread_ 的入口。这个函数需要等待 actionlib 服务器的 CallBack 函数 MoveBase::executeCb 来唤醒启动。
      // 主要作用是调用全局路径规划获取路径，同时保证规划的周期性以及规划超时清除 goal。
      void planThread();

      void executeCb(const move_base_msgs::MoveBaseGoalConstPtr& move_base_goal);

      bool isQuaternionValid(const geometry_msgs::Quaternion& q);

      double distance(const geometry_msgs::PoseStamped& p1, const geometry_msgs::PoseStamped& p2);

      geometry_msgs::PoseStamped goalToGlobalFrame(const geometry_msgs::PoseStamped& goal_pose_msg);

      /**
       * @brief This is used to wake the planner at periodic intervals.
       */
      void wakePlanner(const ros::TimerEvent& event);

      tf::TransformListener& tf_;

      MoveBaseActionServer* as_;  // move_base 的 actionlib 服务器

      boost::shared_ptr<nav_core::BaseLocalPlanner> tc_;  // 局部路径规划器加载并创建实例后的指针
      costmap_2d::Costmap2DROS* planner_costmap_ros_, *controller_costmap_ros_;

      boost::shared_ptr<nav_core::BaseGlobalPlanner> planner_;  // 全局路径规划器加载并创建实例后的指针
      std::string robot_base_frame_, global_frame_;

      // 这个是转圈圈的？恢复状态
      std::vector<boost::shared_ptr<nav_core::RecoveryBehavior> > recovery_behaviors_;
      unsigned int recovery_index_;

      tf::Stamped<tf::Pose> global_pose_;
      double planner_frequency_, controller_frequency_, inscribed_radius_, circumscribed_radius_;
      double planner_patience_, controller_patience_;
      int32_t max_planning_retries_;
      uint32_t planning_retries_;
      double conservative_reset_dist_, clearing_radius_;
      ros::Publisher current_goal_pub_, vel_pub_, action_goal_pub_;
      ros::Subscriber goal_sub_;
      ros::ServiceServer make_plan_srv_, clear_costmaps_srv_;
      bool shutdown_costmaps_, clearing_rotation_allowed_, recovery_behavior_enabled_;
      double oscillation_timeout_, oscillation_distance_;

      MoveBaseState state_;
      RecoveryTrigger recovery_trigger_;

      ros::Time last_valid_plan_, last_valid_control_, last_oscillation_reset_;
      geometry_msgs::PoseStamped oscillation_pose_;
      // 以插件形式实现全局规划器、局部规划器和丢失时恢复规划器。
      // 插件形式可以实现随时动态地加载C++类库，但需要在包中注册该插件，不用这个的话需要提前链接（相当于运行时加载）
      pluginlib::ClassLoader<nav_core::BaseGlobalPlanner> bgp_loader_;
      pluginlib::ClassLoader<nav_core::BaseLocalPlanner> blp_loader_;
      pluginlib::ClassLoader<nav_core::RecoveryBehavior> recovery_loader_;

      //set up plan triple buffer
      // 一般保存规划器中新鲜出路的路径，然后将其给 latest_plan_
      std::vector<geometry_msgs::PoseStamped>* planner_plan_;
      // 作为一个桥梁，在 MoveBase::executeCycle 中传递给 controller_plan_
      std::vector<geometry_msgs::PoseStamped>* latest_plan_;
      std::vector<geometry_msgs::PoseStamped>* controller_plan_;

      //set up the planner's thread
      bool runPlanner_;
      boost::recursive_mutex planner_mutex_;
      // boost 的一种结合了互斥锁的用法，可以使一个线程进入睡眠状态，然后在另一个线程触发唤醒。
      boost::condition_variable_any planner_cond_;
      // 通过这个值将 goal 在 MoveBase::executeCb 与 MoveBase::planThread() 之间传递
      geometry_msgs::PoseStamped planner_goal_;
      boost::thread* planner_thread_;


      boost::recursive_mutex configuration_mutex_;
      dynamic_reconfigure::Server<move_base::MoveBaseConfig> *dsrv_;
      
      void reconfigureCB(move_base::MoveBaseConfig &config, uint32_t level);

      move_base::MoveBaseConfig last_config_;
      move_base::MoveBaseConfig default_config_;
      bool setup_, p_freq_change_, c_freq_change_;
      bool new_global_plan_;
  };
};
#endif

