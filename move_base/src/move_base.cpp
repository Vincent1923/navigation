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
*         Mike Phillips (put the planner in its own thread)
*********************************************************************/
#include <move_base/move_base.h>
#include <cmath>

#include <boost/algorithm/string.hpp>
#include <boost/thread.hpp>

#include <geometry_msgs/Twist.h>

namespace move_base {

  MoveBase::MoveBase(tf::TransformListener& tf) :
    tf_(tf),
    as_(NULL),
    planner_costmap_ros_(NULL), controller_costmap_ros_(NULL),
    bgp_loader_("nav_core", "nav_core::BaseGlobalPlanner"),
    blp_loader_("nav_core", "nav_core::BaseLocalPlanner"), 
    recovery_loader_("nav_core", "nav_core::RecoveryBehavior"),
    planner_plan_(NULL), latest_plan_(NULL), controller_plan_(NULL),
    runPlanner_(false), setup_(false), p_freq_change_(false), c_freq_change_(false), new_global_plan_(false) {

    // 定义一个名为 move_base 的 SimpleActionServer，该服务器的 Callback 为 MoveBase::executeCb
    as_ = new MoveBaseActionServer(ros::NodeHandle(), "move_base", boost::bind(&MoveBase::executeCb, this, _1), false);

    ros::NodeHandle private_nh("~");
    ros::NodeHandle nh;

    recovery_trigger_ = PLANNING_R;

    //get some parameters that will be global to the move base node
    // 从参数服务器获取一些参数，包括两个规划器名称、代价地图坐标系、规划频率、控制周期等
    std::string global_planner, local_planner;
    private_nh.param("base_global_planner", global_planner, std::string("navfn/NavfnROS"));
    private_nh.param("base_local_planner", local_planner, std::string("base_local_planner/TrajectoryPlannerROS"));
    private_nh.param("global_costmap/robot_base_frame", robot_base_frame_, std::string("base_link"));
    private_nh.param("global_costmap/global_frame", global_frame_, std::string("/map"));
    private_nh.param("planner_frequency", planner_frequency_, 0.0);
    private_nh.param("controller_frequency", controller_frequency_, 20.0);
    private_nh.param("planner_patience", planner_patience_, 5.0);
    private_nh.param("controller_patience", controller_patience_, 15.0);
    private_nh.param("max_planning_retries", max_planning_retries_, -1);  // disabled by default

    private_nh.param("oscillation_timeout", oscillation_timeout_, 0.0);
    private_nh.param("oscillation_distance", oscillation_distance_, 0.5);

    //set up plan triple buffer
    planner_plan_ = new std::vector<geometry_msgs::PoseStamped>();
    latest_plan_ = new std::vector<geometry_msgs::PoseStamped>();
    controller_plan_ = new std::vector<geometry_msgs::PoseStamped>();

    //set up the planner's thread
    // 新建 planner 线程，入口为 MoveBase::planThread
    planner_thread_ = new boost::thread(boost::bind(&MoveBase::planThread, this));

    //for comanding the base
    // 控制机器人运动
    vel_pub_ = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    current_goal_pub_ = private_nh.advertise<geometry_msgs::PoseStamped>("current_goal", 0 );

    ros::NodeHandle action_nh("move_base");
    action_goal_pub_ = action_nh.advertise<move_base_msgs::MoveBaseActionGoal>("goal", 1);

    //we'll provide a mechanism for some people to send goals as PoseStamped messages over a topic
    //they won't get any useful information back about its status, but this is useful for tools
    //like nav_view and rviz
    // 订阅 geometry_msgs::PoseStamped 类型的 goal 话题，cb为 MoveBase::goalCB，
    // 你在 rviz 中输入的目标点就是通过这个函数来响应的
    ros::NodeHandle simple_nh("move_base_simple");
    goal_sub_ = simple_nh.subscribe<geometry_msgs::PoseStamped>("goal", 1, boost::bind(&MoveBase::goalCB, this, _1));

    //we'll assume the radius of the robot to be consistent with what's specified for the costmaps
    // 我们假设机器人的半径与为 costmaps 指定的半径一致
    // 从参数服务器获取代价地图相关的一些参数
    private_nh.param("local_costmap/inscribed_radius", inscribed_radius_, 0.325);
    private_nh.param("local_costmap/circumscribed_radius", circumscribed_radius_, 0.46);
    private_nh.param("clearing_radius", clearing_radius_, circumscribed_radius_);
    private_nh.param("conservative_reset_dist", conservative_reset_dist_, 3.0);

    private_nh.param("shutdown_costmaps", shutdown_costmaps_, false);
    private_nh.param("clearing_rotation_allowed", clearing_rotation_allowed_, true);
    private_nh.param("recovery_behavior_enabled", recovery_behavior_enabled_, true);

    //create the ros wrapper for the planner's costmap... and initializer a pointer we'll use with the underlying map
    planner_costmap_ros_ = new costmap_2d::Costmap2DROS("global_costmap", tf_);
    planner_costmap_ros_->pause();

    //initialize the global planner
    // 初始化 global planner，包括查看规划器是否有效，通过代价地图创建实例等
    try {
      planner_ = bgp_loader_.createInstance(global_planner);
      planner_->initialize(bgp_loader_.getName(global_planner), planner_costmap_ros_);
    } catch (const pluginlib::PluginlibException& ex) {
      ROS_FATAL("Failed to create the %s planner, are you sure it is properly registered and that the containing library is built? Exception: %s", global_planner.c_str(), ex.what());
      exit(1);
    }

    //create the ros wrapper for the controller's costmap... and initializer a pointer we'll use with the underlying map
    controller_costmap_ros_ = new costmap_2d::Costmap2DROS("local_costmap", tf_);
    controller_costmap_ros_->pause();

    //create a local planner
    // 初始化 local planner，包括查看规划器是否有效，通过代价地图创建实例等
    try {
      tc_ = blp_loader_.createInstance(local_planner);
      ROS_INFO("Created local_planner %s", local_planner.c_str());
      tc_->initialize(blp_loader_.getName(local_planner), &tf_, controller_costmap_ros_);
    } catch (const pluginlib::PluginlibException& ex) {
      ROS_FATAL("Failed to create the %s planner, are you sure it is properly registered and that the containing library is built? Exception: %s", local_planner.c_str(), ex.what());
      exit(1);
    }

    // Start actively updating costmaps based on sensor data
    // 开启根据传感器数据更新代价地图
    planner_costmap_ros_->start();
    controller_costmap_ros_->start();

    //advertise a service for getting a plan
    // 定义一个名为 make_plan 的服务，cb为 MoveBase::planService
    make_plan_srv_ = private_nh.advertiseService("make_plan", &MoveBase::planService, this);

    //advertise a service for clearing the costmaps
    // 定义一个名为 clear_costmaps 的服务，cb为 MoveBase::clearCostmapsService
    clear_costmaps_srv_ = private_nh.advertiseService("clear_costmaps", &MoveBase::clearCostmapsService, this);

    //if we shutdown our costmaps when we're deactivated... we'll do that now
    if(shutdown_costmaps_){
      ROS_DEBUG_NAMED("move_base","Stopping costmaps initially");
      planner_costmap_ros_->stop();
      controller_costmap_ros_->stop();
    }

    //load any user specified recovery behaviors, and if that fails load the defaults
    // 先 loadRecoveryBehaviors，不行再 loadDefaultRecoveryBehaviors 加载用户自定义的恢复规划器，这里包括了找不到路自转360°
    if(!loadRecoveryBehaviors(private_nh)){
      loadDefaultRecoveryBehaviors();
    }

    //initially, we'll need to make a plan
    state_ = PLANNING;

    //we'll start executing recovery behaviors at the beginning of our list
    recovery_index_ = 0;

    //we're all set up now so we can start the action server
    // 启动actionlib服务器
    as_->start();

    // 启动动态参数服务器，回调函数为 reconfigureCB
    dsrv_ = new dynamic_reconfigure::Server<move_base::MoveBaseConfig>(ros::NodeHandle("~"));
    dynamic_reconfigure::Server<move_base::MoveBaseConfig>::CallbackType cb = boost::bind(&MoveBase::reconfigureCB, this, _1, _2);
    dsrv_->setCallback(cb);
  }

  void MoveBase::reconfigureCB(move_base::MoveBaseConfig &config, uint32_t level){
    boost::recursive_mutex::scoped_lock l(configuration_mutex_);

    //The first time we're called, we just want to make sure we have the
    //original configuration
    if(!setup_)
    {
      last_config_ = config;
      default_config_ = config;
      setup_ = true;
      return;
    }

    if(config.restore_defaults) {
      config = default_config_;
      //if someone sets restore defaults on the parameter server, prevent looping
      config.restore_defaults = false;
    }

    if(planner_frequency_ != config.planner_frequency)
    {
      planner_frequency_ = config.planner_frequency;
      p_freq_change_ = true;
    }

    if(controller_frequency_ != config.controller_frequency)
    {
      controller_frequency_ = config.controller_frequency;
      c_freq_change_ = true;
    }

    planner_patience_ = config.planner_patience;
    controller_patience_ = config.controller_patience;
    max_planning_retries_ = config.max_planning_retries;
    conservative_reset_dist_ = config.conservative_reset_dist;

    recovery_behavior_enabled_ = config.recovery_behavior_enabled;
    clearing_rotation_allowed_ = config.clearing_rotation_allowed;
    shutdown_costmaps_ = config.shutdown_costmaps;

    oscillation_timeout_ = config.oscillation_timeout;
    oscillation_distance_ = config.oscillation_distance;
    if(config.base_global_planner != last_config_.base_global_planner) {
      boost::shared_ptr<nav_core::BaseGlobalPlanner> old_planner = planner_;
      //initialize the global planner
      ROS_INFO("Loading global planner %s", config.base_global_planner.c_str());
      try {
        planner_ = bgp_loader_.createInstance(config.base_global_planner);

        // wait for the current planner to finish planning
        boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);

        // Clean up before initializing the new planner
        planner_plan_->clear();
        latest_plan_->clear();
        controller_plan_->clear();
        resetState();
        planner_->initialize(bgp_loader_.getName(config.base_global_planner), planner_costmap_ros_);

        lock.unlock();
      } catch (const pluginlib::PluginlibException& ex) {
        ROS_FATAL("Failed to create the %s planner, are you sure it is properly registered and that the \
                   containing library is built? Exception: %s", config.base_global_planner.c_str(), ex.what());
        planner_ = old_planner;
        config.base_global_planner = last_config_.base_global_planner;
      }
    }

    if(config.base_local_planner != last_config_.base_local_planner){
      boost::shared_ptr<nav_core::BaseLocalPlanner> old_planner = tc_;
      //create a local planner
      try {
        tc_ = blp_loader_.createInstance(config.base_local_planner);
        // Clean up before initializing the new planner
        planner_plan_->clear();
        latest_plan_->clear();
        controller_plan_->clear();
        resetState();
        tc_->initialize(blp_loader_.getName(config.base_local_planner), &tf_, controller_costmap_ros_);
      } catch (const pluginlib::PluginlibException& ex) {
        ROS_FATAL("Failed to create the %s planner, are you sure it is properly registered and that the \
                   containing library is built? Exception: %s", config.base_local_planner.c_str(), ex.what());
        tc_ = old_planner;
        config.base_local_planner = last_config_.base_local_planner;
      }
    }

    last_config_ = config;
  }

  // 为 rviz 等提供一个简单的调用，该回调函数将 geometry_msgs::PoseStamped 形式的 goal 转换成
  // move_base_msgs::MoveBaseActionGoal，再发布到对应类型的 goal 话题中
  void MoveBase::goalCB(const geometry_msgs::PoseStamped::ConstPtr& goal){
    ROS_DEBUG_NAMED("move_base","In ROS goal callback, wrapping the PoseStamped in the action message and re-sending to the server.");
    move_base_msgs::MoveBaseActionGoal action_goal;
    action_goal.header.stamp = ros::Time::now();
    action_goal.goal.target_pose = *goal;

    // action_goal_pub_ 发布消息的 topic 为"move_base/goal"，消息类型为 move_base_msgs::MoveBaseActionGoal
    action_goal_pub_.publish(action_goal);
  }

  void MoveBase::clearCostmapWindows(double size_x, double size_y){
    tf::Stamped<tf::Pose> global_pose;

    //clear the planner's costmap
    planner_costmap_ros_->getRobotPose(global_pose);

    std::vector<geometry_msgs::Point> clear_poly;
    double x = global_pose.getOrigin().x();
    double y = global_pose.getOrigin().y();
    geometry_msgs::Point pt;

    pt.x = x - size_x / 2;
    pt.y = y - size_y / 2;
    clear_poly.push_back(pt);

    pt.x = x + size_x / 2;
    pt.y = y - size_y / 2;
    clear_poly.push_back(pt);

    pt.x = x + size_x / 2;
    pt.y = y + size_y / 2;
    clear_poly.push_back(pt);

    pt.x = x - size_x / 2;
    pt.y = y + size_y / 2;
    clear_poly.push_back(pt);

    planner_costmap_ros_->getCostmap()->setConvexPolygonCost(clear_poly, costmap_2d::FREE_SPACE);

    //clear the controller's costmap
    controller_costmap_ros_->getRobotPose(global_pose);

    clear_poly.clear();
    x = global_pose.getOrigin().x();
    y = global_pose.getOrigin().y();

    pt.x = x - size_x / 2;
    pt.y = y - size_y / 2;
    clear_poly.push_back(pt);

    pt.x = x + size_x / 2;
    pt.y = y - size_y / 2;
    clear_poly.push_back(pt);

    pt.x = x + size_x / 2;
    pt.y = y + size_y / 2;
    clear_poly.push_back(pt);

    pt.x = x - size_x / 2;
    pt.y = y + size_y / 2;
    clear_poly.push_back(pt);

    controller_costmap_ros_->getCostmap()->setConvexPolygonCost(clear_poly, costmap_2d::FREE_SPACE);
  }

  bool MoveBase::clearCostmapsService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp){
    //clear the costmaps
    boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock_controller(*(controller_costmap_ros_->getCostmap()->getMutex()));
    controller_costmap_ros_->resetLayers();

    boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock_planner(*(planner_costmap_ros_->getCostmap()->getMutex()));
    planner_costmap_ros_->resetLayers();
    return true;
  }


  // 这是 movebase 提供的一个服务。
  // 搜了一下发现，除了 movebase，navfn 以及 global_planner 这两个包也会发布这个服务，但是没有节点订阅～～～～。
  // 这三个包的 cb 其实都是调用相应的全局规划器来获得一条 path 返回给客户端。
  bool MoveBase::planService(nav_msgs::GetPlan::Request &req, nav_msgs::GetPlan::Response &resp){
    // move_base 必须处于非激活状态才能为外部用户制定规划
    if(as_->isActive()){
      ROS_ERROR("move_base must be in an inactive state to make a plan for an external user");
      return false;
    }
    //make sure we have a costmap for our planner
    if(planner_costmap_ros_ == NULL){
      ROS_ERROR("move_base cannot make a plan for you because it doesn't have a costmap");
      return false;
    }

    geometry_msgs::PoseStamped start;
    //if the user does not specify a start pose, identified by an empty frame id, then use the robot's pose
    if(req.start.header.frame_id.empty())
    {
        tf::Stamped<tf::Pose> global_pose;
        if(!planner_costmap_ros_->getRobotPose(global_pose)){
          ROS_ERROR("move_base cannot make a plan for you because it could not get the start pose of the robot");
          return false;
        }
        tf::poseStampedTFToMsg(global_pose, start);
    }
    else
    {
        start = req.start;
    }

    //update the copy of the costmap the planner uses
    clearCostmapWindows(2 * clearing_radius_, 2 * clearing_radius_);

    //first try to make a plan to the exact desired goal
    std::vector<geometry_msgs::PoseStamped> global_plan;
    if(!planner_->makePlan(start, req.goal, global_plan) || global_plan.empty()){
      ROS_DEBUG_NAMED("move_base","Failed to find a plan to exact goal of (%.2f, %.2f), searching for a feasible goal within tolerance", 
          req.goal.pose.position.x, req.goal.pose.position.y);

      //search outwards for a feasible goal within the specified tolerance
      geometry_msgs::PoseStamped p;
      p = req.goal;
      bool found_legal = false;
      float resolution = planner_costmap_ros_->getCostmap()->getResolution();
      float search_increment = resolution*3.0;
      if(req.tolerance > 0.0 && req.tolerance < search_increment) search_increment = req.tolerance;
      for(float max_offset = search_increment; max_offset <= req.tolerance && !found_legal; max_offset += search_increment) {
        for(float y_offset = 0; y_offset <= max_offset && !found_legal; y_offset += search_increment) {
          for(float x_offset = 0; x_offset <= max_offset && !found_legal; x_offset += search_increment) {

            //don't search again inside the current outer layer
            if(x_offset < max_offset-1e-9 && y_offset < max_offset-1e-9) continue;

            //search to both sides of the desired goal
            for(float y_mult = -1.0; y_mult <= 1.0 + 1e-9 && !found_legal; y_mult += 2.0) {

              //if one of the offsets is 0, -1*0 is still 0 (so get rid of one of the two)
              if(y_offset < 1e-9 && y_mult < -1.0 + 1e-9) continue;

              for(float x_mult = -1.0; x_mult <= 1.0 + 1e-9 && !found_legal; x_mult += 2.0) {
                if(x_offset < 1e-9 && x_mult < -1.0 + 1e-9) continue;

                p.pose.position.y = req.goal.pose.position.y + y_offset * y_mult;
                p.pose.position.x = req.goal.pose.position.x + x_offset * x_mult;

                if(planner_->makePlan(start, p, global_plan)){
                  if(!global_plan.empty()){

                    //adding the (unreachable) original goal to the end of the global plan, in case the local planner can get you there
                    //(the reachable goal should have been added by the global planner)
                    global_plan.push_back(req.goal);

                    found_legal = true;
                    ROS_DEBUG_NAMED("move_base", "Found a plan to point (%.2f, %.2f)", p.pose.position.x, p.pose.position.y);
                    break;
                  }
                }
                else{
                  ROS_DEBUG_NAMED("move_base","Failed to find a plan to point (%.2f, %.2f)", p.pose.position.x, p.pose.position.y);
                }
              }
            }
          }
        }
      }
    }

    //copy the plan into a message to send out
    resp.plan.poses.resize(global_plan.size());
    for(unsigned int i = 0; i < global_plan.size(); ++i){
      resp.plan.poses[i] = global_plan[i];
    }

    return true;
  }

  MoveBase::~MoveBase(){
    recovery_behaviors_.clear();

    delete dsrv_;

    if(as_ != NULL)
      delete as_;

    if(planner_costmap_ros_ != NULL)
      delete planner_costmap_ros_;

    if(controller_costmap_ros_ != NULL)
      delete controller_costmap_ros_;

    planner_thread_->interrupt();
    planner_thread_->join();

    delete planner_thread_;

    delete planner_plan_;
    delete latest_plan_;
    delete controller_plan_;

    planner_.reset();
    tc_.reset();
  }

  // MoveBase::makePlan() 函数的作用是获取机器人的位姿作为起点，然后调用全局规划器的 makePlan 返回规划路径，存储在 plan
  // 这个是在 global costmap 下做的 global planner
  bool MoveBase::makePlan(const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan){
    boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*(planner_costmap_ros_->getCostmap()->getMutex()));

    //make sure to set the plan to be empty initially
    // 确保最初将规划路径设置为空
    plan.clear();

    //since this gets called on handle activate
    if(planner_costmap_ros_ == NULL) {
      ROS_ERROR("Planner costmap ROS is NULL, unable to create global plan");
      return false;
    }

    //get the starting pose of the robot
    // 获取机器人的起始位姿
    tf::Stamped<tf::Pose> global_pose;
    if(!planner_costmap_ros_->getRobotPose(global_pose)) {
      ROS_WARN("Unable to get starting pose of robot, unable to create global plan");
      return false;
    }

    geometry_msgs::PoseStamped start;
    // tf::poseStampedTFToMsg() 函数将 Stamped<Pose> 转换为 PoseStamped msg
    tf::poseStampedTFToMsg(global_pose, start);

    //if the planner fails or returns a zero length plan, planning failed
    // 如果全局规划器失败或返回零长度的规划路径，则全局路径规划失败
    if(!planner_->makePlan(start, goal, plan) || plan.empty()){
      ROS_DEBUG_NAMED("move_base","Failed to find a  plan to point (%.2f, %.2f)", goal.pose.position.x, goal.pose.position.y);
      return false;
    }

    return true;
  }

  // 向 cmd_vel 话题发布000的速度信息，机器人停止运动
  void MoveBase::publishZeroVelocity(){
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = 0.0;
    cmd_vel.linear.y = 0.0;
    cmd_vel.angular.z = 0.0;
    vel_pub_.publish(cmd_vel);
  }

  // 检查四元数是否有效，即四元数的z轴必须接近垂直方向。
  bool MoveBase::isQuaternionValid(const geometry_msgs::Quaternion& q){
    //first we need to check if the quaternion has nan's or infs
    // 首先，我们需要检查四元数是否具有 nan's（非数字）或 infs（有限值）
    // std::isfinite(x) 函数返回x是否为有限值。有限值是既不是无限也不是 NaN(Not-A-Number)（非数字）的任何浮点值。
    if(!std::isfinite(q.x) || !std::isfinite(q.y) || !std::isfinite(q.z) || !std::isfinite(q.w)){
      ROS_ERROR("Quaternion has nans or infs... discarding as a navigation goal");
      return false;
    }

    tf::Quaternion tf_q(q.x, q.y, q.z, q.w);

    //next, we need to check if the length of the quaternion is close to zero
    // 接下来，我们需要检查四元数的长度是否接近零
    // tf_q.length2() 函数返回四元数 tf_q 的长度平方
    if(tf_q.length2() < 1e-6){
      // 四元数的长度接近零...放弃作为导航目标
      ROS_ERROR("Quaternion has length close to zero... discarding as navigation goal");
      return false;
    }

    //next, we'll normalize the quaternion and check that it transforms the vertical vector correctly
    // 接下来，我们将对四元数进行归一化并检查其是否正确转换了垂直向量
    tf_q.normalize();  // 返回与向量 tf_q 同方向的单位向量

    tf::Vector3 up(0, 0, 1);

    // tf_q.getAxis() 函数返回此四元数表示的旋转轴（旋转向量）。正常情况下应为z轴，即 tf_q.getAxis() 为 (0, 0, 1)。
    // tf_q.getAngle() 函数返回此四元数表示的旋转角度[0, 2Pi]。
    // up.rotate(tf_q.getAxis(), tf_q.getAngle()) 函数表示向量 up 绕向量 tf_q.getAxis() 旋转角度 tf_q.getAngle()。
    // 正常情况下应为 up 绕 z轴单位向量(0, 0, 1)旋转，所以结果为 up。
    // up.dot() 函数返回两个向量的点积。正常情况下结果为1。
    double dot = up.dot(up.rotate(tf_q.getAxis(), tf_q.getAngle()));

    if(fabs(dot - 1) > 1e-3){
      // 四元数无效...要进行导航，四元数的z轴必须接近垂直方向。
      ROS_ERROR("Quaternion is invalid... for navigation the z-axis of the quaternion must be close to vertical.");
      return false;
    }

    return true;
  }

  // 把目标点 goal_pose_msg 的坐标变换到 global planner 的坐标系下，global planner 所在的坐标系一般为"map"
  geometry_msgs::PoseStamped MoveBase::goalToGlobalFrame(const geometry_msgs::PoseStamped& goal_pose_msg){
    // 获取代价地图 planner_costmap_ros_ 的全局坐标系名称，一般为"map"
    std::string global_frame = planner_costmap_ros_->getGlobalFrameID();
    tf::Stamped<tf::Pose> goal_pose, global_pose;
    // poseStampedMsgToTF() 函数将 PoseStamped msg 转换为 Stamped<Pose>
    poseStampedMsgToTF(goal_pose_msg, goal_pose);

    //just get the latest available transform... for accuracy they should send
    //goals in the frame of the planner
    // 只需获取最新的可用转换...为了准确起见，他们应该在规划器的坐标系内发送目标
    goal_pose.stamp_ = ros::Time();

    try{
      // 把 goal_pose 从自身坐标系 goal_pose.frame_id_ 变换到 global_frame 坐标系下，
      // 即获取 goal_pose 相对于 global_frame 坐标系下的坐标，并赋值给 global_pose。
      tf_.transformPose(global_frame, goal_pose, global_pose);
    }
    catch(tf::TransformException& ex){
      ROS_WARN("Failed to transform the goal pose from %s into the %s frame: %s",
          goal_pose.frame_id_.c_str(), global_frame.c_str(), ex.what());
      return goal_pose_msg;
    }

    geometry_msgs::PoseStamped global_pose_msg;
    // tf::poseStampedTFToMsg() 函数将 Stamped<Pose> 转换为 PoseStamped msg
    tf::poseStampedTFToMsg(global_pose, global_pose_msg);
    return global_pose_msg;
  }

  // wakePlanner() 为定时器函数，主要工作是唤醒全局路径规划线程 planner_thread_。
  // 如果还没到规划周期则定时器休眠，在定时器函数中通过 planner_cond_ 唤醒线程。
  void MoveBase::wakePlanner(const ros::TimerEvent& event)
  {
    // we have slept long enough for rate
    // 我们已经休眠了足够长的时间
    planner_cond_.notify_one();  // 唤醒全局路径规划线程 planner_thread_
  }

  // 这是全局路径规划线程 planner_thread_ 的入口。这个函数需要等待 actionlib 服务器的 CallBack 函数 MoveBase::executeCb 来唤醒启动。
  // 主要作用是调用全局路径规划获取路径，同时保证规划的周期性以及规划超时清除 goal。
  void MoveBase::planThread(){
    ROS_DEBUG_NAMED("move_base_plan_thread","Starting planner thread...");
    ros::NodeHandle n;
    ros::Timer timer;
    bool wait_for_wake = false;
    boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
    while(n.ok()){
      //check if we should run the planner (the mutex is locked)
      // 检查我们是否应该运行规划器（互斥锁已锁定）
      // 等待唤醒同步，等待 executeCb 函数使得 runPlanner_ = true
      while(wait_for_wake || !runPlanner_){
        //if we should not be running the planner then suspend this thread
        // 如果我们不应该运行规划器，则暂停该线程
        ROS_DEBUG_NAMED("move_base_plan_thread","Planner thread is suspending");
        planner_cond_.wait(lock);  // 使线程进入睡眠，等待 MoveBase::executeCb，以及规划周期的唤醒
        wait_for_wake = false;
      }
      ros::Time start_time = ros::Time::now();

      //time to plan! get a copy of the goal and unlock the mutex
      // 规划路径开始！获得目标点的复制并解锁互斥锁
      // planner_goal_ 是在 MoveBase::executeCb 中得到的目标位姿，需要上锁保证线程安全
      geometry_msgs::PoseStamped temp_goal = planner_goal_;
      lock.unlock();
      ROS_DEBUG_NAMED("move_base_plan_thread","Planning...");

      //run planner
      // 运行全局路径规划器
      planner_plan_->clear();  // 清除原来规划出的路径向量
      // MoveBase::makePlan 作用是获取机器人的位姿作为起点，然后调用全局规划器的 makePlan 返回规划路径，存储在 planner_plan_
      bool gotPlan = n.ok() && makePlan(temp_goal, *planner_plan_);  // 全局规划函数

      // 如果规划出路径则更新相应路径，并将 state_ 转换为 CONTROLLING 状态
      if(gotPlan){
        // 成功规划路径
        ROS_DEBUG_NAMED("move_base_plan_thread","Got Plan with %zu points!", planner_plan_->size());
        //pointer swap the plans under mutex (the controller will pull from latest_plan_)
        // 在互斥锁下指针指向的路径规划进行交换（控制器将从 latest_plan_ 中拉出）
        std::vector<geometry_msgs::PoseStamped>* temp_plan = planner_plan_;

        lock.lock();
        planner_plan_ = latest_plan_;
        // 将最新的全局路径放到 latest_plan_ 中，其在 MoveBase::executeCycle 中被传递到 controller_plan_ 中，利用锁来进行同步
        latest_plan_ = temp_plan;
        last_valid_plan_ = ros::Time::now();
        planning_retries_ = 0;
        new_global_plan_ = true;  // 新的全局规划路径

        ROS_DEBUG_NAMED("move_base_plan_thread","Generated a plan from the base_global_planner");

        //make sure we only start the controller if we still haven't reached the goal
        // 确保仅在仍未到达目标点时才启动控制器
        if(runPlanner_)
          state_ = CONTROLLING;
        if(planner_frequency_ <= 0)
          runPlanner_ = false;
        lock.unlock();
      }
      //if we didn't get a plan and we are in the planning state (the robot isn't moving)
      // 如果我们没有成功规划出路径，并且我们处于 PLANNING 状态（机器人没有移动）
      // 如果没有规划出路径，并且处于 PLANNING 状态，则判断是否超过最大规划周期或者规划次数。
      // 如果是则进入自转模式，否则应该会等待 MoveBase::executeCycle 的唤醒再次规划。
      // 仅在 MoveBase::executeCb 及其调用的 MoveBase::executeCycle，
      // 或者重置状态时会被设置为 PLANNING，一般是刚获得新目标，或者得到路径但计算不出下一步控制时重新进行路径规划。
      else if(state_==PLANNING){
        ROS_DEBUG_NAMED("move_base_plan_thread","No Plan...");
        // 计算尝试规划的最大时间（上一次成功规划的时间加上 planner_patience_）
        ros::Time attempt_end = last_valid_plan_ + ros::Duration(planner_patience_);

        //check if we've tried to make a plan for over our time limit or our maximum number of retries
        //issue #496: we stop planning when one of the conditions is true, but if max_planning_retries_
        //is negative (the default), it is just ignored and we have the same behavior as ever
        // 检查我们是否试图在我们的时限或最大重试次数上规划路径。
        // 问题＃496：当其中一个条件为真时，我们将停止规划路径。
        // 但是如果 max_planning_retries_ 为负（默认值），则将其忽略，并且行为与以往相同。
        lock.lock();
        planning_retries_++;
        // 若规划时间超时或次数超限，则进入 CLEARING 状态，recovery 恢复模式
        if(runPlanner_ &&
           (ros::Time::now() > attempt_end || planning_retries_ > uint32_t(max_planning_retries_))){
          //we'll move into our obstacle clearing mode
          // 我们将进入障碍物清除模式
          state_ = CLEARING;
          runPlanner_ = false;  // proper solution for issue #523
          publishZeroVelocity();  // 直接向 cmd_vel 话题发布000的速度信息，机器人停止运动
          recovery_trigger_ = PLANNING_R;
        }

        lock.unlock();
      }

      //take the mutex for the next iteration
      // 将互斥量用于下一次迭代
      lock.lock();

      //setup sleep interface if needed
      // 如果需要，设置休眠接口
      // 如果还没到规划周期则定时器休眠，在定时器中断中通过 planner_cond_ 唤醒，这里规划周期为0
      if(planner_frequency_ > 0){
        // 计算休眠时间
        ros::Duration sleep_time = (start_time + ros::Duration(1.0/planner_frequency_)) - ros::Time::now();
        if (sleep_time > ros::Duration(0.0)){
          wait_for_wake = true;
          // 定时器，多久没有规划路径，就通知一次规划路径。
          // 小于要求的规划时间，则 sleep 多余的时间来唤醒该进程，wait_for_wake 为 True 后下一个循环线程将进入等待状态，
          // 定时器函数 wakePlanner 将唤醒本线程，若 planner_frequency = 0 则只在有新目标时候才唤醒本线程。
          timer = n.createTimer(sleep_time, &MoveBase::wakePlanner, this);
        }
      }
    }
  }

  // movebase 的 actionlib 服务的回调函数
  // 第一次接收到 goal 时会进入该函数，但如果没有完成任务，尚未退出时，再有接收到 goal 并不会再新建线程进入一次
  // （应该也可以这样操作，这里并没有这样选择），而是通过抢断信号的形式通知该函数，
  // 所以在处理 goal 的时候需要经常查看 isPreemptRequested 函数的返回，看是否有抢占。
  void MoveBase::executeCb(const move_base_msgs::MoveBaseGoalConstPtr& move_base_goal)
  {
    // 判断 goal 有效性，检查四元数是否有效，即四元数的z轴必须接近垂直方向。
    if(!isQuaternionValid(move_base_goal->target_pose.pose.orientation)){
      as_->setAborted(move_base_msgs::MoveBaseResult(), "Aborting on goal because it was sent with an invalid quaternion");
      return;
    }

    // 统一转换到全局坐标系
    // 把目标点 move_base_goal->target_pose 的坐标变换到 global planner 的坐标系下，global planner 所在的坐标系一般为"map"
    geometry_msgs::PoseStamped goal = goalToGlobalFrame(move_base_goal->target_pose);

    //we have a goal so start the planner
    // 我们有一个目标点，所以开启规划器
    // 启动新线程来获取规划路径，唤醒 planThread 线程开始规划
    boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
    planner_goal_ = goal;
    runPlanner_ = true;
    // 唤醒等待条件变量的一个线程：即调用 planner_cond_.wait() 的 MoveBase::planThread()
    planner_cond_.notify_one();  // 启用一个线程
    lock.unlock();

    // 在 topic "move_base/current_goal" 上发布目标点 goal，这个话题貌似只有一些rviz上用来显示的
    current_goal_pub_.publish(goal);
    std::vector<geometry_msgs::PoseStamped> global_plan;

    ros::Rate r(controller_frequency_);  // controller_frequency_ 默认为20.0
    if(shutdown_costmaps_){  // shutdown_costmaps_ 默认为 false
      ROS_DEBUG_NAMED("move_base","Starting up costmaps that were shut down previously");
      planner_costmap_ros_->start();     // 开启 global costmap
      controller_costmap_ros_->start();  // 开启 local costmap
    }

    //we want to make sure that we reset the last time we had a valid plan and control
    // 我们想确保我们重设了上一次拥有有效的规划和控制时的时间
    last_valid_control_ = ros::Time::now();
    last_valid_plan_ = ros::Time::now();
    last_oscillation_reset_ = ros::Time::now();
    planning_retries_ = 0;

    ros::NodeHandle n;
    while(n.ok())
    {
      // //更改控制周期，设置控制器频率为 controller_frequency_
      if(c_freq_change_)
      {
        ROS_INFO("Setting controller frequency to %.2f", controller_frequency_);
        r = ros::Rate(controller_frequency_);
        c_freq_change_ = false;
      }

      // 期间会不断检测是否有新的 goal 抢占，或者坐标系变换等，如果有则在 while 循环中重复初始化再跟随
      // 是否有抢占请求，SimpleActionServer 的政策是，新的 goal 都会抢占旧的 goal，这里应该只是为了清除新 goal 的一些状态。
      // （那些待定的 goal 也有可能抢占，或者可以直接 cancel 抢占 Current？）
      if(as_->isPreemptRequested()){  // 被抢占了(可能是发出新的 goal，也可能是取消了)
        if(as_->isNewGoalAvailable()){  // 发布新的 goal，如果是新的 goal 这个函数会将其他 goal 设置为被抢占状态
          //if we're active and a new goal is available, we'll accept it, but we won't shut anything down
          // 如果我们是激活状态并且有新的可用目标点，我们会接受目标点，但是我们不会关闭任何东西
          move_base_msgs::MoveBaseGoal new_goal = *as_->acceptNewGoal();  // 接收新的目标点

          // 判断 goal 有效性，检查四元数是否有效，即四元数的z轴必须接近垂直方向。
          if(!isQuaternionValid(new_goal.target_pose.pose.orientation)){
            as_->setAborted(move_base_msgs::MoveBaseResult(), "Aborting on goal because it was sent with an invalid quaternion");
            return;  // 无效退出
          }

          // 统一转换到全局坐标系
          // 把目标点 move_base_goal->target_pose 的坐标变换到 global planner 的坐标系下，global planner 所在的坐标系一般为"map"
          goal = goalToGlobalFrame(new_goal.target_pose);

          //we'll make sure that we reset our state for the next execution cycle
          // 我们将确保在下一个执行周期重置状态
          recovery_index_ = 0;
          state_ = PLANNING;

          //we have a new goal so make sure the planner is awake
          // 我们有一个新的目标点，所以确保规划器是开启的
          // 启动新线程来获取规划路径，唤醒 planThread 线程开始规划
          lock.lock();
          planner_goal_ = goal;
          runPlanner_ = true;
          // 唤醒等待条件变量的一个线程：即调用 planner_cond_.wait() 的 MoveBase::planThread()
          planner_cond_.notify_one();  // 启用一个线程
          lock.unlock();

          //publish the goal point to the visualizer
          // 将目标点发布到 visualizer
          // 在 topic "move_base/current_goal" 上发布目标点 goal，这个话题貌似只有一些rviz上用来显示的
          ROS_DEBUG_NAMED("move_base","move_base has received a goal of x: %.2f, y: %.2f", goal.pose.position.x, goal.pose.position.y);
          current_goal_pub_.publish(goal);

          //make sure to reset our timeouts and counters
          // 确保重置我们的超时和计数器
          last_valid_control_ = ros::Time::now();
          last_valid_plan_ = ros::Time::now();
          last_oscillation_reset_ = ros::Time::now();
          planning_retries_ = 0;
        }
        else {  // 如果是 cancel 了
          //if we've been preempted explicitly we need to shut things down
          // 如果我们被明确抢占（preempted）了，我们需要关闭一切
          resetState();  // 停止规划线程、停车等

          //notify the ActionServer that we've successfully preempted
          // 通知 ActionServer 我们已成功抢占
          ROS_DEBUG_NAMED("move_base","Move base preempting the current goal");
          as_->setPreempted();  // 设置 current goal 被抢占

          //we'll actually return from execute after preempting
          // 实际上，抢占之后我们将从执行返回
          return;
        }
      }

      //we also want to check if we've changed global frames because we need to transform our goal pose
      // 我们还想检查是否改变了全局坐标系，因为我们需要改变目标点位姿
      if(goal.header.frame_id != planner_costmap_ros_->getGlobalFrameID()){
        goal = goalToGlobalFrame(goal);  // 判断这段时间是否改了坐标系

        //we want to go back to the planning state for the next execution cycle
        // 我们想回到下一个执行周期的规划状态
        recovery_index_ = 0;
        state_ = PLANNING;

        //we have a new goal so make sure the planner is awake
        // 我们有一个新的目标点，所以确保规划器是开启的
        // 启动新线程来获取规划路径，唤醒 planThread 线程开始规划
        lock.lock();
        planner_goal_ = goal;
        runPlanner_ = true;
        // 唤醒等待条件变量的一个线程：即调用 planner_cond_.wait() 的 MoveBase::planThread()
        planner_cond_.notify_one();  // 启用一个线程
        lock.unlock();

        //publish the goal point to the visualizer
        // 将目标点发布到 visualizer
        // 在 topic "move_base/current_goal" 上发布目标点 goal，这个话题貌似只有一些rviz上用来显示的
        ROS_DEBUG_NAMED("move_base","The global frame for move_base has changed, new frame: %s, new goal position x: %.2f, y: %.2f", goal.header.frame_id.c_str(), goal.pose.position.x, goal.pose.position.y);
        current_goal_pub_.publish(goal);

        //make sure to reset our timeouts and counters
        // 确保重置我们的超时和计数器
        last_valid_control_ = ros::Time::now();
        last_valid_plan_ = ros::Time::now();
        last_oscillation_reset_ = ros::Time::now();
        planning_retries_ = 0;
      }

      //for timing that gives real time even in simulation
      // 即使在仿真中也能提供实时的时序
      ros::WallTime start = ros::WallTime::now();

      //the real work on pursuing a goal is done here
      // 跟随目标点的真正工作在这里完成
      // 这是控制机器人跟踪的主要函数，即主要工作在 executeCycle() 函数中
      bool done = executeCycle(goal, global_plan);

      //if we're done, then we'll return from execute
      // 如果我们完成了，那么我们将从执行返回
      // 完成目标终止回调
      if(done)
        return;

      //check if execution of the goal has completed in some way
      // 检查目标的执行是否以某种方式完成

      ros::WallDuration t_diff = ros::WallTime::now() - start;
      ROS_DEBUG_NAMED("move_base","Full control cycle time: %.9f\n", t_diff.toSec());

      r.sleep();
      //make sure to sleep for the remainder of our cycle time
      // 确保在我们剩余的周期时间里休眠
      // 这个是一般的警告信息，规划的时间超时
      if(r.cycleTime() > ros::Duration(1 / controller_frequency_) && state_ == CONTROLLING)
        ROS_WARN("Control loop missed its desired rate of %.4fHz... the loop actually took %.4f seconds", controller_frequency_, r.cycleTime().toSec());
    }

    // ros不ok时清除退出

    //wake up the planner thread so that it can exit cleanly
    // 唤醒规划线程，使其可以干净地退出
    lock.lock();
    runPlanner_ = true;
    planner_cond_.notify_one();
    lock.unlock();

    //if the node is killed then we'll abort and return
    // 如果 node 被关闭了，那么我们将中止并返回
    as_->setAborted(move_base_msgs::MoveBaseResult(), "Aborting on the goal because the node has been killed");
    return;
  }

  double MoveBase::distance(const geometry_msgs::PoseStamped& p1, const geometry_msgs::PoseStamped& p2)
  {
    return hypot(p1.pose.position.x - p2.pose.position.x, p1.pose.position.y - p2.pose.position.y);
  }

  // 该函数的两个参数分别是目标点位姿 goal 以及规划出的全局路径 global_plan。
  // 实现的是通过上述两个已知，利用局部路径规划器直接输出轮子速度，控制机器人按照路径走到目标点，成功返回真，否则返回假。
  // 在actionlib server 的回调 MoveBase::executeCb 中被调用。
  bool MoveBase::executeCycle(geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& global_plan){
    boost::recursive_mutex::scoped_lock ecl(configuration_mutex_);
    // 变量定义并获取机器人坐标发布给 server 的 feedback
    //we need to be able to publish velocity commands
    // 我们需要能够发布速度命令
    geometry_msgs::Twist cmd_vel;    // 发布速度 topic

    //update feedback to correspond to our curent position
    // 更新 feedback 以符合我们当前的位置
    // 获取机器人坐标
    tf::Stamped<tf::Pose> global_pose;
    planner_costmap_ros_->getRobotPose(global_pose);
    geometry_msgs::PoseStamped current_position;
    tf::poseStampedTFToMsg(global_pose, current_position);

    //push the feedback out
    // 发布 feedback
    // 反馈 goal 服务状态，把机器人坐标发布给 server 的 feedback
    move_base_msgs::MoveBaseFeedback feedback;
    feedback.base_position = current_position;
    as_->publishFeedback(feedback);

    //check to see if we've moved far enough to reset our oscillation timeout
    // 检查我们是否已经移动到足够远，以重置振荡超时
    // 需要确认小车不会来回震荡，这通过判断是否运行了超过一定距离实现
    if(distance(current_position, oscillation_pose_) >= oscillation_distance_)
    {
      last_oscillation_reset_ = ros::Time::now();
      oscillation_pose_ = current_position;

      //if our last recovery was caused by oscillation, we want to reset the recovery index 
      // 如果我们的上一次 recovery 是由振荡引起的，我们想重置 recovery index
      if(recovery_trigger_ == OSCILLATION_R)
        recovery_index_ = 0;
    }

    //check that the observation buffers for the costmap are current, we don't want to drive blind
    // 检查 costmap 的观察缓冲区是否是最新的，我们不想盲目移动
    // 需要确认代价地图是否时刻在更新，如果观测传感器数据不够新，则让机器人停机并退出函数
    if(!controller_costmap_ros_->isCurrent()){
      ROS_WARN("[%s]:Sensor data is out of date, we're not going to allow commanding of the base for safety",ros::this_node::getName().c_str());
      publishZeroVelocity();  // 让机器人停止运动
      return false;
    }

    //if we have a new plan then grab it and give it to the controller
    // 如果我们有一个新的规划路径，那么抓住它并将其交给控制器（局部路径规划器）
    // 变量 new_global_plan_ 在规划器线程 planThread 中，当新的全局路径被规划出来，该值被置1
    // 新的全局路径生成给局部控制器
    if(new_global_plan_){
      //make sure to set the new plan flag to false
      // 确保将新路径规划标志设置为false
      new_global_plan_ = false;

      ROS_DEBUG_NAMED("move_base","Got a new plan...swap pointers");

      //do a pointer swap under mutex
      // 在互斥锁下进行指针交换
      // 完成 latest_plan_ 到 controller_plan_ 的转换
      std::vector<geometry_msgs::PoseStamped>* temp_plan = controller_plan_;

      boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
      controller_plan_ = latest_plan_;  // controller_plan_ 指向最新规划的路径
      latest_plan_ = temp_plan;
      lock.unlock();
      ROS_DEBUG_NAMED("move_base","pointers swapped!");

      if(!tc_->setPlan(*controller_plan_)){  // 将全局路径设置到局部路径规划器中
        //ABORT and SHUTDOWN COSTMAPS
        ROS_ERROR("Failed to pass global plan to the controller, aborting.");
        resetState();  // 重置状态

        //disable the planner thread
        // 同时也关闭规划器线程，没必要规划了
        lock.lock();
        runPlanner_ = false;
        lock.unlock();

        // 无法将全局规划路径传递给局部路径规划器
        as_->setAborted(move_base_msgs::MoveBaseResult(), "Failed to pass global plan to the controller.");
        return true;
      }

      //make sure to reset recovery_index_ since we were able to find a valid plan
      // 请确保重置 recovery_index_，因为我们能够找到有效的规划路径
      // 如果全局路径有效，则不需要 recovery
      if(recovery_trigger_ == PLANNING_R)
        recovery_index_ = 0;
    }

    //the move_base state machine, handles the control logic for navigation
    // move_base 状态机，处理导航的控制逻辑
    // 对状态机进行处理
    switch(state_){
      //if we are in a planning state, then we'll attempt to make a plan
      // 如果我们处于 PLANNING（路径规划）状态，那么我们将尝试规划路径
      case PLANNING:
        {
          // 唤醒规划线程
          boost::recursive_mutex::scoped_lock lock(planner_mutex_);
          runPlanner_ = true;
          planner_cond_.notify_one();
        }
        ROS_DEBUG_NAMED("move_base","Waiting for plan, in the planning state.");
        break;

      //if we're controlling, we'll attempt to find valid velocity commands
      // 如果我们在 CONTROLLING（控制）状态，我们将尝试找到有效的速度命令
      case CONTROLLING:
        ROS_DEBUG_NAMED("move_base","In controlling state.");

        //check to see if we've reached our goal
        // 检查我们是否到达目标点
        if(tc_->isGoalReached()){  // 如果已经到达目标点
          ROS_DEBUG_NAMED("move_base","Goal reached!");
          // 重置状态
          resetState();

          //disable the planner thread
          // 关闭规划器线程
          boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
          runPlanner_ = false;
          lock.unlock();

          // 设置告知 Client 结果
          as_->setSucceeded(move_base_msgs::MoveBaseResult(), "Goal reached.");
          return true;
        }

        //check for an oscillation condition
        // 检查振荡条件
        // last_oscillation_reset_ 获得新目标会重置，距离超过震荡距离（默认0.5）会重置，进行 recovery 后会重置。
        // 所以是太久没有发生上面的事（走出方圆 oscillation_distance 的距离）就震动一下，防止长时间在同一个地方徘徊？？？？这里 oscillation_timeout_ 默认为0，不发生。
        // 陷在方圆 oscillation_distance 达 oscillation_timeout 之久，认定机器人在震荡，从而做异常处理。
        if(oscillation_timeout_ > 0.0 &&
            last_oscillation_reset_ + ros::Duration(oscillation_timeout_) < ros::Time::now())
        {
          publishZeroVelocity();
          state_ = CLEARING;
          recovery_trigger_ = OSCILLATION_R;
        }
        
        {
         boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*(controller_costmap_ros_->getCostmap()->getMutex()));
        
        // 调用 base_local_planner（默认为DWA）的计算速度函数，成功则下发给 base_controller。
        // 如果失败，说明遇到了障碍物需要重新规划。当然重新规划有个次数限制，超过了就宣告失败，进入清扫过程了，
        // 按源码的逻辑，这次导航就快没救了，需要关掉节点再重来。。。
        if(tc_->computeVelocityCommands(cmd_vel)){  // 局部路径规划成功，计算速度
          ROS_DEBUG_NAMED( "move_base", "Got a valid command from the local planner: %.3lf, %.3lf, %.3lf",
                           cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z );
          last_valid_control_ = ros::Time::now();
          //make sure that we send the velocity command to the base
          // 确保我们将速度命令发送给机器人
          vel_pub_.publish(cmd_vel);  // 发布控制速度信息
          if(recovery_trigger_ == CONTROLLING_R)
            recovery_index_ = 0;
        }
        else {  // 局部规划失败
          ROS_DEBUG_NAMED("move_base", "The local planner could not find a valid plan.");
          ros::Time attempt_end = last_valid_control_ + ros::Duration(controller_patience_);

          //check if we've tried to find a valid control for longer than our time limit
          // 检查我们是否试图找到有效控制（局部规划）的时间超过了我们的时间限制
          // 判断是否控制超时
          if(ros::Time::now() > attempt_end){
            //we'll move into our obstacle clearing mode
            // 我们将进入障碍物清除模式
            publishZeroVelocity();
            state_ = CLEARING;
            recovery_trigger_ = CONTROLLING_R;
          }
          else{
            //otherwise, if we can't find a valid control, we'll go back to planning
            // 否则，如果我们找不到有效的控制（局部规划），我们将返回规划路径
            // 没超时则启动规划器线程重新规划。
            // 下面这一段是避障绕行的关键，这里直接粗暴地发布一个零速度让车停下，再重新唤醒 planThread，
            // 实操发现这样对车的电机加速度要求很高，很容易抽风～
            last_valid_plan_ = ros::Time::now();
            planning_retries_ = 0;
            state_ = PLANNING;
            publishZeroVelocity();

            //enable the planner thread in case it isn't running on a clock
            // 启用规划器线程，以防它不在时钟上运行
            boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
            runPlanner_ = true;
            planner_cond_.notify_one();
            lock.unlock();
          }
        }
        }

        break;

      //we'll try to clear out space with any user-provided recovery behaviors
      // 我们将尝试通过用户提供的任何恢复行为来清理空间
      case CLEARING:
        // 三种原因需要 recovery（仅有全局规划失败、局部规划失败、长时间困在一片小区域三种原因），
        // 则每次尝试一种 recovery 方法，直到所有尝试完。
        ROS_DEBUG_NAMED("move_base","In clearing/recovery state");
        //we'll invoke whatever recovery behavior we're currently on if they're enabled
        // 如果启用了它们，我们将调用当前正在执行的任何恢复行为
        if(recovery_behavior_enabled_ && recovery_index_ < recovery_behaviors_.size()){  // 遍历 recovery 方法
          ROS_DEBUG_NAMED("move_base_recovery","Executing behavior %u of %zu", recovery_index_, recovery_behaviors_.size());
          recovery_behaviors_[recovery_index_]->runBehavior();

          //we at least want to give the robot some time to stop oscillating after executing the behavior
          // 我们至少要给机器人一些时间，使其在执行行为后停止振荡
          last_oscillation_reset_ = ros::Time::now();

          //we'll check if the recovery behavior actually worked
          ROS_DEBUG_NAMED("move_base_recovery","Going back to planning state");
          last_valid_plan_ = ros::Time::now();
          planning_retries_ = 0;
          state_ = PLANNING;

          //update the index of the next recovery behavior that we'll try
          recovery_index_++;
        }
        else{
          ROS_DEBUG_NAMED("move_base_recovery","All recovery behaviors have failed, locking the planner and disabling it.");
          //disable the planner thread
          boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
          runPlanner_ = false;
          lock.unlock();

          ROS_DEBUG_NAMED("move_base_recovery","Something should abort after this.");

          if(recovery_trigger_ == CONTROLLING_R){
            ROS_ERROR("Aborting because a valid control could not be found. Even after executing all recovery behaviors");
            as_->setAborted(move_base_msgs::MoveBaseResult(), "Failed to find a valid control. Even after executing recovery behaviors.");
          }
          else if(recovery_trigger_ == PLANNING_R){
            ROS_ERROR("Aborting because a valid plan could not be found. Even after executing all recovery behaviors");
            as_->setAborted(move_base_msgs::MoveBaseResult(), "Failed to find a valid plan. Even after executing recovery behaviors.");
          }
          else if(recovery_trigger_ == OSCILLATION_R){
            ROS_ERROR("Aborting because the robot appears to be oscillating over and over. Even after executing all recovery behaviors");
            as_->setAborted(move_base_msgs::MoveBaseResult(), "Robot is oscillating. Even after executing recovery behaviors.");
          }
          resetState();
          return true;
        }
        break;
      default:
        ROS_ERROR("This case should never be reached, something is wrong, aborting");
        resetState();
        //disable the planner thread
        boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
        runPlanner_ = false;
        lock.unlock();
        as_->setAborted(move_base_msgs::MoveBaseResult(), "Reached a case that should not be hit in move_base. This is a bug, please report it.");
        return true;
    }

    //we aren't done yet
    return false;
  }

  bool MoveBase::loadRecoveryBehaviors(ros::NodeHandle node){
    XmlRpc::XmlRpcValue behavior_list;
    if(node.getParam("recovery_behaviors", behavior_list)){
      if(behavior_list.getType() == XmlRpc::XmlRpcValue::TypeArray){
        for(int i = 0; i < behavior_list.size(); ++i){
          if(behavior_list[i].getType() == XmlRpc::XmlRpcValue::TypeStruct){
            if(behavior_list[i].hasMember("name") && behavior_list[i].hasMember("type")){
              //check for recovery behaviors with the same name
              for(int j = i + 1; j < behavior_list.size(); j++){
                if(behavior_list[j].getType() == XmlRpc::XmlRpcValue::TypeStruct){
                  if(behavior_list[j].hasMember("name") && behavior_list[j].hasMember("type")){
                    std::string name_i = behavior_list[i]["name"];
                    std::string name_j = behavior_list[j]["name"];
                    if(name_i == name_j){
                      ROS_ERROR("A recovery behavior with the name %s already exists, this is not allowed. Using the default recovery behaviors instead.", 
                          name_i.c_str());
                      return false;
                    }
                  }
                }
              }
            }
            else{
              ROS_ERROR("Recovery behaviors must have a name and a type and this does not. Using the default recovery behaviors instead.");
              return false;
            }
          }
          else{
            ROS_ERROR("Recovery behaviors must be specified as maps, but they are XmlRpcType %d. We'll use the default recovery behaviors instead.",
                behavior_list[i].getType());
            return false;
          }
        }

        //if we've made it to this point, we know that the list is legal so we'll create all the recovery behaviors
        for(int i = 0; i < behavior_list.size(); ++i){
          try{
            //check if a non fully qualified name has potentially been passed in
            if(!recovery_loader_.isClassAvailable(behavior_list[i]["type"])){
              std::vector<std::string> classes = recovery_loader_.getDeclaredClasses();
              for(unsigned int i = 0; i < classes.size(); ++i){
                if(behavior_list[i]["type"] == recovery_loader_.getName(classes[i])){
                  //if we've found a match... we'll get the fully qualified name and break out of the loop
                  ROS_WARN("Recovery behavior specifications should now include the package name. You are using a deprecated API. Please switch from %s to %s in your yaml file.",
                      std::string(behavior_list[i]["type"]).c_str(), classes[i].c_str());
                  behavior_list[i]["type"] = classes[i];
                  break;
                }
              }
            }

            boost::shared_ptr<nav_core::RecoveryBehavior> behavior(recovery_loader_.createInstance(behavior_list[i]["type"]));

            //shouldn't be possible, but it won't hurt to check
            if(behavior.get() == NULL){
              ROS_ERROR("The ClassLoader returned a null pointer without throwing an exception. This should not happen");
              return false;
            }

            //initialize the recovery behavior with its name
            behavior->initialize(behavior_list[i]["name"], &tf_, planner_costmap_ros_, controller_costmap_ros_);
            recovery_behaviors_.push_back(behavior);
          }
          catch(pluginlib::PluginlibException& ex){
            ROS_ERROR("Failed to load a plugin. Using default recovery behaviors. Error: %s", ex.what());
            return false;
          }
        }
      }
      else{
        ROS_ERROR("The recovery behavior specification must be a list, but is of XmlRpcType %d. We'll use the default recovery behaviors instead.", 
            behavior_list.getType());
        return false;
      }
    }
    else{
      //if no recovery_behaviors are specified, we'll just load the defaults
      return false;
    }

    //if we've made it here... we've constructed a recovery behavior list successfully
    return true;
  }

  //we'll load our default recovery behaviors here
  void MoveBase::loadDefaultRecoveryBehaviors(){
    recovery_behaviors_.clear();
    try{
      //we need to set some parameters based on what's been passed in to us to maintain backwards compatibility
      ros::NodeHandle n("~");
      n.setParam("conservative_reset/reset_distance", conservative_reset_dist_);
      n.setParam("aggressive_reset/reset_distance", circumscribed_radius_ * 4);

      //first, we'll load a recovery behavior to clear the costmap
      boost::shared_ptr<nav_core::RecoveryBehavior> cons_clear(recovery_loader_.createInstance("clear_costmap_recovery/ClearCostmapRecovery"));
      cons_clear->initialize("conservative_reset", &tf_, planner_costmap_ros_, controller_costmap_ros_);
      recovery_behaviors_.push_back(cons_clear);

      //next, we'll load a recovery behavior to rotate in place
      boost::shared_ptr<nav_core::RecoveryBehavior> rotate(recovery_loader_.createInstance("rotate_recovery/RotateRecovery"));
      if(clearing_rotation_allowed_){
        rotate->initialize("rotate_recovery", &tf_, planner_costmap_ros_, controller_costmap_ros_);
        recovery_behaviors_.push_back(rotate);
      }

      //next, we'll load a recovery behavior that will do an aggressive reset of the costmap
      boost::shared_ptr<nav_core::RecoveryBehavior> ags_clear(recovery_loader_.createInstance("clear_costmap_recovery/ClearCostmapRecovery"));
      ags_clear->initialize("aggressive_reset", &tf_, planner_costmap_ros_, controller_costmap_ros_);
      recovery_behaviors_.push_back(ags_clear);

      //we'll rotate in-place one more time
      if(clearing_rotation_allowed_)
        recovery_behaviors_.push_back(rotate);
    }
    catch(pluginlib::PluginlibException& ex){
      ROS_FATAL("Failed to load a plugin. This should not happen on default recovery behaviors. Error: %s", ex.what());
    }

    return;
  }

  void MoveBase::resetState(){
    // Disable the planner thread
    // 禁用路径规划器线程
    boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
    runPlanner_ = false;
    lock.unlock();

    // Reset statemachine
    // 重置状态机
    state_ = PLANNING;
    recovery_index_ = 0;
    recovery_trigger_ = PLANNING_R;
    publishZeroVelocity();

    //if we shutdown our costmaps when we're deactivated... we'll do that now
    // 如果我们在停用后关闭 costmaps...我们将立即执行
    if(shutdown_costmaps_){
      ROS_DEBUG_NAMED("move_base","Stopping costmaps");
      planner_costmap_ros_->stop();
      controller_costmap_ros_->stop();
    }
  }
};
