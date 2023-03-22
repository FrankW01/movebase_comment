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

#include <pluginlib/class_loader.hpp>
#include <std_srvs/Empty.h>

#include <dynamic_reconfigure/server.h>
#include "move_base/MoveBaseConfig.h"

namespace move_base {
  //typedefs to help us out with the action server so that we don't hace to type so much
  typedef actionlib::SimpleActionServer<move_base_msgs::MoveBaseAction> MoveBaseActionServer;

  enum MoveBaseState {//movebase状态枚举，这个是movebase的状态机的移动之间的状态代码
    PLANNING,//规划
    CONTROLLING,//控制
    CLEARING//清空
  }; 

  enum RecoveryTrigger//这个是执行恢复器后也无法恢复的问题报错信息
  {
    PLANNING_R,//没有有效全局规划
    CONTROLLING_R,//没有有效局部规划
    OSCILLATION_R//一直在振荡
  };

  /**
   * @class MoveBase
   * @brief A class that uses the actionlib::ActionServer interface that moves the robot base to a goal location.使用actionlib：：ActionServer接口控制机器人移到目标位置的类。
   */
  class MoveBase {
    public:
      /**
       * @brief  Constructor for the actions
       * @param name The name of the action
       * @param tf A reference to a TransformListener
       */
      MoveBase(tf2_ros::Buffer& tf);//actions的构造函数，输入参数为一个tf引用

      /**
       * @brief  Destructor - Cleans up
       */
      virtual ~MoveBase();

      /**
       * @brief  Performs a control cycle实现一个控制循环
       * @param goal A reference to the goal to pursue
       * @return True if processing of the goal is done, false otherwise//如果这个目标完成了返回true
       */
      bool executeCycle(geometry_msgs::PoseStamped& goal);//输入参数是goal目标点

    private:
      /**
       * @brief  A service call that clears the costmaps of obstacles
       * @param req The service request 
       * @param resp The service response
       * @return True if the service call succeeds, false otherwise
       */
      bool clearCostmapsService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp);//清除障碍物代价地图的服务

      /**
       * @brief  A service call that can be made when the action is inactive that will return a plan
       * @param  req The goal request
       * @param  resp The plan request
       * @return True if planning succeeded, false otherwise
       */
      bool planService(nav_msgs::GetPlan::Request &req, nav_msgs::GetPlan::Response &resp);//当action处于激活状态下，可生成plan的服务

      /**
       * @brief  Make a new global plan
       * @param  goal The goal to plan to
       * @param  plan Will be filled in with the plan made by the planner
       * @return  True if planning succeeds, false otherwise
       */
      bool makePlan(const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan);//做一个新plan的函数，输入为goal，输出为plan点列

      /**
       * @brief  Load the recovery behaviors for the navigation stack from the parameter server
       * @param node The ros::NodeHandle to be used for loading parameters 
       * @return True if the recovery behaviors were loaded successfully, false otherwise
       */
      bool loadRecoveryBehaviors(ros::NodeHandle node);//加载恢复行为对于导航模块来讲

      /**
       * @brief  Loads the default recovery behaviors for the navigation stack
       */
      void loadDefaultRecoveryBehaviors();//加载默认的恢复行为

      /**
       * @brief  Clears obstacles within a window around the robot
       * @param size_x The x size of the window
       * @param size_y The y size of the window
       */
      void clearCostmapWindows(double size_x, double size_y);//清除在机器人附近一个长方形窗口的障碍物

      /**
       * @brief  Publishes a velocity command of zero to the base
       */
      void publishZeroVelocity();//发布一个零速度向底盘，这个是在没有局部规划的时候进行吗？

      /**
       * @brief  Reset the state of the move_base action and send a zero velocity command to the base
       */
      void resetState();//重置movebase 的action并且发布零速度向底盘

      void goalCB(const geometry_msgs::PoseStamped::ConstPtr& goal);//我认为是goal的回调函数

      void planThread();//todo

      void executeCb(const move_base_msgs::MoveBaseGoalConstPtr& move_base_goal);//具体功能待查看，输入参数是move_base_goal

      bool isQuaternionValid(const geometry_msgs::Quaternion& q);//是否为有效的四元数

      bool getRobotPose(geometry_msgs::PoseStamped& global_pose, costmap_2d::Costmap2DROS* costmap);//获得机器人位置，并且更新代价地图

      double distance(const geometry_msgs::PoseStamped& p1, const geometry_msgs::PoseStamped& p2);//两点之间距离

      geometry_msgs::PoseStamped goalToGlobalFrame(const geometry_msgs::PoseStamped& goal_pose_msg);//将goal转换为全局坐标下中？

      /**
       * @brief This is used to wake the planner at periodic intervals.
       */
      void wakePlanner(const ros::TimerEvent& event);//每隔一段时间唤醒规划器？

      tf2_ros::Buffer& tf_;//todo

      MoveBaseActionServer* as_;//action服务器？这个是movebaseactionlib的定义

      boost::shared_ptr<nav_core::BaseLocalPlanner> tc_;//局部规划器
      costmap_2d::Costmap2DROS* planner_costmap_ros_, *controller_costmap_ros_;//两个代价地图,第一个我知道是全局代价地图，局部代价地图

      boost::shared_ptr<nav_core::BaseGlobalPlanner> planner_;//全局规划器
      std::string robot_base_frame_, global_frame_;//这个global_frame是map坐标系，但是robot_base_frame_需要用gdb调试出来,这个是base_link

      std::vector<boost::shared_ptr<nav_core::RecoveryBehavior> > recovery_behaviors_;//恢复行为共享指针向量
      std::vector<std::string> recovery_behavior_names_;//恢复行为名字的向量
      unsigned int recovery_index_;

      geometry_msgs::PoseStamped global_pose_;//全局位置
      double planner_frequency_, controller_frequency_, inscribed_radius_, circumscribed_radius_;
      double planner_patience_, controller_patience_;//双浮点
      int32_t max_planning_retries_;//有符号整形
      uint32_t planning_retries_;//无符号整形
      double conservative_reset_dist_, clearing_radius_;//双浮点
      ros::Publisher current_goal_pub_, vel_pub_, action_goal_pub_, recovery_status_pub_;
      ros::Subscriber goal_sub_;
      ros::ServiceServer make_plan_srv_, clear_costmaps_srv_;//服务的服务器
      bool shutdown_costmaps_, clearing_rotation_allowed_, recovery_behavior_enabled_;
      bool make_plan_clear_costmap_, make_plan_add_unreachable_goal_;//flag变量
      double oscillation_timeout_, oscillation_distance_;

      MoveBaseState state_;//move_base状态
      RecoveryTrigger recovery_trigger_;//恢复触发器状态，initia状态是PLANNING_R

      ros::Time last_valid_plan_, last_valid_control_, last_oscillation_reset_;//时间
      geometry_msgs::PoseStamped oscillation_pose_;//振荡位置
      pluginlib::ClassLoader<nav_core::BaseGlobalPlanner> bgp_loader_;//我认为是全局规划的插件
      pluginlib::ClassLoader<nav_core::BaseLocalPlanner> blp_loader_;//局部规划的插件
      pluginlib::ClassLoader<nav_core::RecoveryBehavior> recovery_loader_;//恢复行为的插件

      //set up plan triple buffer//设置三个缓冲区？
      //下边这三个都是全局路径指针，但是用处，生命周期都不一样
      std::vector<geometry_msgs::PoseStamped>* planner_plan_;//全局路径
      std::vector<geometry_msgs::PoseStamped>* latest_plan_;//全局路径，latest_plan_表示上一次全局规划(makePkan)时规划出的路径。功能是用于同步。在planThread线程执行的makePlan需要点时间，过程中要一直接占用planner_plan_，要是主线程也要读planner_plan_的话，不方便线程同步。
      std::vector<geometry_msgs::PoseStamped>* controller_plan_;//全局路径

      //set up the planner's thread
      bool runPlanner_;//bool向量
      boost::recursive_mutex planner_mutex_;//规划的互斥量
      boost::condition_variable_any planner_cond_;//
          // condition_variable用来唤醒一个或多个等待在某特定条件上的线程，下面列出了condition_variable提供的操作：
    // 所有等待(wait)某个条件的线程都必须使用相同的mutex，且必须使用unique_lock绑定mutex，并且让wait()等待在unique_lock上，否则会发生不明确的行为
      geometry_msgs::PoseStamped planner_goal_;//一个点
      boost::thread* planner_thread_;//一个线程？,这里具体的看一下boost，应该是执行规划的线程,这个线程的功能？这个线程调用的函数planThread
      //todo


      boost::recursive_mutex configuration_mutex_;//可递归上锁的互斥量
      dynamic_reconfigure::Server<move_base::MoveBaseConfig> *dsrv_;//todo.，这是好像是一个动态调参的配置包，还需要继续看一下todo
      
      void reconfigureCB(move_base::MoveBaseConfig &config, uint32_t level);//

      move_base::MoveBaseConfig last_config_;//上一次的配置？
      move_base::MoveBaseConfig default_config_;//默认配置
      bool setup_, p_freq_change_, c_freq_change_;//bool
      bool new_global_plan_;
  };
};
#endif

