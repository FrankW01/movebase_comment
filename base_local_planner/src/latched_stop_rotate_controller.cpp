/*
 * latched_stop_rotate_controller.cpp
 *
 *  Created on: Apr 16, 2012
 *      Author: tkruse
 */

#include <base_local_planner/latched_stop_rotate_controller.h>

#include <cmath>

#include <Eigen/Core>

#include <angles/angles.h>
#include <nav_msgs/Odometry.h>

#include <base_local_planner/goal_functions.h>
#include <base_local_planner/local_planner_limits.h>

#include <tf2/utils.h>

namespace base_local_planner {

LatchedStopRotateController::LatchedStopRotateController(const std::string& name) {
  ros::NodeHandle private_nh("~/" + name);
  private_nh.param("latch_xy_goal_tolerance", latch_xy_goal_tolerance_, false);

  rotating_to_goal_ = false;
}

LatchedStopRotateController::~LatchedStopRotateController() {}


/**
 * returns true if we have passed the goal position.
 * Meaning we might have overshot on the position beyond tolerance, yet still return true.
 * Also goal orientation might not yet be true 如果我们已经超过了目标位置，则返回 true。 这意味着我们可能超出容忍范围，但仍然返回 true。此外，目标方向可能还不正确
 */
bool LatchedStopRotateController::isPositionReached(LocalPlannerUtil* planner_util,
    const geometry_msgs::PoseStamped& global_pose) {
  double xy_goal_tolerance = planner_util->getCurrentLimits().xy_goal_tolerance;

  //we assume the global goal is the last point in the global plan
  geometry_msgs::PoseStamped goal_pose;
  if ( ! planner_util->getGoal(goal_pose)) {
    return false;
  }

  double goal_x = goal_pose.pose.position.x;
  double goal_y = goal_pose.pose.position.y;

  //check to see if we've reached the goal position
  //getGoalPositionDistance返回两点之间距离
  if ((latch_xy_goal_tolerance_ && xy_tolerance_latch_) ||//如果启动了锁存，且xy_tolerance_latch_标志位被标记（即满足过xy_goal_tolerance条件），则认为已经到达
      base_local_planner::getGoalPositionDistance(global_pose, goal_x, goal_y) <= xy_goal_tolerance) {//如果未启用锁存，则需要基于当前位置是否满足xy_goal_tolerance确定是否到达了位置
    xy_tolerance_latch_ = true;
    return true;
  }
  return false;
}


/**
 * returns true if we have passed the goal position and have reached goal orientation.
 * Meaning we might have overshot on the position beyond tolerance, yet still return true. 如果我们已经通过目标位置并达到目标方向，则返回 true。 这意味着我们可能超出容忍范围，但仍然返回 true。
 */
bool LatchedStopRotateController::isGoalReached(LocalPlannerUtil* planner_util,
    OdometryHelperRos& odom_helper,
    const geometry_msgs::PoseStamped& global_pose) {//global_pose。机器人当前位姿
  double xy_goal_tolerance = planner_util->getCurrentLimits().xy_goal_tolerance;
  double theta_stopped_vel = planner_util->getCurrentLimits().theta_stopped_vel;//判断停止的角速度
  double trans_stopped_vel = planner_util->getCurrentLimits().trans_stopped_vel;//判断停止的线速度

  //copy over the odometry information
  nav_msgs::Odometry base_odom;
  odom_helper.getOdom(base_odom);//base_odom。机器人当前线速度、角速度

  //we assume the global goal is the last point in the global plan
  geometry_msgs::PoseStamped goal_pose;
  if ( ! planner_util->getGoal(goal_pose)) {//goal_pose。目标在odom坐标系下的值
    return false;
  }

  double goal_x = goal_pose.pose.position.x;
  double goal_y = goal_pose.pose.position.y;
  //如果不锁存(false)，则必须始终满足机器人当前位置是否满足xy_goal_tolerance的条件，满足则代表到达了目标位置。如果锁存位置就是只要过程中到达一次位置就行，之后就不会再管位置了
  base_local_planner::LocalPlannerLimits limits = planner_util->getCurrentLimits();
  //即位置在xy_goal_tolerance以内，角度在yaw_goal_tolerance以内，且速度小于trans_stopped_velocity，rot_stopped_velocity
  //check to see if we've reached the goal position
  if ((latch_xy_goal_tolerance_ && xy_tolerance_latch_) ||//根据配置变量latch_xy_goal_tolerance_，分为锁存位置和不锁存位置两种处理逻辑
      base_local_planner::getGoalPositionDistance(global_pose, goal_x, goal_y) <= xy_goal_tolerance) {
    //if the user wants to latch goal tolerance, if we ever reach the goal location, we'll
    //just rotate in place
    if (latch_xy_goal_tolerance_ && ! xy_tolerance_latch_) {
      ROS_DEBUG("Goal position reached (check), stopping and turning in place");
      xy_tolerance_latch_ = true;//锁存位置(true)，即如果机器人在行驶过程中出现过位置满足xy_goal_tolerance的条件时，则设置xy_tolerance_latch_为true，代表已经达到过目标位置
    }
    double goal_th = tf2::getYaw(goal_pose.pose.orientation);
    double angle = base_local_planner::getGoalOrientationAngleDifference(global_pose, goal_th);
    //check to see if the goal orientation has been reached
    if (fabs(angle) <= limits.yaw_goal_tolerance) {//后者目标判断定不用再考虑最终位置分量，只旋转至目标角度即可
      //make sure that we're actually stopped before returning success
      if (base_local_planner::stopped(base_odom, theta_stopped_vel, trans_stopped_vel)) {//如果也满足，则判断当前速度是否满足停止条件，即theta速度小于theta_stopped_vel，x和y的速度小于trans_stopped_velocity。
        return true;//只有位置、角度、速度，三个条件都满足的情况下，才算机器人到达了目标位姿，isGoalReached函数才会返回true
      }
    }
  }
  return false;
}

bool LatchedStopRotateController::stopWithAccLimits(const geometry_msgs::PoseStamped& global_pose,
    const geometry_msgs::PoseStamped& robot_vel,
    geometry_msgs::Twist& cmd_vel,
    Eigen::Vector3f acc_lim,
    double sim_period,
    boost::function<bool (Eigen::Vector3f pos,
                          Eigen::Vector3f vel,
                          Eigen::Vector3f vel_samples)> obstacle_check) {//stopWithAccLimits作用是计算出一个尽快能让机器人把当前速度变为0的速度，至于如何对齐目标角度，这不是它考虑的。

  //slow down with the maximum possible acceleration... we should really use the frequency that we're running at to determine what is feasible
  //but we'll use a tenth of a second to be consistent with the implementation of the local planner.
  double vx = sign(robot_vel.pose.position.x) * std::max(0.0, (fabs(robot_vel.pose.position.x) - acc_lim[0] * sim_period));
  double vy = sign(robot_vel.pose.position.y) * std::max(0.0, (fabs(robot_vel.pose.position.y) - acc_lim[1] * sim_period));
  //如何计算出一个尽快能让机器人把当前速度变为0的速度，简单操作，把存放结果的cmd_vel中的线速度、角速度都设为0，就可以了。
  // 这么操作有个问题，如果目前机器人速度很快，快到以最大加速度(acc_lim)减速，到sim_period后还是到达不了0，这时为安全就须要给指定个按最大加速度算出的结束速度。vx、vy、vel_yaw正是以着这个加速度约束去计算。
  double vel_yaw = tf2::getYaw(robot_vel.pose.orientation);
  double vth = sign(vel_yaw) * std::max(0.0, (fabs(vel_yaw) - acc_lim[2] * sim_period));

  //we do want to check whether or not the command is valid
  double yaw = tf2::getYaw(global_pose.pose.orientation);
  bool valid_cmd = obstacle_check(Eigen::Vector3f(global_pose.pose.position.x, global_pose.pose.position.y, yaw),
                                  Eigen::Vector3f(robot_vel.pose.position.x, robot_vel.pose.position.y, vel_yaw),
                                  Eigen::Vector3f(vx, vy, vth));//obstacle_check指向的操作是DWAPlanner::checkTrajectory
  //它试图以当前机器人位姿、当前速度，判断在地图上，是否能以速度(vx, vy, vth)走出一条路径。能走出一条路径(cost > 0)，或目前已基本接近goal(cost == 0)，则返回true。我没有找到这个定义
  //if we have a valid command, we'll pass it on, otherwise we'll command all zeros
  if(valid_cmd){
    ROS_DEBUG_NAMED("latched_stop_rotate", "Slowing down... using vx, vy, vth: %.2f, %.2f, %.2f", vx, vy, vth);
    cmd_vel.linear.x = vx;
    cmd_vel.linear.y = vy;
    cmd_vel.angular.z = vth;
    return true;
  }
  ROS_WARN("Stopping cmd in collision");
  cmd_vel.linear.x = 0.0;
  cmd_vel.linear.y = 0.0;
  cmd_vel.angular.z = 0.0;
  return false;
}
//rotateToGoal负责计算要让机器人旋转至目标角度，该运行什么速度。首先将x和y速度设置为0，对于theta速度，利用当前角度与目标角度的差的比例控制（系数为1）产生，然在三个方面进行微调，让它是目前机器人状态下可到达的速度。接着利用DWAPlanner::checkTrajectory进行轨迹打分，验证这速度是否可行。
bool LatchedStopRotateController::rotateToGoal(//旋转至目标角度
    const geometry_msgs::PoseStamped& global_pose,//机器人当前位姿。DWAPlannerROS运行在本地代价地图，它的全局坐标系是odom，global_pose就是base_footprint-->odom的tf。
    const geometry_msgs::PoseStamped& robot_vel,//机器人当前速度
    double goal_th,//在odom坐标系下，goal要求的角度
    geometry_msgs::Twist& cmd_vel,//为实现按goal_th，希望接下机器人运行的速度。只有返回值true时有效
    Eigen::Vector3f acc_lim,//planner_util_.getCurrentLimits().getAccLimits()。实例：(1.25, 0, 5)。
    double sim_period,//。dp_->getSimPeriod()。实例：5Hz,0.2秒。
    base_local_planner::LocalPlannerLimits& limits,//planner_util->getCurrentLimits()。用了当中两个字段，min_vel_theta(最小角速度)、max_vel_theta(最大角速度)。实例：(-1, 1)。
    boost::function<bool (Eigen::Vector3f pos,
                          Eigen::Vector3f vel,//obstacle_check,一个可执行验证障碍的操作
                          Eigen::Vector3f vel_samples)> obstacle_check) {//rotateToGoal作用是计算出一个尽快能让机器人对准目标角度的速度，这个速度线程度是0，只有角速度有值
  double yaw = tf2::getYaw(global_pose.pose.orientation);
  double vel_yaw = tf2::getYaw(robot_vel.pose.orientation);
  cmd_vel.linear.x = 0;
  cmd_vel.linear.y = 0;
  double ang_diff = angles::shortest_angular_distance(yaw, goal_th);//这个函数不错,计算任意角度差值

  double v_theta_samp = std::min(limits.max_vel_theta, std::max(limits.min_vel_theta, fabs(ang_diff)));//[min_vel_theta, max_vel_theta]是机器人到做到的角速度范围，最终角速度须要在这范围。

  //take the acceleration limits of the robot into account
  double max_acc_vel = fabs(vel_yaw) + acc_lim[2] * sim_period;
  double min_acc_vel = fabs(vel_yaw) - acc_lim[2] * sim_period;

  v_theta_samp = std::min(std::max(fabs(v_theta_samp), min_acc_vel), max_acc_vel);//机器人有个初速度vel_yaw，需是以该速度经过sim_period秒后能到达的速度范围[min_acc_vel, max_acc_vel]。

  //we also want to make sure to send a velocity that allows us to stop when we reach the goal given our acceleration limits
  // 假设机器人做初速度为0的匀加速运动，为转过ang_diff弧度，末速度不能超过max_speed_to_stop
  double max_speed_to_stop = sqrt(2 * acc_lim[2] * fabs(ang_diff));
  v_theta_samp = std::min(max_speed_to_stop, fabs(v_theta_samp));

  v_theta_samp = std::min(limits.max_vel_theta, std::max(limits.min_vel_theta, v_theta_samp));

  if (ang_diff < 0) {
    v_theta_samp = - v_theta_samp;
  }

  //we still want to lay down the footprint of the robot and check if the action is legal
  bool valid_cmd = obstacle_check(Eigen::Vector3f(global_pose.pose.position.x, global_pose.pose.position.y, yaw),
      Eigen::Vector3f(robot_vel.pose.position.x, robot_vel.pose.position.y, vel_yaw),
      Eigen::Vector3f( 0.0, 0.0, v_theta_samp));//检查轨迹的合法性

  if (valid_cmd) {
    ROS_DEBUG_NAMED("dwa_local_planner", "Moving to desired goal orientation, th cmd: %.2f, valid_cmd: %d", v_theta_samp, valid_cmd);
    cmd_vel.angular.z = v_theta_samp;//向上返回的cmd_vel，其线速度一定是0，角速度在失败时是0，成功时v_theta_samp。
    return true;
  }
  ROS_WARN("Rotation cmd in collision");
  cmd_vel.angular.z = 0.0;
  return false;

}

bool LatchedStopRotateController::computeVelocityCommandsStopRotate(geometry_msgs::Twist& cmd_vel,//该函数计算对应的减速停止或者旋转至目标角度的速度指令，否则才会使用DWAPlanner计算最优轨迹对应的速度命令
    Eigen::Vector3f acc_lim,
    double sim_period,
    LocalPlannerUtil* planner_util,
    OdometryHelperRos& odom_helper_,
    const geometry_msgs::PoseStamped& global_pose,
    boost::function<bool (Eigen::Vector3f pos,
                          Eigen::Vector3f vel,
                          Eigen::Vector3f vel_samples)> obstacle_check) {
  //we assume the global goal is the last point in the global plan
  geometry_msgs::PoseStamped goal_pose;
  if ( ! planner_util->getGoal(goal_pose)) {
    ROS_ERROR("Could not get goal pose");
    return false;
  }

  base_local_planner::LocalPlannerLimits limits = planner_util->getCurrentLimits();

  //if the user wants to latch goal tolerance, if we ever reach the goal location, we'll
  //just rotate in place
  if (latch_xy_goal_tolerance_ && ! xy_tolerance_latch_ ) {
    ROS_INFO("Goal position reached, stopping and turning in place");
    xy_tolerance_latch_ = true;
  }
  //check to see if the goal orientation has been reached
  double goal_th = tf2::getYaw(goal_pose.pose.orientation);
  double angle = base_local_planner::getGoalOrientationAngleDifference(global_pose, goal_th);
  if (fabs(angle) <= limits.yaw_goal_tolerance) {//目前机器人角度已满足要求，认为导航已成功
    //set the velocity command to zero
    cmd_vel.linear.x = 0.0;
    cmd_vel.linear.y = 0.0;
    cmd_vel.angular.z = 0.0;
    rotating_to_goal_ = false;
  } else {
    ROS_DEBUG("Angle: %f Tolerance: %f", angle, limits.yaw_goal_tolerance);
    geometry_msgs::PoseStamped robot_vel;
    odom_helper_.getRobotVel(robot_vel);
    nav_msgs::Odometry base_odom;
    odom_helper_.getOdom(base_odom);

    //if we're not stopped yet... we want to stop... taking into account the acceleration limits of the robot
    if ( ! rotating_to_goal_ && !base_local_planner::stopped(base_odom, limits.theta_stopped_vel, limits.trans_stopped_vel)) {//目前机器人角度不满足要求，而且机器人速度很快，需要先把速度降下来，再考虑如何对正角度
      if ( ! stopWithAccLimits(//stopWithAccLimits作用是计算出一个尽快能让机器人把当前速度变为0的速度
          global_pose,
          robot_vel,
          cmd_vel,
          acc_lim,
          sim_period,
          obstacle_check)) {
        ROS_INFO("Error when stopping.");
        return false;
      }
      ROS_DEBUG("Stopping...");
    }
    //if we're stopped... then we want to rotate to goal
    else {
      //set this so that we know its OK to be moving
      rotating_to_goal_ = true; //把rotating_to_goal_设为true，表示机器人进入到达位置后对齐角度过程，在这个过程会不执行操作像上面的降速。
      if ( ! rotateToGoal(
          global_pose,
          robot_vel,
          goal_th,
          cmd_vel,
          acc_lim,
          sim_period,
          limits,
          obstacle_check)) {
        ROS_INFO("Error when rotating.");
        return false;
      }
      ROS_DEBUG("Rotating...");
    }
  }

  return true;

}


} /* namespace base_local_planner */
