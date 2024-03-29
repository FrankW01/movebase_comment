/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, 2013, Willow Garage, Inc.
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
 *   * Neither the name of Willow Garage, Inc. nor the names of its
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
 *         David V. Lu!!
 *********************************************************************/
#ifndef COSTMAP_2D_COSTMAP_2D_ROS_H_
#define COSTMAP_2D_COSTMAP_2D_ROS_H_

#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/costmap_2d_publisher.h>
#include <costmap_2d/Costmap2DConfig.h>
#include <costmap_2d/footprint.h>
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/PolygonStamped.h>
#include <dynamic_reconfigure/server.h>
#include <pluginlib/class_loader.h>

class SuperValue : public XmlRpc::XmlRpcValue
{
public:
  void setStruct(XmlRpc::XmlRpcValue::ValueStruct* a)
  {
    _type = TypeStruct;
    _value.asStruct = new XmlRpc::XmlRpcValue::ValueStruct(*a);
  }
  void setArray(XmlRpc::XmlRpcValue::ValueArray* a)
  {
    _type = TypeArray;
    _value.asArray = new std::vector<XmlRpc::XmlRpcValue>(*a);
  }
};

namespace costmap_2d
{

/** @brief A ROS wrapper for a 2D Costmap. Handles subscribing to
 * topics that provide observations about obstacles in either the form
 * of PointCloud or LaserScan messages. */  //2D Costmap 的 ROS 包装器。 处理订阅主题，这些主题提供关于 PointCloud 或 LaserScan 消息中障碍物的观察。
class Costmap2DROS//对外提供简洁的接口，在所有的包里，带ROS后缀的都是封装功能用的，可以理解加精，给每个层进行封装接口吗？ --> costmap2dROS封装好了全部的功能，从costmap2dROS看是最有效率的
{
public:
  /**
   * @brief  Constructor for the wrapper
   * @param name The name for this costmap
   * @param tf A reference to a TransformListener
   */
  Costmap2DROS(std::string name, tf::TransformListener& tf);
  ~Costmap2DROS();

  /**
   * @brief  Subscribes to sensor topics if necessary and starts costmap
   * updates, can be called to restart the costmap after calls to either
   * stop() or pause()
   */
  void start();

  /**
   * @brief  Stops costmap updates and unsubscribes from sensor topics
   */
  void stop();

  /**
   * @brief  Stops the costmap from updating, but sensor data still comes in over the wire
   */
  void pause();

  /**
   * @brief  Resumes costmap updates  恢复代价地图更新
   */
  void resume();

  void updateMap(); //updatemap也是调用layeredCostmap更新地图

  /**
   * @brief Reset each individual layer
   */
  void resetLayers();//resetlayers还是在调用layeredCostmap

  /** @brief Same as getLayeredCostmap()->isCurrent(). */
  bool isCurrent()
    {
      return layered_costmap_->isCurrent();
    }

  /**
   * @brief Get the pose of the robot in the global frame of the costmap
   * @param global_pose Will be set to the pose of the robot in the global frame of the costmap
   * @return True if the pose was set successfully, false otherwise
   */
  bool getRobotPose(tf::Stamped<tf::Pose>& global_pose) const;//含有get的函数不用看，因为就是读取下变量的

  /** @brief Return a pointer to the "master" costmap which receives updates from all the layers.  返回指向从所有层接收更新的“主”成本图的指针
   *
   * Same as calling getLayeredCostmap()->getCostmap(). */
  Costmap2D* getCostmap()
    {
      return layered_costmap_->getCostmap();
    }

  /**
   * @brief  Returns the global frame of the costmap
   * @return The global frame of the costmap
   */
  std::string getGlobalFrameID()
    {
      return global_frame_;
    }

  /**
   * @brief  Returns the local frame of the costmap
   * @return The local frame of the costmap
   */
  std::string getBaseFrameID()
    {
      return robot_base_frame_;
    }
  LayeredCostmap* getLayeredCostmap()
    {
      return layered_costmap_;
    }

  /** @brief Returns the current padded footprint as a geometry_msgs::Polygon. */  //将当前填充的足迹作为 geometry_msgs::Polygon 返回。
  geometry_msgs::Polygon getRobotFootprintPolygon()//含有footprint不用看，这是处理底座的
  {
    return costmap_2d::toPolygon(padded_footprint_);
  }

  /** @brief Return the current footprint of the robot as a vector of points.
   *
   * This version of the footprint is padded by the footprint_padding_
   * distance, set in the rosparam "footprint_padding".
   *
   * The footprint initially comes from the rosparam "footprint" but
   * can be overwritten by dynamic reconfigure or by messages received
   * on the "footprint" topic. */
  std::vector<geometry_msgs::Point> getRobotFootprint()
  {
    return padded_footprint_;
  }

  /** @brief Return the current unpadded footprint of the robot as a vector of points. 什么叫当前未填充的足迹？
   *
   * This is the raw version of the footprint without padding.
   *
   * The footprint initially comes from the rosparam "footprint" but
   * can be overwritten by dynamic reconfigure or by messages received
   * on the "footprint" topic. */
  std::vector<geometry_msgs::Point> getUnpaddedRobotFootprint()
  {
    return unpadded_footprint_;//什么叫未折叠足迹
  }

  /**
   * @brief  Build the oriented footprint of the robot at the robot's current pose  在机器人的当前姿势下构建机器人的定向足迹
   * @param  oriented_footprint Will be filled with the points in the oriented footprint of the robot
   */
  void getOrientedFootprint(std::vector<geometry_msgs::Point>& oriented_footprint) const;//将padded_footprint_ 按照旋转角 旋转出oriented_footprint

  /** @brief Set the footprint of the robot to be the given set of
   * points, padded by footprint_padding.   将机器人的足迹设置为给定的点集，由 footprint_padding 填充。
   *
   * Should be a convex polygon, though this is not enforced.
   *
   * First expands the given polygon by footprint_padding_ and then
   * sets padded_footprint_ and calls
   * layered_costmap_->setFootprint().  Also saves the unpadded
   * footprint, which is available from
   * getUnpaddedRobotFootprint(). */
  void setUnpaddedRobotFootprint(const std::vector<geometry_msgs::Point>& points);

  /** @brief Set the footprint of the robot to be the given polygon,
   * padded by footprint_padding.
   *
   * Should be a convex polygon, though this is not enforced.
   *
   * First expands the given polygon by footprint_padding_ and then
   * sets padded_footprint_ and calls
   * layered_costmap_->setFootprint().  Also saves the unpadded
   * footprint, which is available from
   * getUnpaddedRobotFootprint(). */
  void setUnpaddedRobotFootprintPolygon(const geometry_msgs::Polygon& footprint);

protected:
  LayeredCostmap* layered_costmap_;//聚合层
  std::string name_;
  tf::TransformListener& tf_;  ///< @brief Used for transforming point clouds
  std::string global_frame_;  ///< @brief The global frame for the costmap
  std::string robot_base_frame_;  ///< @brief The frame_id of the robot base
  double transform_tolerance_;  ///< timeout before transform errors

private: // private不用看，因为肯定是被调用的
  /** @brief Set the footprint from the new_config object.
   *
   * If the values of footprint and robot_radius are the same in
   * new_config and old_config, nothing is changed. */
  void readFootprintFromConfig(const costmap_2d::Costmap2DConfig &new_config,
                               const costmap_2d::Costmap2DConfig &old_config);//修改config

  void resetOldParameters(ros::NodeHandle& nh);
  void reconfigureCB(costmap_2d::Costmap2DConfig &config, uint32_t level);//这个参数是从那里读入的 common costmap吗？从move_base中读入的吗？
  void movementCB(const ros::TimerEvent &event);
  void mapUpdateLoop(double frequency);
  bool map_update_thread_shutdown_;
  bool stop_updates_, initialized_, stopped_, robot_stopped_;
  boost::thread* map_update_thread_;  ///< @brief A thread for updating the map
  ros::Timer timer_;
  ros::Time last_publish_;
  ros::Duration publish_cycle;
  pluginlib::ClassLoader<Layer> plugin_loader_;
  tf::Stamped<tf::Pose> old_pose_;
  Costmap2DPublisher* publisher_;
  dynamic_reconfigure::Server<costmap_2d::Costmap2DConfig> *dsrv_;

  boost::recursive_mutex configuration_mutex_;

  ros::Subscriber footprint_sub_;
  ros::Publisher footprint_pub_;
  bool got_footprint_;
  std::vector<geometry_msgs::Point> unpadded_footprint_;
  std::vector<geometry_msgs::Point> padded_footprint_;
  float footprint_padding_;
  costmap_2d::Costmap2DConfig old_config_;
};
// class Costmap2DROS
}  // namespace costmap_2d

#endif  // COSTMAP_2D_COSTMAP_2D_ROS_H
