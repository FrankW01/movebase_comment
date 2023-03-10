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
#include <costmap_2d/obstacle_layer.h>
#include <costmap_2d/costmap_math.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(costmap_2d::ObstacleLayer, costmap_2d::Layer)//障碍物层插件

using costmap_2d::NO_INFORMATION;
using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::FREE_SPACE;

using costmap_2d::ObservationBuffer;
using costmap_2d::Observation;

namespace costmap_2d
{

void ObstacleLayer::onInitialize()//障碍物层初始化，首先从参数服务器加载参数，确定rolling_window_、track_unknown_space的值，并调用matchSize函数，根据主地图的参数来设置障碍层地图
{
  ros::NodeHandle nh("~/" + name_), g_nh;//ros句柄
  rolling_window_ = layered_costmap_->isRolling();

  bool track_unknown_space;
  nh.param("track_unknown_space", track_unknown_space, layered_costmap_->isTrackingUnknown());//这个参数是如果设置为true，说明未知区域为自由空间
  if (track_unknown_space)
    default_value_ = NO_INFORMATION;
  else
    default_value_ = FREE_SPACE;

  ObstacleLayer::matchSize();
  current_ = true;

  global_frame_ = layered_costmap_->getGlobalFrameID();
  double transform_tolerance;
  nh.param("transform_tolerance", transform_tolerance, 0.2);//默认tf变换阈值为0.2

  std::string topics_string;
  // get the topics that we'll subscribe to from the parameter server   从参数服务器加载订阅的话题名称，即障碍物信息的来源
  nh.param("observation_sources", topics_string, std::string(""));//获得传感器话题
  ROS_INFO("    Subscribed to Topics: %s", topics_string.c_str());

  // get our tf prefix
  ros::NodeHandle prefix_nh;
  const std::string tf_prefix = tf::getPrefixParam(prefix_nh); //Get the tf_prefix from the parameter server. 

  // now we need to split the topics based on whitespace which we can use a stringstream for
  std::stringstream ss(topics_string);//拆分传感器名称，可以增加多个传感器

  std::string source;
  while (ss >> source)//进入循环，将topics_string记录的观测源逐个输出到source字符串，并找到对应每个观测源的参数
  {
    ros::NodeHandle source_node(nh, source);

    // get the parameters for the specific topic
    double observation_keep_time, expected_update_rate, min_obstacle_height, max_obstacle_height;
    std::string topic, sensor_frame, data_type;
    bool inf_is_valid, clearing, marking;

    source_node.param("topic", topic, source);
    source_node.param("sensor_frame", sensor_frame, std::string(""));
    source_node.param("observation_persistence", observation_keep_time, 0.0);//默认只保留最新的障碍物
    source_node.param("expected_update_rate", expected_update_rate, 0.0);//代价地图更新频率默认0
    source_node.param("data_type", data_type, std::string("PointCloud"));
    source_node.param("min_obstacle_height", min_obstacle_height, 0.0);
    source_node.param("max_obstacle_height", max_obstacle_height, 2.0);
    source_node.param("inf_is_valid", inf_is_valid, false);
    source_node.param("clearing", clearing, false);
    source_node.param("marking", marking, true);//marking默认为true，但是这个是什么意思呢

    if (!sensor_frame.empty())
    {
      sensor_frame = tf::resolve(tf_prefix, sensor_frame);//解析tf名称
    }

    if (!(data_type == "PointCloud2" || data_type == "PointCloud" || data_type == "LaserScan"))//只能解析这些数据
    {
      ROS_FATAL("Only topics that use point clouds or laser scans are currently supported");
      throw std::runtime_error("Only topics that use point clouds or laser scans are currently supported");
    }

    std::string raytrace_range_param_name, obstacle_range_param_name;

    // get the obstacle range for the sensor
    double obstacle_range = 2.5;
    if (source_node.searchParam("obstacle_range", obstacle_range_param_name))
    {
      source_node.getParam(obstacle_range_param_name, obstacle_range);
    }

    // get the raytrace range for the sensor
    double raytrace_range = 3.0;
    if (source_node.searchParam("raytrace_range", raytrace_range_param_name))
    {
      source_node.getParam(raytrace_range_param_name, raytrace_range);
    }

    ROS_DEBUG("Creating an observation buffer for source %s, topic %s, frame %s", source.c_str(), topic.c_str(),
              sensor_frame.c_str());

    // create an observation buffer    observation_buffers里面的buff的size等于传感器的个数，一般为1
    observation_buffers_.push_back(
        boost::shared_ptr < ObservationBuffer
            > (new ObservationBuffer(topic, observation_keep_time, expected_update_rate, min_obstacle_height,
                                     max_obstacle_height, obstacle_range, raytrace_range, *tf_, global_frame_,
                                     sensor_frame, transform_tolerance)));//为每个观测源创建一个buffer，并将其指针存放进observation_buffers_中进行管理。并根据标志位确定是否将其添加到marking_buffers与clearing_buffers中。当收到一个点云后，它有两种做作，一是用它标记障碍栅格，二是用它去清除原代码地图。marking、clearing作用是指示从此个sensor收到的点云要用在上面这两个哪些场景。默认这两个值都是true。

    // check if we'll add this buffer to our marking observation buffers  检查我们是否将这个缓冲区添加到我们的标记观察缓冲区
    if (marking)//为marking初始化 实际上 marking_buffers 就是 observation_buffers
      marking_buffers_.push_back(observation_buffers_.back());

    // check if we'll also add this buffer to our clearing observation buffers
    if (clearing) // 同上
      clearing_buffers_.push_back(observation_buffers_.back());

    ROS_DEBUG(
        "Created an observation buffer for source %s, topic %s, global frame: %s, "
        "expected update rate: %.2f, observation persistence: %.2f",
        source.c_str(), topic.c_str(), global_frame_.c_str(), expected_update_rate, observation_keep_time);

    // create a callback for the topic
    if (data_type == "LaserScan")//默认是进入这个回调
    {
      boost::shared_ptr < message_filters::Subscriber<sensor_msgs::LaserScan>
          > sub(new message_filters::Subscriber<sensor_msgs::LaserScan>(g_nh, topic, 50));//sub是一个订阅者的句柄

      boost::shared_ptr < tf::MessageFilter<sensor_msgs::LaserScan>
          > filter(new tf::MessageFilter<sensor_msgs::LaserScan>(*sub, *tf_, global_frame_, 50));//tf::MessageFilter的用法

      if (inf_is_valid)//如果inf 有效
      {
        filter->registerCallback(
            boost::bind(&ObstacleLayer::laserScanValidInfCallback, this, _1, observation_buffers_.back()));//inf有效就执行这个回调函数
      }
      else
      {
        filter->registerCallback(
            boost::bind(&ObstacleLayer::laserScanCallback, this, _1, observation_buffers_.back()));//这个buffer的back
      }

      observation_subscribers_.push_back(sub);
      observation_notifiers_.push_back(filter);

      observation_notifiers_.back()->setTolerance(ros::Duration(0.05));
    }
    else if (data_type == "PointCloud")
    {
      boost::shared_ptr < message_filters::Subscriber<sensor_msgs::PointCloud>
          > sub(new message_filters::Subscriber<sensor_msgs::PointCloud>(g_nh, topic, 50));

      if (inf_is_valid)
      {
       ROS_WARN("obstacle_layer: inf_is_valid option is not applicable to PointCloud observations.");
      }

      boost::shared_ptr < tf::MessageFilter<sensor_msgs::PointCloud>
          > filter(new tf::MessageFilter<sensor_msgs::PointCloud>(*sub, *tf_, global_frame_, 50));
      filter->registerCallback(
          boost::bind(&ObstacleLayer::pointCloudCallback, this, _1, observation_buffers_.back()));

      observation_subscribers_.push_back(sub);
      observation_notifiers_.push_back(filter);
    }
    else
    {
      boost::shared_ptr < message_filters::Subscriber<sensor_msgs::PointCloud2>
          > sub(new message_filters::Subscriber<sensor_msgs::PointCloud2>(g_nh, topic, 50));

      if (inf_is_valid)
      {
       ROS_WARN("obstacle_layer: inf_is_valid option is not applicable to PointCloud observations.");
      }

      boost::shared_ptr < tf::MessageFilter<sensor_msgs::PointCloud2>
          > filter(new tf::MessageFilter<sensor_msgs::PointCloud2>(*sub, *tf_, global_frame_, 50));
      filter->registerCallback(
          boost::bind(&ObstacleLayer::pointCloud2Callback, this, _1, observation_buffers_.back()));

      observation_subscribers_.push_back(sub);
      observation_notifiers_.push_back(filter);
    }

    if (sensor_frame != "")
    {
      std::vector < std::string > target_frames;
      target_frames.push_back(global_frame_);
      target_frames.push_back(sensor_frame);
      observation_notifiers_.back()->setTargetFrames(target_frames);
    }
  }

  dsrv_ = NULL;
  setupDynamicReconfigure(nh);
}

void ObstacleLayer::setupDynamicReconfigure(ros::NodeHandle& nh)//设置动态参数配置
{
  dsrv_ = new dynamic_reconfigure::Server<costmap_2d::ObstaclePluginConfig>(nh);
  dynamic_reconfigure::Server<costmap_2d::ObstaclePluginConfig>::CallbackType cb = boost::bind(
      &ObstacleLayer::reconfigureCB, this, _1, _2);
  dsrv_->setCallback(cb);
}

ObstacleLayer::~ObstacleLayer()
{
    if (dsrv_)
        delete dsrv_;
}
void ObstacleLayer::reconfigureCB(costmap_2d::ObstaclePluginConfig &config, uint32_t level)
{
  enabled_ = config.enabled;
  footprint_clearing_enabled_ = config.footprint_clearing_enabled;
  max_obstacle_height_ = config.max_obstacle_height;
  combination_method_ = config.combination_method;
}

void ObstacleLayer::laserScanCallback(const sensor_msgs::LaserScanConstPtr& message,
                                      const boost::shared_ptr<ObservationBuffer>& buffer)
{
  // project the laser into a point cloud
  sensor_msgs::PointCloud2 cloud;
  cloud.header = message->header;

  // project the scan into a point cloud
  try
  {
    projector_.transformLaserScanToPointCloud(message->header.frame_id, *message, cloud, *tf_);
  }
  catch (tf::TransformException &ex)
  {
    ROS_WARN("High fidelity enabled, but TF returned a transform exception to frame %s: %s", global_frame_.c_str(),
             ex.what());
    projector_.projectLaser(*message, cloud);
  }

  // buffer the point cloud
  buffer->lock();
  buffer->bufferCloud(cloud);
  buffer->unlock();
}

void ObstacleLayer::laserScanValidInfCallback(const sensor_msgs::LaserScanConstPtr& raw_message,
                                              const boost::shared_ptr<ObservationBuffer>& buffer)//laserscan 传感器数据回调函数
{
  // Filter positive infinities ("Inf"s) to max_range. 将正无穷大（“Inf”）过滤到 max_range。
  float epsilon = 0.0001;  // a tenth of a millimeter
  sensor_msgs::LaserScan message = *raw_message;
  for (size_t i = 0; i < message.ranges.size(); i++)
  {
    float range = message.ranges[ i ];
    if (!std::isfinite(range) && range > 0)//如果 arg 具有有限值，则为真，否则为假 inf还有正负吗
    {
      message.ranges[ i ] = message.range_max - epsilon;
    }
  }

  // project the laser into a point cloud
  sensor_msgs::PointCloud2 cloud;
  cloud.header = message.header;

  // project the scan into a point cloud
  try
  {
    projector_.transformLaserScanToPointCloud(message.header.frame_id, message, cloud, *tf_);//将laserscan
  }
  catch (tf::TransformException &ex)
  {
    ROS_WARN("High fidelity enabled, but TF returned a transform exception to frame %s: %s",
             global_frame_.c_str(), ex.what());
    projector_.projectLaser(message, cloud);
  }

  // buffer the point cloud
  buffer->lock();//这个buffer是传入参数
  buffer->bufferCloud(cloud);
  buffer->unlock();
}

void ObstacleLayer::pointCloudCallback(const sensor_msgs::PointCloudConstPtr& message,
                                               const boost::shared_ptr<ObservationBuffer>& buffer)//pointcloud的回调处理
{
  sensor_msgs::PointCloud2 cloud2;

  if (!sensor_msgs::convertPointCloudToPointCloud2(*message, cloud2))
  {
    ROS_ERROR("Failed to convert a PointCloud to a PointCloud2, dropping message");
    return;
  }

  // buffer the point cloud
  buffer->lock();
  buffer->bufferCloud(cloud2);
  buffer->unlock();
}

void ObstacleLayer::pointCloud2Callback(const sensor_msgs::PointCloud2ConstPtr& message,
                                                const boost::shared_ptr<ObservationBuffer>& buffer)//最基本的传感器回调处理形式
{
  // buffer the point cloud
  buffer->lock();
  buffer->bufferCloud(*message);//压入点云
  buffer->unlock();
}

void ObstacleLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x,
                                          double* min_y, double* max_x, double* max_y)//这个函数主要完成：clearing、marking以及确定bound ,gdb调试的时候没有进入该函数，为什么？？
{
  if (rolling_window_)//如果使用滚动窗口，就采用更新原点的方式
    updateOrigin(robot_x - getSizeInMetersX() / 2, robot_y - getSizeInMetersY() / 2);//滚动窗口形式的坐标原点在机器人坐标的左下角
  if (!enabled_)//如果不滑动窗口，则不进行更新bounds
    return;
  useExtraBounds(min_x, min_y, max_x, max_y);
  //在接收到传感器数据后，buffer将被更新，同样，marking_buffers_和clearing_buffers_也更新（即buffer中的内容），将二者分别提取到observations和clearing_observations中存储。
  bool current = true;
  std::vector<Observation> observations, clearing_observations;

  // get the marking observations
  current = current && getMarkingObservations(observations);

  // get the clearing observations
  current = current && getClearingObservations(clearing_observations);

  // update the global current status
  current_ = current;

  // raytrace freespace  // 首先清理出传感器与被测物之间的距离，标记为FREE_SPACE
  for (unsigned int i = 0; i < clearing_observations.size(); ++i)
  {
    raytraceFreespace(clearing_observations[i], min_x, min_y, max_x, max_y);//接下来是第一步，对clearing_observations中的点云点执行clearing操作，即将其与传感器的连线上的点标记为FREE_SPACE。这里调用的raytraceFreespace函数内容后述。
  }

  // place the new obstacles into a priority queue... each with a priority of zero to begin with
  for (std::vector<Observation>::const_iterator it = observations.begin(); it != observations.end(); ++it)  //在标记时通过二重循环，外层迭代各观测轮次，内层迭代一次观测得到的点云点
  {
    const Observation& obs = *it;

    const pcl::PointCloud<pcl::PointXYZ>& cloud = *(obs.cloud_);

    double sq_obstacle_range = obs.obstacle_range_ * obs.obstacle_range_;//obstacle_range的平方
    //然后开始进入mark操作，对于每个测量到的点，标记为LETHAL_OBSTACLE    //即迭代点云中的点，本身太高太远的障碍和跟其他障碍点离得太远的点都不考虑
    for (unsigned int i = 0; i < cloud.points.size(); ++i)
    {
      double px = cloud.points[i].x, py = cloud.points[i].y, pz = cloud.points[i].z;

      // if the obstacle is too high or too far away from the robot we won't add it   //如果障碍太高，则不加入mark
      if (pz > max_obstacle_height_)
      {
        ROS_DEBUG("The point is too high");
        continue;
      }

      // compute the squared distance from the hitpoint to the pointcloud's origin  // 计算点与点之间的距离平方，如果距离太远，不考虑
      double sq_dist = (px - obs.origin_.x) * (px - obs.origin_.x) + (py - obs.origin_.y) * (py - obs.origin_.y)
          + (pz - obs.origin_.z) * (pz - obs.origin_.z);//因为cloud已经转换为世界坐标系，所以这里需要减去原点坐标

      // if the point is far enough away... we won't consider it
      if (sq_dist >= sq_obstacle_range)
      {
        ROS_DEBUG("The point is too far away");
        continue;
      }

      // now we need to compute the map coordinates for the observation
      unsigned int mx, my;    //获得障碍点在地图上的坐标
      if (!worldToMap(px, py, mx, my))
      {
        ROS_DEBUG("Computing map coords failed");
        continue;
      }
      //将该mx，my标记为设置该栅格cost为致命障碍
      unsigned int index = getIndex(mx, my);
      costmap_[index] = LETHAL_OBSTACLE;
      touch(px, py, min_x, min_y, max_x, max_y); //这里意味着如果obstacle_range 大于 设定的mapupdate，地图的尺寸会改变吗？？？？  min_x 这里我认为是地图的边界
    }
  }

  updateFootprint(robot_x, robot_y, robot_yaw, min_x, min_y, max_x, max_y);//它的作用是基于机器人当前位置确定该位置下的足迹，并在内部调用touch函数保证足迹包含在bound范围内
}

void ObstacleLayer::updateFootprint(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y,
                                    double* max_x, double* max_y)//这个min max都是什么？
{
    if (!footprint_clearing_enabled_) return;  //启用足迹清除，这个什么？ 如果不启动足迹清除，就不清除之前老的footprint？
    transformFootprint(robot_x, robot_y, robot_yaw, getFootprint(), transformed_footprint_);

    for (unsigned int i = 0; i < transformed_footprint_.size(); i++)
    {
      touch(transformed_footprint_[i].x, transformed_footprint_[i].y, min_x, min_y, max_x, max_y);
    }
}

void ObstacleLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j)//这个才会真正写入主代价层中
{
  if (!enabled_)
    return;

  if (footprint_clearing_enabled_)//是否将足迹内的点代价清除
  {
    setConvexPolygonCost(transformed_footprint_, costmap_2d::FREE_SPACE);//这个函数就是将机器人足迹范围内设置为FREE_SPACE，并且在bound范围内将本层障碍地图的内容合并到主地图上。
  }

  switch (combination_method_)//代价合并模式
  {
    case 0:  // Overwrite 
      updateWithOverwrite(master_grid, min_i, min_j, max_i, max_j);
      break;
    case 1:  // Maximum  这个是默认形式 即取得层中最大的cost。
      updateWithMax(master_grid, min_i, min_j, max_i, max_j);
      break;
    default:  // Nothing
      break;
  }
}

void ObstacleLayer::addStaticObservation(costmap_2d::Observation& obs, bool marking, bool clearing)//测试目的
{
  if (marking)
    static_marking_observations_.push_back(obs);
  if (clearing)
    static_clearing_observations_.push_back(obs);
}

void ObstacleLayer::clearStaticObservations(bool marking, bool clearing)//测试目的，用来测试的插入数据的函数
{
  if (marking)
    static_marking_observations_.clear();
  if (clearing)
    static_clearing_observations_.clear();
}

bool ObstacleLayer::getMarkingObservations(std::vector<Observation>& marking_observations) const//将marking_buffers_ 压入到 marking_observations观察队列中去
{
  bool current = true;
  // get the marking observations
  for (unsigned int i = 0; i < marking_buffers_.size(); ++i)//单传感器来说，这个size大小为1
  {
    marking_buffers_[i]->lock();
    marking_buffers_[i]->getObservations(marking_observations);
    current = marking_buffers_[i]->isCurrent() && current;
    marking_buffers_[i]->unlock();
  }
  marking_observations.insert(marking_observations.end(),
                              static_marking_observations_.begin(), static_marking_observations_.end());//为什么插入这个static marking obserations队列，这个队列里面是什么？
  return current;
}

bool ObstacleLayer::getClearingObservations(std::vector<Observation>& clearing_observations) const
{
  bool current = true;
  // get the clearing observations
  for (unsigned int i = 0; i < clearing_buffers_.size(); ++i)
  {
    clearing_buffers_[i]->lock();
    clearing_buffers_[i]->getObservations(clearing_observations);//把clearing_buffers_的i压入clearing_observations中去
    current = clearing_buffers_[i]->isCurrent() && current;
    clearing_buffers_[i]->unlock();
  }
  clearing_observations.insert(clearing_observations.end(),
                              static_clearing_observations_.begin(), static_clearing_observations_.end());//再在clearing_observations中压入static_clearing_observations
  return current;
}

void ObstacleLayer::raytraceFreespace(const Observation& clearing_observation, double* min_x, double* min_y,
                                              double* max_x, double* max_y)
{//clearing_observation 的origin是传感器的原点坐标
  double ox = clearing_observation.origin_.x;//障碍物点云的那一帧的原点相对于map的坐标，observation中存放的点是不是保存以map的坐标下的，但是每一帧的原点的坐标是被保存的
  double oy = clearing_observation.origin_.y;//这是相对于全局的坐标
  pcl::PointCloud < pcl::PointXYZ > cloud = *(clearing_observation.cloud_);

  // get the map coordinates of the origin of the sensor 获取传感器原点的地图坐标
  unsigned int x0, y0;//单位是cell，index
  if (!worldToMap(ox, oy, x0, y0))//计算这帧点云的原点在costmap的index
  {
    ROS_WARN_THROTTLE(
        1.0, "The origin for the sensor at (%.2f, %.2f) is out of map bounds. So, the costmap cannot raytrace for it.",
        ox, oy);
    return;
  }

  // we can pre-compute the enpoints of the map outside of the inner loop... we'll need these later   我们可以在内部循环之外预先计算地图的端点……我们稍后会需要这些
  double origin_x = origin_x_,                origin_y = origin_y_;//该地图相当于全局的坐标？，这个相当于代价地图的原点在全局的坐标？
  double map_end_x = origin_x + size_x_ * resolution_;//该地图的最大x边界，单位m
  double map_end_y = origin_y + size_y_ * resolution_;//我感觉这个是这在global_frame下的地图坐标


  touch(ox, oy, min_x, min_y, max_x, max_y);//对于ox oy来更新最大与最小值，保证传感器原点在bound范围内

  // for each point in the cloud, we want to trace a line from the origin and clear obstacles along it  //对于点云中的每个点，我们要追踪原点和clear obstacles之间的一条线
  for (unsigned int i = 0; i < cloud.points.size(); ++i)//这里每个点都是以global_frame下的点
  {
    double wx = cloud.points[i].x;//在global坐标系下的坐标
    double wy = cloud.points[i].y;

    // now we also need to make sure that the enpoint we're raytracing
    // to isn't off the costmap and scale if necessary  // 现在我们还需要确保我们进行光线追踪的端点不在代价地图之外，并在必要时进行缩放
    double a = wx - ox;//这个是单位m，a、b是该点跟传感器原点的距离
    double b = wy - oy;

    // the minimum value to raytrace from is the origin  光线追踪的最小值是原点
    if (wx < origin_x)//这个oringin x是原点吗，感觉是代价地图的原点在global坐标系的坐标,如果当前点x方向比地图原点还小
    {
      double t = (origin_x - ox) / a; //这个应该是占比      //t（比例）=（地图原点-传感器原点）/（点云中的该点-传感器原点）  
      wx = origin_x;//当前点x = 地图原点x
      wy = oy + b * t;//当前点y = 实际上还是把点云点和传感器连线之间清空，只是通过相似三角形丢弃了超出地图原点范围外的部分，下面三个判断结构同理
    }
    if (wy < origin_y)
    {
      double t = (origin_y - oy) / b;
      wx = ox + a * t;
      wy = origin_y;
    }

    // the maximum value to raytrace to is the end of the map
    if (wx > map_end_x)//这个map_end_x是代价地图的边界吗
    {
      double t = (map_end_x - ox) / a;
      wx = map_end_x - .001;
      wy = oy + b * t;
    }
    if (wy > map_end_y)
    {
      double t = (map_end_y - oy) / b;
      wx = ox + a * t;
      wy = map_end_y - .001;
    }

    // now that the vector is scaled correctly... we'll get the map coordinates of its endpoint    现在矢量已正确缩放......我们将获得其端点的地图坐标
    unsigned int x1, y1;//wx,wy的cell map坐标

    // check for legality just in case   检查合法性以防万一  // 转换到区域相关的map坐标系。如果worldToMap返回false，表示该坐标不在区域内。
    if (!worldToMap(wx, wy, x1, y1))
      continue;

    unsigned int cell_raytrace_range = cellDistance(clearing_observation.raytrace_range_);
    MarkCell marker(costmap_, FREE_SPACE);//掩膜吗？
    // and finally... we can execute our trace to clear obstacles along that line   最后...我们可以执行我们的跟踪以清除沿该线的障碍  //调用raytraceLine函数清理传感器原点和障碍物点之间的cell
    raytraceLine(marker, x0, y0, x1, y1, cell_raytrace_range);//x0,y0为全局map的cell坐标，x1，y1是要清除范围内的cell坐标
    // updateRaytraceBounds函数用来根据测量的距离，更新扩张
    updateRaytraceBounds(ox, oy, wx, wy, clearing_observation.raytrace_range_, min_x, min_y, max_x, max_y);
  }
}

void ObstacleLayer::activate()
{
  // if we're stopped we need to re-subscribe to topics
  for (unsigned int i = 0; i < observation_subscribers_.size(); ++i)
  {
    if (observation_subscribers_[i] != NULL)
      observation_subscribers_[i]->subscribe();
  }

  for (unsigned int i = 0; i < observation_buffers_.size(); ++i)
  {
    if (observation_buffers_[i])
      observation_buffers_[i]->resetLastUpdated();
  }
}
void ObstacleLayer::deactivate()
{
  for (unsigned int i = 0; i < observation_subscribers_.size(); ++i)
  {
    if (observation_subscribers_[i] != NULL)
      observation_subscribers_[i]->unsubscribe();
  }
}

void ObstacleLayer::updateRaytraceBounds(double ox, double oy, double wx, double wy, double range,
                                         double* min_x, double* min_y, double* max_x, double* max_y)//updateRaytraceBounds函数的作用是保证连线上的一点（距离传感器特定范围内）被包含进bound。
{
  double dx = wx-ox, dy = wy-oy;
  double full_distance = hypot(dx, dy);//单位m
  double scale = std::min(1.0, range / full_distance);//这个scale的作用：  当rang  > distance ,即清除距离超过到地图边界的大小，ex = 地图边界wx ，就保证到地图边界即可
  //当rang  < distance 当清除距离小于到地图边界的距离,ex 只延伸到清除距离即可，无需到地图边界
  double ex = ox + dx * scale, ey = oy + dy * scale;
  touch(ex, ey, min_x, min_y, max_x, max_y);
}

void ObstacleLayer::reset()
{
    deactivate();
    resetMaps(); // call inherited costmap2d resetMaps  调用继承的costmap2d resetMaps
    current_ = true;
    activate();
}

}  // namespace costmap_2d
