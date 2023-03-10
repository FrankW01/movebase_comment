/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, 2013, Willow Garage, Inc.
 *  Copyright (c) 2015, Fetch Robotics, Inc.
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
#include <costmap_2d/static_layer.h>
#include <costmap_2d/costmap_math.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(costmap_2d::StaticLayer, costmap_2d::Layer)

using costmap_2d::NO_INFORMATION;
using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::FREE_SPACE;

namespace costmap_2d
{

StaticLayer::StaticLayer() : dsrv_(NULL) {}

StaticLayer::~StaticLayer()
{
  if (dsrv_)
    delete dsrv_;
}

void StaticLayer::onInitialize()//Costmap2DROS的构造函数会调用各Layer中的initialize函数，而initialize函数会调用onInitialize函数，真正的初始化工作在这里完成。
{
  ros::NodeHandle nh("~/" + name_), g_nh;
  current_ = true;

  global_frame_ = layered_costmap_->getGlobalFrameID();

  std::string map_topic;
  nh.param("map_topic", map_topic, std::string("map"));
  nh.param("first_map_only", first_map_only_, false);
  nh.param("subscribe_to_updates", subscribe_to_updates_, false);

  nh.param("track_unknown_space", track_unknown_space_, true);//这个是什么参数定义？ 不将未知区域改为自由区域
  nh.param("use_maximum", use_maximum_, false);

  int temp_lethal_threshold, temp_unknown_cost_value;
  nh.param("lethal_cost_threshold", temp_lethal_threshold, int(100));//默认是100？
  nh.param("unknown_cost_value", temp_unknown_cost_value, int(-1));
  nh.param("trinary_costmap", trinary_costmap_, true);

  lethal_threshold_ = std::max(std::min(temp_lethal_threshold, 100), 0);//这个temp_l_t不会超过100，最多100
  unknown_cost_value_ = temp_unknown_cost_value;

  // Only resubscribe if topic has changed 只有当话题改变后才重新订阅
  if (map_sub_.getTopic() != ros::names::resolve(map_topic))
  {
    // we'll subscribe to the latched topic that the map server uses
    ROS_INFO("Requesting the map...");
    map_sub_ = g_nh.subscribe(map_topic, 1, &StaticLayer::incomingMap, this);//订阅地图话题的回调函数
    map_received_ = false;
    has_updated_data_ = false;

    ros::Rate r(10);
    while (!map_received_ && g_nh.ok()) //如果map_received_一直是false，则一直阻塞在这里，只有在接收到后进入回调函数更新为true后才会继续进行
    {
      ros::spinOnce();
      r.sleep();
    }

    ROS_INFO("Received a %d X %d map at %f m/pix", getSizeInCellsX(), getSizeInCellsY(), getResolution());

    if (subscribe_to_updates_)
    {
      ROS_INFO("Subscribing to updates");
      map_update_sub_ = g_nh.subscribe(map_topic + "_updates", 10, &StaticLayer::incomingUpdate, this);//这个是什么回调？

    }
  }
  else
  {
    has_updated_data_ = true;//该行设置的逻辑什么？
  }

  if (dsrv_)
  {
    delete dsrv_;
  }

  dsrv_ = new dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>(nh);
  dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>::CallbackType cb = boost::bind(
      &StaticLayer::reconfigureCB, this, _1, _2);
  dsrv_->setCallback(cb);
}

void StaticLayer::reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level)
{
  if (config.enabled != enabled_)
  {
    enabled_ = config.enabled;
    has_updated_data_ = true;
    x_ = y_ = 0;
    width_ = size_x_;
    height_ = size_y_;
  }
}

void StaticLayer::matchSize()
{
  // If we are using rolling costmap, the static map size is
  //   unrelated to the size of the layered costmap
  if (!layered_costmap_->isRolling())
  {
    Costmap2D* master = layered_costmap_->getCostmap();
    resizeMap(master->getSizeInCellsX(), master->getSizeInCellsY(), master->getResolution(),
              master->getOriginX(), master->getOriginY());
  }
}

unsigned char StaticLayer::interpretValue(unsigned char value)//代价的重新映射
{
  // check if the static value is above the unknown or lethal thresholds 检查静态值是否高于未知或致命阈值
  if (track_unknown_space_ && value == unknown_cost_value_)//如果追踪未知区域，且传入的参数代表未知，设置为NO_INFORMATION
    return NO_INFORMATION;
  else if (!track_unknown_space_ && value == unknown_cost_value_)//如果不追踪未知区域，且传入的参数代表未知，设置为FREE_SPACE
    return FREE_SPACE;
  else if (value >= lethal_threshold_)//这个超过设定的致死参数就设定为 254 LETHAL_OBSTACLE
    return LETHAL_OBSTACLE;
  else if (trinary_costmap_)//用于设置是否对代价地图进行三值化。如果为true，代价地图应该被解释为只有3个值：空闲、占用、未知。不是否则直接使用其按比算出的代价值。
    return FREE_SPACE;

  double scale = (double) value / lethal_threshold_;
  return scale * LETHAL_OBSTACLE;
}

void StaticLayer::incomingMap(const nav_msgs::OccupancyGridConstPtr& new_map)//Occupancy probabilities are in the range [0,100].  Unknown is -1.  收到地图的回调函数
{
  unsigned int size_x = new_map->info.width, size_y = new_map->info.height;

  ROS_DEBUG("Received a %d X %d map at %f m/pix", size_x, size_y, new_map->info.resolution);

  // resize costmap if size, resolution or origin do not match
  Costmap2D* master = layered_costmap_->getCostmap();
  if (!layered_costmap_->isRolling() && (master->getSizeInCellsX() != size_x ||
      master->getSizeInCellsY() != size_y ||
      master->getResolution() != new_map->info.resolution ||
      master->getOriginX() != new_map->info.origin.position.x ||
      master->getOriginY() != new_map->info.origin.position.y ||
      !layered_costmap_->isSizeLocked())) // 地图更新，当大小，分辨率，起点不同时调整地图,如果master map的尺寸、分辨率或原点与获取到的地图不匹配，重新设置master map
  {
    // Update the size of the layered costmap (and all layers, including this one)
    ROS_INFO("Resizing costmap to %d X %d at %f m/pix", size_x, size_y, new_map->info.resolution);//slam过程中常见
    layered_costmap_->resizeMap(size_x, size_y, new_map->info.resolution, new_map->info.origin.position.x,
                                new_map->info.origin.position.y, true);//更新主代价地图? 聚合层指针的resizeMap，按照各层逐一去更新代价地图，最终反馈在主代价地图上
  }
  else if (size_x_ != size_x || size_y_ != size_y ||
           resolution_ != new_map->info.resolution ||
           origin_x_ != new_map->info.origin.position.x ||
           origin_y_ != new_map->info.origin.position.y)//更新静态层代价,若本层的数据和接收到的静态地图的参数不同时，继续以接收到的地图为准，调整本层参数。
  {
    // only update the size of the costmap stored locally in this layer
    ROS_INFO("Resizing static layer to %d X %d at %f m/pix", size_x, size_y, new_map->info.resolution);
    resizeMap(size_x, size_y, new_map->info.resolution,
              new_map->info.origin.position.x, new_map->info.origin.position.y);
  }

  unsigned int index = 0;

  // initialize the costmap with static data
  for (unsigned int i = 0; i < size_y; ++i)
  {
    for (unsigned int j = 0; j < size_x; ++j)
    {
      unsigned char value = new_map->data[index];
      costmap_[index] = interpretValue(value);//这个costmap_是静态层的数据，这里更新的是staticlayer的costmap，之后调用的在本层的updateCosts函数才真正反应到结果上
      ++index;
    }
  }
  map_frame_ = new_map->header.frame_id;

  // we have a new map, update full size of map
  x_ = y_ = 0;
  width_ = size_x_;
  height_ = size_y_;
  map_received_ = true;//map已经收到的标志量
  has_updated_data_ = true;//地图已经更新的标志量

  // shutdown the map subscrber if firt_map_only_ flag is on
  if (first_map_only_)//若first_map_only_开启，则在第一次接收到地图后，话题将不再接收消息
  {
    ROS_INFO("Shutting down the map subscriber. first_map_only flag is on");
    map_sub_.shutdown();//map sub 关闭
  }
}

void StaticLayer::incomingUpdate(const map_msgs::OccupancyGridUpdateConstPtr& update)//只更新update这个地图中的区域
{
  unsigned int di = 0;
  for (unsigned int y = 0; y < update->height ; y++)
  {
    unsigned int index_base = (update->y + y) * size_x_;
    for (unsigned int x = 0; x < update->width ; x++)
    {
      unsigned int index = index_base + x + update->x;
      costmap_[index] = interpretValue(update->data[di++]);
    }
  }
  x_ = update->x;
  y_ = update->y;
  width_ = update->width;
  height_ = update->height;
  has_updated_data_ = true;
}

void StaticLayer::activate()//继承激活接口
{
  onInitialize();
}

void StaticLayer::deactivate()
{
  map_sub_.shutdown();
  if (subscribe_to_updates_)
    map_update_sub_.shutdown();
}

void StaticLayer::reset()
{
  if (first_map_only_)
  {
    has_updated_data_ = true;
  }
  else
  {
    onInitialize();
  }
}

void StaticLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y,
                               double* max_x, double* max_y)//这传入的代价地图都是以global map的坐标下的
{

  if( !layered_costmap_->isRolling() ){//若非rolling地图，在有地图数据更新时更新边界，否则，根据静态层更新的区域的边界更新传入的边界。
    if (!map_received_ || !(has_updated_data_ || has_extra_bounds_))
      return;
  }

  useExtraBounds(min_x, min_y, max_x, max_y);

  double wx, wy;

  mapToWorld(x_, y_, wx, wy);
  *min_x = std::min(wx, *min_x);
  *min_y = std::min(wy, *min_y);

  mapToWorld(x_ + width_, y_ + height_, wx, wy);//将map系中的起点（x_， y_）与终点（x_ + width_,， y_ + height_）转换到世界系，确保传入的bound能包含整个map在世界系中的范围。
  *max_x = std::max(wx, *max_x);
  *max_y = std::max(wy, *max_y);

  has_updated_data_ = false;
}

void StaticLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j)//静态层处理事先建好的地图，才真正反应到结果上，主代价地图以传参的形式传入，不作为该类的成员
{
  if (!map_received_)
    return;

  if (!layered_costmap_->isRolling())//若不是rolling地图，那么直接将静态地图层bound范围内的内容合并到主地图，因为二者的尺寸也一样，updateWithTrueOverwrite和updateWithMax采用不同的合并策略。
  {
    // if not rolling, the layered costmap (master_grid) has same coordinates as this layer  如果不滚动，分层成本图（master_grid）与该层具有相同的坐标
    if (!use_maximum_)//不使用最大值，就是采用重写机制
      updateWithTrueOverwrite(master_grid, min_i, min_j, max_i, max_j);
    else//否则使用最大值更新机制
      updateWithMax(master_grid, min_i, min_j, max_i, max_j);
  }
  else//若为rolling地图，则找到静态地图和global系之间的坐标转换，通过主地图→global→静态地图的转换过程，找到主地图的cell在静态地图上对应的cost，赋值给主地图。 
  {
    // If rolling window, the master_grid is unlikely to have same coordinates as this layer  如果是滚动窗口，则 master_grid 不太可能与该层具有相同的坐标
    unsigned int mx, my;
    double wx, wy;
    // Might even be in a different frame
    tf::StampedTransform transform;
    try
    {
      tf_->lookupTransform(map_frame_, global_frame_, ros::Time(0), transform);//如果是滚动的静态层，那这里的map frame与global frame分别是什么？以局部代价地图为例的话？,这里我认为是同一坐标系
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("%s", ex.what());
      return;
    }
    // Copy map data given proper transformations
    for (unsigned int i = min_i; i < max_i; ++i)
    {
      for (unsigned int j = min_j; j < max_j; ++j)
      {
        // Convert master_grid coordinates (i,j) into global_frame_(wx,wy) coordinates  //master map坐标(i,j)→global坐标(wx,wy)
        layered_costmap_->getCostmap()->mapToWorld(i, j, wx, wy);
        // Transform from global_frame_ to map_frame_
        tf::Point p(wx, wy, 0);
        p = transform(p);
        // Set master_grid with cell from map //global坐标p→静态地图坐标(mx, my)
        if (worldToMap(p.x(), p.y(), mx, my))
        {
          if (!use_maximum_)
            master_grid.setCost(i, j, getCost(mx, my));
          else
            master_grid.setCost(i, j, std::max(getCost(mx, my), master_grid.getCost(i, j)));
        }
      }
    }
  }
}

}  // namespace costmap_2d
