/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2009, Willow Garage, Inc.
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
*********************************************************************/
#include <clear_costmap_recovery/clear_costmap_recovery.h>
#include <pluginlib/class_list_macros.h>
#include <boost/pointer_cast.hpp>
#include <vector>

//register this planner as a RecoveryBehavior plugin
PLUGINLIB_EXPORT_CLASS(clear_costmap_recovery::ClearCostmapRecovery, nav_core::RecoveryBehavior)

using costmap_2d::NO_INFORMATION;

namespace clear_costmap_recovery {
ClearCostmapRecovery::ClearCostmapRecovery(): global_costmap_(NULL), local_costmap_(NULL),
  tf_(NULL), initialized_(false) {}
//首先以当前机器人中心为中心，向上、下、左、右各扩展reset_distance_米生成一个矩形R。然后判断invert_area_to_clear_。是false时，清除R矩形内栅格。是true时，清除代价地图内除R矩形外的栅格
void ClearCostmapRecovery::initialize(std::string name, tf2_ros::Buffer* tf,
    costmap_2d::Costmap2DROS* global_costmap, costmap_2d::Costmap2DROS* local_costmap){
  if(!initialized_){
    name_ = name;
    tf_ = tf;
    global_costmap_ = global_costmap;
    local_costmap_ = local_costmap;

    //get some parameters from the parameter server
    ros::NodeHandle private_nh("~/" + name_);

    private_nh.param("reset_distance", reset_distance_, 3.0);//首先以当前机器人中心为中心，向上、下、左、右各扩展reset_distance_米生成一个矩形R
    private_nh.param("invert_area_to_clear", invert_area_to_clear_, false);//然后判断invert_area_to_clear_。是false时，清除R矩形内栅格。是true时，清除代价地图内除R矩形外的栅格
    private_nh.param("force_updating", force_updating_, false);//对代价地图执行清除操作后，如果fore_upateing_，还会对该地图执行updateMap
    private_nh.param("affected_maps", affected_maps_, std::string("both"));//affected_maps_参数决定要处理哪些地图，是both时同时处理两种，global只处理全局代价地图，local则只处理局部代价地图。
    if (affected_maps_ != "local" && affected_maps_ != "global" && affected_maps_ != "both")
    {
      ROS_WARN("Wrong value for affected_maps parameter: '%s'; valid values are 'local', 'global' or 'both'; " \
               "defaulting to 'both'", affected_maps_.c_str());
      affected_maps_ = "both";
    }

    std::vector<std::string> clearable_layers_default, clearable_layers;
    clearable_layers_default.push_back( std::string("obstacles") );
    private_nh.param("layer_names", clearable_layers, clearable_layers_default);//用gdb调试的时候看到这里，就是一个obstacles而已

    for(unsigned i=0; i < clearable_layers.size(); i++) {
        ROS_INFO("Recovery behavior will clear layer '%s'", clearable_layers[i].c_str());
        clearable_layers_.insert(clearable_layers[i]);//这个clearable_layers到头来就只有一个obstacles，clearable_layers_决定要清除的是哪些层，一般设置的是{obstacle_layer}，即只清除障碍层
    }

    initialized_ = true;
  }
  else{
    ROS_ERROR("You should not call initialize twice on this object, doing nothing");
  }
}

void ClearCostmapRecovery::runBehavior(){
  if(!initialized_){
    ROS_ERROR("This object must be initialized before runBehavior is called");
    return;
  }

  if(global_costmap_ == NULL || local_costmap_ == NULL){
    ROS_ERROR("The costmaps passed to the ClearCostmapRecovery object cannot be NULL. Doing nothing.");
    return;
  }

  if (!invert_area_to_clear_){
    ROS_WARN("Clearing %s costmap%s outside a square (%.2fm) large centered on the robot.", affected_maps_.c_str(),
           affected_maps_ == "both" ? "s" : "", reset_distance_);
  }else {
    ROS_WARN("Clearing %s costmap%s inside a square (%.2fm) large centered on the robot.", affected_maps_.c_str(),
           affected_maps_ == "both" ? "s" : "", reset_distance_);
  }

  ros::WallTime t0 = ros::WallTime::now();
  if (affected_maps_ == "global" || affected_maps_ == "both")
  {
    clear(global_costmap_);

    if (force_updating_)
      global_costmap_->updateMap();//全局地图的更新接口吗

    ROS_DEBUG("Global costmap cleared in %fs", (ros::WallTime::now() - t0).toSec());
  }

  t0 = ros::WallTime::now();
  if (affected_maps_ == "local" || affected_maps_ == "both")
  {
    clear(local_costmap_);

    if (force_updating_)
      local_costmap_->updateMap();

    ROS_DEBUG("Local costmap cleared in %fs", (ros::WallTime::now() - t0).toSec());
  }
}

void ClearCostmapRecovery::clear(costmap_2d::Costmap2DROS* costmap){//clear(...)作用是获取枚举代价地图中的各层costmap，如果该层在clearable_layers_，对其调用clearMap进行清理
  std::vector<boost::shared_ptr<costmap_2d::Layer> >* plugins = costmap->getLayeredCostmap()->getPlugins();

  geometry_msgs::PoseStamped pose;

  if(!costmap->getRobotPose(pose)){
    ROS_ERROR("Cannot clear map because pose cannot be retrieved");
    return;
  }

  double x = pose.pose.position.x;
  double y = pose.pose.position.y;

  for (std::vector<boost::shared_ptr<costmap_2d::Layer> >::iterator pluginp = plugins->begin(); pluginp != plugins->end(); ++pluginp) {
    boost::shared_ptr<costmap_2d::Layer> plugin = *pluginp;
    std::string name = plugin->getName();
    int slash = name.rfind('/');//string 的 rfind 是一个方法，它用来查找字符串中最后一次出现的指定子串的位置
    if( slash != std::string::npos ){//std::string::npos 是一个静态成员常量，它表示 size_t 类型的最大可能值。它通常用来表示字符串中不存在的位置或者没有匹配的结果。例如，如果你用 str.find(str2) 来查找 str2 在 str 中的位置，如果没有找到，那么返回值就是 std::string::npos。1
        name = name.substr(slash+1);
    }
    //plugin->getName()值示例：global_costmap/static_layer。name是它的后半部static_layer。

    if(clearable_layers_.count(name)!=0){

      // check if the value is convertable
      if(!dynamic_cast<costmap_2d::CostmapLayer*>(plugin.get())){
        ROS_ERROR_STREAM("Layer " << name << " is not derived from costmap_2d::CostmapLayer");
        continue;
      }

      boost::shared_ptr<costmap_2d::CostmapLayer> costmap;
      costmap = boost::static_pointer_cast<costmap_2d::CostmapLayer>(plugin);
      // static_pointer_cast 是一个函数模板，它用来将一个 shared_ptr 类型的智能指针静态转换为另一个 shared_ptr 类型的智能指针。12
      // 它相当于对智能指针所管理的原始指针进行 static_cast 操作，但是保持了共享所有权的特性。2
      // 例如，如果你有一个基类 A 和一个派生类 B，你可以用 static_pointer_cast 将一个 shared_ptr<A> 类型的智能指针转换为一个 shared_ptr<B> 类型的智能指针，只要它们所指向的对象是同一个。2
      clearMap(costmap, x, y);
    }
  }
}


void ClearCostmapRecovery::clearMap(boost::shared_ptr<costmap_2d::CostmapLayer> costmap,
                                        double pose_x, double pose_y){
  boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*(costmap->getMutex()));

  double start_point_x = pose_x - reset_distance_ / 2;
  double start_point_y = pose_y - reset_distance_ / 2;
  double end_point_x = start_point_x + reset_distance_;
  double end_point_y = start_point_y + reset_distance_;

  int start_x, start_y, end_x, end_y;
  costmap->worldToMapNoBounds(start_point_x, start_point_y, start_x, start_y);
  costmap->worldToMapNoBounds(end_point_x, end_point_y, end_x, end_y);

  costmap->clearArea(start_x, start_y, end_x, end_y, invert_area_to_clear_);//这个东西的实现我怎么没有找到

  double ox = costmap->getOriginX(), oy = costmap->getOriginY();
  double width = costmap->getSizeInMetersX(), height = costmap->getSizeInMetersY();
  costmap->addExtraBounds(ox, oy, ox + width, oy + height);
  return;
}

};
