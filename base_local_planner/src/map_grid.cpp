/*********************************************************************
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
 *********************************************************************/
#include <base_local_planner/map_grid.h>
#include <costmap_2d/cost_values.h>
using namespace std;

namespace base_local_planner{

  MapGrid::MapGrid()
    : size_x_(0), size_y_(0)
  {
  }

  MapGrid::MapGrid(unsigned int size_x, unsigned int size_y) 
    : size_x_(size_x), size_y_(size_y)
  {
    commonInit();
  }

  MapGrid::MapGrid(const MapGrid& mg){
    size_y_ = mg.size_y_;
    size_x_ = mg.size_x_;
    map_ = mg.map_;
  }

  void MapGrid::commonInit(){
    //don't allow construction of zero size grid
    ROS_ASSERT(size_y_ != 0 && size_x_ != 0);

    map_.resize(size_y_ * size_x_);

    //make each cell aware of its location in the grid
    for(unsigned int i = 0; i < size_y_; ++i){
      for(unsigned int j = 0; j < size_x_; ++j){
        unsigned int id = size_x_ * i + j;
        map_[id].cx = j;
        map_[id].cy = i;
      }
    }
  }

  size_t MapGrid::getIndex(int x, int y){
    return size_x_ * y + x;
  }

  MapGrid& MapGrid::operator= (const MapGrid& mg){
    size_y_ = mg.size_y_;
    size_x_ = mg.size_x_;
    map_ = mg.map_;
    return *this;
  }

  void MapGrid::sizeCheck(unsigned int size_x, unsigned int size_y){
    if(map_.size() != size_x * size_y)//map_当前尺寸和参数costmap要求的尺寸不一致时，sizeCheck会销毁旧的map_、生成新的
      map_.resize(size_x * size_y);//这个resize里面初始化是0吗，resize并不会对原vector已经存在的元素进行重新初始化

    if(size_x_ != size_x || size_y_ != size_y){
      size_x_ = size_x;
      size_y_ = size_y;

      for(unsigned int i = 0; i < size_y_; ++i){
        for(unsigned int j = 0; j < size_x_; ++j){
          int index = size_x_ * i + j;
          map_[index].cx = j;
          map_[index].cy = i;
        }
      }
    }
  }

//computeTargetDistance会膨胀出代价地图中的所有点吗？——答案会，但前提是代价地图中不能存在距离是obstacleCosts(3600)的点。updatePathCell遇到这些点，会强制以它终止膨胀。那些无法膨胀到的点就保留初始化后的默认值，即unreachableCellCosts(3601)。
  inline bool MapGrid::updatePathCell(MapCell* current_cell, MapCell* check_cell,
      const costmap_2d::Costmap2D& costmap){

    //if the cell is an obstacle set the max path distance
    unsigned char cost = costmap.getCost(check_cell->cx, check_cell->cy);
    if(! getCell(check_cell->cx, check_cell->cy).within_robot &&//如果这个cell在足迹之内，或者代价是致死代价，就把这个cell的target_dist设置为size(),并且返回false
        (cost == costmap_2d::LETHAL_OBSTACLE ||
         cost == costmap_2d::INSCRIBED_INFLATED_OBSTACLE ||
         cost == costmap_2d::NO_INFORMATION)){
      check_cell->target_dist = obstacleCosts();//obstacleCosts()数值是map_.size()，示例就是3600。
      return false;
    }

    double new_target_dist = current_cell->target_dist + 1;
    if (new_target_dist < check_cell->target_dist) {//如果当前代价+1 小于 check_cell 的原有代价，则将其更新为更小值
      check_cell->target_dist = new_target_dist;
    }
    return true;
  }


  //reset the path_dist and goal_dist fields for all cells 重置所有单元格的 path_dist 和 goal_dist 字段
  void MapGrid::resetPathDist(){
    for(unsigned int i = 0; i < map_.size(); ++i) {
      map_[i].target_dist = unreachableCellCosts();//重置的时候代价为不可达代价，只有这里用到不可达？怎么判断在墙后？
      map_[i].target_mark = false;
      map_[i].within_robot = false;
    }
  }
//adjustPlanResolution对映射到局部地图中的全局路径进行栅格级插补，从而使得整个路径是栅格连续的。插补思路是这样的，1）所有全局路径的坐标点都会保留。2）相邻两个坐标的距离（直角三角形斜边）超过resolution时，内插坐标点，确保相邻两个坐标距离<=resolution。
  void MapGrid::adjustPlanResolution(const std::vector<geometry_msgs::PoseStamped>& global_plan_in,
      std::vector<geometry_msgs::PoseStamped>& global_plan_out, double resolution) {//根据分辨率来调整路径点，这个具体实现没有看！
    if (global_plan_in.size() == 0) {
      return;
    }
    double last_x = global_plan_in[0].pose.position.x;
    double last_y = global_plan_in[0].pose.position.y;
    global_plan_out.push_back(global_plan_in[0]);

    double min_sq_resolution = resolution * resolution;
   //for循环目的：找到全局路径中第一个离开局部地图的坐标点
    for (unsigned int i = 1; i < global_plan_in.size(); ++i) {//相邻两个坐标的距离（直角三角形斜边）超过resolution时，内插坐标点，确保相邻两个坐标距离<=resolution
      double loop_x = global_plan_in[i].pose.position.x;
      double loop_y = global_plan_in[i].pose.position.y;
      double sqdist = (loop_x - last_x) * (loop_x - last_x) + (loop_y - last_y) * (loop_y - last_y);
      if (sqdist > min_sq_resolution) {
        int steps = ceil((sqrt(sqdist)) / resolution);
        // add a points in-between
        double deltax = (loop_x - last_x) / steps;
        double deltay = (loop_y - last_y) / steps;
        // TODO: Interpolate orientation
        for (int j = 1; j < steps; ++j) {
          geometry_msgs::PoseStamped pose;
          pose.pose.position.x = last_x + j * deltax;
          pose.pose.position.y = last_y + j * deltay;
          pose.pose.position.z = global_plan_in[i].pose.position.z;
          pose.pose.orientation = global_plan_in[i].pose.orientation;
          pose.header = global_plan_in[i].header;
          global_plan_out.push_back(pose);
        }
      }
      global_plan_out.push_back(global_plan_in[i]);
      last_x = loop_x;
      last_y = loop_y;
    }
  }

  //update what map cells are considered path based on the global_plan
  void MapGrid::setTargetCells(const costmap_2d::Costmap2D& costmap,
      const std::vector<geometry_msgs::PoseStamped>& global_plan) {//只把局部路径的所有点作为0代价
    sizeCheck(costmap.getSizeInCellsX(), costmap.getSizeInCellsY());

    bool started_path = false;

    queue<MapCell*> path_dist_queue;

    std::vector<geometry_msgs::PoseStamped> adjusted_global_plan;
    adjustPlanResolution(global_plan, adjusted_global_plan, costmap.getResolution());
    if (adjusted_global_plan.size() != global_plan.size()) {
      ROS_DEBUG("Adjusted global plan resolution, added %zu points", adjusted_global_plan.size() - global_plan.size());
    }
    unsigned int i;
    // put global path points into local map until we reach the border of the local map
    for (i = 0; i < adjusted_global_plan.size(); ++i) {//for循环目的：把全局路径的坐标逐个存入零线（path_dist_queue），直到遇到第一个离开局部地图的坐标点
      double g_x = adjusted_global_plan[i].pose.position.x;
      double g_y = adjusted_global_plan[i].pose.position.y;
      unsigned int map_x, map_y;
      if (costmap.worldToMap(g_x, g_y, map_x, map_y) && costmap.getCost(map_x, map_y) != costmap_2d::NO_INFORMATION) {
        MapCell& current = getCell(map_x, map_y);
        current.target_dist = 0.0;//当前那个cell要设置为0，这个也就是cost,右侧图中可看到，所有零线中的点，距离都置了0
        current.target_mark = true;
        path_dist_queue.push(&current);
        started_path = true;
      } else if (started_path) {
          break;
      }
    }
    if (!started_path) {
      ROS_ERROR("None of the %d first of %zu (%zu) points of the global plan were in the local costmap and free",
          i, adjusted_global_plan.size(), global_plan.size());
      return;
    }

    computeTargetDistance(path_dist_queue, costmap);//computeTargetDistance结束后会改修改各个MapCell的target_dist
  }

  //mark the point of the costmap as local goal where global_plan first leaves the area (or its last point) 将代价地图的点标记为 global_plan 首先离开该区域（或其最后一个点）的局部目标
  void MapGrid::setLocalGoal(const costmap_2d::Costmap2D& costmap,
      const std::vector<geometry_msgs::PoseStamped>& global_plan) {//只把局部路径的最后一个点作为0代价
    sizeCheck(costmap.getSizeInCellsX(), costmap.getSizeInCellsY());

    int local_goal_x = -1;
    int local_goal_y = -1;
    bool started_path = false;

    std::vector<geometry_msgs::PoseStamped> adjusted_global_plan;//for找出距离目标最近、但还落在局部地图的坐标
    adjustPlanResolution(global_plan, adjusted_global_plan, costmap.getResolution());//首先对映射到局部地图中的路径进行栅格级的插补(MapGrid::adjustPlanResolution)，从而使得整个路径是栅格连续的。

    // skip global path points until we reach the border of the local map
    for (unsigned int i = 0; i < adjusted_global_plan.size(); ++i) {
      double g_x = adjusted_global_plan[i].pose.position.x;
      double g_y = adjusted_global_plan[i].pose.position.y;
      unsigned int map_x, map_y;//转换为地图的坐标
      if (costmap.worldToMap(g_x, g_y, map_x, map_y) && costmap.getCost(map_x, map_y) != costmap_2d::NO_INFORMATION) {
        local_goal_x = map_x;
        local_goal_y = map_y;
        started_path = true;
      } else {
        if (started_path) {
          break; 
        }// else we might have a non pruned path, so we just continue 否则我们可能有一条未修剪的路径，所以我们继续 ，什么意思？
      }
    }
    if (!started_path) {//也就是说全局路径没有一个点在局部代价地图中
      ROS_ERROR("None of the points of the global plan were in the local costmap, global plan points too far from robot");
      return;
    }

    queue<MapCell*> path_dist_queue;
    if (local_goal_x >= 0 && local_goal_y >= 0) {
      MapCell& current = getCell(local_goal_x, local_goal_y);
      costmap.mapToWorld(local_goal_x, local_goal_y, goal_x_, goal_y_);//这个goal_x_是全局坐标
      current.target_dist = 0.0;
      current.target_mark = true;
      path_dist_queue.push(&current);
    }
    // 接下要计算局部地图中所有点到[62]坐标的距离。对[62]，把target_mark置为true，表示不必再以它为中心去膨胀了
    computeTargetDistance(path_dist_queue, costmap);//然后调用computeTargetDistance函数采用类似dijkstra算法的逐步探索的方式，计算出MapGrid地图中所有点（栅格级）相对于与标记点的最短距离
  }



  void MapGrid::computeTargetDistance(queue<MapCell*>& dist_queue, const costmap_2d::Costmap2D& costmap){//利用wave propagation的效果对队列进行膨胀
    MapCell* current_cell;
    MapCell* check_cell;
    unsigned int last_col = size_x_ - 1;
    unsigned int last_row = size_y_ - 1;
    while(!dist_queue.empty()){
      current_cell = dist_queue.front();


      dist_queue.pop();

      if(current_cell->cx > 0){
        check_cell = current_cell - 1;
        if(!check_cell->target_mark){//这个target_mark是这个cell是否被访问的标志
          //mark the cell as visisted
          check_cell->target_mark = true;//表示访问过
          if(updatePathCell(current_cell, check_cell, costmap)) {//如果是返回是障碍物区域，则不会压入栈中
            dist_queue.push(check_cell);
          }
        }
      }

      if(current_cell->cx < last_col){
        check_cell = current_cell + 1;
        if(!check_cell->target_mark){
          check_cell->target_mark = true;
          if(updatePathCell(current_cell, check_cell, costmap)) {
            dist_queue.push(check_cell);
          }
        }
      }

      if(current_cell->cy > 0){
        check_cell = current_cell - size_x_;
        if(!check_cell->target_mark){
          check_cell->target_mark = true;
          if(updatePathCell(current_cell, check_cell, costmap)) {
            dist_queue.push(check_cell);
          }
        }
      }

      if(current_cell->cy < last_row){
        check_cell = current_cell + size_x_;
        if(!check_cell->target_mark){
          check_cell->target_mark = true;
          if(updatePathCell(current_cell, check_cell, costmap)) {
            dist_queue.push(check_cell);
          }
        }
      }
    }
  }

};
