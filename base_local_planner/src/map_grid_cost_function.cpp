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
 * Author: TKruse
 *********************************************************************/

#include <base_local_planner/map_grid_cost_function.h>

namespace base_local_planner {

MapGridCostFunction::MapGridCostFunction(costmap_2d::Costmap2D* costmap,
    double xshift,
    double yshift,
    bool is_local_goal_function,
    CostAggregationType aggregationType) ://aggregationType_如果是last，那就是只考虑结果的最后一个点
    costmap_(costmap),
    map_(costmap->getSizeInCellsX(), costmap->getSizeInCellsY()),
    aggregationType_(aggregationType),
    xshift_(xshift),
    yshift_(yshift),
    is_local_goal_function_(is_local_goal_function),
    stop_on_failure_(true) {}//初始化列表按照成员声明顺序进行初始化是很好的

void MapGridCostFunction::setTargetPoses(std::vector<geometry_msgs::PoseStamped> target_poses) {
  target_poses_ = target_poses;
}

bool MapGridCostFunction::prepare() {//MapGridCostFunction有关的四个打分项在prepare中会对局部地图进行预计算，从而方便后边进行打分
  map_.resetPathDist();

  if (is_local_goal_function_) {//如果为true，则
    map_.setLocalGoal(*costmap_, target_poses_);//将最后一个点作为代价值，计算costmap
  } else {
    map_.setTargetCells(*costmap_, target_poses_);//将局部路径所有点作为代价值，计算costmap
  }
  return true;
}

double MapGridCostFunction::getCellCosts(unsigned int px, unsigned int py) {
  double grid_dist = map_(px, py).target_dist;//这个是对哪个map进行的计算？
  return grid_dist;
}

double MapGridCostFunction::scoreTrajectory(Trajectory &traj) {
  double cost = 0.0;
  if (aggregationType_ == Product) {//如果代价是使用乘法，那就cost初始值为1
    cost = 1.0;
  }
  double px, py, pth;
  unsigned int cell_x, cell_y;
  double grid_dist;

  for (unsigned int i = 0; i < traj.getPointsSize(); ++i) {//对轨迹中的每个点进行遍历
    traj.getPoint(i, px, py, pth);

    // translate point forward if specified 如果指定，向前翻译点
    if (xshift_ != 0.0) {//这里注意，这里的shift是将轨迹中每个点都向pth方向移动固定的距离
      px = px + xshift_ * cos(pth);
      py = py + xshift_ * sin(pth);
    }
    // translate point sideways if specified
    if (yshift_ != 0.0) {
      px = px + yshift_ * cos(pth + M_PI_2);
      py = py + yshift_ * sin(pth + M_PI_2);
    }
    //我们不允许偏离地图的轨迹......无论如何不应该经常发生
    //we won't allow trajectories that go off the map... shouldn't happen that often anyways
    if ( ! costmap_->worldToMap(px, py, cell_x, cell_y)) {
      //we're off the map
      ROS_WARN("Off Map %f, %f", px, py);//note-zhijie 遇到墙会出现这个 //因为这个是代价地图窗口导致，当局部路径过长的话超出地图边界，就会报错
      return -4.0;
    }
    grid_dist = getCellCosts(cell_x, cell_y);
    //if a point on this trajectory has no clear path to the goal... it may be invalid 如果这条轨迹上的一个点没有通往目标的明确路径……它可能是无效的
    if (stop_on_failure_) {//即如果轨迹中的某个点是障碍物或不可到达，则表示查找该点的得分失败，这会造成scoreTrajectory函数直接返回对应的负的得分。
      if (grid_dist == map_.obstacleCosts()) {
        return -3.0;//例如，如果轨迹中的某个点本身就是障碍物（在路径MapGrid中会有标记），则scoreTrajectory直接返回-3作为轨迹得分
      } else if (grid_dist == map_.unreachableCellCosts()) {
        return -2.0;//如果轨迹中某个点不可达（在MapGrid地图中没有计算到），则scoreTrajectory直接返回-2作为轨迹得分。
      }
    }
    // 对于goal_front_costs和alignment_costs_两个打分项对应的stop_on_failure参数为false，这是因为前向打分点有可能会出现计算距离出错的情况，例如超出地图范围，但由于前向打分点的打分失败不会带来危险，因此可以不立刻返回负得分

    switch( aggregationType_ ) {//这个来维护最终的代价计算
    case Last:
      cost = grid_dist;
      break;
    case Sum:
      cost += grid_dist;
      break;
    case Product:
      if (cost > 0) {
        cost *= grid_dist;
      }
      break;
    }
  }
  return cost;
}

} /* namespace base_local_planner */
