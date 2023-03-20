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

#include <base_local_planner/obstacle_cost_function.h>
#include <cmath>
#include <Eigen/Core>
#include <ros/console.h>

namespace base_local_planner {

ObstacleCostFunction::ObstacleCostFunction(costmap_2d::Costmap2D* costmap) 
    : costmap_(costmap), sum_scores_(false) {
  if (costmap != NULL) {
    world_model_ = new base_local_planner::CostmapModel(*costmap_);//从初始化函数可以看出来，ObstacleCostFunction选用了CostmapModel作为具体实现，想改为其他具体实现随时可以改这一行。
  }
}

ObstacleCostFunction::~ObstacleCostFunction() {
  if (world_model_ != NULL) {
    delete world_model_;
  }
}


void ObstacleCostFunction::setParams(double max_trans_vel, double max_scaling_factor, double scaling_speed) {
  // TODO: move this to prepare if possible
  max_trans_vel_ = max_trans_vel;
  max_scaling_factor_ = max_scaling_factor;
  scaling_speed_ = scaling_speed;
}

void ObstacleCostFunction::setFootprint(std::vector<geometry_msgs::Point> footprint_spec) {
  footprint_spec_ = footprint_spec;
}

bool ObstacleCostFunction::prepare() {
  return true;
}

double ObstacleCostFunction::scoreTrajectory(Trajectory &traj) {//计算得分思路：依次把机器人中心放在轨迹中所有坐标点，算出机器人足迹涉及到栅格中最大的cost。>=0的返回值中，最大值是253(INSCRIBED_INFLATED_OBSTACLE)。
  double cost = 0;
  double scale = getScalingFactor(traj, scaling_speed_, max_trans_vel_, max_scaling_factor_);
  double px, py, pth;
  if (footprint_spec_.size() == 0) {
    // Bug, should never happen   11月07DWA 运行时在rviz中发送目标点时报错 note by zhijie
    ROS_ERROR("Footprint spec is empty, maybe missing call to setFootprint?");
    return -9;
  }

  for (unsigned int i = 0; i < traj.getPointsSize(); ++i) {//遍历每个轨迹点
    traj.getPoint(i, px, py, pth);
    double f_cost = footprintCost(px, py, pth,
        scale, footprint_spec_,
        costmap_, world_model_);

    if(f_cost < 0){
        return f_cost;//若某个点直接为负，那么直接返回
    }

    if(sum_scores_)//默认为false，不进入累加cost
        cost +=  f_cost;
    else
        cost = std::max(cost, f_cost);//获得轨迹点中最大的cost代价
  }
  return cost;
}

double ObstacleCostFunction::getScalingFactor(Trajectory &traj, double scaling_speed, double max_trans_vel, double max_scaling_factor) {
  double vmag = hypot(traj.xv_, traj.yv_);//生成这条轨迹的最大合速度

  //if we're over a certain speed threshold, we'll scale the robot's
  //footprint to make it either slow down or stay further from walls  如果我们超过某个速度阈值，我们将缩放机器人的足迹以使其减速或远离墙壁
  //如果当前平移速度小于scaling_speed_，则缩放因子为1.0，否则，缩放因子为(vmag - scaling_speed) / (max_trans_vel - scaling_speed) * max_scaling_factor + 1.0。然后，该缩放因子会被用于计算轨迹中各个点的footprintCost。
  double scale = 1.0;
  if (vmag > scaling_speed) {
    //scale up to the max scaling factor linearly... this could be changed later 线性放大到最大比例因子...这可以在以后更改
    double ratio = (vmag - scaling_speed) / (max_trans_vel - scaling_speed);//这里没太看懂
    scale = max_scaling_factor * ratio + 1.0;
  }
  return scale;
}

double ObstacleCostFunction::footprintCost (
    const double& x,
    const double& y,
    const double& th,
    double scale,
    std::vector<geometry_msgs::Point> footprint_spec,
    costmap_2d::Costmap2D* costmap,
    base_local_planner::WorldModel* world_model) {//计算车辆模型的代价类

  std::vector<geometry_msgs::Point> scaled_footprint;//缩放后的footprint
  for(unsigned int i  = 0; i < footprint_spec.size(); ++i) {
    geometry_msgs::Point new_pt;
    new_pt.x = scale * footprint_spec[i].x;
    new_pt.y = scale * footprint_spec[i].y;
    scaled_footprint.push_back(new_pt);
  }

  //check if the footprint is legal 检查足迹是否合法
  // TODO: Cache inscribed radius
  double footprint_cost = world_model->footprintCost(x, y, th, scaled_footprint);//计算footprint的真实位置是否有cost

  if (footprint_cost < 0) {//如果足迹代价小于0，直接返回-6的代价
    return -6.0;
  }
  unsigned int cell_x, cell_y;

  //we won't allow trajectories that go off the map... shouldn't happen that often anyways
  if ( ! costmap->worldToMap(x, y, cell_x, cell_y)) {
    return -7.0;//轨迹点在地图之外，返回-7的代价
  }

  double occ_cost = std::max(std::max(0.0, footprint_cost), double(costmap->getCost(cell_x, cell_y)));//选择当前代价与footprint代价中的最大值

  return occ_cost;
}

} /* namespace base_local_planner */
