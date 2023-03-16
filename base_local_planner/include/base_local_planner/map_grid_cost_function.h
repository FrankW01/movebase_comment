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
//继承打分基类
#ifndef MAP_GRID_COST_FUNCTION_H_
#define MAP_GRID_COST_FUNCTION_H_

#include <base_local_planner/trajectory_cost_function.h>

#include <costmap_2d/costmap_2d.h>
#include <base_local_planner/map_grid.h>

namespace base_local_planner {

/**
 * when scoring a trajectory according to the values in mapgrid, we can take
 *return the value of the last point (if no of the earlier points were in
 * return collision), the sum for all points, or the product of all (non-zero) points * 当根据地图网格中的值对轨迹进行评分时，我们可以返回最后一个点的值（如果之前的点没有返回碰撞），所有点的总和，或所有点的乘积（非零）分
 */
enum CostAggregationType { Last, Sum, Product};//成本汇总类型

/**
 * This class provides cost based on a map_grid of a small area of the world.
 * The map_grid covers a the costmap, the costmap containing the information
 * about sensed obstacles. The map_grid is used by setting
 * certain cells to distance 0, and then propagating distances around them,
 * filling up the area reachable around them.
 *
 * The approach using grid_maps is used for computational efficiency, allowing to
 * score hundreds of trajectories very quickly.
 *
 * This can be used to favor trajectories which stay on a given path, or which
 * approach a given goal. 此类提供基于世界上一小块区域的 map_grid 的成本。 map_grid 覆盖了一个代价地图，代价地图包含关于感知到的障碍物的信息。 通过将某些单元格设置为距离 0 来使用 map_grid，然后在它们周围传播距离，填充它们周围可达的区域。 使用 grid_maps 的方法用于提高计算效率，允许非常快速地对数百条轨迹进行评分。 这可用于支持留在给定路径上或接近给定目标的轨迹
 * @param costmap_ros Reference to object giving updates of obstacles around robot
 * @param xshift where the scoring point is with respect to robot center pose  得分点是相对于机器人中心姿势的
 * @param yshift where the scoring point is with respect to robot center pose
 * @param is_local_goal_function, scores for local goal rather than whole path 对于局部目标而不是整个路径
 * @param aggregationType how to combine costs along trajectory
 */
class MapGridCostFunction: public base_local_planner::TrajectoryCostFunction {//继承了打分基类，MapGridCostFunction用来评估局部规划的轨迹离全局规划的轨迹的距离，也可以用来评估到目标的距离
public:
  MapGridCostFunction(costmap_2d::Costmap2D* costmap,
      double xshift = 0.0,
      double yshift = 0.0,
      bool is_local_goal_function = false,
      CostAggregationType aggregationType = Last);

  ~MapGridCostFunction() {}

  /**
   * set line segments on the grid with distance 0, resets the grid 在距离为0的网格上设置线段，重置网格
   */
  void setTargetPoses(std::vector<geometry_msgs::PoseStamped> target_poses);

  void setXShift(double xshift) {xshift_ = xshift;}
  void setYShift(double yshift) {yshift_ = yshift;}

  /** @brief If true, failures along the path cause the entire path to be rejected.
   *
   * Default is true. */
  void setStopOnFailure(bool stop_on_failure) {stop_on_failure_ = stop_on_failure;}

  /**
   * propagate distances 传播距离
   */
  bool prepare();

  double scoreTrajectory(Trajectory &traj);

  /**
   * return a value that indicates cell is in obstacle
   */
  double obstacleCosts() {//返回一个值，表示单元格处于障碍物中，什么意思？
    return map_.obstacleCosts();
  }

  /**
   * returns a value indicating cell was not reached by wavefront
   * propagation of set cells. (is behind walls, regarding the region covered by grid)//返回一个值，指示单元格未通过设置单元格的波前传播到达。在墙后，关于网格覆盖的区域
   */
  double unreachableCellCosts() {//这是什么意思？无法到达的代价
    return map_.unreachableCellCosts();
  }

  // used for easier debugging用于更容易调试
  double getCellCosts(unsigned int cx, unsigned int cy);

private:
  std::vector<geometry_msgs::PoseStamped> target_poses_;
  costmap_2d::Costmap2D* costmap_;

  base_local_planner::MapGrid map_;//它维护了一个MapGrid，MapGridCostFunction建立后随时知道地图上一个点到全局规划轨迹的距离，或者是到目标的距离。
  CostAggregationType aggregationType_;
  /// xshift and yshift allow scoring for different
  // ooints of robots than center, like fron or back
  // this can help with alignment or keeping specific
  // wheels on tracks both default to 0 :xshift 和 yshift 允许对机器人的不同点而不是中心进行评分，例如前向或后向，这有助于对齐或保持轨道上的特定轮子均默认为 0
  double xshift_;
  double yshift_;
  // if true, we look for a suitable local goal on path, else we use the full path for costs  :如果为真，我们在路径上寻找合适的局部目标，否则我们使用完整路径作为成本,什么意思？
  bool is_local_goal_function_;
  bool stop_on_failure_;
};

} /* namespace base_local_planner */
#endif /* MAP_GRID_COST_FUNCTION_H_ */
