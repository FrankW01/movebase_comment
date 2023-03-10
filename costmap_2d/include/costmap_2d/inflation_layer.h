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
#ifndef COSTMAP_2D_INFLATION_LAYER_H_
#define COSTMAP_2D_INFLATION_LAYER_H_

#include <ros/ros.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/InflationPluginConfig.h>
#include <dynamic_reconfigure/server.h>
#include <boost/thread.hpp>
#include <queue>

namespace costmap_2d
{
/**
 * @class CellData
 * @brief Storage for cell information used during obstacle inflation  存储障碍物膨胀期间使用的单元格信息
 */
class CellData
{
public:
  /**
   * @brief  Constructor for a CellData objects
   * @param  d The distance to the nearest obstacle, used for ordering in the priority queue   到最近障碍物的距离，用于在优先队列中排序
   * @param  i The index of the cell in the cost map
   * @param  x The x coordinate of the cell in the cost map
   * @param  y The y coordinate of the cell in the cost map
   * @param  sx The x coordinate of the closest obstacle cell in the costmap  成本图中最近的障碍物单元格的 x 坐标
   * @param  sy The y coordinate of the closest obstacle cell in the costmap
   * @return
   */
  CellData(double d, double i, unsigned int x, unsigned int y, unsigned int sx, unsigned int sy) :
      distance_(d), index_(i), x_(x), y_(y), src_x_(sx), src_y_(sy)
  {
  }
  double distance_;
  unsigned int index_;
  unsigned int x_, y_;
  unsigned int src_x_, src_y_;
};

/**
 * @brief Provide an ordering between CellData objects in the priority queue
 * @return We want the lowest distance to have the highest priority... so this returns true if a has higher priority than b  我们希望最短的距离具有最高的优先级...因此如果 a 的优先级高于 b，则返回 true
 */
inline bool operator<(const CellData &a, const CellData &b)
{
  return a.distance_ > b.distance_;
}

class InflationLayer : public Layer//膨胀层按自已规则计算出的cost，这cost值语义和costmap一致，但不会出现NO_INFORMATION(255)，以及肯定>FREE_SPACE(0)
{
public:
  InflationLayer();

  virtual ~InflationLayer()
  {
    deleteKernels();
    if (dsrv_)
        delete dsrv_;
  }

  virtual void onInitialize();
  virtual void updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y,
                            double* max_x, double* max_y);//主要部分
  virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);
  virtual bool isDiscretized()
  {
    return true;
  }
  virtual void matchSize();

  virtual void reset() { onInitialize(); }

  /** @brief  Given a distance, compute a cost.  给定距离，计算成本
   * @param  distance The distance from an obstacle in cells  这个距离单位m
   * @return A cost value for the distance */ //根据距离计算代价
  inline unsigned char computeCost(double distance) const
  {
    unsigned char cost = 0;
    if (distance == 0)//若距离为0，则代价值为LETHAL_OBSTACLE ，表示为障碍物本身
      cost = LETHAL_OBSTACLE;
    else if (distance * resolution_ <= inscribed_radius_)//若距离小于内切圆半径，设置值为INSCRIBED_INFLATED_OBSTACLE，即由于机器人有体积造成的障碍物膨胀
      cost = INSCRIBED_INFLATED_OBSTACLE;
    else
    {
      // make sure cost falls off by Euclidean distance
      double euclidean_distance = distance * resolution_;
      double factor = exp(-1.0 * weight_ * (euclidean_distance - inscribed_radius_));
      cost = (unsigned char)((INSCRIBED_INFLATED_OBSTACLE - 1) * factor);
    }
    return cost;
  }

  /**
   * @brief Change the values of the inflation radius parameters
   * @param inflation_radius The new inflation radius
   * @param cost_scaling_factor The new weight
   */
  void setInflationParameters(double inflation_radius, double cost_scaling_factor);

protected:
  virtual void onFootprintChanged();
  boost::recursive_mutex* inflation_access_;

private:
  /**
   * @brief  Lookup pre-computed distances  查找预先计算的距离
   * @param mx The x coordinate of the current cell
   * @param my The y coordinate of the current cell
   * @param src_x The x coordinate of the source cell
   * @param src_y The y coordinate of the source cell
   * @return
   */
  inline double distanceLookup(int mx, int my, int src_x, int src_y)
  {
    unsigned int dx = abs(mx - src_x);
    unsigned int dy = abs(my - src_y);
    return cached_distances_[dx][dy];//使用距离查找表
  }

  /**
   * @brief  Lookup pre-computed costs
   * @param mx The x coordinate of the current cell
   * @param my The y coordinate of the current cell
   * @param src_x The x coordinate of the source cell
   * @param src_y The y coordinate of the source cell
   * @return
   */
  inline unsigned char costLookup(int mx, int my, int src_x, int src_y)//使用代价查找表
  {
    unsigned int dx = abs(mx - src_x);
    unsigned int dy = abs(my - src_y);
    return cached_costs_[dx][dy];
  }

  void computeCaches();
  void deleteKernels();
  void inflate_area(int min_i, int min_j, int max_i, int max_j, unsigned char* master_grid);

  unsigned int cellDistance(double world_dist)
  {
    return layered_costmap_->getCostmap()->cellDistance(world_dist);
  }

  inline void enqueue(unsigned int index, unsigned int mx, unsigned int my,
                      unsigned int src_x, unsigned int src_y);

  double inflation_radius_, inscribed_radius_, weight_;
  unsigned int cell_inflation_radius_;
  unsigned int cached_cell_inflation_radius_;
  std::priority_queue<CellData> inflation_queue_;

  double resolution_;

  bool* seen_;//这个矩阵用于是否cell被访问过
  int seen_size_;

  unsigned char** cached_costs_;//代价缓存
  double** cached_distances_;//距离缓存
  double last_min_x_, last_min_y_, last_max_x_, last_max_y_;

  dynamic_reconfigure::Server<costmap_2d::InflationPluginConfig> *dsrv_;//动态参数配置
  void reconfigureCB(costmap_2d::InflationPluginConfig &config, uint32_t level);

  bool need_reinflation_;  ///< Indicates that the entire costmap should be reinflated next time around.
};

}  // namespace costmap_2d

#endif  // COSTMAP_2D_INFLATION_LAYER_H_
