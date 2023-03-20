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

#ifndef TRAJECTORYCOSTFUNCTION_H_
#define TRAJECTORYCOSTFUNCTION_H_

#include <base_local_planner/trajectory.h>
//轨迹打分cost的基类
namespace base_local_planner {

/**
 * @class TrajectoryCostFunction
 * @brief Provides an interface for critics of trajectories
 * During each sampling run, a batch of many trajectories will be scored using such a cost function.
 * The prepare method is called before each batch run, and then for each
 * trajectory of the sampling set, score_trajectory may be called.
 *@brief为轨迹批评者提供了一个界面
*在每次采样过程中，将使用此类成本函数对一批多个轨迹进行评分。
*在每次批处理运行之前调用prepare方法，然后针对每个批处理运行调用prepare方法
*采样集的轨迹，score_轨迹可以称为。
 */
class TrajectoryCostFunction {//对轨迹进行打分 轨迹代价的抽象类，TrajectoryCostFunction可是说是最重要的一个接口。
public:

  /**
   *
   * General updating of context values if required.
   * Subclasses may overwrite. Return false in case there is any error. 如果需要，一般更新上下文值。 子类可以覆盖。 如果有任何错误，则返回 false。
   */
  virtual bool prepare() = 0;//更新准则的上下文

  /**
   * return a score for trajectory traj
   */
  virtual double scoreTrajectory(Trajectory &traj) = 0;//主要规定了一个scoreTrajectory函数，也就是走过一个轨迹需要付出的代价，这个每个具体代价的必须实现的接口API，对候选轨迹打分

  double getScale() {
    return scale_;
  }

  void setScale(double scale) {
    scale_ = scale;
  }

  virtual ~TrajectoryCostFunction() {}

protected:
  TrajectoryCostFunction(double scale = 1.0): scale_(scale) {}

private:
  double scale_;//私有成员scale_，用于指示该准则得分权重，它不是个虚函数，如果不给专门设置，权重默认1.0。
};

}

#endif /* TRAJECTORYCOSTFUNCTION_H_ */
