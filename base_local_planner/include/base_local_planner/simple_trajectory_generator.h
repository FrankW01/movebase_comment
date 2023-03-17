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

#ifndef SIMPLE_TRAJECTORY_GENERATOR_H_
#define SIMPLE_TRAJECTORY_GENERATOR_H_

#include <base_local_planner/trajectory_sample_generator.h>
#include <base_local_planner/local_planner_limits.h>
#include <Eigen/Core>

namespace base_local_planner {

/**
 * generates trajectories based on equi-distant discretisation of the degrees of freedom.
 * This is supposed to be a simple and robust implementation of the TrajectorySampleGenerator
 * interface, more efficient implementations are thinkable.
 *
 * This can be used for both dwa and trajectory rollout approaches.
 * As an example, assuming these values:
 * sim_time = 1s, sim_period=200ms, dt = 200ms,
 * vsamples_x=5,
 * acc_limit_x = 1m/s^2, vel_x=0 (robot at rest, values just for easy calculations)
 * dwa_planner will sample max-x-velocities from 0m/s to 0.2m/s.
 * trajectory rollout approach will sample max-x-velocities 0m/s up to 1m/s
 * trajectory rollout approach does so respecting the acceleration limit, so it gradually increases velocity
 */
 /*
 基于自由度的等距离散化生成轨迹。 这应该是 TrajectorySampleGenerator 接口的简单而强大的实现，更有效的实现是可以想象的。这可用于 dwa 和轨迹推出方法。 
 例如，假设这些值：sim_time = 1s, sim_period=200ms, dt = 200ms, vsamples_x=5, acc_limit_x = 1m/s^2, vel_x=0（机器人静止，值只是为了便于计算）
 dwa_planner 将采样最大值 -x-速度从 0m/s 到 0.2m/s。 trajectory rollout approach 将采样 max-x-velocity 0m/s 到 1m/s*/
class SimpleTrajectoryGenerator: public base_local_planner::TrajectorySampleGenerator {//继承了TrajectorySampleGenerator类
// 以供后面打分挑选。比如当前速度是5，允许的速度范围是0-10，那么这个类会产生10个目标速度值，
// 然后根据加速度限制将当前速度尽量加速或减速到目标值，持续一定的步长，并记录生成的所有轨迹。可见这个轨迹生成类也算是人如其名，too simple。
public:

  SimpleTrajectoryGenerator() {
    limits_ = NULL;
  }

  ~SimpleTrajectoryGenerator() {}

  /**
   * @param pos current robot position
   * @param vel current robot velocity
   * @param limits Current velocity limits
   * @param vsamples: in how many samples to divide the given dimension
   * @param use_acceleration_limits: if true use physical model, else idealized robot model
   * @param additional_samples (deprecated): Additional velocity samples to generate individual trajectories from.
   * @param discretize_by_time if true, the trajectory is split according in chunks of the same duration, else of same length
   */
  void initialise(
      const Eigen::Vector3f& pos,
      const Eigen::Vector3f& vel,
      const Eigen::Vector3f& goal,
      base_local_planner::LocalPlannerLimits* limits,
      const Eigen::Vector3f& vsamples,
      std::vector<Eigen::Vector3f> additional_samples,
      bool discretize_by_time = false);

  /**
   * @param pos current robot position
   * @param vel current robot velocity
   * @param limits Current velocity limits
   * @param vsamples: in how many samples to divide the given dimension
   * @param use_acceleration_limits: if true use physical model, else idealized robot model  如果 true 使用物理模型，否则理想化机器人模型
   * @param discretize_by_time if true, the trajectory is split according in chunks of the same duration, else of same length如果为真，轨迹将根据相同持续时间的块进行分割，否则相同长度
   */
  void initialise(
      const Eigen::Vector3f& pos,
      const Eigen::Vector3f& vel,
      const Eigen::Vector3f& goal,
      base_local_planner::LocalPlannerLimits* limits,
      const Eigen::Vector3f& vsamples,
      bool discretize_by_time = false);

  /**
   * This function is to be called only when parameters change
   *
   * @param sim_granularity granularity of collision detection
   * @param angular_sim_granularity angular granularity of collision detection
   * @param use_dwa whether to use DWA or trajectory rollout 是否使用 DWA 或 trajectory rollout
   * @param sim_period distance between points in one trajectory
   */
  void setParameters(double sim_time,
      double sim_granularity,
      double angular_sim_granularity,
      bool use_dwa = false,
      double sim_period = 0.0);

  /**
   * Whether this generator can create more trajectories
   */
  bool hasMoreTrajectories();

  /**
   * Whether this generator can create more trajectories
   */
  bool nextTrajectory(Trajectory &traj);


  static Eigen::Vector3f computeNewPositions(const Eigen::Vector3f& pos,
      const Eigen::Vector3f& vel, double dt);

  static Eigen::Vector3f computeNewVelocities(const Eigen::Vector3f& sample_target_vel,
      const Eigen::Vector3f& vel, Eigen::Vector3f acclimits, double dt);

  bool generateTrajectory(
        Eigen::Vector3f pos,
        Eigen::Vector3f vel,
        Eigen::Vector3f sample_target_vel,
        base_local_planner::Trajectory& traj);//轨迹就是速度跟点的集合，在trajectory.h中定义

protected:

  unsigned int next_sample_index_;
  // to store sample params of each sample between init and generation
  std::vector<Eigen::Vector3f> sample_params_;
  base_local_planner::LocalPlannerLimits* limits_;
  Eigen::Vector3f pos_;
  Eigen::Vector3f vel_;

  // whether velocity of trajectory changes over time or not
  bool continued_acceleration_;
  bool discretize_by_time_;

  double sim_time_, sim_granularity_, angular_sim_granularity_;
  bool use_dwa_;//现在use_dwa 为true
  double sim_period_; // only for dwa
};

} /* namespace base_local_planner */
#endif /* SIMPLE_TRAJECTORY_GENERATOR_H_ */
