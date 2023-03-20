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

#ifndef SIMPLE_SCORED_SAMPLING_PLANNER_H_
#define SIMPLE_SCORED_SAMPLING_PLANNER_H_

#include <vector>
#include <base_local_planner/trajectory.h>
#include <base_local_planner/trajectory_cost_function.h>
#include <base_local_planner/trajectory_sample_generator.h>
#include <base_local_planner/trajectory_search.h>

namespace base_local_planner {

/**
 * @class SimpleScoredSamplingPlanner
 * @brief Generates a local plan using the given generator and cost functions.
 * Assumes less cost are best, and negative costs indicate infinite costs
 *
 * This is supposed to be a simple and robust implementation of
 * the TrajectorySearch interface. More efficient search may well be
 * possible using search heuristics, parallel search, etc. 这应该是 TrajectorySearch 接口的简单而强大的实现。 使用搜索启发式、并行搜索等可能会更有效地进行搜索。
 */
class SimpleScoredSamplingPlanner : public base_local_planner::TrajectorySearch {//选出最好的轨迹
public:

  ~SimpleScoredSamplingPlanner() {}

  SimpleScoredSamplingPlanner() {}

  /**
   * Takes a list of generators and critics. Critics return costs > 0, or negative costs for invalid trajectories.
   * Generators other than the first are fallback generators,  meaning they only get to generate if the previous
   * generator did not find a valid trajectory.
   * Will use every generator until it stops returning trajectories or count reaches max_samples.
   * Then resets count and tries for the next in the list.
   * passing max_samples = -1 (default): Each Sampling planner will continue to call
   * generator until generator runs out of samples (or forever if that never happens)
   * 获取生成器和评论家的列表。 评论家返回成本 > 0，或无效轨迹的负成本。 
   * 除了第一个生成器之外的其他生成器都是回退生成器，这意味着它们只有在前一个生成器没有找到有效轨迹时才会生成。 将使用每个生成器，直到它停止返回轨迹或计数达到 max_samples。 然后重置计数并尝试列表中的下一个。
  传递 max_samples = -1（默认）：每个抽样计划器将继续调用生成器，直到生成器用完样本（如果从未发生，则永远调用）
   */
  SimpleScoredSamplingPlanner(std::vector<TrajectorySampleGenerator*> gen_list, std::vector<TrajectoryCostFunction*>& critics, int max_samples = -1);

  /**
   * runs all scoring functions over the trajectory creating a weigthed sum
   * of positive costs, aborting as soon as a negative cost are found or costs greater
   * than positive best_traj_cost accumulated  在轨迹上运行所有评分函数，创建正成本的加权和，一旦发现负成本或成本大于累积的正 best_traj 成本就中止
   */
  double scoreTrajectory(Trajectory& traj, double best_traj_cost);//traj是上面generateTrajectory算出的某个轨迹，best_traj_cost指示之前已比较出的最优轨迹得分。对于得分和轨迹优劣的对应关系，1）得分>=0时，得分越低轨迹越优。2）是负数，意味着该轨迹是不合理的，表示的是错误代码，而不是得分。

  /**
   * Calls generator until generator has no more samples or max_samples is reached.
   * For each generated traj, calls critics in turn. If any critic returns negative
   * value, that value is assumed as costs, else the costs are the sum of all critics
   * result. Returns true and sets the traj parameter to the first trajectory with
   * minimal non-negative costs if sampling yields trajectories with non-negative costs,
   * else returns false.
   *调用生成器，直到生成器没有更多样本或达到最大样本数。
*对于每一个生成的traj，依次调用批评者。如果任何评论家的回答是否定的
*价值，该价值被假设为成本，否则成本是所有批评者的总和
*结果。返回true，并使用
*如果采样产生具有非负成本的轨迹，则非负成本最小，
*else返回false。
   *
   * @param traj The container to write the result to
   * @param all_explored pass NULL or a container to collect all trajectories for debugging (has a penalty)
   *
*@param traj要将结果写入的容器
*@param all\u探索传递NULL或容器以收集所有轨迹以进行调试（有惩罚）
   */
  bool findBestTrajectory(Trajectory& traj, std::vector<Trajectory>* all_explored = 0);


private:
  std::vector<TrajectorySampleGenerator*> gen_list_;//轨迹生成器
  std::vector<TrajectoryCostFunction*> critics_;//打分器
  // 产生一系列轨迹，然后用一系列costFunction打分加起来，选最好的那个。这里的轨迹生成器虽然是一个list，但是其实在dwa_local_planner就放了一个进去。costFunction放了6个。

  int max_samples_;
};




} // namespace

#endif /* SIMPLE_SCORED_SAMPLING_PLANNER_H_ */
