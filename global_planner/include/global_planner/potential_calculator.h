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
#ifndef _POTENTIAL_CALCULATOR_H
#define _POTENTIAL_CALCULATOR_H
namespace global_planner {
//这个是默认的潜在计算代价，默认就是如果传入 -1 ，那就是cost + 四邻域最小潜在代价 ，如果传入 大于0，就是cost加上 prev_potential代价，计算起点到当前点n的potential(成本)
class PotentialCalculator {//计算起点到当前点n的potential(成本)   PotentialCalculator类主要的函数就是calculatePotential，返回当前点四周最小的代价值及累加前面累加的代价值。
    public:
        PotentialCalculator(int nx, int ny) {
            setSize(nx, ny);
        }
        //计算方式  这里主要是一个cell的潜在代价，计算方式是通过计算该cell的4邻域代价的最小值，然后通过cost+该点潜在代价   calculatePotential用于计算起始点s到点n的成本，即g(n)。
        virtual float calculatePotential(float* potential, unsigned char cost, int n, float prev_potential=-1){//核心函数，返回当前点四周最小的代价值及累加前面累加的代价值
            if(prev_potential < 0){//prev_potential<0，这值往往就是-1。此时调用者告知calculatePotential，我不知道g(n-1)，即prev_potential，你内部算出一个g(n-1)。如何计算g(n-1)？——PotentialCalculator::calculatePotential的方法是取出该栅格前、后、左、右最小成本的栅格，把它的成本作为做为起点到栅格n-1的g(n-1)
                // get min of neighbors
                // 分别求出前、后、左、右最小的代价值
                float min_h = std::min( potential[n - 1], potential[n + 1] ),
                      min_v = std::min( potential[n - nx_], potential[n + nx_]);
                prev_potential = std::min(min_h, min_v);// 从最小值中再取最小，也就是四个最小的
            }

            return prev_potential + cost;// 成本 = 当前已经累计的成本 + 代价地图中栅格n代价  g(n) = g(n-1) + (代价地图中栅格n的代价+neutral_cost_)。
        }

        /**
         * @brief  Sets or resets the size of the map
         * @param nx The x size of the map
         * @param ny The y size of the map
         */
        virtual void setSize(int nx, int ny) {
            nx_ = nx;
            ny_ = ny;
            ns_ = nx * ny;
        } /**< sets or resets the size of the map */

    protected:
        inline int toIndex(int x, int y) {
            return x + nx_ * y;
        }

        int nx_, ny_, ns_; /**< size of grid, in pixels */
};

} //end namespace global_planner
#endif
