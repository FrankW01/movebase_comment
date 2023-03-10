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
#ifndef _EXPANDER_H
#define _EXPANDER_H
#include <global_planner/potential_calculator.h>
#include <global_planner/planner_core.h>

namespace global_planner {

class Expander {//expander规定了calculatePotentials的接口，这个主要是计算潜在矩阵的接口，有了潜在代价矩阵，再根据梯度下降的方法来确定最优路径
    public:
        Expander(PotentialCalculator* p_calc, int nx, int ny) :
                unknown_(true), lethal_cost_(253), neutral_cost_(50), factor_(3.0), p_calc_(p_calc) {//这个因子没有用到
            setSize(nx, ny);
        }
        virtual bool calculatePotentials(unsigned char* costs, double start_x, double start_y, double end_x, double end_y,
                                        int cycles, float* potential) = 0;

        /**
         * @brief  Sets or resets the size of the map
         * @param nx The x size of the map
         * @param ny The y size of the map
         *///设置或者重置地图大小x,y
        virtual void setSize(int nx, int ny) {
            nx_ = nx;
            ny_ = ny;
            ns_ = nx * ny;
        } /**< sets or resets the size of the map */   //设置致命代价大小
        void setLethalCost(unsigned char lethal_cost) {
            lethal_cost_ = lethal_cost;
        }
        void setNeutralCost(unsigned char neutral_cost) { //设置中立代价大小
            neutral_cost_ = neutral_cost;
        }
        void setFactor(float factor) {//设置因子大小
            factor_ = factor;
        }
        void setHasUnknown(bool unknown) {
            unknown_ = unknown;
        }
        // 该类主要有清空端点、设置地图大小还有最重要的启发函数的接口* 这个函数以目标点（Potential值已初始化为0）为起点，向整张地图的cell传播，填充potarr数组，直到找到起始点为止，potarr数组的数据能够反映“走了多远”和“附近的障碍情况”，为最后的路径计算提供了依据。
        void clearEndpoint(unsigned char* costs, float* potential, int gx, int gy, int s){//调用这里s = 2  这里 gx与gy都是cell  这个costs是代价地图吗？
            int startCell = toIndex(gx, gy);//这个goal的index坐标
            for(int i=-s;i<=s;i++){
            for(int j=-s;j<=s;j++){
                int n = startCell+i+nx_*j;
                if(potential[n]<POT_HIGH)//如果该点有潜在代价，则跳过   //POT_HIGH 是 未分配的栅格值
                    continue;
                float c = costs[n]+neutral_cost_;//这个neutral_cost的是什么呢，这个是只，如果该点没有被访问过的时候，给它一个初始假设代价 netutral_cost
                float pot = p_calc_->calculatePotential(potential, c, n);//第四个参数默认是-1
                potential[n] = pot;//这个potential是不是可以超过255？
            }//potarr数组的数据能够反映“走了多远”和“附近的障碍情况”，为最后的路径计算提供了依据。
            }
        }//cost大小与cell离障碍物的远近对应，更大的cost对应更大的Potential，并且障碍物点不更新Potential，使得其值停留在无限大，故Potential值的大小也能反映点与障碍物的接近程度。

    protected:
        inline int toIndex(int x, int y) {
            return x + nx_ * y;
        }

        int nx_, ny_, ns_; /**< size of grid, in pixels */ //地图的长宽和面积
        bool unknown_;
        unsigned char lethal_cost_, neutral_cost_; //中立/中性 代价
        int cells_visited_;  //已经遍历过的栅格
        float factor_;//因子默认是3.0
        PotentialCalculator* p_calc_;//这个什么接口呢

};

} //end namespace global_planner
#endif
