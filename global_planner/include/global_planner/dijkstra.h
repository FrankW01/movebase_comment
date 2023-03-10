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
#ifndef _DIJKSTRA_H
#define _DIJKSTRA_H
//优先 大小
#define PRIORITYBUFSIZE 10000 //来宏定义了优先级队列的大小为10000
#include <math.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>

#include <global_planner/planner_core.h>
#include <global_planner/expander.h>
//下边三个宏函数 分别为//一、在current buffer 添加当前点 //二、在next buffer 添加当前点  //三、在end buffer添加当前点，也就是把当前点压入到 end队列里面
// inserting onto the priority blocks  //然后是宏定义往三个队列里面push点，分别有当前队列、下一个队列、关闭队列
#define push_cur(n)  { if (n>=0 && n<ns_ && !pending_[n] && getCost(costs, n)<lethal_cost_ && currentEnd_<PRIORITYBUFSIZE){ currentBuffer_[currentEnd_++]=n; pending_[n]=true; }}
#define push_next(n) { if (n>=0 && n<ns_ && !pending_[n] && getCost(costs, n)<lethal_cost_ &&    nextEnd_<PRIORITYBUFSIZE){    nextBuffer_[   nextEnd_++]=n; pending_[n]=true; }}
#define push_over(n) { if (n>=0 && n<ns_ && !pending_[n] && getCost(costs, n)<lethal_cost_ &&    overEnd_<PRIORITYBUFSIZE){    overBuffer_[   overEnd_++]=n; pending_[n]=true; }}

namespace global_planner {
class DijkstraExpansion : public Expander {
    public:
        DijkstraExpansion(PotentialCalculator* p_calc, int nx, int ny);
        ~DijkstraExpansion();
        bool calculatePotentials(unsigned char* costs, double start_x, double start_y, double end_x, double end_y, int cycles,
                                float* potential);//启发函数，实现的接口函数

        /**
         * @brief  Sets or resets the size of the map
         * @param nx The x size of the map
         * @param ny The y size of the map
         */
        void setSize(int nx, int ny); /**< sets or resets the size of the map */ //设置地图大小

        void setNeutralCost(unsigned char neutral_cost) {//设置中性成本
            neutral_cost_ = neutral_cost;
            priorityIncrement_ = 2 * neutral_cost_;
        }

        void setPreciseStart(bool precise){ precise_ = precise; }
    private:

        /**
         * @brief  Updates the cell at index n
         * @param costs The costmap
         * @param potential The potential array in which we are calculating
         * @param n The index to update
         */
        void updateCell(unsigned char* costs, float* potential, int n); /** updates the cell at index n */  //被接口函数所调用  更新索引为n的单元格updateCell
         //获取某点的代价
        float getCost(unsigned char* costs, int n) {//获取某点代价值getCost
            float c = costs[n];
            if (c < lethal_cost_ - 1 || (unknown_ && c==255)) {//如果改点代价 小于 致命代价-1, 或者 未知参数为1, 并且 该点cost等于255
                c = c * factor_ + neutral_cost_;  //代价等于  当前代价 * 因数 + 中立代价，这个factor = 3 是不是有点大？
                if (c >= lethal_cost_)  //如果结果大于致命代价,则该点等于致命代价-1
                    c = lethal_cost_ - 1;
                return c;
            }
            return lethal_cost_; //返回致命代价
        }

        /** block priority buffers *///块优先级缓冲区
        int *buffer1_, *buffer2_, *buffer3_; /**< storage buffers for priority blocks *///优先级块的存储缓冲区
        int *currentBuffer_, *nextBuffer_, *overBuffer_; /**< priority buffer block ptrs */
        int currentEnd_, nextEnd_, overEnd_; /**< end points of arrays */
        bool *pending_; /**< pending_ cells during propagation */ //传播过程中的未决细胞(估计是没有启发过的点)
        bool precise_;

        /** block priority thresholds */ //终止扩展代价的临界值，刚开始为致命代价
        float threshold_; /**< current threshold *///如果搜索完毕未找到目标点则增大该临界值继续扩展直到扩展到目标点
        float priorityIncrement_; /**< priority threshold increment */  //优先级阈值增量

};
} //end namespace global_planner
#endif
