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
#include<global_planner/astar.h>
#include<costmap_2d/cost_values.h>

namespace global_planner {

AStarExpansion::AStarExpansion(PotentialCalculator* p_calc, int xs, int ys) :
        Expander(p_calc, xs, ys) {
}

bool AStarExpansion::calculatePotentials(unsigned char* costs, double start_x, double start_y, double end_x, double end_y,
                                        int cycles, float* potential) {//A* 的计算所有点的可能性，这个costs应该是全局代价地图，这里引申出一个问题，全局代价地图不是设置了膨胀层了吗，膨胀层不是根据内切半径设置的吗，实际上全局规划出的路径实际上肯定在内切半径之外吧，所有途径点也是一定会在膨胀半径之外吧
    queue_.clear();//这个队列保存这index代价队列
    int start_i = toIndex(start_x, start_y);
    queue_.push_back(Index(start_i, 0));//0是index这个点的代价，//1.将起点放入队列  起点的代价为0

    std::fill(potential, potential + ns_, POT_HIGH);          //2.将所有点的potential都设为一个极大值  先将所有点的潜在代价设置为极大值
    potential[start_i] = 0;                                   //3.将起点的potential设为0，所以呈现灰色

    int goal_i = toIndex(end_x, end_y);
    int cycle = 0;

    while (queue_.size() > 0 && cycle < cycles) {             //4.进入循环，继续循环的判断条件为只要队列大小大于0且循环次数小于所有格子数的2倍，这个cycles已经设置为所有格子数目的两倍
        Index top = queue_[0];//min                           //5.得到最小cost的索引，并删除它，如果索引指向goal(目的地)则退出算法，返回true
        std::pop_heap(queue_.begin(), queue_.end(), greater1());//将队列排序，pop_heap将堆顶(所给范围的最前面)元素移动到所给范围的最后，并且将新的最大值置于所给范围的最前面
        queue_.pop_back();//remove the Index with mini cost

        int i = top.i;//the Index's i from (i,cost)
        if (i == goal_i)//stop condition，找到了终点则返回
            return true;
                                                              //6.对前后左右四个点执行add函数//add the neighborhood 4 points into the search scope
        add(costs, potential, potential[i], i + 1, end_x, end_y);//右边点
        add(costs, potential, potential[i], i - 1, end_x, end_y);//左边点
        add(costs, potential, potential[i], i + nx_, end_x, end_y);//下边点
        add(costs, potential, potential[i], i - nx_, end_x, end_y);//上边点

        cycle++;//循环的次数为什么要小于所有格子数的2倍，可以证明这个完备性吗？
    }

    return false;
}
//add函数中，如果是已经添加的的点则忽略，根据costmap的值如果是障碍物的点也忽略。
void AStarExpansion::add(unsigned char* costs, float* potential, float prev_potential, int next_i, int end_x,
                         int end_y) {//next_i是一个index
    if (next_i < 0 || next_i >= ns_)//如果下一个点超出地图总数，就直接返回
        return;

    if (potential[next_i] < POT_HIGH)//如果下一个点被访问过，也返回
        return;

    if(costs[next_i]>=lethal_cost_ && !(unknown_ && costs[next_i]==costmap_2d::NO_INFORMATION))//如果超过设定致命代价 并且  这个cost已经确定是有障碍物的情况下要返回（因为没有信息的代价是255 最大，要排除这种情况）
        return;
    //AStarExpansion::calculatePotentials()计算每一个点的potential时，用的是p_calc_实例！！//如果使用默认的代价计算，即不使用二次代价插值 g(x) = 当前代价地图代价+ 一个中性代价+加上之前的潜在代价// compute the next_i cell in potential     // costs[next_i] + neutral_cost_:original cost + extra cost
    potential[next_i] = p_calc_->calculatePotential(potential, costs[next_i] + neutral_cost_, next_i, prev_potential); //potential[next_i]：    是起点到当前点的cost即g(n)  这个传入的第二项相当于 一步的代价，最后一项为g(n-1)的代价
    int x = next_i % nx_, y = next_i / nx_;//转化为地图坐标 
    float distance = abs(end_x - x) + abs(end_y - y);//范数1距离，这个就是启发距离h(x)，使用的是1范数距离 * neutral_cost_ 

    queue_.push_back(Index(next_i, potential[next_i] + distance * neutral_cost_));    //distance * neutral_cost_：是当前点到目的点的cost即h(n)。f(n)=g(n)+h(n)：  计算完这两个cost后，加起来即为f(n)，将其存入队列中。
    std::push_heap(queue_.begin(), queue_.end(), greater1());//将最大值代价移动代最前面？ //sort about the previous insert element

} //end namespace global_planner
