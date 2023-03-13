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
#include<global_planner/dijkstra.h>
#include <algorithm>
namespace global_planner {

DijkstraExpansion::DijkstraExpansion(PotentialCalculator* p_calc, int nx, int ny) :
        Expander(p_calc, nx, ny), pending_(NULL), precise_(false) {
    // priority buffers
    buffer1_ = new int[PRIORITYBUFSIZE];//初始化这三个buffer是什么？
    buffer2_ = new int[PRIORITYBUFSIZE];
    buffer3_ = new int[PRIORITYBUFSIZE];

    priorityIncrement_ = 2 * neutral_cost_;
}

DijkstraExpansion::~DijkstraExpansion() {
  delete[] buffer1_;
  delete[] buffer2_;
  delete[] buffer3_;
  if (pending_)
      delete[] pending_;
}

//
// Set/Reset map size
//
void DijkstraExpansion::setSize(int xs, int ys) {////设置或初始化地图的长度
    Expander::setSize(xs, ys);
    if (pending_)
        delete[] pending_;

    pending_ = new bool[ns_];
    memset(pending_, 0, ns_ * sizeof(bool));////初始化未决全部设置为0
}

//
// main propagation function
// Dijkstra method, breadth-first
// runs for a specified number of cycles,
//   or until it runs out of cells to update,
//   or until the Start cell is found (atStart = true)
//

bool DijkstraExpansion::calculatePotentials(unsigned char* costs, double start_x, double start_y, double end_x, double end_y,
                                           it cycles, float* potential) {
    if(costs == nullptr){//note zhijie 为了验证cost异常时为null的测试
        ROS_WARN("the costmap is nullptr");
    }                                        
    cells_visited_ = 0; //设置已经遍历过的栅格为0
    // priority buffers
    threshold_ = lethal_cost_;//阈值设置为致命代价
    currentBuffer_ = buffer1_;//将buffer1的地址传递给当前缓冲区
    currentEnd_ = 0;//当前缓冲区的长度设置为0
    nextBuffer_ = buffer2_; //把第二个缓冲区给下一个缓冲区
    nextEnd_ = 0;
    overBuffer_ = buffer3_;
    overEnd_ = 0;
    memset(pending_, 0, ns_ * sizeof(bool));
    std::fill(potential, potential + ns_, POT_HIGH);//还是先将潜在矩阵初始化为极大值

    // set goal //返回开始点的一维数组下标
    int k = toIndex(start_x, start_y);

    if(precise_) //使用true新的方式 还是 false:老版navfn启发方式 ，默认是false  主要流程是首先初始化，将起始点四周的点加入优先级队列，然后进入主循环
    {
        double dx = start_x - (int)start_x, dy = start_y - (int)start_y;//向下取整 //保留小数点后两位
        dx = floorf(dx * 100 + 0.5) / 100;
        dy = floorf(dy * 100 + 0.5) / 100;//实际就是将dx的小数点后的第三位圆整而已
        potential[k] = neutral_cost_ * 2 * dx * dy; //起始点代价等于  中立代价 *2 * dx * dy  不太明白为啥乘dxdy
        potential[k+1] = neutral_cost_ * 2 * (1-dx)*dy;
        potential[k+nx_] = neutral_cost_*2*dx*(1-dy);
        potential[k+nx_+1] = neutral_cost_*2*(1-dx)*(1-dy);//*/
        //把k附近的点全部压入 current队列
        push_cur(k+2);
        push_cur(k-1);
        push_cur(k+nx_-1);
        push_cur(k+nx_+2);

        push_cur(k-nx_);
        push_cur(k-nx_+1);
        push_cur(k+nx_*2);
        push_cur(k+nx_*2+1);
    }else{
        potential[k] = 0;//当前点的代价设为0
        push_cur(k+1); //把前后左右全部压入current队列
        push_cur(k-1);
        push_cur(k-nx_);
        push_cur(k+nx_);
    }

    int nwv = 0;            // max priority block size  //最大优先级块大小
    int nc = 0;            // number of cells put into priority blocks  //放入优先块的信元数量(放进去的栅格的数量)
    int cycle = 0;        // which cycle we're on

    // set up start cell  //设置开始单元(分明是关闭好吗)//就是目标点
    int startCell = toIndex(end_x, end_y);
    //进行多次循环，除非被打断,或者循环
    for (; cycle < cycles; cycle++) // go for this many cycles, unless interrupted
            {
        // //优先权块为空
        if (currentEnd_ == 0 && nextEnd_ == 0) // priority blocks empty  主循环先判断当前优先级队列是否为空，如果为空则代表没办法扩展了直接退出return false
            return false;

        // stats 统计资料
        nc += currentEnd_;
        if (currentEnd_ > nwv)//最大优先级块大小 不能小于 当前队列大小
            nwv = currentEnd_;

        // reset pending_ flags on current priority buffer  //在当前优先级缓冲区上重置未决标志   //把当前缓冲区指针 给pb
        int *pb = currentBuffer_;
        int i = currentEnd_;
        while (i-- > 0)//把当前队列的未决全部设置为 false
            pending_[*(pb++)] = false;

        // process current priority buffer   在当前优先级缓冲区上重置未决标志
        pb = currentBuffer_; //处理当前优先级缓冲区
        i = currentEnd_;
        while (i-- > 0)//对当前队列每个点调用updatecell函数
            updateCell(costs, potential, *pb++);//调用下边那个更新函数

        // swap priority blocks currentBuffer_ <=> nextBuffer_    //交换优先级块currentBuffer_ <=> nextBuffer_
        currentEnd_ = nextEnd_;
        nextEnd_ = 0;
        pb = currentBuffer_;        // swap buffers
        currentBuffer_ = nextBuffer_;
        nextBuffer_ = pb;

        // see if we're done with this priority level  //查看是否已完成此优先级
        if (currentEnd_ == 0) {
            threshold_ += priorityIncrement_;    // increment priority threshold  //递增优先级阈值
            currentEnd_ = overEnd_;    // set current to overflow block   //将当前设置为溢出块
            overEnd_ = 0;
            pb = currentBuffer_;        // swap buffers
            currentBuffer_ = overBuffer_;
            overBuffer_ = pb;  //交换over 和当前队列
        }

        // check if we've hit the Start cell
        if (potential[startCell] < POT_HIGH)
            break;
    }
    //ROS_INFO("CYCLES %d/%d ", cycle, cycles);
    if (cycle < cycles)
        return true; // finished up here
    else
        return false;
}

//
// Critical function: calculate updated potential value of a cell,  关键函数: 根据邻居的值计算单元更新后的潜在值
//   given its neighbors' values
// Planar-wave update calculation from two lowest neighbors in a 4-grid
// Quadratic approximation to the interpolated value
// No checking of bounds here, this function should be fast
//

#define INVSQRT2 0.707106781

inline void DijkstraExpansion::updateCell(unsigned char* costs, float* potential, int n) {
    cells_visited_++;//已经遍历过的栅格 自加1

    // do planar wave update
    float c = getCost(costs, n);
    if (c >= lethal_cost_)    // don't propagate into obstacles
        return;

    float pot = p_calc_->calculatePotential(potential, c, n);  //pot  =  前后左右最小的potential + 当前的costs 

    // now add affected neighbors to priority blocks  现在将受影响的邻居添加到优先级块
    if (pot < potential[n]) {
        float le = INVSQRT2 * (float)getCost(costs, n - 1);//获得上下左右的 cost值, 但是不明白为啥要乘以二分之根号二
        float re = INVSQRT2 * (float)getCost(costs, n + 1);
        float ue = INVSQRT2 * (float)getCost(costs, n - nx_);
        float de = INVSQRT2 * (float)getCost(costs, n + nx_);
        potential[n] = pot;
        //ROS_INFO("UPDATE %d %d %d %f", n, n%nx, n/nx, potential[n]); //低成本缓冲块,暂时不清楚干啥的
        if (pot < threshold_)    // low-cost buffer block   //如果当前 pot小于阈值
                {
            if (potential[n - 1] > pot + le)
                push_next(n-1);
            if (potential[n + 1] > pot + re)
                push_next(n+1);
            if (potential[n - nx_] > pot + ue)
                push_next(n-nx_);
            if (potential[n + nx_] > pot + de)
                push_next(n+nx_);
        } else            // overflow block  /如果当前点的潜力  大于 潜力-1 + cost*根号二/2 将其push到over的队列
        {
            if (potential[n - 1] > pot + le)
                push_over(n-1);
            if (potential[n + 1] > pot + re)
                push_over(n+1);
            if (potential[n - nx_] > pot + ue)
                push_over(n-nx_);
            if (potential[n + nx_] > pot + de)
                push_over(n+nx_);
        }
    }
}

} //end namespace global_planner
