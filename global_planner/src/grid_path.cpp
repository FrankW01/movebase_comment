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
#include <global_planner/grid_path.h>
#include <algorithm>
#include <stdio.h>
namespace global_planner {
//这个模块负责使用计算好的potential矩阵生成一条全局规划路径。从成本矩阵提取出路径：GridPath算法,取一个点八
bool GridPath::getPath(float* potential, double start_x, double start_y, double end_x, double end_y, std::vector<std::pair<float, float> >& path) {//potential 潜在矩阵
    std::pair<float, float> current;//从终点开始进行搜索
    current.first = end_x;
    current.second = end_y;
                                         //1.将goal（目的地）所在的点的(x,y)作为当前点加入path,从终点附近开始搜索
    int start_index = getIndex(start_x, start_y);//start_x 与 start_y 是map坐标，cell形式

    path.push_back(current);//将终点放入path中
    int c = 0;
    int ns = xs_ * ys_;
                                          //2.进入循环，继续循环的条件为当前点的索引不是起始点
    while (getIndex(current.first, current.second) != start_index) {
        float min_val = 1e10;
        int min_x = 0, min_y = 0;
        for (int xd = -1; xd <= 1; xd++) {//3.搜索当前点的四周的八个临近点，选取这八个临近点的potential的值最小的点，，选取这四个临近点的potential的值最小的点min，然后把这点塞到path
            for (int yd = -1; yd <= 1; yd++) {
                if (xd == 0 && yd == 0)//如果是当前点，跳过
                    continue;
                int x = current.first + xd, y = current.second + yd;
                int index = getIndex(x, y);
                if (potential[index] < min_val) {
                    min_val = potential[index];//然后再将当前点更新为min这点，由于start 点的potential的值是0，所以这是个梯度下降的方法。
                    min_x = x;
                    min_y = y;
                }
            }
        }
        if (min_x == 0 && min_y == 0)//这个条件分支是什么逻辑?
            return false;
        current.first = min_x;           //4.将potential值最小的点更改为当前点，并加入path
        current.second = min_y;
        path.push_back(current);
        
        if(c++>ns*4){//这个偶尔会出现有问题，这个如果超过cell的总数的4倍就会报错
            return false;
        }

    }//返回2，继续循环
    return true;
}

} //end namespace global_planner
/*这个模块负责使用计算好的potential矩阵生成一条全局规划路径。算法很好理解，首先将goal所在的点的(x, y)塞到path，
然后搜索当前的点的四周的八个临近点，选取这八个临近点的potential的值最小的点min，然后把这点塞到path，
然后再将当前点更新为min这点，由于start点的potential的值是0，所以这是个梯度下降的方法。*/

