ROS Navigation Stack
====================

start from 6.12  
https://zhuanlan.zhihu.com/ros-nav  
Want to nav using skecthed or emergency map(not accuracy map) originally, 
but at last it's deprecated for some reasons...  
Now it's just a repo for record.  

### sketched map两篇论文的思路：  
*   修改下粒子滤波的过程，将pose(x,y,theta)多加两个  
    扩展为pose + (a,b)，也就是局部拉伸变形  
    里程计的距离也拉伸下  

论文的问题：  
*   quick change in the scale，因为scale不像pose一样  
    pose有测量保证单步之后是个高斯分布  
    scale也学人家pose，认为是原来的scale发散下  
    那变化快了localization丢了  
    
[report PDF](https://github.com/meiqua/navigation/raw/kinetic-devel/report.pdf)


这个没趣啊的代码
version:navigation-kinetic-devel
这个版本是合并了之前看dwa代码与move_base代码的

