/*
 *  Player - One Hell of a Robot Server
 *  Copyright (C) 2000  Brian Gerkey   &  Kasper Stoy
 *                      gerkey@usc.edu    kaspers@robotics.usc.edu
 *
 *  This library is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 2.1 of the License, or (at your option) any later version.
 *
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this library; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */
/**************************************************************************
 * Desc: KD tree functions
 * Author: Andrew Howard
 * Date: 18 Dec 2002
 * CVS: $Id: pf_kdtree.h 6532 2008-06-11 02:45:56Z gbiggs $
 *************************************************************************/

#ifndef PF_KDTREE_H
#define PF_KDTREE_H

#ifdef INCLUDE_RTKGUI
#include "rtk.h"
#endif


// Info for a node in the tree
typedef struct pf_kdtree_node
{
  // Depth in the tree 是叶子（0 1）吗 有多深？（tree的基本概念）
  int leaf, depth;

  // Pivot dimension and value 比较哪一维（0 1 2）？ 比较值选的是平均值
  int pivot_dim;
  double pivot_value;

  // The key for this node 就是pose啦，只不过等比例放大了点，都存成int
  int key[3];

  // The value for this node  就是权重啦(weight)，pose看到的地图符合观测数据就大，权重！！！
  double value;

  // The cluster label (leaf nodes)  想象pose(x,y,)为一个三维的点，挨着的pose就是一个集群(cluster),具体什么意思呢？
  int cluster;

  // Child nodes tree的基本概念
  struct pf_kdtree_node *children[2];

} pf_kdtree_node_t; //kdtree的基本单元是node


// A kd tree
typedef struct
{
  // Cell size  就是放大pose的比例啦，默认是0.5 0.5 pi/180*10，除一下就放大了，这个不应该设置成static吗
  double size[3];

  // The root node of the tree  tree的基本概念，根节点
  pf_kdtree_node_t *root;

  // The number of nodes in the tree
  int node_count, node_max_count;
  pf_kdtree_node_t *nodes;//kdtree在一开始建立的时候会malloc个内存块存node,nodes指向开头

  // The number of leaf nodes in the tree  leaf_count,叶子数量
  int leaf_count;

} pf_kdtree_t;


// Create a tree
extern pf_kdtree_t *pf_kdtree_alloc(int max_size);

// Destroy a tree
extern void pf_kdtree_free(pf_kdtree_t *self);

// Clear all entries from the tree
extern void pf_kdtree_clear(pf_kdtree_t *self);

// Insert a pose into the tree 里面会进一步调用pf_kdtree_insert_node  将一个位置插入树中
extern void pf_kdtree_insert(pf_kdtree_t *self, pf_vector_t pose, double value);

// Cluster the leaves in the tree
extern void pf_kdtree_cluster(pf_kdtree_t *self);

// Determine the probability estimate for the given pose
extern double pf_kdtree_get_prob(pf_kdtree_t *self, pf_vector_t pose);

// Determine the cluster label for the given pose
extern int pf_kdtree_get_cluster(pf_kdtree_t *self, pf_vector_t pose);


#ifdef INCLUDE_RTKGUI

// Draw the tree
extern void pf_kdtree_draw(pf_kdtree_t *self, rtk_fig_t *fig);

#endif

#endif
