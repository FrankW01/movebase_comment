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
 * Desc: kd-tree functions
 * Author: Andrew Howard
 * Date: 18 Dec 2002
 * CVS: $Id: pf_kdtree.c 7057 2008-10-02 00:44:06Z gbiggs $
 *************************************************************************/

#include <assert.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>


#include "pf_vector.h"
#include "pf_kdtree.h"


// Compare keys to see if they are equal
static int pf_kdtree_equal(pf_kdtree_t *self, int key_a[], int key_b[]);

// Insert a node into the tree
static pf_kdtree_node_t *pf_kdtree_insert_node(pf_kdtree_t *self, pf_kdtree_node_t *parent,
                                               pf_kdtree_node_t *node, int key[], double value);

// Recursive node search
static pf_kdtree_node_t *pf_kdtree_find_node(pf_kdtree_t *self, pf_kdtree_node_t *node, int key[]);

// Recursively label nodes in this cluster
static void pf_kdtree_cluster_node(pf_kdtree_t *self, pf_kdtree_node_t *node, int depth);

// Recursive node printing
//static void pf_kdtree_print_node(pf_kdtree_t *self, pf_kdtree_node_t *node);


#ifdef INCLUDE_RTKGUI

// Recursively draw nodes
static void pf_kdtree_draw_node(pf_kdtree_t *self, pf_kdtree_node_t *node, rtk_fig_t *fig);

#endif



////////////////////////////////////////////////////////////////////////////////
// Create a tree
pf_kdtree_t *pf_kdtree_alloc(int max_size)
{
  pf_kdtree_t *self;

  self = calloc(1, sizeof(pf_kdtree_t));

  self->size[0] = 0.50;
  self->size[1] = 0.50;
  self->size[2] = (10 * M_PI / 180); //angle in degree * 10

  self->root = NULL;

  self->node_count = 0;
  self->node_max_count = max_size;
  self->nodes = calloc(self->node_max_count, sizeof(pf_kdtree_node_t));

  self->leaf_count = 0;

  return self;
}


////////////////////////////////////////////////////////////////////////////////
// Destroy a tree
void pf_kdtree_free(pf_kdtree_t *self)
{
  free(self->nodes);
  free(self);
  return;
}


////////////////////////////////////////////////////////////////////////////////
// Clear all entries from the tree
void pf_kdtree_clear(pf_kdtree_t *self)
{
  self->root = NULL;
  self->leaf_count = 0;
  self->node_count = 0;

  return;
}


////////////////////////////////////////////////////////////////////////////////
// Insert a pose into the tree.
void pf_kdtree_insert(pf_kdtree_t *self, pf_vector_t pose, double value)
{
  int key[3];

  key[0] = floor(pose.v[0] / self->size[0]);//按照size进行成比例放大
  key[1] = floor(pose.v[1] / self->size[1]);
  key[2] = floor(pose.v[2] / self->size[2]);

  self->root = pf_kdtree_insert_node(self, NULL, self->root, key, value);//调用pf_kdtree_insert_node函数进行插入

  // Test code
  /*
  printf("find %d %d %d\n", key[0], key[1], key[2]);
  assert(pf_kdtree_find_node(self, self->root, key) != NULL);

  pf_kdtree_print_node(self, self->root);

  printf("\n");

  for (i = 0; i < self->node_count; i++)
  {
    node = self->nodes + i;
    if (node->leaf)
    {
      printf("find %d %d %d\n", node->key[0], node->key[1], node->key[2]);
      assert(pf_kdtree_find_node(self, self->root, node->key) == node);
    }
  }
  printf("\n\n");
  */

  return;
}


////////////////////////////////////////////////////////////////////////////////
// Determine the probability estimate for the given pose. TODO: this
// should do a kernel density estimate rather than a simple histogram.
double pf_kdtree_get_prob(pf_kdtree_t *self, pf_vector_t pose)
{
  int key[3];
  pf_kdtree_node_t *node;

  key[0] = floor(pose.v[0] / self->size[0]);
  key[1] = floor(pose.v[1] / self->size[1]);
  key[2] = floor(pose.v[2] / self->size[2]);

  node = pf_kdtree_find_node(self, self->root, key);
  if (node == NULL)
    return 0.0;
  return node->value;
}


////////////////////////////////////////////////////////////////////////////////
// Determine the cluster label for the given pose
int pf_kdtree_get_cluster(pf_kdtree_t *self, pf_vector_t pose)
{
  int key[3];
  pf_kdtree_node_t *node;

  key[0] = floor(pose.v[0] / self->size[0]);
  key[1] = floor(pose.v[1] / self->size[1]);
  key[2] = floor(pose.v[2] / self->size[2]);

  node = pf_kdtree_find_node(self, self->root, key);
  if (node == NULL)
    return -1;
  return node->cluster;
}


////////////////////////////////////////////////////////////////////////////////
// Compare keys to see if they are equal  这个self是不是有些没有意义，这又不是一个类成员函数
int pf_kdtree_equal(pf_kdtree_t *self, int key_a[], int key_b[])
{
  //double a, b;

  if (key_a[0] != key_b[0])
    return 0;
  if (key_a[1] != key_b[1])
    return 0;

  if (key_a[2] != key_b[2])
    return 0;

  /* TODO: make this work (pivot selection needs fixing, too)
  // Normalize angles
  a = key_a[2] * self->size[2];
  a = atan2(sin(a), cos(a)) / self->size[2];
  b = key_b[2] * self->size[2];
  b = atan2(sin(b), cos(b)) / self->size[2];

 if ((int) a != (int) b)
    return 0;
  */

  return 1;
}


////////////////////////////////////////////////////////////////////////////////
// Insert a node into the tree  插入功能的实际执行函数
pf_kdtree_node_t *pf_kdtree_insert_node(pf_kdtree_t *self, pf_kdtree_node_t *parent,
                                        pf_kdtree_node_t *node, int key[], double value)//self是当前的插入树，node 是插入点  ，parent是插入点的父节点
{
  int i;
  int split, max_split;

  // If the node doesnt exist yet...
  if (node == NULL)//情况一，还没有节点,这里对应的是向叶子下边插入节点吧
  {
    assert(self->node_count < self->node_max_count);//确保当前树的node个树小于最大容纳值
    node = self->nodes + self->node_count++;// 当node=NULL 时，即不知道插入点时，就需要将插入点node设定为 树的储存空间首位加现有node个数，并且最后把node count加一
    memset(node, 0, sizeof(pf_kdtree_node_t));//分配空间，并将空间中内容初始化为 0 

    node->leaf = 1;//这个node就是新插入点的储存位置，标志这个node是一个叶子node
    //更新深度
    if (parent == NULL)
      node->depth = 0;
    else
      node->depth = parent->depth + 1;

    for (i = 0; i < 3; i++)
      node->key[i] = key[i];

    node->value = value;
    self->leaf_count += 1;//我不太明白这个leaf_count加一，如果荣tree的概念出发，这个leaf_count应该是不变的，本质上是叶子节点个数没有增加，但是非叶节点个数增加了
  }

  // If the node exists, and it is a leaf node... 情况二 若该node是叶子，则进行分家
  else if (node->leaf)
  {
    // If the keys are equal, increment the value  如果该node的坐标与插入的node的坐标相同，则为这个node增加权重
    if (pf_kdtree_equal(self, key, node->key))
    {
      node->value += value;
    }

    // The keys are not equal, so split this node
    else
    {
      // Find the dimension with the largest variance and do a mean
      // split
      max_split = 0;
      node->pivot_dim = -1;
      for (i = 0; i < 3; i++)//搜索差距最大维度
      {
        split = abs(key[i] - node->key[i]);
        if (split > max_split)
        {
          max_split = split;
          node->pivot_dim = i;
        }
      }
      assert(node->pivot_dim >= 0);

      node->pivot_value = (key[node->pivot_dim] + node->key[node->pivot_dim]) / 2.0;//最大维度的中值

      if (key[node->pivot_dim] < node->pivot_value)
      {
        node->children[0] = pf_kdtree_insert_node(self, node, NULL, key, value);//leaf  进入情况一的行为
        node->children[1] = pf_kdtree_insert_node(self, node, NULL, node->key, node->value);
      }
      else
      {
        node->children[0] = pf_kdtree_insert_node(self, node, NULL, node->key, node->value);
        node->children[1] = pf_kdtree_insert_node(self, node, NULL, key, value);
      }

      node->leaf = 0;//将该节点设定为非叶子节点
      self->leaf_count -= 1; //这里理解没有问题
    }
  }

  // If the node exists, and it has children... 如果插入点是非叶node，这里定义非叶node是有两个分支才可以
  else
  {
    assert(node->children[0] != NULL);
    assert(node->children[1] != NULL);

    if (key[node->pivot_dim] < node->pivot_value)//这个node->pivot_value 实际是左孩子与右孩子的分家点
      pf_kdtree_insert_node(self, node, node->children[0], key, value);
    else
      pf_kdtree_insert_node(self, node, node->children[1], key, value);
  }

  return node;
}


////////////////////////////////////////////////////////////////////////////////
// Recursive node search
pf_kdtree_node_t *pf_kdtree_find_node(pf_kdtree_t *self, pf_kdtree_node_t *node, int key[])
{
  if (node->leaf)
  {
    //printf("find  : leaf %p %d %d %d\n", node, node->key[0], node->key[1], node->key[2]);

    // If the keys are the same...
    if (pf_kdtree_equal(self, key, node->key))
      return node;
    else
      return NULL;
  }
  else
  {
    //printf("find  : brch %p %d %f\n", node, node->pivot_dim, node->pivot_value);

    assert(node->children[0] != NULL);
    assert(node->children[1] != NULL);

    // If the keys are different...
    if (key[node->pivot_dim] < node->pivot_value)
      return pf_kdtree_find_node(self, node->children[0], key);
    else
      return pf_kdtree_find_node(self, node->children[1], key);
  }

  return NULL;
}


////////////////////////////////////////////////////////////////////////////////
// Recursive node printing
/*
void pf_kdtree_print_node(pf_kdtree_t *self, pf_kdtree_node_t *node)
{
  if (node->leaf)
  {
    printf("(%+02d %+02d %+02d)\n", node->key[0], node->key[1], node->key[2]);
    printf("%*s", node->depth * 11, "");
  }
  else
  {
    printf("(%+02d %+02d %+02d) ", node->key[0], node->key[1], node->key[2]);
    pf_kdtree_print_node(self, node->children[0]);
    pf_kdtree_print_node(self, node->children[1]);
  }
  return;
}
*/


////////////////////////////////////////////////////////////////////////////////
// Cluster the leaves in the tree
void pf_kdtree_cluster(pf_kdtree_t *self)
{
  int i;
  int queue_count, cluster_count;
  pf_kdtree_node_t **queue, *node;

  queue_count = 0;
  queue = calloc(self->node_count, sizeof(queue[0]));

  // Put all the leaves in a queue
  for (i = 0; i < self->node_count; i++)
  {
    node = self->nodes + i;
    if (node->leaf)
    {
      node->cluster = -1;
      assert(queue_count < self->node_count);
      queue[queue_count++] = node;

      // TESTING; remove
      assert(node == pf_kdtree_find_node(self, self->root, node->key));
    }
  }

  cluster_count = 0;

  // Do connected components for each node
  while (queue_count > 0)
  {
    node = queue[--queue_count];

    // If this node has already been labelled, skip it
    if (node->cluster >= 0)
      continue;

    // Assign a label to this cluster
    node->cluster = cluster_count++;

    // Recursively label nodes in this cluster
    pf_kdtree_cluster_node(self, node, 0);
  }

  free(queue);
  return;
}


////////////////////////////////////////////////////////////////////////////////
// Recursively label nodes in this cluster
void pf_kdtree_cluster_node(pf_kdtree_t *self, pf_kdtree_node_t *node, int depth)
{
  int i;
  int nkey[3];
  pf_kdtree_node_t *nnode;

  for (i = 0; i < 3 * 3 * 3; i++)
  {
    nkey[0] = node->key[0] + (i / 9) - 1;
    nkey[1] = node->key[1] + ((i % 9) / 3) - 1;
    nkey[2] = node->key[2] + ((i % 9) % 3) - 1; // neiborhood

    nnode = pf_kdtree_find_node(self, self->root, nkey);
    if (nnode == NULL)
      continue;

    assert(nnode->leaf);

    // This node already has a label; skip it.  The label should be
    // consistent, however.
    if (nnode->cluster >= 0)
    {
      assert(nnode->cluster == node->cluster);
      continue;
    }

    // Label this node and recurse
    nnode->cluster = node->cluster;

    pf_kdtree_cluster_node(self, nnode, depth + 1);
  }
  return;
}



#ifdef INCLUDE_RTKGUI

////////////////////////////////////////////////////////////////////////////////
// Draw the tree
void pf_kdtree_draw(pf_kdtree_t *self, rtk_fig_t *fig)
{
  if (self->root != NULL)
    pf_kdtree_draw_node(self, self->root, fig);
  return;
}


////////////////////////////////////////////////////////////////////////////////
// Recursively draw nodes
void pf_kdtree_draw_node(pf_kdtree_t *self, pf_kdtree_node_t *node, rtk_fig_t *fig)
{
  double ox, oy;
  char text[64];

  if (node->leaf)
  {
    ox = (node->key[0] + 0.5) * self->size[0];
    oy = (node->key[1] + 0.5) * self->size[1];

    rtk_fig_rectangle(fig, ox, oy, 0.0, self->size[0], self->size[1], 0);

    //snprintf(text, sizeof(text), "%0.3f", node->value);
    //rtk_fig_text(fig, ox, oy, 0.0, text);

    snprintf(text, sizeof(text), "%d", node->cluster);
    rtk_fig_text(fig, ox, oy, 0.0, text);
  }
  else
  {
    assert(node->children[0] != NULL);
    assert(node->children[1] != NULL);
    pf_kdtree_draw_node(self, node->children[0], fig);
    pf_kdtree_draw_node(self, node->children[1], fig);
  }

  return;
}

#endif
