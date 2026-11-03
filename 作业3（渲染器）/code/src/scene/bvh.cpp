#include "bvh.h"

#include "CGL/CGL.h"
#include "triangle.h"

#include <iostream>
#include <stack>

using namespace std;

namespace CGL {
namespace SceneObjects {

BVHAccel::BVHAccel(const std::vector<Primitive *> &_primitives,
                   size_t max_leaf_size) {

  primitives = std::vector<Primitive *>(_primitives);
  root = construct_bvh(primitives.begin(), primitives.end(), max_leaf_size);
}

BVHAccel::~BVHAccel() {
  if (root)
    delete root;
  primitives.clear();
}

BBox BVHAccel::get_bbox() const { return root->bb; }

void BVHAccel::draw(BVHNode *node, const Color &c, float alpha) const {
  if (node->isLeaf()) {
    for (auto p = node->start; p != node->end; p++) {
      (*p)->draw(c, alpha);
    }
  } else {
    draw(node->l, c, alpha);
    draw(node->r, c, alpha);
  }
}

void BVHAccel::drawOutline(BVHNode *node, const Color &c, float alpha) const {
  if (node->isLeaf()) {
    for (auto p = node->start; p != node->end; p++) {
      (*p)->drawOutline(c, alpha);
    }
  } else {
    drawOutline(node->l, c, alpha);
    drawOutline(node->r, c, alpha);
  }
}

BVHNode *BVHAccel::construct_bvh(std::vector<Primitive *>::iterator start,
                                 std::vector<Primitive *>::iterator end,
                                 size_t max_leaf_size) {
  // Construct a BVH from the given vector of primitives and maximum leaf
  // size configuration. The starter code build a BVH aggregate with a
  // single leaf node (which is also the root) that encloses all the
  // primitives.

  BBox bbox;
  BBox centers;

  for (auto p = start; p != end; p++) {
    BBox bb = (*p)->get_bbox();
    bbox.expand(bb);
    centers.expand(bb.centroid());
  }

  BVHNode *node = new BVHNode(bbox);
  node->start = start;
  node->end = end;

  if (end - start <= max_leaf_size)
    return node;

  int axis = 0;
  if (centers.extent.y > centers.extent[axis])
    axis = 1;
  if (centers.extent.z > centers.extent[axis])
    axis = 2;

  auto middle = start + (end - start) / 2;
  std::nth_element(start, middle, end, [axis](Primitive *a, Primitive *b) {
    return a->get_bbox().centroid()[axis] < b->get_bbox().centroid()[axis];
  });

  node->l = construct_bvh(start, middle, max_leaf_size);
  node->r = construct_bvh(middle, end, max_leaf_size);

  return node;
}

bool BVHAccel::has_intersection(const Ray &ray, BVHNode *node) const {
  // Fill in the intersect function.
  // Take note that this function has a short-circuit that the
  // Intersection version cannot, since it returns as soon as it finds
  // a hit, it doesn't actually have to find the closest hit.

  double near = ray.min_t;
  double far = ray.max_t;
  if (!node->bb.intersect(ray, near, far))
    return false;

  if (node->isLeaf()) {
    for (auto p = node->start; p != node->end; p++) {
      total_isects++;
      if ((*p)->has_intersection(ray))
        return true;
    }
    return false;
  }

  return has_intersection(ray, node->l) || has_intersection(ray, node->r);
}

bool BVHAccel::intersect(const Ray &ray, Intersection *i, BVHNode *node) const {
  // Fill in the intersect function.

  double near = ray.min_t;
  double far = ray.max_t;
  if (!node->bb.intersect(ray, near, far))
    return false;

  if (node->isLeaf()) {
    bool hit = false;
    for (auto p = node->start; p != node->end; p++) {
      total_isects++;
      hit = (*p)->intersect(ray, i) || hit;
    }
    return hit;
  }

  bool left = intersect(ray, i, node->l);
  bool right = intersect(ray, i, node->r);
  return left || right;
}

} // namespace SceneObjects
} // namespace CGL
