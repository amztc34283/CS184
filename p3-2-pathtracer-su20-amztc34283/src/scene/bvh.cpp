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

  // TODO (Part 2.1):
  // Construct a BVH from the given vector of primitives and maximum leaf
  // size configuration. The starter code build a BVH aggregate with a
  // single leaf node (which is also the root) that encloses all the
  // primitives.

    BBox centroid_box, bbox;

    for (auto p = start; p != end; p++) {
        BBox bb = (*p)->get_bbox();
        bbox.expand(bb);
        Vector3D c = bb.centroid();
        centroid_box.expand(c); // Not sure why we have centroid box here
    }

    int size = 0;
    for (auto p = start; p != end; p++) {
        size++;
    }

    if (size <= max_leaf_size) {
        BVHNode *leaf = new BVHNode(bbox);
        leaf->start = start;
        leaf->end = end;
        // l and r should be null
        return leaf;
    }

    // This is not a leaf node.
    // get the max axis for splitting
    double max_axis = max(bbox.extent.x, max(bbox.extent.y, bbox.extent.z));
    if (max_axis == bbox.extent.x) {
        // split by x
        float x_split = centroid_box.centroid().x;
        vector<Primitive*>* left = new vector<Primitive*>();
        vector<Primitive*>* right = new vector<Primitive*>();
        // split them to left and right
        for (auto p = start; p != end; p++) {
            if ((*p)->get_bbox().centroid().x <= x_split) {
                left->push_back(*p);
            } else {
                right->push_back(*p);
            }
        }
        BVHNode *leaf = new BVHNode(bbox);
        // We might need to handle the case that all elements go to one side, but I don't think we have to as we are using the centroid box's centroid
        leaf->l = construct_bvh(left->begin(), left->end(), max_leaf_size);
        leaf->r = construct_bvh(right->begin(), right->end(), max_leaf_size);
        return leaf;
    } else if (max_axis == bbox.extent.y) {
        // split by y
        float y_split = centroid_box.centroid().y;
        vector<Primitive*>* left = new vector<Primitive*>();
        vector<Primitive*>* right = new vector<Primitive*>();
        // split them to left and right
        for (auto p = start; p != end; p++) {
            if ((*p)->get_bbox().centroid().y <= y_split) {
                left->push_back(*p);
            } else {
                right->push_back(*p);
            }
        }
        BVHNode *leaf = new BVHNode(bbox);
        leaf->l = construct_bvh(left->begin(), left->end(), max_leaf_size);
        leaf->r = construct_bvh(right->begin(), right->end(), max_leaf_size);
        return leaf;
    } else {
        // split by z
        float z_split = centroid_box.centroid().z;
        vector<Primitive*>* left = new vector<Primitive*>();
        vector<Primitive*>* right = new vector<Primitive*>();
        // split them to left and right
        for (auto p = start; p != end; p++) {
            if ((*p)->get_bbox().centroid().z <= z_split) {
                left->push_back(*p);
            } else {
                right->push_back(*p);
            }
        }
        BVHNode *leaf = new BVHNode(bbox);
        leaf->l = construct_bvh(left->begin(), left->end(), max_leaf_size);
        leaf->r = construct_bvh(right->begin(), right->end(), max_leaf_size);
        return leaf;
    }
}

bool BVHAccel::has_intersection(const Ray &ray, BVHNode *node) const {
  // TODO (Part 2.3):
  // Fill in the intersect function.
  // Take note that this function has a short-circuit that the
  // Intersection version cannot, since it returns as soon as it finds
  // a hit, it doesn't actually have to find the closest hit.

    double min_t_copy = ray.min_t;
    double max_t_copy = ray.max_t;

    if (!node->bb.intersect(ray, min_t_copy, max_t_copy))
        return false;
    if (node->isLeaf()) {
        bool intersected = false;
        for (auto p = node->start; p != node->end; p++) {
            total_isects++; // TODO: comment out to speed up for debug
            if ((*p)->has_intersection(ray))
                intersected = true;
        }
        return intersected;
    }
    bool intersect_left = has_intersection(ray, node->l);
    bool intersect_right = has_intersection(ray, node->r);
    return intersect_left || intersect_right;
}

bool BVHAccel::intersect(const Ray &ray, Intersection *i, BVHNode *node) const {
  // TODO (Part 2.3):
  // Fill in the intersect function.

    double min_t_copy = ray.min_t;
    double max_t_copy = ray.max_t;

    if (!node->bb.intersect(ray, min_t_copy, max_t_copy)) {
        return false;
    }
    if (node->isLeaf()) {
        bool intersected = false;
        for (auto p = node->start; p != node->end; p++) {
            total_isects++; // TODO: comment out to speed up for debug
            if ((*p)->intersect(ray, i))
                intersected = true;
        }
        return intersected;
    }
    bool intersect_left = intersect(ray, i, node->l);
    bool intersect_right = intersect(ray, i, node->r);
    return intersect_left || intersect_right;
}

} // namespace SceneObjects
} // namespace CGL
