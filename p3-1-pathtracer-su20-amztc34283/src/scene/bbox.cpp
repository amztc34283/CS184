#include "bbox.h"

#include "GL/glew.h"

#include <algorithm>
#include <iostream>

namespace CGL {

bool BBox::intersect(const Ray& r, double& t0, double& t1) const {

  // TODO (Part 2.2):
  // Implement ray - bounding box intersection test
  // If the ray intersected the bouding box within the range given by
  // t0, t1, update t0 and t1 with the new intersection times.

    // t0 is smaller than t1
    // when denominator is 0, the ray will never hit the axis aligned bounding box
    double t_min = -INFINITY;
    double t_max = INFINITY;
    if (r.d.x != 0) {
        double t_x_min = (min.x - r.o.x) / r.d.x;
        double t_x_max = (max.x - r.o.x) / r.d.x;
        // It is possible to hit max.x first then min.x
        // It is possible that it never hits the box, need to check if the intersected point is within the range of the box in other axes
        double t_x_start = std::min(t_x_min, t_x_max);
        double t_x_end = std::max(t_x_min, t_x_max);
        Vector3D intersect_x = r.o + t_x_start * r.d;
        if (intersect_x.y <= max.y && intersect_x.y >= min.y
            && intersect_x.z <= max.z && intersect_x.z >= min.z) {
            t_min = std::max(t_min, t_x_start);
            t_max = std::min(t_max, t_x_end);
        }
    }
    if (r.d.y != 0) {
        double t_y_min = (min.y - r.o.y) / r.d.y;
        double t_y_max = (max.y - r.o.y) / r.d.y;
        double t_y_start = std::min(t_y_min, t_y_max);
        double t_y_end = std::max(t_y_min, t_y_max);
        Vector3D intersect_y = r.o + t_y_start * r.d;
        if (intersect_y.x <= max.x && intersect_y.x >= min.x
            && intersect_y.z <= max.z && intersect_y.z >= min.z) {
            t_min = std::max(t_min, t_y_start);
            t_max = std::min(t_max, t_y_end);
        }
    }
    if (r.d.z != 0) {
        double t_z_min = (min.z - r.o.z) / r.d.z;
        double t_z_max = (max.z - r.o.z) / r.d.z;
        double t_z_start = std::min(t_z_min, t_z_max);
        double t_z_end = std::max(t_z_min, t_z_max);
        Vector3D intersect_z = r.o + t_z_start * r.d;
        if (intersect_z.x <= max.x && intersect_z.x >= min.x
            && intersect_z.y <= max.y && intersect_z.y >= min.y) {
            t_min = std::max(t_min, t_z_start);
            t_max = std::min(t_max, t_z_end);
        }
    }

    // Check the case that the t0 and t1 is within the bounding box
    if (t_max != INFINITY && t_min != -INFINITY) {
        // reassign t0 and t1
        // TODO: Do we have to update t0 and t1 here?
        return true;
    }
    return false;
}

void BBox::draw(Color c, float alpha) const {

  glColor4f(c.r, c.g, c.b, alpha);

  // top
  glBegin(GL_LINE_STRIP);
  glVertex3d(max.x, max.y, max.z);
  glVertex3d(max.x, max.y, min.z);
  glVertex3d(min.x, max.y, min.z);
  glVertex3d(min.x, max.y, max.z);
  glVertex3d(max.x, max.y, max.z);
  glEnd();

  // bottom
  glBegin(GL_LINE_STRIP);
  glVertex3d(min.x, min.y, min.z);
  glVertex3d(min.x, min.y, max.z);
  glVertex3d(max.x, min.y, max.z);
  glVertex3d(max.x, min.y, min.z);
  glVertex3d(min.x, min.y, min.z);
  glEnd();

  // side
  glBegin(GL_LINES);
  glVertex3d(max.x, max.y, max.z);
  glVertex3d(max.x, min.y, max.z);
  glVertex3d(max.x, max.y, min.z);
  glVertex3d(max.x, min.y, min.z);
  glVertex3d(min.x, max.y, min.z);
  glVertex3d(min.x, min.y, min.z);
  glVertex3d(min.x, max.y, max.z);
  glVertex3d(min.x, min.y, max.z);
  glEnd();

}

std::ostream& operator<<(std::ostream& os, const BBox& b) {
  return os << "BBOX(" << b.min << ", " << b.max << ")";
}

} // namespace CGL
