#include "triangle.h"

#include "CGL/CGL.h"
#include "GL/glew.h"

namespace CGL {
namespace SceneObjects {

Triangle::Triangle(const Mesh *mesh, size_t v1, size_t v2, size_t v3) {
  p1 = mesh->positions[v1];
  p2 = mesh->positions[v2];
  p3 = mesh->positions[v3];
  n1 = mesh->normals[v1];
  n2 = mesh->normals[v2];
  n3 = mesh->normals[v3];
  bbox = BBox(p1);
  bbox.expand(p2);
  bbox.expand(p3);

  bsdf = mesh->get_bsdf();
}

BBox Triangle::get_bbox() const { return bbox; }

Vector3D moller_trumbore(Vector3D p0, Vector3D p1, Vector3D p2, Vector3D o, Vector3D d) {
    // Moller Trumbore Algorithm
    Vector3D e1 = p1 - p0;
    Vector3D e2 = p2 - p0;
    Vector3D s = o - p0;
    Vector3D s1 = cross(d, e2);
    Vector3D s2 = cross(s, e1);
    double s1_e1 = dot(s1, e1);
    double s2_e2 = dot(s2, e2);
    double s1_s = dot(s1, s);
    double s2_d = dot(s2, d);
    return Vector3D(s2_e2 / s1_e1,
                    s1_s / s1_e1,
                    s2_d / s1_e1);

}

bool Triangle::has_intersection(const Ray &r) const {
  // Part 1, Task 3: implement ray-triangle intersection
  // The difference between this function and the next function is that the next
  // function records the "intersection" while this function only tests whether
  // there is a intersection.
    Vector3D t_b1_b2 = moller_trumbore(p1, p2, p3, r.o, r.d);

    // Check the intersection is inside the triangle !!!
    if (t_b1_b2[0] >= r.min_t && t_b1_b2[0] <= r.max_t && t_b1_b2[1] + t_b1_b2[2] <= 1 && t_b1_b2[1] >= 0 && t_b1_b2[2] >= 0) {
        r.max_t = t_b1_b2[0];
        return true;
    }
    return false;
}

bool Triangle::intersect(const Ray &r, Intersection *isect) const {
  // Part 1, Task 3:
  // implement ray-triangle intersection. When an intersection takes
  // place, the Intersection data should be updated accordingly
    if (has_intersection(r)) {
        // Moller Trumbore Algorithm
        Vector3D t_b1_b2 = moller_trumbore(p1, p2, p3, r.o, r.d);
        double b0 = 1 - t_b1_b2[1] - t_b1_b2[2];
        isect->n = n1 * b0 + n2 * t_b1_b2[1] + n3 * t_b1_b2[2];
        isect->t = t_b1_b2[0];
        isect->primitive = this;
        isect->bsdf = this->get_bsdf();
        return true;
    }
    return false;
  return true;
}

void Triangle::draw(const Color &c, float alpha) const {
  glColor4f(c.r, c.g, c.b, alpha);
  glBegin(GL_TRIANGLES);
  glVertex3d(p1.x, p1.y, p1.z);
  glVertex3d(p2.x, p2.y, p2.z);
  glVertex3d(p3.x, p3.y, p3.z);
  glEnd();
}

void Triangle::drawOutline(const Color &c, float alpha) const {
  glColor4f(c.r, c.g, c.b, alpha);
  glBegin(GL_LINE_LOOP);
  glVertex3d(p1.x, p1.y, p1.z);
  glVertex3d(p2.x, p2.y, p2.z);
  glVertex3d(p3.x, p3.y, p3.z);
  glEnd();
}

} // namespace SceneObjects
} // namespace CGL
