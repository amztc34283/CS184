#include "sphere.h"

#include <cmath>

#include "pathtracer/bsdf.h"
#include "util/sphere_drawing.h"

namespace CGL {
namespace SceneObjects {

bool Sphere::test(const Ray &r, double &t1, double &t2) const {

  // TODO (Part 1.4):
  // Implement ray - sphere intersection test.
  // Return true if there are intersections and writing the
  // smaller of the two intersection times in t1 and the larger in t2.
    float a = dot(r.d, r.d);
    float b = (float) 2 * dot(r.o - this->o, r.d);
    float c = dot(r.o - this->o, r.o - this->o) - this->r2;
    float discriminant = b * b - (float) 4 * a * c;
    if (discriminant > 0) {
        t1 = ((float) -1 * b) / ((float) 2 * a) - sqrt(discriminant) / ((float) 2 * a);
        t2 = ((float) -1 * b) / ((float) 2 * a) + sqrt(discriminant) / ((float) 2 * a);
// TODO: Keep in mind t2 can be a valid point while t1 is not.
        return t1 >= r.min_t && t1 <= r.max_t ? true : false;
    } else if (discriminant == 0) { // If there are imaginary numbers, don't count
        t1 = (float) -1 * b / ((float) 2 * a);
        return t1 >= r.min_t && t1 <= r.max_t ? true : false;;
    }
    return false;
}

bool Sphere::has_intersection(const Ray &r) const {

  // TODO (Part 1.4):
  // Implement ray - sphere intersection.
  // Note that you might want to use the the Sphere::test helper here.
    double t1, t2;
    if (test(r, t1, t2)) {
        r.max_t = t1;
        return true;
    }
    return false;
}

bool Sphere::intersect(const Ray &r, Intersection *i) const {

  // TODO (Part 1.4):
  // Implement ray - sphere intersection.
  // Note again that you might want to use the the Sphere::test helper here.
  // When an intersection takes place, the Intersection data should be updated
  // correspondingly.
    if (has_intersection(r)) {
        double t1, t2;
        test(r, t1, t2);
        i->n = this->normal(r.o + t1 * r.d);
        i->t = t1;
        i->primitive = this;
        i->bsdf = this->get_bsdf();
        return true;
    }
    return false;
}

void Sphere::draw(const Color &c, float alpha) const {
  Misc::draw_sphere_opengl(o, r, c);
}

void Sphere::drawOutline(const Color &c, float alpha) const {
  // Misc::draw_sphere_opengl(o, r, c);
}

} // namespace SceneObjects
} // namespace CGL
