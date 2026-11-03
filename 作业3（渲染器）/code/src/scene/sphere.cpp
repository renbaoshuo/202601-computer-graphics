#include "sphere.h"

#include <cmath>

#include "pathtracer/bsdf.h"
#include "util/sphere_drawing.h"

namespace CGL {
namespace SceneObjects {

bool Sphere::test(const Ray &r, double &t1, double &t2) const {
  // Implement ray - sphere intersection test.
  // Return true if there are intersections and writing the
  // smaller of the two intersection times in t1 and the larger in t2.

  Vector3D offset = r.o - o;
  double a = dot(r.d, r.d);
  double b = dot(offset, r.d);
  double c = dot(offset, offset) - r2;
  double discriminant = b * b - a * c;
  if (discriminant < 0)
    return false;

  double root = sqrt(discriminant);
  t1 = (-b - root) / a;
  t2 = (-b + root) / a;

  return true;
}

bool Sphere::has_intersection(const Ray &r) const {
  // Implement ray - sphere intersection.
  // Note that you might want to use the the Sphere::test helper here.

  double near, far;
  if (!test(r, near, far))
    return false;

  double t = near < r.min_t ? far : near;
  if (t < r.min_t || t > r.max_t)
    return false;

  r.max_t = t;
  return true;
}

bool Sphere::intersect(const Ray &r, Intersection *i) const {
  // Implement ray - sphere intersection.
  // Note again that you might want to use the the Sphere::test helper here.
  // When an intersection takes place, the Intersection data should be updated
  // correspondingly.

  double near, far;
  if (!test(r, near, far))
    return false;

  double t = near < r.min_t ? far : near;
  if (t < r.min_t || t > r.max_t)
    return false;

  r.max_t = t;
  i->t = t;
  i->n = normal(r.at_time(t));
  i->primitive = this;
  i->bsdf = get_bsdf();

  return true;
}

void Sphere::draw(const Color &c, float alpha) const {
  Misc::draw_sphere_opengl(o, r, c);
}

void Sphere::drawOutline(const Color &c, float alpha) const {
  // Misc::draw_sphere_opengl(o, r, c);
}

} // namespace SceneObjects
} // namespace CGL
