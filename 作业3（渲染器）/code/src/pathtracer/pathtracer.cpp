#include "pathtracer.h"

#include "scene/light.h"
#include "scene/sphere.h"
#include "scene/triangle.h"


using namespace CGL::SceneObjects;

namespace CGL {

PathTracer::PathTracer() {
  gridSampler = new UniformGridSampler2D();
  hemisphereSampler = new UniformHemisphereSampler3D();

  tm_gamma = 2.2f;
  tm_level = 1.0f;
  tm_key = 0.18;
  tm_wht = 5.0f;
}

PathTracer::~PathTracer() {
  delete gridSampler;
  delete hemisphereSampler;
}

void PathTracer::set_frame_size(size_t width, size_t height) {
  sampleBuffer.resize(width, height);
  sampleCountBuffer.resize(width * height);
}

void PathTracer::clear() {
  bvh = NULL;
  scene = NULL;
  camera = NULL;
  sampleBuffer.clear();
  sampleCountBuffer.clear();
  sampleBuffer.resize(0, 0);
  sampleCountBuffer.resize(0, 0);
}

void PathTracer::write_to_framebuffer(ImageBuffer &framebuffer, size_t x0,
                                      size_t y0, size_t x1, size_t y1) {
  sampleBuffer.toColor(framebuffer, x0, y0, x1, y1);
}

Vector3D
PathTracer::estimate_direct_lighting_hemisphere(const Ray &r,
                                                const Intersection &isect) {
  // Estimate the lighting from this intersection coming directly from a light.
  // For this function, sample uniformly in a hemisphere.

  // Note: When comparing Cornel Box (CBxxx.dae) results to importance sampling, you may find the "glow" around the light source is gone.
  // This is totally fine: the area lights in importance sampling has directionality, however in hemisphere sampling we don't model this behaviour.

  // make a coordinate system for a hit point
  // with N aligned with the Z direction.
  Matrix3x3 o2w;
  make_coord_space(o2w, isect.n);
  Matrix3x3 w2o = o2w.T();

  // w_out points towards the source of the ray (e.g.,
  // toward the camera if this is a primary ray)
  const Vector3D hit_p = r.o + r.d * isect.t;
  const Vector3D w_out = w2o * (-r.d);

  // This is the same number of total samples as
  // estimate_direct_lighting_importance (outside of delta lights). We keep the
  // same number of samples for clarity of comparison.
  int num_samples = scene->lights.size() * ns_area_light;
  Vector3D L_out;

  for (int sample = 0; sample < num_samples; sample++) {
    Vector3D wi = hemisphereSampler->get_sample();
    Ray shadow(hit_p, (o2w * wi).unit());
    shadow.min_t = EPS_F;
    Intersection light;

    if (bvh->intersect(shadow, &light))
      L_out += isect.bsdf->f(w_out, wi) * light.bsdf->get_emission() *
               (wi.z * 2 * PI / num_samples);
  }

  return L_out;
}

Vector3D
PathTracer::estimate_direct_lighting_importance(const Ray &r,
                                                const Intersection &isect) {
  // Estimate the lighting from this intersection coming directly from a light.
  // To implement importance sampling, sample only from lights, not uniformly in
  // a hemisphere.

  // make a coordinate system for a hit point
  // with N aligned with the Z direction.
  Matrix3x3 o2w;
  make_coord_space(o2w, isect.n);
  Matrix3x3 w2o = o2w.T();

  // w_out points towards the source of the ray (e.g.,
  // toward the camera if this is a primary ray)
  const Vector3D hit_p = r.o + r.d * isect.t;
  const Vector3D w_out = w2o * (-r.d);
  Vector3D L_out;

  for (auto light : scene->lights) {
    int count = light->is_delta_light() ? 1 : ns_area_light;
    Vector3D sum;

    for (int sample = 0; sample < count; sample++) {
      Vector3D direction;
      double distance, pdf;
      Vector3D radiance = light->sample_L(hit_p, &direction, &distance, &pdf);
      Vector3D wi = w2o * direction;
      if (wi.z <= 0)
        continue;

      // The template's area-light PDF uses an unnormalized direction.
      if (dynamic_cast<AreaLight *>(light))
        pdf *= distance;

      Ray shadow(hit_p, direction);
      shadow.min_t = EPS_F;
      shadow.max_t = distance - EPS_F;

      if (!bvh->has_intersection(shadow))
        sum += isect.bsdf->f(w_out, wi) * radiance * (wi.z / pdf);
    }

    L_out += sum / count;
  }

  return L_out;
}

Vector3D PathTracer::zero_bounce_radiance(const Ray &r,
                                          const Intersection &isect) {
  // Returns the light that results from no bounces of light

  return isect.bsdf->get_emission();
}

Vector3D PathTracer::one_bounce_radiance(const Ray &r,
                                         const Intersection &isect) {
  // Returns either the direct illumination by hemisphere or importance sampling
  // depending on `direct_hemisphere_sample`

  return direct_hemisphere_sample
             ? estimate_direct_lighting_hemisphere(r, isect)
             : estimate_direct_lighting_importance(r, isect);
}

Vector3D PathTracer::at_least_one_bounce_radiance(const Ray &r,
                                                  const Intersection &isect) {
  Matrix3x3 o2w;
  make_coord_space(o2w, isect.n);
  Matrix3x3 w2o = o2w.T();

  Vector3D hit_p = r.o + r.d * isect.t;
  Vector3D w_out = w2o * (-r.d);

  Vector3D L_out(0, 0, 0);

  // Returns the one bounce radiance + radiance from extra bounces at this point.
  // Should be called recursively to simulate extra bounces.
  if (isAccumBounces || r.depth == 1)
    L_out = one_bounce_radiance(r, isect);
  if (r.depth == 1)
    return L_out;

  double survival = r.depth == max_ray_depth ? 1.0 : 0.65;
  if (!coin_flip(survival))
    return L_out;

  Vector3D wi;
  double pdf;
  Vector3D value = isect.bsdf->sample_f(w_out, &wi, &pdf);

  Ray bounce(hit_p, (o2w * wi).unit(), int(r.depth - 1));
  bounce.min_t = EPS_F;
  Intersection next;

  if (bvh->intersect(bounce, &next))
    L_out += value * at_least_one_bounce_radiance(bounce, next) *
             (wi.z / (pdf * survival));

  return L_out;
}

Vector3D PathTracer::est_radiance_global_illumination(const Ray &r) {
  Intersection isect;
  Vector3D L_out;

  // You will extend this in assignment 3-2.
  // If no intersection occurs, we simply return black.
  // This changes if you implement hemispherical lighting for extra credit.

  if (!bvh->intersect(r, &isect))
    return envLight ? envLight->sample_dir(r) : L_out;

  if (isAccumBounces || max_ray_depth == 0)
    L_out = zero_bounce_radiance(r, isect);
  if (r.depth > 0)
    L_out += at_least_one_bounce_radiance(r, isect);

  return L_out;
}

void PathTracer::raytrace_pixel(size_t x, size_t y) {
  // Make a loop that generates num_samples camera rays and traces them
  // through the scene. Return the average Vector3D.
  // You should call est_radiance_global_illumination in this function.

  // Modify your implementation to include adaptive sampling.
  // Use the command line parameters "samplesPerBatch" and "maxTolerance"
  int num_samples = ns_aa;          // total samples to evaluate
  Vector2D origin = Vector2D(x, y); // bottom left corner of the pixel

  Vector3D sum;
  double mean = 0;
  double m2 = 0;
  int count = 0;

  for (int sample = 0; sample < num_samples; sample++) {
    Vector2D point = origin + gridSampler->get_sample();
    Ray ray = camera->generate_ray(point.x / sampleBuffer.w,
                                   point.y / sampleBuffer.h);
    ray.depth = max_ray_depth;
    Vector3D radiance = est_radiance_global_illumination(ray);

    sum += radiance;
    count++;

    double delta = radiance.illum() - mean;
    mean += delta / count;
    m2 += delta * (radiance.illum() - mean);

    if (count > 1 && count % samplesPerBatch == 0) {
      double interval = 1.96 * sqrt(m2 / (count - 1) / count);
      if (interval <= maxTolerance * mean)
        break;
    }
  }

  sampleBuffer.update_pixel(sum / count, x, y);
  sampleCountBuffer[x + y * sampleBuffer.w] = count;
}

void PathTracer::autofocus(Vector2D loc) {
  Ray r = camera->generate_ray(loc.x / sampleBuffer.w, loc.y / sampleBuffer.h);
  Intersection isect;

  bvh->intersect(r, &isect);

  camera->focalDistance = isect.t;
}

} // namespace CGL
