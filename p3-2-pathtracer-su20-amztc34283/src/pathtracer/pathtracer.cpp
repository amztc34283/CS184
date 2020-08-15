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

Spectrum
PathTracer::estimate_direct_lighting_hemisphere(const Ray &r,
                                                const Intersection &isect) {
  // Estimate the lighting from this intersection coming directly from a light.
  // For this function, sample uniformly in a hemisphere.

  // make a coordinate system for a hit point
  // with N aligned with the Z direction.
  Matrix3x3 o2w;
  make_coord_space(o2w, isect.n);
  Matrix3x3 w2o = o2w.T();

  // w_out points towards the source of the ray (e.g.,
  // toward the camera if this is a primary ray)
  const Vector3D &hit_p = r.o + r.d * isect.t;
  const Vector3D &w_out = w2o * (-r.d);

  // This is the same number of total samples as
  // estimate_direct_lighting_importance (outside of delta lights). We keep the
  // same number of samples for clarity of comparison.
  int num_samples = scene->lights.size() * ns_area_light;
  Spectrum L_out;

  // TODO (Part 3): Write your sampling loop here
  // TODO BEFORE YOU BEGIN
  // UPDATE `est_radiance_global_illumination` to return direct lighting instead of normal shading 
  Vector3D normal = Vector3D(0,0,1);

  for (int i = 0; i < num_samples; i++) {
      // Transform bounced_vec to world space
      Vector3D bounced_vec = hemisphereSampler->get_sample(); // DO I NEED UNIT()?
      // Travel with the new vector from the hit point to see if it hits any light source
      Intersection bounced_isect;
      Vector3D bounced_vec_world = o2w * bounced_vec;
      Ray second_ray = Ray(hit_p + EPS_D * bounced_vec_world.unit(), bounced_vec_world.unit());
      second_ray.min_t = EPS_F;
      if (bvh->intersect(second_ray, &bounced_isect)) {
          // Get the second intersection point colors
          // Constant Li here and 1 / 2*pi for hemisphere
          // Reflection function
          Spectrum bsdf_val = isect.bsdf->f(w_out, bounced_vec);
          // This should come from direct light
          Spectrum Li = zero_bounce_radiance(second_ray, bounced_isect);
          float cosine = dot(normal, bounced_vec.unit());
          L_out += (bsdf_val * Li * cosine) * (2.0 * PI); // change it to PI to get lighting similar to the doc
      }
  }
  return L_out / (float) num_samples;
}

Spectrum
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
  const Vector3D &hit_p = r.o + r.d * isect.t;
  const Vector3D &w_out = w2o * (-r.d);
  Spectrum L_out;

  Vector3D normal = Vector3D(0,0,1);

  for (SceneLight* light : scene->lights) {
      Spectrum temp;
      int num_samples;
      if (light->is_delta_light()) {
          num_samples = 1;
      } else {
          num_samples = ns_area_light;
      }
      for (int i = 0; i < num_samples; i++) {
          // Transform bounced_vec to world space
          Vector3D wi;
          float t;
          float pdf;
          Spectrum sampled_light = light->sample_L(hit_p, &wi, &t, &pdf);
          // Travel with the new vector from the hit point to see if it hits any light source
          Intersection bounced_isect;
          Vector3D bounced_vec_object = w2o * wi;
          if (bounced_vec_object.z >= 0) {
              Ray second_ray = Ray(hit_p + EPS_D * wi.unit(), wi.unit(), t);
              second_ray.min_t = EPS_F;
              if (!bvh->intersect(second_ray, &bounced_isect)) {
                  Spectrum bsdf_val = isect.bsdf->f(w_out, bounced_vec_object);
                  float cosine = dot(normal, bounced_vec_object.unit());
                  temp += (bsdf_val * sampled_light * cosine) / pdf;
              }
          }
      }
      L_out += temp / (float) num_samples;
  }
  return L_out;
}

Spectrum PathTracer::zero_bounce_radiance(const Ray &r,
                                          const Intersection &isect) {
  // TODO: Part 3, Task 2
  // Returns the light that results from no bounces of light
  return isect.bsdf->get_emission();
}

Spectrum PathTracer::one_bounce_radiance(const Ray &r,
                                         const Intersection &isect) {
  // TODO: Part 3, Task 3
  // Returns either the direct illumination by hemisphere or importance sampling
  // depending on `direct_hemisphere_sample`
  if (direct_hemisphere_sample)
      return estimate_direct_lighting_hemisphere(r, isect);
  else
      return estimate_direct_lighting_importance(r, isect);
}

Spectrum PathTracer::at_least_one_bounce_radiance(const Ray &r,
                                                  const Intersection &isect) {
  Matrix3x3 o2w;
  make_coord_space(o2w, isect.n);
  Matrix3x3 w2o = o2w.T();

  Vector3D hit_p = r.o + r.d * isect.t;
  Vector3D w_out = w2o * (-r.d);

  Spectrum normal(0, 0, 1);
  Spectrum L_out(0, 0, 0);

  // Let's get the one bounce radiance first
  // raytrace_pixel handles multiple samples
  L_out += one_bounce_radiance(r, isect);
  // Sample the material instead of light
  // Transform bounced_vec to world space
  Vector3D wi;
  float pdf;
  Spectrum sampled_light = isect.bsdf->sample_f(w_out, &wi, &pdf);
  float cpdf = coin_flip(0.5) ? 0.6 : 0.7;
  float cosine = dot(normal, wi.unit());
  if (r.depth > 1 && max_ray_depth > 1) {
      Intersection bounced_isect;
      Vector3D wwi = o2w * wi;
      Ray second_ray = Ray(hit_p + EPS_D * wwi.unit(), wwi.unit(), INF_D, r.depth - 1);
      if (bvh->intersect(second_ray, &bounced_isect) && pdf > 0) {
          // Recursive call this function
          if (r.depth == max_ray_depth) { // has to trace this path anyway
              L_out += (at_least_one_bounce_radiance(second_ray, bounced_isect) * cosine * sampled_light) / pdf;
          } else if (!coin_flip(cpdf)) {
              L_out += (at_least_one_bounce_radiance(second_ray, bounced_isect) * cosine * sampled_light) / (pdf * (1 - cpdf));
          }
      }
  }
  return L_out;
}

Spectrum PathTracer::est_radiance_global_illumination(const Ray &r) {
  Intersection isect;
  Spectrum L_out;

  // You will extend this in assignment 3-2.
  // If no intersection occurs, we simply return black.
  // This changes if you implement hemispherical lighting for extra credit.

  if (!bvh->intersect(r, &isect))
    return L_out;

  // The following line of code returns a debug color depending
  // on whether ray intersection with triangles or spheres has
  // been implemented.

  // REMOVE THIS LINE when you are ready to begin Part 3.
//  L_out = (isect.t == INF_D) ? debug_shading(r.d) : normal_shading(isect.n);

  // TODO (Part 3): Return the direct illumination.
//  L_out = one_bounce_radiance(r, isect) + zero_bounce_radiance(r, isect);

  // TODO (Part 4): Accumulate the "direct" and "indirect"
  // parts of global illumination into L_out rather than just direct
  if (max_ray_depth == 0) {
      L_out = zero_bounce_radiance(r, isect);
  } else {
      L_out = at_least_one_bounce_radiance(r, isect) + zero_bounce_radiance(r, isect);
  }
//    L_out = zero_bounce_radiance(r, isect);
//    L_out = at_least_one_bounce_radiance(r, isect);

  return L_out;
}

void PathTracer::raytrace_pixel(size_t x, size_t y) {

    // TODO (Part 1.1):
    // Make a loop that generates num_samples camera rays and traces them
    // through the scene. Return the average Spectrum.
    // You should call est_radiance_global_illumination in this function.
    Spectrum result = Spectrum();
    double s1 = 0;
    double s2 = 0;
    if (ns_aa == 1) {
        Vector2D sample = gridSampler->get_sample();
        Ray sample_ray = camera->generate_ray_for_thin_lens(((double) x + 0.5) / (double) sampleBuffer.w,
                                                            ((double) y + 0.5) / (double) sampleBuffer.h,
                                                            sample[0], sample[1] * 2.0 * PI);
        sample_ray.depth = max_ray_depth;
        result = est_radiance_global_illumination(sample_ray);
        sampleBuffer.update_pixel(Spectrum(result), x, y);
    } else {
        int i;
        for (i = 0; i < ns_aa; i++)
        {
            if (i > 0 && i % samplesPerBatch == 0)
            {
                double mean = s1 / (double) i;
                double var = (s2 - (s1 * s1) / (double) i) / (i - 1.0);
                if (1.96 * sqrt(var / (double) i) <= maxTolerance * mean)
                {
                    break;
                }
            }
            Vector2D sample = gridSampler->get_sample();
            Vector2D sample_2 = gridSampler->get_sample();
            Ray sample_ray = camera->generate_ray_for_thin_lens(((double) x + sample[0]) / (double) sampleBuffer.w,
                                                                ((double) y + sample[1]) / (double) sampleBuffer.h,
                                                                sample_2[0], sample_2[1] * 2.0 * PI);
            sample_ray.depth = max_ray_depth;
            Spectrum tmp = est_radiance_global_illumination(sample_ray);
            result += tmp;
            s1 += tmp.illum();
            s2 += tmp.illum() * tmp.illum();
        }
        sampleCountBuffer[x + y * sampleBuffer.w] = i;
        sampleBuffer.update_pixel(Spectrum(result / (double) i), x, y);
    }

    // TODO (Part 5):
    // Modify your implementation to include adaptive sampling.
    // Use the command line parameters "samplesPerBatch" and "maxTolerance"
}

void PathTracer::autofocus(Vector2D loc) {
    Ray r = camera->generate_ray(loc.x / sampleBuffer.w, loc.y / sampleBuffer.h);
    Intersection isect;
    bvh->intersect(r, &isect);
    camera->focalDistance = isect.t;
}

} // namespace CGL