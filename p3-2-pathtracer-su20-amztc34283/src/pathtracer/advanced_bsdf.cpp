#include "bsdf.h"

#include <algorithm>
#include <iostream>
#include <utility>


using std::max;
using std::min;
using std::swap;

namespace CGL {

// Mirror BSDF //

Spectrum MirrorBSDF::f(const Vector3D& wo, const Vector3D& wi) {
  return Spectrum();
}

Spectrum MirrorBSDF::sample_f(const Vector3D& wo, Vector3D* wi, float* pdf) {
  // TODO:
  // Implement MirrorBSDF
  return Spectrum();
}

// Microfacet BSDF //

double MicrofacetBSDF::G(const Vector3D& wo, const Vector3D& wi) {
  return 1.0 / (1.0 + Lambda(wi) + Lambda(wo));
}

double MicrofacetBSDF::D(const Vector3D& h) {
  // TODO: proj3-2, part 3
  // Compute Beckmann normal distribution function (NDF) here.
  // You will need the roughness alpha.
  double cos_h = dot(Vector3D(0, 0, 1), h.unit());
  double theta_h = std::acos(cos_h);
  double numerator = std::exp((-std::tan(theta_h)*std::tan(theta_h))/(alpha * alpha));
  double denominator = PI * alpha * alpha * pow(std::cos(theta_h), 4);
  return numerator / denominator;
}

double calculate_rs(double n, double k, double cos_theta_i) {
    return ((n * n + k * k) - 2.0 * n * cos_theta_i + cos_theta_i * cos_theta_i) / ((n * n + k * k) + 2.0 * n * cos_theta_i + cos_theta_i * cos_theta_i);
}

double calculate_rp(double n, double k, double cos_theta_i) {
    return ((n * n + k * k) * cos_theta_i * cos_theta_i - 2.0 * n * cos_theta_i + 1.0) / ((n * n + k * k) * cos_theta_i * cos_theta_i + 2.0 * n * cos_theta_i + 1.0);
}

double calculate_F(double n, double k, double cos_theta_i) {
    return (calculate_rs(n, k, cos_theta_i) + calculate_rp(n, k, cos_theta_i)) / 2.0;
}

Spectrum MicrofacetBSDF::F(const Vector3D& wi) {
  // TODO: proj3-2, part 3
  // Compute Fresnel term for reflection on dielectric-conductor interface.
  // You will need both eta and etaK, both of which are Spectrum.
  // Fixed wavelength assumption
  // Taking Silver Here: https://refractiveindex.info/
  // 614 nm (red), 549 nm (green) and 466 nm (blue)
  // TODO: reuse eta, k, alpha?
  double n_red = 0.059193;
  double k_red = 4.1283;
  double n_green = 0.059881;
  double k_green = 3.5892;
  double n_blue = 0.047366;
  double k_blue = 2.8132;
  double cos_theta_i = dot(Vector3D(0, 0, 1), wi.unit());
  return Spectrum(calculate_F(eta.x, k.x, cos_theta_i), calculate_F(eta.y, k.y, cos_theta_i), calculate_F(eta.z, k.z, cos_theta_i));
}

Spectrum MicrofacetBSDF::f(const Vector3D& wo, const Vector3D& wi) {
  // TODO: proj3-2, part 3
  // Implement microfacet model here.
  Vector3D norm = Vector3D(0, 0, 1);
  if (dot(norm, wo) <= 0 || dot(norm, wi) <= 0)
      return Spectrum();
  Vector3D half = Vector3D((wi + wo) / (wi + wo).norm());
  return (F(wi) * G(wo, wi) * D(half)) / ((double) 4 * dot(norm, wo) * dot(norm, wi));
}

double theta_pdf(double theta_h, double alpha) {
    return (2.0 * std::sin(theta_h) * std::exp((-std::tan(theta_h)*std::tan(theta_h))/(alpha * alpha))) / (alpha * alpha * pow(std::cos(theta_h), 3));
}

double phi_pdf(double phi_h) {
    return 1.0 / (2.0 * PI);
}

double solid_angle_pdf(double theta_h, double phi_h, double alpha) {
    return theta_pdf(theta_h, alpha) * phi_pdf(phi_h) / std::sin(theta_h);
}

Spectrum MicrofacetBSDF::sample_f(const Vector3D& wo, Vector3D* wi, float* pdf) {
  // TODO: proj3-2, part 3
  // *Importance* sample Beckmann normal distribution function (NDF) here.
  // Note: You should fill in the sampled direction *wi and the corresponding *pdf,
  //       and return the sampled BRDF value.
//  *wi = cosineHemisphereSampler.get_sample(pdf); //placeholder
  Vector2D r1_r2 = sampler.get_sample();
  double theta_h = std::atan(std::sqrt(-alpha * alpha * std::log(1.0 - r1_r2[0])));
  double phi_h = 2.0 * PI * r1_r2[1];
  Vector3D half = Vector3D(std::sin(theta_h) * std::cos(phi_h), std::sin(theta_h) * std::sin(phi_h), std::cos(theta_h)).unit();
  // Reflection wi
  *wi = (2.0 * dot(half, wo) * half - wo).unit();
  if (dot(Vector3D(0, 0, 1), *wi) <= 0) {
      *pdf = 0;
      return Spectrum();
  }
  double solid_wrt_h_pdf_val = solid_angle_pdf(theta_h, phi_h, alpha);
  double solid_wrt_wi_pdf_val = solid_wrt_h_pdf_val / (4.0 * dot(*wi, half));
  *pdf = solid_wrt_wi_pdf_val;
  return MicrofacetBSDF::f(wo, *wi);
}

// Refraction BSDF //

Spectrum RefractionBSDF::f(const Vector3D& wo, const Vector3D& wi) {
  return Spectrum();
}

Spectrum RefractionBSDF::sample_f(const Vector3D& wo, Vector3D* wi, float* pdf) {
  // TODO:
  // Implement RefractionBSDF
  return Spectrum();
}

// Glass BSDF //

Spectrum GlassBSDF::f(const Vector3D& wo, const Vector3D& wi) {
  return Spectrum();
}

Spectrum GlassBSDF::sample_f(const Vector3D& wo, Vector3D* wi, float* pdf) {
  // TODO:
  // Compute Fresnel coefficient and use it as the probability of reflection
  // - Fundamentals of Computer Graphics page 305
  return Spectrum();
}

void BSDF::reflect(const Vector3D& wo, Vector3D* wi) {
  // TODO:
  // Implement reflection of wo about normal (0,0,1) and store result in wi.

}

bool BSDF::refract(const Vector3D& wo, Vector3D* wi, float ior) {
  // TODO:
  // Use Snell's Law to refract wo surface and store result ray in wi.
  // Return false if refraction does not occur due to total internal reflection
  // and true otherwise. When dot(wo,n) is positive, then wo corresponds to a
  // ray entering the surface through vacuum.

  return true;
}

} // namespace CGL
