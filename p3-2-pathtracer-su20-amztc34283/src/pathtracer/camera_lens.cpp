#include "camera.h"

#include <iostream>
#include <sstream>
#include <fstream>

#include "CGL/misc.h"
#include "CGL/vector2D.h"
#include "CGL/vector3D.h"

using std::cout;
using std::endl;
using std::max;
using std::min;
using std::ifstream;
using std::ofstream;

namespace CGL {

using Collada::CameraInfo;

Ray Camera::generate_ray_for_thin_lens(double x, double y, double rndR, double rndTheta) const {
  // Part 2, Task 4:
  // compute position and direction of ray from the input sensor sample coordinate.
  // Note: use rndR and rndTheta to uniformly sample a unit disk.
  double hFov_rad = hFov * (double) 2 * PI / (double) 360;
  double vFov_rad = vFov * (double) 2 * PI / (double) 360;

  // From image-space to camera-space
  Matrix3x3 img_to_cam = Matrix3x3( (double) 2 * tan(hFov_rad * 0.5), 0, - tan(hFov_rad * 0.5),
                                      0, (double) 2 * tan(vFov_rad * 0.5), - tan(vFov_rad * 0.5),
                                      0, 0, -1);
  Vector3D camera_space = img_to_cam * Vector3D(x, y, 1); // homogenous coordinate

  Vector3D pLens = Vector3D(lensRadius * std::sqrt(rndR) * std::cos(2 * PI * rndTheta), lensRadius * std::sqrt(rndR) * std::sin(2 * PI * rndTheta), 0);
  Vector3D red_dir = Vector3D(camera_space.x, camera_space.y, -1).unit();
//  std::cout << focalDistance << std::endl;
  double t = -focalDistance / red_dir.z;
  Vector3D pFocus = t * red_dir;

  Vector3D ray_from_sample = pFocus - pLens;
  Vector3D world_space = c2w * ray_from_sample.unit();
  Ray result = Ray(pos + c2w * pLens, world_space.unit(), fClip);
  result.min_t = nClip;
  return result;
}


} // namespace CGL
