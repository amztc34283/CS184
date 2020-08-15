#include "rasterizer.h"

using namespace std;

namespace CGL {

RasterizerImp::RasterizerImp(PixelSampleMethod psm, LevelSampleMethod lsm,
                                       size_t width, size_t height,
                                       unsigned int sample_rate) {
  this->psm = psm;
  this->lsm = lsm;
  this->width = width;
  this->height = height;
  this->sample_rate = sample_rate;

  supersample_buffer.resize(width * height * sample_rate, Color::White);
}

// Used by rasterize_point and rasterize_line
void RasterizerImp::fill_pixel(size_t x, size_t y, Color c) {
  // TODO: Task 2: You need to change this function to fix points and lines (such as the black rectangle border in test4.svg)
  // NOTE: You are not required to implement proper supersampling for points and lines
  // It is sufficient to use the same color for all supersamples of a pixel for points and lines (not triangles)
  // NOTE: You should not use this function to fill the framebuffer pixel! In task 2, you should change this function so that
  // it renders to the internal buffer instead of the framebuffer!

  rgb_framebuffer_target[3 * (y * width + x)    ] = (unsigned char)(c.r * 255);
  rgb_framebuffer_target[3 * (y * width + x) + 1] = (unsigned char)(c.g * 255);
  rgb_framebuffer_target[3 * (y * width + x) + 2] = (unsigned char)(c.b * 255);
}

// Optional helper function to add a sample to the supersample_buffer
void RasterizerImp::fill_supersample(size_t x, size_t y, size_t s, Color c) {
  // TODO: Task 2: You may want to implement this function. Hint: our solution uses one line
  supersample_buffer[width * y * sample_rate + x * sample_rate + s] = c;
};

// Rasterize a point: simple example to help you start familiarizing
// yourself with the starter code.
void RasterizerImp::rasterize_point(float x, float y, Color color) {
  // fill in the nearest pixel
  int sx = (int)floor(x);
  int sy = (int)floor(y);

  // check bounds
  if (sx < 0 || sx >= width) return;
  if (sy < 0 || sy >= height) return;

  fill_pixel(sx, sy, color);
  return;
}

// Rasterize a line.
void RasterizerImp::rasterize_line(float x0, float y0,
  float x1, float y1,
  Color color) {
  if (x0 > x1) {
    swap(x0, x1); swap(y0, y1);
  }

  float pt[] = { x0,y0 };
  float m = (y1 - y0) / (x1 - x0);
  float dpt[] = { 1,m };
  int steep = abs(m) > 1;
  if (steep) {
    dpt[0] = x1 == x0 ? 0 : 1 / abs(m);
    dpt[1] = x1 == x0 ? (y1 - y0) / abs(y1 - y0) : m / abs(m);
  }

  while (floor(pt[0]) <= floor(x1) && abs(pt[1] - y0) <= abs(y1 - y0)) {
    rasterize_point(pt[0], pt[1], color);
    pt[0] += dpt[0]; pt[1] += dpt[1];
  }
}

// Check if the point is with in the triangle
bool in_triangle(float x0, float y0,
                                float x1, float y1,
                                float x2, float y2,
                                float x, float y) {

    float edgeA = - (x - x0) * (y1 - y0) + (y - y0) * (x1 - x0);
    float edgeB = - (x - x1) * (y2 - y1) + (y - y1) * (x2 - x1);
    float edgeC = - (x - x2) * (y0 - y2) + (y - y2) * (x0 - x2);

    // Check the winding order - order should not affect
    float winding = - (x2 - x0) * (y1 - y0) + (y2 - y0) * (x1 - x0);

    // TODO: Work on Edge Case
    if (winding > 0)
        return (edgeA >= -1E-6 && edgeB >= -1E-6 && edgeC >= -1E-6) ? true : false;
    return (edgeA <= 1E-6 && edgeB <= 1E-6 && edgeC <= 1E-6) ? true : false;
}

// Rasterize a triangle.
void RasterizerImp::rasterize_triangle(float x0, float y0,
                                       float x1, float y1,
                                       float x2, float y2,
                                       Color color) {
  // Task 1: Implement basic triangle rasterization here, no supersampling
  // Optimization - Check the bounding box rather than all pixels
  int max_x = (int) ceil(max(max(x0, x1), x2));
  int min_x = (int) floor(min(min(x0, x1), x2));
  int max_y = (int) ceil(max(max(y0, y1), y2));
  int min_y = (int) floor(min(min(y0, y1), y2));

    if (sample_rate == 1) {
        for (int i = min_x; i < max_x; i++) {
            for (int j = min_y; j < max_y; j++) {
                if ((i >= 0 && i < width) &&
                    (j >= 0 && j < height) &&
                    // check bounds - the width and height is relative to the zoomed box
                    in_triangle(x0, y0, x1, y1, x2, y2, (float) i + 0.5, (float) j + 0.5)) {
                    // fill_pixel takes in int coordinate
                    fill_pixel(i, j, color);
                }
            }
        }
    } else {
        // Task 2: Update to implement super-sampled rasterization
        // Check if the supersamples' centers are within the triangle for higher granularity
        for (int i = min_x; i < max_x; i++) {
            for (int j = min_y; j < max_y; j++) {
                if ((i >= 0 && i < width) && (j >= 0 && j < height)) {
                    // check if centers of supersamples are inside the triangle
                    float incre = 1/std::sqrt(sample_rate); // Might introduce numerical error?
                    for (int q = 0; q < std::sqrt(sample_rate); q++) {
                        for (int p = 0; p < std::sqrt(sample_rate); p++) {
                            if (in_triangle(x0, y0, x1, y1, x2, y2,
                                            (float) i + incre/2 + p * incre,
                                            (float) j + incre/2 + q * incre)) {
                                fill_supersample(i, j, p + q * std::sqrt(sample_rate), color);
                            }
                        }
                    }
                }
            }
        }
    }
}

// alpha for (x0, y0), beta for (x1, y1), gamma for (x2, y2)
std::vector<float> compute_barycentric(float x0, float y0,
                                       float x1, float y1,
                                       float x2, float y2,
                                       float x, float y)
{
    // Do not need to handle the winding problem
    float alpha = (- (x - x1) * (y2 - y1) + (y - y1) * (x2 - x1)) /
                  (- (x0 - x1) * (y2 - y1) + (y0 - y1) * (x2 - x1));
    float beta = (- (x - x2) * (y0 - y2) + (y - y2) * (x0 - x2)) /
                 (- (x1 - x2) * (y0 - y2) + (y1 - y2) * (x0 - x2));
    float gamma = (- (x - x0) * (y1 - y0) + (y - y0) * (x1 - x0)) /
                  (- (x2 - x0) * (y1 - y0) + (y2 - y0) * (x1 - x0));
    return std::vector<float> {alpha, beta, gamma};
}

void RasterizerImp::rasterize_interpolated_color_triangle(float x0, float y0, Color c0,
                                                          float x1, float y1, Color c1,
                                                          float x2, float y2, Color c2)
{
  // TODO: Task 4: Rasterize the triangle, calculating barycentric coordinates and using them to interpolate vertex colors across the triangle
  // Hint: You can reuse code from rasterize_triangle
  // Hint: Can you render a normal single colored triangle using this function?
    int max_x = (int) ceil(max(max(x0, x1), x2));
    int min_x = (int) floor(min(min(x0, x1), x2));
    int max_y = (int) ceil(max(max(y0, y1), y2));
    int min_y = (int) floor(min(min(y0, y1), y2));

    if (sample_rate == 1) {
        for (int i = min_x; i < max_x; i++) {
            for (int j = min_y; j < max_y; j++) {
                if ((i >= 0 && i < width) &&
                    (j >= 0 && j < height) &&
                    // check bounds - the width and height is relative to the zoomed box
                    in_triangle(x0, y0, x1, y1, x2, y2, (float) i + 0.5, (float) j + 0.5)) {
                    // Calculate Barycentric Coordinated Color
                    std::vector<float> bary_coords = compute_barycentric(x0, y0,
                                                                         x1, y1,
                                                                         x2, y2,
                                                                         (float) i + 0.5, (float) j + 0.5);
                    // fill_pixel takes in int coordinate
                    fill_pixel(i, j, c0 * bary_coords[0] + c1 * bary_coords[1] + c2 * bary_coords[2]);
                }
            }
        }
    } else {
        // Check if the supersamples' centers are within the triangle for higher granularity
        for (int i = min_x; i < max_x; i++) {
            for (int j = min_y; j < max_y; j++) {
                if ((i >= 0 && i < width) && (j >= 0 && j < height)) {
                    // check if centers of supersamples are inside the triangle
                    float incre = 1/std::sqrt(sample_rate); // Might introduce numerical error?
                    for (int q = 0; q < std::sqrt(sample_rate); q++) {
                        for (int p = 0; p < std::sqrt(sample_rate); p++) {
                            if (in_triangle(x0, y0, x1, y1, x2, y2,
                                            (float) i + incre/2 + p * incre,
                                            (float) j + incre/2 + q * incre)) {
                                // Calculate Barycentric Coordinated Color
                                std::vector<float> bary_coords =
                                        compute_barycentric(x0, y0, x1, y1, x2, y2,
                                                            (float) i + incre/2 + p * incre,
                                                            (float) j + incre/2 + q * incre);
                                fill_supersample(i, j,
                                                 p + q * std::sqrt(sample_rate),
                                                 c0 * bary_coords[0] + c1 * bary_coords[1] + c2 * bary_coords[2]);
                            }
                        }
                    }
                }
            }
        }
    }

}


void RasterizerImp::rasterize_textured_triangle(float x0, float y0, float u0, float v0,
                                                     float x1, float y1, float u1, float v1,
                                                     float x2, float y2, float u2, float v2,
                                                     Texture &tex)
{
  // TODO: Task 5: Fill in the SampleParams struct and pass it to the tex.sample function.
  // TODO: Task 6: Set the correct barycentric differentials in the SampleParams struct.
  // Hint: You can reuse code from rasterize_triangle/rasterize_interpolated_color_triangle
    int max_x = (int) ceil(max(max(x0, x1), x2));
    int min_x = (int) floor(min(min(x0, x1), x2));
    int max_y = (int) ceil(max(max(y0, y1), y2));
    int min_y = (int) floor(min(min(y0, y1), y2));

    if (sample_rate == 1) {
        for (int i = min_x; i < max_x; i++) {
            for (int j = min_y; j < max_y; j++) {
                if ((i >= 0 && i < width) &&
                    (j >= 0 && j < height) &&
                    // check bounds - the width and height is relative to the zoomed box
                    in_triangle(x0, y0, x1, y1, x2, y2, (float) i + 0.5, (float) j + 0.5)) {
                    // Calculate Barycentric Coordinated Color
                    std::vector<float> bary_coords = compute_barycentric(x0, y0,
                                                                         x1, y1,
                                                                         x2, y2,
                                                                         (float) i + 0.5, (float) j + 0.5);
                    std::vector<float> bary_coords_xplus = compute_barycentric(x0, y0,
                                                                               x1, y1,
                                                                               x2, y2,
                                                                               (float) i + 1.5, (float) j + 0.5);
                    std::vector<float> bary_coords_yplus = compute_barycentric(x0, y0,
                                                                               x1, y1,
                                                                               x2, y2,
                                                                               (float) i + 0.5, (float) j + 1.5);

                    float final_u = bary_coords[0] * u0
                                    + bary_coords[1] * u1
                                    + bary_coords[2] * u2;
                    float final_v = bary_coords[0] * v0
                                    + bary_coords[1] * v1
                                    + bary_coords[2] * v2;
                    float dx_u = bary_coords_xplus[0] * u0
                                 + bary_coords_xplus[1] * u1
                                 + bary_coords_xplus[2] * u2;
                    float dx_v = bary_coords_xplus[0] * v0
                                 + bary_coords_xplus[1] * v1
                                 + bary_coords_xplus[2] * v2;
                    float dy_u = bary_coords_yplus[0] * u0
                                 + bary_coords_yplus[1] * u1
                                 + bary_coords_yplus[2] * u2;
                    float dy_v = bary_coords_yplus[0] * v0
                                 + bary_coords_yplus[1] * v1
                                 + bary_coords_yplus[2] * v2;
                    SampleParams params = { Vector2D(final_u, final_v), // uv
                                            Vector2D(dx_u, dx_v), // dx_uv
                                            Vector2D(dy_u, dy_v), // dy_uv
                                            psm, lsm };
                    // fill_pixel takes in int coordinate
                    if (final_u * (tex.width - 1) >= 0 && final_u * (tex.width - 1) < tex.width
                        && final_v * (tex.height - 1) >= 0 && final_v * (tex.height - 1) < tex.height
                        && dx_u * (tex.width - 1) >= 0 && dx_u * (tex.width - 1) < tex.width
                        && dx_v * (tex.height - 1) >= 0 && dx_v * (tex.height - 1) < tex.height
                        && dy_u * (tex.width - 1) >= 0 && dy_u * (tex.width - 1) < tex.width
                        && dy_v * (tex.height - 1) >= 0 && dy_v * (tex.height - 1) < tex.height)
                        fill_pixel(i, j, tex.sample(params));
                }
            }
        }
    } else {
        // Check if the supersamples' centers are within the triangle for higher granularity
        for (int i = min_x; i < max_x; i++) {
            for (int j = min_y; j < max_y; j++) {
                if ((i >= 0 && i < width) && (j >= 0 && j < height)) {
                    // check if centers of supersamples are inside the triangle
                    float incre = 1/std::sqrt(sample_rate); // Might introduce numerical error?
                    for (int q = 0; q < std::sqrt(sample_rate); q++) {
                        for (int p = 0; p < std::sqrt(sample_rate); p++) {
                            if (in_triangle(x0, y0, x1, y1, x2, y2,
                                            (float) i + incre/2 + p * incre,
                                            (float) j + incre/2 + q * incre)) {
                                // Calculate Barycentric Coordinated Color
                                std::vector<float> bary_coords =
                                        compute_barycentric(x0, y0, x1, y1, x2, y2,
                                                            (float) i + incre/2 + p * incre,
                                                            (float) j + incre/2 + q * incre);
                                std::vector<float> bary_coords_xplus =
                                        compute_barycentric(x0, y0, x1, y1, x2, y2,
                                                            (float) i + 1 + incre/2 + p * incre,
                                                            (float) j + incre/2 + q * incre);
                                std::vector<float> bary_coords_yplus =
                                        compute_barycentric(x0, y0, x1, y1, x2, y2,
                                                            (float) i + incre/2 + p * incre,
                                                            (float) j + 1 + incre/2 + q * incre);

                                float final_u = bary_coords[0] * u0
                                                + bary_coords[1] * u1
                                                + bary_coords[2] * u2;
                                float final_v = bary_coords[0] * v0
                                                + bary_coords[1] * v1
                                                + bary_coords[2] * v2;
                                float dx_u = bary_coords_xplus[0] * u0
                                             + bary_coords_xplus[1] * u1
                                             + bary_coords_xplus[2] * u2;
                                float dx_v = bary_coords_xplus[0] * v0
                                             + bary_coords_xplus[1] * v1
                                             + bary_coords_xplus[2] * v2;
                                float dy_u = bary_coords_yplus[0] * u0
                                             + bary_coords_yplus[1] * u1
                                             + bary_coords_yplus[2] * u2;
                                float dy_v = bary_coords_yplus[0] * v0
                                             + bary_coords_yplus[1] * v1
                                             + bary_coords_yplus[2] * v2;
                                SampleParams params = { Vector2D(final_u, final_v), // uv
                                                        Vector2D(dx_u, dx_v), // dx_uv
                                                        Vector2D(dy_u, dy_v), // dy_uv
                                                        psm, lsm };
                                // fill_pixel takes in int coordinate
                                if (final_u * (tex.width - 1) >= 0 && final_u * (tex.width - 1) < tex.width
                                    && final_v * (tex.height - 1) >= 0 && final_v * (tex.height - 1) < tex.height
                                    && dx_u * (tex.width - 1) >= 0 && dx_u * (tex.width - 1) < tex.width
                                    && dx_v * (tex.height - 1) >= 0 && dx_v * (tex.height - 1) < tex.height
                                    && dy_u * (tex.width - 1) >= 0 && dy_u * (tex.width - 1) < tex.width
                                    && dy_v * (tex.height - 1) >= 0 && dy_v * (tex.height - 1) < tex.height)
                                    fill_supersample(i, j,
                                                     p + q * std::sqrt(sample_rate),
                                                     tex.sample(params));
                            }
                        }
                    }
                }
            }
        }
    }


}

void RasterizerImp::set_sample_rate(unsigned int rate) {
  // TODO: Task 2: You may want to update this function for supersampling support
  // HINT: Different sampleing rate means you need different amount of space to store the samples

  this->sample_rate = rate;

    // Initialize the supersample buffer with correct size
    if (rate > 1)
        this->supersample_buffer = std::vector<Color>(width * height * rate);
}


void RasterizerImp::set_framebuffer_target( unsigned char* rgb_framebuffer,
                                                size_t width, size_t height )
{
  // TODO: Task 2: You may want to update this function for supersampling support

  this->width = width;
  this->height = height;
  this->rgb_framebuffer_target = rgb_framebuffer;

    // Change the size based on user
    if (sample_rate > 1)
        this->supersample_buffer.resize(width * height * sample_rate);

}


void RasterizerImp::clear_buffers() {
  // TODO: Task 2: You may want to update this function for supersampling support
  // Hint: With supersampling, you have an additional buffer to take care of

  std::fill(rgb_framebuffer_target, rgb_framebuffer_target + 3 * width * height, 255);

    // Fill supersamling buffer as well
    if (sample_rate > 1)
        std::fill(supersample_buffer.begin(), supersample_buffer.end(), Color::White);

}


// This function is called at the end of rasterizing all elements of the
// SVG file.  If you use a supersample buffer to rasterize SVG elements
// for antialising, you could use this call to fill the target framebuffer
// pixels from the supersample buffer data.
void RasterizerImp::resolve_to_framebuffer() {
  // TODO: Task 2: You will likely want to update this function for supersampling support
  // NOTE: Check the rendering pipeline description in task 2 specs.
    if (sample_rate > 1) {
        // Supersampling buffer to frame buffer
        for (int i = 0; i < height; i++) {
            for (int j = 0; j < width; j++) {
                float pixel_r = 0;
                float pixel_g = 0;
                float pixel_b = 0;
                for (int k = 0; k < sample_rate; k++) {
                    pixel_r += supersample_buffer[i * width * sample_rate + sample_rate * j + k].r;
                    pixel_g += supersample_buffer[i * width * sample_rate + sample_rate * j + k].g;
                    pixel_b += supersample_buffer[i * width * sample_rate + sample_rate * j + k].b;
                }
                fill_pixel(j, i, Color(pixel_r/sample_rate, pixel_g/sample_rate, pixel_b/sample_rate));
            }
        }
    }
}

Rasterizer::~Rasterizer() { }


}// CGL
