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

    sample_buffer.resize(width * height * sample_rate, Color::White);
  }

  // Used by rasterize_point and rasterize_line
  void RasterizerImp::fill_pixel(size_t x, size_t y, Color c) {
    size_t offset = (y * width + x) * sample_rate;
    fill_n(sample_buffer.begin() + offset, sample_rate, c);
  }

  // Rasterize a point: simple example to help you start familiarizing
  // yourself with the starter code.
  //
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

  // Rasterize a triangle.
  void RasterizerImp::rasterize_triangle(float x0, float y0,
    float x1, float y1,
    float x2, float y2,
    Color color) {
    rasterize_interpolated_color_triangle(x0, y0, color,
      x1, y1, color, x2, y2, color);
  }


  void RasterizerImp::rasterize_interpolated_color_triangle(float x0, float y0, Color c0,
    float x1, float y1, Color c1,
    float x2, float y2, Color c2)
  {
    int left = max(0, (int)floor(min({x0, x1, x2})));
    int right = min((int)width, (int)ceil(max({x0, x1, x2})));
    int top = max(0, (int)floor(min({y0, y1, y2})));
    int bottom = min((int)height, (int)ceil(max({y0, y1, y2})));
    int side = (int)sqrt(sample_rate);

    Vector2D a(x0, y0), b(x1, y1), c(x2, y2);
    double inverse = 1.0 / cross(b - a, c - a);
    double step = 1.0 / side;

    for (int y = top; y < bottom; ++y) {
      for (int x = left; x < right; ++x) {
        for (int sy = 0; sy < side; ++sy) {
          for (int sx = 0; sx < side; ++sx) {
            Vector2D p(x + (sx + 0.5) * step, y + (sy + 0.5) * step);
            double alpha = cross(b - p, c - p) * inverse;
            double beta = cross(c - p, a - p) * inverse;
            double gamma = cross(a - p, b - p) * inverse;

            if (alpha >= 0 && beta >= 0 && gamma >= 0) {
              Color color = c0 * alpha + c1 * beta + c2 * gamma;
              size_t offset = (y * width + x) * sample_rate + sy * side + sx;
              sample_buffer[offset] = color;
            }
          }
        }
      }
    }
  }


  void RasterizerImp::rasterize_textured_triangle(float x0, float y0, float u0, float v0,
    float x1, float y1, float u1, float v1,
    float x2, float y2, float u2, float v2,
    Texture& tex)
  {
    int left = max(0, (int)floor(min({x0, x1, x2})));
    int right = min((int)width, (int)ceil(max({x0, x1, x2})));
    int top = max(0, (int)floor(min({y0, y1, y2})));
    int bottom = min((int)height, (int)ceil(max({y0, y1, y2})));
    int side = (int)sqrt(sample_rate);

    Vector2D a(x0, y0), b(x1, y1), c(x2, y2);
    Vector2D uv0(u0, v0), uv1(u1, v1), uv2(u2, v2);
    double inverse = 1.0 / cross(b - a, c - a);
    double step = 1.0 / side;
    Vector2D dx = ((b.y - c.y) * uv0 + (c.y - a.y) * uv1
      + (a.y - b.y) * uv2) * inverse;
    Vector2D dy = ((c.x - b.x) * uv0 + (a.x - c.x) * uv1
      + (b.x - a.x) * uv2) * inverse;

    SampleParams sp;
    sp.psm = psm;
    sp.lsm = lsm;

    for (int y = top; y < bottom; ++y) {
      for (int x = left; x < right; ++x) {
        for (int sy = 0; sy < side; ++sy) {
          for (int sx = 0; sx < side; ++sx) {
            Vector2D p(x + (sx + 0.5) * step, y + (sy + 0.5) * step);
            double alpha = cross(b - p, c - p) * inverse;
            double beta = cross(c - p, a - p) * inverse;
            double gamma = cross(a - p, b - p) * inverse;

            if (alpha >= 0 && beta >= 0 && gamma >= 0) {
              sp.p_uv = alpha * uv0 + beta * uv1 + gamma * uv2;
              sp.p_dx_uv = sp.p_uv + dx;
              sp.p_dy_uv = sp.p_uv + dy;

              size_t offset = (y * width + x) * sample_rate + sy * side + sx;
              sample_buffer[offset] = tex.sample(sp);
            }
          }
        }
      }
    }
  }

  void RasterizerImp::set_sample_rate(unsigned int rate) {
    this->sample_rate = rate;


    this->sample_buffer.resize(width * height * sample_rate, Color::White);
  }


  void RasterizerImp::set_framebuffer_target(unsigned char* rgb_framebuffer,
    size_t width, size_t height)
  {
    // TODO: Task 2: You may want to update this function for supersampling support

    this->width = width;
    this->height = height;
    this->rgb_framebuffer_target = rgb_framebuffer;


    this->sample_buffer.resize(width * height * sample_rate, Color::White);
  }


  void RasterizerImp::clear_buffers() {
    std::fill(rgb_framebuffer_target, rgb_framebuffer_target + 3 * width * height, 255);
    std::fill(sample_buffer.begin(), sample_buffer.end(), Color::White);
  }


  // This function is called at the end of rasterizing all elements of the
  // SVG file.  If you use a supersample buffer to rasterize SVG elements
  // for antialising, you could use this call to fill the target framebuffer
  // pixels from the supersample buffer data.
  //
  void RasterizerImp::resolve_to_framebuffer() {
    for (int y = 0; y < height; ++y) {
      for (int x = 0; x < width; ++x) {
        size_t offset = (y * width + x) * sample_rate;
        Color color;
        for (unsigned int s = 0; s < sample_rate; ++s) {
          color += sample_buffer[offset + s];
        }
        color *= 1.0f / sample_rate;

        for (int k = 0; k < 3; ++k) {
          this->rgb_framebuffer_target[3 * (y * width + x) + k] =
            static_cast<unsigned char>(round(color[k] * 255));
        }
      }
    }

  }

  Rasterizer::~Rasterizer() { }


}// CGL
