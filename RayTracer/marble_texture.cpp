//
//  marble_texture.cpp
//  JHU_605.667
//
//  Created by Bryan Garry.
//

#include "RayTracer/marble_texture.hpp"
#include <cmath>

namespace cg
{

MarbleTexture::MarbleTexture(const Color3 &base_color,
                             const Color3 &vein_color,
                             float         vein_freq,
                             float         turb_scale,
                             float         noise_scale)
    : base_color_(base_color),
      vein_color_(vein_color),
      vein_freq_(vein_freq),
      turb_scale_(turb_scale),
      noise_scale_(noise_scale)
{
}

Color3 MarbleTexture::get_color(const Point3 &p)
{
    // Compute turbulence at this world space point.
    float turb = noise_.turbulence(noise_scale_, p);

    // Feed the turbulence distorted position into a sin wave.
    float val = std::sin(vein_freq_ * p.y + turb_scale_ * turb);

    // Map sin output from [-1, 1] to [0, 1]
    val = (val + 1.0f) * 0.5f;

    // Soften vein edges
    val = val * val * (3.0f - 2.0f * val);

    // Linearly interpolate between base color and vein color
    Color3 result;
    result.r = base_color_.r + val * (vein_color_.r - base_color_.r);
    result.g = base_color_.g + val * (vein_color_.g - base_color_.g);
    result.b = base_color_.b + val * (vein_color_.b - base_color_.b);
    return result;
}

} // namespace cg
