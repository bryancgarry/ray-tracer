//
//  checkboard_texture.cpp
//  JHU_605.667
//
//  Created by Bryan Garry.
//

#include "RayTracer/checkerboard_texture.hpp"
#include <cmath>

namespace cg
{

CheckerboardTexture::CheckerboardTexture(const Color3 &color1, const Color3 &color2, float scale)
    : color1_(color1), color2_(color2), scale_(scale)
{
}

Color3 CheckerboardTexture::get_color(const Point3 &p)
{
    // Use division so scale represents the world-space size of one square
    // Add a tiny epsilon to keep edges clean
    int x_grid = static_cast<int>(std::floor((p.x + 0.00001f) / scale_));
    int y_grid = static_cast<int>(std::floor((p.y + 0.00001f) / scale_));
    
    return ((x_grid + y_grid) % 2 == 0) ? color1_ : color2_;
}

} // namespace cg

