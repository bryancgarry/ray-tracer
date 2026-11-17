//
//  checkboard_texture.hpp
//  JHU_605.667
//
//  Created by Bryan Garry.
//

#ifndef __RAY_TRACER_CHECKERBOARD_TEXTURE_HPP__
#define __RAY_TRACER_CHECKERBOARD_TEXTURE_HPP__

#include "RayTracer/procedural_texture.hpp"

namespace cg
{

class CheckerboardTexture : public ProceduralTexture
{
  public:
    /**
     * Constructor
     * @param color1 First checkerboard color
     * @param color2 Second checkerboard color
     * @param scale Size of each checker square
     */
    CheckerboardTexture(const Color3 &color1, const Color3 &color2, float scale = 10.0f);
    
    /**
     * Get the color based on a 3D point
     * @param p Point to compute color
     * @return Returns the color assigned to this point
     */
    Color3 get_color(const Point3 &p) override;
    
  private:
    Color3 color1_;
    Color3 color2_;
    float scale_;
};

} // namespace cg

#endif
