//
//  marble_texture.hpp
//  JHU_605.667
//
//  Created by Bryan Garry.
//

#ifndef __RAY_TRACER_MARBLE_TEXTURE_HPP__
#define __RAY_TRACER_MARBLE_TEXTURE_HPP__

#include "RayTracer/procedural_texture.hpp"
#include "geometry/noise.hpp"

namespace cg
{

/**
 * Procedural marble texture.
 */
class MarbleTexture : public ProceduralTexture
{
  public:
    /**
     * Constructor.
     * @param base_color    The dominant background color of the marble.
     * @param vein_color    The color of the dark veins running through it.
     * @param vein_freq     Frequency of the vein sine wave. Higher = more veins.
     * @param turb_scale    How strongly turbulence distorts the veins.
     * @param noise_scale   World-space scale fed to the noise function.
     */
    MarbleTexture(const Color3 &base_color,
                  const Color3 &vein_color,
                  float         vein_freq   = 4.0f,
                  float         turb_scale  = 3.0f,
                  float         noise_scale = 0.08f);

    /**
     * Compute marble color at world-space point p.
     * @param  p  World-space intersection point.
     * @return    Interpolated marble color.
     */
    Color3 get_color(const Point3 &p) override;

  private:
    Color3 base_color_;
    Color3 vein_color_;
    float  vein_freq_;
    float  turb_scale_;
    float  noise_scale_;
    Noise  noise_;
};

} // namespace cg

#endif
