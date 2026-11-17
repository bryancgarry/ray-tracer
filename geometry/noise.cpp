#include "geometry/noise.hpp"

#include "geometry/geometry.hpp"

namespace cg
{

// NOTE - not required until 605.767!

namespace
{

// Fixed permutation table (Ken Perlin's original values, pulled from the Perlin Noise wiki)
const uint8_t PERM[512] = {
    151,160,137, 91, 90, 15,131, 13,201, 95, 96, 53,194,233,  7,225,
    140, 36,103, 30, 69,142,  8, 99, 37,240, 21, 10, 23,190,  6,148,
    247,120,234, 75,  0, 26,197, 62, 94,252,219,203,117, 35, 11, 32,
     57,177, 33, 88,237,149, 56, 87,174, 20,125,136,171,168, 68,175,
     74,165, 71,134,139, 48, 27,166, 77,146,158,231, 83,111,229,122,
     60,211,133,230,220,105, 92, 41, 55, 46,245, 40,244,102,143, 54,
     65, 25, 63,161,  1,216, 80, 73,209, 76,132,187,208, 89, 18,169,
    200,196,135,130,116,188,159, 86,164,100,109,198,173,186,  3, 64,
     52,217,226,250,124,123,  5,202, 38,147,118,126,255, 82, 85,212,
    207,206, 59,227, 47, 16, 58, 17,182,189, 28, 42,223,183,170,213,
    119,248,152,  2, 44,154,163, 70,221,153,101,155,167, 43,172,  9,
    129, 22, 39,253, 19, 98,108,110, 79,113,224,232,178,185,112,104,
    218,246, 97,228,251, 34,242,193,238,210,144, 12,191,179,162,241,
     81, 51,145,235,249, 14,239,107, 49,192,214, 31,181,199,106,157,
    184, 84,204,176,115,121, 50, 45,127,  4,150,254,138,236,205, 93,
    222,114, 67, 29, 24, 72,243,141,128,195, 78, 66,215, 61,156,180,
    // Second half mirrors the first for easy wrapping
    151,160,137, 91, 90, 15,131, 13,201, 95, 96, 53,194,233,  7,225,
    140, 36,103, 30, 69,142,  8, 99, 37,240, 21, 10, 23,190,  6,148,
    247,120,234, 75,  0, 26,197, 62, 94,252,219,203,117, 35, 11, 32,
     57,177, 33, 88,237,149, 56, 87,174, 20,125,136,171,168, 68,175,
     74,165, 71,134,139, 48, 27,166, 77,146,158,231, 83,111,229,122,
     60,211,133,230,220,105, 92, 41, 55, 46,245, 40,244,102,143, 54,
     65, 25, 63,161,  1,216, 80, 73,209, 76,132,187,208, 89, 18,169,
    200,196,135,130,116,188,159, 86,164,100,109,198,173,186,  3, 64,
     52,217,226,250,124,123,  5,202, 38,147,118,126,255, 82, 85,212,
    207,206, 59,227, 47, 16, 58, 17,182,189, 28, 42,223,183,170,213,
    119,248,152,  2, 44,154,163, 70,221,153,101,155,167, 43,172,  9,
    129, 22, 39,253, 19, 98,108,110, 79,113,224,232,178,185,112,104,
    218,246, 97,228,251, 34,242,193,238,210,144, 12,191,179,162,241,
     81, 51,145,235,249, 14,239,107, 49,192,214, 31,181,199,106,157,
    184, 84,204,176,115,121, 50, 45,127,  4,150,254,138,236,205, 93,
    222,114, 67, 29, 24, 72,243,141,128,195, 78, 66,215, 61,156,180
};

// Smoothstep: maps t in [0,1] to a smooth S-curve.
// Eliminates visible grid artifacts at lattice cell boundaries.
inline float fade(float t)
{
    return t * t * (3.0f - 2.0f * t);
}

// Linear interpolation
inline float lerp(float a, float b, float t)
{
    return a + t * (b - a);
}

// Pseudorandom float in [0, 1] from integer lattice coordinates
inline float lattice_value(int ix, int iy, int iz)
{
    int idx = PERM[(PERM[(PERM[ix & 255] + iy) & 255] + iz) & 255];
    return static_cast<float>(idx) / 255.0f;
}

} // namespace

Noise::Noise() {}

float Noise::noise(const Point3 &p, float scale)
{
    // Scale the input point into lattice space
    float sx = p.x * scale;
    float sy = p.y * scale;
    float sz = p.z * scale;

    // Integer lattice cell
    int ix = static_cast<int>(std::floor(sx));
    int iy = static_cast<int>(std::floor(sy));
    int iz = static_cast<int>(std::floor(sz));

    // Fractional position within cell
    float fx = sx - static_cast<float>(ix);
    float fy = sy - static_cast<float>(iy);
    float fz = sz - static_cast<float>(iz);

    // Smoothstep weights for each axis
    float ux = fade(fx);
    float uy = fade(fy);
    float uz = fade(fz);

    // Sample all 8 corners of the unit cube
    float v000 = lattice_value(ix,     iy,     iz    );
    float v100 = lattice_value(ix + 1, iy,     iz    );
    float v010 = lattice_value(ix,     iy + 1, iz    );
    float v110 = lattice_value(ix + 1, iy + 1, iz    );
    float v001 = lattice_value(ix,     iy,     iz + 1);
    float v101 = lattice_value(ix + 1, iy,     iz + 1);
    float v011 = lattice_value(ix,     iy + 1, iz + 1);
    float v111 = lattice_value(ix + 1, iy + 1, iz + 1);

    // Trilinear interpolation
    float x00 = lerp(v000, v100, ux);
    float x10 = lerp(v010, v110, ux);
    float x01 = lerp(v001, v101, ux);
    float x11 = lerp(v011, v111, ux);

    float y0  = lerp(x00, x10, uy);
    float y1  = lerp(x01, x11, uy);

    return lerp(y0, y1, uz);
}

float Noise::turbulence(float scale, const Point3 &p)
{
    // Sum 6 octaves of noise, each at double the frequency and half the amplitude of the previous
    float result    = 0.0f;
    float amplitude = 1.0f;
    float frequency = scale;
    float total_amp = 0.0f;

    for (int octave = 0; octave < 6; ++octave)
    {
        float n = noise(p, frequency);  // n in [0, 1]
        result    += std::abs(n - 0.5f) * amplitude;
        total_amp += 0.5f * amplitude;  // max contribution per octave
        amplitude *= 0.5f;
        frequency *= 2.0f;
    }

    // Normalize to [0, 1]
    return (result / total_amp);
}

} // namespace cg
