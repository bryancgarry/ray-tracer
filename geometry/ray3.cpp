#include "geometry/ray3.hpp"

#include "geometry/geometry.hpp"

#include <cmath>
#include <limits>

namespace cg
{

Ray3::Ray3() : o{0.0f, 0.0f, 0.0f}, d{1.0f, 0.0f, 0.0f} {}

Ray3::Ray3(const Point3 &p1, const Point3 &p2, bool normalize) : o(p1), d(p1, p2)
{
    if(normalize) d.normalize();
}

Ray3::Ray3(const Point3 &origin, const Vector3 &dir) : o(origin), d(dir) {}

Ray3::Ray3(const Point3 &origin, const Vector3 &dir, bool normalize)
{
    o = origin;
    d = dir;
    if(normalize) d.normalize();
}

Point3 Ray3::intersect(const float t) const { return o + d * t; }

Ray3 Ray3::reflect(const Point3 &int_pt, const Vector3 &n) const
{
    Vector3 r = d.reflect(n);
    r.normalize();
    return Ray3(int_pt, r);
}

// Refract (Snell's law, vector form)
// u1 = index of refraction of medium being left
// u2 = index of refraction of medium being entered
// n  = surface normal pointing into the medium being left
RayRefractionResult Ray3::refract(const Point3 &int_pt, Vector3 &n, float u1, float u2) const
{
    float ratio = u1 / u2;

    // Cos of incident angle, flip normal if ray is inside the object
    float cos_i = -(d.dot(n));
    if(cos_i < 0.0f)
    {
        n = n * -1.0f; // flip normal
        cos_i = -cos_i;
    }

    // Sin squared of transmitted angle via Snell's law
    float sin2_t = ratio * ratio * (1.0f - cos_i * cos_i);

    if(sin2_t > 1.0f)
    {
        // Total internal reflection, return the reflected ray
        Ray3 reflected = reflect(int_pt, n);
        return RayRefractionResult{reflected, true};
    }

    float cos_t = std::sqrt(1.0f - sin2_t);

    // Transmitted direction: ratio * d + (ratio * cos_i - cos_t) * n
    Vector3 refracted_dir = d * ratio + n * (ratio * cos_i - cos_t);
    refracted_dir.normalize();

    return RayRefractionResult{Ray3(int_pt, refracted_dir), false};
}

// Ray / Plane intersection
RayObjectIntersectResult Ray3::intersect(const Plane &p) const
{
    Vector3 n = p.get_normal();
    float n_d = n.dot(d);

    // Ray is close to parallel to the plane
    if(std::abs(n_d) < EPSILON)
        return {false, 0.0f};

    // Signed distance of ray origin from the plane
    float t = p.solve(o) / (-n_d);

    if(t < 0.0f)
        return {false, 0.0f}; // Intersection is behind the ray origin

    return {true, t};
}

// Ray / BoundingSphere intersection
RayObjectIntersectResult Ray3::intersect(const BoundingSphere &sphere) const
{
    Vector3 l  = sphere.center - o;
    float   l2 = l.norm_squared();
    float   s  = l.dot(d);
    float   r2 = sphere.radius * sphere.radius;

    if(s < 0.0f && l2 > r2)
        return {false, 0.0f};

    float m2 = l2 - s * s;
    if(m2 > r2)
        return {false, 0.0f};

    float q = std::sqrt(r2 - m2);

    if(l2 - r2 > EPSILON)
        return {true, s - q}; // ray origin outside sphere
    else
        return {true, s + q}; // ray origin inside sphere
}

// Ray / AABB intersection  (slab method)
// For each axis compute the two t values where the ray crosses
// the two bounding planes. The intersection of all three slab
// intervals gives the overlap. If t_near < t_far the ray hits.
RayObjectIntersectResult Ray3::intersect(const AABB &box) const
{
    float t_near = -std::numeric_limits<float>::max();
    float t_far  =  std::numeric_limits<float>::max();

    // Lambda to process one axis slab
    auto process_slab = [&](float o_comp, float d_comp, float min_comp, float max_comp) -> bool
    {
        if(std::abs(d_comp) < EPSILON)
        {
            // Ray is parallel to this slab, check if origin is inside
            if(o_comp < min_comp || o_comp > max_comp)
            {
                return false; // parallel and outside
            }
            else
            {
                return true;
            }
        }
        float inv = 1.0f / d_comp;
        float t1 = (min_comp - o_comp) * inv;
        float t2 = (max_comp - o_comp) * inv;
        if(t1 > t2) std::swap(t1, t2);
        t_near = std::max(t_near, t1);
        t_far = std::min(t_far,  t2);
        return (t_near <= t_far);
    };

    if(!process_slab(o.x, d.x, box.min.x, box.max.x)) return {false, 0.0f};
    if(!process_slab(o.y, d.y, box.min.y, box.max.y)) return {false, 0.0f};
    if(!process_slab(o.z, d.z, box.min.z, box.max.z)) return {false, 0.0f};

    if(t_far < 0.0f)
        return {false, 0.0f}; // Box is entirely behind the ray

    float t = (t_near >= 0.0f) ? t_near : t_far;
    return {true, t};
}

// Ray / Polygon intersection
RayObjectIntersectResult Ray3::intersect(const std::vector<Point3> &polygon,
                                         const Vector3             &normal) const
{
    if(polygon.empty())
        return {false, 0.0f};

    // Build plane from the first vertex and the normal
    Plane plane(polygon[0], normal);

    auto plane_result = intersect(plane);
    if(!plane_result.intersects)
        return {false, 0.0f};

    // Point on the polygon plane
    Point3 pt = intersect(plane_result.distance);

    // Test containment
    if(!pt.is_in_polygon(polygon, normal))
        return {false, 0.0f};

    return {true, plane_result.distance};
}

// Ray / Triangle intersection  (Moller–Trumbore algorithm)
RayTriangleIntersectResult
    Ray3::intersect(const Point3 &v0, const Point3 &v1, const Point3 &v2) const
{
    Vector3 e1(v0, v1);  // v1 - v0
    Vector3 e2(v0, v2);  // v2 - v0

    Vector3 h = d.cross(e2);
    float   a = e1.dot(h);

    // Ray is parallel to the triangle
    if(std::abs(a) < EPSILON)
        return {false, 0.0f, 0.0f, 0.0f};

    float   inv_a = 1.0f / a;
    Vector3 s(v0, o);   // o - v0

    float u = inv_a * s.dot(h);
    if(u < 0.0f || u > 1.0f)
        return {false, 0.0f, 0.0f, 0.0f};

    Vector3 q = s.cross(e1);
    float   v = inv_a * d.dot(q);
    if(v < 0.0f || u + v > 1.0f)
        return {false, 0.0f, 0.0f, 0.0f};

    float t = inv_a * e2.dot(q);
    if(t < EPSILON)
        return {false, 0.0f, 0.0f, 0.0f}; // Intersection behind ray origin

    return {true, t, u, v};
}

// Does a ray/triangle intersection exist? (fast version)
bool Ray3::does_intersect_exist(const Point3 &v0, const Point3 &v1, const Point3 &v2) const
{
    return intersect(v0, v1, v2).intersects;
}

// Ray / Triangle mesh intersection
RayMeshIntersectResult Ray3::intersect(const std::vector<Point3>   &vertex_list,
                                       const std::vector<uint16_t> &face_list,
                                       float                        t_min) const
{
    RayMeshIntersectResult best{false, t_min, 0.0f, 0.0f, 0};

    uint32_t face_count = static_cast<uint32_t>(face_list.size() / 3);
    for(uint32_t i = 0; i < face_count; ++i)
    {
        const Point3 &v0 = vertex_list[face_list[i * 3    ]];
        const Point3 &v1 = vertex_list[face_list[i * 3 + 1]];
        const Point3 &v2 = vertex_list[face_list[i * 3 + 2]];

        auto result = intersect(v0, v1, v2);
        if(result.intersects && result.distance < best.distance)
        {
            best.intersects    = true;
            best.distance      = result.distance;
            best.barycentric_u = result.barycentric_u;
            best.barycentric_v = result.barycentric_v;
            best.face_index    = i;
        }
    }

    return best;
}

// Does intersection exist in mesh (early exit, no normal lookup)
bool Ray3::does_intersect_exist(const std::vector<Point3>   &vertex_list,
                                const std::vector<uint16_t> &face_list,
                                float                        t_min) const
{
    uint32_t face_count = static_cast<uint32_t>(face_list.size() / 3);
    for(uint32_t i = 0; i < face_count; ++i)
    {
        const Point3 &v0 = vertex_list[face_list[i * 3    ]];
        const Point3 &v1 = vertex_list[face_list[i * 3 + 1]];
        const Point3 &v2 = vertex_list[face_list[i * 3 + 2]];

        auto result = intersect(v0, v1, v2);
        if(result.intersects && result.distance < t_min)
            return true;
    }
    return false;
}

// Does intersection exist in mesh with vertex normals (early exit)
bool Ray3::does_intersect_exist(const std::vector<VertexAndNormal> &vertex_list,
                                const std::vector<uint16_t>        &face_list,
                                float                               t_min) const
{
    uint32_t face_count = static_cast<uint32_t>(face_list.size() / 3);
    for(uint32_t i = 0; i < face_count; ++i)
    {
        const Point3 &v0 = vertex_list[face_list[i * 3    ]].vertex;
        const Point3 &v1 = vertex_list[face_list[i * 3 + 1]].vertex;
        const Point3 &v2 = vertex_list[face_list[i * 3 + 2]].vertex;

        auto result = intersect(v0, v1, v2);
        if(result.intersects && result.distance < t_min)
            return true;
    }
    return false;
}

} // namespace cg
