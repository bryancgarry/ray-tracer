#include "geometry/bounding_sphere.hpp"

#include "geometry/geometry.hpp"

namespace cg
{

BoundingSphere::BoundingSphere() : center{0.0f, 0.0f, 0.0f}, radius(1.0f) {}

BoundingSphere::BoundingSphere(const BoundingSphere &s) : center(s.center), radius(s.radius) {}

BoundingSphere::BoundingSphere(const Point3 &c, float r) : center(c), radius(r) {}

// Ritter's method
BoundingSphere::BoundingSphere(std::vector<Point3> &vertex_list)
{
    // Find the most separated pair of points along each axis,
    // then pick the axis with the largest spread to seed the sphere.
    int min_x = 0, max_x = 0;
    int min_y = 0, max_y = 0;
    int min_z = 0, max_z = 0;
    
    for (int i = 1; i < vertex_list.size(); ++i)
    {
        if(vertex_list[i].x < vertex_list[min_x].x) min_x = i;
        if(vertex_list[i].x > vertex_list[max_x].x) max_x = i;
        if(vertex_list[i].y < vertex_list[min_y].y) min_y = i;
        if(vertex_list[i].y > vertex_list[max_y].y) max_y = i;
        if(vertex_list[i].z < vertex_list[min_z].z) min_z = i;
        if(vertex_list[i].z > vertex_list[max_z].z) max_z = i;
    }
    
    // Compute squared distances between each axis-aligned pair
    Vector3 dx(vertex_list[min_x], vertex_list[max_x]);
    Vector3 dy(vertex_list[min_y], vertex_list[max_y]);
    Vector3 dz(vertex_list[min_z], vertex_list[max_z]);
    
    float spread_x = dx.norm_squared();
    float spread_y = dy.norm_squared();
    float spread_z = dz.norm_squared();
    
    // Seed the sphere with the most separated pair
    int p1, p2;
    if(spread_x >= spread_y && spread_x >= spread_z)
    {
        p1 = min_x;
        p2 = max_x;
    }
    else if (spread_y >= spread_x && spread_y >= spread_z)
    {
        p1 = min_y;
        p2 = max_y;
    }
    else
    {
        p1 = min_z;
        p2 = max_z;
    }
    
    // Initial sphere: center is midpoint, radius is half the distance
    center.x = (vertex_list[p1].x + vertex_list[p2].x) * 0.5f;
    center.y = (vertex_list[p1].y + vertex_list[p2].y) * 0.5f;
    center.z = (vertex_list[p1].z + vertex_list[p2].z) * 0.5f;
    
    Vector3 initial_span(vertex_list[p1], vertex_list[p2]);
    radius = initial_span.norm() * 0.5f;
    
    // Expand the sphere to enclose any vertices that fall outside it
    for (size_t i = 0; i < vertex_list.size(); ++i)
    {
        Vector3 to_pt(center, vertex_list[i]);
        float dist = to_pt.norm();
        
        if(dist > radius)
        {
            // New radius is halfway between the old far side and this point
            float new_radius = (radius + dist) * 0.5f;
            float offset = new_radius - radius;
            
            // Move center toward the outlier by the offset
            center.x += to_pt.x * (offset / dist);
            center.y += to_pt.y * (offset / dist);
            center.z += to_pt.z * (offset / dist);
            
            radius = new_radius;
        }
    }
}

// Merge two bounding spheres into the smallest enclosing sphere
BoundingSphere &BoundingSphere::merge_with(const BoundingSphere &s2)
{
    Vector3 between(center, s2.center);
    float   dist = between.norm();

    // One sphere is already inside the other
    if(dist + s2.radius <= radius)
    {
        return *this;  // s2 is entirely inside this sphere
    }
    if(dist + radius <= s2.radius)
    {
        center = s2.center;  // this sphere is entirely inside s2
        radius = s2.radius;
        return *this;
    }

    // General case where new sphere spans from the far side of each sphere
    float new_radius = (dist + radius + s2.radius) * 0.5f;
    float offset = new_radius - radius;  // How far to move center toward s2

    center.x += between.x * (offset / dist);
    center.y += between.y * (offset / dist);
    center.z += between.z * (offset / dist);

    radius = new_radius;
    return *this;
}

BoundingSphere BoundingSphere::merge(const BoundingSphere &s2) const
{
    BoundingSphere combined(*this);
    combined.merge_with(s2);
    return combined;
}

} // namespace cg
