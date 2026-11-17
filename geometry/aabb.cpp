#include "geometry/aabb.hpp"

#include "geometry/geometry.hpp"

namespace cg
{

// NOTE - this is not required until 605.767!

AABB::AABB() : min(0.0f, 0.0f, 0.0f), max(0.0f, 0.0f, 0.0f) {}

AABB::AABB(const Point3 &min, const Point3 &max)
{
    update(min, max);
}

AABB::AABB(const std::vector<Point3> &vertex_list)
{
    create(vertex_list);
}

void AABB::create(const std::vector<Point3> &vertex_list)
{
    if (vertex_list.empty())
    {
        min = Point3(0.0f, 0.0f, 0.0f);
        max = Point3(0.0f, 0.0f, 0.0f);
        return;
    }
    
    // Initialize with first point
    min = vertex_list[0];
    max = vertex_list[0];
    
    // Expand to encompass all points
    for (const auto &v : vertex_list)
    {
        if (v.x < min.x) min.x = v.x;
        if (v.y < min.y) min.y = v.y;
        if (v.z < min.z) min.z = v.z;
        
        if (v.x > max.x) max.x = v.x;
        if (v.y > max.y) max.y = v.y;
        if (v.z > max.z) max.z = v.z;
    }
}

void AABB::update(const Point3 &min, const Point3 &max)
{
    this->min = min;
    this->max = max;
}

void AABB::merge(const AABB &box)
{
    min.x = std::min(min.x, box.min.x);
    min.y = std::min(min.y, box.min.y);
    min.z = std::min(min.z, box.min.z);
    
    max.x = std::max(max.x, box.max.x);
    max.y = std::max(max.y, box.max.y);
    max.z = std::max(max.z, box.max.z);
}

Point3 AABB::min_pt() const
{
    return min;
}

Point3 AABB::max_pt() const
{
    return max;
}

void AABB::compute_center()
{
    center.x = (min.x + max.x) * 0.5f;
    center.y = (min.y + max.y) * 0.5f;
    center.z = (min.z + max.z) * 0.5f;

    half_diagonal.x = (max.x - min.x) * 0.5f;
    half_diagonal.y = (max.y - min.y) * 0.5f;
    half_diagonal.z = (max.z - min.z) * 0.5f;
}

} // namespace cg
