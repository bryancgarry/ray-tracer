//
//  ray_tracer_test.cpp
//  JHU_605.667
//
//  Created by Bryan Garry.
//

#include "common/logging.hpp"
#include "geometry/geometry.hpp"

#include <algorithm>

namespace cg
{

void ray_tracer_test()
{
    // ============================================================
    // Plane Tests
    // ============================================================
    log_msg("--- Plane Tests ---");

    {
        // Plane at y=3, normal pointing +y. Ray from (0,0,0) in +y direction.
        // Should hit at t=3, point (0,3,0).
        Plane plane(Point3(0.0f, 3.0f, 0.0f), Vector3(0.0f, 1.0f, 0.0f));
        Ray3  ray(Point3(0.0f, 0.0f, 0.0f), Vector3(0.0f, 1.0f, 0.0f));
        auto  result = ray.intersect(plane);
        Point3 hit = ray.intersect(result.distance);
        log_msg("   Plane1 (expect hit at 0,3,0 t=3): hit=%d, point=(%f,%f,%f), t=%f",
                result.intersects, hit.x, hit.y, hit.z, result.distance);
    }

    {
        // Ray parallel to plane — no intersection.
        Plane plane(Point3(0.0f, 3.0f, 0.0f), Vector3(0.0f, 1.0f, 0.0f));
        Ray3  ray(Point3(0.0f, 0.0f, 0.0f), Vector3(1.0f, 0.0f, 0.0f));
        auto  result = ray.intersect(plane);
        log_msg("   Plane2 (expect no hit, parallel): hit=%d", result.intersects);
    }

    {
        // Ray pointing away from plane — no intersection (t would be negative).
        Plane plane(Point3(0.0f, 3.0f, 0.0f), Vector3(0.0f, 1.0f, 0.0f));
        Ray3  ray(Point3(0.0f, 5.0f, 0.0f), Vector3(0.0f, 1.0f, 0.0f));
        auto  result = ray.intersect(plane);
        log_msg("   Plane3 (expect no hit, facing away): hit=%d", result.intersects);
    }

    {
        // Angled ray hitting plane at y=0. Ray from (0,5,0) direction (1,-1,0)
        // normalized. Hits y=0 at (5,0,0), t = 5*sqrt(2).
        Vector3 dir(1.0f, -1.0f, 0.0f);
        dir.normalize();
        Plane  plane(Point3(0.0f, 0.0f, 0.0f), Vector3(0.0f, 1.0f, 0.0f));
        Ray3   ray(Point3(0.0f, 5.0f, 0.0f), dir);
        auto   result = ray.intersect(plane);
        Point3 hit = ray.intersect(result.distance);
        log_msg("   Plane4 (expect hit at 5,0,0 t=~7.07): hit=%d, point=(%f,%f,%f), t=%f",
                result.intersects, hit.x, hit.y, hit.z, result.distance);
    }

    // ============================================================
    // AABB Tests
    // ============================================================
    log_msg("--- AABB Tests ---");

    {
        // Ray hitting front face of a unit box at origin. Ray from (0.5, 0.5, -5)
        // in +z. Should hit at t=5, point (0.5, 0.5, 0).
        AABB AABB(Point3(0.0f, 0.0f, 0.0f), Point3(1.0f, 1.0f, 1.0f));
        Ray3 ray(Point3(0.5f, 0.5f, -5.0f), Vector3(0.0f, 0.0f, 1.0f));
        auto result = ray.intersect(AABB);
        Point3 hit = ray.intersect(result.distance);
        log_msg("   AABB1 (expect hit at 0.5,0.5,0 t=5): hit=%d, point=(%f,%f,%f), t=%f",
                result.intersects, hit.x, hit.y, hit.z, result.distance);
    }

    {
        // Ray that misses the box entirely — passes above it.
        AABB AABB(Point3(0.0f, 0.0f, 0.0f), Point3(1.0f, 1.0f, 1.0f));
        Ray3 ray(Point3(0.5f, 5.0f, -5.0f), Vector3(0.0f, 0.0f, 1.0f));
        auto result = ray.intersect(AABB);
        log_msg("   AABB2 (expect no hit, misses above): hit=%d", result.intersects);
    }

    {
        // Ray originating inside the box. Should return the exit intersection.
        // Box [0,1]^3, ray from center (0.5,0.5,0.5) in +x. Exits at x=1, t=0.5.
        AABB AABB(Point3(0.0f, 0.0f, 0.0f), Point3(1.0f, 1.0f, 1.0f));
        Ray3 ray(Point3(0.5f, 0.5f, 0.5f), Vector3(1.0f, 0.0f, 0.0f));
        auto result = ray.intersect(AABB);
        Point3 hit = ray.intersect(result.distance);
        log_msg("   AABB3 (expect hit at 1,0.5,0.5 t=0.5): hit=%d, point=(%f,%f,%f), t=%f",
                result.intersects, hit.x, hit.y, hit.z, result.distance);
    }

    {
        // Ray pointing away from box — no intersection.
        AABB AABB(Point3(0.0f, 0.0f, 0.0f), Point3(1.0f, 1.0f, 1.0f));
        Ray3 ray(Point3(0.5f, 0.5f, -5.0f), Vector3(0.0f, 0.0f, -1.0f));
        auto result = ray.intersect(AABB);
        log_msg("   AABB4 (expect no hit, facing away): hit=%d", result.intersects);
    }

    {
        // Angled ray hitting corner region of the box.
        Vector3 dir(1.0f, 1.0f, 1.0f);
        dir.normalize();
        AABB   AABB(Point3(2.0f, 2.0f, 2.0f), Point3(4.0f, 4.0f, 4.0f));
        Ray3   ray(Point3(0.0f, 0.0f, 0.0f), dir);
        auto   result = ray.intersect(AABB);
        Point3 hit = ray.intersect(result.distance);
        log_msg("   AABB5 (expect hit at 2,2,2 t=~3.46): hit=%d, point=(%f,%f,%f), t=%f",
                result.intersects, hit.x, hit.y, hit.z, result.distance);
    }

    // ============================================================
    // BoundingSphere Construction Tests (Ritter)
    // ============================================================
    log_msg("--- BoundingSphere Ritter Construction Tests ---");

    {
        // Vertices form a simple axis-aligned spread. Ritter should seed on
        // the x-axis (most separated) then expand to fit y and z outliers.
        std::vector<Point3> verts = {
            Point3(-5.0f, 0.0f, 0.0f),
            Point3( 5.0f, 0.0f, 0.0f),
            Point3( 0.0f, 3.0f, 0.0f),
            Point3( 0.0f,-3.0f, 0.0f),
            Point3( 0.0f, 0.0f, 2.0f),
            Point3( 0.0f, 0.0f,-2.0f)
        };
        BoundingSphere sphere(verts);
        log_msg("   Ritter1 (expect center ~(0,0,0) radius>=5): center=(%f,%f,%f), radius=%f",
                sphere.center.x, sphere.center.y, sphere.center.z, sphere.radius);

        // Verify all vertices are enclosed
        bool all_enclosed = true;
        for(auto &v : verts)
        {
            Vector3 to_v(sphere.center, v);
            if(to_v.norm() > sphere.radius + 0.001f) { all_enclosed = false; break; }
        }
        log_msg("   Ritter1 all vertices enclosed: %s", all_enclosed ? "YES" : "NO");
    }

    {
        // Asymmetric point cloud — the expansion pass should shift center.
        std::vector<Point3> verts = {
            Point3(0.0f, 0.0f, 0.0f),
            Point3(10.0f, 0.0f, 0.0f),
            Point3(5.0f, 8.0f, 0.0f),
            Point3(5.0f, 0.0f, 8.0f)
        };
        BoundingSphere sphere(verts);
        log_msg("   Ritter2: center=(%f,%f,%f), radius=%f",
                sphere.center.x, sphere.center.y, sphere.center.z, sphere.radius);

        bool all_enclosed = true;
        for(auto &v : verts)
        {
            Vector3 to_v(sphere.center, v);
            if(to_v.norm() > sphere.radius + 0.001f) { all_enclosed = false; break; }
        }
        log_msg("   Ritter2 all vertices enclosed: %s", all_enclosed ? "YES" : "NO");
    }

    // ============================================================
    // BoundingSphere Merge Tests
    // ============================================================
    log_msg("--- BoundingSphere Merge Tests ---");

    {
        // Two non-overlapping spheres. Merged sphere should enclose both.
        BoundingSphere s1(Point3(0.0f, 0.0f, 0.0f), 1.0f);
        BoundingSphere s2(Point3(5.0f, 0.0f, 0.0f), 1.0f);
        BoundingSphere merged = s1.merge(s2);
        log_msg("   Merge1 (expect center ~(2.5,0,0) radius=3.5): center=(%f,%f,%f), radius=%f",
                merged.center.x, merged.center.y, merged.center.z, merged.radius);
    }

    {
        // s2 entirely inside s1 — merged should equal s1.
        BoundingSphere s1(Point3(0.0f, 0.0f, 0.0f), 10.0f);
        BoundingSphere s2(Point3(1.0f, 0.0f, 0.0f), 2.0f);
        BoundingSphere merged = s1.merge(s2);
        log_msg("   Merge2 (expect center=(0,0,0) radius=10): center=(%f,%f,%f), radius=%f",
                merged.center.x, merged.center.y, merged.center.z, merged.radius);
    }

    // ============================================================
    // Ray-Triangle Intersection Tests
    // ============================================================
    log_msg("--- Ray-Triangle Tests ---");

    {
        // Triangle in the xz-plane at y=0. Ray from (0.25, 5, 0.25) straight
        // down in -y. Should hit at t=5, point (0.25, 0, 0.25).
        Point3 v0(0.0f, 0.0f, 0.0f);
        Point3 v1(1.0f, 0.0f, 0.0f);
        Point3 v2(0.0f, 0.0f, 1.0f);
        Ray3   ray(Point3(0.25f, 5.0f, 0.25f), Vector3(0.0f, -1.0f, 0.0f));
        auto   result = ray.intersect(v0, v1, v2);
        Point3 hit = ray.intersect(result.distance);
        log_msg("   Tri1 (expect hit at 0.25,0,0.25 t=5): hit=%d, point=(%f,%f,%f), t=%f, u=%f, v=%f",
                result.intersects, hit.x, hit.y, hit.z,
                result.distance, result.barycentric_u, result.barycentric_v);
    }

    {
        // Same triangle, ray misses to the side.
        Point3 v0(0.0f, 0.0f, 0.0f);
        Point3 v1(1.0f, 0.0f, 0.0f);
        Point3 v2(0.0f, 0.0f, 1.0f);
        Ray3   ray(Point3(5.0f, 5.0f, 5.0f), Vector3(0.0f, -1.0f, 0.0f));
        auto   result = ray.intersect(v0, v1, v2);
        log_msg("   Tri2 (expect no hit, misses): hit=%d", result.intersects);
    }

    {
        // Ray pointing away from triangle — no hit.
        Point3 v0(0.0f, 0.0f, 0.0f);
        Point3 v1(1.0f, 0.0f, 0.0f);
        Point3 v2(0.0f, 0.0f, 1.0f);
        Ray3   ray(Point3(0.25f, 5.0f, 0.25f), Vector3(0.0f, 1.0f, 0.0f));
        auto   result = ray.intersect(v0, v1, v2);
        log_msg("   Tri3 (expect no hit, facing away): hit=%d", result.intersects);
    }

    {
        // Ray hitting exactly on v0 — edge case, should still register as hit.
        Point3 v0(0.0f, 0.0f, 0.0f);
        Point3 v1(1.0f, 0.0f, 0.0f);
        Point3 v2(0.0f, 0.0f, 1.0f);
        Ray3   ray(Point3(0.0f, 5.0f, 0.0f), Vector3(0.0f, -1.0f, 0.0f));
        auto   result = ray.intersect(v0, v1, v2);
        Point3 hit = ray.intersect(result.distance);
        log_msg("   Tri4 (expect hit at 0,0,0 t=5): hit=%d, point=(%f,%f,%f), t=%f, u=%f, v=%f",
                result.intersects, hit.x, hit.y, hit.z,
                result.distance, result.barycentric_u, result.barycentric_v);
    }

    {
        // does_intersect_exist should match the intersect result
        Point3 v0(0.0f, 0.0f, 0.0f);
        Point3 v1(1.0f, 0.0f, 0.0f);
        Point3 v2(0.0f, 0.0f, 1.0f);
        Ray3   ray_hit(Point3(0.25f, 5.0f, 0.25f), Vector3(0.0f, -1.0f, 0.0f));
        Ray3   ray_miss(Point3(5.0f, 5.0f, 5.0f), Vector3(0.0f, -1.0f, 0.0f));
        log_msg("   Tri5 (expect exists=1, not_exists=0): hit=%d, miss=%d",
                ray_hit.does_intersect_exist(v0, v1, v2),
                ray_miss.does_intersect_exist(v0, v1, v2));
    }

    // ============================================================
    // Ray-Mesh Intersection Tests
    // ============================================================
    log_msg("--- Ray-Mesh Tests ---");

    {
        // Two triangles forming a quad in the xz-plane at y=0.
        //   v2---v3
        //   |  / |
        //   v0---v1
        std::vector<Point3>   verts = {
            Point3(0.0f, 0.0f, 0.0f),   // v0
            Point3(1.0f, 0.0f, 0.0f),   // v1
            Point3(0.0f, 0.0f, 1.0f),   // v2
            Point3(1.0f, 0.0f, 1.0f)    // v3
        };
        std::vector<uint16_t> faces = {
            0, 1, 2,   // triangle 0
            1, 3, 2    // triangle 1
        };

        {
            // Ray hits triangle 0
            Ray3 ray(Point3(0.25f, 5.0f, 0.25f), Vector3(0.0f, -1.0f, 0.0f));
            auto result = ray.intersect(verts, faces, 100.0f);
            Point3 hit = ray.intersect(result.distance);
            log_msg("   Mesh1 (expect hit tri 0 at 0.25,0,0.25): hit=%d, point=(%f,%f,%f), face=%u",
                    result.intersects, hit.x, hit.y, hit.z, result.face_index);
        }

        {
            // Ray hits triangle 1
            Ray3 ray(Point3(0.75f, 5.0f, 0.75f), Vector3(0.0f, -1.0f, 0.0f));
            auto result = ray.intersect(verts, faces, 100.0f);
            Point3 hit = ray.intersect(result.distance);
            log_msg("   Mesh2 (expect hit tri 1 at 0.75,0,0.75): hit=%d, point=(%f,%f,%f), face=%u",
                    result.intersects, hit.x, hit.y, hit.z, result.face_index);
        }

        {
            // Ray misses the quad entirely
            Ray3 ray(Point3(5.0f, 5.0f, 5.0f), Vector3(0.0f, -1.0f, 0.0f));
            auto result = ray.intersect(verts, faces, 100.0f);
            log_msg("   Mesh3 (expect no hit): hit=%d", result.intersects);
        }

        {
            // t_min culling: hit exists at t=5, but t_min=2 so it should be culled.
            Ray3 ray(Point3(0.25f, 5.0f, 0.25f), Vector3(0.0f, -1.0f, 0.0f));
            auto result = ray.intersect(verts, faces, 2.0f);
            log_msg("   Mesh4 (expect no hit, t_min culled): hit=%d", result.intersects);
        }

        {
            // does_intersect_exist matches
            Ray3 ray_hit(Point3(0.25f, 5.0f, 0.25f), Vector3(0.0f, -1.0f, 0.0f));
            Ray3 ray_miss(Point3(5.0f, 5.0f, 5.0f), Vector3(0.0f, -1.0f, 0.0f));
            log_msg("   Mesh5 (expect exists=1, not_exists=0): hit=%d, miss=%d",
                    ray_hit.does_intersect_exist(verts, faces, 100.0f),
                    ray_miss.does_intersect_exist(verts, faces, 100.0f));
        }
    }
}

} // namespace cg
