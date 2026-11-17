//============================================================================
//    Johns Hopkins University Engineering Programs for Professionals
//    605.667 Computer Graphics and 605.767 Applied Computer Graphics
//    Instructor:    Brian Russin
//
//    Author:      David W. Nesbitt
//    File:     tri_surface.hpp
//    Purpose:  Scene graph geometry node for a triangle mesh surface.
//            Uses indexed vertex arrays with per-vertex normals.
//============================================================================

#ifndef __SCENE_TRI_SURFACE_HPP__
#define __SCENE_TRI_SURFACE_HPP__

#include "scene/geometry_node.hpp"
#include "geometry/aabb.hpp"
#include "geometry/ray3.hpp"

#include <limits>

namespace cg
{

/**
 * Triangle mesh surface. Uses indexed vertex arrays with per-vertex normals
 * stored as VertexAndNormal pairs.
 */
class TriSurface : public GeometryNode
{
  public:
    TriSurface();
    ~TriSurface();

    /**
     * Draw this geometry node.
     */
    void draw(SceneState &scene_state) override;

    /**
     * Construct the surface directly from a vertex list and face index list.
     * @param  v  Vertices (position + normal).
     * @param  f  Triangle index list (3 indices per face).
     */
    void construct(const std::vector<VertexAndNormal> &v, const std::vector<uint16_t> &f);

    /**
     * Incrementally add a triangle to the mesh. Shared vertices are
     * deduplicated automatically. Call end() when finished to compute
     * vertex normals. Vertices must be in counter-clockwise order.
     * @param  v0  First vertex.
     * @param  v1  Second vertex.
     * @param  v2  Third vertex.
     */
    void add(const Point3 &v0, const Point3 &v1, const Point3 &v2);

    /**
     * Add a convex polygon as a triangle fan.
     * @param  vertex_list  Polygon vertices in counter-clockwise order.
     */
    void add_polygon(const std::vector<Point3> &vertex_list);

    /**
     * Finalize the mesh: compute per-vertex normals and upload to GPU.
     * Must be called after all add() / add_polygon() calls.
     */
    void end(int32_t position_loc, int32_t normal_loc);

    /**
     * Upload vertex data to GPU buffers.
     */
    void create_vertex_buffers(int32_t position_loc, int32_t normal_loc);

    /**
     * Compute the axis-aligned bounding box for this mesh.
     * Call after mesh construction is complete.
     */
    void compute_bounding_box();

    // ---- Ray tracing overrides ----

    /**
     * Find the closest ray-mesh intersection. Updates closest if a nearer
     * hit is found. Must be called before get_normal() or get_texture_coord().
     */
    void find_closest_intersect(Ray3 ray, SceneState &current_state,
                                SceneState &closest) override;

    /**
     * Fast intersection test for shadow rays.
     * @param  d  Maximum distance to consider (light distance).
     */
    bool does_intersect_exist(Ray3 ray, float d,
                              SceneState &current_state) override;

    /**
     * Returns the interpolated surface normal at the last intersection point.
     * NOTE: result is only valid after a successful find_closest_intersect call.
     * The point parameter is unused; barycentric coordinates from the last
     * intersection are used instead.
     */
    Vector3 get_normal(const Point3 &point) override;

    /**
     * Returns the texture coordinates at the last intersection point.
     * NOTE: result is only valid after a successful find_closest_intersect call.
     * The point parameter is unused; barycentric coordinates from the last
     * intersection are used instead.
     */
    Point2 get_texture_coord(const Point3 &point) override;

  protected:
    // GPU resources
    GLsizei face_count_;
    GLuint  vao_;
    GLuint  vbo_;
    GLuint  facebuffer_;

    // Mesh data
    std::vector<VertexAndNormal> vertices_;
    std::vector<uint16_t>        faces_;

    // Ray tracing support
    AABB    local_bounds_;
    mutable Vector3 last_hit_normal_;
    mutable Point2  last_hit_texcoord_;

    /**
     * Form triangle face indices for a grid surface built in row-major order.
     * @param  num_rows  Number of rows in the grid.
     * @param  num_cols  Number of columns in the grid.
     */
    void construct_row_col_face_list(uint32_t num_rows, uint32_t num_cols);

    /**
     * Returns the flat vertex index for a grid position.
     */
    uint16_t get_index(uint32_t row, uint32_t col, uint32_t num_cols) const;

    /**
     * Add a vertex to the mesh, deduplicating if already present.
     * @return Index of the vertex in the vertex list.
     */
    uint16_t add_vertex(const Point3 &vtx);
};

} // namespace cg

#endif
