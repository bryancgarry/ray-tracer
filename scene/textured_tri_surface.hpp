//============================================================================
//    Johns Hopkins University Engineering Programs for Professionals
//    605.667 Computer Graphics and 605.767 Applied Computer Graphics
//    Instructor:    Brian Russin
//
//    Author:      David W. Nesbitt
//    File:     textured_tri_surface.hpp
//    Purpose:  Scene graph geometry node for a texture-mapped triangle mesh.
//            Uses indexed vertex arrays with per-vertex normals and UVs.
//============================================================================

#ifndef __SCENE_TEXTURED_TRISURFACE_HPP__
#define __SCENE_TEXTURED_TRISURFACE_HPP__

#include "scene/geometry_node.hpp"
#include "geometry/aabb.hpp"

namespace cg
{

/**
 * Texture-mapped triangle mesh surface. Stores vertices as PNTVertex
 * (position, normal, texture coordinate).
 */
class TexturedTriSurface : public GeometryNode
{
  public:
    TexturedTriSurface();
    ~TexturedTriSurface();

    /**
     * Draw this geometry node.
     */
    void draw(SceneState &scene_state) override;

    /**
     * Construct the surface directly from a vertex list and face index list,
     * then upload to GPU.
     * @param  vertex_list   Vertices (position, normal, texture coord).
     * @param  face_list     Triangle index list (3 indices per face).
     * @param  position_loc  Shader attribute location for position.
     * @param  normal_loc    Shader attribute location for normal.
     * @param  texture_loc   Shader attribute location for texture coord.
     */
    void construct(const std::vector<PNTVertex>  &vertex_list,
                   const std::vector<uint16_t>   &face_list,
                   int32_t                        position_loc,
                   int32_t                        normal_loc,
                   int32_t                        texture_loc);

    /**
     * Finalize the mesh: compute per-vertex normals and upload to GPU.
     * Must be called after the vertex and face lists have been populated.
     */
    void end(int32_t position_loc, int32_t normal_loc, int32_t texture_loc);

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
     * Upload vertex and face data to GPU buffers.
     */
    void create_vertex_buffers(int32_t position_loc, int32_t normal_loc, int32_t texture_loc);

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
     */
    Vector3 get_normal(const Point3 &point) override;

    /**
     * Returns the interpolated texture coordinates at the last intersection point.
     * NOTE: result is only valid after a successful find_closest_intersect call.
     */
    Point2 get_texture_coord(const Point3 &point) override;

  protected:
    // GPU resources
    GLsizei face_count_;
    GLuint  vao_;
    GLuint  vbo_;
    GLuint  facebuffer_;

    // Mesh data
    std::vector<PNTVertex>  vertices_;
    std::vector<uint16_t>   faces_;

    // Ray tracing support
    AABB    local_bounds_;
    mutable Vector3 last_hit_normal_;
    mutable Point2  last_hit_texcoord_;
};

} // namespace cg

#endif
