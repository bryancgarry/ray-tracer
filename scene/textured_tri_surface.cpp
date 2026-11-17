#include "scene/textured_tri_surface.hpp"

namespace cg
{

TexturedTriSurface::TexturedTriSurface()
    : GeometryNode(), vao_{0}, vbo_{0}, facebuffer_{0}
{
}

TexturedTriSurface::~TexturedTriSurface()
{
    glDeleteBuffers(1, &vbo_);
    glDeleteBuffers(1, &facebuffer_);
    glDeleteVertexArrays(1, &vao_);
}

void TexturedTriSurface::draw(SceneState &scene_state)
{
    glBindVertexArray(vao_);
    glDrawElements(GL_TRIANGLES, face_count_, GL_UNSIGNED_SHORT, (void *)0);
    glBindVertexArray(0);
    
    // Disable texture vertex attribute
    glDisableVertexAttribArray(scene_state.texture_loc);
}

void TexturedTriSurface::construct(const std::vector<PNTVertex>  &vertex_list,
                                   const std::vector<uint16_t>   &face_list,
                                   int32_t                        position_loc,
                                   int32_t                        normal_loc,
                                   int32_t                        texture_loc)
{
    vertices_ = vertex_list;
    faces_    = face_list;
    create_vertex_buffers(position_loc, normal_loc, texture_loc);
}

void TexturedTriSurface::end(int32_t position_loc, int32_t normal_loc, int32_t texture_loc)
{
    // Accumulate face normals into each vertex, then normalize to average them.
    // Vertex normals are initialized to (0,0,0) by PNTVertex's constructor.
    uint32_t v0, v1, v2;
    Vector3  e1, e2, face_normal;
    auto     face_vertex = faces_.begin();
    while(face_vertex != faces_.end())
    {
        v0 = *face_vertex++;
        v1 = *face_vertex++;
        v2 = *face_vertex++;

        e1.set(vertices_[v0].vertex, vertices_[v1].vertex);
        e2.set(vertices_[v0].vertex, vertices_[v2].vertex);
        face_normal = (e1.cross(e2)).normalize();

        vertices_[v0].normal += face_normal;
        vertices_[v1].normal += face_normal;
        vertices_[v2].normal += face_normal;
    }

    for(auto &v : vertices_) { v.normal.normalize(); }

    create_vertex_buffers(position_loc, normal_loc, texture_loc);
}

void TexturedTriSurface::construct_row_col_face_list(uint32_t num_rows, uint32_t num_cols)
{
    for(uint32_t row = 0; row < num_rows - 1; row++)
    {
        for(uint32_t col = 0; col < num_cols - 1; col++)
        {
            faces_.push_back(get_index(row + 1, col,     num_cols));
            faces_.push_back(get_index(row,     col,     num_cols));
            faces_.push_back(get_index(row,     col + 1, num_cols));

            faces_.push_back(get_index(row + 1, col,     num_cols));
            faces_.push_back(get_index(row,     col + 1, num_cols));
            faces_.push_back(get_index(row + 1, col + 1, num_cols));
        }
    }
}

uint16_t TexturedTriSurface::get_index(uint32_t row, uint32_t col, uint32_t num_cols) const
{
    return static_cast<uint16_t>(row * num_cols + col);
}

void TexturedTriSurface::create_vertex_buffers(int32_t position_loc,
                                               int32_t normal_loc,
                                               int32_t texture_loc)
{
    glGenBuffers(1, &vbo_);
    glGenBuffers(1, &facebuffer_);

    glBindBuffer(GL_ARRAY_BUFFER, vbo_);
    glBufferData(GL_ARRAY_BUFFER,
                 vertices_.size() * sizeof(PNTVertex),
                 (void *)&vertices_[0],
                 GL_STATIC_DRAW);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, facebuffer_);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER,
                 faces_.size() * sizeof(uint16_t),
                 (void *)&faces_[0],
                 GL_STATIC_DRAW);

    face_count_ = static_cast<GLsizei>(faces_.size());

    glGenVertexArrays(1, &vao_);
    glBindVertexArray(vao_);

    glBindBuffer(GL_ARRAY_BUFFER, vbo_);
    glVertexAttribPointer(position_loc, 3, GL_FLOAT, GL_FALSE,
                          sizeof(PNTVertex), (void *)0);
    glVertexAttribPointer(normal_loc, 3, GL_FLOAT, GL_FALSE,
                          sizeof(PNTVertex), (void *)(sizeof(Point3)));
    glVertexAttribPointer(texture_loc, 2, GL_FLOAT, GL_FALSE,
                          sizeof(PNTVertex), (void *)(sizeof(Point3) + sizeof(Vector3)));
    glEnableVertexAttribArray(position_loc);
    glEnableVertexAttribArray(normal_loc);
    glEnableVertexAttribArray(texture_loc);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, facebuffer_);
    glBindVertexArray(0);
}

// --- Ray tracing ---
// The ray is transformed into local object space using the inverse model matrix.
// The direction's post-transform length (local_ray_length) encodes the
// world-to-local scale factor:
//
// world_t = local_t / local_ray_length
// local_t = world_t * local_ray_length
void TexturedTriSurface::compute_bounding_box()
{
    if(vertices_.empty()) return;

    Point3 mn = vertices_[0].vertex;
    Point3 mx = vertices_[0].vertex;
    for(const auto &v : vertices_)
    {
        const Point3 &p = v.vertex;
        if(p.x < mn.x) mn.x = p.x;  if(p.x > mx.x) mx.x = p.x;
        if(p.y < mn.y) mn.y = p.y;  if(p.y > mx.y) mx.y = p.y;
        if(p.z < mn.z) mn.z = p.z;  if(p.z > mx.z) mx.z = p.z;
    }
    local_bounds_.update(mn, mx);
}

void TexturedTriSurface::find_closest_intersect(Ray3        ray,
                                                SceneState &current_state,
                                                SceneState &closest)
{
    // Transform ray origin (point, w=1) and direction (vector, w=0) to local space
    HPoint3 local_o_h  = current_state.inverse_matrix * ray.o;
    Point3  local_o(local_o_h.x / local_o_h.w,
                    local_o_h.y / local_o_h.w,
                    local_o_h.z / local_o_h.w);

    Vector3 local_dir = current_state.inverse_matrix * ray.d;
    float   local_ray_length = local_dir.norm();
    if(local_ray_length < 0.000001f) return;

    Ray3 local_ray(local_o, local_dir * (1.0f / local_ray_length));

    // AABB early rejection
    if(!local_ray.intersect(local_bounds_).intersects) return;

    // Convert world space closest distance to local space, then test all triangles
    float local_t_min = closest.t_min * local_ray_length;

    std::vector<Point3> positions;
    positions.reserve(vertices_.size());
    for(const auto &v : vertices_) positions.push_back(v.vertex);

    auto result = local_ray.intersect(positions, faces_, local_t_min);
    if(!result.intersects) return;

    // Convert local distance back to world space
    float world_t = result.distance / local_ray_length;
    if(world_t >= closest.t_min) return;

    // Record the hit
    closest.t_min = world_t;
    closest.geometry_node = this;
    closest.material_node = current_state.material_node;
    closest.texture_node = current_state.texture_node;

    uint32_t f_idx = result.face_index * 3;
    uint16_t i0 = faces_[f_idx];
    uint16_t i1 = faces_[f_idx + 1];
    uint16_t i2 = faces_[f_idx + 2];

    float u = result.barycentric_u;
    float v = result.barycentric_v;
    float w = 1.0f - u - v;

    // Interpolate normal in local space, transform to world space via normal matrix.
    Vector3 local_normal = vertices_[i0].normal * w
                         + vertices_[i1].normal * u
                         + vertices_[i2].normal * v;
    local_normal.normalize();
    last_hit_normal_ = current_state.normal_matrix * local_normal;
    last_hit_normal_.normalize();

    // Interpolate stored vertex texture coordinates
    last_hit_texcoord_ = Point2(vertices_[i0].texture.x * w + vertices_[i1].texture.x * u + vertices_[i2].texture.x * v,
                                vertices_[i0].texture.y * w + vertices_[i1].texture.y * u + vertices_[i2].texture.y * v);
}

bool TexturedTriSurface::does_intersect_exist(Ray3 ray, float d, SceneState &current_state)
{
    HPoint3 local_o_h  = current_state.inverse_matrix * ray.o;
    Point3  local_o(local_o_h.x / local_o_h.w,
                    local_o_h.y / local_o_h.w,
                    local_o_h.z / local_o_h.w);

    Vector3 local_dir = current_state.inverse_matrix * ray.d;
    float local_ray_length = local_dir.norm();
    if(local_ray_length < 0.000001f) return false;

    Ray3 local_ray(local_o, local_dir * (1.0f / local_ray_length));

    if(!local_ray.intersect(local_bounds_).intersects) return false;

    std::vector<Point3> positions;
    positions.reserve(vertices_.size());
    for(const auto &v : vertices_) positions.push_back(v.vertex);

    return local_ray.does_intersect_exist(positions, faces_, d * local_ray_length);
}

Vector3 TexturedTriSurface::get_normal(const Point3 &point) { return last_hit_normal_; }
Point2 TexturedTriSurface::get_texture_coord(const Point3 &point) { return last_hit_texcoord_; }

} // namespace cg
