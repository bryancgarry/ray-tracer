#include "scene/tri_surface.hpp"

#include <limits>

namespace cg
{

TriSurface::TriSurface() : GeometryNode(), vao_{0}, vbo_{0}, facebuffer_{0} {}

TriSurface::~TriSurface()
{
    glDeleteBuffers(1, &vbo_);
    glDeleteBuffers(1, &facebuffer_);
    glDeleteVertexArrays(1, &vao_);
}

void TriSurface::draw(SceneState &scene_state)
{
    glBindVertexArray(vao_);
    glDrawElements(GL_TRIANGLES, face_count_, GL_UNSIGNED_SHORT, (void *)0);
    glBindVertexArray(0);
}

void TriSurface::construct(const std::vector<VertexAndNormal> &v, const std::vector<uint16_t> &f)
{
    vertices_ = v;
    faces_    = f;
}

void TriSurface::add_polygon(const std::vector<Point3> &vertex_list)
{
    // Compute the polygon face normal from the first two edges
    VertexAndNormal vertex;
    Vector3         e1(vertex_list[0], vertex_list[1]);
    Vector3         e2(vertex_list[0], vertex_list[2]);
    vertex.normal = (e1.cross(e2)).normalize();

    // Append all polygon vertices, then form a triangle fan anchored at index 0
    size_t base = vertices_.size();
    for(const auto &v : vertex_list)
    {
        vertex.vertex = v;
        vertices_.push_back(vertex);
    }

    for(size_t i = 2, n = vertex_list.size(); i < n; i++)
    {
        faces_.push_back(static_cast<uint16_t>(base));
        faces_.push_back(static_cast<uint16_t>(base + i - 1));
        faces_.push_back(static_cast<uint16_t>(base + i));
    }
}

void TriSurface::add(const Point3 &v0, const Point3 &v1, const Point3 &v2)
{
    faces_.push_back(add_vertex(v0));
    faces_.push_back(add_vertex(v1));
    faces_.push_back(add_vertex(v2));
}

void TriSurface::end(int32_t position_loc, int32_t normal_loc)
{
    // Accumulate face normals into each vertex, then normalize to average them.
    // Vertex normals are initialized to (0,0,0) by VertexAndNormal's constructor.
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

    create_vertex_buffers(position_loc, normal_loc);
}

void TriSurface::create_vertex_buffers(int32_t position_loc, int32_t normal_loc)
{
    glGenBuffers(1, &vbo_);
    glGenBuffers(1, &facebuffer_);

    glBindBuffer(GL_ARRAY_BUFFER, vbo_);
    glBufferData(GL_ARRAY_BUFFER,
                 vertices_.size() * sizeof(VertexAndNormal),
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
                          sizeof(VertexAndNormal), (void *)0);
    glVertexAttribPointer(normal_loc, 3, GL_FLOAT, GL_FALSE,
                          sizeof(VertexAndNormal), (void *)(sizeof(Point3)));
    glEnableVertexAttribArray(position_loc);
    glEnableVertexAttribArray(normal_loc);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, facebuffer_);
    glBindVertexArray(0);
}

void TriSurface::construct_row_col_face_list(uint32_t num_rows, uint32_t num_cols)
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

uint16_t TriSurface::get_index(uint32_t row, uint32_t col, uint32_t num_cols) const
{
    return static_cast<uint16_t>(row * num_cols + col);
}

uint16_t TriSurface::add_vertex(const Point3 &vtx)
{
    uint16_t index = 0;
    for(const auto &v : vertices_)
    {
        if(vtx == v.vertex) return index;
        index++;
    }

    vertices_.push_back(VertexAndNormal(vtx));
    return static_cast<uint16_t>(vertices_.size() - 1);
}

// --- Ray tracing ---
// The ray is transformed into local object space using the inverse model
// matrix. The direction's length after transformation encodes the
// world-to-local scale factor (local_ray_length). All intersection distances
// are in local space and must be converted back:
//
// world_t = local_t / local_ray_length
// local_t = world_t * local_ray_length
void TriSurface::compute_bounding_box()
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

void TriSurface::find_closest_intersect(Ray3 ray, SceneState &current_state,
                                        SceneState &closest)
{
    // Transform ray origin (point, w=1) and direction (vector, w=0) to local space
    HPoint3 local_o_h  = current_state.inverse_matrix * ray.o;
    Point3  local_o(local_o_h.x / local_o_h.w,
                    local_o_h.y / local_o_h.w,
                    local_o_h.z / local_o_h.w);

    Vector3 local_dir = current_state.inverse_matrix * ray.d;
    float local_ray_length = local_dir.norm();
    if(local_ray_length < 0.000001f) return;

    Ray3 local_ray(local_o, local_dir * (1.0f / local_ray_length));

    // AABB early rejection
    if(!local_ray.intersect(local_bounds_).intersects) return;

    // Convert the current world space closest distance to local space
    float local_t_min = closest.t_min * local_ray_length;

    // Iterate triangles directly — no temporary vertex array copy.
    // Previously a std::vector<Point3> was built per ray, causing heap
    // allocation on every intersection test and incorrect t ordering.
    float best_t = local_t_min;
    uint32_t best_face = 0;
    float best_u = 0.0f, best_v = 0.0f;
    bool hit = false;

    const uint32_t num_faces = static_cast<uint32_t>(faces_.size()) / 3;
    for(uint32_t fi = 0; fi < num_faces; ++fi)
    {
        const Point3 &v0 = vertices_[faces_[fi * 3    ]].vertex;
        const Point3 &v1 = vertices_[faces_[fi * 3 + 1]].vertex;
        const Point3 &v2 = vertices_[faces_[fi * 3 + 2]].vertex;

        Vector3 e1(v0, v1);
        Vector3 e2(v0, v2);
        Vector3 h = local_ray.d.cross(e2);
        float   a = e1.dot(h);

        if(std::abs(a) < 0.001f) continue;

        float   inv_a = 1.0f / a;
        Vector3 s(v0, local_ray.o);
        float   u = inv_a * s.dot(h);
        if(u < 0.0f || u > 1.0f) continue;

        Vector3 q = s.cross(e1);
        float   v = inv_a * local_ray.d.dot(q);
        if(v < 0.0f || (u + v) > 1.0f) continue;

        float t = inv_a * e2.dot(q);
        if(t < 0.001f || t >= best_t) continue;

        best_t    = t;
        best_face = fi;
        best_u    = u;
        best_v    = v;
        hit       = true;
    }

    if(!hit) return;

    // Convert local distance back to world space
    float world_t = best_t / local_ray_length;
    if(world_t >= closest.t_min) return;

    // Record the hit
    closest.t_min         = world_t;
    closest.geometry_node = this;
    closest.material_node = current_state.material_node;
    closest.texture_node  = current_state.texture_node;

    // Interpolate vertex normals using barycentric coordinates
    uint32_t f_idx = best_face * 3;
    float u = best_u;
    float v = best_v;
    float w = 1.0f - u - v;

    Vector3 local_normal = vertices_[faces_[f_idx    ]].normal * w
                         + vertices_[faces_[f_idx + 1]].normal * u
                         + vertices_[faces_[f_idx + 2]].normal * v;
    local_normal.normalize();

    // Transform to world space and store directly in closest — thread-safe
    closest.hit_normal = current_state.normal_matrix * local_normal;
    closest.hit_normal.normalize();

    closest.hit_texcoord = Point2(u, v);
}

bool TriSurface::does_intersect_exist(Ray3 ray, float d, SceneState &current_state)
{
    HPoint3 local_o_h  = current_state.inverse_matrix * ray.o;
    Point3  local_o(local_o_h.x / local_o_h.w,
                    local_o_h.y / local_o_h.w,
                    local_o_h.z / local_o_h.w);

    Vector3 local_dir        = current_state.inverse_matrix * ray.d;
    float   local_ray_length = local_dir.norm();
    if(local_ray_length < 0.000001f) return false;

    Ray3 local_ray(local_o, local_dir * (1.0f / local_ray_length));

    if(!local_ray.intersect(local_bounds_).intersects) return false;

    std::vector<Point3> positions;
    positions.reserve(vertices_.size());
    for(const auto &v : vertices_) positions.push_back(v.vertex);

    return local_ray.does_intersect_exist(positions, faces_, d * local_ray_length);
}

Vector3 TriSurface::get_normal(const Point3 &point) { return last_hit_normal_; }

Point2 TriSurface::get_texture_coord(const Point3 &point) { return last_hit_texcoord_; }

} // namespace cg

//#include "scene/tri_surface.hpp"
//
//#include <limits>
//
//namespace cg
//{
//
//TriSurface::TriSurface() : GeometryNode(), vao_{0}, vbo_{0}, facebuffer_{0} {}
//
//TriSurface::~TriSurface()
//{
//    glDeleteBuffers(1, &vbo_);
//    glDeleteBuffers(1, &facebuffer_);
//    glDeleteVertexArrays(1, &vao_);
//}
//
//void TriSurface::draw(SceneState &scene_state)
//{
//    glBindVertexArray(vao_);
//    glDrawElements(GL_TRIANGLES, face_count_, GL_UNSIGNED_SHORT, (void *)0);
//    glBindVertexArray(0);
//}
//
//void TriSurface::construct(const std::vector<VertexAndNormal> &v, const std::vector<uint16_t> &f)
//{
//    vertices_ = v;
//    faces_    = f;
//}
//
//void TriSurface::add_polygon(const std::vector<Point3> &vertex_list)
//{
//    // Compute the polygon face normal from the first two edges
//    VertexAndNormal vertex;
//    Vector3         e1(vertex_list[0], vertex_list[1]);
//    Vector3         e2(vertex_list[0], vertex_list[2]);
//    vertex.normal = (e1.cross(e2)).normalize();
//
//    // Append all polygon vertices, then form a triangle fan anchored at index 0
//    size_t base = vertices_.size();
//    for(const auto &v : vertex_list)
//    {
//        vertex.vertex = v;
//        vertices_.push_back(vertex);
//    }
//
//    for(size_t i = 2, n = vertex_list.size(); i < n; i++)
//    {
//        faces_.push_back(static_cast<uint16_t>(base));
//        faces_.push_back(static_cast<uint16_t>(base + i - 1));
//        faces_.push_back(static_cast<uint16_t>(base + i));
//    }
//}
//
//void TriSurface::add(const Point3 &v0, const Point3 &v1, const Point3 &v2)
//{
//    faces_.push_back(add_vertex(v0));
//    faces_.push_back(add_vertex(v1));
//    faces_.push_back(add_vertex(v2));
//}
//
//void TriSurface::end(int32_t position_loc, int32_t normal_loc)
//{
//    // Accumulate face normals into each vertex, then normalize to average them.
//    // Vertex normals are initialized to (0,0,0) by VertexAndNormal's constructor.
//    uint32_t v0, v1, v2;
//    Vector3  e1, e2, face_normal;
//    auto     face_vertex = faces_.begin();
//    while(face_vertex != faces_.end())
//    {
//        v0 = *face_vertex++;
//        v1 = *face_vertex++;
//        v2 = *face_vertex++;
//
//        e1.set(vertices_[v0].vertex, vertices_[v1].vertex);
//        e2.set(vertices_[v0].vertex, vertices_[v2].vertex);
//        face_normal = (e1.cross(e2)).normalize();
//
//        vertices_[v0].normal += face_normal;
//        vertices_[v1].normal += face_normal;
//        vertices_[v2].normal += face_normal;
//    }
//
//    for(auto &v : vertices_) { v.normal.normalize(); }
//
//    create_vertex_buffers(position_loc, normal_loc);
//}
//
//void TriSurface::create_vertex_buffers(int32_t position_loc, int32_t normal_loc)
//{
//    glGenBuffers(1, &vbo_);
//    glGenBuffers(1, &facebuffer_);
//
//    glBindBuffer(GL_ARRAY_BUFFER, vbo_);
//    glBufferData(GL_ARRAY_BUFFER,
//                 vertices_.size() * sizeof(VertexAndNormal),
//                 (void *)&vertices_[0],
//                 GL_STATIC_DRAW);
//
//    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, facebuffer_);
//    glBufferData(GL_ELEMENT_ARRAY_BUFFER,
//                 faces_.size() * sizeof(uint16_t),
//                 (void *)&faces_[0],
//                 GL_STATIC_DRAW);
//
//    face_count_ = static_cast<GLsizei>(faces_.size());
//
//    glGenVertexArrays(1, &vao_);
//    glBindVertexArray(vao_);
//
//    glBindBuffer(GL_ARRAY_BUFFER, vbo_);
//    glVertexAttribPointer(position_loc, 3, GL_FLOAT, GL_FALSE,
//                          sizeof(VertexAndNormal), (void *)0);
//    glVertexAttribPointer(normal_loc, 3, GL_FLOAT, GL_FALSE,
//                          sizeof(VertexAndNormal), (void *)(sizeof(Point3)));
//    glEnableVertexAttribArray(position_loc);
//    glEnableVertexAttribArray(normal_loc);
//
//    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, facebuffer_);
//    glBindVertexArray(0);
//}
//
//void TriSurface::construct_row_col_face_list(uint32_t num_rows, uint32_t num_cols)
//{
//    for(uint32_t row = 0; row < num_rows - 1; row++)
//    {
//        for(uint32_t col = 0; col < num_cols - 1; col++)
//        {
//            faces_.push_back(get_index(row + 1, col,     num_cols));
//            faces_.push_back(get_index(row,     col,     num_cols));
//            faces_.push_back(get_index(row,     col + 1, num_cols));
//
//            faces_.push_back(get_index(row + 1, col,     num_cols));
//            faces_.push_back(get_index(row,     col + 1, num_cols));
//            faces_.push_back(get_index(row + 1, col + 1, num_cols));
//        }
//    }
//}
//
//uint16_t TriSurface::get_index(uint32_t row, uint32_t col, uint32_t num_cols) const
//{
//    return static_cast<uint16_t>(row * num_cols + col);
//}
//
//uint16_t TriSurface::add_vertex(const Point3 &vtx)
//{
//    uint16_t index = 0;
//    for(const auto &v : vertices_)
//    {
//        if(vtx == v.vertex) return index;
//        index++;
//    }
//
//    vertices_.push_back(VertexAndNormal(vtx));
//    return static_cast<uint16_t>(vertices_.size() - 1);
//}
//
//// --- Ray tracing ---
//// The ray is transformed into local object space using the inverse model
//// matrix. The direction's length after transformation encodes the
//// world-to-local scale factor (local_ray_length). All intersection distances
//// are in local space and must be converted back:
////
//// world_t = local_t / local_ray_length
//// local_t = world_t * local_ray_length
//void TriSurface::compute_bounding_box()
//{
//    if(vertices_.empty()) return;
//
//    Point3 mn = vertices_[0].vertex;
//    Point3 mx = vertices_[0].vertex;
//    for(const auto &v : vertices_)
//    {
//        const Point3 &p = v.vertex;
//        if(p.x < mn.x) mn.x = p.x;  if(p.x > mx.x) mx.x = p.x;
//        if(p.y < mn.y) mn.y = p.y;  if(p.y > mx.y) mx.y = p.y;
//        if(p.z < mn.z) mn.z = p.z;  if(p.z > mx.z) mx.z = p.z;
//    }
//    local_bounds_.update(mn, mx);
//}
//
//void TriSurface::find_closest_intersect(Ray3 ray, SceneState &current_state,
//                                        SceneState &closest)
//{
//    // Transform ray origin (point, w=1) and direction (vector, w=0) to local space
//    HPoint3 local_o_h  = current_state.inverse_matrix * ray.o;
//    Point3  local_o(local_o_h.x / local_o_h.w,
//                    local_o_h.y / local_o_h.w,
//                    local_o_h.z / local_o_h.w);
//
//    Vector3 local_dir = current_state.inverse_matrix * ray.d;
//    float local_ray_length = local_dir.norm();
//    if(local_ray_length < 0.000001f) return;
//
//    Ray3 local_ray(local_o, local_dir * (1.0f / local_ray_length));
//
//    // AABB early rejection
//    if(!local_ray.intersect(local_bounds_).intersects) return;
//
//    // Convert the current world space closest distance to local space
//    float local_t_min = closest.t_min * local_ray_length;
//
//    // Build a position list
//    std::vector<Point3> positions;
//    positions.reserve(vertices_.size());
//    for(const auto &v : vertices_) positions.push_back(v.vertex);
//
//    auto result = local_ray.intersect(positions, faces_, local_t_min);
//    if(!result.intersects) return;
//
//    // Convert local distance back to world space
//    float world_t = result.distance / local_ray_length;
//    if(world_t >= closest.t_min) return;
//
//    // Record the hit
//    closest.t_min = world_t;
//    closest.geometry_node = this;
//    closest.material_node = current_state.material_node;
//    closest.texture_node = current_state.texture_node;
//
//    // Interpolate vertex normals using barycentric coordinates
//    uint32_t f_idx = result.face_index * 3;
//    float u = result.barycentric_u;
//    float v = result.barycentric_v;
//    float w = 1.0f - u - v;
//
//    Vector3 local_normal = vertices_[faces_[f_idx    ]].normal * w
//                         + vertices_[faces_[f_idx + 1]].normal * u
//                         + vertices_[faces_[f_idx + 2]].normal * v;
//    local_normal.normalize();
//
//    // Transform to world space via the normal matrix (transpose of inverse)
//    last_hit_normal_ = current_state.normal_matrix * local_normal;
//    last_hit_normal_.normalize();
//
//    last_hit_texcoord_ = Point2(u, v);
//}
//
//bool TriSurface::does_intersect_exist(Ray3 ray, float d, SceneState &current_state)
//{
//    HPoint3 local_o_h = current_state.inverse_matrix * ray.o;
//    Point3  local_o(local_o_h.x / local_o_h.w,
//                    local_o_h.y / local_o_h.w,
//                    local_o_h.z / local_o_h.w);
//
//    Vector3 local_dir = current_state.inverse_matrix * ray.d;
//    float   local_ray_length = local_dir.norm();
//    if(local_ray_length < 0.000001f) return false;
//
//    Ray3 local_ray(local_o, local_dir * (1.0f / local_ray_length));
//
//    if(!local_ray.intersect(local_bounds_).intersects) return false;
//
//    std::vector<Point3> positions;
//    positions.reserve(vertices_.size());
//    for(const auto &v : vertices_) positions.push_back(v.vertex);
//
//    return local_ray.does_intersect_exist(positions, faces_, d * local_ray_length);
//}
//
//Vector3 TriSurface::get_normal(const Point3 &point) { return last_hit_normal_; }
//
//Point2 TriSurface::get_texture_coord(const Point3 &point) { return last_hit_texcoord_; }
//
//} // namespace cg
