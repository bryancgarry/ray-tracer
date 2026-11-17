//============================================================================
//    Johns Hopkins University Engineering Programs for Professionals
//    605.767 Applied Computer Graphics
//    Instructor:    Brian Russin
//
//    Author:  David W. Nesbitt
//    File:    RayTracer/main.cpp
//    Purpose: OpenGL program to draw polygon mesh objects.
//
//============================================================================

#include "common/event.hpp"
#include "common/logging.hpp"
#include "filesystem_support/file_locator.hpp"
#include "geometry/geometry.hpp"
#include "scene/graphics.hpp"
#include "scene/scene.hpp"

#include "RayTracer/framebuffer.hpp"
#include "RayTracer/lighting.hpp"
#include "RayTracer/ray_tracer.hpp"
#include "RayTracer/rt_sphere_node.hpp"
#include "RayTracer/rt_quad_node.hpp"
#include "RayTracer/rt_ellipsoid_node.hpp"
#include "RayTracer/rt_cylinder_node.hpp"
#include "RayTracer/checkerboard_texture.hpp"
#include "RayTracer/marble_texture.hpp"
#include "RayTracer/image_texture.hpp"
#include "scene/bounding_aabb_node.hpp"

#include <chrono>
#include <iostream>
#include <thread>
#include <vector>

#define MULTITHREAD

// SDL Objects
SDL_Window   *g_sdl_window = nullptr;
SDL_GLContext g_gl_context;

// Window width and height. Developement trick: for faster ray tracing keep
// these values small until final image is ready
int32_t g_image_width = 640;
int32_t g_image_height = 480;
//int32_t g_image_width = 1280;
//int32_t g_image_height = 960;


// Constants. Set up the view plane a distance of 1.0 from the camera.
// Set a field of view angle of 60 degrees.
float g_near_plane_distance = 1.0f;
float g_far_plane_distance = 300.0f;
float g_field_of_view = 60.0;

// Camera support. This will be part of the scene graph but keep a global pointer
// so we can maniplulate it with GLUT
std::shared_ptr<cg::CameraNode> g_camera;

// Mouse look state
bool    g_mouse_look   = false;  // true while right mouse button is held
int32_t g_last_mouse_x = 0;
int32_t g_last_mouse_y = 0;

// Tuning constants
const float MOVE_SPEED = 2.0f;   // world units per keypress
const float LOOK_SPEED = 0.3f;   // degrees per pixel of mouse movement

// Framebuffer
std::unique_ptr<cg::Framebuffer> g_frame_buffer;

// Maximum depth to trace and adaptive threshold.
int32_t     g_max_depth = 15;
const float DEPTH_THRESHOLD = 0.025f;

// Ray Tracer
cg::RayTracer *g_ray_tracer = 0;

// Light Nodes
std::shared_ptr<cg::LightNode> g_light1;
std::shared_ptr<cg::LightNode> g_light2;
std::shared_ptr<cg::LightNode> g_light3;

// Scene construction
std::shared_ptr<cg::SceneNode> g_scene_root;

// Bounding volume nodes — kept as globals so frustum_cull_scene() can reach them
std::shared_ptr<cg::AABBNode> g_gem_sphere_bounds;
std::shared_ptr<cg::AABBNode> g_mesh_bounds;

// Minimal SceneState used only for the pre-render frustum cull pass
cg::SceneState g_cull_state;

std::shared_ptr<cg::SceneNode> construct_scene(std::shared_ptr<cg::CameraNode> camera)
{
    auto scene_root = std::make_shared<cg::SceneNode>();

    // --- CAMERA SETUP ---
    cg::Point3 eye(-10.0f, -45.0f, 20.0f);
    cg::Point3 lookat(-5.0f, 0.0f, 20.0f);
//    cg::Point3 eye(0.0f, 40.0f, 20.0f);
//    cg::Point3 lookat(20.0f, 40.0f, 10.0f);
    camera->set_view_up(cg::Vector3(0.0f, 0.0f, 1.0f));
    camera->set_position_and_look_at_pt(eye, lookat);

    // --- LIGHTING ---
    g_light1 = std::make_shared<cg::LightNode>(0);
    g_light1->set_position(cg::HPoint3(-45.0f, -40.0f, 55.0f, 1.0f));
    g_light1->set_diffuse(cg::Color4(0.3f, 0.6f, 0.9f, 1.0f));
    scene_root->add_child(g_light1);
    
    g_light2 = std::make_shared<cg::LightNode>(1);
    g_light2->set_position(cg::HPoint3(45.0f, -40.0f, 55.0f, 1.0f));
    g_light2->set_diffuse(cg::Color4(0.9f, 0.6f, 0.3f, 1.0f));
    scene_root->add_child(g_light2);

    // --- ROOM CONSTRUCTION ---
    auto marble_tex = std::make_shared<cg::MarbleTexture>(cg::Color3(0.9f, 0.9f, 0.9f), cg::Color3(0.6f, 0.6f, 0.6f), 0.15f, 5.0f, 0.1f); // Marble Texture
    auto checkerboard_tex = std::make_shared<cg::CheckerboardTexture>(cg::Color3(0.95f, 0.95f, 0.95f), cg::Color3(0.0f, 0.0f, 0.0f), 10.0f); // Checkerboard Texture
    
    // Floor (Procedural Checkerboard)
    auto floor_quad = std::make_shared<cg::RTQuadNode>(cg::Point3(0,0,0), cg::Vector3(100,0,0), cg::Vector3(0,100,0));
        
    auto floor_mat = std::make_shared<cg::MaterialNode>();
    floor_mat->set_ambient_and_diffuse(cg::Color4(0.95f, 0.95f, 0.95f, 1.0f));
    floor_mat->set_specular(cg::Color4(1.0f, 1.0f, 1.0f, 1.0f));
    floor_mat->set_shininess(100.0f);
    floor_mat->set_global_reflectivity(0.3f, 0.3f, 0.3f);

    floor_mat->add_child(checkerboard_tex);
    checkerboard_tex->add_child(floor_quad);
    scene_root->add_child(floor_mat);
    
    // Back Wall
    auto back_wall_quad = std::make_shared<cg::RTQuadNode>(cg::Point3(0, 50, 30), cg::Vector3(100,0,0), cg::Vector3(0,0,60));
    
    auto back_wall_mat = std::make_shared<cg::MaterialNode>();
    back_wall_mat->set_ambient_and_diffuse(cg::Color4(0.7f, 0.5f, 0.3f, 1.0f));
    
//    back_wall_mat->add_child(marble_tex);
//    marble_tex->add_child(back_wall_quad);
//    scene_root->add_child(back_wall_mat);
    
    // Front Wall
    auto front_wall_quad = std::make_shared<cg::RTQuadNode>(cg::Point3(0, -50, 30), cg::Vector3(0,0,60), cg::Vector3(100,0,0));
    
    auto front_wall_mat = std::make_shared<cg::MaterialNode>();
    front_wall_mat->set_ambient_and_diffuse(cg::Color4(0.7f, 0.5f, 0.3f, 1.0f));
    
//    front_wall_mat->add_child(marble_tex);
//    marble_tex->add_child(front_wall_quad);
//    scene_root->add_child(front_wall_mat);
    
    // Side Walls
    auto left_wall_quad = std::make_shared<cg::RTQuadNode>(cg::Point3(-50, 0, 30), cg::Vector3(0,100,0), cg::Vector3(0,0,60));
    auto right_wall_quad = std::make_shared<cg::RTQuadNode>(cg::Point3(50, 0, 30), cg::Vector3(0,0,60), cg::Vector3(0,100,0));
    
    auto left_wall_mat = std::make_shared<cg::MaterialNode>();
    auto right_wall_mat = std::make_shared<cg::MaterialNode>();
    left_wall_mat->set_ambient_and_diffuse(cg::Color4(0.6f, 0.3f, 0.3f, 1.0f));
    right_wall_mat->set_ambient_and_diffuse(cg::Color4(0.3f, 0.3f, 0.6f, 1.0f));
    
//    left_wall_mat->add_child(marble_tex);
//    right_wall_mat->add_child(marble_tex);
//    marble_tex->add_child(left_wall_quad);
//    marble_tex->add_child(right_wall_quad);
//    scene_root->add_child(left_wall_mat);
//    scene_root->add_child(right_wall_mat);
    
    // Ceiling
    auto ceiling_quad = std::make_shared<cg::RTQuadNode>(cg::Point3(0,0,60), cg::Vector3(0,100,0), cg::Vector3(100,0,0));
    
    auto ceiling_mat = std::make_shared<cg::MaterialNode>();
    ceiling_mat->set_ambient_and_diffuse(cg::Color4(0.9f, 0.9f, 0.9f, 1.0f));
    
//    ceiling_mat->add_child(ceiling_quad);
//    scene_root->add_child(ceiling_mat);
    
    // --- OBJECT CONSTRUCTION ---
    // Back Mirror (RT_Ellipsoid)
    auto rt_ellipsoid = std::make_shared<cg::RTEllipsoidNode>(cg::Point3(0.0f, 50.0f, 30.0f), 40.0f, 5.0f, 20.0f);
    
    auto ellipsoid_mat = std::make_shared<cg::MaterialNode>();
    ellipsoid_mat->set_ambient_and_diffuse(cg::Color4(0.05f, 0.05f, 0.05f, 1.0f));
    ellipsoid_mat->set_specular(cg::Color4(1.0f, 1.0f, 1.0f, 1.0f));
    ellipsoid_mat->set_shininess(256.0f);
    ellipsoid_mat->set_global_reflectivity(0.9f, 0.9f, 0.9f);
    
//    ellipsoid_mat->add_child(rt_ellipsoid);
//    scene_root->add_child(ellipsoid_mat);
    
    // Image Textured Columns
    auto col_1 = std::make_shared<cg::RTCylinderNode>(cg::Point3(-30.0f, 30.0f, 30.0f), 2.0f, 30.0f);
    auto col_2 = std::make_shared<cg::RTCylinderNode>(cg::Point3(30.0f, 30.0f, 30.0f), 2.0f, 30.0f);
    auto column_image = std::make_shared<cg::ImageTexture>("textures/marble_column.jpg");
    
    auto column_mat = std::make_shared<cg::MaterialNode>();
    column_mat->set_ambient_and_diffuse(cg::Color4(0.9f, 0.85f, 0.80f, 1.0f));
    column_mat->set_specular(cg::Color4(1.0f, 1.0f, 1.0f, 1.0f));
    column_mat->set_shininess(96.0f);
    column_mat->set_index_of_refraction(1.486f);
    column_mat->set_global_reflectivity(0.08f, 0.08f, 0.08f);
        
//    column_mat->add_child(column_image);
//    column_image->add_child(col_1);
//    column_image->add_child(col_2);
//    scene_root->add_child(column_mat);
    
    // BOUNDING HIERARCHY - GEM SPHERES
    // AABBNode enclosing the 3 analytic gem spheres
    auto gem_sphere_bounds = std::make_shared<cg::AABBNode>();
    gem_sphere_bounds->set(cg::Point3(-35.0f, -5.0f,  0.0f),
                           cg::Point3( 35.0f, 25.0f, 25.0f));
    gem_sphere_bounds->set_name("Gem Spheres");
    g_gem_sphere_bounds = gem_sphere_bounds;
    scene_root->add_child(gem_sphere_bounds);

    // Sapphire Sphere (RTSphere 1)
    auto saph_sphere = std::make_shared<cg::RTSphereNode>(cg::Point3(-22.0f, 10.0f, 10.0f), 10.0f);
    auto saph_sphere_mat = std::make_shared<cg::MaterialNode>();
    saph_sphere_mat->set_ambient_and_diffuse(cg::Color4(0.0f, 0.05f, 0.2f, 1.0f));
    saph_sphere_mat->set_specular(cg::Color4(1.0f, 1.0f, 1.0f, 1.0f));
    saph_sphere_mat->set_shininess(128.0f);
    saph_sphere_mat->set_index_of_refraction(1.77f);
    saph_sphere_mat->set_global_reflectivity(0.15f, 0.15f, 0.15f);
    saph_sphere_mat->set_global_transmission(0.1f, 0.2f, 0.95f);
    saph_sphere_mat->add_child(saph_sphere);
    gem_sphere_bounds->add_child(saph_sphere_mat);

    // Ruby Sphere (RTSphere 2)
    auto ruby_sphere = std::make_shared<cg::RTSphereNode>(cg::Point3(22.0f, 10.0f, 10.0f), 10.0f);
    auto ruby_sphere_mat = std::make_shared<cg::MaterialNode>();
    ruby_sphere_mat->set_ambient_and_diffuse(cg::Color4(0.2f, 0.0f, 0.0f, 1.0f));
    ruby_sphere_mat->set_specular(cg::Color4(1.0f, 1.0f, 1.0f, 1.0f));
    ruby_sphere_mat->set_shininess(128.0f);
    ruby_sphere_mat->set_index_of_refraction(1.77f);
    ruby_sphere_mat->set_global_reflectivity(0.15f, 0.15f, 0.15f);
    ruby_sphere_mat->set_global_transmission(0.95f, 0.1f, 0.15f);
    ruby_sphere_mat->add_child(ruby_sphere);
    gem_sphere_bounds->add_child(ruby_sphere_mat);

    // Emerald Sphere (RTSphere 3)
    auto emerald_sphere = std::make_shared<cg::RTSphereNode>(cg::Point3(0.0f, 0.0f, 10.0f), 10.0f);
    auto emerald_sphere_mat = std::make_shared<cg::MaterialNode>();
    emerald_sphere_mat->set_ambient_and_diffuse(cg::Color4(0.0f, 0.2f, 0.05f, 1.0f));
    emerald_sphere_mat->set_specular(cg::Color4(1.0f, 1.0f, 1.0f, 1.0f));
    emerald_sphere_mat->set_shininess(128.0f);
    emerald_sphere_mat->set_index_of_refraction(1.58f);
    emerald_sphere_mat->set_global_reflectivity(0.12f, 0.12f, 0.12f);
    emerald_sphere_mat->set_global_transmission(0.15f, 0.95f, 0.2f);
    emerald_sphere_mat->add_child(emerald_sphere);
    gem_sphere_bounds->add_child(emerald_sphere_mat);
    
    // BOUNDING HIERARCHY - TRIANGLE MESH OBJECTS
    // AABBNode enclosing all 3 mesh objects (diamonds + torus)
    auto mesh_bounds = std::make_shared<cg::AABBNode>();
    mesh_bounds->set(cg::Point3(-25.0f, 18.0f,  0.0f), cg::Point3( 36.0f, 48.0f, 22.0f));
    mesh_bounds->set_name("Mesh Objects");
    g_mesh_bounds = mesh_bounds;
    scene_root->add_child(mesh_bounds);

    // Yellow Diamond (Low Poly Sphere Mesh)
    auto yellow_diamond_mesh = std::make_shared<cg::SphereSection>(-90.0f, 90.0f, 2, -180.0f, 180.0f, 4, 5.0f, 0, 1);

    auto yellow_diamond_transform = std::make_shared<cg::TransformNode>();
    yellow_diamond_transform->translate(-9.0f, 25.0f, 5.0f);

    auto yellow_diamond_mat = std::make_shared<cg::MaterialNode>();
    yellow_diamond_mat->set_ambient_and_diffuse(cg::Color4(0.8f, 0.8f, 0.0f, 1.0f));
    yellow_diamond_mat->set_specular(cg::Color4(1.0f, 1.0f, 1.0f, 1.0f));
    yellow_diamond_mat->set_shininess(256.0f);
    yellow_diamond_mat->set_global_reflectivity(0.12f, 0.12f, 0.12f);

//    yellow_diamond_transform->add_child(yellow_diamond_mat);
//    yellow_diamond_mat->add_child(yellow_diamond_mesh);
//    mesh_bounds->add_child(yellow_diamond_transform);

    // Cyan Diamond (Low Poly Sphere Mesh)
    auto cyan_diamond_mesh = std::make_shared<cg::SphereSection>(-90.0f, 90.0f, 2, -180.0f, 180.0f, 4, 7.0f, 0, 1);

    auto cyan_diamond_transform = std::make_shared<cg::TransformNode>();
    cyan_diamond_transform->translate(-16.0f, 35.0f, 12.0f);

    auto cyan_diamond_mat = std::make_shared<cg::MaterialNode>();
    cyan_diamond_mat->set_ambient_and_diffuse(cg::Color4(0.0f, 0.8f, 0.8f, 1.0f));
    cyan_diamond_mat->set_specular(cg::Color4(1.0f, 1.0f, 1.0f, 1.0f));
    cyan_diamond_mat->set_shininess(256.0f);
    cyan_diamond_mat->set_global_reflectivity(0.12f, 0.12f, 0.12f);

//    cyan_diamond_transform->add_child(cyan_diamond_mat);
//    cyan_diamond_mat->add_child(cyan_diamond_mesh);
//    mesh_bounds->add_child(cyan_diamond_transform);

    // Purple Torus
    auto torus_mesh = std::make_shared<cg::TorusSurface>(7.0f, 1.5f, 36, 18, 0, 1);

    auto torus_transform = std::make_shared<cg::TransformNode>();
    torus_transform->translate(25.0f, 37.0f, 12.0f);
    torus_transform->rotate_x(45.0f);
    torus_transform->rotate_y(-45.0f);

    auto torus_mat = std::make_shared<cg::MaterialNode>();
    torus_mat->set_diffuse(cg::Color4(0.45f, 0.0f, 0.55f, 1.0f));
    torus_mat->set_ambient(cg::Color4(0.08f, 0.0f, 0.12f, 1.0f));
    torus_mat->set_specular(cg::Color4(1.0f, 0.9f, 1.0f, 1.0f));
    torus_mat->set_shininess(192.0f);
    torus_mat->set_index_of_refraction(1.54f);
    torus_mat->set_global_reflectivity(0.12f, 0.08f, 0.14f);
    torus_mat->set_global_transmission(0.15f, 0.02f, 0.25f);
    
//    torus_transform->add_child(torus_mat);
//    torus_mat->add_child(torus_mesh);
//    mesh_bounds->add_child(torus_transform);
    
    // Return the scene root
    return scene_root;
}


/**
 * Pre-render frustum cull pass (Option B).
 * Extracts the current frustum planes from the camera, then tests each
 * top-level AABBNode.  Results are stored on the nodes themselves so
 * find_closest_intersect() can skip culled subtrees without any extra overhead.
 * A console message is printed whenever a node's visibility status changes.
 */
void frustum_cull_scene()
{
    g_camera->prepare_frustum(g_cull_state);
//    if(g_gem_sphere_bounds) g_gem_sphere_bounds->cull_check(g_cull_state);
//    if(g_mesh_bounds)       g_mesh_bounds->cull_check(g_cull_state);
}

void render_rows(int32_t start_row, int32_t end_row, int32_t block_size)
{
    for(int32_t row = start_row; row <= end_row; ++row)
    {
        int32_t y = row * block_size;
        for(int32_t x = 0; x < g_image_width; x += block_size)
        {
            // Check if framebuffer has been set for this pixel
            if(!g_frame_buffer->set(x, y))
            {
                // Construct a ray through the specified pixel and find the color
                // using the recursive ray tracer
                cg::Ray3 ray =
                    g_camera->construct_ray(static_cast<float>(x), static_cast<float>(y));
                cg::Color3 color = g_ray_tracer->trace_ray(ray, g_max_depth, DEPTH_THRESHOLD);
                g_frame_buffer->set(x, y, color, block_size);
            }
        }
    }
}

#ifdef MULTITHREAD

/**
 * Display function, multi-threaded
 */
void display()
{
    int32_t num_threads = std::thread::hardware_concurrency() - 1;
    num_threads = num_threads > 0 ? num_threads : 1;

    // Pre-render frustum cull pass — must run before any rays are cast
    frustum_cull_scene();

    // Clear the memory framebuffer
    g_frame_buffer->clear();

    for(int32_t block_size = cg::FB_BLOCK_SIZE; block_size > 0; block_size /= 2)
    {
        std::thread *threads = new std::thread[num_threads];
        int32_t      num_rows = g_image_height / block_size;
        int32_t      start_row = 0;

        for(int32_t t_i = 0; t_i < num_threads; ++t_i)
        {
            int32_t end_row = std::min((t_i + 1) * num_rows / num_threads, num_rows - 1);
            threads[t_i] = std::thread(render_rows, start_row, end_row, block_size);
            start_row = end_row + 1;
        }

        // Wait for threads to complete
        for(int32_t t_i = 0; t_i < num_threads; ++t_i) { threads[t_i].join(); }

        g_frame_buffer->render();
        SDL_GL_SwapWindow(g_sdl_window);
    }

    // Final scene - simple anti-aliasing (without shooting additional rays)
    g_frame_buffer->anti_alias();
    SDL_GL_SwapWindow(g_sdl_window);
}

#else

/**
 * Display function, single-threaded
 */
void display()
{
    // Pre-render frustum cull pass — must run before any rays are cast
    frustum_cull_scene();

    // Clear the memory framebuffer
    g_frame_buffer->clear();

    for(int32_t block_size = cg::FB_BLOCK_SIZE; block_size > 0; block_size /= 2)
    {
        render_rows(0, g_image_height+1, block_size);
        
        g_frame_buffer->render();
        SDL_GL_SwapWindow(g_sdl_window);
    }

    // Final scene - simple anti-aliasing (without shooting additional rays)
    g_frame_buffer->anti_alias();
    SDL_GL_SwapWindow(g_sdl_window);
}

#endif

/**
 * Reshape method.
 */
void reshape(int32_t width, int32_t height)
{
    // Update the view colume
    g_camera->set_view_volume(
        width, height, g_field_of_view, g_near_plane_distance, g_far_plane_distance);
    g_image_width = width;
    g_image_height = height;

    // Create framebuffer (old pointer is automatically deleted)
    g_frame_buffer = std::make_unique<cg::Framebuffer>(width, height);

    glViewport(0, 0, width, height);
}

/**
 * Window event handler.
 */
cg::EventType handle_window_event(const SDL_Event &event)
{
    cg::EventType result = cg::EventType::NONE;

    switch(event.type)
    {
        case SDL_EVENT_WINDOW_RESIZED:
        case SDL_EVENT_WINDOW_PIXEL_SIZE_CHANGED:
            reshape(event.window.data1, event.window.data2);
            result = cg::EventType::REDRAW;
            break;
        default: break;
    }

    return result;
}

/**
 * Keyboard event handler.
 */
cg::EventType handle_key_event(const SDL_Event &event)
{
    cg::EventType result = cg::EventType::NONE;

    bool upper_case = (event.key.mod & SDL_KMOD_SHIFT) || (event.key.mod & SDL_KMOD_CAPS);

    switch(event.key.key)
    {
        case SDLK_ESCAPE: result = cg::EventType::EXIT; break;

        // WASD - move forward / back / strafe left / strafe right
        case SDLK_W:
            g_camera->slide(0.0f, 0.0f, -MOVE_SPEED);
            result = cg::EventType::REDRAW;
            break;
        case SDLK_S:
            g_camera->slide(0.0f, 0.0f, MOVE_SPEED);
            result = cg::EventType::REDRAW;
            break;
        case SDLK_A:
            g_camera->slide(-MOVE_SPEED, 0.0f, 0.0f);
            result = cg::EventType::REDRAW;
            break;
        case SDLK_D:
            g_camera->slide(MOVE_SPEED, 0.0f, 0.0f);
            result = cg::EventType::REDRAW;
            break;

        // Q / E - move up / down
        case SDLK_Q:
            g_camera->slide(0.0f, MOVE_SPEED, 0.0f);
            result = cg::EventType::REDRAW;
            break;
        case SDLK_E:
            g_camera->slide(0.0f, -MOVE_SPEED, 0.0f);
            result = cg::EventType::REDRAW;
            break;

        // R - roll (kept from original)
        case SDLK_R:
            if(upper_case) g_camera->roll(-5.0f);
            else           g_camera->roll( 5.0f);
            result = cg::EventType::REDRAW;
            break;

        // Arrow keys - discrete pitch / heading for keyboard-only navigation
        case SDLK_UP:
            g_camera->pitch( 3.0f);
            result = cg::EventType::REDRAW;
            break;
        case SDLK_DOWN:
            g_camera->pitch(-3.0f);
            result = cg::EventType::REDRAW;
            break;
        case SDLK_LEFT:
            g_camera->heading( 3.0f);
            result = cg::EventType::REDRAW;
            break;
        case SDLK_RIGHT:
            g_camera->heading(-3.0f);
            result = cg::EventType::REDRAW;
            break;

        default: break;
    }

    return result;
}

/**
 * Mouse button event handler.
 * Hold right mouse button to enable mouse look.
 */
cg::EventType handle_mouse_button_event(const SDL_Event &event)
{
    if(event.button.button == SDL_BUTTON_RIGHT)
    {
        if(event.type == SDL_EVENT_MOUSE_BUTTON_DOWN)
        {
            g_mouse_look   = true;
            g_last_mouse_x = static_cast<int32_t>(event.button.x);
            g_last_mouse_y = static_cast<int32_t>(event.button.y);
        }
        else
        {
            g_mouse_look = false;
        }
    }
    return cg::EventType::NONE;
}

/**
 * Mouse motion event handler.
 * While right mouse button is held:
 *   horizontal drag → heading (yaw)
 *   vertical drag   → pitch
 */
cg::EventType handle_mouse_motion_event(const SDL_Event &event)
{
    if(!g_mouse_look) return cg::EventType::NONE;

    int32_t mx = static_cast<int32_t>(event.motion.x);
    int32_t my = static_cast<int32_t>(event.motion.y);

    int32_t dx = mx - g_last_mouse_x;
    int32_t dy = my - g_last_mouse_y;

    g_last_mouse_x = mx;
    g_last_mouse_y = my;

    if(dx != 0) g_camera->heading(-static_cast<float>(dx) * LOOK_SPEED);
    if(dy != 0) g_camera->pitch  (-static_cast<float>(dy) * LOOK_SPEED);

    return cg::EventType::REDRAW;
}

/**
 * Handle Events function.
 */
cg::EventType handle_events()
{
    cg::EventType result = cg::EventType::NONE;
    SDL_Event     e;
    while(SDL_PollEvent(&e))
    {
        switch(e.type)
        {
            case SDL_EVENT_QUIT:
            case SDL_EVENT_WINDOW_CLOSE_REQUESTED: result |= cg::EventType::EXIT; break;

            case SDL_EVENT_WINDOW_RESIZED:
            case SDL_EVENT_WINDOW_PIXEL_SIZE_CHANGED: result |= handle_window_event(e); break;

            case SDL_EVENT_KEY_UP: break;
            case SDL_EVENT_KEY_DOWN: result |= handle_key_event(e); break;

            case SDL_EVENT_MOUSE_BUTTON_DOWN:
            case SDL_EVENT_MOUSE_BUTTON_UP:   result |= handle_mouse_button_event(e); break;

            case SDL_EVENT_MOUSE_MOTION:      result |= handle_mouse_motion_event(e); break;

            default: break;
        }
    }
    return result;
}

// Sleep function to help run a reasonable timer
void sleep(int32_t milliseconds)
{
    std::this_thread::sleep_for(std::chrono::milliseconds(milliseconds));
}

/**
 * Main
 */
int main(int argc, char **argv)
{
    cg::set_root_paths(argv[0]);
    cg::init_logging("RayTracer.log");

    // Print options
    std::cout << "Camera Controls:\n";
    std::cout << "  W/S       - Move forward / backward\n";
    std::cout << "  A/D       - Strafe left / right\n";
    std::cout << "  Q/E       - Move up / down\n";
    std::cout << "  Arrow keys- Pitch and heading (discrete)\n";
    std::cout << "  R/r       - Roll camera\n";
    std::cout << "  RMB drag  - Mouse look (hold right mouse button)\n";
    std::cout << "  ESC       - Exit\n";

    // Initialize SDL
    if(!SDL_Init(SDL_INIT_VIDEO))
    {
        std::cout << "Error initializing SDL: " << SDL_GetError() << '\n';
        exit(1);
    }

    // Initialize display mode and window
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);

    SDL_PropertiesID props = SDL_CreateProperties();
    if(props == 0)
    {
        std::cout << "Error creating SDL Window Properties: " << SDL_GetError() << '\n';
        exit(1);
    }

    // Initialize display mode and window
    SDL_SetStringProperty(props, SDL_PROP_WINDOW_CREATE_TITLE_STRING, "Ray Tracer - Bryan Garry");
    SDL_SetBooleanProperty(props, SDL_PROP_WINDOW_CREATE_RESIZABLE_BOOLEAN, true);
    SDL_SetBooleanProperty(props, SDL_PROP_WINDOW_CREATE_OPENGL_BOOLEAN, true);
    SDL_SetNumberProperty(props, SDL_PROP_WINDOW_CREATE_WIDTH_NUMBER, g_image_width);
    SDL_SetNumberProperty(props, SDL_PROP_WINDOW_CREATE_HEIGHT_NUMBER, g_image_height);
    SDL_SetNumberProperty(props, SDL_PROP_WINDOW_CREATE_X_NUMBER, 100);
    SDL_SetNumberProperty(props, SDL_PROP_WINDOW_CREATE_Y_NUMBER, 100);

    g_sdl_window = SDL_CreateWindowWithProperties(props);
    if(g_sdl_window == nullptr)
    {
        std::cout << "Error initializing SDL Window" << SDL_GetError() << '\n';
        exit(1);
    }

    // Initialize OpenGL
    g_gl_context = SDL_GL_CreateContext(g_sdl_window);

    std::cout << "OpenGL  " << glGetString(GL_VERSION) << ", GLSL "
              << glGetString(GL_SHADING_LANGUAGE_VERSION) << '\n';

#if BUILD_WINDOWS
    int32_t glew_init_result = glewInit();
    if(GLEW_OK != glew_init_result)
    {
        std::cout << "GLEW Error: " << glewGetErrorString(glew_init_result) << std::endl;
        exit(EXIT_FAILURE);
    }
#endif

    g_frame_buffer = std::make_unique<cg::Framebuffer>(g_image_width, g_image_height);

    // 605.767 - Student to define. Set up camera parameters for initial view.
    g_camera = std::make_shared<cg::CameraNode>();

    // Initialize projection matrix immediately — set_view_volume() is normally
    // called from reshape() on SDL_EVENT_WINDOW_RESIZED, but that may fire after
    // the first render.  Calling it here ensures proj_ is valid before display().
    g_camera->set_view_volume(g_image_width, g_image_height,
                              g_field_of_view, g_near_plane_distance, g_far_plane_distance);

    // Construct the scene, pass in the camera node to add to the root node.
    auto scene_root = construct_scene(g_camera);

    // Construct ray tracer. Pass in the scene graph root node
    g_ray_tracer = new cg::RayTracer(scene_root);
    
    // Add lights to ray tracer
    g_ray_tracer->add_light(g_light1);
    g_ray_tracer->add_light(g_light2);

    // Initialize the projection matrix and frustum planes before the event loop.
    // reshape() is normally triggered by SDL_EVENT_WINDOW_RESIZED but that event
    // is not guaranteed to fire at startup, so we call it explicitly here.
    reshape(g_image_width, g_image_height);

    // Main loop
    cg::EventType event_result = cg::EventType::NONE;
    while(true)
    {
        event_result = handle_events();

        if(event_result & cg::EventType::EXIT) break;

        if(event_result & cg::EventType::REDRAW) display();
    }

    // Destroy OpenGL Context, SDL Window and SDL
    SDL_GL_DestroyContext(g_gl_context);
    SDL_DestroyWindow(g_sdl_window);
    SDL_Quit();
    return 0;
}

////============================================================================
////    Johns Hopkins University Engineering Programs for Professionals
////    605.767 Applied Computer Graphics
////    Instructor:    Brian Russin
////
////    Author:  David W. Nesbitt
////    File:    RayTracer/main.cpp
////    Purpose: OpenGL program to draw polygon mesh objects.
////
////============================================================================
//
//#include "common/event.hpp"
//#include "common/logging.hpp"
//#include "filesystem_support/file_locator.hpp"
//#include "geometry/geometry.hpp"
//#include "scene/graphics.hpp"
//#include "scene/scene.hpp"
//
//#include "RayTracer/framebuffer.hpp"
//#include "RayTracer/lighting.hpp"
//#include "RayTracer/ray_tracer.hpp"
//#include "RayTracer/rt_sphere_node.hpp"
//#include "RayTracer/rt_quad_node.hpp"
//#include "RayTracer/rt_ellipsoid_node.hpp"
//#include "RayTracer/rt_cylinder_node.hpp"
//#include "RayTracer/checkerboard_texture.hpp"
//#include "RayTracer/marble_texture.hpp"
//#include "RayTracer/image_texture.hpp"
//#include "scene/bounding_aabb_node.hpp"
//
//#include <chrono>
//#include <iostream>
//#include <thread>
//#include <vector>
//
//#define MULTITHREAD
//
//// SDL Objects
//SDL_Window   *g_sdl_window = nullptr;
//SDL_GLContext g_gl_context;
//
//// Window width and height. Developement trick: for faster ray tracing keep
//// these values small until final image is ready
//int32_t g_image_width = 640;
//int32_t g_image_height = 480;
////int32_t g_image_width = 1280;
////int32_t g_image_height = 960;
//
//
//// Constants. Set up the view plane a distance of 1.0 from the camera.
//// Set a field of view angle of 60 degrees.
//float g_near_plane_distance = 1.0f;
//float g_far_plane_distance = 300.0f;
//float g_field_of_view = 60.0;
//
//// Camera support. This will be part of the scene graph but keep a global pointer
//// so we can maniplulate it with GLUT
//std::shared_ptr<cg::CameraNode> g_camera;
//
//// Framebuffer
//std::unique_ptr<cg::Framebuffer> g_frame_buffer;
//
//// Maximum depth to trace and adaptive threshold.
//int32_t     g_max_depth = 15;
//const float DEPTH_THRESHOLD = 0.025f;
//
//// Ray Tracer
//cg::RayTracer *g_ray_tracer = 0;
//
//// Light Nodes
//std::shared_ptr<cg::LightNode> g_light1;
//std::shared_ptr<cg::LightNode> g_light2;
//std::shared_ptr<cg::LightNode> g_light3;
//
//// Scene construction
//std::shared_ptr<cg::SceneNode> g_scene_root;
//
//std::shared_ptr<cg::SceneNode> construct_scene(std::shared_ptr<cg::CameraNode> camera)
//{
//    auto scene_root = std::make_shared<cg::SceneNode>();
//
//    // --- CAMERA SETUP ---
//    cg::Point3 eye(-10.0f, -45.0f, 20.0f);
//    cg::Point3 lookat(-5.0f, 0.0f, 20.0f);
////    cg::Point3 eye(0.0f, 40.0f, 20.0f);
////    cg::Point3 lookat(20.0f, 40.0f, 10.0f);
//    camera->set_view_up(cg::Vector3(0.0f, 0.0f, 1.0f));
//    camera->set_position_and_look_at_pt(eye, lookat);
//
//    // --- LIGHTING ---
//    g_light1 = std::make_shared<cg::LightNode>(0);
//    g_light1->set_position(cg::HPoint3(-45.0f, -40.0f, 55.0f, 1.0f));
//    g_light1->set_diffuse(cg::Color4(0.3f, 0.6f, 0.9f, 1.0f));
//    scene_root->add_child(g_light1);
//    
//    g_light2 = std::make_shared<cg::LightNode>(1);
//    g_light2->set_position(cg::HPoint3(45.0f, -40.0f, 55.0f, 1.0f));
//    g_light2->set_diffuse(cg::Color4(0.9f, 0.6f, 0.3f, 1.0f));
//    scene_root->add_child(g_light2);
//
//    // --- ROOM CONSTRUCTION ---
//    auto marble_tex = std::make_shared<cg::MarbleTexture>(cg::Color3(0.9f, 0.9f, 0.9f), cg::Color3(0.6f, 0.6f, 0.6f), 0.15f, 5.0f, 0.1f); // Marble Texture
//    auto checkerboard_tex = std::make_shared<cg::CheckerboardTexture>(cg::Color3(0.95f, 0.95f, 0.95f), cg::Color3(0.0f, 0.0f, 0.0f), 10.0f); // Checkerboard Texture
//    
//    // Floor (Procedural Checkerboard)
//    auto floor_quad = std::make_shared<cg::RTQuadNode>(cg::Point3(0,0,0), cg::Vector3(100,0,0), cg::Vector3(0,100,0));
//        
//    auto floor_mat = std::make_shared<cg::MaterialNode>();
//    floor_mat->set_ambient_and_diffuse(cg::Color4(0.95f, 0.95f, 0.95f, 1.0f));
//    floor_mat->set_specular(cg::Color4(1.0f, 1.0f, 1.0f, 1.0f));
//    floor_mat->set_shininess(100.0f);
//    floor_mat->set_global_reflectivity(0.3f, 0.3f, 0.3f);
//
//    floor_mat->add_child(checkerboard_tex);
//    checkerboard_tex->add_child(floor_quad);
//    scene_root->add_child(floor_mat);
//    
//    // Back Wall
//    auto back_wall_quad = std::make_shared<cg::RTQuadNode>(cg::Point3(0, 50, 30), cg::Vector3(100,0,0), cg::Vector3(0,0,60));
//    
//    auto back_wall_mat = std::make_shared<cg::MaterialNode>();
//    back_wall_mat->set_ambient_and_diffuse(cg::Color4(0.7f, 0.5f, 0.3f, 1.0f));
//    
////    back_wall_mat->add_child(marble_tex);
////    marble_tex->add_child(back_wall_quad);
////    scene_root->add_child(back_wall_mat);
//    
//    // Front Wall
//    auto front_wall_quad = std::make_shared<cg::RTQuadNode>(cg::Point3(0, -50, 30), cg::Vector3(0,0,60), cg::Vector3(100,0,0));
//    
//    auto front_wall_mat = std::make_shared<cg::MaterialNode>();
//    front_wall_mat->set_ambient_and_diffuse(cg::Color4(0.7f, 0.5f, 0.3f, 1.0f));
//    
////    front_wall_mat->add_child(marble_tex);
////    marble_tex->add_child(front_wall_quad);
////    scene_root->add_child(front_wall_mat);
//    
//    // Side Walls
//    auto left_wall_quad = std::make_shared<cg::RTQuadNode>(cg::Point3(-50, 0, 30), cg::Vector3(0,100,0), cg::Vector3(0,0,60));
//    auto right_wall_quad = std::make_shared<cg::RTQuadNode>(cg::Point3(50, 0, 30), cg::Vector3(0,0,60), cg::Vector3(0,100,0));
//    
//    auto left_wall_mat = std::make_shared<cg::MaterialNode>();
//    auto right_wall_mat = std::make_shared<cg::MaterialNode>();
//    left_wall_mat->set_ambient_and_diffuse(cg::Color4(0.6f, 0.3f, 0.3f, 1.0f));
//    right_wall_mat->set_ambient_and_diffuse(cg::Color4(0.3f, 0.3f, 0.6f, 1.0f));
//    
////    left_wall_mat->add_child(marble_tex);
////    right_wall_mat->add_child(marble_tex);
////    marble_tex->add_child(left_wall_quad);
////    marble_tex->add_child(right_wall_quad);
////    scene_root->add_child(left_wall_mat);
////    scene_root->add_child(right_wall_mat);
//    
//    // Ceiling
//    auto ceiling_quad = std::make_shared<cg::RTQuadNode>(cg::Point3(0,0,60), cg::Vector3(0,100,0), cg::Vector3(100,0,0));
//    
//    auto ceiling_mat = std::make_shared<cg::MaterialNode>();
//    ceiling_mat->set_ambient_and_diffuse(cg::Color4(0.9f, 0.9f, 0.9f, 1.0f));
//    
////    ceiling_mat->add_child(ceiling_quad);
////    scene_root->add_child(ceiling_mat);
//    
//    // --- OBJECT CONSTRUCTION ---
//    // Back Mirror (RT_Ellipsoid)
//    auto rt_ellipsoid = std::make_shared<cg::RTEllipsoidNode>(cg::Point3(0.0f, 50.0f, 30.0f), 40.0f, 5.0f, 20.0f);
//    
//    auto ellipsoid_mat = std::make_shared<cg::MaterialNode>();
//    ellipsoid_mat->set_ambient_and_diffuse(cg::Color4(0.05f, 0.05f, 0.05f, 1.0f));
//    ellipsoid_mat->set_specular(cg::Color4(1.0f, 1.0f, 1.0f, 1.0f));
//    ellipsoid_mat->set_shininess(256.0f);
//    ellipsoid_mat->set_global_reflectivity(0.9f, 0.9f, 0.9f);
//    
////    ellipsoid_mat->add_child(rt_ellipsoid);
////    scene_root->add_child(ellipsoid_mat);
//    
//    // Image Textured Columns
//    auto col_1 = std::make_shared<cg::RTCylinderNode>(cg::Point3(-30.0f, 30.0f, 30.0f), 2.0f, 30.0f);
//    auto col_2 = std::make_shared<cg::RTCylinderNode>(cg::Point3(30.0f, 30.0f, 30.0f), 2.0f, 30.0f);
//    auto column_image = std::make_shared<cg::ImageTexture>("textures/marble_column.jpg");
//    
//    auto column_mat = std::make_shared<cg::MaterialNode>();
//    column_mat->set_ambient_and_diffuse(cg::Color4(0.9f, 0.85f, 0.80f, 1.0f));
//    column_mat->set_specular(cg::Color4(1.0f, 1.0f, 1.0f, 1.0f));
//    column_mat->set_shininess(96.0f);
//    column_mat->set_index_of_refraction(1.486f);
//    column_mat->set_global_reflectivity(0.08f, 0.08f, 0.08f);
//        
////    column_mat->add_child(column_image);
////    column_image->add_child(col_1);
////    column_image->add_child(col_2);
////    scene_root->add_child(column_mat);
//    
//    // BOUNDING HIERARCHY - GEM SPHERES
//    // AABBNode enclosing the 3 analytic gem spheres
//    auto gem_sphere_bounds = std::make_shared<cg::AABBNode>();
//    gem_sphere_bounds->set(cg::Point3(-35.0f, -5.0f,  0.0f),
//                           cg::Point3( 35.0f, 25.0f, 25.0f));
//    scene_root->add_child(gem_sphere_bounds);
//
//    // Sapphire Sphere (RTSphere 1)
//    auto saph_sphere = std::make_shared<cg::RTSphereNode>(cg::Point3(-22.0f, 10.0f, 10.0f), 10.0f);
//    auto saph_sphere_mat = std::make_shared<cg::MaterialNode>();
//    saph_sphere_mat->set_ambient_and_diffuse(cg::Color4(0.0f, 0.05f, 0.2f, 1.0f));
//    saph_sphere_mat->set_specular(cg::Color4(1.0f, 1.0f, 1.0f, 1.0f));
//    saph_sphere_mat->set_shininess(128.0f);
//    saph_sphere_mat->set_index_of_refraction(1.77f);
//    saph_sphere_mat->set_global_reflectivity(0.15f, 0.15f, 0.15f);
//    saph_sphere_mat->set_global_transmission(0.1f, 0.2f, 0.95f);
//    saph_sphere_mat->add_child(saph_sphere);
//    gem_sphere_bounds->add_child(saph_sphere_mat);
//
//    // Ruby Sphere (RTSphere 2)
//    auto ruby_sphere = std::make_shared<cg::RTSphereNode>(cg::Point3(22.0f, 10.0f, 10.0f), 10.0f);
//    auto ruby_sphere_mat = std::make_shared<cg::MaterialNode>();
//    ruby_sphere_mat->set_ambient_and_diffuse(cg::Color4(0.2f, 0.0f, 0.0f, 1.0f));
//    ruby_sphere_mat->set_specular(cg::Color4(1.0f, 1.0f, 1.0f, 1.0f));
//    ruby_sphere_mat->set_shininess(128.0f);
//    ruby_sphere_mat->set_index_of_refraction(1.77f);
//    ruby_sphere_mat->set_global_reflectivity(0.15f, 0.15f, 0.15f);
//    ruby_sphere_mat->set_global_transmission(0.95f, 0.1f, 0.15f);
//    ruby_sphere_mat->add_child(ruby_sphere);
//    gem_sphere_bounds->add_child(ruby_sphere_mat);
//
//    // Emerald Sphere (RTSphere 3)
//    auto emerald_sphere = std::make_shared<cg::RTSphereNode>(cg::Point3(0.0f, 0.0f, 10.0f), 10.0f);
//    auto emerald_sphere_mat = std::make_shared<cg::MaterialNode>();
//    emerald_sphere_mat->set_ambient_and_diffuse(cg::Color4(0.0f, 0.2f, 0.05f, 1.0f));
//    emerald_sphere_mat->set_specular(cg::Color4(1.0f, 1.0f, 1.0f, 1.0f));
//    emerald_sphere_mat->set_shininess(128.0f);
//    emerald_sphere_mat->set_index_of_refraction(1.58f);
//    emerald_sphere_mat->set_global_reflectivity(0.12f, 0.12f, 0.12f);
//    emerald_sphere_mat->set_global_transmission(0.15f, 0.95f, 0.2f);
//    emerald_sphere_mat->add_child(emerald_sphere);
//    gem_sphere_bounds->add_child(emerald_sphere_mat);
//    
//    // BOUNDING HIERARCHY - TRIANGLE MESH OBJECTS
//    // AABBNode enclosing all 3 mesh objects (diamonds + torus)
//    auto mesh_bounds = std::make_shared<cg::AABBNode>();
//    mesh_bounds->set(cg::Point3(-25.0f, 18.0f,  0.0f), cg::Point3( 36.0f, 48.0f, 22.0f));
//    scene_root->add_child(mesh_bounds);
//
//    // Yellow Diamond (Low Poly Sphere Mesh)
//    auto yellow_diamond_mesh = std::make_shared<cg::SphereSection>(-90.0f, 90.0f, 2, -180.0f, 180.0f, 4, 5.0f, 0, 1);
//
//    auto yellow_diamond_transform = std::make_shared<cg::TransformNode>();
//    yellow_diamond_transform->translate(-9.0f, 25.0f, 5.0f);
//
//    auto yellow_diamond_mat = std::make_shared<cg::MaterialNode>();
//    yellow_diamond_mat->set_ambient_and_diffuse(cg::Color4(0.8f, 0.8f, 0.0f, 1.0f));
//    yellow_diamond_mat->set_specular(cg::Color4(1.0f, 1.0f, 1.0f, 1.0f));
//    yellow_diamond_mat->set_shininess(256.0f);
//    yellow_diamond_mat->set_global_reflectivity(0.12f, 0.12f, 0.12f);
//
////    yellow_diamond_transform->add_child(yellow_diamond_mat);
////    yellow_diamond_mat->add_child(yellow_diamond_mesh);
////    mesh_bounds->add_child(yellow_diamond_transform);
//
//    // Cyan Diamond (Low Poly Sphere Mesh)
//    auto cyan_diamond_mesh = std::make_shared<cg::SphereSection>(-90.0f, 90.0f, 2, -180.0f, 180.0f, 4, 7.0f, 0, 1);
//
//    auto cyan_diamond_transform = std::make_shared<cg::TransformNode>();
//    cyan_diamond_transform->translate(-16.0f, 35.0f, 12.0f);
//
//    auto cyan_diamond_mat = std::make_shared<cg::MaterialNode>();
//    cyan_diamond_mat->set_ambient_and_diffuse(cg::Color4(0.0f, 0.8f, 0.8f, 1.0f));
//    cyan_diamond_mat->set_specular(cg::Color4(1.0f, 1.0f, 1.0f, 1.0f));
//    cyan_diamond_mat->set_shininess(256.0f);
//    cyan_diamond_mat->set_global_reflectivity(0.12f, 0.12f, 0.12f);
//
////    cyan_diamond_transform->add_child(cyan_diamond_mat);
////    cyan_diamond_mat->add_child(cyan_diamond_mesh);
////    mesh_bounds->add_child(cyan_diamond_transform);
//
//    // Purple Torus
//    auto torus_mesh = std::make_shared<cg::TorusSurface>(7.0f, 1.5f, 36, 18, 0, 1);
//
//    auto torus_transform = std::make_shared<cg::TransformNode>();
//    torus_transform->translate(25.0f, 37.0f, 12.0f);
//    torus_transform->rotate_x(45.0f);
//    torus_transform->rotate_y(-45.0f);
//
//    auto torus_mat = std::make_shared<cg::MaterialNode>();
//    torus_mat->set_diffuse(cg::Color4(0.45f, 0.0f, 0.55f, 1.0f));
//    torus_mat->set_ambient(cg::Color4(0.08f, 0.0f, 0.12f, 1.0f));
//    torus_mat->set_specular(cg::Color4(1.0f, 0.9f, 1.0f, 1.0f));
//    torus_mat->set_shininess(192.0f);
//    torus_mat->set_index_of_refraction(1.54f);
//    torus_mat->set_global_reflectivity(0.12f, 0.08f, 0.14f);
//    torus_mat->set_global_transmission(0.15f, 0.02f, 0.25f);
//    
////    torus_transform->add_child(torus_mat);
////    torus_mat->add_child(torus_mesh);
////    mesh_bounds->add_child(torus_transform);
//    
//    // Return the scene root
//    return scene_root;
//}
//
//
//void render_rows(int32_t start_row, int32_t end_row, int32_t block_size)
//{
//    for(int32_t row = start_row; row <= end_row; ++row)
//    {
//        int32_t y = row * block_size;
//        for(int32_t x = 0; x < g_image_width; x += block_size)
//        {
//            // Check if framebuffer has been set for this pixel
//            if(!g_frame_buffer->set(x, y))
//            {
//                // Construct a ray through the specified pixel and find the color
//                // using the recursive ray tracer
//                cg::Ray3 ray =
//                    g_camera->construct_ray(static_cast<float>(x), static_cast<float>(y));
//                cg::Color3 color = g_ray_tracer->trace_ray(ray, g_max_depth, DEPTH_THRESHOLD);
//                g_frame_buffer->set(x, y, color, block_size);
//            }
//        }
//    }
//}
//
//#ifdef MULTITHREAD
//
///**
// * Display function, multi-threaded
// */
//void display()
//{
//    int32_t num_threads = std::thread::hardware_concurrency() - 1;
//    num_threads = num_threads > 0 ? num_threads : 1;
//
//    // Clear the memory framebuffer
//    g_frame_buffer->clear();
//
//    for(int32_t block_size = cg::FB_BLOCK_SIZE; block_size > 0; block_size /= 2)
//    {
//        std::thread *threads = new std::thread[num_threads];
//        int32_t      num_rows = g_image_height / block_size;
//        int32_t      start_row = 0;
//
//        for(int32_t t_i = 0; t_i < num_threads; ++t_i)
//        {
//            int32_t end_row = std::min((t_i + 1) * num_rows / num_threads, num_rows - 1);
//            threads[t_i] = std::thread(render_rows, start_row, end_row, block_size);
//            start_row = end_row + 1;
//        }
//
//        // Wait for threads to complete
//        for(int32_t t_i = 0; t_i < num_threads; ++t_i) { threads[t_i].join(); }
//
//        g_frame_buffer->render();
//        SDL_GL_SwapWindow(g_sdl_window);
//    }
//
//    // Final scene - simple anti-aliasing (without shooting additional rays)
//    g_frame_buffer->anti_alias();
//    SDL_GL_SwapWindow(g_sdl_window);
//}
//
//#else
//
///**
// * Display function, single-threaded
// */
//void display()
//{
//    // Clear the memory framebuffer
//    g_frame_buffer->clear();
//
//    for(int32_t block_size = cg::FB_BLOCK_SIZE; block_size > 0; block_size /= 2)
//    {
//        render_rows(0, g_image_height+1, block_size);
//        
//        g_frame_buffer->render();
//        SDL_GL_SwapWindow(g_sdl_window);
//    }
//
//    // Final scene - simple anti-aliasing (without shooting additional rays)
//    g_frame_buffer->anti_alias();
//    SDL_GL_SwapWindow(g_sdl_window);
//}
//
//#endif
//
///**
// * Reshape method.
// */
//void reshape(int32_t width, int32_t height)
//{
//    // Update the view colume
//    g_camera->set_view_volume(
//        width, height, g_field_of_view, g_near_plane_distance, g_far_plane_distance);
//    g_image_width = width;
//    g_image_height = height;
//
//    // Create framebuffer (old pointer is automatically deleted)
//    g_frame_buffer = std::make_unique<cg::Framebuffer>(width, height);
//
//    glViewport(0, 0, width, height);
//}
//
///**
// * Window event handler.
// */
//cg::EventType handle_window_event(const SDL_Event &event)
//{
//    cg::EventType result = cg::EventType::NONE;
//
//    switch(event.type)
//    {
//        case SDL_EVENT_WINDOW_RESIZED:
//        case SDL_EVENT_WINDOW_PIXEL_SIZE_CHANGED:
//            reshape(event.window.data1, event.window.data2);
//            result = cg::EventType::REDRAW;
//            break;
//        default: break;
//    }
//
//    return result;
//}
//
///**
// * Keyboard event handler.
// */
//cg::EventType handle_key_event(const SDL_Event &event)
//{
//    cg::EventType result = cg::EventType::NONE;
//
//    bool upper_case = (event.key.mod & SDL_KMOD_SHIFT) || (event.key.mod & SDL_KMOD_CAPS);
//
//    switch(event.key.key)
//    {
//        case SDLK_ESCAPE: result = cg::EventType::EXIT; break;
//
//        // Roll the camera by +/-5 degrees (no need to update spotlight)
//        case SDLK_R:
//            if(upper_case) g_camera->roll(-5);
//            else g_camera->roll(5);
//            result = cg::EventType::REDRAW;
//            break;
//
//        // Change the pitch of the camera by +/-5 degrees
//        case SDLK_P:
//            if(upper_case) g_camera->pitch(-5);
//            else g_camera->pitch(5);
//            result = cg::EventType::REDRAW;
//            break;
//
//        // Change the heading of the camera by +/-5 degrees
//        case SDLK_H:
//            if(upper_case) g_camera->heading(-5);
//            else g_camera->heading(5);
//            result = cg::EventType::REDRAW;
//            break;
//
//        default: break;
//    }
//
//    return result;
//}
//
///**
// * Handle Events function.
// */
//cg::EventType handle_events()
//{
//    cg::EventType result = cg::EventType::NONE;
//    SDL_Event     e;
//    while(SDL_PollEvent(&e))
//    {
//        switch(e.type)
//        {
//            case SDL_EVENT_QUIT:
//            case SDL_EVENT_WINDOW_CLOSE_REQUESTED: result |= cg::EventType::EXIT; break;
//
//            case SDL_EVENT_WINDOW_RESIZED:
//            case SDL_EVENT_WINDOW_PIXEL_SIZE_CHANGED: result |= handle_window_event(e); break;
//
//            case SDL_EVENT_KEY_UP: break;
//            case SDL_EVENT_KEY_DOWN: result |= handle_key_event(e); break;
//
//            default: break;
//        }
//    }
//    return result;
//}
//
//// Sleep function to help run a reasonable timer
//void sleep(int32_t milliseconds)
//{
//    std::this_thread::sleep_for(std::chrono::milliseconds(milliseconds));
//}
//
///**
// * Main
// */
//int main(int argc, char **argv)
//{
//    cg::set_root_paths(argv[0]);
//    cg::init_logging("RayTracer.log");
//
//    // Print options
//    std::cout << "Transforms:" << std::endl;
//    std::cout << "r,R - Change camera roll\n";
//    std::cout << "p,P - Change camera pitch\n";
//    std::cout << "h,H - Change camera heading\n";
//
//    // Initialize SDL
//    if(!SDL_Init(SDL_INIT_VIDEO))
//    {
//        std::cout << "Error initializing SDL: " << SDL_GetError() << '\n';
//        exit(1);
//    }
//
//    // Initialize display mode and window
//    SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);
//
//    SDL_PropertiesID props = SDL_CreateProperties();
//    if(props == 0)
//    {
//        std::cout << "Error creating SDL Window Properties: " << SDL_GetError() << '\n';
//        exit(1);
//    }
//
//    // Initialize display mode and window
//    SDL_SetStringProperty(props, SDL_PROP_WINDOW_CREATE_TITLE_STRING, "Ray Tracer - Bryan Garry");
//    SDL_SetBooleanProperty(props, SDL_PROP_WINDOW_CREATE_RESIZABLE_BOOLEAN, true);
//    SDL_SetBooleanProperty(props, SDL_PROP_WINDOW_CREATE_OPENGL_BOOLEAN, true);
//    SDL_SetNumberProperty(props, SDL_PROP_WINDOW_CREATE_WIDTH_NUMBER, g_image_width);
//    SDL_SetNumberProperty(props, SDL_PROP_WINDOW_CREATE_HEIGHT_NUMBER, g_image_height);
//    SDL_SetNumberProperty(props, SDL_PROP_WINDOW_CREATE_X_NUMBER, 100);
//    SDL_SetNumberProperty(props, SDL_PROP_WINDOW_CREATE_Y_NUMBER, 100);
//
//    g_sdl_window = SDL_CreateWindowWithProperties(props);
//    if(g_sdl_window == nullptr)
//    {
//        std::cout << "Error initializing SDL Window" << SDL_GetError() << '\n';
//        exit(1);
//    }
//
//    // Initialize OpenGL
//    g_gl_context = SDL_GL_CreateContext(g_sdl_window);
//
//    std::cout << "OpenGL  " << glGetString(GL_VERSION) << ", GLSL "
//              << glGetString(GL_SHADING_LANGUAGE_VERSION) << '\n';
//
//#if BUILD_WINDOWS
//    int32_t glew_init_result = glewInit();
//    if(GLEW_OK != glew_init_result)
//    {
//        std::cout << "GLEW Error: " << glewGetErrorString(glew_init_result) << std::endl;
//        exit(EXIT_FAILURE);
//    }
//#endif
//
//    g_frame_buffer = std::make_unique<cg::Framebuffer>(g_image_width, g_image_height);
//
//    // 605.767 - Student to define. Set up camera parameters for initial view.
//    g_camera = std::make_shared<cg::CameraNode>();
//
//    // Construct the scene, pass in the camera node to add to the root node.
//    auto scene_root = construct_scene(g_camera);
//
//    // Construct ray tracer. Pass in the scene graph root node
//    g_ray_tracer = new cg::RayTracer(scene_root);
//    
//    // Add lights to ray tracer
//    g_ray_tracer->add_light(g_light1);
//    g_ray_tracer->add_light(g_light2);
//
//    // Main loop
//    cg::EventType event_result = cg::EventType::NONE;
//    while(true)
//    {
//        event_result = handle_events();
//
//        if(event_result & cg::EventType::EXIT) break;
//
//        if(event_result & cg::EventType::REDRAW) display();
//    }
//
//    // Destroy OpenGL Context, SDL Window and SDL
//    SDL_GL_DestroyContext(g_gl_context);
//    SDL_DestroyWindow(g_sdl_window);
//    SDL_Quit();
//    return 0;
//}
