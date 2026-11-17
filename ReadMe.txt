EN.605.767.8VL.SP26
Bryan Garry
Assignment 1 - Recursive Ray Tracer

========================================

Files Added / Modified:

- RayTracer/main.cpp
- RayTracer/ray_tracer.hpp/.cpp
- RayTracer/rt_sphere_node.hpp/.cpp
- RayTracer/rt_quad_node.hpp/.cpp
- RayTracer/rt_ellipsoid_node.hpp/.cpp
- RayTracer/checkerboard_texture.hpp/.cpp
- RayTracer/marble_texture.hpp/.cpp
- Scene/camera_node.hpp/.cpp
- Scene/scene_state.hpp/.cpp
- Scene/geometry_node.hpp/.cpp
- Scene/tri_surface.hpp/.cpp
- Scene/textured_tri_surface.hpp/.cpp
- Scene/transform_node.hpp/.cpp
- Scene/bounding_aabb_node.hpp/.cpp
- Scene/bounding_sphere_node.hpp/.cpp
- Geometry/aabb.hpp/.cpp
- Geometry/bounding_sphere.hpp/.cpp
- Geometry/ray3.hpp/.cpp
- Geometry/noise.hpp/.cpp

========================================

The approach to building this ray tracer was to improve the existing
scene graph framework from last semester, and extend it with new
intersection and shading capabilities. The core of the ray tracer is
the trace_ray() method in RayTracer which recursively evaluates
reflection and refraction rays up to a maximum depth, while using
adaptive depth testing based on a threshold for a ray’s contribution
weight to get the final color of a pixel. 

Scene graph traversal is essentially the same as when we 
implemented rasterization. The find_closest_intersect() method is
what propagates through the tree, and with each passing node the
SceneState is updated to carry forward important matrix, material,
and texture data. The closest intersections are used for shading within
the scene. Shadow rays are also handled with the does_intersect_exist()
method, which essentially checks to see if any object is blocking a
surface point from a light source.

I have implemented analytical intersection for spheres (RTSphereNode),
quads (RTQuadNode), ellipsoids (RTEllipsoidNode), and finite cylinders
(RTCylinderNode). Triangle mesh objects are also included here
(SphereSection and Torus), but any object with the tri_surface as it’s
base could reasonably be implemented. This is done using Moller-
Trumbore ray/triangle intersection. Each mesh is meant to store a 
local space AABB for testing before iterating through individual 
triangles. Each ray is transformed into local space using the inverse
modeling matrix, and the resulting t value is scaled back to world
space using a ration of the local ray direction length.

Bounding hierarchy is implemented here when constructing the scene
in the main.cpp file. Two AABBNodes are used to encompass various
subsets of objects within the scene to help speed up the ray tracing 
process. Ideally this helps reduce the number of intersection tests
based on rays missing the world space AABB and skipping their
children.

Local shading still uses the Phong reflection model with ambient,
diffuse, and specular components. I have placed two color lights at
the back of the room within the scene. A bluish and redish light are
used to try and give some interesting colored shadows and highlights
within the scene. Reflection rays are computed based on the incident
ray direction and surface normal. The refraction rays are computed
using Snell’s law alongside the object’s material properties (index
of refraction). Both of these rays are offset by a small epsilon along
the ray direction to avoid self-intersection. I did (and still) have some
issues with some artifacts and acne in my triangle mesh objects. I
am unsure where exactly these issues are stemming from as I have
tweaked the epsilon values to try to reduce the effect. I believe the
issue may still lie somewhere in my intersection calculations in
my tri_surface class, but am still not fully sure.

Image texturing was applied to two vertical cylinders in the scene
using a marble_column .jpg image (this is not to be confused with
the marble procedural texture). This works using UV coordinates
from a given intersection point. Procedural checkerboard and 
marble textures were also implemented on the floor and walls of
the scene. The marble texture utilizes Perlin noise alongside so
other tweak-able factors to get the desired effect.

Overall, the scene is meant to showcase a room with a checkerboard
floor, colored marble walls, a large ellipsoid mirror along the back wall,
two image textured pillars, two diamonds (SphereSection) and a torus 
scattered throughout, and a focus on three gem spheres in the center
meant to model the reflective and refractive properties of ruby,
sapphire, and emerald. The marble procedural texturing and quadric
objects where the two extensions I chose to implement. Attached is also
a screenshot of the scene rendered in 1280x960 resolution.

Not including the lib or include folders in submission as they make
the zip file too large.

(Using MacOS)
