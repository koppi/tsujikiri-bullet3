-- Collision shapes: construction, shape queries, AABBs, inertia and compound/mesh shapes.

local T = require("testlib")
local b = bullet3

local DEFAULT_MARGIN = 0.04

-- Shape type ids (BroadphaseNativeTypes)
local BOX, TRIANGLE, SPHERE, CAPSULE, CONE, CYLINDER = 0, 1, 8, 10, 11, 13
-- (Bullet registers btMinkowskiSumShape with the MINKOWSKI_DIFFERENCE type id)
local UNIFORM_SCALING, MINKOWSKI_SUM, BOX_2D, CONVEX_2D = 14, 16, 17, 18
local TRIANGLE_MESH, EMPTY, STATIC_PLANE, COMPOUND = 21, 27, 28, 31

local function identity_transform()
    local t = b.btTransform()
    t:set_identity()
    return t
end

local function transform_at(x, y, z)
    return b.btTransform(b.btQuaternion(0, 0, 0, 1), b.btVector3(x, y, z))
end

local function aabb_of(shape, transform)
    local min, max = b.btVector3(0, 0, 0), b.btVector3(0, 0, 0)
    shape:get_aabb(transform or identity_transform(), min, max)
    return min, max
end

local function inertia_of(shape, mass)
    local inertia = b.btVector3(9, 9, 9)
    shape:calculate_local_inertia(mass, inertia)
    return inertia
end

-- ---------------------------------------------------------------------------------------
-- btCollisionShape queries shared by all shapes
-- ---------------------------------------------------------------------------------------

T.test("shape kinds: names, types and category flags", function()
    local cases = {
        { shape = b.btBoxShape(b.btVector3(1, 1, 1)), name = "Box", type = BOX, convex = true, polyhedral = true },
        { shape = b.btSphereShape(1), name = "SPHERE", type = SPHERE, convex = true },
        { shape = b.btCapsuleShape(1, 2), name = "CapsuleShape", type = CAPSULE, convex = true },
        { shape = b.btCylinderShape(b.btVector3(1, 1, 1)), name = "CylinderY", type = CYLINDER, convex = true },
        { shape = b.btConeShape(1, 2), name = "Cone", type = CONE, convex = true },
        { shape = b.btStaticPlaneShape(b.btVector3(0, 1, 0), 0), name = "STATICPLANE", type = STATIC_PLANE,
            concave = true, non_moving = true, infinite = true },
        { shape = b.btEmptyShape(), name = "Empty", type = EMPTY, concave = true, non_moving = true },
        { shape = b.btCompoundShape(), name = "Compound", type = COMPOUND, compound = true },
        { shape = b.btTriangleShape(b.btVector3(0, 0, 0), b.btVector3(1, 0, 0), b.btVector3(0, 1, 0)),
            name = "Triangle", type = TRIANGLE, convex = true, polyhedral = true },
        { shape = b.btBox2dShape(b.btVector3(1, 1, 0)), name = "Box2d", type = BOX_2D, convex = true },
    }
    for _, case in ipairs(cases) do
        local shape = case.shape
        T.eq(shape:get_name(), case.name)
        T.eq(shape:get_shape_type(), case.type, case.name .. " shape type")
        T.eq(shape:is_convex(), case.convex or false, case.name .. " is_convex")
        T.eq(shape:is_concave(), case.concave or false, case.name .. " is_concave")
        T.eq(shape:is_polyhedral(), case.polyhedral or false, case.name .. " is_polyhedral")
        T.eq(shape:is_compound(), case.compound or false, case.name .. " is_compound")
        T.eq(shape:is_non_moving(), case.non_moving or false, case.name .. " is_non_moving")
        T.eq(shape:is_soft_body(), false, case.name .. " is_soft_body")
        T.eq(shape:is_infinite(), case.infinite or false, case.name .. " is_infinite")
    end
end)

T.test("btCollisionShape: user pointer and indices", function()
    local shape = b.btSphereShape(1)
    shape:set_user_index(7)
    shape:set_user_index2(8)
    T.eq(shape:get_user_index(), 7)
    T.eq(shape:get_user_index2(), 8)
    -- a void* comes back as userdata, a null one included
    T.type_is(shape:get_user_pointer(), "userdata")
end)

T.test("btCollisionShape: margin", function()
    local box = b.btBoxShape(b.btVector3(1, 1, 1))
    T.near(box:get_margin(), DEFAULT_MARGIN)
    box:set_margin(0.1)
    T.near(box:get_margin(), 0.1)
end)

T.test("btCollisionShape: contact breaking threshold and angular motion disc", function()
    local sphere = b.btSphereShape(2)
    -- the radius of the sphere around the AABB: 2 * sqrt(3)
    T.near(sphere:get_angular_motion_disc(), 2 * math.sqrt(3), 1e-4)
    -- the angular motion disc times the factor
    T.near(sphere:get_contact_breaking_threshold(0.02), 2 * math.sqrt(3) * 0.02, 1e-5)
end)

T.test("btCollisionShape: calculate_temporal_aabb sweeps the AABB along the velocity", function()
    local sphere = b.btSphereShape(1)
    local min, max = b.btVector3(0, 0, 0), b.btVector3(0, 0, 0)
    sphere:calculate_temporal_aabb(identity_transform(), b.btVector3(4, 0, 0), b.btVector3(0, 0, 0), 1, min, max)
    T.near(min:x(), -1, 1e-4)
    T.near(max:x(), 5, 1e-4)
    T.near(min:y(), -1, 1e-4)
    T.near(max:y(), 1, 1e-4)
end)

-- ---------------------------------------------------------------------------------------
-- Convex primitives
-- ---------------------------------------------------------------------------------------

T.test("btSphereShape: radius, scaling, AABB and inertia", function()
    local sphere = b.btSphereShape(2)
    T.near(sphere:get_radius(), 2)
    T.near(sphere:get_margin(), 2)

    local min, max = aabb_of(sphere, transform_at(1, 2, 3))
    T.vec3(min, -1, 0, 1)
    T.vec3(max, 3, 4, 5)

    -- 2/5 m r^2 about every axis
    local inertia = inertia_of(sphere, 5)
    T.vec3(inertia, 8, 8, 8)

    sphere:set_unscaled_radius(3)
    T.near(sphere:get_radius(), 3)
    sphere:set_local_scaling(b.btVector3(2, 2, 2))
    T.near(sphere:get_radius(), 6)
    T.vec3(sphere:get_local_scaling(), 2, 2, 2)
end)

T.test("btSphereShape: supporting vertex", function()
    local sphere = b.btSphereShape(2)
    T.vec3(sphere:local_get_supporting_vertex(b.btVector3(1, 0, 0)), 2, 0, 0)
    T.vec3(sphere:local_get_supporting_vertex(b.btVector3(0, -3, 0)), 0, -2, 0)
    T.vec3(sphere:local_get_supporting_vertex_without_margin(b.btVector3(1, 0, 0)), 0, 0, 0)
end)

T.test("btBoxShape: extents, AABB and inertia", function()
    local box = b.btBoxShape(b.btVector3(1, 2, 3))
    T.vec3(box:get_half_extents_with_margin(), 1, 2, 3)
    local without = box:get_half_extents_without_margin()
    T.vec3(without, 1 - DEFAULT_MARGIN, 2 - DEFAULT_MARGIN, 3 - DEFAULT_MARGIN)

    local min, max = aabb_of(box)
    T.vec3(min, -1, -2, -3)
    T.vec3(max, 1, 2, 3)
    min, max = aabb_of(box, transform_at(10, 0, 0))
    T.vec3(min, 9, -2, -3)
    T.vec3(max, 11, 2, 3)

    -- m/12 (ly^2 + lz^2), ... with l = 2 * half extents
    T.vec3(inertia_of(box, 12), 52, 40, 20)
end)

T.test("btBoxShape: topology", function()
    local box = b.btBoxShape(b.btVector3(1, 2, 3))
    T.eq(box:get_num_vertices(), 8)
    T.eq(box:get_num_edges(), 12)
    T.eq(box:get_num_planes(), 6)

    -- every vertex is a corner of the box
    for i = 0, 7 do
        local vertex = b.btVector3(0, 0, 0)
        box:get_vertex(i, vertex)
        T.near(math.abs(vertex:x()), 1)
        T.near(math.abs(vertex:y()), 2)
        T.near(math.abs(vertex:z()), 3)
    end
    -- every edge joins two corners that differ along one axis only
    for i = 0, 11 do
        local a, c = b.btVector3(0, 0, 0), b.btVector3(0, 0, 0)
        box:get_edge(i, a, c)
        local diff = a:distance(c)
        T.ok(math.abs(diff - 2) < 1e-4 or math.abs(diff - 4) < 1e-4 or math.abs(diff - 6) < 1e-4,
            "edge " .. i .. " has length " .. diff)
    end
    -- every plane has a unit axis-aligned normal, and its equation puts it one half extent
    -- (without the margin) away
    local extents = { 1, 2, 3 }
    for i = 0, 5 do
        local normal, support = b.btVector3(0, 0, 0), b.btVector3(0, 0, 0)
        box:get_plane(normal, support, i)
        T.near(normal:length(), 1, 1e-5)
        local equation = b.btVector4(0, 0, 0, 0)
        box:get_plane_equation(equation, i)
        T.vec3(equation, normal:x(), normal:y(), normal:z())
        local axis = (math.abs(normal:x()) > 0.5 and 1) or (math.abs(normal:y()) > 0.5 and 2) or 3
        T.near(math.abs(equation:w()), extents[axis] - DEFAULT_MARGIN, 1e-4)
    end
    T.ok(box:is_inside(b.btVector3(0, 0, 0), 0))
    T.ok(not box:is_inside(b.btVector3(5, 0, 0), 0))
end)

T.test("btBoxShape: supporting vertex follows the direction", function()
    local box = b.btBoxShape(b.btVector3(1, 2, 3))
    T.vec3(box:local_get_supporting_vertex(b.btVector3(1, 1, 1)), 1, 2, 3, 1e-4)
    T.vec3(box:local_get_supporting_vertex(b.btVector3(-1, 1, -1)), -1, 2, -3, 1e-4)
    T.vec3(box:local_get_supporting_vertex_without_margin(b.btVector3(1, 1, 1)),
        1 - DEFAULT_MARGIN, 2 - DEFAULT_MARGIN, 3 - DEFAULT_MARGIN, 1e-4)
end)

T.test("btBoxShape: local scaling scales the extents", function()
    local box = b.btBoxShape(b.btVector3(1, 1, 1))
    box:set_local_scaling(b.btVector3(2, 3, 4))
    T.vec3(box:get_local_scaling(), 2, 3, 4)
    local min, max = aabb_of(box)
    T.vec3(min, -2, -3, -4, 1e-4)
    T.vec3(max, 2, 3, 4, 1e-4)
end)

T.test("btCapsuleShape: axis variants", function()
    local y = b.btCapsuleShape(1, 2)
    T.eq(y:get_up_axis(), 1)
    T.near(y:get_radius(), 1)
    T.near(y:get_half_height(), 1)
    local min, max = aabb_of(y)
    T.vec3(min, -1, -2, -1)
    T.vec3(max, 1, 2, 1)
    -- box approximation of the inertia: l = (2, 4, 2)
    T.vec3(inertia_of(y, 12), 20, 8, 20)

    local x = b.btCapsuleShapeX(1, 2)
    T.eq(x:get_name(), "CapsuleX")
    T.eq(x:get_up_axis(), 0)
    min, max = aabb_of(x)
    T.vec3(max, 2, 1, 1)

    local z = b.btCapsuleShapeZ(1, 2)
    T.eq(z:get_name(), "CapsuleZ")
    T.eq(z:get_up_axis(), 2)
    min, max = aabb_of(z)
    T.vec3(max, 1, 1, 2)
    T.vec3(z:get_anisotropic_rolling_friction_direction(), 0, 0, 1)
end)

T.test("btCylinderShape: axis variants", function()
    local y = b.btCylinderShape(b.btVector3(1, 2, 1))
    T.eq(y:get_up_axis(), 1)
    T.near(y:get_radius(), 1)
    T.vec3(y:get_half_extents_with_margin(), 1, 2, 1)
    local min, max = aabb_of(y)
    T.vec3(min, -1, -2, -1)
    T.vec3(max, 1, 2, 1)
    -- radius^2 = 1, height^2 = 16: (m/12 h^2 + m/4 r^2, m/2 r^2, ...)
    T.vec3(inertia_of(y, 12), 19, 6, 19)

    local x = b.btCylinderShapeX(b.btVector3(2, 1, 1))
    T.eq(x:get_name(), "CylinderX")
    T.eq(x:get_up_axis(), 0)
    local z = b.btCylinderShapeZ(b.btVector3(1, 1, 2))
    T.eq(z:get_name(), "CylinderZ")
    T.eq(z:get_up_axis(), 2)
end)

T.test("btConeShape: radius, height and axis variants", function()
    local cone = b.btConeShape(1, 2)
    T.near(cone:get_radius(), 1)
    T.near(cone:get_height(), 2)
    T.eq(cone:get_cone_up_index(), 1)
    cone:set_radius(3)
    cone:set_height(4)
    T.near(cone:get_radius(), 3)
    T.near(cone:get_height(), 4)

    T.eq(b.btConeShapeX(1, 2):get_name(), "ConeX")
    T.eq(b.btConeShapeX(1, 2):get_cone_up_index(), 0)
    T.eq(b.btConeShapeZ(1, 2):get_name(), "ConeZ")
    T.eq(b.btConeShapeZ(1, 2):get_cone_up_index(), 2)

    -- the apex is the support point along +y, the base is wide at -y
    local fresh = b.btConeShape(1, 2)
    local top = fresh:local_get_supporting_vertex_without_margin(b.btVector3(0, 1, 0))
    T.near(top:y(), 1, 1e-4)
    local inertia = inertia_of(fresh, 12)
    T.near(inertia:x(), inertia:z(), 1e-4)
    T.ok(inertia:x() > 0 and inertia:y() > 0)
end)

T.test("btStaticPlaneShape: plane, AABB and inertia", function()
    local plane = b.btStaticPlaneShape(b.btVector3(0, 2, 0), 3)
    T.vec3(plane:get_plane_normal(), 0, 1, 0)
    T.near(plane:get_plane_constant(), 3)
    T.vec3(plane:get_local_scaling(), 1, 1, 1)
    plane:set_local_scaling(b.btVector3(1, 2, 3))
    T.vec3(plane:get_local_scaling(), 1, 2, 3)
    -- a static plane cannot move: zero inertia
    T.vec3(inertia_of(plane, 10), 0, 0, 0)
    local min, max = aabb_of(plane)
    T.ok(max:x() > 1e10 and min:x() < -1e10, "the AABB of a plane is practically infinite")
end)

T.test("btEmptyShape: nothing to collide with", function()
    -- (calculate_local_inertia is only an assertion for this shape, so it is not called)
    local empty = b.btEmptyShape()
    local min, max = aabb_of(empty, transform_at(1, 2, 3))
    T.vec3(min, 1, 2, 3, 1e-3)
    T.vec3(max, 1, 2, 3, 1e-3)
    empty:set_local_scaling(b.btVector3(2, 2, 2))
    T.vec3(empty:get_local_scaling(), 2, 2, 2)
end)

T.test("btTriangleShape: vertices, edges and plane", function()
    local a, c, d = b.btVector3(0, 0, 0), b.btVector3(2, 0, 0), b.btVector3(0, 2, 0)
    local triangle = b.btTriangleShape(a, c, d)
    T.eq(triangle:get_num_vertices(), 3)
    T.eq(triangle:get_num_edges(), 3)
    T.eq(triangle:get_num_planes(), 1)

    local vertex = b.btVector3(0, 0, 0)
    triangle:get_vertex(1, vertex)
    T.vec3(vertex, 2, 0, 0)
    T.vec3(triangle:get_vertex_ptr(2), 0, 2, 0)

    local pa, pb = b.btVector3(0, 0, 0), b.btVector3(0, 0, 0)
    triangle:get_edge(0, pa, pb)
    T.vec3(pa, 0, 0, 0)
    T.vec3(pb, 2, 0, 0)

    local normal = b.btVector3(0, 0, 0)
    triangle:calc_normal(normal)
    T.vec3(normal, 0, 0, 1)

    local plane_normal, plane_support = b.btVector3(0, 0, 0), b.btVector3(0, 0, 0)
    triangle:get_plane_equation(0, plane_normal, plane_support)
    T.vec3(plane_normal, 0, 0, 1)

    -- the AABB contains the triangle, padded by the margin
    local min, max = aabb_of(triangle)
    T.ok(min:x() < 0 and min:x() > -0.2 and min:y() < 0 and min:y() > -0.2 and min:z() < 0 and min:z() > -0.2)
    T.ok(max:x() > 2 and max:x() < 2.2 and max:y() > 2 and max:y() < 2.2 and max:z() > 0 and max:z() < 0.2)
end)

T.test("btTriangleShape: editing a vertex through get_vertex_ptr", function()
    local triangle = b.btTriangleShape(b.btVector3(0, 0, 0), b.btVector3(1, 0, 0), b.btVector3(0, 1, 0))
    triangle:get_vertex_ptr(0):set_value(5, 5, 5)
    local vertex = b.btVector3(0, 0, 0)
    triangle:get_vertex(0, vertex)
    T.vec3(vertex, 5, 5, 5)
end)

T.test("btBox2dShape: a flat box", function()
    local box = b.btBox2dShape(b.btVector3(1, 2, 0))
    T.eq(box:get_vertex_count(), 4)
    T.eq(box:get_num_vertices(), 4)
    T.eq(box:get_num_edges(), 12)
    T.eq(box:get_num_planes(), 6)
    T.vec3(box:get_centroid(), 0, 0, 0)
    T.vec3(box:get_half_extents_with_margin(), 1, 2, 0, 1e-4)
    -- the zero thickness sits inside the margin, so give the point some tolerance
    T.ok(box:is_inside(b.btVector3(0, 0, 0), 0.1))
    T.ok(not box:is_inside(b.btVector3(5, 0, 0), 0.1))
end)

-- ---------------------------------------------------------------------------------------
-- Shapes that wrap other shapes
-- ---------------------------------------------------------------------------------------

T.test("btUniformScalingShape: scales a convex child", function()
    local child = b.btBoxShape(b.btVector3(1, 1, 1))
    local scaled = b.btUniformScalingShape(child, 3)
    T.near(scaled:get_uniform_scaling_factor(), 3)
    T.eq(scaled:get_name(), "UniformScalingShape")
    T.eq(scaled:get_shape_type(), UNIFORM_SCALING)
    T.eq(scaled:get_child_shape():get_name(), "Box")
    local min, max = aabb_of(scaled)
    T.vec3(min, -3, -3, -3, 1e-3)
    T.vec3(max, 3, 3, 3, 1e-3)
end)

T.test("btConvex2dShape: wraps a convex child", function()
    local child = b.btBoxShape(b.btVector3(1, 1, 1))
    local flat = b.btConvex2dShape(child)
    T.eq(flat:get_name(), "Convex2dShape")
    T.eq(flat:get_shape_type(), CONVEX_2D)
    T.eq(flat:get_child_shape():get_name(), "Box")
end)

T.test("btMinkowskiSumShape: sum of two shapes", function()
    local a = b.btSphereShape(1)
    local c = b.btBoxShape(b.btVector3(1, 1, 1))
    local sum = b.btMinkowskiSumShape(a, c)
    T.eq(sum:get_name(), "MinkowskiSum")
    T.eq(sum:get_shape_type(), MINKOWSKI_SUM)
    T.eq(sum:get_shape_a():get_name(), "SPHERE")
    T.eq(sum:get_shape_b():get_name(), "Box")
    sum:set_transform_a(transform_at(1, 0, 0))
    sum:set_transform_b(transform_at(0, 2, 0))
    T.vec3(sum:get_transform_a():get_origin(), 1, 0, 0)
    T.vec3(sum:get_transform_b():get_origin(), 0, 2, 0)
end)

T.test("btCompoundShape: managing children", function()
    local sphere = b.btSphereShape(1)
    local box = b.btBoxShape(b.btVector3(1, 1, 1))
    local compound = b.btCompoundShape()
    T.eq(compound:get_num_child_shapes(), 0)

    compound:add_child_shape(transform_at(-5, 0, 0), sphere)
    compound:add_child_shape(transform_at(5, 0, 0), box)
    T.eq(compound:get_num_child_shapes(), 2)
    T.eq(compound:get_child_shape(0):get_name(), "SPHERE")
    T.eq(compound:get_child_shape(1):get_name(), "Box")
    T.vec3(compound:get_child_transform(0):get_origin(), -5, 0, 0)
    T.vec3(compound:get_child_transform(1):get_origin(), 5, 0, 0)

    local min, max = aabb_of(compound)
    T.vec3(min, -6, -1, -1, 1e-3)
    T.vec3(max, 6, 1, 1, 1e-3)

    compound:update_child_transform(1, transform_at(2, 0, 0))
    T.vec3(compound:get_child_transform(1):get_origin(), 2, 0, 0)
    min, max = aabb_of(compound)
    T.vec3(max, 3, 1, 1, 1e-3)

    compound:remove_child_shape(sphere)
    T.eq(compound:get_num_child_shapes(), 1)
    T.eq(compound:get_child_shape(0):get_name(), "Box")
    compound:remove_child_shape_by_index(0)
    T.eq(compound:get_num_child_shapes(), 0)
end)

T.test("btCompoundShape: update revision, AABB tree and inertia", function()
    local sphere = b.btSphereShape(1)
    local compound = b.btCompoundShape()
    T.ok(compound:get_dynamic_aabb_tree() ~= nil, "the dynamic AABB tree is on by default")
    local revision = compound:get_update_revision()
    compound:add_child_shape(transform_at(0, 0, 0), sphere)
    T.ne(compound:get_update_revision(), revision)

    -- the box around a unit sphere has l = 2: m/12 * (4 + 4)
    T.vec3(inertia_of(compound, 12), 8, 8, 8, 1e-3)

    local without_tree = b.btCompoundShape(false)
    T.is_nil(without_tree:get_dynamic_aabb_tree())
    without_tree:add_child_shape(transform_at(0, 0, 0), sphere)
    without_tree:create_aabb_tree_from_children()
    T.ok(without_tree:get_dynamic_aabb_tree() ~= nil)

    compound:set_margin(0.5)
    T.near(compound:get_margin(), 0.5)
    compound:set_local_scaling(b.btVector3(2, 2, 2))
    T.vec3(compound:get_local_scaling(), 2, 2, 2)
    compound:recalculate_local_aabb()
end)

-- ---------------------------------------------------------------------------------------
-- Triangle meshes
-- ---------------------------------------------------------------------------------------

local function make_quad_mesh()
    local mesh = b.btTriangleMesh()
    local v0, v1, v2, v3 = b.btVector3(0, 0, 0), b.btVector3(2, 0, 0), b.btVector3(2, 0, 2), b.btVector3(0, 0, 2)
    mesh:add_triangle(v0, v1, v2)
    mesh:add_triangle(v0, v2, v3)
    return mesh
end

T.test("btTriangleMesh: collects triangles", function()
    local mesh = make_quad_mesh()
    T.eq(mesh:get_num_triangles(), 2)
    T.ok(mesh:get_use32bit_indices())
    T.ok(mesh:get_use4component_vertices())

    local small = b.btTriangleMesh(false, false)
    T.ok(not small:get_use32bit_indices())
    T.ok(not small:get_use4component_vertices())
    small:add_triangle(b.btVector3(0, 0, 0), b.btVector3(1, 0, 0), b.btVector3(0, 1, 0))
    T.eq(small:get_num_triangles(), 1)
end)

T.test("btBvhTriangleMeshShape: a static concave mesh", function()
    local mesh = make_quad_mesh()
    local shape = b.btBvhTriangleMeshShape(mesh, true)
    T.eq(shape:get_name(), "BVHTRIANGLEMESH")
    T.eq(shape:get_shape_type(), TRIANGLE_MESH)
    T.ok(shape:is_concave())
    T.ok(shape:get_owns_bvh())
    T.ok(shape:uses_quantized_aabb_compression())
    T.ok(shape:get_optimized_bvh() ~= nil)
    T.vec3(inertia_of(shape, 10), 0, 0, 0)

    local min, max = aabb_of(shape)
    T.near(min:x(), 0, 0.1)
    T.near(max:x(), 2, 0.1)
    T.near(max:z(), 2, 0.1)
    shape:set_local_scaling(b.btVector3(2, 1, 1))
    T.vec3(shape:get_local_scaling(), 2, 1, 1)

    local uncompressed = b.btBvhTriangleMeshShape(mesh, false)
    T.ok(not uncompressed:uses_quantized_aabb_compression())
end)

T.test("btConvexTriangleMeshShape: a convex hull of a mesh", function()
    local mesh = make_quad_mesh()
    local shape = b.btConvexTriangleMeshShape(mesh)
    T.ok(shape:is_convex())
    T.ok(shape:is_polyhedral())
    T.eq(shape:get_name(), "ConvexTrimesh")
    T.eq(shape:get_mesh_interface():get_num_sub_parts(), 1)
end)

T.run()
