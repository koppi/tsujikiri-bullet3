-- GImpact: the AABB helpers, the triangle/tetrahedron primitives, and GImpact mesh shapes used as
-- dynamic concave bodies.

local T = require("testlib")
local P = require("physics")
local b = bullet3

local function vec(x, y, z)
    return b.btVector3(x, y, z)
end

local function at(x, y, z)
    return b.btTransform(b.btQuaternion(0, 0, 0, 1), vec(x, y, z))
end

-- The box spanned by three points: (0,0,0) .. (2,1,1)
local function sample_box(class)
    return (class or b.btAABB)(vec(0, 0, 0), vec(2, 1, 0), vec(1, 0, 1))
end

local function center_extent(aabb)
    local center, extent = vec(0, 0, 0), vec(0, 0, 0)
    aabb:get_center_extend(center, extent)
    return center, extent
end

-- ---------------------------------------------------------------------------------------
-- btAABB and GIM_AABB (the same API in two classes)
-- ---------------------------------------------------------------------------------------

for _, class_name in ipairs({ "btAABB", "GIM_AABB" }) do
    local class = b[class_name]

    T.test(class_name .. ": built from three points", function()
        local center, extent = center_extent(class(vec(0, 0, 0), vec(2, 1, 0), vec(1, 0, 1)))
        T.vec3(center, 1, 0.5, 0.5)
        T.vec3(extent, 1, 0.5, 0.5)

        -- with a margin on all sides
        center, extent = center_extent(class(vec(0, 0, 0), vec(2, 1, 0), vec(1, 0, 1), 0.5))
        T.vec3(center, 1, 0.5, 0.5)
        T.vec3(extent, 1.5, 1, 1)
    end)

    T.test(class_name .. ": copies, margins and merging", function()
        local original = sample_box(class)
        local copy = class(original)
        local _, extent = center_extent(copy)
        T.vec3(extent, 1, 0.5, 0.5)

        copy:increment_margin(1)
        _, extent = center_extent(copy)
        T.vec3(extent, 2, 1.5, 1.5)
        _, extent = center_extent(original)
        T.vec3(extent, 1, 0.5, 0.5, 1e-6)

        local with_margin = class()
        with_margin:copy_with_margin(original, 0.25)
        _, extent = center_extent(with_margin)
        T.vec3(extent, 1.25, 0.75, 0.75)
        T.ok(class(original, 0.5) ~= nil)

        local far = class(vec(10, 10, 10), vec(11, 11, 11), vec(10, 11, 10))
        original:merge(far)
        local center
        center, extent = center_extent(original)
        T.vec3(center, 5.5, 5.5, 5.5)
        T.vec3(extent, 5.5, 5.5, 5.5)
    end)

    T.test(class_name .. ": moving with a transform", function()
        local aabb = sample_box(class)
        aabb:appy_transform(at(10, 0, 0))
        local center, extent = center_extent(aabb)
        T.vec3(center, 11, 0.5, 0.5)
        T.vec3(extent, 1, 0.5, 0.5)
    end)

    T.test(class_name .. ": collision and intersection", function()
        local aabb = sample_box(class)
        local overlapping = class(vec(1, 0.5, 0.5), vec(3, 2, 2), vec(2, 1, 1))
        local apart = class(vec(5, 5, 5), vec(6, 6, 6), vec(5, 6, 5))
        T.ok(aabb:has_collision(overlapping))
        T.ok(not aabb:has_collision(apart))

        local intersection = class()
        aabb:find_intersection(overlapping, intersection)
        local center, extent = center_extent(intersection)
        T.vec3(center, 1.5, 0.75, 0.75)
        T.vec3(extent, 0.5, 0.25, 0.25)
    end)

    T.test(class_name .. ": rays and planes", function()
        local aabb = sample_box(class)
        T.ok(aabb:collide_ray(vec(-1, 0.5, 0.5), vec(1, 0, 0)), "a ray along x through the box")
        T.ok(not aabb:collide_ray(vec(-1, 5, 0.5), vec(1, 0, 0)), "a ray passing above")
        -- GImpact writes a plane as (n, w) with n.x = w
        T.ok(aabb:collide_plane(b.btVector4(0, 1, 0, 0.5)), "the plane y = 0.5 cuts the box")
        T.ok(not aabb:collide_plane(b.btVector4(0, 1, 0, 5)), "the plane y = 5 does not")
    end)

    T.test(class_name .. ": a triangle that touches the box", function()
        local aabb = sample_box(class)
        local plane = b.btVector4(0, 0, 1, 0.5) -- the triangle lies in z = 0.5
        T.ok(aabb:collide_triangle_exact(vec(0.5, 0.2, 0.5), vec(1.5, 0.2, 0.5), vec(1, 0.8, 0.5), plane))
        local far_plane = b.btVector4(0, 0, 1, 9)
        T.ok(not aabb:collide_triangle_exact(vec(0.5, 0.2, 9), vec(1.5, 0.2, 9), vec(1, 0.8, 9), far_plane))
    end)

    T.test(class_name .. ": boxes with a relative transform", function()
        local a = sample_box(class)
        local c = sample_box(class)
        T.ok(a:overlapping_trans_conservative(c, at(0.5, 0, 0)))
        T.ok(not a:overlapping_trans_conservative(c, at(10, 0, 0)))
    end)

    T.test(class_name .. ": invalidate", function()
        local aabb = sample_box(class)
        aabb:invalidate()
        T.ok(not aabb:has_collision(sample_box(class)))
    end)
end

T.test("btAABB: the box test through a cached transform", function()
    local cache = b.BT_BOX_BOX_TRANSFORM_CACHE()
    cache:calc_from_homogenic(at(0, 0, 0), at(0.5, 0, 0))
    cache:calc_absolute_matrix()
    T.ok(sample_box():overlapping_trans_cache(sample_box(), cache, true))
    T.ok(sample_box():overlapping_trans_conservative2(sample_box(), cache))
    sample_box():appy_transform_trans_cache(cache)

    cache:calc_from_homogenic(at(0, 0, 0), at(10, 0, 0))
    cache:calc_absolute_matrix()
    T.ok(not sample_box():overlapping_trans_cache(sample_box(), cache, true))
end)

T.test("BT_BOX_BOX_TRANSFORM_CACHE: the transform of box 1 into the frame of box 0", function()
    local cache = b.BT_BOX_BOX_TRANSFORM_CACHE()
    cache:calc_from_homogenic(at(0, 0, 0), at(1, 2, 3))
    T.vec3(cache:transform(vec(0, 0, 0)), 1, 2, 3)
    T.vec3(cache:transform(vec(1, 1, 1)), 2, 3, 4)

    cache:calc_from_full_invert(at(1, 0, 0), at(1, 2, 3))
    T.vec3(cache:transform(vec(0, 0, 0)), 0, 2, 3)
end)

-- ---------------------------------------------------------------------------------------
-- Triangles and tetrahedra
-- ---------------------------------------------------------------------------------------

T.test("btTriangleShapeEx: plane and overlap", function()
    local triangle = b.btTriangleShapeEx(vec(0, 0, 0), vec(2, 0, 0), vec(0, 2, 0))
    local plane = b.btVector4(0, 0, 0, 0)
    triangle:build_tri_plane(plane)
    T.vec3(plane, 0, 0, 1)
    T.near(plane:w(), 0, 1e-5)

    local crossing = b.btTriangleShapeEx(vec(0.5, 0.5, -1), vec(0.5, 0.5, 1), vec(1, 1, 0))
    local apart = b.btTriangleShapeEx(vec(0, 0, 5), vec(2, 0, 5), vec(0, 2, 5))
    T.ok(triangle:overlap_test_conservative(crossing))
    T.ok(not triangle:overlap_test_conservative(apart))

    triangle:apply_transform(at(0, 0, 3))
    triangle:build_tri_plane(plane)
    T.near(plane:w(), 3, 1e-5, "the plane moved with the triangle (n.x = w)")

    T.ok(b.btTriangleShapeEx(triangle) ~= nil)
    T.ok(b.btTriangleShapeEx() ~= nil)
    local min, max = vec(0, 0, 0), vec(0, 0, 0)
    triangle:get_aabb(at(0, 0, 0), min, max)
    T.ok(max:z() >= 3)
end)

T.test("btTetrahedronShapeEx and btBU_Simplex1to4: a growing simplex", function()
    local simplex = b.btBU_Simplex1to4()
    T.eq(simplex:get_num_vertices(), 0)
    simplex:add_vertex(vec(0, 0, 0))
    simplex:add_vertex(vec(1, 0, 0))
    T.eq(simplex:get_num_vertices(), 2)
    T.eq(simplex:get_num_edges(), 1)
    simplex:add_vertex(vec(0, 1, 0))
    simplex:add_vertex(vec(0, 0, 1))
    T.eq(simplex:get_num_vertices(), 4)
    T.eq(simplex:get_num_edges(), 6)
    T.eq(simplex:get_num_planes(), 4)
    T.eq(simplex:get_name(), "btBU_Simplex1to4")

    local vertex = vec(0, 0, 0)
    simplex:get_vertex(1, vertex)
    T.vec3(vertex, 1, 0, 0)
    local a, c = vec(0, 0, 0), vec(0, 0, 0)
    simplex:get_edge(0, a, c)
    T.near(a:distance(c), 1, 1e-5)
    T.eq(simplex:get_index(0), 0)
    T.ok(not simplex:is_inside(vec(0.1, 0.1, 0.1), 0.01), "Bullet's simplex never reports a point inside")
    simplex:reset()
    T.eq(simplex:get_num_vertices(), 0)

    T.eq(b.btBU_Simplex1to4(vec(0, 0, 0), vec(1, 0, 0)):get_num_vertices(), 2)
    T.eq(b.btBU_Simplex1to4(vec(0, 0, 0)):get_num_vertices(), 1)

    local tetrahedron = b.btTetrahedronShapeEx()
    tetrahedron:set_vertices(vec(0, 0, 0), vec(1, 0, 0), vec(0, 1, 0), vec(0, 0, 1))
    T.eq(tetrahedron:get_num_vertices(), 4)
    T.eq(tetrahedron:get_num_planes(), 4)
end)

-- ---------------------------------------------------------------------------------------
-- GImpact shapes
-- ---------------------------------------------------------------------------------------

-- A unit cube centred on the origin made of 12 triangles
local function cube_mesh()
    local mesh = b.btTriangleMesh()
    local h = 0.5
    local corners = {
        vec(-h, -h, -h), vec(h, -h, -h), vec(h, h, -h), vec(-h, h, -h),
        vec(-h, -h, h), vec(h, -h, h), vec(h, h, h), vec(-h, h, h),
    }
    local faces = {
        { 1, 2, 3 }, { 1, 3, 4 }, { 5, 7, 6 }, { 5, 8, 7 }, -- back, front
        { 1, 5, 6 }, { 1, 6, 2 }, { 4, 3, 7 }, { 4, 7, 8 }, -- bottom, top
        { 1, 4, 8 }, { 1, 8, 5 }, { 2, 6, 7 }, { 2, 7, 3 }, -- left, right
    }
    for _, f in ipairs(faces) do
        mesh:add_triangle(corners[f[1]], corners[f[2]], corners[f[3]])
    end
    return mesh
end

T.test("btGImpactMeshShape: a triangle mesh as a dynamic shape", function()
    local mesh = cube_mesh()
    local shape = b.btGImpactMeshShape(mesh)
    shape:set_margin(0.01)
    shape:update_bound()

    T.eq(shape:get_name(), "GImpactMesh")
    T.eq(shape:get_mesh_part_count(), 1)
    -- the primitives live in the mesh parts (the mesh shape's own counter is only an assertion)
    T.eq(shape:get_mesh_part(0):get_num_child_shapes(), 12)
    T.ok(shape:get_mesh_interface() ~= nil)
    T.ok(shape:get_mesh_part(0):needs_retrieve_triangles())
    T.ok(not shape:get_mesh_part(0):needs_retrieve_tetrahedrons())
    T.ok(not shape:get_mesh_part(0):children_has_transform())
    T.ok(shape:get_mesh_part(0):get_primitive_manager() ~= nil)

    local center, extent = center_extent(shape:get_local_box())
    T.vec3(center, 0, 0, 0, 1e-3)
    T.vec3(extent, 0.5, 0.5, 0.5, 0.05)

    local min, max = vec(0, 0, 0), vec(0, 0, 0)
    shape:get_aabb(at(3, 0, 0), min, max)
    T.near(min:x(), 2.5, 0.05)
    T.near(max:x(), 3.5, 0.05)
end)

T.test("btGImpactMeshShape: inertia is that of point masses on the vertices", function()
    local shape = b.btGImpactMeshShape(cube_mesh())
    shape:update_bound()
    local inertia = vec(0, 0, 0)
    shape:calculate_local_inertia(6, inertia)
    -- every vertex is 0.5 away from each axis plane: m * (y^2 + z^2) = 6 * 0.5
    T.vec3(inertia, 3, 3, 3, 0.05)
end)

T.test("btGImpactMeshShape: individual triangles", function()
    local shape = b.btGImpactMeshShape(cube_mesh())
    shape:update_bound()
    -- (the mesh shape forwards to its parts; the triangles are read from a part)
    local part = shape:get_mesh_part(0)
    part:lock_child_shapes()
    local triangle = b.btTriangleShapeEx()
    part:get_bullet_triangle(0, triangle)
    local vertex = vec(0, 0, 0)
    triangle:get_vertex(0, vertex)
    T.vec3(vertex, -0.5, -0.5, -0.5, 1e-4)
    triangle:get_vertex(1, vertex)
    T.vec3(vertex, 0.5, -0.5, -0.5, 1e-4)

    local primitive = b.btPrimitiveTriangle()
    part:get_primitive_triangle(0, primitive)
    primitive:build_tri_plane()
    local edge_plane = b.btVector4(0, 0, 0, 0)
    primitive:get_edge_plane(0, edge_plane)
    T.near(edge_plane:length(), 1, 1e-3, "an edge plane has a unit normal")
    primitive:apply_transform(at(1, 0, 0))

    -- the vertex data is only valid while the child shapes are locked
    local min, max = vec(0, 0, 0), vec(0, 0, 0)
    part:get_child_aabb(0, at(0, 0, 0), min, max)
    T.ok(max:x() >= min:x())
    part:unlock_child_shapes()
    T.ok(part:needs_retrieve_triangles())
end)

T.test("btGImpactMeshShapePart: one part of a mesh", function()
    local mesh = cube_mesh()
    local part = b.btGImpactMeshShapePart(mesh, 0)
    part:update_bound()
    T.eq(part:get_name(), "GImpactMeshShapePart")
    T.eq(part:get_part(), 0)
    T.eq(part:get_num_child_shapes(), 12)
    T.eq(part:get_vertex_count(), 36, "three vertices per triangle: duplicates are not merged")
    local vertex = vec(0, 0, 0)
    part:lock_child_shapes() -- the vertex data can only be read while locked
    part:get_vertex(0, vertex)
    part:unlock_child_shapes()
    T.near(math.abs(vertex:x()), 0.5, 1e-4)
    T.ok(part:get_trimesh_primitive_manager() ~= nil)
    part:set_margin(0.02)
    T.near(part:get_margin(), 0.02, 1e-6)
    part:set_local_scaling(vec(2, 2, 2))
    T.vec3(part:get_local_scaling(), 2, 2, 2)
    T.ok(b.btGImpactMeshShapePart() ~= nil)
end)

T.test("btGImpactCompoundShape: shapes combined into one GImpact shape", function()
    local box = b.btBoxShape(vec(0.5, 0.5, 0.5))
    local sphere = b.btSphereShape(0.5)
    local compound = b.btGImpactCompoundShape()
    T.eq(compound:get_name(), "GImpactCompound")
    compound:add_child_shape(at(-1, 0, 0), box)
    compound:add_child_shape(at(1, 0, 0), sphere)
    compound:update_bound()
    T.eq(compound:get_num_child_shapes(), 2)
    T.eq(compound:get_child_shape(0):get_name(), "Box")
    T.eq(compound:get_child_shape(1):get_name(), "SPHERE")
    T.ok(compound:children_has_transform())
    T.vec3(compound:get_child_transform(0):get_origin(), -1, 0, 0)
    compound:set_child_transform(1, at(2, 0, 0))
    T.vec3(compound:get_child_transform(1):get_origin(), 2, 0, 0)
    T.ok(compound:get_compound_primitive_manager() ~= nil)

    local min, max = vec(0, 0, 0), vec(0, 0, 0)
    compound:get_child_aabb(0, at(0, 0, 0), min, max)
    T.near(min:x(), -1.5, 0.1)
    T.ok(b.btGImpactCompoundShape(false) ~= nil)
    compound:add_child_shape(b.btBoxShape(vec(1, 1, 1))) -- without a transform
    T.eq(compound:get_num_child_shapes(), 3)
end)

T.test("btGImpactMeshShape: a cube falls onto a floor and rests on it", function()
    local p = P.new(T)
    b.btGImpactCollisionAlgorithm.register_algorithm(p.dispatcher)
    p:add_floor(0)

    local mesh = p:hold(cube_mesh())
    local shape = p:hold(b.btGImpactMeshShape(mesh))
    shape:set_margin(0.01)
    shape:update_bound()
    local body = p:add_body({ shape = shape, mass = 1, position = { 0, 2, 0 } })
    p:step(300)
    local _, y = P.position(body)
    T.near(y, 0.5, 0.1, "resting on a face of the cube")
    local _, vy = P.velocity(body)
    T.near(vy, 0, 0.1)
end)

T.test("btGImpactBvh: construction", function()
    T.ok(b.btGImpactBvh() ~= nil)
    local part = b.btGImpactMeshShapePart(cube_mesh(), 0)
    T.ok(b.btGImpactBvh(part:get_trimesh_primitive_manager()) ~= nil)
end)

T.run()
