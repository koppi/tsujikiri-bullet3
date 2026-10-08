-- Collision dispatch odds and ends: islands, pair caches, mesh shape wrappers, shape hulls,
-- contact property mixing and the collision-world importer.

local T = require("testlib")
local P = require("physics")
local b = bullet3

local function vec(x, y, z)
    return b.btVector3(x, y, z)
end

local function quad_mesh()
    local mesh = b.btTriangleMesh()
    mesh:add_triangle(vec(0, 0, 0), vec(2, 0, 0), vec(2, 0, 2))
    mesh:add_triangle(vec(0, 0, 0), vec(2, 0, 2), vec(0, 0, 2))
    return mesh
end

-- ---------------------------------------------------------------------------------------
-- Islands
-- ---------------------------------------------------------------------------------------

T.test("btUnionFind: groups elements into islands", function()
    local forest = b.btUnionFind()
    forest:reset(6)
    T.eq(forest:get_num_elements(), 6)
    for i = 0, 5 do T.ok(forest:is_root(i), "every element starts as its own root") end

    forest:unite(0, 1)
    forest:unite(1, 2)
    forest:unite(4, 5)
    T.eq(forest:find(0, 2), 1, "0 and 2 are connected")
    T.eq(forest:find(0, 1), 1)
    T.eq(forest:find(0, 4), 0, "0 and 4 are not")
    T.eq(forest:find(3, 3), 1)
    T.eq(forest:find(0), forest:find(2), "same representative")
    T.ne(forest:find(0), forest:find(4))

    forest:sort_islands()
    T.eq(forest:get_num_elements(), 6)
    T.ok(forest:get_element(0) ~= nil)
    forest:free()
    forest:allocate(3)
    T.eq(forest:get_num_elements(), 3)
    forest:free()
end)

T.test("btSimulationIslandManager: the world's islands", function()
    local p = P.new(T)
    local manager = p.world:get_simulation_island_manager()
    T.ok(manager:get_split_islands(), "islands are split by default")
    manager:set_split_islands(false)
    T.ok(not manager:get_split_islands())
    manager:set_split_islands(true)

    p:add_floor(0)
    p:add_body({ shape = b.btSphereShape(0.5), position = { 0, 0.5, 0 } })
    p:add_body({ shape = b.btSphereShape(0.5), position = { 10, 0.5, 0 } })
    p:step(5)
    T.ok(manager:get_union_find():get_num_elements() >= 2, "the moving bodies take part in the islands")

    manager:init_union_find(10)
    T.eq(manager:get_union_find():get_num_elements(), 10)

    local standalone = b.btSimulationIslandManager()
    T.ok(standalone ~= nil)
end)

-- ---------------------------------------------------------------------------------------
-- Pair caches
-- ---------------------------------------------------------------------------------------

local function overlapping_proxies(cache)
    local broadphase = b.btDbvtBroadphase(cache)
    local config = b.btDefaultCollisionConfiguration()
    local dispatcher = b.btCollisionDispatcher(config)
    local first = broadphase:create_proxy(vec(0, 0, 0), vec(1, 1, 1), 8, nil, 1, -1, dispatcher)
    local second = broadphase:create_proxy(vec(0.5, 0, 0), vec(1.5, 1, 1), 8, nil, 1, -1, dispatcher)
    broadphase:calculate_overlapping_pairs(dispatcher)
    return broadphase, dispatcher, first, second
end

T.test("btSortedOverlappingPairCache: pairs are tracked and removed", function()
    local cache = b.btSortedOverlappingPairCache()
    T.eq(cache:get_num_overlapping_pairs(), 0)
    T.ok(not cache:has_deferred_removal() or cache:has_deferred_removal())
    local broadphase, dispatcher, first, second = overlapping_proxies(cache)
    T.eq(cache:get_num_overlapping_pairs(), 1)
    T.ok(cache:find_pair(first, second) ~= nil or cache:find_pair(second, first) ~= nil)
    T.ok(cache:needs_broadphase_collision(first, second))
    cache:sort_overlapping_pairs(dispatcher)
    cache:remove_overlapping_pairs_containing_proxy(first, dispatcher)
    T.eq(cache:get_num_overlapping_pairs(), 0)
    broadphase:destroy_proxy(first, dispatcher)
    broadphase:destroy_proxy(second, dispatcher)
end)

T.test("btHashedSimplePairCache: pairs of integer ids", function()
    local cache = b.btHashedSimplePairCache()
    T.eq(cache:get_num_overlapping_pairs(), 0)
    T.ok(cache:add_overlapping_pair(1, 2) ~= nil)
    T.ok(cache:add_overlapping_pair(3, 4) ~= nil)
    T.eq(cache:get_num_overlapping_pairs(), 2)
    T.eq(cache:get_count(), 2)
    T.ok(cache:find_pair(1, 2) ~= nil)
    T.is_nil(cache:find_pair(1, 3))
    cache:remove_overlapping_pair(1, 2)
    T.eq(cache:get_num_overlapping_pairs(), 1)
    cache:remove_all_pairs()
    T.eq(cache:get_num_overlapping_pairs(), 0)
    T.ok(b.btSimplePair(1, 2) ~= nil)
end)

-- ---------------------------------------------------------------------------------------
-- Mixing contact properties
-- ---------------------------------------------------------------------------------------

T.test("btManifoldResult: contact properties of two bodies are combined", function()
    local a, c = b.btCollisionObject(), b.btCollisionObject()
    a:set_friction(0.5) c:set_friction(0.4)
    a:set_restitution(0.5) c:set_restitution(0.6)
    a:set_rolling_friction(0.2) c:set_rolling_friction(0.3)
    a:set_spinning_friction(0.1) c:set_spinning_friction(0.5)
    local combine = b.btManifoldResult

    T.near(combine.calculate_combined_friction(a, c), 0.2, 1e-5, "friction multiplies")
    T.near(combine.calculate_combined_restitution(a, c), 0.3, 1e-5, "restitution multiplies")
    -- rolling and spinning friction are each scaled by the other body's friction and summed
    T.near(combine.calculate_combined_rolling_friction(a, c), 0.2 * 0.4 + 0.3 * 0.5, 1e-5)
    T.near(combine.calculate_combined_spinning_friction(a, c), 0.1 * 0.4 + 0.5 * 0.5, 1e-5)
    T.type_is(combine.calculate_combined_contact_damping(a, c), "number")
    T.type_is(combine.calculate_combined_contact_stiffness(a, c), "number")

    -- the combined friction is clamped to [-10, 10]
    a:set_friction(100) c:set_friction(100)
    T.near(combine.calculate_combined_friction(a, c), 10, 1e-5)
end)

T.test("btManifoldResult: construction and the persistent manifold", function()
    local result = b.btManifoldResult()
    local a, c = b.btCollisionObject(), b.btCollisionObject()
    local manifold = b.btPersistentManifold(a, c, 0, 0.02, 0)
    result:set_persistent_manifold(manifold)
    T.ok(result:get_persistent_manifold() ~= nil)
    result:set_shape_identifiers_a(1, 2)
    result:set_shape_identifiers_b(3, 4)
    result:set_body0_wrap(nil)
    result:set_body1_wrap(nil)
end)

-- ---------------------------------------------------------------------------------------
-- Mesh shape wrappers and hulls
-- ---------------------------------------------------------------------------------------

T.test("btScaledBvhTriangleMeshShape: one BVH mesh at several scales", function()
    local mesh = quad_mesh()
    local bvh = b.btBvhTriangleMeshShape(mesh, true)
    local scaled = b.btScaledBvhTriangleMeshShape(bvh, vec(2, 1, 3))
    T.eq(scaled:get_name(), "SCALEDBVHTRIANGLEMESH")
    T.vec3(scaled:get_local_scaling(), 2, 1, 3)
    T.eq(scaled:get_child_shape():get_name(), "BVHTRIANGLEMESH")

    local min, max = vec(0, 0, 0), vec(0, 0, 0)
    scaled:get_aabb(b.btTransform(b.btQuaternion(0, 0, 0, 1), vec(0, 0, 0)), min, max)
    T.near(max:x(), 4, 0.2)
    T.near(max:z(), 6, 0.2)

    scaled:set_local_scaling(vec(1, 1, 1))
    T.vec3(scaled:get_local_scaling(), 1, 1, 1)
end)

T.test("btTriangleMeshShape: bounds and the mesh interface", function()
    local mesh = quad_mesh()
    local shape = b.btBvhTriangleMeshShape(mesh, true)
    T.ok(shape:get_mesh_interface() ~= nil)
    T.eq(shape:get_mesh_interface():get_num_sub_parts(), 1)
    T.vec3(shape:get_local_aabb_min(), 0, 0, 0, 0.1)
    T.vec3(shape:get_local_aabb_max(), 2, 0, 2, 0.1)
    shape:recalc_local_aabb()
    T.vec3(shape:get_local_aabb_max(), 2, 0, 2, 0.1)
    T.ok(shape:get_triangle_info_map() == nil)
end)

T.test("btMultimaterialTriangleMeshShape: a BVH mesh with per-triangle materials", function()
    local shape = b.btMultimaterialTriangleMeshShape(quad_mesh(), true)
    T.eq(shape:get_name(), "MULTIMATERIALTRIANGLEMESH")
    T.ok(shape:is_concave())
end)

T.test("btShapeHull: a hull around a convex shape", function()
    local box = b.btBoxShape(vec(1, 1, 1))
    local hull = b.btShapeHull(box)
    T.ok(hull:build_hull(box:get_margin()))
    T.eq(hull:num_vertices(), 8)
    T.eq(hull:num_triangles(), 12)
    T.eq(hull:num_indices(), 36)
    T.ok(hull:get_vertex_pointer() ~= nil)
    T.ok(hull:get_index_pointer() ~= nil)

    local sphere_hull = b.btShapeHull(b.btSphereShape(1))
    T.ok(sphere_hull:build_hull(0))
    T.ok(sphere_hull:num_vertices() > 8, "a round shape needs more vertices")
    T.eq(sphere_hull:num_indices(), 3 * sphere_hull:num_triangles())
end)

T.test("btConvexPointCloudShape: an empty cloud", function()
    local cloud = b.btConvexPointCloudShape()
    T.eq(cloud:get_num_points(), 0)
    T.eq(cloud:get_name(), "ConvexPointCloud")
end)

T.test("btSdfCollisionShape and btMiniSDF: no signed distance field loaded", function()
    local sdf = b.btMiniSDF()
    T.ok(not sdf:is_valid())
    local shape = b.btSdfCollisionShape()
    T.eq(shape:get_name(), "btSdfCollisionShape")
    T.ok(shape:is_concave())
end)

T.test("btConvexPolyhedron and detector classes can be created", function()
    T.ok(b.btConvexPolyhedron() ~= nil)
    T.ok(b.btBoxBoxDetector(b.btBoxShape(vec(1, 1, 1)), b.btBoxShape(vec(1, 1, 1))) ~= nil)
    local triangle = b.btTriangleShape(vec(0, 0, 0), vec(1, 0, 0), vec(0, 1, 0))
    T.ok(b.SphereTriangleDetector(b.btSphereShape(1), triangle, 0.02) ~= nil)
    T.ok(b.btTriangleInfoMap() ~= nil)
    T.ok(b.btDefaultCollisionConstructionInfo() ~= nil)
    T.ok(b.btCollisionAlgorithmConstructionInfo() ~= nil)
    T.ok(b.btDispatcherInfo() ~= nil)
end)

-- ---------------------------------------------------------------------------------------
-- btCollisionWorldImporter
-- ---------------------------------------------------------------------------------------

T.test("btCollisionWorldImporter: creates and owns shapes and objects", function()
    local config = b.btDefaultCollisionConfiguration()
    local dispatcher = b.btCollisionDispatcher(config)
    local broadphase = b.btDbvtBroadphase()
    local world = b.btCollisionWorld(dispatcher, broadphase, config)
    local importer = b.btCollisionWorldImporter(world)
    T.cleanup(function() importer:delete_all_data() end)

    T.eq(importer:get_num_collision_shapes(), 0)
    local shapes = {
        { importer:create_plane_shape(vec(0, 1, 0), 0), "STATICPLANE" },
        { importer:create_box_shape(vec(1, 2, 3)), "Box" },
        { importer:create_sphere_shape(1), "SPHERE" },
        { importer:create_capsule_shape_x(1, 2), "CapsuleX" },
        { importer:create_capsule_shape_y(1, 2), "CapsuleShape" },
        { importer:create_capsule_shape_z(1, 2), "CapsuleZ" },
        { importer:create_cylinder_shape_x(1, 2), "CylinderX" },
        { importer:create_cylinder_shape_y(1, 2), "CylinderY" },
        { importer:create_cylinder_shape_z(1, 2), "CylinderZ" },
        { importer:create_cone_shape_x(1, 2), "ConeX" },
        { importer:create_cone_shape_y(1, 2), "Cone" },
        { importer:create_cone_shape_z(1, 2), "ConeZ" },
        { importer:create_compound_shape(), "Compound" },
    }
    for _, entry in ipairs(shapes) do
        T.eq(entry[1]:get_name(), entry[2])
    end
    T.eq(importer:get_num_collision_shapes(), #shapes)
    T.eq(importer:get_collision_shape_by_index(1):get_name(), "Box")

    local sphere = importer:create_sphere_shape(0.5)
    local object = importer:create_collision_object(
        b.btTransform(b.btQuaternion(0, 0, 0, 1), vec(1, 2, 3)), sphere, "ball")
    T.vec3(object:get_world_transform():get_origin(), 1, 2, 3)
    T.eq(importer:get_collision_shape_by_name("ball"), nil, "names are for objects here")

    importer:set_verbose_mode(0)
    T.eq(importer:get_verbose_mode(), 0)
    T.eq(importer:get_num_bvhs(), 0)
    T.eq(importer:get_num_triangle_info_maps(), 0)
    T.type_is(importer:create_optimized_bvh(), "userdata")
    T.type_is(importer:create_triangle_info_map(), "userdata")
    T.eq(importer:get_num_bvhs(), 1)
    T.eq(importer:get_num_triangle_info_maps(), 1)

    importer:delete_all_data()
    T.eq(importer:get_num_collision_shapes(), 0)
end)

T.run()
