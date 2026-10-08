-- Soft bodies: the btSoftBodyHelpers factories, btSoftBody itself, and btSoftRigidDynamicsWorld.
--
-- btSoftBody has no bound constructor, and its nodes, links and configuration are public data
-- members, which are not bound either. The helpers create the bodies; node and link counts are
-- observed through masses, link checks, volumes and bounds.

local T = require("testlib")
local P = require("physics")
local b = bullet3

local function vec(x, y, z)
    return b.btVector3(x, y, z)
end

local function bounds(body)
    local min, max = vec(0, 0, 0), vec(0, 0, 0)
    body:get_aabb(min, max)
    return min, max
end

-- A world that holds soft bodies, and the info object the bodies are created with
local function soft_world()
    local p = P.new(T, { soft = true })
    return p, p.world:get_world_info()
end

-- ---------------------------------------------------------------------------------------
-- Factories
-- ---------------------------------------------------------------------------------------

T.test("btSoftBodyHelpers.create_rope: a chain of res + 2 nodes", function()
    local p, info = soft_world()
    -- 9 interior nodes plus the two end points; the first one is fixed
    local rope = b.btSoftBodyHelpers.create_rope(info, vec(0, 0, 0), vec(10, 0, 0), 9, 1)
    T.ok(rope ~= nil)

    T.near(rope:get_mass(0), 0, 1e-6, "a fixed node has no mass")
    T.near(rope:get_mass(1), 1, 1e-6)
    T.near(rope:get_mass(10), 1, 1e-6)
    T.near(rope:get_total_mass(), 10, 1e-4)

    for i = 0, 9 do
        T.ok(rope:check_link(i, i + 1), "node " .. i .. " is linked to node " .. i + 1)
    end
    T.ok(not rope:check_link(0, 2))

    -- (the bounds are padded by the collision margin, 0.25 by default)
    local min, max = bounds(rope)
    T.near(min:x(), 0, 0.4)
    T.near(max:x(), 10, 0.4)

    -- fixing both ends
    local both = b.btSoftBodyHelpers.create_rope(info, vec(0, 0, 0), vec(10, 0, 0), 4, 3)
    T.near(both:get_mass(0), 0, 1e-6)
    T.near(both:get_mass(5), 0, 1e-6)
    T.near(both:get_mass(2), 1, 1e-6)
    T.ok(p ~= nil)
end)

T.test("btSoftBodyHelpers.create_patch: a grid of nodes with faces", function()
    local p, info = soft_world()
    local patch = b.btSoftBodyHelpers.create_patch(info,
        vec(0, 0, 0), vec(4, 0, 0), vec(0, 0, 4), vec(4, 0, 4), 5, 5, 0, true)
    T.ok(patch ~= nil)
    T.near(patch:get_total_mass(), 25, 1e-3, "5 x 5 nodes of mass 1")
    T.ok(patch:check_link(0, 1))
    T.ok(patch:check_link(0, 5))
    T.ok(patch:check_link(0, 6) or patch:check_link(1, 5), "gendiags adds diagonals")
    -- the first grid cell is made of two triangles over the nodes 0, 1, 5 and 6
    local cell = { 0, 1, 5, 6 }
    local has_face = false
    for _, a in ipairs(cell) do
        for _, c in ipairs(cell) do
            for _, d in ipairs(cell) do
                if a ~= c and c ~= d and a ~= d and patch:check_face(a, c, d) then has_face = true end
            end
        end
    end
    T.ok(has_face, "faces cover the first cell")
    T.ok(not patch:check_face(0, 12, 24), "no face spans the far corners")
    local min, max = bounds(patch)
    T.near(min:x(), 0, 0.4)
    T.near(max:x(), 4, 0.4)
    T.near(max:z(), 4, 0.4)

    -- fixing corners leaves massless nodes
    local pinned = b.btSoftBodyHelpers.create_patch(info,
        vec(0, 0, 0), vec(4, 0, 0), vec(0, 0, 4), vec(4, 0, 4), 5, 5, 1 + 2 + 4 + 8, false)
    T.near(pinned:get_mass(0), 0, 1e-6)
    T.near(pinned:get_total_mass(), 21, 1e-3)
    T.type_is(b.btSoftBodyHelpers.calculate_uv(5, 5, 1, 1, 0), "number")
    T.ok(p ~= nil)
end)

T.test("btSoftBodyHelpers.create_ellipsoid: a closed surface", function()
    local p, info = soft_world()
    local ball = b.btSoftBodyHelpers.create_ellipsoid(info, vec(0, 5, 0), vec(1, 1, 1), 128)
    T.ok(ball ~= nil)
    T.vec3(ball:get_center_of_mass(), 0, 5, 0, 0.1)
    -- the polyhedron inscribed in a unit sphere has a volume close to 4/3 pi
    T.near(ball:get_volume(), 4 / 3 * math.pi, 0.8)
    T.ok(ball:get_volume() > 2)

    local min, max = bounds(ball)
    T.near(max:y() - min:y(), 2, 0.6, "diameter plus the margin")

    local stretched = b.btSoftBodyHelpers.create_ellipsoid(info, vec(0, 0, 0), vec(2, 1, 1), 128)
    local smin, smax = bounds(stretched)
    T.ok(smax:x() - smin:x() > smax:y() - smin:y() + 1, "wider along x")
    T.ok(p ~= nil)
end)

T.test("btSoftBodyHelpers: barycentric weights", function()
    local weights = b.btVector4(0, 0, 0, 0)
    -- the centroid of a triangle weights its three corners equally
    b.btSoftBodyHelpers.get_barycentric_weights(vec(0, 0, 0), vec(3, 0, 0), vec(0, 3, 0), vec(1, 1, 0), weights)
    T.near(weights:x(), 1 / 3, 1e-4)
    T.near(weights:y(), 1 / 3, 1e-4)
    T.near(weights:z(), 1 / 3, 1e-4)

    -- a corner of a tetrahedron weights only that corner
    b.btSoftBodyHelpers.get_barycentric_weights(
        vec(0, 0, 0), vec(1, 0, 0), vec(0, 1, 0), vec(0, 0, 1), vec(1, 0, 0), weights)
    T.near(weights:x(), 0, 1e-4)
    T.near(weights:y(), 1, 1e-4)
    T.near(weights:z(), 0, 1e-4)
    T.near(weights:w(), 0, 1e-4)
end)

-- ---------------------------------------------------------------------------------------
-- btSoftBody properties
-- ---------------------------------------------------------------------------------------

local function rope(info)
    return b.btSoftBodyHelpers.create_rope(info, vec(0, 5, 0), vec(10, 5, 0), 9, 1)
end

T.test("btSoftBody: masses", function()
    local p, info = soft_world()
    local body = rope(info)
    body:set_mass(3, 2.5)
    T.near(body:get_mass(3), 2.5, 1e-6)
    body:set_total_mass(22, false)
    T.near(body:get_total_mass(), 22, 1e-3)
    T.near(body:get_mass(0), 0, 1e-6, "a fixed node stays fixed")
    T.ok(p ~= nil)
end)

T.test("btSoftBody: moving and shaping the body", function()
    local p, info = soft_world()
    local body = rope(info)
    local min, max = bounds(body)
    local width = max:x() - min:x()

    body:translate(vec(10, 0, 0))
    local min2, max2 = bounds(body)
    T.near(min2:x(), min:x() + 10, 1e-3)
    T.near(max2:x() - min2:x(), width, 1e-3)

    body:scale(vec(2, 1, 1))
    local min3, max3 = bounds(body)
    T.near(max3:x() - min3:x(), 2 * width, 0.5)

    local before = body:get_center_of_mass()
    body:transform(b.btTransform(b.btQuaternion(0, 0, 0, 1), vec(0, 3, 0)))
    T.near(body:get_center_of_mass():y(), before:y() + 3, 0.1)
    T.ok(body:get_rigid_transform() ~= nil)
    body:rotate(b.btQuaternion(0, 0, 0, 1))
    body:transform_to(b.btTransform(b.btQuaternion(0, 0, 0, 1), vec(0, 0, 0)))
    T.ok(p ~= nil)
end)

T.test("btSoftBody: velocity and force setters", function()
    local p, info = soft_world()
    local body = rope(info)
    body:set_velocity(vec(0, 0, 2))
    T.near(body:get_linear_velocity():z(), 2, 0.01)
    body:add_velocity(vec(0, 0, 1))
    T.near(body:get_linear_velocity():z(), 3, 0.01)
    body:add_velocity(vec(0, 0, 1), 4)
    body:set_linear_velocity(vec(1, 0, 0))
    T.near(body:get_linear_velocity():x(), 1, 0.01)
    body:set_angular_velocity(vec(0, 1, 0))
    body:add_force(vec(0, 1, 0))
    body:add_force(vec(0, 1, 0), 3)
    body:add_aero_force_to_node(vec(1, 0, 0), 2)
    body:set_zero_velocity()
    T.near(body:get_linear_velocity():length(), 0, 1e-4)
    T.ok(p ~= nil)
end)

T.test("btSoftBody: wind and tuning knobs", function()
    local p, info = soft_world()
    local body = rope(info)
    body:set_wind_velocity(vec(3, 0, 0))
    T.vec3(body:get_wind_velocity(), 3, 0, 0)
    body:set_damping_coefficient(0.5)
    body:set_spring_stiffness(0.8)
    body:set_gravity_factor(0.5)
    body:set_max_stress(100)
    body:set_collision_quadrature(2)
    body:set_cache_barycenter(true)
    body:set_rest_length_scale(1.5)
    T.near(body:get_rest_length_scale(), 1.5)
    T.ok(not body:use_self_collision())
    body:set_self_collision(true)
    T.ok(body:use_self_collision())
    T.ok(p ~= nil)
end)

T.test("btSoftBody: building and cutting", function()
    local p, info = soft_world()
    local body = rope(info)
    local before = body:get_total_mass()

    body:append_node(vec(11, 5, 0), 2)
    T.near(body:get_total_mass(), before + 2, 1e-4)
    body:append_link(10, 11)
    T.ok(body:check_link(10, 11))

    -- a rope link cut in the middle is no longer connected
    T.ok(body:cut_link(4, 5, 0.5))
    T.ok(not body:check_link(4, 5))
    T.ok(p ~= nil)
end)

T.test("btSoftBody: generating constraints and clusters", function()
    local p, info = soft_world()
    local patch = b.btSoftBodyHelpers.create_patch(info,
        vec(0, 0, 0), vec(4, 0, 0), vec(0, 0, 4), vec(4, 0, 4), 5, 5, 0, true)
    T.ok(patch:generate_bending_constraints(2) > 0, "bending links were added")
    T.eq(patch:cluster_count(), 0)
    T.ok(patch:generate_clusters(4) >= 1)
    T.ok(patch:cluster_count() >= 1)
    patch:release_clusters()
    T.eq(patch:cluster_count(), 0)
    patch:randomize_constraints()
    patch:reset_link_rest_lengths()
    patch:set_pose(false, true)
    T.ok(p ~= nil)
end)

T.test("btSoftBody: upcast from a collision object", function()
    local p, info = soft_world()
    local body = rope(info)
    T.ok(b.btSoftBody.upcast(body) ~= nil)
    T.is_nil(b.btSoftBody.upcast(b.btCollisionObject()))
    T.ok(body:get_world_info() ~= nil)
    T.ok(p ~= nil)
end)

T.test("btSoftBody: anchoring a node to a rigid body", function()
    local p, info = soft_world()
    local anchor = p:add_body({ shape = b.btSphereShape(0.5), mass = 0, position = { 0, 5, 0 } })
    local body = b.btSoftBodyHelpers.create_rope(info, vec(0, 5, 0), vec(10, 5, 0), 9, 0)
    body:append_anchor(0, anchor)
    p:add_soft_body(body)
    p:step(60)
    -- the anchored end stays with the body it is anchored to while the rest sags
    local min, max = bounds(body)
    T.ok(min:y() < 4.5, "the free end of the rope sags, lowest y = " .. min:y())
    body:remove_anchor(0)
end)

-- ---------------------------------------------------------------------------------------
-- Simulation
-- ---------------------------------------------------------------------------------------

T.test("btSoftRigidDynamicsWorld: a rope pinned at one end swings down", function()
    local p, info = soft_world()
    local body = rope(info)
    p:add_soft_body(body)
    local min0 = bounds(body)
    p:step(120)
    local min, max = bounds(body)
    T.ok(min:y() < min0:y() - 3, "the free end fell, lowest y = " .. min:y())
    T.near(math.min(max:y(), 5), 5, 0.3, "the pinned end stays at its height")
end)

T.test("btSoftRigidDynamicsWorld: a soft ball comes to rest on a rigid floor", function()
    local p, info = soft_world()
    p:add_floor(0)
    local ball = b.btSoftBodyHelpers.create_ellipsoid(info, vec(0, 1.5, 0), vec(1, 1, 1), 64)
    ball:set_total_mass(10, false)
    p:add_soft_body(ball)
    p:step(240)
    local min, max = bounds(ball)
    T.ok(min:y() > -0.2, "does not sink through the floor, lowest y = " .. min:y())
    T.ok(min:y() < 0.3, "rests on the floor, lowest y = " .. min:y())
    T.ok(max:y() - min:y() > 0.8, "keeps most of its height")
end)

T.test("btSoftRigidDynamicsWorld: world bookkeeping", function()
    local p, info = soft_world()
    local body = rope(info)
    p:add_soft_body(body)
    T.eq(p.world:get_num_collision_objects(), 1)
    p.world:remove_soft_body(body)
    T.eq(p.world:get_num_collision_objects(), 0)
    p.world:add_soft_body(body) -- the fixture removes it again
    p.world:set_draw_flags(3)
    T.eq(p.world:get_draw_flags(), 3)
    T.ok(info ~= nil)
end)

T.test("btSoftBodySolvers and world info", function()
    T.ok(b.btSoftBodyWorldInfo() ~= nil)
    local solver = b.btDefaultSoftBodySolver()
    T.ok(solver:check_initialized())
    local config = b.btSoftBodyRigidBodyCollisionConfiguration()
    local dispatcher = b.btCollisionDispatcher(config)
    local broadphase = b.btDbvtBroadphase()
    local constraint_solver = b.btSequentialImpulseConstraintSolver()
    local world = b.btSoftRigidDynamicsWorld(dispatcher, broadphase, constraint_solver, config, solver)
    T.ok(world:get_world_info() ~= nil)
end)

T.run()
