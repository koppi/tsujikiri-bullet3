-- Narrow-phase collision detection: the simplex solver, convex casts, persistent manifolds
-- and manifold points.
--
-- Bullet hands results back through public data members (CastResult::m_fraction,
-- ClosestPointInput::m_transformA, ...) and the bindings do not expose data members, so
-- these tests check what is observable from Lua: return values, counts and distances.

local T = require("testlib")
local b = bullet3

local function vec(x, y, z)
    return b.btVector3(x, y, z)
end

local function at(x, y, z)
    return b.btTransform(b.btQuaternion(0, 0, 0, 1), vec(x, y, z))
end

-- ---------------------------------------------------------------------------------------
-- btVoronoiSimplexSolver: the closest point of a simplex to the origin
-- ---------------------------------------------------------------------------------------

-- Add a vertex w of the Minkowski difference (w = p - q)
local function add(solver, x, y, z)
    solver:add_vertex(vec(x, y, z), vec(x, y, z), vec(0, 0, 0))
end

T.test("btVoronoiSimplexSolver: an empty simplex", function()
    local solver = b.btVoronoiSimplexSolver()
    solver:reset()
    T.ok(solver:empty_simplex())
    T.eq(solver:num_vertices(), 0)
    T.ok(not solver:full_simplex())
end)

T.test("btVoronoiSimplexSolver: a single vertex is its own closest point", function()
    local solver = b.btVoronoiSimplexSolver()
    solver:reset()
    add(solver, 1, 2, 3)
    T.eq(solver:num_vertices(), 1)
    T.ok(not solver:empty_simplex())
    local closest = vec(0, 0, 0)
    T.ok(solver:closest(closest))
    T.vec3(closest, 1, 2, 3)
    T.near(solver:max_vertex(), 14, 1e-4)
    T.ok(solver:in_simplex(vec(1, 2, 3)))
    T.ok(not solver:in_simplex(vec(9, 9, 9)))
end)

T.test("btVoronoiSimplexSolver: the closest point on a segment", function()
    local solver = b.btVoronoiSimplexSolver()
    solver:reset()
    add(solver, 1, 0, 0)
    add(solver, 1, 2, 0)
    local closest = vec(0, 0, 0)
    solver:closest(closest)
    T.vec3(closest, 1, 0, 0, 1e-5) -- the foot of the perpendicular from the origin
    T.eq(solver:num_vertices(), 1, "the simplex shrinks to the vertex that carries the closest point")

    -- a segment whose perpendicular foot lies outside it: the nearer end point
    local other = b.btVoronoiSimplexSolver()
    other:reset()
    add(other, 2, 1, 0)
    add(other, 2, 3, 0)
    other:closest(closest)
    T.vec3(closest, 2, 1, 0, 1e-5)
end)

T.test("btVoronoiSimplexSolver: the closest point on a triangle", function()
    local solver = b.btVoronoiSimplexSolver()
    solver:reset()
    add(solver, 1, -1, 1)
    add(solver, 1, 3, 1)
    add(solver, 1, -1, -3)
    local closest = vec(0, 0, 0)
    solver:closest(closest)
    T.vec3(closest, 1, 0, 0, 1e-4) -- the plane x = 1 is nearest, and the origin projects inside
    T.eq(solver:num_vertices(), 3)
end)

T.test("btVoronoiSimplexSolver: a tetrahedron around the origin", function()
    local solver = b.btVoronoiSimplexSolver()
    solver:reset()
    add(solver, 1, 1, 1)
    add(solver, -1, 1, -1)
    add(solver, -1, -1, 1)
    add(solver, 1, -1, -1)
    T.ok(solver:full_simplex())
    local closest = vec(9, 9, 9)
    solver:closest(closest)
    T.vec3(closest, 0, 0, 0, 1e-4) -- the origin is inside
end)

T.test("btVoronoiSimplexSolver: points on the original shapes", function()
    local solver = b.btVoronoiSimplexSolver()
    solver:reset()
    -- w = p - q with p on shape A and q on shape B
    solver:add_vertex(vec(2, 0, 0), vec(3, 0, 0), vec(1, 0, 0))
    local pa, pb = vec(0, 0, 0), vec(0, 0, 0)
    local closest = vec(0, 0, 0)
    solver:closest(closest)
    solver:compute_points(pa, pb)
    T.vec3(pa, 3, 0, 0)
    T.vec3(pb, 1, 0, 0)
end)

T.test("btVoronoiSimplexSolver: vertex removal and the equal-vertex threshold", function()
    local solver = b.btVoronoiSimplexSolver()
    solver:reset()
    add(solver, 1, 0, 0)
    add(solver, 0, 1, 0)
    T.eq(solver:num_vertices(), 2)
    solver:remove_vertex(0)
    T.eq(solver:num_vertices(), 1)
    solver:reset()
    T.eq(solver:num_vertices(), 0)

    solver:set_equal_vertex_threshold(0.25)
    T.near(solver:get_equal_vertex_threshold(), 0.25)
end)

T.test("btVoronoiSimplexSolver: point_outside_of_plane", function()
    local solver = b.btVoronoiSimplexSolver()
    local a, c, d = vec(0, 0, 0), vec(1, 0, 0), vec(0, 1, 0)
    local reference = vec(0, 0, -1) -- the fourth vertex defines the inside
    T.eq(solver:point_outside_of_plane(vec(0.2, 0.2, 1), a, c, d, reference), 1)
    T.eq(solver:point_outside_of_plane(vec(0.2, 0.2, -1), a, c, d, reference), 0)
end)

T.test("btUsageBitfield: reset", function()
    local bits = b.btUsageBitfield()
    bits:reset()
end)

-- ---------------------------------------------------------------------------------------
-- Convex casts: does the moving shape touch the other one?
-- ---------------------------------------------------------------------------------------

local function casters()
    local simplex = b.btVoronoiSimplexSolver()
    local depth = b.btGjkEpaPenetrationDepthSolver()
    return {
        {
            "btGjkConvexCast",
            function(a, c) return b.btGjkConvexCast(a, c, simplex) end,
        },
        {
            "btSubsimplexConvexCast",
            function(a, c) return b.btSubsimplexConvexCast(a, c, simplex) end,
        },
        {
            "btContinuousConvexCollision",
            function(a, c) return b.btContinuousConvexCollision(a, c, simplex, depth) end,
        },
    }, simplex, depth
end

T.test("convex casts: a sphere sweeping into another sphere", function()
    local moving, target = b.btSphereShape(1), b.btSphereShape(1)
    local list, simplex, depth = casters()
    for _, case in ipairs(list) do
        local cast = case[2](moving, target)
        local result = b.CastResult()
        -- from x = -5 to x = 5, straight through the target at the origin
        T.ok(cast:calc_time_of_impact(at(-5, 0, 0), at(5, 0, 0), at(0, 0, 0), at(0, 0, 0), result),
            case[1] .. " reports the impact")
        -- a lane 5 units away never touches
        T.ok(not cast:calc_time_of_impact(at(-5, 5, 0), at(5, 5, 0), at(0, 0, 0), at(0, 0, 0), b.CastResult()),
            case[1] .. " reports a miss")
        -- stops before arriving
        T.ok(not cast:calc_time_of_impact(at(-5, 0, 0), at(-3.5, 0, 0), at(0, 0, 0), at(0, 0, 0), b.CastResult()),
            case[1] .. " stops short")
    end
    T.ok(simplex ~= nil and depth ~= nil)
end)

T.test("convex casts: a box sweeping into a sphere", function()
    local moving, target = b.btBoxShape(vec(0.5, 0.5, 0.5)), b.btSphereShape(1)
    local simplex = b.btVoronoiSimplexSolver()
    local cast = b.btGjkConvexCast(moving, target, simplex)
    T.ok(cast:calc_time_of_impact(at(0, 5, 0), at(0, -5, 0), at(0, 0, 0), at(0, 0, 0), b.CastResult()))
    T.ok(not cast:calc_time_of_impact(at(4, 5, 0), at(4, -5, 0), at(0, 0, 0), at(0, 0, 0), b.CastResult()))
end)

T.test("convex casts: the target moves too", function()
    local a, c = b.btSphereShape(1), b.btSphereShape(1)
    local simplex = b.btVoronoiSimplexSolver()
    local cast = b.btGjkConvexCast(a, c, simplex)
    -- A stays put; B rushes through it
    T.ok(cast:calc_time_of_impact(at(0, 0, 0), at(0, 0, 0), at(10, 0, 0), at(-10, 0, 0), b.CastResult()))
end)

T.test("btContinuousConvexCollision: a convex shape against a static plane", function()
    local ball = b.btSphereShape(1)
    local plane = b.btStaticPlaneShape(vec(0, 1, 0), 0)
    local cast = b.btContinuousConvexCollision(ball, plane)
    T.ok(cast:calc_time_of_impact(at(0, 5, 0), at(0, -5, 0), at(0, 0, 0), at(0, 0, 0), b.CastResult()))
    T.ok(not cast:calc_time_of_impact(at(0, 5, 0), at(0, 3, 0), at(0, 0, 0), at(0, 0, 0), b.CastResult()))
end)

T.test("CastResult: the diagnostic hooks can be called", function()
    local result = b.CastResult()
    result:debug_draw(0.5)
    result:draw_coord_system(at(0, 0, 0))
    result:report_failure(1, 2)
end)

-- ---------------------------------------------------------------------------------------
-- Penetration depth solvers and the pair detector
-- ---------------------------------------------------------------------------------------

T.test("btGjkPairDetector: construction with the two shapes", function()
    local a, c = b.btSphereShape(1), b.btBoxShape(vec(1, 1, 1))
    local simplex = b.btVoronoiSimplexSolver()
    local depth = b.btGjkEpaPenetrationDepthSolver()
    local detector = b.btGjkPairDetector(a, c, simplex, depth)

    detector:set_cached_separating_axis(vec(0, 1, 0))
    T.vec3(detector:get_cached_separating_axis(), 0, 1, 0)
    -- (the cached separating distance is not initialised until the detector has run, so it is not read)
    detector:set_minkowski_a(c)
    detector:set_minkowski_b(a)
    detector:set_ignore_margin(true)
    detector:set_penetration_depth_solver(depth)

    local with_ids = b.btGjkPairDetector(a, c, 0, 0, 0.04, 0.04, simplex, depth)
    T.ok(with_ids ~= nil)
end)

T.test("penetration depth solvers: construction", function()
    T.ok(b.btGjkEpaPenetrationDepthSolver() ~= nil)
    T.ok(b.btMinkowskiPenetrationDepthSolver ~= nil)
    T.ok(b.btPointCollector() ~= nil)
    T.ok(b.ClosestPointInput() ~= nil)
end)

T.test("btPointCollector: the shape identifiers can be set", function()
    local collector = b.btPointCollector()
    collector:set_shape_identifiers_a(1, 2)
    collector:set_shape_identifiers_b(3, 4)
    collector:add_contact_point(vec(0, 1, 0), vec(0, 0, 0), -0.1)
end)

T.test("btGjkEpaSolver2: stack size requirement", function()
    T.ok(b.btGjkEpaSolver2.stack_size_requirement() > 0)
end)

-- ---------------------------------------------------------------------------------------
-- btManifoldPoint and btPersistentManifold
-- ---------------------------------------------------------------------------------------

T.test("btManifoldPoint: constructed from local points, a normal and a distance", function()
    local point = b.btManifoldPoint(vec(0, 0, 0), vec(0, 0, 0), vec(0, 1, 0), -0.05)
    T.near(point:get_distance(), -0.05)
    point:set_distance(0.25)
    T.near(point:get_distance(), 0.25)
    T.eq(point:get_life_time(), 0)
    T.near(point:get_applied_impulse(), 0)
    T.ok(b.btManifoldPoint() ~= nil)
end)

local function manifold()
    local body_a, body_b = b.btCollisionObject(), b.btCollisionObject()
    return b.btPersistentManifold(body_a, body_b, 0, 0.02, 0.0), body_a, body_b
end

T.test("btPersistentManifold: contact points are cached up to four", function()
    local m = manifold()
    T.eq(m:get_num_contacts(), 0)
    for i = 0, 3 do
        local index = m:add_manifold_point(b.btManifoldPoint(vec(i, 0, 0), vec(i, 0, 0), vec(0, 1, 0), -0.01))
        T.eq(index, i)
    end
    T.eq(m:get_num_contacts(), 4)
    -- a fifth point replaces one of the four instead of growing the cache
    m:add_manifold_point(b.btManifoldPoint(vec(9, 0, 0), vec(9, 0, 0), vec(0, 1, 0), -0.01))
    T.eq(m:get_num_contacts(), 4)

    m:remove_contact_point(0)
    T.eq(m:get_num_contacts(), 3)
    m:clear_manifold()
    T.eq(m:get_num_contacts(), 0)
end)

T.test("btPersistentManifold: thresholds and validity", function()
    local m = manifold()
    T.near(m:get_contact_breaking_threshold(), 0.02, 1e-6)
    T.near(m:get_contact_processing_threshold(), 0, 1e-6)
    m:set_contact_breaking_threshold(0.5)
    m:set_contact_processing_threshold(0.25)
    T.near(m:get_contact_breaking_threshold(), 0.5, 1e-6)
    T.near(m:get_contact_processing_threshold(), 0.25, 1e-6)

    T.ok(m:valid_contact_distance(b.btManifoldPoint(vec(0, 0, 0), vec(0, 0, 0), vec(0, 1, 0), 0.4)))
    T.ok(not m:valid_contact_distance(b.btManifoldPoint(vec(0, 0, 0), vec(0, 0, 0), vec(0, 1, 0), 0.6)))
end)

T.test("btPersistentManifold: the cache entry of a point nearby", function()
    local m = manifold()
    T.eq(m:get_cache_entry(b.btManifoldPoint(vec(0, 0, 0), vec(0, 0, 0), vec(0, 1, 0), 0)), -1)
    m:add_manifold_point(b.btManifoldPoint(vec(0, 0, 0), vec(0, 0, 0), vec(0, 1, 0), 0))
    T.eq(m:get_cache_entry(b.btManifoldPoint(vec(0, 0, 0), vec(0, 0, 0), vec(0, 1, 0), 0)), 0)
end)

T.test("btPersistentManifold: bodies", function()
    local m, a, c = manifold()
    a:set_user_index(1)
    c:set_user_index(2)
    T.eq(m:get_body0():get_user_index(), 1)
    T.eq(m:get_body1():get_user_index(), 2)
    m:set_bodies(c, a)
    T.eq(m:get_body0():get_user_index(), 2)
    T.eq(m:get_body1():get_user_index(), 1)
end)

T.test("btPersistentManifold: refresh_contact_points drops points that moved apart", function()
    local m = manifold()
    m:add_manifold_point(b.btManifoldPoint(vec(0, 0, 0), vec(0, 0, 0), vec(0, 1, 0), 0))
    T.eq(m:get_num_contacts(), 1)

    -- body B moves up: the surfaces overlap, the point stays
    m:refresh_contact_points(at(0, 0, 0), at(0, 1, 0))
    T.eq(m:get_num_contacts(), 1)
    T.eq(m:get_contact_point(0):get_life_time(), 1)
    -- the distance along the normal is the offset of A relative to B: (0, 0, 0) - (0, 1, 0)
    T.near(m:get_contact_point(0):get_distance(), -1, 1e-6)

    -- body B drops away: the gap exceeds the breaking threshold and the point is removed
    m:refresh_contact_points(at(0, 0, 0), at(0, -1, 0))
    T.eq(m:get_num_contacts(), 0)
end)

T.test("btPersistentManifold: replacing and clearing points", function()
    local m = manifold()
    m:add_manifold_point(b.btManifoldPoint(vec(1, 0, 0), vec(1, 0, 0), vec(0, 1, 0), -0.01))
    m:replace_contact_point(b.btManifoldPoint(vec(2, 0, 0), vec(2, 0, 0), vec(0, 1, 0), -0.03), 0)
    T.eq(m:get_num_contacts(), 1)
    T.near(m:get_contact_point(0):get_distance(), -0.03, 1e-6)
    m:set_num_contacts(0)
    T.eq(m:get_num_contacts(), 0)
    T.ok(b.btPersistentManifold() ~= nil)
end)

T.run()
