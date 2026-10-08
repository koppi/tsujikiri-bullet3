-- Deformable and reduced-deformable bodies: the parts of the API that do not need a
-- btAlignedObjectArray (which is not bound): material stress, force bookkeeping and solver switches.
--
-- btDeformableMultiBodyDynamicsWorld cannot be built from Lua because its
-- btDeformableMultiBodyConstraintSolver has no bound constructor, so there is no deformable
-- simulation here; see test_soft_body.lua for soft bodies in a btSoftRigidDynamicsWorld.

local T = require("testlib")
local P = require("physics")
local b = bullet3

local function vec(x, y, z)
    return b.btVector3(x, y, z)
end

local function diagonal(x, y, z)
    return b.btMatrix3x3(x, 0, 0, 0, y, 0, 0, 0, z)
end

-- ---------------------------------------------------------------------------------------
-- Material models
-- ---------------------------------------------------------------------------------------

T.test("btDeformableCorotatedForce: the first Piola-Kirchhoff stress", function()
    local force = b.btDeformableCorotatedForce(1, 1) -- mu = 1, lambda = 1
    local stress = b.btMatrix3x3(9, 9, 9, 9, 9, 9, 9, 9, 9)

    -- at rest (F = I) there is no stress
    force:first_piola(diagonal(1, 1, 1), stress)
    for row = 0, 2 do T.vec3(stress:get_row(row), 0, 0, 0, 1e-4) end

    -- stretching along x: P = lambda (J - 1) cof(F) + 2 mu (F - R) = diag(1, 2, 2) + diag(2, 0, 0)
    force:first_piola(diagonal(2, 1, 1), stress)
    T.vec3(stress:get_row(0), 3, 0, 0, 1e-3)
    T.vec3(stress:get_row(1), 0, 2, 0, 1e-3)
    T.vec3(stress:get_row(2), 0, 0, 2, 1e-3)

    -- a pure rotation is also stress free
    local rotation = b.btMatrix3x3(b.btQuaternion(b.btVector3(0, 0, 1), math.pi / 3))
    force:first_piola(rotation, stress)
    for row = 0, 2 do T.vec3(stress:get_row(row), 0, 0, 0, 1e-3) end

    T.eq(b.btDeformableCorotatedForce():first_piola(diagonal(1, 1, 1), stress), nil)
end)

T.test("btDeformableNeoHookeanForce: material parameters and the double contraction", function()
    local force = b.btDeformableNeoHookeanForce(1, 2, 0.01)
    force:set_youngs_modulus(100)
    force:set_poisson_ratio(0.3)
    force:update_lame_parameters()
    force:set_lame_parameters(5, 7)
    force:update_youngs_modulus_and_poisson_ratio()
    force:set_damping(0.1)

    -- A : B, the sum of the products of matching entries
    local a = b.btMatrix3x3(1, 2, 3, 4, 5, 6, 7, 8, 9)
    T.near(force:dot_product(a, diagonal(1, 1, 1)), 1 + 5 + 9)
    T.near(force:dot_product(a, a), 285)
    T.near(force:dot_product(a, diagonal(0, 0, 0)), 0)

    T.ok(b.btDeformableNeoHookeanForce() ~= nil)
    T.ok(b.btDeformableNeoHookeanForce(1, 2) ~= nil)
end)

T.test("btDeformableLinearElasticityForce: parameters", function()
    local force = b.btDeformableLinearElasticityForce(1, 2, 0.01, 0.02)
    force:set_youngs_modulus(50)
    force:set_poisson_ratio(0.25)
    force:set_lame_parameters(3, 4)
    force:set_damping(0.1, 0.2)
    force:update_lame_parameters()
    force:update_youngs_modulus_and_poisson_ratio()
    T.ok(b.btDeformableLinearElasticityForce() ~= nil)
    T.ok(b.btDeformableLinearElasticityForce(1, 2) ~= nil)
    T.ok(b.btDeformableLinearElasticityForce(1, 2, 0.1) ~= nil)
end)

T.test("btDeformableMassSpringForce and btDeformableGravityForce: construction", function()
    T.ok(b.btDeformableMassSpringForce() ~= nil)
    T.ok(b.btDeformableMassSpringForce(1, 0.1) ~= nil)
    T.ok(b.btDeformableMassSpringForce(1, 0.1, true) ~= nil)
    T.ok(b.btDeformableMassSpringForce(1, 0.1, true, 0.5) ~= nil)
    T.ok(b.btDeformableGravityForce(vec(0, -10, 0)) ~= nil)
end)

-- ---------------------------------------------------------------------------------------
-- Force bookkeeping
-- ---------------------------------------------------------------------------------------

T.test("btDeformableLagrangianForce: tracks the nodes of its soft bodies", function()
    local p = P.new(T, { soft = true })
    local info = p.world:get_world_info()
    local rope = b.btSoftBodyHelpers.create_rope(info, vec(0, 0, 0), vec(10, 0, 0), 9, 0)
    local patch = b.btSoftBodyHelpers.create_patch(info,
        vec(0, 0, 0), vec(4, 0, 0), vec(0, 0, 4), vec(4, 0, 4), 3, 3, 0, false)

    local force = b.btDeformableGravityForce(vec(0, -10, 0))
    T.eq(force:get_num_nodes(), 0)
    force:add_soft_body(rope)
    T.eq(force:get_num_nodes(), 11, "9 interior nodes and two ends")
    force:add_soft_body(patch)
    T.eq(force:get_num_nodes(), 11 + 9)
    force:remove_soft_body(rope)
    T.eq(force:get_num_nodes(), 9)
    force:remove_soft_body(patch)
    T.eq(force:get_num_nodes(), 0)
end)

-- ---------------------------------------------------------------------------------------
-- Solvers
-- ---------------------------------------------------------------------------------------

T.test("btDeformableBodySolver: switches", function()
    local solver = b.btDeformableBodySolver()
    T.ok(not solver:is_reduced_solver())
    solver:set_gravity(vec(0, -10, 0))
    solver:set_implicit(true)
    solver:set_line_search(true)
    solver:set_strain_limiting(true)
    solver:set_preconditioner(0)
    T.near(solver:kinetic_energy(), 0, 1e-6)
    T.ok(solver:check_initialized())
    -- the force array is a btAlignedObjectArray, which is not bound
    T.throws(function() return solver:get_lagrangian_force_array() end, "not registered")
end)

T.test("btReducedDeformableBodySolver: reports itself as reduced", function()
    local solver = b.btReducedDeformableBodySolver()
    T.ok(solver:is_reduced_solver())
    solver:set_gravity(vec(0, -10, 0))
end)

T.test("btReducedDeformableBodyHelpers: the inertia of a box", function()
    local inertia = vec(0, 0, 0)
    -- half extents (1, 2, 3) plus a margin of 0: full sizes (2, 4, 6), mass 12
    b.btReducedDeformableBodyHelpers.calculate_local_inertia(inertia, 12, vec(1, 2, 3), vec(0, 0, 0))
    T.vec3(inertia, 52, 40, 20, 1e-3)
end)

T.test("preconditioners and contact projection: construction", function()
    local p = P.new(T, { soft = true })
    T.ok(p.world:get_world_info() ~= nil)
    T.ok(b.btDeformableContactProjection ~= nil)
end)

T.run()
