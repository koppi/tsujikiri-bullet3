-- Featherstone multibodies: btMultiBody, btMultiBodyDynamicsWorld and the multibody constraints.
--
-- btMultiBodyConstraintSolver has no bound constructor, so the world is built with its
-- derived btMultiBodyMLCPConstraintSolver (on a Dantzig MLCP solver) instead.

local T = require("testlib")
local b = bullet3

local DT = 1 / 60

local function vec(x, y, z)
    return b.btVector3(x, y, z)
end

local IDENTITY = function()
    return b.btQuaternion(0, 0, 0, 1)
end

-- A multibody world; multibodies and constraints are removed again when the test ends.
--
-- Bullet solves a multibody constraint only if the multibody has a collider in the world: the
-- collider carries the island id that selects the constraints to solve, and static colliders
-- are given none. Arms therefore get a tiny, non-static base collider here (the base itself
-- stays fixed, which is a property of the multibody).
local DEFAULT_FILTER, ALL_FILTER = 1, -1

local function new_world()
    local w = { multibodies = {}, constraints = {}, colliders = {}, keep = {} }
    w.config = b.btDefaultCollisionConfiguration()
    w.dispatcher = b.btCollisionDispatcher(w.config)
    w.broadphase = b.btDbvtBroadphase()
    w.mlcp = b.btDantzigSolver()
    w.solver = b.btMultiBodyMLCPConstraintSolver(w.mlcp)
    w.world = b.btMultiBodyDynamicsWorld(w.dispatcher, w.broadphase, w.solver, w.config)
    w.world:set_gravity(vec(0, -10, 0))

    function w:add(multibody)
        local collider = b.btMultiBodyLinkCollider(multibody, -1)
        collider:set_collision_shape(self:hold(b.btSphereShape(0.1)))
        collider:set_world_transform(b.btTransform(IDENTITY(), multibody:get_base_pos()))
        multibody:set_base_collider(collider)
        self.world:add_collision_object(collider, DEFAULT_FILTER, ALL_FILTER)
        table.insert(self.colliders, collider)

        self.world:add_multi_body(multibody)
        table.insert(self.multibodies, multibody)
        return multibody
    end

    function w:hold(object)
        table.insert(self.keep, object)
        return object
    end

    function w:add_constraint(constraint)
        constraint:finalize_multi_dof() -- the constraint sizes its Jacobians here, before it is used
        self.world:add_multi_body_constraint(constraint)
        table.insert(self.constraints, constraint)
        return constraint
    end

    function w:step(count)
        for _ = 1, count or 1 do self.world:step_simulation(DT, 1, DT) end
    end

    T.cleanup(function()
        for i = #w.constraints, 1, -1 do w.world:remove_multi_body_constraint(w.constraints[i]) end
        for i = #w.multibodies, 1, -1 do w.world:remove_multi_body(w.multibodies[i]) end
        for i = #w.colliders, 1, -1 do w.world:remove_collision_object(w.colliders[i]) end
    end)
    return w
end

-- A fixed base at height 5 with `links` revolute joints about z, each link a 1 m bar whose
-- centre of mass sits half a metre beyond its pivot, lying along +x
local function arm(links)
    local mb = b.btMultiBody(links, 0, vec(0, 0, 0), true, false)
    mb:set_base_pos(vec(0, 5, 0))
    for i = 0, links - 1 do
        local pivot = (i == 0) and vec(0, 0, 0) or vec(1, 0, 0) -- from the parent's centre of mass
        mb:setup_revolute(i, 1.0, vec(0.1, 0.1, 0.1), i - 1, IDENTITY(), vec(0, 0, 1), pivot, vec(0.5, 0, 0), true)
    end
    mb:finalize_multi_dof()
    return mb
end

-- ---------------------------------------------------------------------------------------
-- btMultiBody structure and state
-- ---------------------------------------------------------------------------------------

T.test("btMultiBody: structure", function()
    local mb = arm(2)
    T.eq(mb:get_num_links(), 2)
    T.eq(mb:get_num_dofs(), 2)
    T.eq(mb:get_num_pos_vars(), 2)
    T.eq(mb:get_parent(0), -1)
    T.eq(mb:get_parent(1), 0)
    T.near(mb:get_base_mass(), 0)
    T.near(mb:get_link_mass(0), 1)
    T.near(mb:get_link_mass(1), 1)
    T.vec3(mb:get_link_inertia(0), 0.1, 0.1, 0.1)
    T.ok(mb:has_fixed_base())
    T.vec3(mb:get_base_pos(), 0, 5, 0)
    T.vec3(mb:get_r_vector(0), 0.5, 0, 0, 1e-5)
end)

T.test("btMultiBody: other joint kinds", function()
    local mb = b.btMultiBody(4, 1, vec(1, 1, 1), false, true)
    mb:setup_fixed(0, 1, vec(1, 1, 1), -1, IDENTITY(), vec(0, 0, 0), vec(0, 0, 0))
    mb:setup_prismatic(1, 1, vec(1, 1, 1), 0, IDENTITY(), vec(1, 0, 0), vec(0, 0, 0), vec(0, 0, 0), true)
    mb:setup_spherical(2, 1, vec(1, 1, 1), 1, IDENTITY(), vec(0, 0, 0), vec(0, 0, 0))
    mb:setup_planar(3, 1, vec(1, 1, 1), 2, IDENTITY(), vec(0, 0, 1), vec(0, 0, 0))
    mb:finalize_multi_dof()
    T.eq(mb:get_num_links(), 4)
    -- fixed 0 dof, prismatic 1, spherical 3, planar 3
    T.eq(mb:get_num_dofs(), 0 + 1 + 3 + 3)
    T.ok(not mb:has_fixed_base())
    T.near(mb:get_base_mass(), 1)
end)

T.test("btMultiBody: joint state", function()
    local mb = arm(2)
    mb:set_joint_pos(0, 0.5)
    mb:set_joint_vel(1, -2)
    T.near(mb:get_joint_pos(0), 0.5, 1e-6)
    T.near(mb:get_joint_vel(1), -2, 1e-6)
    T.near(mb:get_joint_pos(1), 0, 1e-6)
    mb:clear_velocities()
    T.near(mb:get_joint_vel(1), 0, 1e-6)
end)

T.test("btMultiBody: base state", function()
    local mb = arm(1)
    mb:set_base_pos(vec(1, 2, 3))
    T.vec3(mb:get_base_pos(), 1, 2, 3)
    mb:set_base_vel(vec(0, 0, 0))
    mb:set_base_omega(vec(0, 1, 0))
    T.vec3(mb:get_base_omega(), 0, 1, 0)
    mb:set_world_to_base_rot(IDENTITY())
    T.quat(mb:get_world_to_base_rot(), 0, 0, 0, 1)
    mb:set_base_mass(2)
    T.near(mb:get_base_mass(), 2)
    mb:set_base_inertia(vec(1, 2, 3))
    T.vec3(mb:get_base_inertia(), 1, 2, 3)

    mb:set_base_world_transform(b.btTransform(b.btQuaternion(0, 0, 0, 1), vec(4, 5, 6)))
    T.vec3(mb:get_base_world_transform():get_origin(), 4, 5, 6)
    mb:set_interpolate_base_pos(vec(7, 8, 9))
    T.vec3(mb:get_interpolate_base_pos(), 7, 8, 9)
end)

T.test("btMultiBody: forces and torques accumulate until cleared", function()
    local mb = arm(2)
    mb:add_base_force(vec(1, 2, 3))
    mb:add_base_torque(vec(0, 1, 0))
    mb:add_link_force(0, vec(0, 4, 0))
    mb:add_link_torque(1, vec(0, 0, 5))
    mb:add_joint_torque(0, 0.5)
    T.vec3(mb:get_base_force(), 1, 2, 3)
    T.vec3(mb:get_base_torque(), 0, 1, 0)
    T.vec3(mb:get_link_force(0), 0, 4, 0)
    T.vec3(mb:get_link_torque(1), 0, 0, 5)
    T.near(mb:get_joint_torque(0), 0.5)

    mb:clear_forces_and_torques()
    T.vec3(mb:get_base_force(), 0, 0, 0)
    T.vec3(mb:get_link_force(0), 0, 0, 0)
    T.near(mb:get_joint_torque(0), 0)

    mb:add_base_constraint_force(vec(1, 0, 0))
    mb:add_base_constraint_torque(vec(1, 0, 0))
    mb:add_link_constraint_force(0, vec(1, 0, 0))
    mb:add_link_constraint_torque(0, vec(1, 0, 0))
    mb:clear_constraint_forces()
end)

T.test("btMultiBody: coordinate conversions", function()
    local mb = arm(1)
    local world = new_world()
    world:add(mb)
    world.world:forward_kinematics()
    -- the link's centre of mass is half a metre along x from the base pivot
    T.vec3(mb:local_pos_to_world(0, vec(0, 0, 0)), 0.5, 5, 0, 1e-4)
    T.vec3(mb:local_dir_to_world(0, vec(1, 0, 0)), 1, 0, 0, 1e-4)
    T.vec3(mb:world_pos_to_local(0, vec(0.5, 5, 0)), 0, 0, 0, 1e-4)
    T.vec3(mb:world_dir_to_local(0, vec(0, 1, 0)), 0, 1, 0, 1e-4)

    -- a quarter turn of the joint swings the link up to +y
    mb:set_joint_pos(0, math.pi / 2)
    world.world:forward_kinematics()
    T.vec3(mb:local_pos_to_world(0, vec(0, 0, 0)), 0, 5.5, 0, 1e-3)
end)

T.test("btMultiBody: options", function()
    local mb = arm(1)
    mb:set_linear_damping(0.1)
    mb:set_angular_damping(0.2)
    T.near(mb:get_linear_damping(), 0.1)
    T.near(mb:get_angular_damping(), 0.2)
    mb:set_use_gyro_term(false)
    T.ok(not mb:get_use_gyro_term())
    mb:set_max_coordinate_velocity(50)
    T.near(mb:get_max_coordinate_velocity(), 50)
    mb:set_max_applied_impulse(30)
    T.near(mb:get_max_applied_impulse(), 30)
    mb:set_has_self_collision(true)
    T.ok(mb:has_self_collision())
    mb:use_rk4_integration(true)
    T.ok(mb:is_using_rk4_integration())
    mb:use_global_velocities(true)
    T.ok(mb:is_using_global_velocities())
    mb:set_companion_id(9)
    T.eq(mb:get_companion_id(), 9)
    mb:set_base_name("arm")
    T.eq(mb:get_base_name(), "arm")
    mb:set_user_index(3)
    mb:set_user_index2(4)
    T.eq(mb:get_user_index(), 3)
    T.eq(mb:get_user_index2(), 4)
    mb:set_pos_updated(true)
    T.ok(mb:is_pos_updated())
    mb:set_sleep_threshold(0.5)
    mb:set_sleep_timeout(1.5)
end)

T.test("btMultiBody: sleeping", function()
    local mb = arm(1)
    mb:set_can_sleep(true)
    T.ok(mb:get_can_sleep())
    T.ok(mb:is_awake())
    mb:go_to_sleep()
    T.ok(not mb:is_awake())
    mb:wake_up()
    T.ok(mb:is_awake())
    mb:set_can_wakeup(false)
    T.ok(not mb:get_can_wakeup())
end)

T.test("btMultiBody: kinematic and static links", function()
    local mb = arm(2)
    -- the dynamic type lives on the colliders, which the base is the only part to get one here
    T.ok(not mb:is_base_kinematic())
    local collider = b.btMultiBodyLinkCollider(mb, -1)
    mb:set_base_collider(collider)
    mb:set_base_dynamic_type(2) -- CF_KINEMATIC_OBJECT
    T.ok(mb:is_base_kinematic())
    T.ok(mb:is_base_static_or_kinematic())

    T.type_is(mb:is_link_kinematic(0), "boolean")
    T.type_is(mb:is_link_static_or_kinematic(1), "boolean")
    T.type_is(mb:is_link_and_all_ancestors_kinematic(1), "boolean")
    T.type_is(mb:is_link_and_all_ancestors_static_or_kinematic(1), "boolean")
    mb:set_link_dynamic_type(1, 2)
end)

T.test("btMultibodyLink: joint axes", function()
    local mb = arm(1)
    local link = mb:get_link(0)
    T.ok(link ~= nil)
    link:set_axis_top(0, vec(0, 0, 1))
    link:set_axis_bottom(0, 1, 2, 3)
    T.vec3(link:get_axis_top(0), 0, 0, 1)
    T.vec3(link:get_axis_bottom(0), 1, 2, 3)
    T.ok(b.btMultibodyLink() ~= nil)
end)

T.test("btMultiBodyLinkCollider: a collision object for a link", function()
    local mb = arm(1)
    local collider = b.btMultiBodyLinkCollider(mb, 0)
    T.ok(collider ~= nil)
    mb:set_base_collider(b.btMultiBodyLinkCollider(mb, -1))
    T.ok(mb:get_base_collider() ~= nil)
    T.ok(not collider:is_kinematic())
    collider:set_dynamic_type(2)
    T.ok(collider:is_kinematic())
    T.ok(collider:is_static_or_kinematic())
    T.ok(b.btMultiBodyLinkCollider.upcast(collider) ~= nil)
end)

-- ---------------------------------------------------------------------------------------
-- btMultiBodyDynamicsWorld
-- ---------------------------------------------------------------------------------------

T.test("btMultiBodyDynamicsWorld: bookkeeping", function()
    local w = new_world()
    T.eq(w.world:get_num_multibodies(), 0)
    local mb = w:add(arm(1))
    T.eq(w.world:get_num_multibodies(), 1)
    T.near(w.world:get_multi_body(0):get_link_mass(0), 1)
    T.eq(w.world:get_num_multi_body_constraints(), 0)
    w.world:remove_multi_body(mb)
    T.eq(w.world:get_num_multibodies(), 0)
    w.world:add_multi_body(mb) -- removed again by the cleanup
end)

T.test("btMultiBodyDynamicsWorld: a pendulum swings under gravity", function()
    local w = new_world()
    local mb = w:add(arm(1))
    local furthest = 0
    for _ = 1, 60 do
        w:step()
        furthest = math.max(furthest, math.abs(mb:get_joint_pos(0)))
    end
    T.ok(furthest > 0.8, "the bar swung through " .. furthest .. " rad")
    T.vec3(mb:get_base_pos(), 0, 5, 0, 1e-6) -- the fixed base does not move
end)

T.test("btMultiBodyDynamicsWorld: a two-link arm hangs straight down at rest", function()
    local w = new_world()
    local mb = w:add(arm(2))
    mb:set_linear_damping(0.9)
    mb:set_angular_damping(0.9)
    w.world:set_gravity(vec(0, -10, 0))
    -- strong damping lets the arm settle quickly
    for i = 0, 1 do mb:set_joint_pos(i, 0) end
    w:step(600)
    T.near(math.abs(mb:get_joint_pos(0)), math.pi / 2, 0.15)
end)

T.test("btMultiBodyJointMotor: drives a joint at a target velocity", function()
    local w = new_world()
    w.world:set_gravity(vec(0, 0, 0))
    local mb = w:add(arm(1))
    local motor = b.btMultiBodyJointMotor(mb, 0, 1.0, 100)
    w:add_constraint(motor)
    T.eq(w.world:get_num_multi_body_constraints(), 1)
    w:step(30)
    T.near(mb:get_joint_vel(0), 1, 0.1)
    T.near(mb:get_joint_pos(0), 0.5, 0.1)

    motor:set_velocity_target(-1)
    w:step(60)
    T.near(mb:get_joint_vel(0), -1, 0.1)

    motor:set_erp(0.5)
    T.near(motor:get_erp(), 0.5)
    motor:set_rhs_clamp(2)
    motor:set_position_target(0.25, 1.0)
end)

T.test("btMultiBodyJointLimitConstraint: stops a joint at its bounds", function()
    local w = new_world()
    local mb = w:add(arm(1))
    local limit = b.btMultiBodyJointLimitConstraint(mb, 0, -0.5, 0.5)
    T.near(limit:get_lower_bound(), -0.5)
    T.near(limit:get_upper_bound(), 0.5)
    w:add_constraint(limit)
    local lowest, highest = 0, 0
    for _ = 1, 180 do
        w:step()
        lowest, highest = math.min(lowest, mb:get_joint_pos(0)), math.max(highest, mb:get_joint_pos(0))
    end
    T.ok(lowest > -0.7 and highest < 0.7, "stayed within the limits, range " .. lowest .. " to " .. highest)
    T.ok(lowest < -0.3 or highest > 0.3, "and reached them")
    limit:set_lower_bound(-1)
    limit:set_upper_bound(1)
    T.near(limit:get_lower_bound(), -1)
    T.near(limit:get_upper_bound(), 1)
end)

-- ---------------------------------------------------------------------------------------
-- Other multibody constraints
-- ---------------------------------------------------------------------------------------

T.test("btMultiBodyConstraint: common interface", function()
    local w = new_world()
    local mb = w:add(arm(1))
    local motor = b.btMultiBodyJointMotor(mb, 0, 0, 10)
    motor:finalize_multi_dof()
    T.eq(motor:get_link_a(), 0)
    T.ok(motor:get_multi_body_a() ~= nil)
    T.eq(motor:get_num_rows(), 1)
    T.near(motor:get_max_applied_impulse(), 10)
    motor:set_max_applied_impulse(5)
    T.near(motor:get_max_applied_impulse(), 5)
    T.type_is(motor:is_unilateral(), "boolean")
    T.type_is(motor:get_island_id_a(), "number")
    T.type_is(motor:get_island_id_b(), "number")
    motor:set_position(0, 1.5)
    T.near(motor:get_position(0), 1.5)
end)

T.test("btMultiBodyPoint2Point, Fixed, Slider and Gear constraints: construction and accessors", function()
    local w = new_world()
    local a, c = w:add(arm(1)), w:add(arm(1))
    local ball = b.btRigidBody(1, b.btDefaultMotionState(), b.btSphereShape(0.5))
    local identity = b.btMatrix3x3(1, 0, 0, 0, 1, 0, 0, 0, 1)

    local p2p = b.btMultiBodyPoint2Point(a, 0, ball, vec(0, 0, 0), vec(0, 1, 0))
    T.vec3(p2p:get_pivot_in_b(), 0, 1, 0)
    p2p:set_pivot_in_b(vec(1, 1, 1))
    T.vec3(p2p:get_pivot_in_b(), 1, 1, 1)
    T.ok(b.btMultiBodyPoint2Point(a, 0, c, 0, vec(0, 0, 0), vec(0, 1, 0)) ~= nil)

    local fixed = b.btMultiBodyFixedConstraint(a, 0, ball, vec(1, 0, 0), vec(0, 1, 0), identity, identity)
    T.vec3(fixed:get_pivot_in_a(), 1, 0, 0)
    T.vec3(fixed:get_pivot_in_b(), 0, 1, 0)
    fixed:set_pivot_in_a(vec(2, 0, 0))
    fixed:set_pivot_in_b(vec(0, 2, 0))
    T.vec3(fixed:get_pivot_in_a(), 2, 0, 0)
    T.vec3(fixed:get_pivot_in_b(), 0, 2, 0)
    fixed:set_frame_in_a(identity)
    fixed:set_frame_in_b(identity)
    T.vec3(fixed:get_frame_in_a():get_row(0), 1, 0, 0)
    T.vec3(fixed:get_frame_in_b():get_row(1), 0, 1, 0)

    local slider = b.btMultiBodySliderConstraint(a, 0, ball, vec(0, 0, 0), vec(0, 0, 0), identity, identity, vec(1, 0, 0))
    T.vec3(slider:get_joint_axis(), 1, 0, 0)
    slider:set_joint_axis(vec(0, 1, 0))
    T.vec3(slider:get_joint_axis(), 0, 1, 0)
    slider:set_pivot_in_a(vec(1, 1, 1))
    T.vec3(slider:get_pivot_in_a(), 1, 1, 1)

    local gear = b.btMultiBodyGearConstraint(a, 0, c, 0, vec(0, 0, 0), vec(0, 0, 0), identity, identity)
    gear:set_gear_ratio(2)
    gear:set_gear_aux_link(0)
    gear:set_relative_position_target(0.5)
    gear:set_erp(0.2)
    -- (the constructor leaves the pivot unset, so write it before reading it back)
    gear:set_pivot_in_a(vec(1, 2, 3))
    T.vec3(gear:get_pivot_in_a(), 1, 2, 3)
end)

T.test("btMultiBodySphericalJointMotor and Limit: construction and settings", function()
    local mb = b.btMultiBody(1, 1, vec(1, 1, 1), false, false)
    mb:setup_spherical(0, 1, vec(1, 1, 1), -1, IDENTITY(), vec(0, 0, 0), vec(0, 0, 0))
    mb:finalize_multi_dof()

    local motor = b.btMultiBodySphericalJointMotor(mb, 0, 10)
    motor:set_erp(0.5)
    T.near(motor:get_erp(), 0.5)
    motor:set_rhs_clamp(1)
    motor:set_velocity_target(vec(0, 1, 0), 1)
    motor:set_position_target(IDENTITY(), 1)
    motor:set_max_applied_impulse_multi_dof(vec(1, 2, 3))
    T.near(motor:get_max_applied_impulse_multi_dof(1), 2)
    motor:set_damping(vec(0.1, 0.2, 0.3))
    T.near(motor:get_damping(2), 0.3, 1e-6)

    local limit = b.btMultiBodySphericalJointLimit(mb, 0, -1, 1, -1, 1)
    T.ok(limit ~= nil)
    limit:set_erp(0.1)
    T.near(limit:get_erp(), 0.1)
end)

T.test("MLCP solvers: the solver pair behind the multibody world", function()
    local dantzig = b.btDantzigSolver()
    local lemke = b.btLemkeSolver()
    local solver = b.btMLCPSolver(dantzig)
    solver:set_mlcp_solver(lemke)
    solver:set_num_fallbacks(3)
    T.eq(solver:get_num_fallbacks(), 3)

    local multibody_solver = b.btMultiBodyMLCPConstraintSolver(dantzig)
    multibody_solver:set_num_fallbacks(5)
    T.eq(multibody_solver:get_num_fallbacks(), 5)
    multibody_solver:set_mlcp_solver(lemke)
end)

T.run()
