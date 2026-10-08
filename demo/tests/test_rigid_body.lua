-- btRigidBody and btCollisionObject: mass properties, forces, impulses, velocities,
-- damping, flags and sleeping. Everything here works on a body outside of any world.

local T = require("testlib")
local b = bullet3

local ACTIVE_TAG, ISLAND_SLEEPING, WANTS_DEACTIVATION, DISABLE_DEACTIVATION = 1, 2, 3, 4
local CF_STATIC_OBJECT, CF_KINEMATIC_OBJECT, CF_NO_CONTACT_RESPONSE = 1, 2, 4
local BT_DISABLE_WORLD_GRAVITY, BT_ENABLE_GYROSCOPIC_FORCE_IMPLICIT_BODY = 1, 8

local function transform_at(x, y, z)
    return b.btTransform(b.btQuaternion(0, 0, 0, 1), b.btVector3(x, y, z))
end

-- A body made of a unit sphere with the given mass. The shape and motion state are returned
-- as well: the caller has to keep them referenced while the body is in use.
local function sphere_body(mass, start)
    local shape = b.btSphereShape(1)
    local state = b.btDefaultMotionState(start or transform_at(0, 0, 0))
    local inertia = b.btVector3(0, 0, 0)
    if mass ~= 0 then
        shape:calculate_local_inertia(mass, inertia)
    end
    return b.btRigidBody(mass, state, shape, inertia), shape, state
end

-- ---------------------------------------------------------------------------------------
-- Construction and mass properties
-- ---------------------------------------------------------------------------------------

T.test("btRigidBody: mass and inverse mass", function()
    local body, shape, state = sphere_body(2)
    T.near(body:get_mass(), 2)
    T.near(body:get_inv_mass(), 0.5)
    T.ok(not body:is_static_object())

    local static, shape2, state2 = sphere_body(0)
    T.near(static:get_mass(), 0)
    T.near(static:get_inv_mass(), 0)
    T.ok(static:is_static_object())
    T.ok(static:is_static_or_kinematic_object())
    T.eq(static:get_collision_flags() & CF_STATIC_OBJECT, CF_STATIC_OBJECT)
end)

T.test("btRigidBody: constructor variants", function()
    local shape = b.btSphereShape(1)
    local state = b.btDefaultMotionState(transform_at(1, 2, 3))

    local without_inertia = b.btRigidBody(3, state, shape)
    T.near(without_inertia:get_mass(), 3)
    -- no inertia given: the body does not rotate
    T.vec3(without_inertia:get_inv_inertia_diag_local(), 0, 0, 0)

    local info = b.btRigidBodyConstructionInfo(3, state, shape, b.btVector3(1, 2, 4))
    local from_info = b.btRigidBody(info)
    T.near(from_info:get_mass(), 3)
    T.vec3(from_info:get_inv_inertia_diag_local(), 1, 0.5, 0.25)
    T.vec3(from_info:get_world_transform():get_origin(), 1, 2, 3)

    local info_without_inertia = b.btRigidBodyConstructionInfo(3, state, shape)
    T.near(b.btRigidBody(info_without_inertia):get_mass(), 3)
end)

T.test("btRigidBody: shape and motion state", function()
    local body, shape, state = sphere_body(1, transform_at(4, 5, 6))
    T.eq(body:get_collision_shape():get_name(), "SPHERE")
    T.ok(body:get_motion_state() ~= nil)
    T.vec3(body:get_world_transform():get_origin(), 4, 5, 6)

    local other_state = b.btDefaultMotionState(transform_at(7, 8, 9))
    body:set_motion_state(other_state)
    local out = b.btTransform(b.btQuaternion(0, 0, 0, 1), b.btVector3(0, 0, 0))
    body:get_motion_state():get_world_transform(out)
    T.vec3(out:get_origin(), 7, 8, 9)
end)

T.test("btRigidBody: set_mass_props", function()
    local body, shape, state = sphere_body(1)
    body:set_mass_props(4, b.btVector3(1, 2, 4))
    T.near(body:get_mass(), 4)
    T.near(body:get_inv_mass(), 0.25)
    T.vec3(body:get_inv_inertia_diag_local(), 1, 0.5, 0.25)
    T.vec3(body:get_local_inertia(), 1, 2, 4)

    body:set_inv_inertia_diag_local(b.btVector3(2, 2, 2))
    T.vec3(body:get_inv_inertia_diag_local(), 2, 2, 2)
    body:update_inertia_tensor()
    local tensor = body:get_inv_inertia_tensor_world()
    T.vec3(tensor:get_row(0), 2, 0, 0)
    T.vec3(tensor:get_row(2), 0, 0, 2)

    -- mass 0 turns the body static
    body:set_mass_props(0, b.btVector3(0, 0, 0))
    T.near(body:get_inv_mass(), 0)
    T.ok(body:is_static_object())
end)

-- ---------------------------------------------------------------------------------------
-- Forces and impulses
-- ---------------------------------------------------------------------------------------

T.test("btRigidBody: forces accumulate until cleared", function()
    local body, shape, state = sphere_body(2)
    body:apply_central_force(b.btVector3(1, 2, 3))
    body:apply_central_force(b.btVector3(1, 0, 0))
    T.vec3(body:get_total_force(), 2, 2, 3)
    T.vec3(body:get_total_torque(), 0, 0, 0)

    body:apply_torque(b.btVector3(0, 0, 5))
    T.vec3(body:get_total_torque(), 0, 0, 5)

    -- a force off the centre of mass also applies a torque: r x F
    body:clear_forces()
    T.vec3(body:get_total_force(), 0, 0, 0)
    body:apply_force(b.btVector3(0, 3, 0), b.btVector3(1, 0, 0))
    T.vec3(body:get_total_force(), 0, 3, 0)
    T.vec3(body:get_total_torque(), 0, 0, 3)

    body:clear_forces()
    T.vec3(body:get_total_force(), 0, 0, 0)
    T.vec3(body:get_total_torque(), 0, 0, 0)
end)

T.test("btRigidBody: gravity is an acceleration, applied as mass * acceleration", function()
    local body, shape, state = sphere_body(2)
    body:set_gravity(b.btVector3(0, -10, 0))
    T.vec3(body:get_gravity(), 0, -10, 0)
    body:apply_gravity()
    T.vec3(body:get_total_force(), 0, -20, 0)
    body:clear_gravity()
    T.vec3(body:get_total_force(), 0, 0, 0)

    local static, shape2, state2 = sphere_body(0)
    static:set_gravity(b.btVector3(0, -10, 0))
    static:apply_gravity()
    T.vec3(static:get_total_force(), 0, 0, 0)
end)

T.test("btRigidBody: impulses change the velocity at once", function()
    local body, shape, state = sphere_body(2) -- inverse mass 0.5, sphere inertia 0.8
    body:apply_central_impulse(b.btVector3(4, 0, 2))
    T.vec3(body:get_linear_velocity(), 2, 0, 1)

    body:apply_torque_impulse(b.btVector3(0, 0, 1.6))
    T.vec3(body:get_angular_velocity(), 0, 0, 2)

    -- an impulse of (0, 2, 0) at (1, 0, 0): v += 1 in y, w += I^-1 (r x J) = 1.25 * (0, 0, 2)
    body:set_linear_velocity(b.btVector3(0, 0, 0))
    body:set_angular_velocity(b.btVector3(0, 0, 0))
    body:apply_impulse(b.btVector3(0, 2, 0), b.btVector3(1, 0, 0))
    T.vec3(body:get_linear_velocity(), 0, 1, 0)
    T.vec3(body:get_angular_velocity(), 0, 0, 2.5)
end)

T.test("btRigidBody: linear and angular factors restrict the motion", function()
    local body, shape, state = sphere_body(1)
    body:set_linear_factor(b.btVector3(1, 0, 1))
    T.vec3(body:get_linear_factor(), 1, 0, 1)
    body:apply_central_impulse(b.btVector3(1, 1, 1))
    T.vec3(body:get_linear_velocity(), 1, 0, 1)

    body:set_angular_factor(b.btVector3(0, 1, 0))
    T.vec3(body:get_angular_factor(), 0, 1, 0)
    body:set_angular_factor(0.5)
    T.vec3(body:get_angular_factor(), 0.5, 0.5, 0.5)
end)

T.test("btRigidBody: push and turn velocities", function()
    local body, shape, state = sphere_body(2)
    T.vec3(body:get_push_velocity(), 0, 0, 0)
    body:apply_central_push_impulse(b.btVector3(2, 0, 0))
    T.vec3(body:get_push_velocity(), 1, 0, 0)
    body:apply_torque_turn_impulse(b.btVector3(0, 1.6, 0))
    T.vec3(body:get_turn_velocity(), 0, 2, 0)

    body:set_push_velocity(b.btVector3(0, 0, 3))
    body:set_turn_velocity(b.btVector3(1, 1, 1))
    T.vec3(body:get_push_velocity(), 0, 0, 3)
    T.vec3(body:get_turn_velocity(), 1, 1, 1)

    -- v_push + w_turn x r
    body:set_push_velocity(b.btVector3(1, 0, 0))
    body:set_turn_velocity(b.btVector3(0, 0, 2))
    T.vec3(body:get_push_velocity_in_local_point(b.btVector3(1, 0, 0)), 1, 2, 0)

    body:set_push_velocity(b.btVector3(0, 0, 0))
    body:apply_push_impulse(b.btVector3(0, 4, 0), b.btVector3(0, 0, 0))
    T.vec3(body:get_push_velocity(), 0, 2, 0)
end)

-- ---------------------------------------------------------------------------------------
-- Velocities, damping and integration
-- ---------------------------------------------------------------------------------------

T.test("btRigidBody: velocity at a point", function()
    local body, shape, state = sphere_body(1)
    body:set_linear_velocity(b.btVector3(1, 0, 0))
    body:set_angular_velocity(b.btVector3(0, 0, 2))
    T.vec3(body:get_linear_velocity(), 1, 0, 0)
    T.vec3(body:get_angular_velocity(), 0, 0, 2)
    -- v + w x r
    T.vec3(body:get_velocity_in_local_point(b.btVector3(1, 0, 0)), 1, 2, 0)
    T.vec3(body:get_velocity_in_local_point(b.btVector3(0, 1, 0)), -1, 0, 0)
end)

T.test("btRigidBody: damping", function()
    local body, shape, state = sphere_body(1)
    body:set_damping(0.5, 0.25)
    T.near(body:get_linear_damping(), 0.5)
    T.near(body:get_angular_damping(), 0.25)

    body:set_linear_velocity(b.btVector3(4, 0, 0))
    body:set_angular_velocity(b.btVector3(0, 8, 0))
    body:apply_damping(1) -- v *= (1 - damping)^dt
    T.vec3(body:get_linear_velocity(), 2, 0, 0, 1e-4)
    T.vec3(body:get_angular_velocity(), 0, 6, 0, 1e-4)
end)

T.test("btRigidBody: integrate_velocities and predict_integrated_transform", function()
    local body, shape, state = sphere_body(2)
    body:apply_central_force(b.btVector3(10, 0, 0))
    body:integrate_velocities(0.5) -- v += F * invMass * dt
    T.vec3(body:get_linear_velocity(), 2.5, 0, 0)

    local predicted = b.btTransform(b.btQuaternion(0, 0, 0, 1), b.btVector3(0, 0, 0))
    body:predict_integrated_transform(2, predicted)
    T.vec3(predicted:get_origin(), 5, 0, 0)
    -- predicting does not move the body
    T.vec3(body:get_world_transform():get_origin(), 0, 0, 0)
end)

T.test("btRigidBody: moving the body", function()
    local body, shape, state = sphere_body(1, transform_at(1, 1, 1))
    body:translate(b.btVector3(1, 2, 3))
    T.vec3(body:get_center_of_mass_position(), 2, 3, 4)
    T.vec3(body:get_world_transform():get_origin(), 2, 3, 4)

    body:set_center_of_mass_transform(transform_at(10, 0, 0))
    T.vec3(body:get_center_of_mass_transform():get_origin(), 10, 0, 0)
    T.vec3(body:get_center_of_mass_position(), 10, 0, 0)

    local turned = b.btTransform(b.btQuaternion(b.btVector3(0, 0, 1), math.pi / 2), b.btVector3(0, 0, 0))
    body:proceed_to_transform(turned)
    T.quat(body:get_orientation(), 0, 0, math.sqrt(0.5), math.sqrt(0.5))
    T.vec3(body:get_center_of_mass_position(), 0, 0, 0)
end)

T.test("btRigidBody: the world inertia tensor follows the rotation", function()
    local shape = b.btBoxShape(b.btVector3(1, 2, 3))
    local state = b.btDefaultMotionState()
    local inertia = b.btVector3(0, 0, 0)
    shape:calculate_local_inertia(12, inertia) -- (52, 40, 20)
    local body = b.btRigidBody(12, state, shape, inertia)
    local tensor = body:get_inv_inertia_tensor_world()
    T.near(tensor:get_row(0):x(), 1 / 52, 1e-6)

    -- turn 90 degrees about z: the x and y diagonal terms swap
    body:set_center_of_mass_transform(b.btTransform(b.btQuaternion(b.btVector3(0, 0, 1), math.pi / 2), b.btVector3(0, 0, 0)))
    tensor = body:get_inv_inertia_tensor_world()
    T.near(tensor:get_row(0):x(), 1 / 40, 1e-5)
    T.near(tensor:get_row(1):y(), 1 / 52, 1e-5)
    T.near(tensor:get_row(2):z(), 1 / 20, 1e-5)
end)

T.test("btRigidBody: impulse denominators", function()
    local body, shape, state = sphere_body(1) -- inertia 0.4
    -- invMass + n . ((r x n) I^-1 x r) = 1 + 2.5
    T.near(body:compute_impulse_denominator(b.btVector3(1, 0, 0), b.btVector3(0, 1, 0)), 3.5, 1e-4)
    -- n . (I^-1 n)
    T.near(body:compute_angular_impulse_denominator(b.btVector3(0, 1, 0)), 2.5, 1e-4)
end)

T.test("btRigidBody: gyroscopic terms vanish without spin", function()
    local shape = b.btBoxShape(b.btVector3(1, 2, 3))
    local state = b.btDefaultMotionState()
    local inertia = b.btVector3(0, 0, 0)
    shape:calculate_local_inertia(1, inertia)
    local body = b.btRigidBody(1, state, shape, inertia)

    T.near(body:compute_gyroscopic_impulse_implicit__body(0.01):length(), 0, 1e-6)
    T.near(body:compute_gyroscopic_impulse_implicit__world(0.01):length(), 0, 1e-6)
    T.near(body:compute_gyroscopic_force_explicit(100):length(), 0, 1e-6)

    -- spinning around an axis that is not a principal axis of the box
    body:set_angular_velocity(b.btVector3(5, 5, 5))
    T.ok(body:compute_gyroscopic_impulse_implicit__body(0.01):length() > 0)
    T.ok(body:compute_gyroscopic_force_explicit(1000):length() > 0)
end)

T.test("btRigidBody: AABB of the shape at the body's position", function()
    local body, shape, state = sphere_body(1, transform_at(5, 0, 0))
    local min, max = b.btVector3(0, 0, 0), b.btVector3(0, 0, 0)
    body:get_aabb(min, max)
    T.vec3(min, 4, -1, -1, 1e-3)
    T.vec3(max, 6, 1, 1, 1e-3)
end)

T.test("btRigidBody: kinematic bodies take their velocity from the motion state", function()
    local shape = b.btSphereShape(1)
    local state = b.btDefaultMotionState(transform_at(0, 0, 0))
    local body = b.btRigidBody(0, state, shape)
    body:set_collision_flags(body:get_collision_flags() | CF_KINEMATIC_OBJECT)
    T.ok(body:is_kinematic_object())
    state:set_world_transform(transform_at(2, 0, 0))
    body:save_kinematic_state(1)
    T.vec3(body:get_linear_velocity(), 2, 0, 0)
    T.vec3(body:get_world_transform():get_origin(), 2, 0, 0)
end)

-- ---------------------------------------------------------------------------------------
-- Sleeping and flags
-- ---------------------------------------------------------------------------------------

T.test("btRigidBody: sleeping thresholds and deactivation", function()
    local body, shape, state = sphere_body(1)
    T.near(body:get_linear_sleeping_threshold(), 0.8, 1e-5)
    T.near(body:get_angular_sleeping_threshold(), 1.0, 1e-5)
    body:set_sleeping_thresholds(0.1, 0.2)
    T.near(body:get_linear_sleeping_threshold(), 0.1, 1e-6)
    T.near(body:get_angular_sleeping_threshold(), 0.2, 1e-6)

    T.ok(not body:wants_sleeping())
    body:update_deactivation(1)
    T.near(body:get_deactivation_time(), 1, 1e-5)
    T.ok(not body:wants_sleeping())
    body:update_deactivation(1.5)
    T.ok(body:wants_sleeping(), "at rest for longer than the deactivation time")

    -- motion resets the timer
    body:set_linear_velocity(b.btVector3(5, 0, 0))
    body:update_deactivation(1)
    T.near(body:get_deactivation_time(), 0, 1e-6)
end)

T.test("btRigidBody: flags", function()
    local body, shape, state = sphere_body(1)
    T.eq(body:get_flags(), BT_ENABLE_GYROSCOPIC_FORCE_IMPLICIT_BODY, "implicit gyroscopic force is the default")
    body:set_flags(BT_DISABLE_WORLD_GRAVITY)
    T.eq(body:get_flags(), BT_DISABLE_WORLD_GRAVITY)
end)

T.test("btRigidBody: not in a world until added", function()
    local body, shape, state = sphere_body(1)
    T.ok(not body:is_in_world())
    T.is_nil(body:get_broadphase_proxy())
end)

T.test("btRigidBody: upcast from a collision object", function()
    local body, shape, state = sphere_body(1)
    local same = b.btRigidBody.upcast(body)
    T.ok(same ~= nil)
    T.near(same:get_mass(), 1)
    T.is_nil(b.btRigidBody.upcast(b.btCollisionObject()), "a plain collision object is not a rigid body")
end)

-- ---------------------------------------------------------------------------------------
-- btCollisionObject
-- ---------------------------------------------------------------------------------------

T.test("btCollisionObject: material properties", function()
    local object = b.btCollisionObject()
    T.near(object:get_friction(), 0.5)
    T.near(object:get_restitution(), 0)
    T.near(object:get_rolling_friction(), 0)
    T.near(object:get_spinning_friction(), 0)

    object:set_friction(0.9)
    object:set_restitution(0.3)
    object:set_rolling_friction(0.1)
    object:set_spinning_friction(0.2)
    T.near(object:get_friction(), 0.9)
    T.near(object:get_restitution(), 0.3)
    T.near(object:get_rolling_friction(), 0.1)
    T.near(object:get_spinning_friction(), 0.2)

    object:set_contact_stiffness_and_damping(1000, 10)
    T.near(object:get_contact_stiffness(), 1000)
    T.near(object:get_contact_damping(), 10)

    object:set_contact_processing_threshold(0.5)
    T.near(object:get_contact_processing_threshold(), 0.5)
    object:set_hit_fraction(0.25)
    T.near(object:get_hit_fraction(), 0.25)
end)

T.test("btCollisionObject: collision flags", function()
    local object = b.btCollisionObject()
    T.eq(object:get_collision_flags(), CF_STATIC_OBJECT)
    T.ok(object:is_static_object())
    T.ok(object:has_contact_response())

    object:set_collision_flags(CF_KINEMATIC_OBJECT)
    T.ok(object:is_kinematic_object())
    T.ok(not object:is_static_object())
    T.ok(object:is_static_or_kinematic_object())

    object:set_collision_flags(CF_NO_CONTACT_RESPONSE)
    T.ok(not object:has_contact_response())

    object:set_collision_flags(0)
    T.ok(not object:is_static_or_kinematic_object())
end)

T.test("btCollisionObject: activation", function()
    local object = b.btCollisionObject()
    T.eq(object:get_activation_state(), ACTIVE_TAG)
    T.ok(object:is_active())

    object:set_activation_state(ISLAND_SLEEPING)
    T.eq(object:get_activation_state(), ISLAND_SLEEPING)
    T.ok(not object:is_active())

    -- a plain collision object is static, and static objects are only woken by force
    object:activate()
    T.eq(object:get_activation_state(), ISLAND_SLEEPING)
    object:activate(true)
    T.eq(object:get_activation_state(), ACTIVE_TAG)
    T.ok(object:is_active())

    object:set_activation_state(ISLAND_SLEEPING)
    object:set_collision_flags(0)
    object:activate()
    T.eq(object:get_activation_state(), ACTIVE_TAG)

    -- a body that must never sleep ignores set_activation_state until forced
    object:force_activation_state(DISABLE_DEACTIVATION)
    object:set_activation_state(ISLAND_SLEEPING)
    T.eq(object:get_activation_state(), DISABLE_DEACTIVATION)
    object:force_activation_state(ACTIVE_TAG)
    T.eq(object:get_activation_state(), ACTIVE_TAG)

    object:set_deactivation_time(1.5)
    T.near(object:get_deactivation_time(), 1.5)
end)

T.test("btCollisionObject: world transform and shape", function()
    local object = b.btCollisionObject()
    object:set_world_transform(transform_at(1, 2, 3))
    T.vec3(object:get_world_transform():get_origin(), 1, 2, 3)

    local shape = b.btBoxShape(b.btVector3(1, 1, 1))
    object:set_collision_shape(shape)
    T.eq(object:get_collision_shape():get_name(), "Box")
end)

T.test("btCollisionObject: anisotropic friction", function()
    local object = b.btCollisionObject()
    T.ok(not object:has_anisotropic_friction())
    object:set_anisotropic_friction(b.btVector3(1, 0.5, 0.25))
    T.ok(object:has_anisotropic_friction())
    T.vec3(object:get_anisotropic_friction(), 1, 0.5, 0.25)
end)

T.test("btCollisionObject: ignoring other objects", function()
    local object, other = b.btCollisionObject(), b.btCollisionObject()
    T.ok(object:check_collide_with(other))
    T.eq(object:get_num_objects_without_collision(), 0)
    object:set_ignore_collision_check(other, true)
    T.eq(object:get_num_objects_without_collision(), 1)
    T.ok(object:check_collide_with_override(other) == false)
    object:set_ignore_collision_check(other, false)
    T.eq(object:get_num_objects_without_collision(), 0)
end)

T.test("btCollisionObject: continuous collision detection settings", function()
    local object = b.btCollisionObject()
    object:set_ccd_swept_sphere_radius(0.25)
    T.near(object:get_ccd_swept_sphere_radius(), 0.25)
    object:set_ccd_motion_threshold(0.5)
    T.near(object:get_ccd_motion_threshold(), 0.5)
    T.near(object:get_ccd_square_motion_threshold(), 0.25)
end)

T.test("btCollisionObject: user indices", function()
    local object = b.btCollisionObject()
    object:set_user_index(1)
    object:set_user_index2(2)
    object:set_user_index3(3)
    T.eq(object:get_user_index(), 1)
    T.eq(object:get_user_index2(), 2)
    T.eq(object:get_user_index3(), 3)
end)

T.test("btCollisionObject: custom debug colour", function()
    local object = b.btCollisionObject()
    local color = b.btVector3(0, 0, 0)
    T.ok(not object:get_custom_debug_color(color))
    object:set_custom_debug_color(b.btVector3(1, 0.5, 0.25))
    T.ok(object:get_custom_debug_color(color))
    T.vec3(color, 1, 0.5, 0.25)
    object:remove_custom_debug_color()
    T.ok(not object:get_custom_debug_color(color))
end)

T.test("btCollisionObject: interpolation state", function()
    local object = b.btCollisionObject()
    object:set_interpolation_world_transform(transform_at(1, 0, 0))
    object:set_interpolation_linear_velocity(b.btVector3(0, 2, 0))
    object:set_interpolation_angular_velocity(b.btVector3(0, 0, 3))
    T.vec3(object:get_interpolation_world_transform():get_origin(), 1, 0, 0)
    T.vec3(object:get_interpolation_linear_velocity(), 0, 2, 0)
    T.vec3(object:get_interpolation_angular_velocity(), 0, 0, 3)
end)

T.test("btCollisionObject: bookkeeping ids", function()
    local object = b.btCollisionObject()
    object:set_island_tag(4)
    object:set_companion_id(5)
    object:set_world_array_index(6)
    T.eq(object:get_island_tag(), 4)
    T.eq(object:get_companion_id(), 5)
    T.eq(object:get_world_array_index(), 6)
    T.eq(object:get_internal_type(), 1) -- CO_COLLISION_OBJECT
    T.eq(b.btRigidBody(1, b.btDefaultMotionState(), b.btSphereShape(1)):get_internal_type(), 2) -- CO_RIGID_BODY
end)

T.run()
