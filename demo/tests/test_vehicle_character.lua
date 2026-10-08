-- btRaycastVehicle and btKinematicCharacterController, both driven as world actions.

local T = require("testlib")
local P = require("physics")
local b = bullet3

local function vec(x, y, z)
    return b.btVector3(x, y, z)
end

-- ---------------------------------------------------------------------------------------
-- Raycast vehicle
-- ---------------------------------------------------------------------------------------

-- The wheel travel: the default tuning is soft (the spring force scales with the chassis mass,
-- stiffness 5.88), so the car sags by about 0.4 m on each wheel and needs room for it
local SUSPENSION_REST_LENGTH = 0.8
local WHEEL_RADIUS = 0.4

-- A four-wheeled car on a floor. Forward is +z, up is +y, right is +x.
local function car(p)
    p:add_floor(0)
    local chassis = p:add_body({ shape = b.btBoxShape(vec(0.8, 0.3, 1.6)), mass = 800, position = { 0, 1.8, 0 } })
    chassis:set_activation_state(4) -- DISABLE_DEACTIVATION: a vehicle must stay awake

    local tuning = p:hold(b.btVehicleTuning())
    local raycaster = p:hold(b.btDefaultVehicleRaycaster(p.world))
    local vehicle = b.btRaycastVehicle(tuning, chassis, raycaster)
    vehicle:set_coordinate_system(0, 1, 2)
    p:add_action(vehicle)

    local wheel_direction, wheel_axle = vec(0, -1, 0), vec(-1, 0, 0)
    for _, wheel in ipairs({ { 0.8, 1.2, true }, { -0.8, 1.2, true }, { 0.8, -1.2, false }, { -0.8, -1.2, false } }) do
        vehicle:add_wheel(vec(wheel[1], 0, wheel[2]), wheel_direction, wheel_axle,
            SUSPENSION_REST_LENGTH, WHEEL_RADIUS, tuning, wheel[3])
    end
    return chassis, vehicle
end

T.test("btRaycastVehicle: axes and wheels", function()
    local p = P.new(T)
    local chassis, vehicle = car(p)
    T.eq(vehicle:get_num_wheels(), 4)
    T.eq(vehicle:get_right_axis(), 0)
    T.eq(vehicle:get_up_axis(), 1)
    T.eq(vehicle:get_forward_axis(), 2)
    T.ok(vehicle:get_rigid_body() ~= nil)
    T.near(vehicle:get_rigid_body():get_mass(), 800)

    for i = 0, 3 do
        T.near(vehicle:get_wheel_info(i):get_suspension_rest_length(), SUSPENSION_REST_LENGTH, 1e-5)
    end

    vehicle:set_coordinate_system(0, 2, 1)
    T.eq(vehicle:get_up_axis(), 2)
    T.eq(vehicle:get_forward_axis(), 1)
    vehicle:set_coordinate_system(0, 1, 2)
    T.ok(chassis ~= nil)
end)

T.test("btRaycastVehicle: user constraint ids", function()
    local p = P.new(T)
    local _, vehicle = car(p)
    vehicle:set_user_constraint_id(12)
    vehicle:set_user_constraint_type(3)
    T.eq(vehicle:get_user_constraint_id(), 12)
    T.eq(vehicle:get_user_constraint_type(), 3)
end)

T.test("btRaycastVehicle: steering, braking and engine force can be set per wheel", function()
    local p = P.new(T)
    local _, vehicle = car(p)
    T.near(vehicle:get_steering_value(0), 0)
    vehicle:set_steering_value(0.3, 0)
    vehicle:set_steering_value(0.3, 1)
    T.near(vehicle:get_steering_value(0), 0.3, 1e-6)
    T.near(vehicle:get_steering_value(1), 0.3, 1e-6)
    T.near(vehicle:get_steering_value(2), 0)

    vehicle:apply_engine_force(500, 2)
    vehicle:apply_engine_force(500, 3)
    vehicle:set_brake(20, 0)
    vehicle:set_pitch_control(0.1)
end)

T.test("btRaycastVehicle: the suspension holds the chassis above the floor", function()
    local p = P.new(T)
    local chassis, vehicle = car(p)
    p:step(180)
    local _, y = P.position(chassis)
    -- the wheels carry the chassis: it stays clear of the floor (its half height is 0.3)
    T.ok(y > 0.6 and y < 1.6, "chassis height " .. y)
    for i = 0, 3 do
        T.ok(vehicle:get_wheel_info(i) ~= nil)
    end
    T.ok(vehicle:get_wheel_transform_ws(0):get_origin():y() < y, "wheels hang below the chassis")
    T.near(vehicle:get_current_speed_km_hour(), 0, 1)
end)

T.test("btRaycastVehicle: engine force drives the car forward", function()
    local p = P.new(T)
    local chassis, vehicle = car(p)
    p:step(120) -- settle
    local _, _, z0 = P.position(chassis)
    for _ = 1, 180 do
        vehicle:apply_engine_force(2000, 2)
        vehicle:apply_engine_force(2000, 3)
        p:step(1)
    end
    local _, _, z1 = P.position(chassis)
    T.ok(z1 - z0 > 2, "drove forward by " .. (z1 - z0))
    T.ok(vehicle:get_current_speed_km_hour() > 3, "speed " .. vehicle:get_current_speed_km_hour())
    T.ok(vehicle:get_forward_vector():z() > 0.9, "the car still points along +z")
end)

T.test("btRaycastVehicle: steering turns the car", function()
    local p = P.new(T)
    local chassis, vehicle = car(p)
    p:step(120)
    for _ = 1, 240 do
        vehicle:set_steering_value(0.4, 0)
        vehicle:set_steering_value(0.4, 1)
        vehicle:apply_engine_force(2000, 2)
        vehicle:apply_engine_force(2000, 3)
        p:step(1)
    end
    local forward = vehicle:get_forward_vector()
    T.ok(math.abs(forward:x()) > 0.05, "the heading turned away from +z, forward.x = " .. forward:x())
end)

T.test("btRaycastVehicle: brakes bring the car to a stop", function()
    local p = P.new(T)
    local chassis, vehicle = car(p)
    p:step(120)
    chassis:set_linear_velocity(vec(0, 0, 10))
    p:step(10)
    local fast = vehicle:get_current_speed_km_hour()
    T.ok(fast > 20, "speed " .. fast)
    for _ = 1, 240 do
        for wheel = 0, 3 do vehicle:set_brake(200, wheel) end
        p:step(1)
    end
    T.ok(vehicle:get_current_speed_km_hour() < fast / 2, "braked to " .. vehicle:get_current_speed_km_hour())
end)

T.test("btRaycastVehicle: reset_suspension and the wheel transforms", function()
    local p = P.new(T)
    local _, vehicle = car(p)
    p:step(30)
    vehicle:reset_suspension()
    vehicle:update_wheel_transform(0, true)
    vehicle:update_wheel_transform(1, false)
    T.type_is(vehicle:get_wheel_transform_ws(0):get_origin():x(), "number")
    T.ok(vehicle:get_chassis_world_transform() ~= nil)
end)

-- ---------------------------------------------------------------------------------------
-- Kinematic character controller
-- ---------------------------------------------------------------------------------------

local CHARACTER_FILTER, DEFAULT_FILTER, STATIC_FILTER = 16, 1, 2

-- A capsule-shaped character standing above a floor in a world with ghost support
local function character(p, start_y)
    p.broadphase:get_overlapping_pair_cache():set_internal_ghost_pair_callback(p:hold(b.btGhostPairCallback()))
    p:add_floor(0)

    -- The controller rotates the ghost so that the shape's local z axis points along `up`, so a
    -- standing capsule is a z-axis capsule. 1.8 m tall with the caps.
    local capsule = p:hold(b.btCapsuleShapeZ(0.4, 1.0))
    local ghost = p:hold(b.btPairCachingGhostObject())
    ghost:set_world_transform(b.btTransform(b.btQuaternion(0, 0, 0, 1), vec(0, start_y or 1.5, 0)))
    ghost:set_collision_shape(capsule)
    ghost:set_collision_flags(16) -- CF_CHARACTER_OBJECT
    p.world:add_collision_object(ghost, CHARACTER_FILTER, DEFAULT_FILTER | STATIC_FILTER)

    -- (the default up vector of the three-argument constructor is the x axis)
    local controller = b.btKinematicCharacterController(ghost, capsule, 0.35, vec(0, 1, 0))
    p:add_action(controller)
    T.cleanup(function() p.world:remove_collision_object(ghost) end)
    return controller, ghost
end

local function height(ghost)
    return ghost:get_world_transform():get_origin():y()
end

T.test("btKinematicCharacterController: settings", function()
    local p = P.new(T)
    local controller = character(p)

    controller:set_step_height(0.5)
    T.near(controller:get_step_height(), 0.5)
    controller:set_fall_speed(30)
    T.near(controller:get_fall_speed(), 30)
    controller:set_jump_speed(8)
    T.near(controller:get_jump_speed(), 8)
    controller:set_max_slope(0.5)
    T.near(controller:get_max_slope(), 0.5)
    controller:set_max_penetration_depth(0.1)
    T.near(controller:get_max_penetration_depth(), 0.1)
    controller:set_gravity(vec(0, -20, 0))
    T.vec3(controller:get_gravity(), 0, -20, 0)
    controller:set_up(vec(0, 1, 0))
    T.vec3(controller:get_up(), 0, 1, 0)
    controller:set_linear_damping(0.2)
    T.near(controller:get_linear_damping(), 0.2)
    controller:set_angular_damping(0.3)
    T.near(controller:get_angular_damping(), 0.3)
    controller:set_angular_velocity(vec(0, 1, 0))
    T.vec3(controller:get_angular_velocity(), 0, 1, 0)
    controller:set_linear_velocity(vec(1, 0, 0))
    T.type_is(controller:get_linear_velocity():x(), "number")
    controller:set_use_ghost_sweep_test(true)
    controller:set_up_interpolate(true)
    controller:set_max_jump_height(2)
    T.ok(controller:get_ghost_object() ~= nil)
end)

T.test("btKinematicCharacterController: falls and lands on the floor", function()
    local p = P.new(T)
    local controller, ghost = character(p, 3)
    p:step(5)
    T.ok(not controller:on_ground(), "still falling")
    T.ok(height(ghost) < 3)
    p:step(175)
    T.ok(controller:on_ground(), "standing on the floor")
    -- capsule: radius 0.4 + half height 0.5 = 0.9 above the floor
    T.near(height(ghost), 0.9, 0.1)
end)

T.test("btKinematicCharacterController: walking moves the character", function()
    local p = P.new(T)
    local controller, ghost = character(p, 1)
    p:step(60)
    T.ok(controller:on_ground())
    local z0 = ghost:get_world_transform():get_origin():z()
    for _ = 1, 60 do
        controller:set_walk_direction(vec(0, 0, 0.05)) -- the distance to walk per step
        p:step(1)
    end
    local z1 = ghost:get_world_transform():get_origin():z()
    T.near(z1 - z0, 3, 0.3)
    T.near(height(ghost), 0.9, 0.1)

    -- no walk direction: the character stops
    controller:set_walk_direction(vec(0, 0, 0))
    local before = ghost:get_world_transform():get_origin():z()
    p:step(30)
    T.near(ghost:get_world_transform():get_origin():z(), before, 0.05)
end)

T.test("btKinematicCharacterController: jumping", function()
    local p = P.new(T)
    local controller, ghost = character(p, 1)
    p:step(60)
    T.ok(controller:can_jump())
    local ground = height(ghost)
    controller:set_jump_speed(8)
    controller:jump()
    local highest = ground
    for _ = 1, 60 do
        p:step(1)
        highest = math.max(highest, height(ghost))
    end
    T.ok(highest > ground + 0.3, "jumped from " .. ground .. " up to " .. highest)
    p:step(180)
    T.ok(controller:on_ground(), "landed again")
end)

T.test("btKinematicCharacterController: warp and reset", function()
    local p = P.new(T)
    local controller, ghost = character(p, 1)
    p:step(30)
    controller:warp(vec(10, 5, 10))
    local origin = ghost:get_world_transform():get_origin()
    T.vec3(origin, 10, 5, 10, 1e-3)
    controller:reset(p.world)
    controller:apply_impulse(vec(0, 0, 0))
end)

T.test("btKinematicCharacterController: a velocity for a time interval", function()
    local p = P.new(T)
    local controller, ghost = character(p, 1)
    p:step(60)
    local z0 = ghost:get_world_transform():get_origin():z()
    controller:set_velocity_for_time_interval(vec(0, 0, 2), 1)
    p:step(60)
    local z1 = ghost:get_world_transform():get_origin():z()
    T.ok(z1 - z0 > 1, "moved " .. (z1 - z0) .. " in a second at 2 m/s")
end)

T.run()
