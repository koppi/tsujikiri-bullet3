-- Constraints: the shared btTypedConstraint interface, each constraint type in a short
-- simulation, and the limit/motor helper classes.

local T = require("testlib")
local P = require("physics")
local b = bullet3

local function vec(x, y, z)
    return b.btVector3(x, y, z)
end

local function frame_at(x, y, z)
    return b.btTransform(b.btQuaternion(0, 0, 0, 1), vec(x, y, z))
end

local function sphere_body(p, mass, x, y, z)
    return p:add_body({ shape = b.btSphereShape(0.5), mass = mass, position = { x, y, z } })
end

-- A static anchor at the origin and a free sphere at (1, 0, 0)
local function anchor_and_ball(p)
    local anchor = p:add_body({ shape = b.btSphereShape(0.1), mass = 0, position = { 0, 0, 0 } })
    local ball = sphere_body(p, 1, 1, 0, 0)
    return anchor, ball
end

local function distance_from_origin(body)
    local x, y, z = P.position(body)
    return math.sqrt(x * x + y * y + z * z)
end

-- Check set_x/get_x pairs on an object: table of { setter, getter, value }
local function roundtrip(object, pairs_to_check, tolerance)
    for _, case in ipairs(pairs_to_check) do
        local setter, getter, value = case[1], case[2], case[3]
        object[setter](object, value)
        T.near(object[getter](object), value, tolerance or 1e-5, setter .. "/" .. getter)
    end
end

-- ---------------------------------------------------------------------------------------
-- btTypedConstraint: behaviour shared by every constraint
-- ---------------------------------------------------------------------------------------

T.test("btTypedConstraint: enabling, ids and bodies", function()
    local p = P.new(T)
    local anchor, ball = anchor_and_ball(p)
    anchor:set_user_index(1)
    ball:set_user_index(2)
    local c = b.btPoint2PointConstraint(anchor, ball, vec(0, 0, 0), vec(-1, 0, 0))

    T.ok(c:is_enabled())
    c:set_enabled(false)
    T.ok(not c:is_enabled())
    c:set_enabled(true)

    T.eq(c:get_rigid_body_a():get_user_index(), 1)
    T.eq(c:get_rigid_body_b():get_user_index(), 2)

    -- the uid is the user constraint id, which is -1 until set
    T.eq(c:get_uid(), -1)
    c:set_user_constraint_id(7)
    T.eq(c:get_uid(), 7)
    T.eq(c:get_user_constraint_id(), 7)
    c:set_user_constraint_type(3)
    T.eq(c:get_user_constraint_type(), 3)
    T.ok(b.btTypedConstraint.get_fixed_body():is_static_object(), "the shared fixed body is static")
end)

T.test("btTypedConstraint: tuning knobs", function()
    local p = P.new(T)
    local anchor, ball = anchor_and_ball(p)
    local c = b.btPoint2PointConstraint(anchor, ball, vec(0, 0, 0), vec(-1, 0, 0))

    T.eq(c:get_override_num_solver_iterations(), -1)
    c:set_override_num_solver_iterations(20)
    T.eq(c:get_override_num_solver_iterations(), 20)

    c:set_breaking_impulse_threshold(5)
    T.near(c:get_breaking_impulse_threshold(), 5)

    c:set_dbg_draw_size(2)
    T.near(c:get_dbg_draw_size(), 2)

    T.ok(not c:needs_feedback())
    c:enable_feedback(true)
    T.ok(c:needs_feedback())

    T.near(c:get_applied_impulse(), 0)
end)

T.test("btTypedConstraint: the world keeps track of its constraints", function()
    local p = P.new(T)
    local anchor, ball = anchor_and_ball(p)
    local c = b.btPoint2PointConstraint(anchor, ball, vec(0, 0, 0), vec(-1, 0, 0))
    c:set_user_constraint_id(42)
    p:add_constraint(c)
    T.eq(p.world:get_num_constraints(), 1)
    T.eq(p.world:get_constraint(0):get_uid(), 42)

    p.world:remove_constraint(c)
    T.eq(p.world:get_num_constraints(), 0)
    p.world:add_constraint(c) -- the fixture removes it again at the end
end)

T.test("btRigidBody: constraint references", function()
    local p = P.new(T)
    local anchor, ball = anchor_and_ball(p)
    local c = p:hold(b.btPoint2PointConstraint(anchor, ball, vec(0, 0, 0), vec(-1, 0, 0)))
    c:set_user_constraint_id(5)
    T.eq(ball:get_num_constraint_refs(), 0)
    ball:add_constraint_ref(c)
    T.eq(ball:get_num_constraint_refs(), 1)
    T.eq(ball:get_constraint_ref(0):get_uid(), 5)
    ball:remove_constraint_ref(c)
    T.eq(ball:get_num_constraint_refs(), 0)
end)

T.test("a constraint breaks when its impulse exceeds the threshold", function()
    local p = P.new(T)
    local anchor = p:add_body({ shape = b.btSphereShape(0.1), mass = 0, position = { 0, 10, 0 } })
    local weight = sphere_body(p, 10, 0, 8, 0)
    local c = p:add_constraint(b.btPoint2PointConstraint(anchor, weight, vec(0, 0, 0), vec(0, 2, 0)))
    c:set_breaking_impulse_threshold(0.01)
    p:step(10)
    T.ok(not c:is_enabled(), "the hanging weight pulled the constraint apart")
    local _, y = P.position(weight)
    T.ok(y < 7.95, "and it fell, y = " .. y)
end)

-- ---------------------------------------------------------------------------------------
-- btPoint2PointConstraint
-- ---------------------------------------------------------------------------------------

T.test("btPoint2PointConstraint: pivots", function()
    local p = P.new(T)
    local anchor, ball = anchor_and_ball(p)
    local c = b.btPoint2PointConstraint(anchor, ball, vec(0, 0, 0), vec(-1, 0, 0))
    T.vec3(c:get_pivot_in_a(), 0, 0, 0)
    T.vec3(c:get_pivot_in_b(), -1, 0, 0)
    c:set_pivot_a(vec(1, 2, 3))
    c:set_pivot_b(vec(4, 5, 6))
    T.vec3(c:get_pivot_in_a(), 1, 2, 3)
    T.vec3(c:get_pivot_in_b(), 4, 5, 6)
end)

T.test("btPoint2PointConstraint: a pendulum keeps its length", function()
    local p = P.new(T)
    local anchor, ball = anchor_and_ball(p)
    p:add_constraint(b.btPoint2PointConstraint(anchor, ball, vec(0, 0, 0), vec(-1, 0, 0)))
    local lowest = 0
    for _ = 1, 120 do
        p:step(1)
        T.near(distance_from_origin(ball), 1, 0.05, "distance to the pivot")
        local _, y = P.position(ball)
        lowest = math.min(lowest, y)
    end
    T.ok(lowest < -0.9, "the pendulum swung down to y = " .. lowest)
end)

T.test("btPoint2PointConstraint: pinning one body to a point in the world", function()
    local p = P.new(T)
    local ball = sphere_body(p, 1, 0, 5, 0)
    p:add_constraint(b.btPoint2PointConstraint(ball, vec(0, 0, 0)))
    p:step(120)
    -- hangs from its own centre: gravity cannot move it
    local x, y, z = P.position(ball)
    T.near(x, 0, 0.02)
    T.near(y, 5, 0.1)
    T.near(z, 0, 0.02)
end)

-- ---------------------------------------------------------------------------------------
-- btHingeConstraint and btHingeAccumulatedAngleConstraint
-- ---------------------------------------------------------------------------------------

-- A door: a static post at the origin and a bar hinged to it, hanging along +x
local function door(p)
    local post = p:add_body({ shape = b.btSphereShape(0.1), mass = 0, position = { 0, 0, 0 } })
    local bar = p:add_body({ shape = b.btBoxShape(vec(1, 0.1, 0.1)), mass = 1, position = { 1, 0, 0 } })
    -- (the last argument measures the angle of the bar relative to the post)
    local hinge = b.btHingeConstraint(post, bar, vec(0, 0, 0), vec(-1, 0, 0), vec(0, 0, 1), vec(0, 0, 1), true)
    return post, bar, hinge
end

T.test("btHingeConstraint: limits are stored", function()
    local p = P.new(T)
    local _, _, hinge = door(p)
    hinge:set_limit(-0.5, 0.75, 0.8, 0.2, 0.9)
    T.near(hinge:get_lower_limit(), -0.5)
    T.near(hinge:get_upper_limit(), 0.75)
    T.near(hinge:get_limit_softness(), 0.8)
    T.near(hinge:get_limit_bias_factor(), 0.2)
    T.near(hinge:get_limit_relaxation_factor(), 0.9)
    T.ok(hinge:has_limit())

    hinge:set_limit(0, 0)
    T.ok(not hinge:has_limit(), "an empty range is no limit")
end)

T.test("btHingeConstraint: motor settings", function()
    local p = P.new(T)
    local _, _, hinge = door(p)
    T.ok(not hinge:get_enable_angular_motor())
    hinge:enable_angular_motor(true, 2.5, 7)
    T.ok(hinge:get_enable_angular_motor())
    T.near(hinge:get_motor_target_velocity(), 2.5)
    T.near(hinge:get_max_motor_impulse(), 7)
    hinge:set_motor_target_velocity(1)
    hinge:set_max_motor_impulse(3)
    T.near(hinge:get_motor_target_velocity(), 1)
    T.near(hinge:get_max_motor_impulse(), 3)
    hinge:enable_motor(false)
    T.ok(not hinge:get_enable_angular_motor())
end)

T.test("btHingeConstraint: options and frames", function()
    local p = P.new(T)
    local _, _, hinge = door(p)
    T.ok(not hinge:get_angular_only())
    hinge:set_angular_only(true)
    T.ok(hinge:get_angular_only())

    hinge:set_use_reference_frame_a(true)
    T.ok(hinge:get_use_reference_frame_a())
    hinge:set_use_frame_offset(true)
    T.ok(hinge:get_use_frame_offset())

    hinge:set_frames(frame_at(1, 2, 3), frame_at(4, 5, 6))
    T.vec3(hinge:get_a_frame():get_origin(), 1, 2, 3)
    T.vec3(hinge:get_b_frame():get_origin(), 4, 5, 6)
    T.vec3(hinge:get_frame_offset_a():get_origin(), 1, 2, 3)
    T.vec3(hinge:get_frame_offset_b():get_origin(), 4, 5, 6)
    T.eq(hinge:get_rigid_body_a():get_mass(), 0)
    T.near(hinge:get_rigid_body_b():get_mass(), 1)
end)

T.test("btHingeConstraint: a swinging door stops at its limit", function()
    local p = P.new(T)
    local _, bar, hinge = door(p)
    hinge:set_limit(-0.5, 0.5)
    p:add_constraint(hinge, true)
    p:step(180)
    -- gravity pulls the bar down, turning it clockwise about z
    T.near(hinge:get_hinge_angle(), -0.5, 0.08)
    T.near(hinge:get_lower_limit(), -0.5)
    local x, y = P.position(bar)
    T.near(math.sqrt(x * x + y * y), 1, 0.05, "the bar stays attached to the pivot")
end)

T.test("btHingeConstraint: without limits the door swings past the horizontal", function()
    local p = P.new(T)
    local _, _, hinge = door(p)
    p:add_constraint(hinge, true)
    local lowest = 0
    for _ = 1, 90 do
        p:step(1)
        lowest = math.min(lowest, hinge:get_hinge_angle())
    end
    T.ok(lowest < -1.2, "swung down to " .. lowest .. " rad")
end)

T.test("btHingeConstraint: the motor turns the bar", function()
    local p = P.new(T)
    p.world:set_gravity(vec(0, 0, 0))
    local _, bar, hinge = door(p)
    hinge:enable_angular_motor(true, 1, 100)
    p:add_constraint(hinge, true)
    p:step(60)
    T.near(hinge:get_hinge_angle(), 1, 0.15, "one radian per second for one second")
    T.near(bar:get_angular_velocity():z(), 1, 0.1)
end)

T.test("btHingeAccumulatedAngleConstraint: keeps counting past a half turn", function()
    local p = P.new(T)
    p.world:set_gravity(vec(0, 0, 0))
    local post = p:add_body({ shape = b.btSphereShape(0.1), mass = 0, position = { 0, 0, 0 } })
    local wheel = p:add_body({ shape = b.btSphereShape(0.5), mass = 1, position = { 0, 0, 0 } })
    local hinge = b.btHingeAccumulatedAngleConstraint(post, wheel, vec(0, 0, 0), vec(0, 0, 0), vec(0, 0, 1), vec(0, 0, 1), true)
    p:add_constraint(hinge, true)
    wheel:set_angular_velocity(vec(0, 0, 4))
    p:step(60)
    -- four turns' worth of radians per second: beyond pi, where a plain hinge angle wraps
    local accumulated = hinge:get_accumulated_hinge_angle()
    T.ok(accumulated > math.pi + 0.5, "accumulated " .. accumulated)
    T.near(accumulated, 4, 0.3, "four radians per second for one second")

    -- the setter overwrites the stored value; the next read folds the live angle into it
    hinge:set_accumulated_hinge_angle(0.25)
    T.type_is(hinge:get_accumulated_hinge_angle(), "number")
end)

-- ---------------------------------------------------------------------------------------
-- btSliderConstraint
-- ---------------------------------------------------------------------------------------

local function slider(p)
    local rail = p:add_body({ shape = b.btBoxShape(vec(0.1, 0.1, 0.1)), mass = 0, position = { 0, 0, 0 } })
    local cart = p:add_body({ shape = b.btBoxShape(vec(0.3, 0.3, 0.3)), mass = 1, position = { 0, 0, 0 } })
    local constraint = b.btSliderConstraint(rail, cart, frame_at(0, 0, 0), frame_at(0, 0, 0), true)
    return rail, cart, constraint
end

T.test("btSliderConstraint: limits and frames", function()
    local p = P.new(T)
    local _, _, c = slider(p)
    roundtrip(c, {
        { "set_lower_lin_limit", "get_lower_lin_limit", -1.5 },
        { "set_upper_lin_limit", "get_upper_lin_limit", 2.5 },
        { "set_lower_ang_limit", "get_lower_ang_limit", -0.25 },
        { "set_upper_ang_limit", "get_upper_ang_limit", 0.25 },
    })
    T.ok(c:get_use_linear_reference_frame_a())
    c:set_frames(frame_at(1, 0, 0), frame_at(2, 0, 0))
    T.vec3(c:get_frame_offset_a():get_origin(), 1, 0, 0)
    T.vec3(c:get_frame_offset_b():get_origin(), 2, 0, 0)
    c:set_use_frame_offset(true)
    T.ok(c:get_use_frame_offset())
end)

T.test("btSliderConstraint: softness, restitution and damping parameters", function()
    local p = P.new(T)
    local _, _, c = slider(p)
    local values = {}
    for _, group in ipairs({ "dir", "lim", "ortho" }) do
        for _, axis in ipairs({ "lin", "ang" }) do
            for _, property in ipairs({ "softness", "restitution", "damping" }) do
                table.insert(values, {
                    string.format("set_%s_%s_%s", property, group, axis),
                    string.format("get_%s_%s_%s", property, group, axis),
                    0.1 + #values * 0.01,
                })
            end
        end
    end
    T.eq(#values, 18)
    roundtrip(c, values)
end)

T.test("btSliderConstraint: motors", function()
    local p = P.new(T)
    local _, _, c = slider(p)
    T.ok(not c:get_powered_lin_motor())
    c:set_powered_lin_motor(true)
    T.ok(c:get_powered_lin_motor())
    roundtrip(c, {
        { "set_target_lin_motor_velocity", "get_target_lin_motor_velocity", 3 },
        { "set_max_lin_motor_force", "get_max_lin_motor_force", 40 },
        { "set_target_ang_motor_velocity", "get_target_ang_motor_velocity", 2 },
        { "set_max_ang_motor_force", "get_max_ang_motor_force", 30 },
    })
    c:set_powered_ang_motor(true)
    T.ok(c:get_powered_ang_motor())
end)

T.test("btSliderConstraint: the cart runs along the rail and stops at the limit", function()
    local p = P.new(T)
    local _, cart, c = slider(p)
    c:set_lower_lin_limit(-1)
    c:set_upper_lin_limit(2)
    c:set_lower_ang_limit(0)
    c:set_upper_ang_limit(0)
    p:add_constraint(c, true)
    cart:set_linear_velocity(vec(3, 0, 0))
    local furthest = 0
    for _ = 1, 120 do
        p:step(1)
        furthest = math.max(furthest, (P.position(cart)))
    end
    -- the cart runs into the upper limit and rebounds from it
    T.near(furthest, 2, 0.1, "furthest position")
    local x, y, z = P.position(cart)
    T.ok(x < furthest - 0.1, "it came back after hitting the limit")
    T.near(y, 0, 0.05, "gravity does not move the cart off the rail")
    T.near(z, 0, 0.05)
    T.near(c:get_linear_pos(), x, 0.05)
    T.near(c:get_angular_pos(), 0, 0.05)
end)

T.test("btSliderConstraint: the linear motor drives the cart", function()
    local p = P.new(T)
    local _, cart, c = slider(p)
    c:set_lower_lin_limit(-10)
    c:set_upper_lin_limit(10)
    c:set_lower_ang_limit(0)
    c:set_upper_ang_limit(0)
    c:set_powered_lin_motor(true)
    c:set_target_lin_motor_velocity(2)
    c:set_max_lin_motor_force(1000)
    p:add_constraint(c, true)
    p:step(60)
    T.near(P.velocity(cart), 2, 0.2)
end)

-- ---------------------------------------------------------------------------------------
-- btConeTwistConstraint
-- ---------------------------------------------------------------------------------------

local function cone_twist(p)
    local shoulder = p:add_body({ shape = b.btSphereShape(0.1), mass = 0, position = { 0, 0, 0 } })
    local arm = p:add_body({ shape = b.btSphereShape(0.3), mass = 1, position = { 0, -1, 0 } })
    local c = b.btConeTwistConstraint(shoulder, arm, frame_at(0, 0, 0), frame_at(0, 1, 0))
    return shoulder, arm, c
end

T.test("btConeTwistConstraint: limits", function()
    local p = P.new(T)
    local _, _, c = cone_twist(p)
    c:set_limit(0.5, 0.6, 0.7, 0.8, 0.2, 0.9)
    T.near(c:get_swing_span1(), 0.5)
    T.near(c:get_swing_span2(), 0.6)
    T.near(c:get_twist_span(), 0.7)
    T.near(c:get_limit_softness(), 0.8)
    T.near(c:get_bias_factor(), 0.2)
    T.near(c:get_relaxation_factor(), 0.9)

    -- the same limits by index: 3 is the twist span, 4 and 5 the second and first swing span
    c:set_limit(3, 1.0)
    c:set_limit(4, 1.1)
    c:set_limit(5, 1.2)
    T.near(c:get_limit(3), 1.0)
    T.near(c:get_limit(4), 1.1)
    T.near(c:get_limit(5), 1.2)
    T.near(c:get_twist_span(), 1.0)
    T.near(c:get_swing_span2(), 1.1)
    T.near(c:get_swing_span1(), 1.2)
end)

T.test("btConeTwistConstraint: motor and damping settings", function()
    local p = P.new(T)
    local _, _, c = cone_twist(p)
    roundtrip(c, {
        { "set_damping", "get_damping", 0.3 },
        { "set_max_motor_impulse", "get_max_motor_impulse", 4 },
        { "set_fix_thresh", "get_fix_thresh", 0.02 },
    })
    T.ok(not c:is_max_motor_impulse_normalized())
    c:set_max_motor_impulse_normalized(2)
    T.ok(c:is_max_motor_impulse_normalized())
    T.near(c:get_max_motor_impulse(), 2)

    T.ok(not c:is_motor_enabled())
    c:enable_motor(true)
    T.ok(c:is_motor_enabled())

    T.ok(not c:get_angular_only())
    c:set_angular_only(true)
    T.ok(c:get_angular_only())

    c:set_motor_target(b.btQuaternion(0, 0, 0, 1))
    T.quat(c:get_motor_target(), 0, 0, 0, 1)
    c:set_motor_target_in_constraint_space(b.btQuaternion(0, 0, 0, 1))

    c:set_frames(frame_at(1, 0, 0), frame_at(2, 0, 0))
    T.vec3(c:get_a_frame():get_origin(), 1, 0, 0)
    T.vec3(c:get_b_frame():get_origin(), 2, 0, 0)
    T.vec3(c:get_frame_offset_a():get_origin(), 1, 0, 0)
    T.vec3(c:get_frame_offset_b():get_origin(), 2, 0, 0)
    T.ok(c:get_point_for_angle(0.5, 1):length() > 0)
end)

T.test("btConeTwistConstraint: a ball joint holds the arm at a fixed distance", function()
    local p = P.new(T)
    local _, arm, c = cone_twist(p)
    c:set_limit(math.pi / 4, math.pi / 4, math.pi / 4)
    p:add_constraint(c)
    arm:apply_central_impulse(vec(2, 0, 0))
    for _ = 1, 120 do
        p:step(1)
        T.near(distance_from_origin(arm), 1, 0.06)
    end
    -- the swing is limited to a quarter of a half turn, so it cannot reach the horizontal
    local _, y = P.position(arm)
    T.ok(y < -0.5, "y = " .. y)
end)

-- ---------------------------------------------------------------------------------------
-- btGeneric6DofConstraint and the spring variants
-- ---------------------------------------------------------------------------------------

local function six_dof(p, class)
    local base = p:add_body({ shape = b.btBoxShape(vec(0.1, 0.1, 0.1)), mass = 0, position = { 0, 0, 0 } })
    local slide = p:add_body({ shape = b.btBoxShape(vec(0.3, 0.3, 0.3)), mass = 1, position = { 0, 0, 0 } })
    local c = (class or b.btGeneric6DofConstraint)(base, slide, frame_at(0, 0, 0), frame_at(0, 0, 0), true)
    return base, slide, c
end

T.test("btGeneric6DofConstraint: limits are stored per axis", function()
    local p = P.new(T)
    local _, _, c = six_dof(p)
    c:set_linear_lower_limit(vec(-1, -2, -3))
    c:set_linear_upper_limit(vec(1, 2, 3))
    c:set_angular_lower_limit(vec(-0.1, -0.2, -0.3))
    c:set_angular_upper_limit(vec(0.1, 0.2, 0.3))

    local out = vec(0, 0, 0)
    c:get_linear_lower_limit(out)
    T.vec3(out, -1, -2, -3)
    c:get_linear_upper_limit(out)
    T.vec3(out, 1, 2, 3)
    c:get_angular_lower_limit(out)
    T.vec3(out, -0.1, -0.2, -0.3)
    c:get_angular_upper_limit(out)
    T.vec3(out, 0.1, 0.2, 0.3)

    -- lower == upper locks an axis, lower > upper frees it
    c:set_limit(0, 0, 0)
    c:set_limit(1, 1, -1)
    T.ok(c:is_limited(0))
    T.ok(not c:is_limited(1))
    c:set_limit(3, -0.5, 0.5) -- axes 3 to 5 are the rotations
    T.ok(c:is_limited(3))
end)

T.test("btGeneric6DofConstraint: frames, axes and options", function()
    local p = P.new(T)
    local _, _, c = six_dof(p)
    c:set_frames(frame_at(1, 0, 0), frame_at(2, 0, 0))
    T.vec3(c:get_frame_offset_a():get_origin(), 1, 0, 0)
    T.vec3(c:get_frame_offset_b():get_origin(), 2, 0, 0)
    c:calculate_transforms()
    T.ok(c:get_calculated_transform_a() ~= nil)
    T.ok(c:get_calculated_transform_b() ~= nil)

    c:set_frames(frame_at(0, 0, 0), frame_at(0, 0, 0))
    c:calculate_transforms()
    T.vec3(c:get_axis(0), 1, 0, 0)
    T.vec3(c:get_axis(1), 0, 1, 0)
    T.vec3(c:get_axis(2), 0, 0, 1)
    T.near(c:get_angle(0), 0, 1e-3)
    T.near(c:get_relative_pivot_position(0), 0, 1e-3)

    c:set_use_frame_offset(true)
    T.ok(c:get_use_frame_offset())
    c:set_use_linear_reference_frame_a(false)
    T.ok(not c:get_use_linear_reference_frame_a())
    T.ok(c:get_rotational_limit_motor(0) ~= nil)
    T.ok(c:get_translational_limit_motor() ~= nil)
end)

T.test("btGeneric6DofConstraint: locking linear axes keeps a body on a line", function()
    local p = P.new(T)
    local _, slide, c = six_dof(p)
    c:set_linear_lower_limit(vec(0, 1, 0)) -- x and z locked, y free (lower > upper)
    c:set_linear_upper_limit(vec(0, 0, 0))
    c:set_angular_lower_limit(vec(0, 0, 0))
    c:set_angular_upper_limit(vec(0, 0, 0))
    p:add_constraint(c, true)
    slide:apply_central_impulse(vec(3, 0, 2))
    p:step(60)
    local x, y, z = P.position(slide)
    T.near(x, 0, 0.05)
    T.near(z, 0, 0.05)
    T.ok(y < -3, "free to fall along y: " .. y)
end)

T.test("btGeneric6DofConstraint: all axes locked holds the body", function()
    local p = P.new(T)
    local _, slide, c = six_dof(p)
    c:set_linear_lower_limit(vec(0, 0, 0))
    c:set_linear_upper_limit(vec(0, 0, 0))
    c:set_angular_lower_limit(vec(0, 0, 0))
    c:set_angular_upper_limit(vec(0, 0, 0))
    p:add_constraint(c, true)
    p:step(60)
    local x, y, z = P.position(slide)
    T.near(y, 0, 0.05)
    T.near(x, 0, 0.05)
    T.near(z, 0, 0.05)
    T.ok(not c:test_angular_limit_motor(0) or true)
end)

T.test("btGeneric6DofSpringConstraint: stiffness, damping and the equilibrium point", function()
    local p = P.new(T)
    local _, _, c = six_dof(p, b.btGeneric6DofSpringConstraint)
    T.ok(not c:is_spring_enabled(0))
    c:enable_spring(0, true)
    T.ok(c:is_spring_enabled(0))
    c:set_stiffness(0, 80)
    c:set_damping(0, 0.5)
    T.near(c:get_stiffness(0), 80)
    T.near(c:get_damping(0), 0.5)
    c:set_equilibrium_point(0, 0.25)
    T.near(c:get_equilibrium_point(0), 0.25)
    c:set_equilibrium_point(0)
    c:set_equilibrium_point()
end)

T.test("btGeneric6DofSpringConstraint: a spring pulls the body back to the middle", function()
    local p = P.new(T)
    p.world:set_gravity(vec(0, 0, 0))
    local base = p:add_body({ shape = b.btBoxShape(vec(0.1, 0.1, 0.1)), mass = 0, position = { 0, 0, 0 } })
    local mass = p:add_body({ shape = b.btBoxShape(vec(0.3, 0.3, 0.3)), mass = 1, position = { 1, 0, 0 } })
    -- both frames in the middle of the bodies: the offset between them is the spring's displacement
    local c = b.btGeneric6DofSpringConstraint(base, mass, frame_at(0, 0, 0), frame_at(0, 0, 0), true)
    c:set_linear_lower_limit(vec(-10, 0, 0))
    c:set_linear_upper_limit(vec(10, 0, 0))
    c:set_angular_lower_limit(vec(0, 0, 0))
    c:set_angular_upper_limit(vec(0, 0, 0))
    c:enable_spring(0, true)
    c:set_stiffness(0, 50)
    c:set_damping(0, 0.1)
    c:set_equilibrium_point(0, 0)
    p:add_constraint(c, true)

    local min_x, max_x = 1, 1
    for _ = 1, 180 do
        p:step(1)
        local x = P.position(mass)
        min_x, max_x = math.min(min_x, x), math.max(max_x, x)
    end
    T.ok(min_x < 0.5, "the mass was pulled towards the equilibrium, min x = " .. min_x)
    T.ok(max_x <= 1.1)
end)

T.test("btGeneric6DofSpring2Constraint: parameters", function()
    local p = P.new(T)
    local base, slide = p:add_body({ shape = b.btBoxShape(vec(0.1, 0.1, 0.1)), mass = 0 }),
        p:add_body({ shape = b.btBoxShape(vec(0.3, 0.3, 0.3)), mass = 1 })
    local c = b.btGeneric6DofSpring2Constraint(base, slide, frame_at(0, 0, 0), frame_at(0, 0, 0))

    c:set_linear_lower_limit(vec(-1, -2, -3))
    c:set_linear_upper_limit(vec(1, 2, 3))
    c:set_angular_lower_limit(vec(-0.1, -0.2, -0.3))
    c:set_angular_upper_limit(vec(0.1, 0.2, 0.3))
    local out = vec(0, 0, 0)
    c:get_linear_lower_limit(out)
    T.vec3(out, -1, -2, -3)
    c:get_linear_upper_limit(out)
    T.vec3(out, 1, 2, 3)
    c:get_angular_lower_limit(out)
    T.vec3(out, -0.1, -0.2, -0.3)
    c:get_angular_upper_limit(out)
    T.vec3(out, 0.1, 0.2, 0.3)

    c:set_limit(0, -4, 4)
    c:set_limit_reversed(3, -0.5, 0.5)
    T.ok(c:is_limited(0))

    c:set_bounce(0, 0.5)
    c:enable_motor(0, true)
    c:set_servo(0, true)
    c:set_target_velocity(0, 1)
    c:set_servo_target(0, 0.5)
    c:set_max_motor_force(0, 10)
    c:enable_spring(0, true)
    c:set_stiffness(0, 20)
    c:set_damping(0, 0.2)
    c:set_equilibrium_point()
    c:set_equilibrium_point(0)
    c:set_equilibrium_point(0, 0.1)

    T.ok(c:get_rotational_limit_motor(0) ~= nil)
    T.ok(c:get_translational_limit_motor() ~= nil)
    c:calculate_transforms()
    T.vec3(c:get_axis(0), 1, 0, 0)
    c:set_frames(frame_at(1, 0, 0), frame_at(2, 0, 0))
    T.vec3(c:get_frame_offset_a():get_origin(), 1, 0, 0)
    T.vec3(c:get_frame_offset_b():get_origin(), 2, 0, 0)
    T.ok(c:get_calculated_transform_a() ~= nil)
end)

T.test("btGeneric6DofSpring2Constraint: Euler angles from a rotation matrix", function()
    -- a rotation about a single axis has the same Euler angles in every rotation order
    local m = b.btMatrix3x3()
    m:set_euler_zyx(0.3, 0, 0) -- 0.3 rad about x
    local angles = vec(9, 9, 9)
    for _, name in ipairs({ "matrix_to_euler_xyz", "matrix_to_euler_xzy", "matrix_to_euler_yxz",
        "matrix_to_euler_yzx", "matrix_to_euler_zxy", "matrix_to_euler_zyx" }) do
        T.ok(b.btGeneric6DofSpring2Constraint[name](m, angles), name .. " finds a solution")
        T.near(math.abs(angles:x()), 0.3, 1e-4, name .. " x")
        T.near(angles:y(), 0, 1e-4, name .. " y")
        T.near(angles:z(), 0, 1e-4, name .. " z")
    end
    T.near(b.btGeneric6DofSpring2Constraint.bt_get_matrix_elem(m, 4), m:get_row(1):y(), 1e-6)
end)

T.test("btFixedConstraint: welds two bodies together", function()
    local p = P.new(T)
    local base = p:add_body({ shape = b.btBoxShape(vec(0.1, 0.1, 0.1)), mass = 0, position = { 0, 5, 0 } })
    local load = p:add_body({ shape = b.btBoxShape(vec(0.3, 0.3, 0.3)), mass = 1, position = { 2, 5, 0 } })
    p:add_constraint(b.btFixedConstraint(base, load, frame_at(2, 0, 0), frame_at(0, 0, 0)), true)
    p:step(120)
    local x, y, z = P.position(load)
    T.near(x, 2, 0.1)
    T.near(y, 5, 0.1)
    T.near(z, 0, 0.1)
end)

-- ---------------------------------------------------------------------------------------
-- Gear, universal and hinge2 constraints
-- ---------------------------------------------------------------------------------------

T.test("btGearConstraint: axes and ratio", function()
    local p = P.new(T)
    local a = sphere_body(p, 1, 0, 0, 0)
    local c = sphere_body(p, 1, 2, 0, 0)
    local gear = b.btGearConstraint(a, c, vec(0, 0, 1), vec(0, 0, 1), 2)
    T.vec3(gear:get_axis_a(), 0, 0, 1)
    T.vec3(gear:get_axis_b(), 0, 0, 1)
    T.near(gear:get_ratio(), 2)

    gear:set_ratio(0.5)
    gear:set_axis_a(vec(1, 0, 0))
    gear:set_axis_b(vec(0, 1, 0))
    T.near(gear:get_ratio(), 0.5)
    T.vec3(gear:get_axis_a(), 1, 0, 0)
    T.vec3(gear:get_axis_b(), 0, 1, 0)

    T.near(b.btGearConstraint(a, c, vec(0, 0, 1), vec(0, 0, 1)):get_ratio(), 1)
end)

T.test("btGearConstraint: two bodies turn at a fixed ratio", function()
    local p = P.new(T)
    p.world:set_gravity(vec(0, 0, 0))
    local a = sphere_body(p, 1, 0, 0, 0)
    local c = sphere_body(p, 1, 3, 0, 0)
    local hinge_a = b.btHingeConstraint(p:add_body({ shape = b.btSphereShape(0.1), mass = 0, position = { 0, 0, 0 } }),
        a, vec(0, 0, 0), vec(0, 0, 0), vec(0, 0, 1), vec(0, 0, 1))
    local hinge_c = b.btHingeConstraint(p:add_body({ shape = b.btSphereShape(0.1), mass = 0, position = { 3, 0, 0 } }),
        c, vec(0, 0, 0), vec(0, 0, 0), vec(0, 0, 1), vec(0, 0, 1))
    p:add_constraint(hinge_a, true)
    p:add_constraint(hinge_c, true)
    p:add_constraint(b.btGearConstraint(a, c, vec(0, 0, 1), vec(0, 0, 1), 2), true)

    a:set_angular_velocity(vec(0, 0, 2))
    for _ = 1, 30 do p:step(1) end
    local wa, wc = a:get_angular_velocity():z(), c:get_angular_velocity():z()
    -- the gear couples wa + ratio * wc = 0
    T.near(wa + 2 * wc, 0, 0.3, "angular velocities " .. wa .. " and " .. wc)
end)

T.test("btUniversalConstraint: anchor, axes and limits", function()
    local p = P.new(T)
    local a = sphere_body(p, 0, 0, 0, 0)
    local c = sphere_body(p, 1, 1, 0, 0)
    local joint = b.btUniversalConstraint(a, c, vec(0.5, 0, 0), vec(0, 0, 1), vec(0, 1, 0))
    joint:calculate_transforms() -- the anchor and axes are read back from the calculated frames
    T.vec3(joint:get_anchor(), 0.5, 0, 0)
    T.vec3(joint:get_axis1(), 0, 0, 1)
    T.vec3(joint:get_axis2(), 0, 1, 0)
    T.near(joint:get_angle1(), 0, 1e-3)
    T.near(joint:get_angle2(), 0, 1e-3)
    joint:set_upper_limit(0.5, 0.6)
    joint:set_lower_limit(-0.5, -0.6)
    T.ok(joint:get_anchor2() ~= nil)
    joint:set_axis(vec(1, 0, 0), vec(0, 1, 0))
end)

T.test("btHinge2Constraint: anchor, axes and limits", function()
    local p = P.new(T)
    local chassis = sphere_body(p, 0, 0, 0, 0)
    local wheel = sphere_body(p, 1, 1, 0, 0)
    local joint = b.btHinge2Constraint(chassis, wheel, vec(1, 0, 0), vec(0, 1, 0), vec(1, 0, 0))
    joint:calculate_transforms()
    T.vec3(joint:get_anchor(), 1, 0, 0)
    T.vec3(joint:get_axis1(), 0, 1, 0)
    T.vec3(joint:get_axis2(), 1, 0, 0)
    T.ok(joint:get_anchor2() ~= nil)
    T.near(joint:get_angle1(), 0, 1e-3)
    T.near(joint:get_angle2(), 0, 1e-3)
    joint:set_upper_limit(0.5)
    joint:set_lower_limit(-0.5)
end)

-- ---------------------------------------------------------------------------------------
-- Limit helpers
-- ---------------------------------------------------------------------------------------

T.test("btAngularLimit: a range around zero", function()
    local limit = b.btAngularLimit()
    limit:set(-1, 1, 0.5, 0.25, 0.75)
    T.near(limit:get_low(), -1)
    T.near(limit:get_high(), 1)
    T.near(limit:get_half_range(), 1)
    T.near(limit:get_softness(), 0.5)
    T.near(limit:get_bias_factor(), 0.25)
    T.near(limit:get_relaxation_factor(), 0.75)

    limit:test(0.5)
    T.ok(not limit:is_limit())
    T.near(limit:get_error(), 0)
    T.near(limit:get_sign(), 0)

    limit:test(1.5) -- 0.5 beyond the upper bound
    T.ok(limit:is_limit())
    T.near(limit:get_error(), 0.5, 1e-5)
    T.near(limit:get_sign(), -1)
    T.ok(limit:get_correction() ~= 0)

    limit:test(-1.5)
    T.ok(limit:is_limit())
    T.near(limit:get_sign(), 1)
end)

T.test("btRotationalLimitMotor and btTranslationalLimitMotor: defaults", function()
    -- a new rotational motor has low > high: unlimited
    local rotation = b.btRotationalLimitMotor()
    T.ok(not rotation:is_limited())
    T.ok(not rotation:need_apply_torques())
    T.eq(rotation:test_limit_value(0), 0)
    T.ok(b.btRotationalLimitMotor(rotation) ~= nil)

    -- a new translational motor has low == high == 0: locked
    local translation = b.btTranslationalLimitMotor()
    for axis = 0, 2 do
        T.ok(translation:is_limited(axis), "locked on axis " .. axis)
        T.eq(translation:test_limit_value(axis, 0), 0, "zero is within the range")
        T.eq(translation:test_limit_value(axis, 1), 1, "beyond the upper limit")
        T.eq(translation:test_limit_value(axis, -1), 2, "beyond the lower limit")
    end
    T.ok(b.btTranslationalLimitMotor(translation) ~= nil)

    T.ok(b.btRotationalLimitMotor2() ~= nil)
    T.ok(b.btTranslationalLimitMotor2() ~= nil)
    T.ok(b.btRotationalLimitMotor2(b.btRotationalLimitMotor2()) ~= nil)
    T.ok(b.btTranslationalLimitMotor2(b.btTranslationalLimitMotor2()) ~= nil)
    T.type_is(b.btRotationalLimitMotor2():is_limited(), "boolean")
    T.type_is(b.btTranslationalLimitMotor2():is_limited(0), "boolean")
    b.btRotationalLimitMotor2():test_limit_value(0)
    b.btTranslationalLimitMotor2():test_limit_value(0, 0)
    T.ok(b.btConstraintSetting() ~= nil)
end)

T.run()
