-- btDiscreteDynamicsWorld simulations plus the collision world, dispatcher, broadphase and
-- solver objects it is built from. Each scenario runs a short simulation and checks the
-- physics against closed-form results.

local T = require("testlib")
local P = require("physics")
local b = bullet3

local DT = 1 / 60
local ISLAND_SLEEPING = 2

local function vec(x, y, z)
    return b.btVector3(x, y, z)
end

local function sphere(radius)
    return b.btSphereShape(radius)
end

local function box(x, y, z)
    return b.btBoxShape(vec(x, y, z))
end

-- ---------------------------------------------------------------------------------------
-- World set-up
-- ---------------------------------------------------------------------------------------

T.test("btDiscreteDynamicsWorld: components and gravity", function()
    local p = P.new(T)
    T.vec3(p.world:get_gravity(), 0, -10, 0)
    p.world:set_gravity(vec(0, -3.71, 0))
    T.vec3(p.world:get_gravity(), 0, -3.71, 0)

    T.ok(p.world:get_dispatcher() ~= nil)
    T.ok(p.world:get_broadphase() ~= nil)
    T.ok(p.world:get_constraint_solver() ~= nil)
    T.ok(p.world:get_pair_cache() ~= nil)
    T.ok(p.world:get_collision_world() ~= nil)
    T.ok(p.world:get_simulation_island_manager() ~= nil)
    T.eq(p.world:get_num_collision_objects(), 0)
    T.eq(p.world:get_num_constraints(), 0)
end)

T.test("btDiscreteDynamicsWorld: bodies are counted as they are added and removed", function()
    local p = P.new(T)
    local body = p:add_body({ shape = sphere(1) })
    T.eq(p.world:get_num_collision_objects(), 1)
    T.ok(body:is_in_world())
    T.ok(body:get_broadphase_proxy() ~= nil)

    local floor = p:add_floor()
    T.eq(p.world:get_num_collision_objects(), 2)

    p.world:remove_rigid_body(body)
    T.eq(p.world:get_num_collision_objects(), 1)
    T.ok(not body:is_in_world())
    -- the fixture removes whatever is still in the world, so put the body back to keep its list right
    p.world:add_rigid_body(body)
    T.ok(floor ~= nil)
end)

T.test("btDiscreteDynamicsWorld: collision objects without a rigid body", function()
    local p = P.new(T)
    local shape = p:hold(sphere(1))
    local object = p:hold(b.btCollisionObject())
    object:set_collision_shape(shape)
    p.world:add_collision_object(object)
    T.eq(p.world:get_num_collision_objects(), 1)
    p.world:remove_collision_object(object)
    T.eq(p.world:get_num_collision_objects(), 0)
end)

-- ---------------------------------------------------------------------------------------
-- Stepping
-- ---------------------------------------------------------------------------------------

T.test("step_simulation: the number of sub-steps follows the elapsed time", function()
    local p = P.new(T)
    T.eq(p.world:step_simulation(DT, 10, DT), 1)
    T.eq(p.world:step_simulation(2 * DT, 10, DT), 2)
    T.eq(p.world:step_simulation(5 * DT, 10, DT), 5)
    -- not enough time for a full step yet: nothing is simulated, the time is remembered
    T.eq(p.world:step_simulation(DT / 4, 10, DT), 0)
    T.eq(p.world:step_simulation(DT * 0.9, 10, DT), 1)
    -- default arguments: one sub-step of 1/60
    T.eq(p.world:step_simulation(DT), 1)
end)

T.test("step_simulation: maxSubSteps limits the work done, not the reported count", function()
    local p = P.new(T)
    p.world:set_gravity(vec(0, -10, 0))
    local body = p:add_body({ shape = sphere(1), position = { 0, 100, 0 } })
    -- ten steps worth of time are reported, but only three are simulated
    T.eq(p.world:step_simulation(10 * DT, 3, DT), 10)
    local _, vy = P.velocity(body)
    T.near(vy, -10 * 3 * DT, 1e-3)
end)

T.test("step_simulation: a variable time step with maxSubSteps = 0", function()
    local p = P.new(T)
    local body = p:add_body({ shape = sphere(1), position = { 0, 100, 0 } })
    T.eq(p.world:step_simulation(0.5, 0), 1)
    local _, vy = P.velocity(body)
    T.near(vy, -5, 1e-3, "one step of 0.5 s under gravity 10")
end)

-- ---------------------------------------------------------------------------------------
-- Free fall and rest
-- ---------------------------------------------------------------------------------------

T.test("a free-falling body follows semi-implicit Euler integration", function()
    local p = P.new(T)
    local body = p:add_body({ shape = sphere(0.5), position = { 0, 10, 0 } })
    p:step(60)
    local x, y, z = P.position(body)
    local _, vy = P.velocity(body)
    -- v = -g t = -10 after 60 steps; y = 10 - g dt^2 * (1 + ... + 60) = 10 - 10 * 1830 / 3600
    T.near(vy, -10, 1e-3)
    T.near(y, 10 - 10 * 1830 / 3600, 1e-3)
    T.near(x, 0, 1e-6)
    T.near(z, 0, 1e-6)
end)

T.test("a body with a different gravity falls accordingly", function()
    local p = P.new(T)
    p.world:set_gravity(vec(0, -3.71, 0))
    local body = p:add_body({ shape = sphere(0.5), position = { 0, 100, 0 } })
    p:step(60)
    local _, vy = P.velocity(body)
    T.near(vy, -3.71, 1e-3)
end)

T.test("a static body stays where it is", function()
    local p = P.new(T)
    local anchor = p:add_body({ shape = sphere(1), mass = 0, position = { 1, 2, 3 } })
    p:step(30)
    local x, y, z = P.position(anchor)
    T.near(x, 1, 1e-6)
    T.near(y, 2, 1e-6)
    T.near(z, 3, 1e-6)
end)

T.test("a body comes to rest on the floor", function()
    local p = P.new(T)
    p:add_floor(0)
    local ball = p:add_body({ shape = sphere(0.5), position = { 0, 3, 0 } })
    p:step(180)
    local x, y, z = P.position(ball)
    T.near(y, 0.5, 0.05, "resting height equals the radius")
    local vx, vy, vz = P.velocity(ball)
    T.near(vx, 0, 1e-2)
    T.near(vy, 0, 0.05)
    T.near(vz, 0, 1e-2)
    T.near(x, 0, 1e-3)
    T.near(z, 0, 1e-3)
end)

T.test("a box stacked on a box rests on top of it", function()
    local p = P.new(T)
    p:add_floor(0)
    local lower = p:add_body({ shape = box(1, 0.5, 1), mass = 2, position = { 0, 0.5, 0 } })
    local upper = p:add_body({ shape = box(1, 0.5, 1), mass = 1, position = { 0, 1.6, 0 } })
    p:step(240)
    local _, y_lower = P.position(lower)
    local _, y_upper = P.position(upper)
    T.near(y_lower, 0.5, 0.05)
    T.near(y_upper, 1.5, 0.08)
end)

T.test("a resting body falls asleep", function()
    local p = P.new(T)
    p:add_floor(0)
    local ball = p:add_body({ shape = sphere(0.5), position = { 0, 0.5, 0 } })
    p:step(300)
    T.eq(ball:get_activation_state(), ISLAND_SLEEPING)
    T.ok(not ball:is_active())
    -- a sleeping body is woken up by activate
    ball:activate(true)
    T.ok(ball:is_active())
end)

-- ---------------------------------------------------------------------------------------
-- Contact response
-- ---------------------------------------------------------------------------------------

T.test("restitution makes a body bounce", function()
    local p = P.new(T)
    local floor = p:add_floor(0)
    floor:set_restitution(1)
    local ball = p:add_body({ shape = sphere(0.5), position = { 0, 2, 0 } })
    ball:set_restitution(1)

    local bounced = false
    for _ = 1, 120 do
        p:step(1)
        local _, vy = P.velocity(ball)
        if vy > 1 then
            bounced = true
            break
        end
    end
    T.ok(bounced, "the ball moves upwards again after hitting the floor")
end)

T.test("without restitution a body does not bounce", function()
    local p = P.new(T)
    local floor = p:add_floor(0)
    floor:set_restitution(0)
    local ball = p:add_body({ shape = sphere(0.5), position = { 0, 2, 0 } })
    ball:set_restitution(0)
    local highest_after_landing, landed = 0, false
    for _ = 1, 150 do
        p:step(1)
        local _, y = P.position(ball)
        if y < 0.6 then landed = true end
        if landed then highest_after_landing = math.max(highest_after_landing, y) end
    end
    T.ok(landed)
    T.ok(highest_after_landing < 0.7, "highest height after landing: " .. highest_after_landing)
end)

T.test("friction slows a sliding box, no friction does not", function()
    local function slide(friction)
        local p = P.new(T)
        local floor = p:add_floor(0)
        floor:set_friction(friction)
        -- a flat box: it slides instead of tipping over
        local slider = p:add_body({ shape = box(1, 0.2, 1), mass = 1, position = { 0, 0.2, 0 } })
        slider:set_friction(friction)
        slider:set_linear_velocity(vec(5, 0, 0))
        p:step(60)
        local vx = P.velocity(slider)
        p:dispose()
        return vx
    end
    local frictionless = slide(0)
    local rough = slide(1)
    T.near(frictionless, 5, 0.05)
    T.ok(rough < 0.5, "a rough floor stops the box, remaining speed: " .. rough)
end)

T.test("collision groups and masks decide what collides", function()
    local GROUP_BALL, GROUP_FLOOR, GROUP_OTHER = 2, 4, 8
    local function drop(floor_mask)
        local p = P.new(T)
        p:add_body({
            shape = b.btStaticPlaneShape(vec(0, 1, 0), 0), mass = 0,
            group = GROUP_FLOOR, mask = floor_mask,
        })
        local ball = p:add_body({
            shape = sphere(0.5), position = { 0, 1, 0 },
            group = GROUP_BALL, mask = GROUP_FLOOR,
        })
        p:step(120)
        local _, y = P.position(ball)
        p:dispose()
        return y
    end
    T.near(drop(GROUP_BALL), 0.5, 0.05) -- the floor accepts the ball's group: it lands
    local fell_through = drop(GROUP_OTHER) -- the floor only accepts another group
    T.ok(fell_through < -5, "the ball fell through the floor, y = " .. fell_through)
end)

T.test("a body's contacts are visible in the dispatcher's manifolds", function()
    local p = P.new(T)
    -- (userdata made from the same C++ pointer do not compare equal, so bodies are told
    -- apart by their user index)
    local floor = p:add_floor(0)
    floor:set_user_index(100)
    local ball = p:add_body({ shape = sphere(0.5), position = { 0, 0.45, 0 } })
    ball:set_user_index(200)
    p:step(10)

    T.eq(p.dispatcher:get_num_manifolds(), 1)
    local manifold = p.dispatcher:get_manifold_by_index_internal(0)
    T.ok(manifold:get_num_contacts() >= 1)
    local indices = { [manifold:get_body0():get_user_index()] = true, [manifold:get_body1():get_user_index()] = true }
    T.ok(indices[100], "the floor takes part in the contact")
    T.ok(indices[200], "so does the ball")

    local point = manifold:get_contact_point(0)
    T.ok(point:get_distance() < 0.05, "the surfaces touch")
    T.ok(point:get_applied_impulse() > 0, "the solver pushed the bodies apart")
    T.ok(point:get_life_time() >= 1)
    local on_ball, on_floor = point:get_position_world_on_a(), point:get_position_world_on_b()
    T.near(on_ball:y() - on_floor:y(), point:get_distance(), 1e-2)
end)

T.test("dispatcher: needs_collision and needs_response", function()
    local p = P.new(T)
    local a = p:add_body({ shape = sphere(1) })
    local c = p:add_body({ shape = sphere(1), position = { 0, 5, 0 } })
    T.ok(p.dispatcher:needs_collision(a, c))
    T.ok(p.dispatcher:needs_response(a, c))
    -- two static objects never collide
    local s1 = p:add_body({ shape = sphere(1), mass = 0, position = { 10, 0, 0 } })
    local s2 = p:add_body({ shape = sphere(1), mass = 0, position = { 20, 0, 0 } })
    T.ok(not p.dispatcher:needs_collision(s1, s2))
end)

-- ---------------------------------------------------------------------------------------
-- Forces applied while simulating
-- ---------------------------------------------------------------------------------------

T.test("a force accelerates a body, and is cleared after the step", function()
    local p = P.new(T)
    p.world:set_gravity(vec(0, 0, 0))
    local body = p:add_body({ shape = sphere(1), mass = 2, position = { 0, 0, 0 } })
    body:apply_central_force(vec(10, 0, 0)) -- a = F / m = 5
    p:step(1)
    local vx = P.velocity(body)
    T.near(vx, 5 * DT, 1e-4)
    T.vec3(body:get_total_force(), 0, 0, 0)

    -- a force has to be applied again to keep accelerating
    p:step(59)
    T.near(P.velocity(body), 5 * DT, 1e-4)
end)

T.test("an impulse sets the velocity, which a body without gravity keeps", function()
    local p = P.new(T)
    p.world:set_gravity(vec(0, 0, 0))
    local body = p:add_body({ shape = sphere(1), mass = 4, position = { 0, 0, 0 } })
    body:apply_central_impulse(vec(8, 0, 0))
    p:step(60)
    local x = P.position(body)
    T.near(P.velocity(body), 2, 1e-4)
    T.near(x, 2, 0.05) -- two units per second for one second
end)

T.test("a body with its own gravity ignores the world's", function()
    local p = P.new(T)
    local body = p:add_body({ shape = sphere(1), position = { 0, 50, 0 } })
    body:set_gravity(vec(0, 0, 0))
    p:step(30)
    local _, y = P.position(body)
    T.near(y, 50, 1e-3)
end)

T.test("a body spins from an applied torque", function()
    local p = P.new(T)
    p.world:set_gravity(vec(0, 0, 0))
    local body = p:add_body({ shape = sphere(1), mass = 1, position = { 0, 0, 0 } })
    body:apply_torque_impulse(vec(0, 0, 0.4)) -- inertia 0.4
    p:step(1)
    T.vec3(body:get_angular_velocity(), 0, 0, 1, 1e-3)
end)

T.test("the motion state follows the simulated body, one step behind by default", function()
    local p = P.new(T)
    local body = p:add_body({ shape = sphere(1), position = { 0, 10, 0 } })
    T.ok(p.world:get_latency_motion_state_interpolation())
    p:step(30)
    local out = b.btTransform(b.btQuaternion(0, 0, 0, 1), vec(0, 0, 0))
    body:get_motion_state():get_world_transform(out)
    local _, y = P.position(body)
    local _, vy = P.velocity(body)
    T.ok(y < 10)
    -- latency interpolation reports where the body was one fixed step earlier
    T.near(out:get_origin():y(), y - vy * DT, 1e-3)

    p.world:set_latency_motion_state_interpolation(false)
    T.ok(not p.world:get_latency_motion_state_interpolation())
    p:step(1)
    body:get_motion_state():get_world_transform(out)
    _, y = P.position(body)
    T.near(out:get_origin():y(), y, 1e-4)
end)

T.test("btDiscreteDynamicsWorld: simulation options", function()
    local p = P.new(T)
    T.ok(p.world:get_synchronize_all_motion_states() ~= nil)
    p.world:set_synchronize_all_motion_states(true)
    T.ok(p.world:get_synchronize_all_motion_states())
    p.world:set_apply_speculative_contact_restitution(true)
    T.ok(p.world:get_apply_speculative_contact_restitution())
    p.world:set_apply_speculative_contact_restitution(false)
    T.ok(not p.world:get_apply_speculative_contact_restitution())
    p.world:set_num_tasks(2)
end)

T.test("btDiscreteDynamicsWorld: clear_forces and apply_gravity", function()
    local p = P.new(T)
    local body = p:add_body({ shape = sphere(1), mass = 2, position = { 0, 10, 0 } })
    body:apply_central_force(vec(5, 0, 0))
    p.world:clear_forces()
    T.vec3(body:get_total_force(), 0, 0, 0)

    body:set_gravity(vec(0, -10, 0))
    p.world:apply_gravity()
    T.vec3(body:get_total_force(), 0, -20, 0)
end)

T.test("a kinematic body moves with its motion state and pushes others", function()
    local p = P.new(T)
    p.world:set_gravity(vec(0, 0, 0))
    local mover = p:add_body({ shape = box(1, 1, 1), mass = 0, position = { 0, 0, 0 } })
    mover:set_collision_flags(mover:get_collision_flags() | 2) -- CF_KINEMATIC_OBJECT
    mover:set_activation_state(4) -- DISABLE_DEACTIVATION
    local target = p:add_body({ shape = sphere(0.5), mass = 1, position = { 2.0, 0, 0 } })

    for i = 1, 90 do
        local t = b.btTransform(b.btQuaternion(0, 0, 0, 1), vec(i * 0.03, 0, 0))
        mover:get_motion_state():set_world_transform(t)
        p:step(1)
    end
    local x = P.position(mover)
    T.near(x, 2.7, 0.01)
    local tx = P.position(target)
    T.ok(tx > 2.5, "the target was pushed along, x = " .. tx)
end)

-- ---------------------------------------------------------------------------------------
-- Ray tests and the broadphase
-- ---------------------------------------------------------------------------------------

T.test("ray_test: closest hit", function()
    local p = P.new(T)
    p:add_body({ shape = sphere(1), mass = 0, position = { 5, 0, 0 } })
    p.world:update_aabbs()

    local hit = b.ClosestRayResultCallback(vec(0, 0, 0), vec(10, 0, 0))
    p.world:ray_test(vec(0, 0, 0), vec(10, 0, 0), hit)
    T.ok(hit:has_hit())

    local miss = b.ClosestRayResultCallback(vec(0, 5, 0), vec(10, 5, 0))
    p.world:ray_test(vec(0, 5, 0), vec(10, 5, 0), miss)
    T.ok(not miss:has_hit())

    -- a ray that stops short of the sphere
    local short = b.ClosestRayResultCallback(vec(0, 0, 0), vec(3, 0, 0))
    p.world:ray_test(vec(0, 0, 0), vec(3, 0, 0), short)
    T.ok(not short:has_hit())
end)

T.test("ray_test: all hits", function()
    local p = P.new(T)
    p:add_body({ shape = sphere(1), mass = 0, position = { 5, 0, 0 } })
    p:add_body({ shape = sphere(1), mass = 0, position = { 10, 0, 0 } })
    p.world:update_aabbs()
    local hits = b.AllHitsRayResultCallback(vec(0, 0, 0), vec(20, 0, 0))
    p.world:ray_test(vec(0, 0, 0), vec(20, 0, 0), hits)
    T.ok(hits:has_hit())
end)

T.test("broadphase: AABB of the whole scene and overlapping pairs", function()
    local p = P.new(T)
    p:add_body({ shape = sphere(1), mass = 0, position = { 0, 0, 0 } })
    p:add_body({ shape = sphere(1), position = { 0.5, 0, 0 } }) -- overlaps the first
    p:add_body({ shape = sphere(1), mass = 0, position = { 50, 0, 0 } })
    p.world:update_aabbs()
    p.world:compute_overlapping_pairs()

    local min, max = vec(0, 0, 0), vec(0, 0, 0)
    p.broadphase:get_broadphase_aabb(min, max)
    T.ok(min:x() < -0.5 and max:x() > 50, "the scene bounds cover every body")

    T.eq(p.world:get_pair_cache():get_num_overlapping_pairs(), 1)
    T.eq(p.broadphase:get_overlapping_pair_cache():get_num_overlapping_pairs(), 1)
end)

T.test("btDbvtBroadphase: proxies are created, moved and destroyed directly", function()
    local broadphase = b.btDbvtBroadphase()
    local config = b.btDefaultCollisionConfiguration()
    local dispatcher = b.btCollisionDispatcher(config)

    local proxy = broadphase:create_proxy(vec(-1, -1, -1), vec(1, 1, 1), 8, nil, 1, -1, dispatcher)
    T.ok(proxy ~= nil)
    T.type_is(proxy:get_uid(), "number")

    local min, max = vec(0, 0, 0), vec(0, 0, 0)
    broadphase:get_aabb(proxy, min, max)
    T.vec3(min, -1, -1, -1)
    T.vec3(max, 1, 1, 1)

    broadphase:set_aabb(proxy, vec(4, 4, 4), vec(6, 6, 6), dispatcher)
    broadphase:get_aabb(proxy, min, max)
    T.vec3(min, 4, 4, 4)
    T.vec3(max, 6, 6, 6)

    broadphase:set_velocity_prediction(0.5)
    T.near(broadphase:get_velocity_prediction(), 0.5)
    broadphase:optimize()
    broadphase:calculate_overlapping_pairs(dispatcher)
    broadphase:destroy_proxy(proxy, dispatcher)
end)

T.test("btSimpleBroadphase: a broadphase with a fixed capacity", function()
    local broadphase = b.btSimpleBroadphase(16)
    local config = b.btDefaultCollisionConfiguration()
    local dispatcher = b.btCollisionDispatcher(config)
    local proxy = broadphase:create_proxy(vec(0, 0, 0), vec(2, 2, 2), 8, nil, 1, -1, dispatcher)
    local min, max = vec(0, 0, 0), vec(0, 0, 0)
    broadphase:get_aabb(proxy, min, max)
    T.vec3(max, 2, 2, 2)
    local other = broadphase:create_proxy(vec(1, 1, 1), vec(3, 3, 3), 8, nil, 1, -1, dispatcher)
    T.ok(broadphase:test_aabb_overlap(proxy, other))
    broadphase:destroy_proxy(other, dispatcher)
    broadphase:destroy_proxy(proxy, dispatcher)
end)

T.test("btHashedOverlappingPairCache: pairs are added and found", function()
    local cache = b.btHashedOverlappingPairCache()
    T.eq(cache:get_num_overlapping_pairs(), 0)
    local broadphase = b.btDbvtBroadphase(cache)
    local config = b.btDefaultCollisionConfiguration()
    local dispatcher = b.btCollisionDispatcher(config)
    local p0 = broadphase:create_proxy(vec(0, 0, 0), vec(1, 1, 1), 8, nil, 1, -1, dispatcher)
    local p1 = broadphase:create_proxy(vec(0.5, 0, 0), vec(1.5, 1, 1), 8, nil, 1, -1, dispatcher)
    broadphase:calculate_overlapping_pairs(dispatcher)
    T.eq(cache:get_num_overlapping_pairs(), 1)
    T.ok(cache:find_pair(p0, p1) ~= nil or cache:find_pair(p1, p0) ~= nil)
    T.ok(cache:needs_broadphase_collision(p0, p1))
    cache:remove_overlapping_pairs_containing_proxy(p0, dispatcher)
    T.eq(cache:get_num_overlapping_pairs(), 0)
    broadphase:destroy_proxy(p0, dispatcher)
    broadphase:destroy_proxy(p1, dispatcher)
end)

-- ---------------------------------------------------------------------------------------
-- Solver and other world types
-- ---------------------------------------------------------------------------------------

T.test("btSequentialImpulseConstraintSolver: random numbers are reproducible", function()
    local solver = b.btSequentialImpulseConstraintSolver()
    solver:set_rand_seed(12345)
    T.eq(solver:get_rand_seed(), 12345)
    local first = { solver:bt_rand2(), solver:bt_rand2(), solver:bt_rand2() }
    solver:set_rand_seed(12345)
    T.eq(solver:bt_rand2(), first[1])
    T.eq(solver:bt_rand2(), first[2])
    T.eq(solver:bt_rand2(), first[3])
    for _ = 1, 50 do
        local n = solver:bt_rand_int2(10)
        T.ok(n >= 0 and n < 10, "random int below 10: " .. n)
    end
    solver:reset()
end)

T.test("btSimpleDynamicsWorld: free fall without constraints", function()
    local config = b.btDefaultCollisionConfiguration()
    local dispatcher = b.btCollisionDispatcher(config)
    local broadphase = b.btDbvtBroadphase()
    local solver = b.btSequentialImpulseConstraintSolver()
    local world = b.btSimpleDynamicsWorld(dispatcher, broadphase, solver, config)
    world:set_gravity(vec(0, -10, 0))
    T.vec3(world:get_gravity(), 0, -10, 0)

    local shape = sphere(1)
    local state = b.btDefaultMotionState()
    local inertia = vec(0, 0, 0)
    shape:calculate_local_inertia(1, inertia)
    local body = b.btRigidBody(1, state, shape, inertia)
    world:add_rigid_body(body)
    T.cleanup(function() world:remove_rigid_body(body) end)
    for _ = 1, 60 do world:step_simulation(DT, 1, DT) end
    local _, vy = P.velocity(body)
    T.near(vy, -10, 0.05)
    T.ok(select(2, P.position(body)) < 0)
end)

T.test("btCollisionWorld: detection without dynamics", function()
    local config = b.btDefaultCollisionConfiguration()
    local dispatcher = b.btCollisionDispatcher(config)
    local broadphase = b.btDbvtBroadphase()
    local world = b.btCollisionWorld(dispatcher, broadphase, config)
    T.eq(world:get_num_collision_objects(), 0)

    local shape_a, shape_b = sphere(1), sphere(1)
    local a, c = b.btCollisionObject(), b.btCollisionObject()
    a:set_collision_shape(shape_a)
    c:set_collision_shape(shape_b)
    a:set_world_transform(b.btTransform(b.btQuaternion(0, 0, 0, 1), vec(0, 0, 0)))
    c:set_world_transform(b.btTransform(b.btQuaternion(0, 0, 0, 1), vec(1.5, 0, 0)))
    world:add_collision_object(a)
    world:add_collision_object(c)
    T.cleanup(function()
        world:remove_collision_object(c)
        world:remove_collision_object(a)
    end)
    T.eq(world:get_num_collision_objects(), 2)

    world:perform_discrete_collision_detection()
    T.eq(dispatcher:get_num_manifolds(), 1)
    local point = dispatcher:get_manifold_by_index_internal(0):get_contact_point(0)
    T.near(point:get_distance(), -0.5, 1e-3, "the spheres overlap by 0.5")

    T.ok(world:get_dispatch_info() ~= nil)
    T.ok(world:get_force_update_all_aabbs())
    world:set_force_update_all_aabbs(false)
    T.ok(not world:get_force_update_all_aabbs())
end)

T.test("btGhostObject: sees the bodies that overlap it", function()
    local p = P.new(T)
    local ghost_callback = b.btGhostPairCallback()
    p.broadphase:get_overlapping_pair_cache():set_internal_ghost_pair_callback(ghost_callback)

    local ghost = p:hold(b.btGhostObject())
    ghost:set_collision_shape(p:hold(sphere(2)))
    ghost:set_collision_flags(4) -- CF_NO_CONTACT_RESPONSE
    p.world:add_collision_object(ghost, 1, -1)
    T.cleanup(function() p.world:remove_collision_object(ghost) end)

    p.world:set_gravity(vec(0, 0, 0))
    local inside = p:add_body({ shape = sphere(0.5), position = { 0, 0, 0 } })
    inside:set_user_index(11)
    local outside = p:add_body({ shape = sphere(0.5), position = { 30, 0, 0 } })
    outside:set_user_index(22)
    p:step(2)
    T.eq(ghost:get_num_overlapping_objects(), 1)
    T.eq(ghost:get_overlapping_object(0):get_user_index(), 11)
    T.ok(b.btGhostObject.upcast(ghost) ~= nil)
end)

T.run()
