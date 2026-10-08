-- How the generated LuaBridge3 layer behaves: naming, overloads, default arguments,
-- references and copies, null pointers, errors, and the things the bindings deliberately or
-- unavoidably leave out. The "limitation" tests pin down today's behaviour; if one of them starts
-- failing because a limitation was lifted, update the test (and the README) rather than the code.

local T = require("testlib")
local P = require("physics")
local b = bullet3

local function vec(x, y, z)
    return b.btVector3(x, y, z)
end

-- ---------------------------------------------------------------------------------------
-- Naming and structure
-- ---------------------------------------------------------------------------------------

T.test("the only global the bindings add is the bullet3 namespace", function()
    T.type_is(_G.bullet3, "table")
    T.is_nil(_G.btVector3)
    T.is_nil(_G.btRigidBody)
end)

T.test("camelCase C++ names become snake_case", function()
    local v = vec(1, 2, 3)
    T.type_is(v.set_value, "function") -- setValue
    T.type_is(v.get_skew_symmetric_matrix, "function") -- getSkewSymmetricMatrix
    T.is_nil(v.setValue)
    T.is_nil(v.getX)
    T.type_is(b.btTransform.get_identity, "function") -- static getIdentity
    T.type_is(b.btQuaternion.get_identity, "function")
end)

T.test("methods are found on instances and through inheritance", function()
    local sphere = b.btSphereShape(1)
    T.type_is(sphere.get_radius, "function") -- own
    T.type_is(sphere.get_margin, "function") -- btConvexInternalShape
    T.type_is(sphere.get_name, "function")
    T.type_is(sphere.is_compound, "function") -- btCollisionShape, three levels up
    T.is_nil(sphere.get_half_extents_with_margin, "a box-only function")

    -- the class table lists a class's own functions only
    T.type_is(b.btSphereShape.get_radius, "function")
    T.is_nil(b.btSphereShape.is_compound)
end)

T.test("a derived instance is accepted where a base class pointer is expected", function()
    local p = P.new(T)
    local body = p:add_body({ shape = b.btSphereShape(1) }) -- btSphereShape as btCollisionShape*, btDefaultMotionState as btMotionState*
    T.ok(body:is_in_world())
    -- a btRigidBody is a btCollisionObject
    local object = b.btCollisionObject()
    T.ok(object:check_collide_with(body))
end)

-- ---------------------------------------------------------------------------------------
-- Overloads and default arguments
-- ---------------------------------------------------------------------------------------

T.test("constructor overloads are chosen by argument count and type", function()
    local q = b.btQuaternion(0, 0, 0, 1) -- 4 numbers
    local axis_angle = b.btQuaternion(vec(0, 0, 1), 1) -- vector, number
    local euler = b.btQuaternion(1, 0, 0) -- 3 numbers
    T.near(q:w(), 1)
    T.near(axis_angle:z(), math.sin(0.5), 1e-5)
    T.near(euler:y(), math.sin(0.5), 1e-5)

    local t1 = b.btTransform(q)
    local t2 = b.btTransform(b.btMatrix3x3(q))
    local t3 = b.btTransform(t1)
    T.quat(t1:get_rotation(), 0, 0, 0, 1)
    T.quat(t2:get_rotation(), 0, 0, 0, 1)
    T.quat(t3:get_rotation(), 0, 0, 0, 1)
end)

T.test("method overloads are chosen by argument count and type", function()
    local p = P.new(T)
    local a = p:add_body({ shape = b.btSphereShape(0.5), mass = 0, position = { 0, 0, 0 } })
    local c = p:add_body({ shape = b.btSphereShape(0.5), position = { 1, 0, 0 } })
    local hinge = b.btHingeConstraint(a, c, vec(0, 0, 0), vec(-1, 0, 0), vec(0, 0, 1), vec(0, 0, 1), true)
    T.type_is(hinge:get_hinge_angle(), "number") -- no arguments
    local identity = b.btTransform(b.btQuaternion(0, 0, 0, 1))
    T.type_is(hinge:get_hinge_angle(identity, identity), "number") -- two transforms

    local angular = b.btMatrix3x3()
    angular:set_euler_zyx(0, 0, 0.1)
    local body = p:add_body({ shape = b.btSphereShape(0.5), position = { 5, 0, 0 } })
    body:set_angular_factor(0.5) -- number overload
    body:set_angular_factor(vec(1, 0, 1)) -- vector overload
    T.vec3(body:get_angular_factor(), 1, 0, 1)
end)

T.test("trailing arguments with default values can be left out", function()
    local p = P.new(T)
    -- step_simulation(timeStep, maxSubSteps = 1, fixedTimeStep = 1 / 60)
    T.eq(p.world:step_simulation(1 / 60), 1)
    T.eq(p.world:step_simulation(1 / 60, 5), 1)
    T.eq(p.world:step_simulation(1 / 60, 5, 1 / 60), 1)

    -- btCollisionObject::activate(forceActivation = false)
    local object = b.btCollisionObject()
    object:activate()
    object:activate(true)

    -- btCollisionObject::setAnisotropicFriction(friction, mode = CF_ANISOTROPIC_FRICTION)
    object:set_anisotropic_friction(vec(1, 1, 1))
    object:set_anisotropic_friction(vec(1, 1, 1), 1)
end)

-- ---------------------------------------------------------------------------------------
-- References, copies, pointers
-- ---------------------------------------------------------------------------------------

T.test("functions returning by value hand out independent copies", function()
    local v = vec(3, 4, 0)
    local n = v:normalized()
    n:set_x(100)
    T.vec3(v, 3, 4, 0)

    local t = b.btTransform(b.btQuaternion(0, 0, 0, 1), vec(1, 2, 3))
    local rotation = t:get_rotation()
    rotation:set_value(9, 9, 9, 9)
    T.quat(t:get_rotation(), 0, 0, 0, 1)
end)

T.test("functions returning a reference hand out the object itself", function()
    local t = b.btTransform(b.btQuaternion(0, 0, 0, 1), vec(1, 2, 3))
    t:get_origin():set_x(9)
    T.vec3(t:get_origin(), 9, 2, 3)

    -- and the chained mutators return the object they were called on
    local v = vec(0, 3, 4)
    v:normalize():set_x(1)
    T.vec3(v, 1, 0.6, 0.8)
end)

T.test("arguments passed by const reference are copied, by reference are written to", function()
    local source = vec(1, 2, 3)
    local target = b.btTransform(b.btQuaternion(0, 0, 0, 1), vec(0, 0, 0))
    target:set_origin(source)
    source:set_x(100)
    T.vec3(target:get_origin(), 1, 2, 3)

    -- an out parameter (btVector3&) is filled in place
    local min, max = vec(0, 0, 0), vec(0, 0, 0)
    b.btSphereShape(2):get_aabb(target, min, max)
    T.ok(max:x() > min:x())
end)

T.test("a null pointer comes back as nil", function()
    local body = b.btRigidBody(1, b.btDefaultMotionState(), b.btSphereShape(1))
    T.is_nil(body:get_broadphase_proxy())
    T.is_nil(b.btRigidBody.upcast(b.btCollisionObject()))
    local shape = b.btCollisionObject()
    T.is_nil(shape:get_collision_shape())
end)

T.test("strings are returned as Lua strings", function()
    T.eq(b.btSphereShape(1):get_name(), "SPHERE")
    T.type_is(b.btBoxShape(vec(1, 1, 1)):get_name(), "string")
end)

T.test("a null void pointer comes back as userdata, not nil", function()
    T.type_is(b.btSphereShape(1):get_user_pointer(), "userdata")
end)

-- ---------------------------------------------------------------------------------------
-- Errors
-- ---------------------------------------------------------------------------------------

T.test("bad arguments raise Lua errors instead of crashing", function()
    T.throws(function() return b.btVector3("a", 1, 2) end)
    T.throws(function() return b.btSphereShape("big") end)
    T.throws(function() return vec(1, 2, 3):dot() end, nil)
    T.throws(function() return vec(1, 2, 3):dot(42) end)
    T.throws(function() return vec(1, 2, 3):dot(b.btSphereShape(1)) end)
    T.throws(function() return b.btVector3.dot(42, vec(1, 2, 3)) end)
    T.throws(function() return b.btVector3(1, 2) end)
end)

T.test("a wrong class where a pointer is expected raises an error", function()
    local p = P.new(T)
    T.throws(function() p.world:add_rigid_body(b.btSphereShape(1)) end)
    T.throws(function() p.world:add_rigid_body(nil) end, nil)
end)

T.test("calling a method that does not exist is a plain Lua error", function()
    T.throws(function() return vec(1, 2, 3):no_such_method() end)
end)

T.test("classes that are abstract or have no bound constructor cannot be called", function()
    T.throws(function() return b.btCollisionShape() end) -- abstract
    T.throws(function() return b.btDispatcher() end) -- abstract
    T.throws(function() return b.btVector3FloatData() end) -- plain data, no declared constructor
    T.throws(function() return b.btSoftBody() end) -- only built by btSoftBodyHelpers
end)

-- ---------------------------------------------------------------------------------------
-- Known limitations
-- ---------------------------------------------------------------------------------------

T.test("limitation: operators are not bound", function()
    local a, c = vec(1, 2, 3), vec(4, 5, 6)
    T.throws(function() return a + c end)
    T.throws(function() return a * 2 end)
    T.throws(function() return a - c end)
    T.type_is(a.dot, "function") -- the named methods are the way to do arithmetic
end)

T.test("limitation: public data members are not bound", function()
    local callback = b.ClosestRayResultCallback(vec(0, 0, 0), vec(1, 0, 0))
    T.is_nil(callback.m_closestHitFraction)
    T.is_nil(callback.m_collisionObject)
    T.is_nil(vec(1, 2, 3).m_floats)
    T.ok(callback:has_hit() == false, "has_hit() is what the callback offers")
end)

T.test("limitation: free functions and enum constants are not bound", function()
    -- the namespace holds classes only
    for name, value in pairs(b) do
        if type(name) == "string" and not name:find("^__") then
            T.type_is(value, "table", name)
        end
    end
    T.is_nil(b.btSetTaskScheduler)
    T.is_nil(b.btFabs)
    T.is_nil(b.CF_STATIC_OBJECT)
    T.is_nil(b.ACTIVE_TAG)
    T.is_nil(b.BT_DISCRETE_DYNAMICS_WORLD)
end)

T.test("limitation: functions returning an enum or a container raise an error", function()
    local p = P.new(T)
    -- enum results (btDynamicsWorldType, btConstraintSolverType, btTypedConstraintType, ...)
    T.throws(function() return p.world:get_world_type() end, "not registered")
    T.throws(function() return p.solver:get_solver_type() end, "not registered")
    local anchor = p:add_body({ shape = b.btSphereShape(0.5), mass = 0 })
    local ball = p:add_body({ shape = b.btSphereShape(0.5), position = { 1, 0, 0 } })
    local joint = b.btPoint2PointConstraint(anchor, ball, vec(0, 0, 0), vec(-1, 0, 0))
    T.throws(function() return joint:get_constraint_type() end, "not registered")

    -- btAlignedObjectArray results
    T.throws(function() return p.world:get_collision_object_array() end, "not registered")
    T.throws(function() return p.world:get_non_static_rigid_bodies() end, "not registered")
end)

T.test("limitation: pointers to bound objects are new userdata on every call", function()
    local shape = b.btBoxShape(vec(1, 1, 1))
    local body = b.btRigidBody(1, b.btDefaultMotionState(), shape)
    local first, second = body:get_collision_shape(), body:get_collision_shape()
    -- both wrap the same C++ shape, but `==` does not know that
    T.ok(first ~= second)
    first:set_margin(0.1)
    T.near(second:get_margin(), 0.1, 1e-6, "they still share state")
    T.near(shape:get_margin(), 0.1, 1e-6)
end)

T.test("limitation: classes without a declared constructor cannot be created", function()
    -- including btMultiBodyConstraintSolver, which a multibody world is constructed with
    T.throws(function() return b.btMultiBodyConstraintSolver() end)
    -- btMultiBodyMLCPConstraintSolver derives from it and has a constructor
    T.ok(b.btMultiBodyMLCPConstraintSolver(b.btDantzigSolver()) ~= nil)
end)

T.test("limitation: btAxisSweep3 does not derive from btBroadphaseInterface", function()
    -- its template base class is not bound, so it cannot be passed to a world
    local sweep = b.btAxisSweep3(vec(-100, -100, -100), vec(100, 100, 100))
    T.ok(sweep ~= nil)
    local config = b.btDefaultCollisionConfiguration()
    local dispatcher = b.btCollisionDispatcher(config)
    local solver = b.btSequentialImpulseConstraintSolver()
    T.throws(function() return b.btDiscreteDynamicsWorld(dispatcher, sweep, solver, config) end)
end)

T.run()
