-- Helpers for tests that need a running btDiscreteDynamicsWorld.
--
-- Bullet's objects only hold raw pointers to each other, while Lua frees whatever it can no
-- longer reach, in no promised order. A fixture therefore keeps every piece it creates alive
-- and, through T.cleanup, takes constraints and bodies back out of the world before the test
-- ends (whether it passed or not).

local b = bullet3

local Physics = {}
Physics.__index = Physics

--- Create a world with the default collision setup and register its cleanup with the test.
--- With `soft = true` the world is a btSoftRigidDynamicsWorld that also takes soft bodies.
function Physics.new(T, options)
    options = options or {}
    local self = setmetatable({ keep = {}, bodies = {}, constraints = {}, actions = {}, soft_bodies = {} }, Physics)
    if options.soft then
        self.config = b.btSoftBodyRigidBodyCollisionConfiguration()
    else
        self.config = b.btDefaultCollisionConfiguration(b.btDefaultCollisionConstructionInfo())
    end
    self.dispatcher = b.btCollisionDispatcher(self.config)
    self.broadphase = b.btDbvtBroadphase()
    self.solver = b.btSequentialImpulseConstraintSolver()
    if options.soft then
        self.world = b.btSoftRigidDynamicsWorld(self.dispatcher, self.broadphase, self.solver, self.config)
        self.info = self.world:get_world_info()
    else
        self.world = b.btDiscreteDynamicsWorld(self.dispatcher, self.broadphase, self.solver, self.config)
    end
    self.world:set_gravity(b.btVector3(0, -10, 0))
    T.cleanup(function() self:dispose() end)
    return self
end

function Physics:dispose()
    for i = #self.soft_bodies, 1, -1 do
        self.world:remove_soft_body(self.soft_bodies[i])
    end
    self.soft_bodies = {}
    for i = #self.actions, 1, -1 do
        self.world:remove_action(self.actions[i])
    end
    for i = #self.constraints, 1, -1 do
        self.world:remove_constraint(self.constraints[i])
    end
    for i = #self.bodies, 1, -1 do
        self.world:remove_rigid_body(self.bodies[i])
    end
    self.actions, self.constraints, self.bodies = {}, {}, {}
end

--- Keep a Lua object (a shape, a mesh, ...) alive as long as the fixture
function Physics:hold(object)
    table.insert(self.keep, object)
    return object
end

--- Create a rigid body, add it to the world and return it.
--- `options`: shape, mass (default 1), position {x, y, z}, group and mask
function Physics:add_body(options)
    local shape = self:hold(options.shape)
    local mass = options.mass or 1
    local position = options.position or { 0, 0, 0 }
    local start = b.btTransform(b.btQuaternion(0, 0, 0, 1), b.btVector3(position[1], position[2], position[3]))
    local state = self:hold(b.btDefaultMotionState(start))

    local inertia = b.btVector3(0, 0, 0)
    if mass ~= 0 then
        shape:calculate_local_inertia(mass, inertia)
    end
    local body = b.btRigidBody(mass, state, shape, inertia)
    self:hold(body)
    if options.group then
        self.world:add_rigid_body(body, options.group, options.mask)
    else
        self.world:add_rigid_body(body)
    end
    table.insert(self.bodies, body)
    return body
end

--- Add a soft body (a btSoftRigidDynamicsWorld is needed) and return it
function Physics:add_soft_body(body)
    self.world:add_soft_body(body)
    table.insert(self.soft_bodies, body)
    return body
end

--- A static, infinite floor at height `y`
function Physics:add_floor(y)
    return self:add_body({
        shape = b.btStaticPlaneShape(b.btVector3(0, 1, 0), y or 0),
        mass = 0,
    })
end

function Physics:add_constraint(constraint, disable_collisions)
    self:hold(constraint)
    self.world:add_constraint(constraint, disable_collisions or false)
    table.insert(self.constraints, constraint)
    return constraint
end

function Physics:add_action(action)
    self:hold(action)
    self.world:add_action(action)
    table.insert(self.actions, action)
    return action
end

--- Advance the simulation `count` fixed steps of `dt` seconds (default 1/60)
function Physics:step(count, dt)
    dt = dt or 1 / 60
    for _ = 1, count or 1 do
        self.world:step_simulation(dt, 1, dt)
    end
end

--- Position of a body's centre of mass as x, y, z
function Physics.position(body)
    local origin = body:get_world_transform():get_origin()
    return origin:x(), origin:y(), origin:z()
end

function Physics.velocity(body)
    local v = body:get_linear_velocity()
    return v:x(), v:y(), v:z()
end

return Physics
