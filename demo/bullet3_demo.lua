print("=== Bullet3 Physics Simulation Demo ===")
print()

-- 1. Set up physics world
print("1. Setting up physics world...")
local constructionInfo = bullet3.btDefaultCollisionConstructionInfo()
local collisionConfig = bullet3.btDefaultCollisionConfiguration(constructionInfo)
local dispatcher = bullet3.btCollisionDispatcher(collisionConfig)
local solver = bullet3.btSequentialImpulseConstraintSolver()

-- Create broadphase and dynamics world
local broadphase = bullet3.btDbvtBroadphase()
local dynamicsWorld = bullet3.btDiscreteDynamicsWorld(dispatcher, broadphase, solver, collisionConfig)

-- Set gravity
local gravity = bullet3.btVector3(0.0, -9.81, 0.0)
dynamicsWorld:set_gravity(gravity)

print("   - Created physics world with gravity: (" .. gravity:x() .. ", " .. gravity:y() .. ", " .. gravity:z() .. ")")

-- 2. Test basic simulation step
print()
print("2. Testing physics simulation...")
local timeStep = 1.0 / 60.0
print("   - Time step: " .. timeStep .. " seconds")

-- Run a few simulation steps
for i = 1, 5 do
    local numSubSteps = dynamicsWorld:step_simulation(timeStep, 10, timeStep)
    print("   - Step " .. i .. ": simulation ran with " .. numSubSteps .. " sub-steps")
end

-- 3. Test constraint solver
print()
print("3. Testing constraint solver...")
print("   - Random seed: " .. solver:get_rand_seed())

-- Set new random seed
solver:set_rand_seed(12345)
print("   - Set new random seed to 12345")
print("   - New random seed: " .. solver:get_rand_seed())

-- Test some random functions
print("   - Random value 1: " .. solver:bt_rand2())
print("   - Random value 2: " .. solver:bt_rand2())
print("   - Random int 1: " .. solver:bt_rand_int2(100))
print("   - Random int 2: " .. solver:bt_rand_int2(100))

-- 4. Test dynamics world properties
print()
print("4. Testing dynamics world properties...")
local currentGravity = dynamicsWorld:get_gravity()
print("   - Current gravity: (" .. currentGravity:x() .. ", " .. currentGravity:y() .. ", " .. currentGravity:z() .. ")")

print("   - Number of constraints: " .. dynamicsWorld:get_num_constraints())

-- Test gravity change
local newGravity = bullet3.btVector3(0.0, -3.71, 0.0)  -- Mars gravity
dynamicsWorld:set_gravity(newGravity)
local marsGravity = dynamicsWorld:get_gravity()
print("   - Changed to Mars gravity: (" .. marsGravity:x() .. ", " .. marsGravity:y() .. ", " .. marsGravity:z() .. ")")

-- 5. Test simulation with different gravity
print()
print("5. Testing simulation with Mars gravity...")
for i = 1, 3 do
    local numSubSteps = dynamicsWorld:step_simulation(1.0/60.0, 10, 1.0/60.0)
    print("   - Mars step " .. i .. ": simulation ran with " .. numSubSteps .. " sub-steps")
end

-- 6. Test solver reset and cleanup
print()
print("6. Testing solver operations...")
solver:reset()
print("   - Solver reset completed")

dynamicsWorld:clear_forces()
print("   - All forces cleared from dynamics world")

print("   - All physics simulation tests completed successfully!")
print()
print("=== Physics Simulation Demo Complete ===")