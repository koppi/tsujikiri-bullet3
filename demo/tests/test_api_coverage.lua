-- Checks every class, constructor and function tsujikiri bound against the manifest that
-- gen_api_manifest.py reads back from the generated bindings (bullet3_api.lua):
--
--   * the bullet3 namespace holds exactly the bound classes,
--   * every function and static function is reachable from its class table,
--   * a class without bound constructors cannot be called,
--   * every class that can be built from its constructor signatures is instantiated, and
--     every own and inherited member function is then reachable on the instance (checks the
--     deriveClass wiring).
--
-- What each function *does* is covered by the behavioural tests in the other test_*.lua files.

local T = require("testlib")
local api = require("bullet3_api")

local ns = _G[api.namespace]

local by_name, children = {}, {}
for _, class in ipairs(api.classes) do
    by_name[class.name] = class
    if class.base then
        children[class.base] = children[class.base] or {}
        table.insert(children[class.base], class.name)
    end
end

-- Member functions of a class plus everything it inherits. Instances see these, the class
-- table itself only holds the class's own functions
local function member_functions_of(class)
    local names, seen = {}, {}
    while class do
        for _, name in ipairs(class.methods) do
            if not seen[name] then
                seen[name] = true
                table.insert(names, name)
            end
        end
        class = class.base and by_name[class.base]
    end
    return names
end

-- ---------------------------------------------------------------------------------------
-- Building instances from constructor signatures
-- ---------------------------------------------------------------------------------------

local NUMBERS = {}
for name in ([[int unsigned short long char float double size_t btScalar int8_t int16_t int32_t
    int64_t uint8_t uint16_t uint32_t uint64_t]]):gmatch("%S+") do
    NUMBERS[name] = true
end

-- Value types get valid, non-degenerate values rather than whatever their default constructor makes
local VALUES = {
    btVector3 = function() return ns.btVector3(1, 1, 1) end,
    btQuaternion = function() return ns.btQuaternion(0, 0, 0, 1) end,
    btMatrix3x3 = function() return ns.btMatrix3x3(1, 0, 0, 0, 1, 0, 0, 0, 1) end,
    btTransform = function() return ns.btTransform(ns.btQuaternion(0, 0, 0, 1), ns.btVector3(0, 0, 0)) end,
    -- the BVH mesh shapes crash while building their tree from an empty mesh
    btTriangleMesh = function()
        local mesh = ns.btTriangleMesh()
        mesh:add_triangle(ns.btVector3(0, 0, 0), ns.btVector3(1, 0, 0), ns.btVector3(0, 1, 0))
        return mesh
    end,
}

-- Interfaces get a concrete implementation that is known to be cheap to build
local PREFERRED = {
    btMotionState = "btDefaultMotionState",
    btCollisionShape = "btSphereShape",
    btConvexShape = "btSphereShape",
    btConcaveShape = "btStaticPlaneShape",
    btStridingMeshInterface = "btTriangleMesh",
    btBroadphaseInterface = "btDbvtBroadphase",
    btOverlappingPairCache = "btHashedOverlappingPairCache",
    btDispatcher = "btCollisionDispatcher",
    btCollisionConfiguration = "btDefaultCollisionConfiguration",
    btConstraintSolver = "btSequentialImpulseConstraintSolver",
    btTypedConstraint = "btPoint2PointConstraint",
    btCollisionObject = "btCollisionObject",
    btDynamicsWorld = "btDiscreteDynamicsWorld",
}

-- Classes that cannot be built from made-up arguments (reason in the value); they are still
-- checked for registration. Everything not listed here is constructed.
local SKIP = {}

-- The multithreaded classes need a task scheduler, which is installed with the free function
-- btSetTaskScheduler (free functions are not bound); without one they dereference null.
for _, name in ipairs({ "btCollisionDispatcherMt", "btDiscreteDynamicsWorldMt",
    "btSequentialImpulseConstraintSolverMt", "btConstraintSolverPoolMt", "btSimulationIslandManagerMt" }) do
    SKIP[name] = "needs a task scheduler, and btSetTaskScheduler is not bound"
end

-- Whole families (the class and everything derived from it) that need a real collision
-- context. Bullet's collision algorithms leave m_ownManifold and m_manifoldPtr uninitialised
-- in their constructors that take only a btCollisionAlgorithmConstructionInfo, and the
-- destructors then release a garbage manifold through a null dispatcher.
local SKIP_FAMILIES = {
    btCollisionAlgorithm = "collision algorithms are only valid when built by a dispatcher",
}
for family, reason in pairs(SKIP_FAMILIES) do
    local queue = { family }
    while #queue > 0 do
        local current = table.remove(queue, 1)
        SKIP[current] = reason
        for _, child in ipairs(children[current] or {}) do
            table.insert(queue, child)
        end
    end
end

-- Split `const Foo &` into ("Foo", is_pointer)
local function parse_type(spelling)
    local is_pointer = spelling:find("*", 1, true) ~= nil
    local name = spelling:gsub("const", ""):gsub("volatile", ""):gsub("struct ", ""):gsub("[&*]", "")
    name = name:gsub("^%s+", ""):gsub("%s+$", ""):gsub("%s+", " ")
    return name, is_pointer
end

local function is_number_type(name)
    for word in name:gmatch("%S+") do
        if NUMBERS[word] then return true end
    end
    return false
end

local instantiate

-- Returns found, value for one parameter; `found` is false when a bound class it needs cannot
-- be built. `value` is nil for a null pointer. Anything that is neither a number, a string, a
-- pointer nor a bound class (an enum, say) is passed as 0, which LuaBridge reads as an integer.
local function synthesize(spelling, keep, depth)
    local name, is_pointer = parse_type(spelling)
    if is_number_type(name) and not is_pointer then
        return true, 1
    elseif name == "bool" and not is_pointer then
        return true, false
    elseif name == "char" and is_pointer then
        return true, "x"
    elseif by_name[name] then
        local object = instantiate(name, keep, depth + 1)
        if object == nil then
            return false
        end
        return true, object
    elseif is_pointer then
        return true, nil
    end
    return true, 0
end

-- Returns the argument list for a constructor signature, or nil if a parameter cannot be built
local function build_args(signature, keep, depth)
    local args = {}
    for i, spelling in ipairs(signature) do
        local found, value = synthesize(spelling, keep, depth)
        if not found then
            return nil
        end
        args[i] = value
    end
    return args
end

-- Try each constructor, fewest parameters first. Returns the object, or nil + reason
local function construct(class, keep, depth)
    local signatures = {}
    for _, signature in ipairs(class.constructors) do
        table.insert(signatures, signature)
    end
    table.sort(signatures, function(a, b) return #a < #b end)

    local reason = "no constructor signature could be built from known types"
    for _, signature in ipairs(signatures) do
        local args = build_args(signature, keep, depth)
        if args then
            local ok, object = pcall(ns[class.name], table.unpack(args, 1, #signature))
            if ok then
                return object
            end
            reason = tostring(object)
        end
    end
    return nil, reason
end

-- Lowest remaining depth budget at which a class already failed to build; trying it again
-- with no more budget cannot succeed and would only make the search exponential
local failed_at = {}

instantiate = function(name, keep, depth)
    if depth > 3 or SKIP[name] or (failed_at[name] and depth >= failed_at[name]) then
        return nil
    end

    -- Concrete class: build it. Interface: build the preferred implementation, else the
    -- first derived class that can be built
    local candidates = { name }
    if PREFERRED[name] and PREFERRED[name] ~= name then
        table.insert(candidates, 1, PREFERRED[name])
    end
    local queue = { name }
    while #queue > 0 do
        local current = table.remove(queue, 1)
        for _, child in ipairs(children[current] or {}) do
            table.insert(candidates, child)
            table.insert(queue, child)
        end
    end

    for _, candidate in ipairs(candidates) do
        local class = by_name[candidate]
        local object
        if VALUES[candidate] then
            object = VALUES[candidate]()
        elseif class and #class.constructors > 0 and not SKIP[candidate] then
            object = construct(class, keep, depth)
        end
        if object ~= nil then
            table.insert(keep, object)
            return object
        end
    end
    failed_at[name] = math.min(failed_at[name] or depth, depth)
    return nil
end

-- ---------------------------------------------------------------------------------------
-- Tests
-- ---------------------------------------------------------------------------------------

T.test("the manifest is not empty", function()
    T.eq(api.namespace, "bullet3")
    T.ok(#api.classes > 500, "expected hundreds of bound classes, manifest has " .. #api.classes)
end)

T.test("the namespace holds exactly the bound classes", function()
    T.type_is(ns, "table")
    local present = 0
    for name, value in pairs(ns) do
        -- LuaBridge keeps its own metamethods (__index, ...) and light userdata keys in the
        -- namespace table
        if type(name) == "string" and not name:find("^__") then
            present = present + 1
            T.ok(by_name[name], "bullet3." .. tostring(name) .. " is not in the generated bindings")
            T.type_is(value, "table", "bullet3." .. name)
        end
    end
    T.eq(present, #api.classes, "number of classes in the bullet3 namespace")
end)

local constructed, constructible, report = 0, 0, {}

for _, class in ipairs(api.classes) do
    T.test(class.name .. ": registered with all its functions", function()
        local lua_class = ns[class.name]
        T.type_is(lua_class, "table")

        for _, name in ipairs(class.methods) do
            T.type_is(lua_class[name], "function", class.name .. ":" .. name)
        end
        for _, name in ipairs(class.statics) do
            T.type_is(lua_class[name], "function", class.name .. "." .. name)
        end
        T.is_nil(lua_class.this_function_does_not_exist)
    end)

    T.test(class.name .. ": constructors", function()
        local lua_class = ns[class.name]
        if #class.constructors == 0 then
            T.throws(function() return lua_class() end)
            return
        end

        constructible = constructible + 1
        if SKIP[class.name] then
            report[#report + 1] = class.name .. ": skipped, " .. SKIP[class.name]
            return
        end
        local keep = {}
        local object, reason = construct(class, keep, 0)
        if object == nil then
            report[#report + 1] = class.name .. ": " .. tostring(reason)
            return
        end
        constructed = constructed + 1
        table.insert(keep, object)

        T.type_is(object, "userdata", class.name)
        -- own and inherited member functions are reachable on the instance
        for _, name in ipairs(member_functions_of(class)) do
            T.type_is(object[name], "function", class.name .. " instance :" .. name)
        end
        -- a second, independent instance
        local other = construct(class, keep, 0)
        T.ne(other, nil, class.name .. " built a second time")
        table.insert(keep, other)
    end)
end

T.test("most bound constructors are exercised", function()
    if T.verbose then
        for _, line in ipairs(report) do print("  not built: " .. line) end
    end
    print(string.format("  %d of %d classes with constructors were built from their signatures",
        constructed, constructible))
    -- guards against the argument synthesis above silently rotting
    T.ok(constructed >= constructible * 0.6,
        string.format("only %d of %d constructible classes could be built", constructed, constructible))
end)

T.run()
