-- LinearMath helpers beyond the vector/matrix/transform types: hashing, allocators, clocks,
-- polar decomposition, Givens rotations, spatial algebra and small containers.

local T = require("testlib")
local b = bullet3

local function vec(x, y, z)
    return b.btVector3(x, y, z)
end

-- ---------------------------------------------------------------------------------------
-- Hashing
-- ---------------------------------------------------------------------------------------

T.test("btHashInt: equal keys hash alike", function()
    local a, same, other = b.btHashInt(7), b.btHashInt(7), b.btHashInt(8)
    T.eq(a:get_uid1(), 7)
    T.ok(a:equals(same))
    T.ok(not a:equals(other))
    T.eq(a:get_hash(), same:get_hash())
    T.ne(a:get_hash(), other:get_hash())
    a:set_uid1(8)
    T.eq(a:get_uid1(), 8)
    T.ok(a:equals(other))
end)

T.test("btHashString: equal strings hash alike", function()
    local a, same, other = b.btHashString("bullet"), b.btHashString("bullet"), b.btHashString("lua")
    T.ok(a:equals(same))
    T.ok(not a:equals(other))
    T.eq(a:get_hash(), same:get_hash())
    T.ne(a:get_hash(), other:get_hash())
    T.type_is(b.btHashString():get_hash(), "number")
end)

T.test("btHashPtr: hashes an address", function()
    local a, same = b.btHashPtr(nil), b.btHashPtr(nil)
    T.ok(a:equals(same))
    T.eq(a:get_hash(), same:get_hash())
    T.type_is(a:get_pointer(), "userdata")
end)

-- ---------------------------------------------------------------------------------------
-- Allocators
-- ---------------------------------------------------------------------------------------

T.test("btPoolAllocator: a fixed number of fixed-size elements", function()
    local pool = b.btPoolAllocator(16, 4)
    T.eq(pool:get_element_size(), 16)
    T.eq(pool:get_max_count(), 4)
    T.eq(pool:get_free_count(), 4)
    T.eq(pool:get_used_count(), 0)

    local first, second = pool:allocate(16), pool:allocate(16)
    T.eq(pool:get_free_count(), 2)
    T.eq(pool:get_used_count(), 2)
    T.ok(pool:valid_ptr(first))
    T.ok(pool:valid_ptr(second))
    T.ok(pool:get_pool_address() ~= nil)

    pool:free_memory(first)
    T.eq(pool:get_used_count(), 1)
    pool:free_memory(second)
    T.eq(pool:get_used_count(), 0)
    T.eq(pool:get_free_count(), 4)
end)

T.test("btStackAlloc: allocations come off the available memory", function()
    local stack = b.btStackAlloc(1024)
    local available = stack:get_available_memory()
    T.ok(available > 0 and available <= 1024)

    local block = stack:begin_block()
    local memory = stack:allocate(256)
    T.ok(memory ~= nil)
    T.ok(stack:get_available_memory() <= available - 256)
    stack:end_block(block)
    T.eq(stack:get_available_memory(), available, "ending the block gives the memory back")
    stack:destroy()
    stack:create(512)
    T.ok(stack:get_available_memory() > 0)
end)

-- ---------------------------------------------------------------------------------------
-- Time, threads, profiling
-- ---------------------------------------------------------------------------------------

T.test("btClock: elapsed time only moves forward", function()
    local clock = b.btClock()
    clock:reset()
    local start = clock:get_time_microseconds()
    local spin = 0
    for i = 1, 2e5 do spin = spin + i end
    T.ok(spin > 0)
    local finish = clock:get_time_microseconds()
    T.ok(finish >= start)
    T.ok(clock:get_time_nanoseconds() >= finish * 1000 - 1e6, "nanoseconds agree with microseconds")
    T.ok(clock:get_time_milliseconds() <= clock:get_time_microseconds() / 1000 + 1)
    T.ok(clock:get_time_seconds() >= 0)

    local copy = b.btClock(clock)
    T.ok(copy:get_time_microseconds() >= 0)
    clock:reset()
    T.ok(clock:get_time_microseconds() <= finish + 1e6)
end)

T.test("btSpinMutex: lock, try_lock and unlock", function()
    -- Bullet only builds a real spin lock with BT_THREADSAFE; otherwise these are no-ops that
    -- always succeed, so only the calls themselves are checked.
    local mutex = b.btSpinMutex()
    mutex:lock()
    mutex:unlock()
    T.type_is(mutex:try_lock(), "boolean")
    mutex:unlock()
end)

T.test("CProfileSample: a scoped profiling marker", function()
    T.ok(b.CProfileSample("test sample") ~= nil)
end)

T.test("btCpuFeatureUtility, btInfMaskConverter and btTypedObject", function()
    T.type_is(b.btCpuFeatureUtility.get_cpu_features(), "number")
    T.ok(b.btInfMaskConverter(7) ~= nil)
    T.ok(b.btInfMaskConverter() ~= nil)
    T.eq(b.btTypedObject(42):get_object_type(), 42)
end)

-- ---------------------------------------------------------------------------------------
-- Matrix decompositions
-- ---------------------------------------------------------------------------------------

T.test("btPolarDecomposition: splits a matrix into a rotation and a stretch", function()
    -- A = R S with R a quarter turn about z and S = diag(2, 3, 4)
    local a = b.btMatrix3x3(0, -3, 0, 2, 0, 0, 0, 0, 4)
    local u, h = b.btMatrix3x3(1, 0, 0, 0, 1, 0, 0, 0, 1), b.btMatrix3x3(1, 0, 0, 0, 1, 0, 0, 0, 1)
    local polar = b.btPolarDecomposition()
    local iterations = polar:decompose(a, u, h)
    T.ok(iterations >= 1)
    T.ok(iterations <= polar:max_iterations())

    T.vec3(u:get_row(0), 0, -1, 0, 1e-3)
    T.vec3(u:get_row(1), 1, 0, 0, 1e-3)
    T.vec3(u:get_row(2), 0, 0, 1, 1e-3)
    T.near(u:determinant(), 1, 1e-3)
    T.vec3(h:get_row(0), 2, 0, 0, 1e-3)
    T.vec3(h:get_row(1), 0, 3, 0, 1e-3)
    T.vec3(h:get_row(2), 0, 0, 4, 1e-3)
end)

T.test("btPolarDecomposition: tolerance and iteration limit", function()
    T.ok(b.btPolarDecomposition(1e-6):max_iterations() > 0)
    T.eq(b.btPolarDecomposition(1e-6, 3):max_iterations(), 3)
end)

T.test("GivensRotation: zeroes one entry of a column", function()
    local rotation = b.GivensRotation(0, 1)
    rotation:compute(3, 4)
    local a = b.btMatrix3x3(3, 1, 0, 4, 2, 0, 0, 0, 1)
    rotation:row_rotation(a)
    T.near(math.abs(a:get_row(0):x()), 5, 1e-4, "the length of (3, 4)")
    T.near(a:get_row(1):x(), 0, 1e-4)

    local r = b.btMatrix3x3(1, 0, 0, 0, 1, 0, 0, 0, 1)
    rotation:fill(r)
    T.near(r:determinant(), 1, 1e-4)
    T.near(r:get_row(2):z(), 1, 1e-6)

    local by_columns = b.btMatrix3x3(3, 4, 0, 1, 2, 0, 0, 0, 1)
    b.GivensRotation(0, 1):column_rotation(by_columns)
    local direct = b.GivensRotation(3, 4, 0, 1)
    direct:transpose_in_place()
    direct:compute_unconventional(1, 1)
    T.ok(b.btMatrix2x2() ~= nil)
    local small = b.btMatrix2x2()
    small:set_identity()
    direct:fill(small)
    direct:row_rotation(small)
    direct:column_rotation(small)
end)

-- ---------------------------------------------------------------------------------------
-- Spatial algebra
-- ---------------------------------------------------------------------------------------

T.test("btSpatialMotionVector and btSpatialForceVector: components", function()
    local motion = b.btSpatialMotionVector(vec(1, 2, 3), vec(4, 5, 6))
    T.vec3(motion:get_angular(), 1, 2, 3)
    T.vec3(motion:get_linear(), 4, 5, 6)

    motion:set_value(1, 0, 0, 0, 1, 0)
    T.vec3(motion:get_angular(), 1, 0, 0)
    T.vec3(motion:get_linear(), 0, 1, 0)
    motion:add_value(1, 0, 0, 0, 1, 0)
    T.vec3(motion:get_angular(), 2, 0, 0)
    T.vec3(motion:get_linear(), 0, 2, 0)
    motion:add_vector(vec(0, 0, 1), vec(0, 0, 1))
    T.vec3(motion:get_angular(), 2, 0, 1)
    motion:set_angular(vec(5, 5, 5))
    motion:set_linear(vec(6, 6, 6))
    motion:add_angular(vec(1, 0, 0))
    motion:add_linear(vec(0, 1, 0))
    T.vec3(motion:get_angular(), 6, 5, 5)
    T.vec3(motion:get_linear(), 6, 7, 6)
    motion:set_vector(vec(1, 1, 1), vec(2, 2, 2))
    T.vec3(motion:get_linear(), 2, 2, 2)
    motion:set_zero()
    T.vec3(motion:get_angular(), 0, 0, 0)
    T.vec3(motion:get_linear(), 0, 0, 0)

    local force = b.btSpatialForceVector(1, 2, 3, 4, 5, 6)
    T.vec3(force:get_angular(), 1, 2, 3)
    T.vec3(force:get_linear(), 4, 5, 6)
    force:set_zero()
    force:set_vector(vec(1, 0, 0), vec(0, 1, 0))
    force:add_vector(vec(1, 0, 0), vec(0, 1, 0))
    force:set_value(7, 0, 0, 0, 7, 0)
    force:add_value(1, 0, 0, 0, 1, 0)
    force:set_angular(vec(1, 0, 0))
    force:set_linear(vec(0, 1, 0))
    force:add_angular(vec(0, 0, 1))
    force:add_linear(vec(0, 0, 1))
    T.vec3(force:get_angular(), 1, 0, 1)
    T.vec3(force:get_linear(), 0, 1, 1)
    T.ok(b.btSpatialForceVector(vec(0, 0, 0), vec(0, 0, 0)) ~= nil)
    T.ok(b.btSpatialForceVector() ~= nil)
    T.ok(b.btSpatialMotionVector() ~= nil)
end)

T.test("btSpatialMotionVector: the dot product with a force is the work rate", function()
    local motion = b.btSpatialMotionVector(vec(1, 2, 3), vec(4, 5, 6))
    local force = b.btSpatialForceVector(vec(1, 0, 0), vec(0, 1, 0))
    T.near(motion:dot(force), 1 * 1 + 5 * 1)
    T.near(motion:dot(b.btSpatialForceVector(vec(0, 0, 0), vec(0, 0, 0))), 0)
end)

T.test("btSymmetricSpatialDyad: construction and editing", function()
    local identity = b.btMatrix3x3(1, 0, 0, 0, 1, 0, 0, 0, 1)
    local dyad = b.btSymmetricSpatialDyad(identity, identity, identity)
    dyad:set_identity()
    dyad:set_matrix(identity, identity, identity)
    dyad:add_matrix(identity, identity, identity)
    T.ok(b.btSymmetricSpatialDyad() ~= nil)
end)

-- ---------------------------------------------------------------------------------------
-- Small containers
-- ---------------------------------------------------------------------------------------

T.test("btGEN_Link and btGEN_List: an intrusive doubly linked list", function()
    local list = b.btGEN_List()
    T.ok(list:get_head():is_tail(), "an empty list starts at its end")

    local a, c = b.btGEN_Link(), b.btGEN_Link()
    T.ok(a:is_head() and a:is_tail(), "an unlinked link has neither neighbour")

    list:add_tail(a)
    T.ok(not list:get_head():is_tail())
    T.ok(list:get_head():get_next():is_tail(), "one element")
    list:add_tail(c)
    T.ok(not list:get_head():get_next():is_tail(), "two elements")

    c:remove()
    T.ok(list:get_head():get_next():is_tail())
    a:remove()
    T.ok(list:get_head():is_tail())

    list:add_head(a)
    c:insert_after(a)
    T.ok(not a:get_next():is_tail())
    c:remove()
    a:remove()
    T.ok(list:get_tail() ~= nil)
    T.ok(b.btGEN_Link(a, c) ~= nil)
end)

T.test("btReducedVector: a sparse vector", function()
    local zero = b.btReducedVector(4)
    T.near(zero:length2(), 0)
    T.near(zero:dot(b.btReducedVector(4)), 0)
    zero:simplify()
    zero:sort()
    T.ok(b.btReducedVector() ~= nil)
end)

T.run()
