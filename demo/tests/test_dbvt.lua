-- Bounding volumes: btDbvtAabbMm, the dynamic AABB tree btDbvt, and the BVH classes.

local T = require("testlib")
local b = bullet3

local function vec(x, y, z)
    return b.btVector3(x, y, z)
end

local function box(min, max)
    return b.btDbvtAabbMm.from_mm(vec(min[1], min[2], min[3]), vec(max[1], max[2], max[3]))
end

-- ---------------------------------------------------------------------------------------
-- btDbvtAabbMm
-- ---------------------------------------------------------------------------------------

T.test("btDbvtAabbMm: the three ways to build a box", function()
    local from_mm = b.btDbvtAabbMm.from_mm(vec(-1, -2, -3), vec(1, 2, 3))
    T.vec3(from_mm:mins(), -1, -2, -3)
    T.vec3(from_mm:maxs(), 1, 2, 3)

    local from_ce = b.btDbvtAabbMm.from_ce(vec(10, 0, 0), vec(1, 2, 3))
    T.vec3(from_ce:mins(), 9, -2, -3)
    T.vec3(from_ce:maxs(), 11, 2, 3)

    local from_cr = b.btDbvtAabbMm.from_cr(vec(0, 5, 0), 2)
    T.vec3(from_cr:mins(), -2, 3, -2)
    T.vec3(from_cr:maxs(), 2, 7, 2)

    T.ok(b.btDbvtAabbMm() ~= nil)
end)

T.test("btDbvtAabbMm: centre, lengths and extents", function()
    local aabb = box({ 1, 2, 3 }, { 5, 8, 9 })
    T.vec3(aabb:center(), 3, 5, 6)
    T.vec3(aabb:lengths(), 4, 6, 6)
    T.vec3(aabb:extents(), 2, 3, 3)
end)

T.test("btDbvtAabbMm: expand and signed_expand", function()
    local aabb = box({ 0, 0, 0 }, { 1, 1, 1 })
    aabb:expand(vec(1, 2, 3))
    T.vec3(aabb:mins(), -1, -2, -3)
    T.vec3(aabb:maxs(), 2, 3, 4)

    -- signed_expand grows towards the sign of each component only
    local directed = box({ 0, 0, 0 }, { 1, 1, 1 })
    directed:signed_expand(vec(2, -3, 0))
    T.vec3(directed:mins(), 0, -3, 0)
    T.vec3(directed:maxs(), 3, 1, 1)
end)

T.test("btDbvtAabbMm: contain", function()
    local outer = box({ 0, 0, 0 }, { 10, 10, 10 })
    T.ok(outer:contain(box({ 1, 1, 1 }, { 2, 2, 2 })))
    T.ok(outer:contain(box({ 0, 0, 0 }, { 10, 10, 10 })))
    T.ok(not outer:contain(box({ 5, 5, 5 }, { 11, 6, 6 })))
    T.ok(not box({ 1, 1, 1 }, { 2, 2, 2 }):contain(outer))
end)

T.test("btDbvtAabbMm: classify against a plane", function()
    local aabb = box({ 1, 0, 0 }, { 2, 1, 1 })
    local normal = vec(1, 0, 0)
    -- plane n.x + offset = 0; the sign mask has a bit set for each positive component of the
    -- normal (x = 1, y = 2, z = 4) so that the box corners are picked the right way round
    T.eq(aabb:classify(normal, 0, 1), 1, "entirely in front")
    T.eq(aabb:classify(normal, -5, 1), -1, "entirely behind")
    T.eq(aabb:classify(normal, -1.5, 1), 0, "straddling")
end)

T.test("btDbvtAabbMm: project_minimum picks a corner by sign bits", function()
    local aabb = box({ -1, -2, -3 }, { 4, 5, 6 })
    local v = vec(1, 1, 1)
    T.near(aabb:project_minimum(v, 0), 4 + 5 + 6)
    T.near(aabb:project_minimum(v, 7), -1 - 2 - 3)
    T.near(aabb:project_minimum(v, 1), -1 + 5 + 6)
end)

T.test("btDbvtAabbMm: the transformed bounds accessors", function()
    local aabb = box({ 0, 0, 0 }, { 1, 1, 1 })
    T.vec3(aabb:t_mins(), 0, 0, 0)
    T.vec3(aabb:t_maxs(), 1, 1, 1)
    -- they alias the bounds
    aabb:t_maxs():set_x(5)
    T.near(aabb:maxs():x(), 5)
end)

-- ---------------------------------------------------------------------------------------
-- btDbvt
-- ---------------------------------------------------------------------------------------

T.test("btDbvt: insert and remove leaves", function()
    local tree = b.btDbvt()
    T.ok(tree:empty())

    local leaf = tree:insert(box({ 0, 0, 0 }, { 1, 1, 1 }), nil)
    T.ok(not tree:empty())
    T.ok(leaf:isleaf())
    T.ok(not leaf:isinternal())
    T.eq(b.btDbvt.count_leaves(leaf), 1)
    T.eq(b.btDbvt.maxdepth(leaf), 1, "a lone leaf is one level deep")

    local second = tree:insert(box({ 5, 5, 5 }, { 6, 6, 6 }), nil)
    T.ok(second:isleaf())

    tree:remove(leaf)
    T.ok(not tree:empty())
    tree:remove(second)
    T.ok(tree:empty())
end)

T.test("btDbvt: update reports whether the leaf had to move", function()
    local tree = b.btDbvt()
    local leaf = tree:insert(box({ 0, 0, 0 }, { 2, 2, 2 }), nil)

    local inside = box({ 0.5, 0.5, 0.5 }, { 1, 1, 1 })
    T.ok(not tree:update(leaf, inside, vec(0, 0, 0), 0), "still inside the old volume: nothing to do")

    local moved = box({ 10, 10, 10 }, { 11, 11, 11 })
    T.ok(tree:update(leaf, moved, vec(0, 0, 0), 0.1), "outside the old volume: the leaf was reinserted")
    T.ok(not tree:update(leaf, moved, vec(0, 0, 0), 0.1))
    T.ok(not tree:update(leaf, moved, vec(0, 0, 0)))
    T.ok(not tree:update(leaf, moved, 0.1))

    tree:update(leaf, box({ 20, 20, 20 }, { 21, 21, 21 }))
    tree:update(leaf, 1)
    tree:update(leaf)
    tree:remove(leaf)
    T.ok(tree:empty())
end)

T.test("btDbvt: a tree of many leaves can be restructured", function()
    local tree = b.btDbvt()
    local leaves = {}
    for i = 0, 63 do
        leaves[#leaves + 1] = tree:insert(box({ i, 0, 0 }, { i + 0.5, 1, 1 }), nil)
    end
    T.ok(not tree:empty())
    tree:optimize_bottom_up()
    tree:optimize_top_down()
    tree:optimize_top_down(8)
    tree:optimize_incremental(10)
    tree:optimize_incremental(-1)
    T.ok(not tree:empty())

    local writer_available = b.btDbvt.write ~= nil
    T.ok(writer_available)

    for _, leaf in ipairs(leaves) do tree:remove(leaf) end
    T.ok(tree:empty())

    tree:insert(box({ 0, 0, 0 }, { 1, 1, 1 }), nil)
    tree:clear()
    T.ok(tree:empty())
end)

T.test("btDbvtNode and btDbvntNode", function()
    local tree = b.btDbvt()
    local leaf = tree:insert(box({ 0, 0, 0 }, { 1, 1, 1 }), nil)
    local with_normals = b.btDbvntNode(leaf)
    T.ok(with_normals:isleaf())
    T.ok(not with_normals:isinternal())
    tree:remove(leaf)
end)

T.test("btDbvtProxy: a broadphase proxy with an AABB", function()
    -- (the unique id is only assigned when a broadphase creates the proxy)
    local proxy = b.btDbvtProxy(vec(0, 0, 0), vec(1, 1, 1), nil, 1, -1)
    T.ok(proxy ~= nil)

    local broadphase = b.btDbvtBroadphase()
    local config = b.btDefaultCollisionConfiguration()
    local dispatcher = b.btCollisionDispatcher(config)
    local first = broadphase:create_proxy(vec(0, 0, 0), vec(1, 1, 1), 8, nil, 1, -1, dispatcher)
    local second = broadphase:create_proxy(vec(0, 0, 0), vec(1, 1, 1), 8, nil, 1, -1, dispatcher)
    T.ne(first:get_uid(), second:get_uid(), "each proxy gets its own id")
    broadphase:destroy_proxy(first, dispatcher)
    broadphase:destroy_proxy(second, dispatcher)
end)

-- ---------------------------------------------------------------------------------------
-- BVH classes
-- ---------------------------------------------------------------------------------------

T.test("btOptimizedBvh: built from a triangle mesh", function()
    local mesh = b.btTriangleMesh()
    mesh:add_triangle(vec(0, 0, 0), vec(1, 0, 0), vec(0, 1, 0))
    mesh:add_triangle(vec(1, 1, 0), vec(1, 0, 0), vec(0, 1, 0))

    local bvh = b.btOptimizedBvh()
    bvh:build(mesh, true, vec(-1, -1, -1), vec(2, 2, 2))
    T.ok(bvh:is_quantized())

    local plain = b.btOptimizedBvh()
    plain:build(mesh, false, vec(-1, -1, -1), vec(2, 2, 2))
    T.ok(not plain:is_quantized())

    -- refitting after the mesh changed keeps the tree valid
    bvh:refit(mesh, vec(-1, -1, -1), vec(2, 2, 2))
    bvh:refit_partial(mesh, vec(-1, -1, -1), vec(2, 2, 2))
end)

T.test("btQuantizedBvh: quantization setup", function()
    local bvh = b.btQuantizedBvh()
    bvh:set_quantization_values(vec(-1, -1, -1), vec(1, 1, 1))
    bvh:set_quantization_values(vec(-1, -1, -1), vec(1, 1, 1), 0.5)
    T.type_is(b.btQuantizedBvh.get_alignment_serialization_padding(), "number")
    T.type_is(bvh:is_quantized(), "boolean")
end)

T.test("btBroadphasePair: pairs of proxies", function()
    -- the constructor orders the pair by the proxies' ids, so they must come from a broadphase
    local broadphase = b.btDbvtBroadphase()
    local config = b.btDefaultCollisionConfiguration()
    local dispatcher = b.btCollisionDispatcher(config)
    local first = broadphase:create_proxy(vec(0, 0, 0), vec(1, 1, 1), 8, nil, 1, -1, dispatcher)
    local second = broadphase:create_proxy(vec(0, 0, 0), vec(1, 1, 1), 8, nil, 1, -1, dispatcher)
    T.ok(b.btBroadphasePair() ~= nil)
    T.ok(b.btBroadphasePair(first, second) ~= nil)
    broadphase:destroy_proxy(first, dispatcher)
    broadphase:destroy_proxy(second, dispatcher)
end)

T.run()
