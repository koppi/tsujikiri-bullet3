-- LinearMath: vectors, quaternions, matrices, transforms and motion states.
-- Expected values are worked out by hand; btScalar is a float, hence the tolerances.

local T = require("testlib")
local b = bullet3

local HALF_PI = math.pi / 2
local SQRT_HALF = math.sqrt(0.5)

local function identity_transform()
    local t = b.btTransform()
    t:set_identity()
    return t
end

-- ---------------------------------------------------------------------------------------
-- btVector3
-- ---------------------------------------------------------------------------------------

-- Bullet's default constructors (btVector3(), btMatrix3x3(), btTransform(), ...) leave the
-- components uninitialised, so the tests always set a value before reading one.
T.test("btVector3: constructors and accessors", function()
    local v = b.btVector3(1, 2, 3)
    T.vec3(v, 1, 2, 3)
    T.eq(v:get_x(), 1)
    T.eq(v:get_y(), 2)
    T.eq(v:get_z(), 3)
    T.eq(v:w(), 0)
end)

T.test("btVector3: setters", function()
    local v = b.btVector3()
    v:set_x(4)
    v:set_y(5)
    v:set_z(6)
    T.vec3(v, 4, 5, 6)
    v:set_w(7)
    T.eq(v:w(), 7)
    v:set_value(1, 2, 3)
    T.vec3(v, 1, 2, 3)
    v:set_zero()
    T.vec3(v, 0, 0, 0)
end)

T.test("btVector3: dot, cross and triple product", function()
    local a, c = b.btVector3(1, 2, 3), b.btVector3(4, 5, 6)
    T.near(a:dot(c), 32)
    T.vec3(a:cross(c), -3, 6, -3)
    T.vec3(b.btVector3(1, 0, 0):cross(b.btVector3(0, 1, 0)), 0, 0, 1)
    T.near(b.btVector3(1, 0, 0):triple(b.btVector3(0, 1, 0), b.btVector3(0, 0, 1)), 1)
    T.near(b.btVector3(0, 1, 0):triple(b.btVector3(1, 0, 0), b.btVector3(0, 0, 1)), -1)
end)

T.test("btVector3: length and distance", function()
    local v = b.btVector3(1, 2, 2)
    T.near(v:length2(), 9)
    T.near(v:length(), 3)
    T.near(v:norm(), 3)
    T.near(v:safe_norm(), 3)
    T.near(v:distance2(b.btVector3(1, 2, 5)), 9)
    T.near(v:distance(b.btVector3(1, 2, 5)), 3)
end)

T.test("btVector3: normalize works in place, normalized returns a copy", function()
    local v = b.btVector3(0, 3, 4)
    T.vec3(v:normalized(), 0, 0.6, 0.8)
    T.vec3(v, 0, 3, 4)
    local returned = v:normalize()
    T.vec3(v, 0, 0.6, 0.8)
    T.vec3(returned, 0, 0.6, 0.8)
end)

T.test("btVector3: safe_normalize leaves a zero vector usable", function()
    local v = b.btVector3(0, 0, 0)
    v:safe_normalize()
    T.near(v:length(), 1)
    local w = b.btVector3(0, 0, 2)
    w:safe_normalize()
    T.vec3(w, 0, 0, 1)
end)

T.test("btVector3: rotate and angle", function()
    T.vec3(b.btVector3(1, 0, 0):rotate(b.btVector3(0, 0, 1), HALF_PI), 0, 1, 0, 1e-5)
    T.vec3(b.btVector3(0, 1, 0):rotate(b.btVector3(1, 0, 0), math.pi), 0, -1, 0, 1e-5)
    T.near(b.btVector3(1, 0, 0):angle(b.btVector3(0, 1, 0)), HALF_PI)
    T.near(b.btVector3(1, 0, 0):angle(b.btVector3(-1, 0, 0)), math.pi)
    T.near(b.btVector3(1, 1, 0):angle(b.btVector3(1, 0, 0)), math.pi / 4)
end)

T.test("btVector3: absolute and axis queries", function()
    T.vec3(b.btVector3(-1, 2, -3):absolute(), 1, 2, 3)
    local v = b.btVector3(-5, 1, 2)
    T.eq(v:min_axis(), 0, "axis of the smallest component")
    T.eq(v:max_axis(), 2, "axis of the largest component")
    T.eq(v:furthest_axis(), 1, "axis of the smallest magnitude")
    T.eq(v:closest_axis(), 0, "axis of the largest magnitude")
end)

T.test("btVector3: lerp and set_interpolate3", function()
    local a, c = b.btVector3(0, 0, 0), b.btVector3(10, 20, 30)
    T.vec3(a:lerp(c, 0.5), 5, 10, 15)
    T.vec3(a:lerp(c, 0), 0, 0, 0)
    T.vec3(a:lerp(c, 1), 10, 20, 30)
    local out = b.btVector3()
    out:set_interpolate3(a, c, 0.25)
    T.vec3(out, 2.5, 5, 7.5)
end)

T.test("btVector3: set_max and set_min", function()
    local v = b.btVector3(1, 5, 3)
    v:set_max(b.btVector3(2, 2, 2))
    T.vec3(v, 2, 5, 3)
    v:set_min(b.btVector3(0, 4, 4))
    T.vec3(v, 0, 4, 3)
end)

T.test("btVector3: is_zero and fuzzy_zero", function()
    T.ok(b.btVector3(0, 0, 0):is_zero())
    T.ok(not b.btVector3(0, 0, 1e-9):is_zero())
    T.ok(b.btVector3(0, 0, 1e-9):fuzzy_zero())
    T.ok(not b.btVector3(0, 0, 1):fuzzy_zero())
end)

T.test("btVector3: dot3 returns the three dot products", function()
    local v = b.btVector3(1, 2, 3)
    T.vec3(v:dot3(b.btVector3(1, 0, 0), b.btVector3(0, 1, 0), b.btVector3(0, 0, 1)), 1, 2, 3)
    T.vec3(v:dot3(b.btVector3(1, 1, 1), b.btVector3(2, 0, 0), b.btVector3(0, 0, -1)), 6, 2, -3)
end)

T.test("btVector3: get_skew_symmetric_matrix fills three row vectors", function()
    local r0, r1, r2 = b.btVector3(), b.btVector3(), b.btVector3()
    b.btVector3(1, 2, 3):get_skew_symmetric_matrix(r0, r1, r2)
    T.vec3(r0, 0, -3, 2)
    T.vec3(r1, 3, 0, -1)
    T.vec3(r2, -2, 1, 0)
end)

T.test("btVector4: fourth component", function()
    local v = b.btVector4(1, -2, 3, -4)
    T.eq(v:get_w(), -4)
    T.eq(v:x(), 1)
    local abs = v:absolute4()
    T.vec3(abs, 1, 2, 3)
    T.eq(abs:w(), 4)
    T.eq(v:max_axis4(), 2)
    T.eq(v:min_axis4(), 3)
    T.eq(v:closest_axis4(), 3)
    v:set_value(5, 6, 7, 8)
    T.vec3(v, 5, 6, 7)
    T.eq(v:get_w(), 8)
    -- inherits the btVector3 functions
    T.near(b.btVector4(3, 4, 0, 0):length(), 5)
end)

T.test("btQuadWord: components", function()
    local q = b.btQuadWord(1, 2, 3, 4)
    T.eq(q:x(), 1)
    T.eq(q:y(), 2)
    T.eq(q:z(), 3)
    T.eq(q:w(), 4)
    T.eq(q:get_x(), 1)
    T.eq(q:get_y(), 2)
    T.eq(q:get_z(), 3)
    q:set_x(5) q:set_y(6) q:set_z(7) q:set_w(8)
    T.eq(q:x() + q:y() + q:z() + q:w(), 26)
    q:set_value(1, 2, 3)
    T.eq(q:z(), 3)
    q:set_value(1, 2, 3, 4)
    T.eq(q:w(), 4)
    q:set_max(b.btQuadWord(0, 9, 0, 9))
    T.eq(q:y(), 9)
    T.eq(q:x(), 1)
    q:set_min(b.btQuadWord(0, 5, 5, 5))
    T.eq(q:x(), 0)
    T.eq(q:y(), 5)
    T.eq(b.btQuadWord(1, 2, 3):w(), 0)
end)

-- ---------------------------------------------------------------------------------------
-- btQuaternion
-- ---------------------------------------------------------------------------------------

T.test("btQuaternion: constructors", function()
    T.quat(b.btQuaternion(0.1, 0.2, 0.3, 0.4), 0.1, 0.2, 0.3, 0.4)
    T.quat(b.btQuaternion(b.btVector3(0, 0, 1), HALF_PI), 0, 0, SQRT_HALF, SQRT_HALF)
    -- yaw, pitch, roll: yaw turns about the y axis
    T.quat(b.btQuaternion(HALF_PI, 0, 0), 0, SQRT_HALF, 0, SQRT_HALF)
    T.quat(b.btQuaternion(0, HALF_PI, 0), SQRT_HALF, 0, 0, SQRT_HALF)
    T.quat(b.btQuaternion(0, 0, HALF_PI), 0, 0, SQRT_HALF, SQRT_HALF)
    T.quat(b.btQuaternion.get_identity(), 0, 0, 0, 1)
end)

T.test("btQuaternion: set_rotation and the Euler setters", function()
    local q = b.btQuaternion(0, 0, 0, 1)
    q:set_rotation(b.btVector3(1, 0, 0), HALF_PI)
    T.quat(q, SQRT_HALF, 0, 0, SQRT_HALF)
    q:set_euler(HALF_PI, 0, 0)
    T.quat(q, 0, SQRT_HALF, 0, SQRT_HALF)
    q:set_euler_zyx(HALF_PI, 0, 0)
    T.quat(q, 0, 0, SQRT_HALF, SQRT_HALF)
    q:set_euler_zyx(0, HALF_PI, 0)
    T.quat(q, 0, SQRT_HALF, 0, SQRT_HALF)
    q:set_euler_zyx(0, 0, HALF_PI)
    T.quat(q, SQRT_HALF, 0, 0, SQRT_HALF)
end)

T.test("btQuaternion: length, dot and normalisation", function()
    local q = b.btQuaternion(0, 0, 3, 4)
    T.near(q:length2(), 25)
    T.near(q:length(), 5)
    T.near(q:dot(b.btQuaternion(1, 1, 1, 1)), 7)
    T.quat(q:normalized(), 0, 0, 0.6, 0.8)
    T.near(q:length(), 5, 1e-5, "normalized leaves the original alone")
    local returned = q:normalize()
    T.quat(q, 0, 0, 0.6, 0.8)
    T.quat(returned, 0, 0, 0.6, 0.8)
    -- safe_normalize skips a quaternion that is too short to normalise
    local zero = b.btQuaternion(0, 0, 0, 0)
    zero:safe_normalize()
    T.near(zero:length(), 0)
    local long = b.btQuaternion(0, 0, 0, 2)
    long:safe_normalize()
    T.quat(long, 0, 0, 0, 1)
end)

T.test("btQuaternion: angle and axis", function()
    local q = b.btQuaternion(b.btVector3(0, 1, 0), HALF_PI)
    T.near(q:get_angle(), HALF_PI)
    T.near(q:get_angle_shortest_path(), HALF_PI)
    local axis = q:get_axis()
    T.vec3(axis, 0, 1, 0)
    T.near(q:get_w(), SQRT_HALF)
    -- Bullet's angle() is the *half* angle between two quaternions, angle_shortest_path() the full one
    T.near(b.btQuaternion.get_identity():angle(q), HALF_PI / 2)
    T.near(b.btQuaternion.get_identity():angle_shortest_path(q), HALF_PI)
    T.near(q:angle(q), 0, 1e-3)
    T.near(q:angle_shortest_path(q), 0, 1e-3)
end)

T.test("btQuaternion: inverse is the conjugate", function()
    local q = b.btQuaternion(b.btVector3(0, 0, 1), HALF_PI)
    T.quat(q:inverse(), 0, 0, -SQRT_HALF, SQRT_HALF)
    local m = b.btMatrix3x3(q)
    local back = b.btMatrix3x3(q:inverse())
    local product = m:times_transpose(back:transpose())
    T.near(product:determinant(), 1)
    T.vec3(product:get_row(0), 1, 0, 0)
end)

T.test("btQuaternion: slerp, nearest and farthest", function()
    local identity = b.btQuaternion(0, 0, 0, 1)
    local q = b.btQuaternion(b.btVector3(0, 0, 1), HALF_PI)
    T.near(identity:slerp(q, 0.5):get_angle(), HALF_PI / 2)
    T.quat(identity:slerp(q, 0), 0, 0, 0, 1)
    T.quat(identity:slerp(q, 1), 0, 0, SQRT_HALF, SQRT_HALF)

    local negated = b.btQuaternion(0, 0, -SQRT_HALF, -SQRT_HALF)
    T.ok(identity:nearest(negated):dot(identity) > 0, "nearest has the same hemisphere")
    T.ok(identity:farthest(negated):dot(identity) < 0, "farthest has the opposite hemisphere")
end)

-- ---------------------------------------------------------------------------------------
-- btMatrix3x3
-- ---------------------------------------------------------------------------------------

local function expect_rows(m, r0, r1, r2, tolerance)
    T.vec3(m:get_row(0), r0[1], r0[2], r0[3], tolerance)
    T.vec3(m:get_row(1), r1[1], r1[2], r1[3], tolerance)
    T.vec3(m:get_row(2), r2[1], r2[2], r2[3], tolerance)
end

T.test("btMatrix3x3: constructors", function()
    expect_rows(b.btMatrix3x3(1, 2, 3, 4, 5, 6, 7, 8, 9), { 1, 2, 3 }, { 4, 5, 6 }, { 7, 8, 9 })
    expect_rows(b.btMatrix3x3(b.btVector3(1, 2, 3), b.btVector3(4, 5, 6), b.btVector3(7, 8, 9)),
        { 1, 2, 3 }, { 4, 5, 6 }, { 7, 8, 9 })
    expect_rows(b.btMatrix3x3(b.btMatrix3x3(1, 2, 3, 4, 5, 6, 7, 8, 9)), { 1, 2, 3 }, { 4, 5, 6 }, { 7, 8, 9 })
    expect_rows(b.btMatrix3x3(b.btQuaternion(0, 0, 0, 1)), { 1, 0, 0 }, { 0, 1, 0 }, { 0, 0, 1 })
    expect_rows(b.btMatrix3x3(b.btQuaternion(b.btVector3(0, 0, 1), HALF_PI)),
        { 0, -1, 0 }, { 1, 0, 0 }, { 0, 0, 1 })
end)

T.test("btMatrix3x3: rows and columns", function()
    local m = b.btMatrix3x3(1, 2, 3, 4, 5, 6, 7, 8, 9)
    T.vec3(m:get_row(1), 4, 5, 6)
    T.vec3(m:get_column(1), 2, 5, 8)
    T.vec3(m:get_column(2), 3, 6, 9)
end)

T.test("btMatrix3x3: setters", function()
    local m = b.btMatrix3x3()
    m:set_value(1, 2, 3, 4, 5, 6, 7, 8, 9)
    expect_rows(m, { 1, 2, 3 }, { 4, 5, 6 }, { 7, 8, 9 })
    m:set_identity()
    expect_rows(m, { 1, 0, 0 }, { 0, 1, 0 }, { 0, 0, 1 })
    m:set_zero()
    expect_rows(m, { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 })
    m:set_rotation(b.btQuaternion(b.btVector3(1, 0, 0), HALF_PI))
    expect_rows(m, { 1, 0, 0 }, { 0, 0, -1 }, { 0, 1, 0 }, 1e-6)
    expect_rows(b.btMatrix3x3.get_identity(), { 1, 0, 0 }, { 0, 1, 0 }, { 0, 0, 1 })
end)

T.test("btMatrix3x3: Euler angles round-trip through get_rotation", function()
    local m = b.btMatrix3x3()
    m:set_euler_zyx(0, 0, HALF_PI) -- about z
    expect_rows(m, { 0, -1, 0 }, { 1, 0, 0 }, { 0, 0, 1 }, 1e-6)
    local q = b.btQuaternion(0, 0, 0, 1)
    m:get_rotation(q)
    T.quat(q, 0, 0, SQRT_HALF, SQRT_HALF)

    -- the yaw/pitch/roll setter turns about z, then y, then x
    m:set_euler_ypr(HALF_PI, 0, 0)
    expect_rows(m, { 0, -1, 0 }, { 1, 0, 0 }, { 0, 0, 1 }, 1e-6)
    m:set_euler_ypr(0, HALF_PI, 0)
    expect_rows(m, { 0, 0, 1 }, { 0, 1, 0 }, { -1, 0, 0 }, 1e-6)
    m:set_euler_ypr(0, 0, HALF_PI)
    expect_rows(m, { 1, 0, 0 }, { 0, 0, -1 }, { 0, 1, 0 }, 1e-6)
end)

T.test("btMatrix3x3: determinant, transpose, absolute, scaled", function()
    local m = b.btMatrix3x3(1, 2, 3, 0, 1, 4, 5, 6, 0)
    T.near(m:determinant(), 1)
    T.near(b.btMatrix3x3(2, 0, 0, 0, 3, 0, 0, 0, 4):determinant(), 24)
    expect_rows(m:transpose(), { 1, 0, 5 }, { 2, 1, 6 }, { 3, 4, 0 })
    expect_rows(b.btMatrix3x3(-1, 2, -3, 4, -5, 6, -7, 8, -9):absolute(), { 1, 2, 3 }, { 4, 5, 6 }, { 7, 8, 9 })
    -- scaled multiplies column j by s[j]
    expect_rows(b.btMatrix3x3(1, 2, 3, 4, 5, 6, 7, 8, 9):scaled(b.btVector3(2, 3, 4)),
        { 2, 6, 12 }, { 8, 15, 24 }, { 14, 24, 36 })
end)

T.test("btMatrix3x3: inverse, adjoint and solve33", function()
    local m = b.btMatrix3x3(1, 2, 3, 0, 1, 4, 5, 6, 0)
    local inverse = { { -24, 18, 5 }, { 20, -15, -4 }, { -5, 4, 1 } }
    expect_rows(m:inverse(), inverse[1], inverse[2], inverse[3], 1e-3)
    expect_rows(m:adjoint(), inverse[1], inverse[2], inverse[3], 1e-3) -- the determinant is 1
    local x = m:solve33(b.btVector3(14, 14, 17)) -- m * (1, 2, 3)
    T.vec3(x, 1, 2, 3, 1e-4)
end)

T.test("btMatrix3x3: products with transposes and tdot", function()
    local m = b.btMatrix3x3(1, 2, 3, 4, 5, 6, 7, 8, 9)
    local n = b.btMatrix3x3(1, 0, 0, 0, 2, 0, 0, 0, 3)
    -- m^T * n
    expect_rows(m:transpose_times(n), { 1, 8, 21 }, { 2, 10, 24 }, { 3, 12, 27 })
    -- m * n^T
    expect_rows(m:times_transpose(n), { 1, 4, 9 }, { 4, 10, 18 }, { 7, 16, 27 })
    local v = b.btVector3(1, 1, 1)
    T.near(m:tdotx(v), 1 + 4 + 7)
    T.near(m:tdoty(v), 2 + 5 + 8)
    T.near(m:tdotz(v), 3 + 6 + 9)
    -- cofactor of the 2x2 minor taken from rows 1,2 and columns 1,2: 5*9 - 6*8
    T.near(m:cofac(1, 1, 2, 2), -3)
end)

T.test("btMatrix3x3: extract_rotation recovers the rotation", function()
    local m = b.btMatrix3x3(b.btQuaternion(b.btVector3(0, 0, 1), HALF_PI))
    local q = b.btQuaternion(0, 0, 0, 1)
    m:extract_rotation(q, 1e-9, 100)
    T.near(math.abs(q:z()), SQRT_HALF, 1e-4)
    T.near(math.abs(q:w()), SQRT_HALF, 1e-4)
    T.near(q:length(), 1, 1e-4)
end)

T.test("btMatrix3x3: diagonalize a symmetric matrix", function()
    local m = b.btMatrix3x3(2, 1, 0, 1, 2, 0, 0, 0, 3)
    local rotation = b.btMatrix3x3()
    m:diagonalize(rotation, 1e-6, 20)
    T.near(m:get_row(0):y(), 0, 1e-4)
    T.near(m:get_row(0):z(), 0, 1e-4)
    T.near(m:get_row(1):z(), 0, 1e-4)
    local trace = m:get_row(0):x() + m:get_row(1):y() + m:get_row(2):z()
    T.near(trace, 7, 1e-4)
    T.near(m:determinant(), 9, 1e-3) -- the eigenvalues are 1, 3 and 3
    T.near(rotation:determinant(), 1, 1e-4)
end)

-- ---------------------------------------------------------------------------------------
-- btTransform
-- ---------------------------------------------------------------------------------------

T.test("btTransform: constructors", function()
    local q = b.btQuaternion(b.btVector3(0, 0, 1), HALF_PI)
    local origin = b.btVector3(1, 2, 3)

    local full = b.btTransform(q, origin)
    T.vec3(full:get_origin(), 1, 2, 3)
    T.quat(full:get_rotation(), 0, 0, SQRT_HALF, SQRT_HALF)

    local rotation_only = b.btTransform(q)
    T.vec3(rotation_only:get_origin(), 0, 0, 0)

    local from_matrix = b.btTransform(b.btMatrix3x3(q), origin)
    T.vec3(from_matrix:get_origin(), 1, 2, 3)
    T.quat(from_matrix:get_rotation(), 0, 0, SQRT_HALF, SQRT_HALF)

    local from_basis = b.btTransform(b.btMatrix3x3(q))
    T.vec3(from_basis:get_origin(), 0, 0, 0)

    local copy = b.btTransform(full)
    T.vec3(copy:get_origin(), 1, 2, 3)
    copy:set_origin(b.btVector3(9, 9, 9))
    T.vec3(full:get_origin(), 1, 2, 3)
end)

T.test("btTransform: setters", function()
    local t = identity_transform()
    T.vec3(t:get_origin(), 0, 0, 0)
    T.quat(t:get_rotation(), 0, 0, 0, 1)
    t:set_origin(b.btVector3(1, 2, 3))
    T.vec3(t:get_origin(), 1, 2, 3)
    t:set_rotation(b.btQuaternion(b.btVector3(0, 1, 0), HALF_PI))
    T.quat(t:get_rotation(), 0, SQRT_HALF, 0, SQRT_HALF)
    t:set_basis(b.btMatrix3x3(1, 0, 0, 0, 1, 0, 0, 0, 1))
    T.quat(t:get_rotation(), 0, 0, 0, 1)
    T.vec3(t:get_basis():get_row(1), 0, 1, 0)
    t:set_identity()
    T.vec3(t:get_origin(), 0, 0, 0)
    local id = b.btTransform.get_identity()
    T.vec3(id:get_origin(), 0, 0, 0)
    T.quat(id:get_rotation(), 0, 0, 0, 1)
end)

T.test("btTransform: mult composes two transforms", function()
    local translate = b.btTransform(b.btQuaternion(0, 0, 0, 1), b.btVector3(1, 0, 0))
    local turn = b.btTransform(b.btQuaternion(b.btVector3(0, 0, 1), HALF_PI), b.btVector3(0, 1, 0))
    local product = identity_transform()
    product:mult(translate, turn)
    T.vec3(product:get_origin(), 1, 1, 0)
    T.quat(product:get_rotation(), 0, 0, SQRT_HALF, SQRT_HALF)

    -- the other order rotates the translation as well
    product:mult(turn, translate)
    T.vec3(product:get_origin(), 0, 2, 0, 1e-5)
end)

T.test("btTransform: inverse, inv_xform and inverse_times", function()
    local t = b.btTransform(b.btQuaternion(b.btVector3(0, 0, 1), HALF_PI), b.btVector3(1, 2, 3))
    -- the world-space point (1, 2, 3) is the local origin
    T.vec3(t:inv_xform(b.btVector3(1, 2, 3)), 0, 0, 0)
    -- local x axis maps to world (1, 3, 3), and back
    T.vec3(t:inv_xform(b.btVector3(1, 3, 3)), 1, 0, 0, 1e-5)

    local inverse = t:inverse()
    T.vec3(inverse:get_origin(), -2, 1, -3, 1e-5)
    T.quat(inverse:get_rotation(), 0, 0, -SQRT_HALF, SQRT_HALF)

    local relative = t:inverse_times(t)
    T.vec3(relative:get_origin(), 0, 0, 0, 1e-5)
    T.quat(relative:get_rotation(), 0, 0, 0, 1, 1e-5)

    local other = b.btTransform(b.btQuaternion(0, 0, 0, 1), b.btVector3(1, 2, 4))
    T.vec3(t:inverse_times(other):get_origin(), 0, 0, 1, 1e-5)
end)

-- ---------------------------------------------------------------------------------------
-- btTransformUtil and btConvexSeparatingDistanceUtil
-- ---------------------------------------------------------------------------------------

T.test("btTransformUtil: integrate_transform moves and turns a transform", function()
    local start = b.btTransform(b.btQuaternion(0, 0, 0, 1), b.btVector3(1, 0, 0))
    local predicted = identity_transform()
    -- one second at 0.5 rad/s about z (larger steps are clamped by Bullet)
    b.btTransformUtil.integrate_transform(start, b.btVector3(2, 0, 0), b.btVector3(0, 0, 0.5), 1, predicted)
    T.vec3(predicted:get_origin(), 3, 0, 0)
    T.near(predicted:get_rotation():get_angle(), 0.5, 1e-3)
    T.vec3(predicted:get_rotation():get_axis(), 0, 0, 1, 1e-3)

    -- no motion leaves the transform alone
    b.btTransformUtil.integrate_transform(start, b.btVector3(0, 0, 0), b.btVector3(0, 0, 0), 1, predicted)
    T.vec3(predicted:get_origin(), 1, 0, 0)
    T.quat(predicted:get_rotation(), 0, 0, 0, 1)
end)

T.test("btTransformUtil: calculate_velocity recovers the motion", function()
    local t0 = b.btTransform(b.btQuaternion(0, 0, 0, 1), b.btVector3(0, 0, 0))
    local t1 = b.btTransform(b.btQuaternion(b.btVector3(0, 0, 1), HALF_PI), b.btVector3(2, 0, 0))
    local lin, ang = b.btVector3(), b.btVector3()
    b.btTransformUtil.calculate_velocity(t0, t1, 2, lin, ang)
    T.vec3(lin, 1, 0, 0)
    T.vec3(ang, 0, 0, HALF_PI / 2, 1e-4)
end)

T.test("btTransformUtil: calculate_velocity_quaternion matches calculate_velocity", function()
    local lin, ang = b.btVector3(), b.btVector3()
    b.btTransformUtil.calculate_velocity_quaternion(
        b.btVector3(0, 0, 0), b.btVector3(0, 3, 0),
        b.btQuaternion(0, 0, 0, 1), b.btQuaternion(b.btVector3(0, 1, 0), HALF_PI), 1, lin, ang)
    T.vec3(lin, 0, 3, 0)
    T.vec3(ang, 0, HALF_PI, 0, 1e-4)
end)

T.test("btConvexSeparatingDistanceUtil: conservative separating distance", function()
    local util = b.btConvexSeparatingDistanceUtil(1, 1)
    local at_origin = b.btTransform(b.btQuaternion(0, 0, 0, 1), b.btVector3(0, 0, 0))
    local far_away = b.btTransform(b.btQuaternion(0, 0, 0, 1), b.btVector3(10, 0, 0))
    util:init_separating_distance(b.btVector3(1, 0, 0), 8, at_origin, far_away)
    T.near(util:get_conservative_separating_distance(), 8)
    util:update_separating_distance(at_origin, far_away)
    T.near(util:get_conservative_separating_distance(), 8)
    -- the distance shrinks by the motion of B relative to A along the separating vector:
    -- A moving 3 units against the vector closes the gap by 3
    local moved = b.btTransform(b.btQuaternion(0, 0, 0, 1), b.btVector3(-3, 0, 0))
    util:update_separating_distance(moved, far_away)
    T.near(util:get_conservative_separating_distance(), 5, 1e-4)
end)

-- ---------------------------------------------------------------------------------------
-- btMotionState / btDefaultMotionState
-- ---------------------------------------------------------------------------------------

T.test("btDefaultMotionState: stores and reports the world transform", function()
    local start = b.btTransform(b.btQuaternion(0, 0, 0, 1), b.btVector3(1, 2, 3))
    local state = b.btDefaultMotionState(start)
    local out = identity_transform()
    state:get_world_transform(out)
    T.vec3(out:get_origin(), 1, 2, 3)

    state:set_world_transform(b.btTransform(b.btQuaternion(0, 0, 0, 1), b.btVector3(4, 5, 6)))
    state:get_world_transform(out)
    T.vec3(out:get_origin(), 4, 5, 6)
end)

T.test("btDefaultMotionState: the centre of mass offset is applied", function()
    local start = b.btTransform(b.btQuaternion(0, 0, 0, 1), b.btVector3(0, 0, 0))
    local offset = b.btTransform(b.btQuaternion(0, 0, 0, 1), b.btVector3(0, 1, 0))
    local state = b.btDefaultMotionState(start, offset)
    local out = identity_transform()
    state:get_world_transform(out)
    T.vec3(out:get_origin(), 0, -1, 0)

    local default = b.btDefaultMotionState()
    default:get_world_transform(out)
    T.vec3(out:get_origin(), 0, 0, 0)
    T.quat(out:get_rotation(), 0, 0, 0, 1)
end)

T.run()
