-- A small test library for the Bullet3 Lua bindings (no external dependencies).
--
--   local T = require("testlib")
--   T.test("btVector3: dot", function()
--       T.near(bullet3.btVector3(1, 2, 3):dot(bullet3.btVector3(4, 5, 6)), 32)
--   end)
--   T.run()   -- last line of the file; raises an error if any test failed
--
-- Environment variables:
--   BULLET3_TEST_VERBOSE=1       print a line for every passing test as well
--   BULLET3_TEST_FILTER=<text>   only run tests whose name contains <text> (plain match)

local T = {}

--- true when BULLET3_TEST_VERBOSE is set to something other than an empty string
local function env_flag(name)
    local value = os.getenv(name)
    return value ~= nil and value ~= ""
end
T.verbose = env_flag("BULLET3_TEST_VERBOSE")

-- Unbuffered, so the output up to a crash is not lost if a binding takes the process down
io.stdout:setvbuf("no")

local tests = {}
local cleanups

local function fail(message)
    -- level 3: skip `fail` and the assertion helper, point at the test's line
    error(message, 3)
end

local function show(value)
    if type(value) == "number" then
        return string.format("%.9g", value)
    elseif type(value) == "string" then
        return string.format("%q", value)
    end
    return tostring(value)
end

--- Register a test case.
function T.test(name, fn)
    tests[#tests + 1] = { name = name, fn = fn }
end

--- Run `fn` after the current test, whether it passed or not. Cleanups run last-in
--- first-out; use them to take bodies out of a world before Lua collects either.
function T.cleanup(fn)
    cleanups[#cleanups + 1] = fn
end

function T.ok(condition, message)
    if not condition then
        fail(message or "expected a truthy value, got " .. show(condition))
    end
end

function T.eq(actual, expected, message)
    if actual ~= expected then
        fail(string.format("%sexpected %s, got %s", message and (message .. ": ") or "", show(expected), show(actual)))
    end
end

function T.ne(actual, unexpected, message)
    if actual == unexpected then
        fail(string.format("%sdid not expect %s", message and (message .. ": ") or "", show(actual)))
    end
end

function T.near(actual, expected, tolerance, message)
    tolerance = tolerance or 1e-5
    if type(actual) ~= "number" or math.abs(actual - expected) > tolerance then
        fail(string.format("%sexpected %s +/- %s, got %s", message and (message .. ": ") or "", show(expected), show(tolerance), show(actual)))
    end
end

function T.is_nil(value, message)
    if value ~= nil then
        fail(string.format("%sexpected nil, got %s", message and (message .. ": ") or "", show(value)))
    end
end

function T.type_is(value, expected, message)
    if type(value) ~= expected then
        fail(string.format("%sexpected a %s, got a %s", message and (message .. ": ") or "", expected, type(value)))
    end
end

--- Assert that calling `fn` raises an error whose message contains `pattern` (plain text).
function T.throws(fn, pattern)
    local ok, err = pcall(fn)
    if ok then
        fail("expected an error, but the call succeeded")
    end
    if pattern and not tostring(err):find(pattern, 1, true) then
        fail(string.format("expected an error containing %s, got %s", show(pattern), show(tostring(err))))
    end
end

--- Compare a btVector3 against components.
function T.vec3(v, x, y, z, tolerance)
    tolerance = tolerance or 1e-5
    local ax, ay, az = v:x(), v:y(), v:z()
    if math.abs(ax - x) > tolerance or math.abs(ay - y) > tolerance or math.abs(az - z) > tolerance then
        fail(string.format("expected (%s, %s, %s) +/- %s, got (%s, %s, %s)",
            show(x), show(y), show(z), show(tolerance), show(ax), show(ay), show(az)))
    end
end

--- Compare a btQuaternion against components (x, y, z, w).
function T.quat(q, x, y, z, w, tolerance)
    tolerance = tolerance or 1e-5
    local ax, ay, az, aw = q:x(), q:y(), q:z(), q:w()
    if math.abs(ax - x) > tolerance or math.abs(ay - y) > tolerance
        or math.abs(az - z) > tolerance or math.abs(aw - w) > tolerance then
        fail(string.format("expected (%s, %s, %s, %s) +/- %s, got (%s, %s, %s, %s)",
            show(x), show(y), show(z), show(w), show(tolerance), show(ax), show(ay), show(az), show(aw)))
    end
end

--- Run every registered test, print a summary and raise an error if any failed.
function T.run()
    local verbose = T.verbose
    local filter = os.getenv("BULLET3_TEST_FILTER")
    if filter == "" then filter = nil end
    local failures = {}
    if filter then
        local selected = {}
        for _, case in ipairs(tests) do
            if case.name:find(filter, 1, true) then selected[#selected + 1] = case end
        end
        tests = selected
    end
    for _, case in ipairs(tests) do
        cleanups = {}
        if verbose then io.write("  ...   " .. case.name .. "\r") end
        local ok, err = pcall(case.fn)
        for i = #cleanups, 1, -1 do
            local cleaned, cleanup_err = pcall(cleanups[i])
            if not cleaned and ok then
                ok, err = false, "cleanup failed: " .. tostring(cleanup_err)
            end
        end
        cleanups = nil
        collectgarbage()
        if ok then
            if verbose then print("  ok    " .. case.name .. "      ") end
        else
            failures[#failures + 1] = case.name
            print("  FAIL  " .. case.name .. "\n        " .. tostring(err):gsub("\n", "\n        "))
        end
    end
    local total = #tests
    print(string.format("%d tests, %d passed, %d failed", total, total - #failures, #failures))
    tests = {}
    if #failures > 0 then
        error(string.format("%d of %d tests failed", #failures, total), 0)
    end
end

return T
