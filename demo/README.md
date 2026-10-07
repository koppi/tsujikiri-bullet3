# Bullet3 + LuaBridge3 Demo

This demo shows how to use [tsujikiri](https://github.com/kunitoki/tsujikiri) to automatically generate Bullet3 bindings for LuaBridge3, featuring a physics simulation driven from Lua.

## Features

- **Automatic Binding Generation**: Uses upstream tsujikiri (installed from PyPI) and its built-in `luabridge3` format to analyze Bullet3 headers and generate LuaBridge3 bindings
- **CMake Integration**: Fetches Bullet3, LuaBridge3, and Lua automatically using FetchContent
- **Physics Simulation**: Demonstrates constraint solver, dynamics world, gravity manipulation and a falling rigid body
- **Code Formatting**: Generated bindings are formatted with the clang-format bundled with tsujikiri
- **External Lua Scripts**: Demo script is externalized to `bullet3_demo.lua` for easy modification

## Directory Structure

```
demo/
├── CMakeLists.txt                    # CMake build configuration
├── main.cpp                          # Demo application entry point
├── bullet3_demo.lua                  # External Lua physics simulation script
├── bullet3_headers.h                 # Header file for inspection
├── bullet3.input.yml.in              # tsujikiri input config template
├── requirements.txt                  # Pinned tsujikiri version
├── build_and_run.sh                  # Build and run script
└── README.md                         # This file
```

Note: `bullet3_bindings.cpp` is generated automatically during the build process.

## Building and Running

### Prerequisites

- CMake 3.28 or later
- C++17 compatible compiler
- Python 3.10 or later with `venv` module (for binding generation)

Note: tsujikiri is automatically installed in a virtual environment during the build process.

### Build and Run

```bash
./build_and_run.sh
```

Or manually:

```bash
mkdir -p build
cd build
cmake ..
make -j$(nproc)
./bullet3_demo
```

The CMake build system will automatically:
1. Fetch Bullet3, LuaBridge3, and Lua dependencies
2. Create a Python virtual environment and install tsujikiri from `requirements.txt`
3. Generate and format `bullet3_bindings.cpp` using tsujikiri during the build process
4. Compile and link everything into the final executable

## How It Works

1. **Configuration**: `bullet3.input.yml.in` tells tsujikiri which header to parse, which classes and methods to bind, and what to include in the generated file. CMake configures it into `build/bullet3.input.yml`, filling in the Bullet3 source path.

2. **Build-time Generation**: During the CMake build, tsujikiri runs as:
   ```bash
   python -m tsujikiri --input bullet3.input.yml --target luabridge3 bullet3_bindings.cpp --strict
   ```
   `--strict` makes the build fail if libclang reports any parse error.

3. **Integration**: The generated `register_bullet3(lua_State*)` function is called from `main.cpp`, which then runs `bullet3_demo.lua`. The program exits with a non-zero code if the script fails.

## Customization

You can customize the binding generation by modifying:

- **bullet3.input.yml.in**: Filters, transforms and generation settings (see the [tsujikiri input file reference](https://tsujikiri.readthedocs.io/en/latest/input-file-reference.html))
- **bullet3_headers.h**: Add/remove Bullet3 headers to inspect
- **requirements.txt**: Change the tsujikiri version

The bindings are regenerated automatically whenever these files change during the build.

`bullet3_demo.lua` is loaded at runtime, so changes to it don't need a rebuild.

## Generated Bindings

Every public class of `LinearMath`, `BulletCollision`, `BulletDynamics` and `BulletSoftBody` is bound (about 560 classes, including nested ones such as `btSoftBody::Node`), together with their inheritance chains. `bullet3_headers.h` includes all of those headers and `bullet3.input.yml.in` has no class whitelist.

All bindings are registered in the `bullet3` Lua namespace. Method names are converted from `camelCase` to `snake_case` (e.g. `setGravity` becomes `set_gravity`), and methods with default arguments can be called with any number of trailing arguments.

Not everything can be exposed to Lua, so the config leaves out:

- constructors of abstract classes, and of classes whose only constructors take raw pointer/array arguments (e.g. `btConvexHullShape`, `btSoftBody`)
- methods taking raw pointers to primitives, arrays, templates or function pointers, and methods Bullet declares but never defines
- nested classes whose name is shared with another nested class (`sResults`, `Range`, `CreateFunc`, `SwappedCreateFunc`, `Specs`), since all classes share one flat Lua namespace
- template specializations used as base classes (`btAlignedObjectArray<T>`, ...)

tsujikiri emits nested types unqualified in signatures, so `generation.prefix` in `bullet3.input.yml.in` declares a global alias for each unambiguous nested type (`using Node = btSoftBody::Node;`).

### Upgrading Bullet

Bullet is fetched from `master`. When the headers change, regenerate the include list and check that the bindings still compile:

```bash
cd demo/build/_deps/bullet3-src/src
find LinearMath BulletCollision BulletDynamics BulletSoftBody -name '*.h' | sort \
    | grep -v btReducedDeformableContactConstraint.h | sed 's/.*/#include <&>/'
```

(`btReducedDeformableContactConstraint.h` has no include guard and is already pulled in by `btReducedDeformableBodySolver.h`.) New nested types or unbindable methods show up as compile errors in `bullet3_bindings.cpp`; add an alias to `generation.prefix` or a blacklist entry to `bullet3.input.yml.in`.

## Demo Output

```
=== Bullet3 Physics Simulation Demo ===

1. Setting up physics world...
   - Created physics world with gravity: (0.0, -9.8100004196167, 0.0)

2. Testing physics simulation...
   - Time step: 0.016666666666667 seconds
   - Step 1: simulation ran with 1 sub-steps
   [...]

3. Testing constraint solver...
   - Random seed: 0
   - Set new random seed to 12345
   - New random seed: 12345
   - Random value 1: 87628868
   [...]

4. Testing dynamics world properties...
   - Current gravity: (0.0, -9.8100004196167, 0.0)
   - Number of constraints: 0
   - Changed to Mars gravity: (0.0, -3.710000038147, 0.0)

5. Testing simulation with Mars gravity...
   - Mars step 1: simulation ran with 1 sub-steps
   [...]

6. Testing solver operations...
   - Solver reset completed
   - All forces cleared from dynamics world
   - All physics simulation tests completed successfully!

=== Physics Simulation Demo Complete ===
```

## Example Lua Usage

```lua
-- Create a physics world
local constructionInfo = bullet3.btDefaultCollisionConstructionInfo()
local collisionConfig = bullet3.btDefaultCollisionConfiguration(constructionInfo)
local dispatcher = bullet3.btCollisionDispatcher(collisionConfig)
local broadphase = bullet3.btDbvtBroadphase()
local solver = bullet3.btSequentialImpulseConstraintSolver()
local dynamicsWorld = bullet3.btDiscreteDynamicsWorld(dispatcher, broadphase, solver, collisionConfig)

-- Set gravity
local gravity = bullet3.btVector3(0.0, -9.81, 0.0)
dynamicsWorld:set_gravity(gravity)

-- Step simulation (timeStep, maxSubSteps, fixedTimeStep)
local numSubSteps = dynamicsWorld:step_simulation(1.0/60.0, 10, 1.0/60.0)
```
