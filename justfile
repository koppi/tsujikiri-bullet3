build_dir := "demo/build"

# List all targets
default:
    @just --list

# Configure and compile the demo
build:
    cmake -S demo -B {{build_dir}}
    cmake --build {{build_dir}} --parallel

# Compile and run the demo
run: build
    {{build_dir}}/bullet3_demo
