# tsujikiri-bullet3
Bullet3 Lua bindings generated with [tsujikiri](https://github.com/kunitoki/tsujikiri) 辻斬り and [LuaBridge3](https://github.com/kunitoki/LuaBridge3).

See [demo/README.md](demo/README.md).

## Building and Running

With [just](https://github.com/casey/just) installed, from the repository root:

```bash
just build   # Configure and compile the demo (generates the bindings with tsujikiri)
just run     # Compile and run the demo
```

Run `just --list` to see all available recipes. The build goes to `demo/build`.

## Testing

```bash
just test    # build, then run the Lua tests for the bindings
```

The tests check every bound class and function and exercise the physics in short simulations; see [demo/README.md](demo/README.md#tests).
