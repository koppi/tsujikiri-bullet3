#!/usr/bin/env python3
"""Turn the generated LuaBridge3 bindings into a Lua table describing the bound API.

The tests need to know every class, constructor and function that tsujikiri bound,
and that list changes whenever Bullet (fetched from master) or the tsujikiri config
changes. Reading it back from the generated `bullet3_bindings.cpp` keeps the tests in
sync without a checked-in copy that could rot.

Usage: gen_api_manifest.py <bullet3_bindings.cpp> <bullet3_api.lua>

The generated file is one fluent chain made of just these calls:

    .beginNamespace("bullet3")
    .beginClass<T>("T") | .deriveClass<T, Base>("T")
    .addConstructor<void (*)(args), ...>()
    .addFunction("lua_name", ...)
    .addStaticFunction("lua_name", ...)
    .endClass()
    .endNamespace()

Anything else in the chain makes this script fail loudly instead of silently producing
a weaker manifest.
"""

import re
import sys

CALL = re.compile(
    r"""\.(?P<call>beginNamespace|beginClass|deriveClass|addConstructor|addFunction|
                 addStaticFunction|endClass|endNamespace)
        (?:<(?P<targs>[^"]*?)>)?       # template arguments (not for addFunction & co)
        \(\s*(?P<args>"[^"]*")?""",
    re.VERBOSE | re.DOTALL,
)


def split_top_level(text, sep=","):
    """Split on `sep` ignoring separators nested in <>, () or []."""
    parts, depth, current = [], 0, []
    for ch in text:
        if ch in "<([":
            depth += 1
        elif ch in ">)]":
            depth -= 1
        if ch == sep and depth == 0:
            parts.append("".join(current).strip())
            current = []
        else:
            current.append(ch)
    tail = "".join(current).strip()
    if tail:
        parts.append(tail)
    return parts


def parse_constructors(targs):
    """`void (*)(int), void (*)(const btVector3 &)` -> [["int"], ["const btVector3 &"]]"""
    signatures = []
    for sig in split_top_level(targs):
        match = re.fullmatch(r"void\s*\(\*\)\s*\((.*)\)", sig, re.DOTALL)
        if not match:
            raise SystemExit(f"unrecognised constructor signature: {sig!r}")
        signatures.append(split_top_level(match.group(1)))
    return signatures


def parse(source):
    # Drop the prefix (`using` aliases, includes): the chain starts at beginNamespace
    start = source.index(".beginNamespace(")
    chain = re.sub(r"\s+", " ", source[start:])

    classes, current, cpp_to_lua = [], None, {}
    namespace = None
    for match in CALL.finditer(chain):
        call, targs, args = match.group("call"), match.group("targs"), match.group("args")
        name = args.strip('"') if args else None
        if call == "beginNamespace":
            namespace = name
        elif call in ("beginClass", "deriveClass"):
            if current is not None:
                raise SystemExit(f"{name}: previous class was not closed")
            template = split_top_level(targs)
            cpp_type, base_cpp = template[0], (template[1] if len(template) > 1 else None)
            base = None
            if base_cpp is not None:
                if base_cpp not in cpp_to_lua:
                    raise SystemExit(f"{name}: base class {base_cpp} was not registered before it")
                base = cpp_to_lua[base_cpp]
            current = {"name": name, "base": base, "constructors": [], "methods": [], "statics": []}
            cpp_to_lua[cpp_type] = name
        elif call == "endClass":
            classes.append(current)
            current = None
        elif call == "endNamespace":
            pass
        elif current is None:
            raise SystemExit(f"{call} outside of a class")
        elif call == "addConstructor":
            current["constructors"].extend(parse_constructors(targs))
        elif call == "addFunction":
            current["methods"].append(name)
        elif call == "addStaticFunction":
            current["statics"].append(name)
    if current is not None:
        raise SystemExit(f"{current['name']}: class was not closed")
    return namespace, classes, chain


def check_complete(classes, chain):
    """Fail if the regex walk missed any call of the chain."""
    seen = {}
    for call in re.findall(r"\.([A-Za-z_]\w*)\s*[<(]", chain):
        seen[call] = seen.get(call, 0) + 1
    unknown = set(seen) - {
        "beginNamespace", "beginClass", "deriveClass", "addConstructor",
        "addFunction", "addStaticFunction", "endClass", "endNamespace",
    }
    # `.x` inside expressions such as `&Foo::bar` has no leading dot, so anything
    # left over is a chain call this script does not understand
    if unknown:
        raise SystemExit(f"unsupported calls in the generated bindings: {sorted(unknown)}")
    expected = {
        "beginClass": sum(1 for c in classes if c["base"] is None),
        "deriveClass": sum(1 for c in classes if c["base"] is not None),
        "addFunction": sum(len(c["methods"]) for c in classes),
        "addStaticFunction": sum(len(c["statics"]) for c in classes),
        "endClass": len(classes),
    }
    for call, count in expected.items():
        if seen.get(call, 0) != count:
            raise SystemExit(f"parsed {count} {call} calls but the file has {seen.get(call, 0)}")


def lua_string(text):
    return '"' + text.replace("\\", "\\\\").replace('"', '\\"') + '"'


def lua_list(items):
    return "{" + ", ".join(lua_string(i) for i in items) + "}"


def emit(namespace, classes):
    out = [
        "-- Generated by gen_api_manifest.py from bullet3_bindings.cpp. Do not edit.",
        "return {",
        f"  namespace = {lua_string(namespace)},",
        "  classes = {",
    ]
    for c in classes:
        ctors = "{" + ", ".join(lua_list(sig) for sig in c["constructors"]) + "}"
        out.append(
            "    {{ name = {n}, base = {b}, constructors = {k}, methods = {m}, statics = {s} }},".format(
                n=lua_string(c["name"]),
                b=lua_string(c["base"]) if c["base"] else "false",
                k=ctors,
                m=lua_list(c["methods"]),
                s=lua_list(c["statics"]),
            )
        )
    out += ["  },", "}", ""]
    return "\n".join(out)


def main(argv):
    if len(argv) != 3:
        raise SystemExit(__doc__)
    with open(argv[1], encoding="utf-8") as f:
        namespace, classes, chain = parse(f.read())
    check_complete(classes, chain)
    names = [c["name"] for c in classes]
    if len(set(names)) != len(names):
        raise SystemExit("duplicate class names in the generated bindings")
    with open(argv[2], "w", encoding="utf-8") as f:
        f.write(emit(namespace, classes))
    print(
        f"{argv[2]}: {len(classes)} classes, "
        f"{sum(len(c['methods']) for c in classes)} functions, "
        f"{sum(len(c['statics']) for c in classes)} static functions, "
        f"{sum(len(c['constructors']) for c in classes)} constructors"
    )


if __name__ == "__main__":
    main(sys.argv)
