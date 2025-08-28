# EZPZ

Constraint solver for use in Zoo Design Studio, or wherever you want to use it.

## Overview

This project has 3 Rust crates:

 - `kcl-ezpz`: The core constraint solver library
 - `ezpz-cli`: A CLI tool that lets you easily solve, analyze and benchmark constraint systems using `kcl-ezpz`.
 - `ezpz-wasm`: A WebAssembly library that wraps `kcl-ezpz` and exposes a few core functions. Intended for the ezpz maintainers to check if `kcl-ezpz` compiles and works in WebAssembly, and to benchmark its performance vs. native code.
  - Users probably won't need to use this library yourself, it's really intended as a sample application for the maintainers to test with.
  - Users should probably integrate `kcl-ezpz` into your web projects directly.

## Using the CLI

The CLI lets you pass a constraint problem file (described below) and analyze it, solve it, visualize the solution and benchmark how long it took.

First, install it. From this repo's root, run:

```
cargo install --path ezpz-cli
```

Then you can use it like this, by passing it a constraint problem file:

```
$ ezpz --filepath myconstraints.txt
Problem size: 2000 rows, 2000 vars
Iterations needed: 2
Solved in 2943μs (mean over 100 iterations)
i.e. 339 solves per second
```

You can also add the `--gnuplot` option to visualize the resulting points in a gnuplot window, or `--gnuplot-png-path points.png` to write the visualization to a PNG at the given path instead. If you'd rather print the final points to stdout and process them in your own tool, use `--show-points` instead.


## Constraint problem files

ezpz defines a text format for writing out constraint problems. You don't have to use this format -- you can use the Rust library directly -- but it's a very convenient format. It looks like this:

```
# constraints
point p
point q
p.x = 0
p.y = 0
q.y = 0
vertical(p, q)

# guesses
p roughly (3, 4)
q roughly (5, 6)
```

There's two sections, Constraints and Guesses. You define each point (like `p` and `q`) and once defined, you can write constraints that use them. For example, you can fix a point's X or Y component (`p.x = 0`). Or you can relate two points, e.g. `vertical(p, q)`.

For more examples, see the [`test_cases/`](https://github.com/KittyCAD/ezpz/tree/main/test_cases) directory.
