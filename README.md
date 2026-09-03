# Fish World

An interactive boids simulation written in C++ with [raylib](https://www.raylib.com/).
The primary 2D simulation renders 500 boids and exposes sliders for repulsion,
attraction, and velocity alignment. A separate 3D experiment implements zonal
repulsion, orientation, and attraction behavior with an orbit camera.

## Requirements

- C++17 compiler
- A raylib source checkout, including a platform-appropriate static library
- Emscripten, only for the web build

The Makefile expects raylib at `../raylib/src` by default. Override it when
raylib lives elsewhere:

```sh
make desktop RAYLIB_SRC=/path/to/raylib/src
```

Build raylib for the platform you plan to target before building this project.
For the web target, `RAYLIB_SRC` must contain `libraylib.web.a`.

## Run The 2D Simulation

On Linux:

```sh
make desktop
./boids
```

On macOS:

```sh
make mac
./boids
```

Close the window to stop the simulation. Drag the sliders in the lower-right
corner to tune the flocking behavior.

## Build For The Web

With the Emscripten environment activated:

```sh
make web
```

This writes `boids.js` and `boids.wasm` to `public/boids/`. The JavaScript
output exports a module factory named `createBoids`; the simulation starts when
the module is initialized and provides `_StopSim` for host applications to call
when unmounting it.

Set `WEB_OUT` to emit the bundle somewhere else:

```sh
make web WEB_OUT=/path/to/static/boids
```

## 3D Experiment

`boid3d.cpp` is a standalone native experiment. It displays 500 boids in 3D,
reports polarization and momentum, and lets you orbit with the left mouse button
and zoom with the scroll wheel. Its two sliders control the orientation and
attraction zone widths.

## Clean Generated Files

```sh
make clean
```

This removes the native `boids` executable and the web output directory.
