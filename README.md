# mesh-simplifier

A small C++17 tool that loads OBJ meshes, builds a Nanite-style cluster hierarchy with [vcglib](https://github.com/cnr-isti-vclab/vcglib), and produces simplified meshes at a requested ratio using a view-dependent cut over that hierarchy. Supports exporting/importing precomputed Nanite trees to skip rebuilds.

## Build

Prerequisites: CMake, a C++17 compiler, and vcpkg.

```sh
cmake -B build
cmake --build build
```

## Usage

The executable is `build/mesh-simplifier`.

Simplify from an OBJ and export Nanite data:
```sh
./build/mesh-simplifier -i ./data/wall.obj -r 0.50 -o wall.out.obj -X wall.nanite.bin
```

Simplify using a precomputed Nanite tree:
```sh
./build/mesh-simplifier -N wall.nanite.bin -r 0.25 -o wall.out.obj
```

Options:
- `-i <input.obj>`: input OBJ file (polygons are triangulated on load).
- `-r <ratio>`: target triangle ratio (0 < r ≤ 1), default 0.5.
- `-o <output.obj>`: write simplified mesh as OBJ (positions, normals, UVs, triangle faces).
- `-X <nanite.bin>`: export built Nanite hierarchy to a binary blob.
- `-N <nanite.bin>`: import a prebuilt Nanite hierarchy; skips OBJ parsing and rebuild.

## Behavior

- OBJ loading uses vcglib's OBJ importer; positions/normals/UVs are kept.
- Clustering splits faces spatially until leaves have at most 256 triangles. Each node stores a quadric-simplified proxy mesh built with vcglib's edge-collapse decimator.
- View-dependent cut starts from the root proxy and iteratively refines the node with the largest geometric error until the requested ratio is met. Active proxies are merged and exported as the simplified mesh.

## Notes

- Ratio outside (0,1] is clamped to [0.01, 1].
- Output OBJ uses 1-based indexing with per-vertex position/normal/UV triplets.

## Credits
- [meshoptimizer](https://github.com/zeux/meshoptimizer)
- [fast_obj](https://github.com/thisistherk/fast_obj)
- [Brian Karis. Nanite: A Deep Dive. 2021](https://www.youtube.com/watch?v=eviSykqSUUw)