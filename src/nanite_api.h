#pragma once

#include <stdint.h>
#include <vector>

#include "mesh_types.h"

struct NaniteTree {
    // levels[depth] -> list of clusters; each cluster is an index buffer slice referencing
    // source.vertices
    std::vector<std::vector<std::vector<unsigned int>>> levels;
    std::vector<size_t> trianglesPerLevel;
    size_t originalTriangles = 0;
    uint64_t hash            = 0; // hash of the source mesh data
    Mesh source;
};

// Build Nanite-style cluster hierarchy using meshoptimizer's cluster LOD (clod) pipeline.
NaniteTree init_mesh(const Mesh &mesh);

// Pick a level from the Nanite hierarchy based on triangle ratio and produce a simplified mesh.
Mesh simplify_mesh(const NaniteTree &tree, float ratio);

// Serialize NaniteTree (including source mesh) to a binary file; returns true on success.
bool export_nanite(const NaniteTree &tree, const char *path);

// Deserialize NaniteTree from a binary file; returns empty tree on error.
NaniteTree import_nanite(const char *path);
