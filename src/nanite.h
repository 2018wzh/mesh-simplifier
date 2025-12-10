#pragma once

#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <vcg/complex/algorithms/clean.h>
#include <vcg/complex/algorithms/update/bounding.h>
#include <vcg/complex/algorithms/update/normal.h>
#include <vcg/complex/algorithms/update/topology.h>
#include <vcg/complex/complex.h>

#if defined(__clang__)
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wreturn-stack-address"
#pragma clang diagnostic ignored "-Wdeprecated-declarations"
#endif
#include <wrap/io_trimesh/export_obj.h>
#include <wrap/io_trimesh/import_obj.h>
#if defined(__clang__)
#pragma clang diagnostic pop
#endif

namespace nanite {

// Basic VCG mesh types with normals and texcoords.
class Vertex;
class Face;
struct UsedTypes
    : public vcg::UsedTypes<vcg::Use<Vertex>::AsVertexType, vcg::Use<Face>::AsFaceType> {};
class Vertex
    : public vcg::Vertex<UsedTypes, vcg::vertex::Coord3f, vcg::vertex::Normal3f,
                         vcg::vertex::TexCoord2f, vcg::vertex::Qualityf, vcg::vertex::BitFlags> {};
class Face
    : public vcg::Face<UsedTypes, vcg::face::VertexRef, vcg::face::Normal3f, vcg::face::VFAdj,
                       vcg::face::FFAdj, vcg::face::Qualityf, vcg::face::BitFlags> {};
using Mesh = vcg::tri::TriMesh<std::vector<Vertex>, std::vector<Face>>;

struct BuildSettings {
    size_t maxLeafFaces  = 256;   // Maximum triangles per leaf cluster
    size_t maxDepth      = 16;    // Max recursion depth for clustering
    float proxyRatio     = 0.35f; // Ratio of faces kept in proxy meshes
    size_t proxyMinFaces = 2;     // Minimum faces per proxy
    size_t proxyMaxFaces = 96;    // Cap proxy complexity
};

struct Node {
    vcg::Box3f bbox;             // Bounding box of faces feeding this node
    float geometricError = 0;    // Screen-space prioritization cue
    uint32_t firstChild  = 0;    // Index of the first child in the flat vector
    uint32_t childCount  = 0;    // 0 means leaf
    std::shared_ptr<Mesh> proxy; // Simplified proxy representing this cluster
};

struct Tree {
    std::vector<Node> nodes;
    size_t originalFaceCount   = 0;
    size_t originalVertexCount = 0;

    bool empty() const { return nodes.empty(); }
    const Node &root() const { return nodes.front(); }
};

struct ViewCut {
    std::shared_ptr<Mesh> mesh; // Simplified mesh built from active proxies
    size_t usedFaces    = 0;    // Number of proxy triangles emitted
    float achievedRatio = 0;    // usedFaces / originalFaceCount
};

// Pipeline entry points
std::shared_ptr<Mesh> LoadObj(const std::string &path, std::string &error);
bool SaveObj(Mesh &mesh, const std::string &path, std::string &error);

Tree BuildTree(const Mesh &source, const BuildSettings &settings);
ViewCut BuildViewDependentCut(const Tree &tree, float ratio, bool verbose = false);

bool SaveTree(const Tree &tree, const std::string &path, std::string &error);
std::optional<Tree> LoadTree(const std::string &path, std::string &error);

} // namespace nanite
