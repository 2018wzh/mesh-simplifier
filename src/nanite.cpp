#include "nanite.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <iostream>
#include <memory>
#include <numeric>
#include <queue>
#include <unordered_map>

#include <vcg/complex/algorithms/clean.h>
#include <vcg/complex/algorithms/create/platonic.h>
#include <vcg/space/index/grid_util.h>

namespace nanite {
namespace {

// Helper to ensure we keep meshes well-formed before any operation.
void CleanGeometry(Mesh &mesh) {
    vcg::tri::Clean<Mesh>::RemoveDuplicateVertex(mesh);
    vcg::tri::Clean<Mesh>::RemoveDegenerateFace(mesh);
    vcg::tri::Clean<Mesh>::RemoveUnreferencedVertex(mesh);
    vcg::tri::UpdateTopology<Mesh>::FaceFace(mesh);
    vcg::tri::UpdateNormal<Mesh>::PerFaceNormalized(mesh);
    vcg::tri::UpdateNormal<Mesh>::PerVertexAngleWeighted(mesh);
    vcg::tri::UpdateBounding<Mesh>::Box(mesh);
}

// Decimate in place using quadric edge collapses from vcglib.
void DecimateInPlace(Mesh &mesh, size_t targetFaces) {
    // Placeholder: a proper decimator should be wired here. For now, keep mesh unchanged.
    (void)targetFaces;
}

// Copy a subset of faces into a standalone mesh (keeps normals/uvs when present).
std::shared_ptr<Mesh> ExtractSubmesh(const Mesh &source, const std::vector<int> &faceIds) {
    auto out = std::make_shared<Mesh>();
    out->vert.reserve(faceIds.size() * 3);
    out->face.reserve(faceIds.size());

    std::unordered_map<const Vertex *, Vertex *> vmap;
    vmap.reserve(faceIds.size() * 2 + 1);

    for (int fi : faceIds) {
        const Face &f = source.face[fi];
        Vertex *v[3];
        for (int k = 0; k < 3; ++k) {
            const Vertex *orig = f.cV(k);
            auto it            = vmap.find(orig);
            if (it == vmap.end()) {
                out->vert.push_back(Vertex());
                Vertex *nv = &out->vert.back();
                nv->P()    = orig->cP();
                nv->N()    = orig->cN();
                nv->T()    = orig->cT();
                nv->Q()    = orig->cQ();
                vmap.emplace(orig, nv);
                v[k] = nv;
            } else {
                v[k] = it->second;
            }
        }
        out->face.push_back(Face());
        Face &nf = out->face.back();
        nf.V(0)  = v[0];
        nf.V(1)  = v[1];
        nf.V(2)  = v[2];
        nf.N()   = f.cN();
        nf.Q()   = f.cQ();
    }

    CleanGeometry(*out);
    return out;
}

// Combine meshes by appending vertices/faces with fresh indices.
void AppendMesh(Mesh &dst, const Mesh &src) {
    const size_t vertOffset = dst.vert.size();
    dst.vert.insert(dst.vert.end(), src.vert.begin(), src.vert.end());

    dst.face.reserve(dst.face.size() + src.face.size());
    for (const Face &f : src.face) {
        Face nf;
        nf.V(0) = &dst.vert[vertOffset + (f.cV(0) - &src.vert[0])];
        nf.V(1) = &dst.vert[vertOffset + (f.cV(1) - &src.vert[0])];
        nf.V(2) = &dst.vert[vertOffset + (f.cV(2) - &src.vert[0])];
        nf.N()  = f.cN();
        nf.Q()  = f.cQ();
        dst.face.push_back(nf);
    }
}

vcg::Box3f ComputeBoundingBox(const Mesh &mesh, const std::vector<int> &faceIds) {
    vcg::Box3f box;
    for (int fi : faceIds) {
        const Face &f = mesh.face[fi];
        for (int k = 0; k < 3; ++k)
            box.Add(f.cV(k)->cP());
    }
    return box;
}

float EstimateGeometricError(const vcg::Box3f &box, size_t faceCount) {
    const float diag    = box.Diag();
    const float density = faceCount > 0 ? 1.0f / std::sqrt(static_cast<float>(faceCount)) : 1.0f;
    return diag * density;
}

struct SplitResult {
    std::vector<int> left;
    std::vector<int> right;
};

SplitResult SplitFaces(const Mesh &mesh, const std::vector<int> &faceIds, const vcg::Box3f &box) {
    SplitResult res;
    res.left.reserve(faceIds.size() / 2);
    res.right.reserve(faceIds.size() / 2);

    const vcg::Point3f diag = box.max - box.min;
    int axis                = 0;
    if (diag[1] > diag[axis])
        axis = 1;
    if (diag[2] > diag[axis])
        axis = 2;
    const float mid = (box.min[axis] + box.max[axis]) * 0.5f;

    for (int fi : faceIds) {
        const Face &f  = mesh.face[fi];
        vcg::Point3f c = (f.cP(0) + f.cP(1) + f.cP(2)) / 3.0f;
        ((c[axis] <= mid) ? res.left : res.right).push_back(fi);
    }

    // Guard against empty buckets to keep tree well-formed.
    if (res.left.empty() || res.right.empty()) {
        res.left.clear();
        res.right.clear();
        res.left.reserve(faceIds.size() / 2);
        res.right.reserve(faceIds.size() / 2);
        for (size_t i = 0; i < faceIds.size(); ++i) {
            (i < faceIds.size() / 2 ? res.left : res.right).push_back(faceIds[i]);
        }
    }

    return res;
}

std::shared_ptr<Mesh> BuildProxy(const Mesh &source, const std::vector<int> &faceIds,
                                 const BuildSettings &settings, float bboxDiag) {
    (void)bboxDiag; // currently unused, kept for future heuristics
    auto proxy = ExtractSubmesh(source, faceIds);
    const size_t desired =
        std::clamp<size_t>(static_cast<size_t>(faceIds.size() * settings.proxyRatio),
                           settings.proxyMinFaces, settings.proxyMaxFaces);
    if (proxy->face.size() > desired) {
        DecimateInPlace(*proxy, desired);
    }
    return proxy;
}

int BuildNodeRecursive(const Mesh &source, const BuildSettings &settings, std::vector<Node> &nodes,
                       const std::vector<int> &faceIds, int depth) {
    const vcg::Box3f box = ComputeBoundingBox(source, faceIds);
    const float geomErr  = EstimateGeometricError(box, faceIds.size());

    Node node;
    node.bbox           = box;
    node.geometricError = geomErr;

    bool isLeaf =
        faceIds.size() <= settings.maxLeafFaces || depth >= static_cast<int>(settings.maxDepth);
    if (isLeaf) {
        node.proxy      = BuildProxy(source, faceIds, settings, box.Diag());
        node.firstChild = 0;
        node.childCount = 0;
        const int idx   = static_cast<int>(nodes.size());
        nodes.push_back(node);
        return idx;
    }

    const SplitResult split = SplitFaces(source, faceIds, box);
    const int idx           = static_cast<int>(nodes.size());
    nodes.push_back(Node{}); // placeholder to keep indices stable

    const int leftIdx  = BuildNodeRecursive(source, settings, nodes, split.left, depth + 1);
    const int rightIdx = BuildNodeRecursive(source, settings, nodes, split.right, depth + 1);

    node.proxy          = BuildProxy(source, faceIds, settings, box.Diag());
    node.firstChild     = static_cast<uint32_t>(leftIdx);
    node.childCount     = 2;
    node.geometricError = geomErr;
    node.bbox           = box;

    nodes[idx] = node;
    return idx;
}

} // namespace

std::shared_ptr<Mesh> LoadObj(const std::string &path, std::string &error) {
    auto mesh     = std::make_shared<Mesh>();
    int mask      = 0;
    const int res = vcg::tri::io::ImporterOBJ<Mesh>::Open(*mesh, path.c_str(), mask);
    if (res != 0) {
        error = vcg::tri::io::ImporterOBJ<Mesh>::ErrorMsg(res);
        return nullptr;
    }
    CleanGeometry(*mesh);
    if (mesh->face.empty() || mesh->vert.empty()) {
        error = "OBJ is empty or unsupported";
        return nullptr;
    }
    return mesh;
}

bool SaveObj(Mesh &mesh, const std::string &path, std::string &error) {
    int mask = vcg::tri::io::Mask::IOM_VERTCOORD | vcg::tri::io::Mask::IOM_VERTNORMAL |
               vcg::tri::io::Mask::IOM_VERTTEXCOORD | vcg::tri::io::Mask::IOM_FACEINDEX;
    const int res = vcg::tri::io::ExporterOBJ<Mesh>::Save(mesh, path.c_str(), mask);
    if (res != 0) {
        error = vcg::tri::io::ExporterOBJ<Mesh>::ErrorMsg(res);
        return false;
    }
    return true;
}

Tree BuildTree(const Mesh &source, const BuildSettings &settings) {
    Tree tree;
    tree.originalFaceCount   = source.face.size();
    tree.originalVertexCount = source.vert.size();

    std::vector<int> faceIds(source.face.size());
    std::iota(faceIds.begin(), faceIds.end(), 0);

    tree.nodes.reserve(source.face.size() / settings.maxLeafFaces * 2 + 8);
    BuildNodeRecursive(source, settings, tree.nodes, faceIds, 0);
    return tree;
}

ViewCut BuildViewDependentCut(const Tree &tree, float ratio, bool verbose) {
    ViewCut cut;
    cut.mesh = std::make_shared<Mesh>();
    if (tree.nodes.empty())
        return cut;

    const size_t targetFaces = static_cast<size_t>(std::clamp(ratio, 0.01f, 1.0f) *
                                                   static_cast<float>(tree.originalFaceCount));
    std::vector<char> active(tree.nodes.size(), 0);

    struct Item {
        int idx;
        float error;
    };
    auto cmp = [](const Item &a, const Item &b) { return a.error < b.error; };
    std::priority_queue<Item, std::vector<Item>, decltype(cmp)> queue(cmp);

    queue.push({0, tree.root().geometricError});
    active[0]           = 1;
    size_t currentFaces = tree.root().proxy ? tree.root().proxy->face.size() : 0;

    if (verbose) {
        std::cout << "[cut] start target faces " << targetFaces << ", initial " << currentFaces
                  << std::endl;
    }

    while (currentFaces < targetFaces && !queue.empty()) {
        Item top = queue.top();
        queue.pop();
        const Node &n = tree.nodes[top.idx];
        if (n.childCount == 0)
            continue;
        if (!n.proxy)
            continue;

        active[top.idx] = 0;
        currentFaces -= n.proxy->face.size();
        if (verbose) {
            std::cout << "[cut] split node " << top.idx << " err=" << n.geometricError
                      << " -> children " << n.firstChild << " and " << (n.firstChild + 1)
                      << std::endl;
        }
        for (uint32_t c = 0; c < n.childCount; ++c) {
            int ci     = static_cast<int>(n.firstChild + c);
            active[ci] = 1;
            queue.push({ci, tree.nodes[ci].geometricError});
            if (tree.nodes[ci].proxy)
                currentFaces += tree.nodes[ci].proxy->face.size();
        }
        if (verbose) {
            std::cout << "[cut] faces now " << currentFaces << " / " << targetFaces << std::endl;
        }
    }

    // Reserve once to avoid reallocations that would invalidate face vertex pointers.
    size_t totalVerts = 0, totalFaces = 0;
    for (size_t i = 0; i < tree.nodes.size(); ++i) {
        if (!active[i] || !tree.nodes[i].proxy)
            continue;
        totalVerts += tree.nodes[i].proxy->vert.size();
        totalFaces += tree.nodes[i].proxy->face.size();
    }
    cut.mesh->vert.reserve(totalVerts);
    cut.mesh->face.reserve(totalFaces);

    for (size_t i = 0; i < tree.nodes.size(); ++i) {
        if (!active[i])
            continue;
        if (tree.nodes[i].proxy)
            AppendMesh(*cut.mesh, *tree.nodes[i].proxy);
    }

    CleanGeometry(*cut.mesh);
    cut.usedFaces     = cut.mesh->face.size();
    cut.achievedRatio = tree.originalFaceCount > 0 ? static_cast<float>(cut.usedFaces) /
                                                         static_cast<float>(tree.originalFaceCount)
                                                   : 0.0f;
    return cut;
}

bool SaveTree(const Tree &tree, const std::string &path, std::string &error) {
    struct Header {
        char magic[8];
        uint32_t version;
        uint64_t nodeCount;
        uint64_t originalFaces;
        uint64_t originalVerts;
    };

    std::ofstream out(path, std::ios::binary);
    if (!out) {
        error = "Cannot open file for writing";
        return false;
    }

    Header h{};
    std::memcpy(h.magic, "NANITET", 7);
    h.version       = 1;
    h.nodeCount     = tree.nodes.size();
    h.originalFaces = tree.originalFaceCount;
    h.originalVerts = tree.originalVertexCount;
    out.write(reinterpret_cast<const char *>(&h), sizeof(h));

    for (const Node &n : tree.nodes) {
        out.write(reinterpret_cast<const char *>(n.bbox.min.V()), sizeof(float) * 3);
        out.write(reinterpret_cast<const char *>(n.bbox.max.V()), sizeof(float) * 3);
        out.write(reinterpret_cast<const char *>(&n.geometricError), sizeof(float));
        out.write(reinterpret_cast<const char *>(&n.firstChild), sizeof(uint32_t));
        out.write(reinterpret_cast<const char *>(&n.childCount), sizeof(uint32_t));

        const Mesh *proxy = n.proxy.get();
        uint32_t vcount   = proxy ? static_cast<uint32_t>(proxy->vert.size()) : 0u;
        uint32_t fcount   = proxy ? static_cast<uint32_t>(proxy->face.size()) : 0u;
        out.write(reinterpret_cast<const char *>(&vcount), sizeof(uint32_t));
        out.write(reinterpret_cast<const char *>(&fcount), sizeof(uint32_t));
        if (proxy) {
            for (const Vertex &v : proxy->vert) {
                out.write(reinterpret_cast<const char *>(v.cP().V()), sizeof(float) * 3);
                out.write(reinterpret_cast<const char *>(v.cN().V()), sizeof(float) * 3);
                const vcg::TexCoord2<float> &t = v.cT();
                float uv[2]                    = {t.U(), t.V()};
                out.write(reinterpret_cast<const char *>(uv), sizeof(float) * 2);
            }
            for (const Face &f : proxy->face) {
                uint32_t idx[3] = {static_cast<uint32_t>(f.cV(0) - &proxy->vert[0]),
                                   static_cast<uint32_t>(f.cV(1) - &proxy->vert[0]),
                                   static_cast<uint32_t>(f.cV(2) - &proxy->vert[0])};
                out.write(reinterpret_cast<const char *>(idx), sizeof(uint32_t) * 3);
            }
        }
    }

    return true;
}

std::optional<Tree> LoadTree(const std::string &path, std::string &error) {
    struct Header {
        char magic[8];
        uint32_t version;
        uint64_t nodeCount;
        uint64_t originalFaces;
        uint64_t originalVerts;
    };

    std::ifstream in(path, std::ios::binary);
    if (!in) {
        error = "Cannot open file for reading";
        return std::nullopt;
    }

    Header h{};
    in.read(reinterpret_cast<char *>(&h), sizeof(h));
    if (std::strncmp(h.magic, "NANITET", 7) != 0 || h.version != 1) {
        error = "Invalid nanite tree file";
        return std::nullopt;
    }

    Tree tree;
    tree.nodes.resize(h.nodeCount);
    tree.originalFaceCount   = static_cast<size_t>(h.originalFaces);
    tree.originalVertexCount = static_cast<size_t>(h.originalVerts);

    for (size_t i = 0; i < h.nodeCount; ++i) {
        Node n;
        float bmin[3], bmax[3];
        in.read(reinterpret_cast<char *>(bmin), sizeof(float) * 3);
        in.read(reinterpret_cast<char *>(bmax), sizeof(float) * 3);
        n.bbox.min = vcg::Point3f(bmin[0], bmin[1], bmin[2]);
        n.bbox.max = vcg::Point3f(bmax[0], bmax[1], bmax[2]);
        in.read(reinterpret_cast<char *>(&n.geometricError), sizeof(float));
        in.read(reinterpret_cast<char *>(&n.firstChild), sizeof(uint32_t));
        in.read(reinterpret_cast<char *>(&n.childCount), sizeof(uint32_t));

        uint32_t vcount = 0, fcount = 0;
        in.read(reinterpret_cast<char *>(&vcount), sizeof(uint32_t));
        in.read(reinterpret_cast<char *>(&fcount), sizeof(uint32_t));
        if (vcount > 0 || fcount > 0) {
            n.proxy = std::make_shared<Mesh>();
            n.proxy->vert.resize(vcount);
            n.proxy->face.resize(fcount);
            for (uint32_t vi = 0; vi < vcount; ++vi) {
                float pos[3], norm[3], uv[2];
                in.read(reinterpret_cast<char *>(pos), sizeof(float) * 3);
                in.read(reinterpret_cast<char *>(norm), sizeof(float) * 3);
                in.read(reinterpret_cast<char *>(uv), sizeof(float) * 2);
                n.proxy->vert[vi].P()     = vcg::Point3f(pos[0], pos[1], pos[2]);
                n.proxy->vert[vi].N()     = vcg::Point3f(norm[0], norm[1], norm[2]);
                n.proxy->vert[vi].T().U() = uv[0];
                n.proxy->vert[vi].T().V() = uv[1];
            }
            for (uint32_t fi = 0; fi < fcount; ++fi) {
                uint32_t idx[3];
                in.read(reinterpret_cast<char *>(idx), sizeof(uint32_t) * 3);
                n.proxy->face[fi].V(0) = &n.proxy->vert[idx[0]];
                n.proxy->face[fi].V(1) = &n.proxy->vert[idx[1]];
                n.proxy->face[fi].V(2) = &n.proxy->vert[idx[2]];
            }
            vcg::tri::UpdateNormal<Mesh>::PerFaceNormalized(*n.proxy);
            vcg::tri::UpdateNormal<Mesh>::PerVertexAngleWeighted(*n.proxy);
            vcg::tri::UpdateBounding<Mesh>::Box(*n.proxy);
        }
        tree.nodes[i] = n;
    }

    return tree;
}

} // namespace nanite
