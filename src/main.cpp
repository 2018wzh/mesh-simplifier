#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <optional>
#include <string>

#include "nanite.h"

namespace {

void PrintUsage() {
    std::cout << "Usage: mesh-simplifier -i <input.obj> [-r <ratio>] [-o <output.obj>] [-X "
                 "<nanite.bin>] [-N <nanite.bin>] [-v]"
              << std::endl;
}

} // namespace

int main(int argc, char **argv) {
    const char *inputPath    = nullptr;
    const char *outputPath   = nullptr;
    const char *naniteImport = nullptr;
    const char *naniteExport = nullptr;
    float ratio              = 0.5f;
    bool verbose             = false;

    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "-i") == 0 && i + 1 < argc) {
            inputPath = argv[i + 1];
            i++;
        } else if (std::strcmp(argv[i], "-r") == 0 && i + 1 < argc) {
            ratio = static_cast<float>(std::atof(argv[i + 1]));
            i++;
        } else if (std::strcmp(argv[i], "-o") == 0 && i + 1 < argc) {
            outputPath = argv[i + 1];
            i++;
        } else if (std::strcmp(argv[i], "-N") == 0 && i + 1 < argc) {
            naniteImport = argv[i + 1];
            i++;
        } else if (std::strcmp(argv[i], "-X") == 0 && i + 1 < argc) {
            naniteExport = argv[i + 1];
            i++;
        } else if (std::strcmp(argv[i], "-v") == 0) {
            verbose = true;
        } else {
            PrintUsage();
            return 1;
        }
    }

    ratio = std::clamp(ratio, 0.01f, 1.0f);
    if (!inputPath && !naniteImport) {
        PrintUsage();
        return 1;
    }

    std::string error;
    std::optional<nanite::Tree> tree;

    if (naniteImport) {
        tree = nanite::LoadTree(naniteImport, error);
        if (!tree) {
            std::cerr << "Failed to load nanite tree: " << error << std::endl;
            return 1;
        }
        if (verbose) {
            std::cout << "Loaded Nanite tree from " << naniteImport << " (" << tree->nodes.size()
                      << " nodes)" << std::endl;
        }
    }

    if (!tree && inputPath) {
        auto mesh = nanite::LoadObj(inputPath, error);
        if (!mesh) {
            std::cerr << "Failed to load OBJ: " << error << std::endl;
            return 1;
        }
        nanite::BuildSettings settings;
        tree = nanite::BuildTree(*mesh, settings);
        if (verbose) {
            std::cout << "Built Nanite tree: " << tree->nodes.size() << " nodes, "
                      << tree->originalFaceCount << " faces" << std::endl;
        }

        if (naniteExport) {
            if (!nanite::SaveTree(*tree, naniteExport, error)) {
                std::cerr << "Failed to save Nanite tree: " << error << std::endl;
                return 1;
            }
            if (verbose) {
                std::cout << "Exported Nanite tree to " << naniteExport << std::endl;
            }
        }
    }

    if (!tree) {
        std::cerr << "No input mesh or nanite tree supplied" << std::endl;
        return 1;
    }

    nanite::ViewCut cut = nanite::BuildViewDependentCut(*tree, ratio, verbose);
    std::cout << "Target ratio " << ratio << ", achieved " << cut.achievedRatio << " ("
              << cut.usedFaces << "/" << tree->originalFaceCount << " triangles)" << std::endl;

    if (outputPath) {
        if (!nanite::SaveObj(*cut.mesh, outputPath, error)) {
            std::cerr << "Failed to write OBJ: " << error << std::endl;
            return 1;
        }
        if (verbose) {
            std::cout << "Wrote simplified mesh to " << outputPath << std::endl;
        }
    }

    return 0;
}
