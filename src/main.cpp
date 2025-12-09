#include <cstdio>
#include <cstdlib>
#include <cstring>

int main(int argc, char **argv) {
    // Example: mesh-simplifier -i model.obj -r 0.35
    const char *inputPath    = nullptr;
    const char *outputPath   = nullptr;
    const char *naniteImport = nullptr;
    const char *naniteExport = nullptr;
    float ratio              = 0.5f;

    for (int i = 1; i < argc; ++i) {
        if (strcmp(argv[i], "-i") == 0 && i + 1 < argc) {
            inputPath = argv[i + 1];
            i++;
        } else if (strcmp(argv[i], "-r") == 0 && i + 1 < argc) {
            ratio = float(atof(argv[i + 1]));
            i++;
        } else if (strcmp(argv[i], "-o") == 0 && i + 1 < argc) {
            outputPath = argv[i + 1];
            i++;
        } else if (strcmp(argv[i], "-N") == 0 && i + 1 < argc) {
            naniteImport = argv[i + 1];
            i++;
        } else if (strcmp(argv[i], "-X") == 0 && i + 1 < argc) {
            naniteExport = argv[i + 1];
            i++;
        } else {
            printf("Usage: mesh-simplifier -i <input.obj> [-r <ratio>] [-o <output.obj>] [-X "
                   "<nanite.bin>] [-N <nanite.bin>]\n");
            return 1;
        }
    }

    printf("Usage: mesh-simplifier -i <input.obj> [-r <ratio>] [-o <output.obj>] [-X <nanite.bin>] "
           "[-N <nanite.bin>]\n");
    return 1;
}
