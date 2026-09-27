#pragma once

#include "../scene/scene.hpp"

#include <string>
#include <vector>

namespace kiln {

// What an FBX (or other assimp-readable) import actually brought in. Anything
// the reader could not represent is named here rather than dropped silently,
// which is the whole point of the struct: a caller must never have to guess
// whether a file's UVs survived.
struct FbxImportStats {
    int nodes = 0;
    int meshes = 0;
    int materials = 0;
    int faces = 0;
    int vertices = 0;
    int cameras = 0;
    int lights = 0;
    int root = -1;
    std::vector<std::string> dropped_attributes;
    std::string notice;
};

// Reads an FBX file, or any other format assimp can read, into `scene`,
// appending to whatever is already there. Returns false and sets `err` on a
// hard failure; a file that is merely partly unrepresentable is reported
// through the stats instead of aborting.
FbxImportStats import_fbx(Scene& scene, const std::string& path, std::string& err);

// Extensions the linked assimp can read, for capability reporting. Includes
// .fbx, plus .blend, .dae, .stl, .3ds and others.
std::vector<std::string> fbx_supported_extensions();

// True when assimp recognises this file, without reading it fully.
bool fbx_can_read(const std::string& path);

/// The extension list assimp is queried against, so the caller does not have to
/// carry its own copy and drift out of step with the linked library.
const char* const* assimp_extension_list();

}  // namespace kiln
