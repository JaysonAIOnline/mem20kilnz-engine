#pragma once

#include "../scene/scene.hpp"

#include <cstdint>
#include <string>
#include <vector>

namespace kiln {

// A preview PNG to bake into an exported GLB. `enabled == false` -- the default
// -- means the exporter writes exactly the file it wrote before this option
// existed: same bytes, and no extra glTF objects.
struct GlbPreview {
    bool enabled = false;
    int w = 0;
    int h = 0;
    int samples = 0;           // recorded in the scene extras, for provenance
    std::vector<uint8_t> png;  // an encoded PNG payload, taken as-is
};

// What an import actually brought in, so a caller never has to guess. Anything
// the reader could not represent is named in `dropped_attributes` or
// `unsupported_extensions` rather than quietly discarded.
struct GltfImportStats {
    int nodes = 0;
    int meshes = 0;
    int primitives = 0;
    int materials = 0;
    int faces = 0;
    int vertices = 0;
    int cameras = 0;
    int skins = 0;
    int bones = 0;
    int skinned_vertices = 0;
    int skipped_modes = 0;
    std::vector<std::string> dropped_attributes;
    std::vector<std::string> unsupported_extensions;
};

// Two-argument form: the historical call. Writes exactly what it always did.
bool export_gltf(const Scene& scene, const std::string& path, std::string& err);

// Three-argument form: bakes `preview` into the file when preview.enabled.
bool export_gltf(const Scene& scene, const std::string& path, const GlbPreview& preview,
                 std::string& err);

// Reads a glTF 2.0 or GLB file into `scene`, appending to whatever is already
// there. Returns false and sets `err` on a hard failure; a primitive that is
// merely unrepresentable is counted in the stats instead of aborting.
GltfImportStats import_gltf(Scene& scene, const std::string& path, std::string& err);

bool save_scene(const Scene& scene, const std::string& path, std::string& err);
bool load_scene(Scene& scene, const std::string& path, std::string& err);

}  // namespace kiln
