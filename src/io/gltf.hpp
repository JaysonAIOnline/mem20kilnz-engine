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

// Two-argument form: the historical call. Writes exactly what it always did.
bool export_gltf(const Scene& scene, const std::string& path, std::string& err);

// Three-argument form: bakes `preview` into the file when preview.enabled.
bool export_gltf(const Scene& scene, const std::string& path, const GlbPreview& preview,
                 std::string& err);

bool save_scene(const Scene& scene, const std::string& path, std::string& err);
bool load_scene(Scene& scene, const std::string& path, std::string& err);

}  // namespace kiln
