#pragma once

#include "../core/ambient_occlusion.hpp"
#include "../core/contact_shadow.hpp"
#include "../core/samples.hpp"
#include "../scene/scene.hpp"

#include <string>
#include <vector>

namespace kiln {

struct Preview {
    int w = 960;
    int h = 540;
    std::vector<uint8_t> rgba;
};

// Non-geometric look settings for the preview. A default-constructed Shading
// leaves both passes off, which is why an unconfigured run renders exactly as
// it did before they existed. The range and the default of each dial live in
// its own core header, so the CLI flag, the REPL command and the renderer all
// read one authority.
struct Shading {
    int contact_shadow = kDefaultContactShadow;
    int ao = kDefaultAo;
};

// ss = supersampling factor (1 = off, 2 = 2x2 grid AA). Default 2, range
// [kSamplesMin, kSamplesMax] from core/samples.hpp.
//
// The shading passes run on the finished preview, after the supersampled buffer
// has been box-filtered back down. They therefore add to the antialiased image
// rather than changing what the antialiasing did, which is what lets them
// compose with --samples and leaves --samples 1 and 8 looking like themselves.
Preview render_preview(const Scene& scene, int w = 960, int h = 540, int ss = kDefaultSamples,
                       const Shading& shading = Shading{});
bool write_png(const Preview& img, const std::string& path, std::string& err);

// Encode a preview to PNG bytes in memory instead of to a file. Same pixels as
// write_png; this is the form the exporter needs to bake a thumbnail into a GLB.
bool encode_png(const Preview& img, std::vector<uint8_t>& out, std::string& err);

}  // namespace kiln
