#pragma once

#include "agent/agent.hpp"
#include "core/preview.hpp"
#include "core/samples.hpp"
#include "core/shading_note.hpp"
#include "io/gltf.hpp"
#include "render/software.hpp"

#include <string>

namespace kiln {

struct Session {
    Document doc;
    AgentConfig agent;
    std::string last_preview = "kiln_preview.png";
    // Supersampling factor shared by every path that draws: headless previews,
    // the preview baked into an export, and the GUI readout. Defaults to the
    // historical 2x2 grid so an unconfigured run renders exactly as it did
    // before.
    int samples = kDefaultSamples;
    // The two look passes for the software preview. Both default to 0 (off), so
    // an unconfigured run renders exactly as this build did before the passes
    // existed. Their ranges live in core/contact_shadow.hpp and
    // core/ambient_occlusion.hpp, which the CLI flag, the REPL command and the
    // renderer all read.
    int contact_shadow = kDefaultContactShadow;
    int ao = kDefaultAo;
    // Bundled for the renderer. A method rather than a stored struct so the CLI,
    // the REPL and the GUI read one pair of fields and cannot disagree.
    Shading shading() const { return Shading{contact_shadow, ao}; }
    // Preview baked into the next export. Off by default and additive when on:
    // an export that does not ask for a preview stays byte-identical to what
    // this build wrote before the option existed.
    bool glb_preview = kGlbPreviewDefault;
    int glb_preview_w = kPreviewDefaultW;
    int glb_preview_h = kPreviewDefaultH;
    // Whether the last `export` actually wrote its file. A failed export used
    // to be reported on stdout but still exit 0, so a script could not tell.
    bool last_export_ok = true;
    bool running = true;
};

bool handle_line(Session& s, const std::string& line, std::string& reply);
int run_repl(Session& s);
int run_oneshot(Session& s, const std::vector<std::string>& prompts, const std::string& export_path,
                const std::string& preview_path);

}  // namespace kiln
