#include "app.hpp"
#include "core/ambient_occlusion.hpp"
#include "core/contact_shadow.hpp"
#include "core/preview.hpp"
#include "core/samples.hpp"
#include "rpc/rpc.hpp"

#ifdef KILN_GUI
#include "gui/gui.hpp"
#endif

#include <cstring>
#include <iostream>
#include <string>
#include <vector>

int main(int argc, char** argv) {
    kiln::Session s;
    s.agent = kiln::agent_config_from_env();
    kiln::ensure_default_camera(s.doc.scene);
    kiln::ensure_default_sun(s.doc.scene);

    bool headless = false;
    bool repl = false;
    bool rpc_mode = false;
    std::string export_path;
    std::string preview_path;
    std::string samples_inline;  // value taken from the --flag=VALUE spelling
    std::vector<std::string> prompts;

    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        if (a == "--headless" || a == "-H") headless = true;
        else if (a == "--gui" || a == "--headed") {
#ifdef KILN_GUI
            headless = false;
#else
            std::cerr << "kiln: GUI not built (cmake with Vulkan)\n";
            return 2;
#endif
        } else if (a == "--repl" || a == "-i") repl = true;
        // Machine interface: JSON-RPC 2.0 on stdin/stdout. Additive -- the
        // human paths below are unchanged.
        else if (a == "--rpc") rpc_mode = true;
        else if (a == "--help" || a == "-h") {
            std::cout << kiln::help_text() << "\n\n"
                      << "usage: kiln [--headless|--gui] [--repl] [--rpc] [--export FILE]\n"
                      << "            [--preview FILE] [--samples N | --ss N] [--glb-preview]\n"
                      << "            [--preview-size WxH] [--contact-shadow N] [--ao N] [prompt...]\n\n"
                      << kiln::samples_usage() << "\n"
                      << kiln::preview_usage() << "\n"
                      << kiln::contact_shadow_usage() << "\n"
                      << kiln::ao_usage() << "\n"
                      << kiln::rpc_usage() << "\n";
            return 0;
        } else if ((a == "--export" || a == "-o") && i + 1 < argc)
            export_path = argv[++i];
        else if ((a == "--preview" || a == "-p") && i + 1 < argc)
            preview_path = argv[++i];
        // Bakes a preview PNG into the exported GLB. Off unless asked for, so
        // an export that omits it stays byte-identical to what kiln wrote
        // before this flag existed. --no-glb-preview makes that explicit.
        else if (a == "--glb-preview" || a == "--embed-preview")
            s.glb_preview = true;
        else if (a == "--no-glb-preview")
            s.glb_preview = false;
        // Resolution of that baked preview. Only affects the GLB payload, never
        // the mesh, so an invalid value is rejected rather than clamped.
        else if (a == "--preview-size" || a == "--glb-preview-size") {
            if (i + 1 >= argc) {
                std::cerr << "kiln: --preview-size needs a value\n" << kiln::preview_usage() << "\n";
                return 2;
            }
            std::string raw = argv[++i];
            int pw = 0, ph = 0;
            std::string err;
            if (!kiln::parse_preview_size(raw, pw, ph, err)) {
                std::cerr << "kiln: bad --preview-size: " << err << "\n" << kiln::preview_usage() << "\n";
                return 2;
            }
            s.glb_preview_w = pw;
            s.glb_preview_h = ph;
        }
        // --samples sets the supersampling factor for every preview this run
        // draws, headless or GUI. Defaults to 2x2, so omitting it changes nothing.
        // --ss is the short alias: it shares the one parser and the one pair of
        // messages below, so the two spellings cannot disagree. --aa is the older
        // long alias and keeps working.
        else if (a == kiln::kSamplesFlag || a == kiln::kSamplesFlagAlias || a == "--aa") {
            if (i + 1 >= argc) {
                std::cerr << "kiln: " << kiln::kSamplesFlag << " needs a value\n"
                          << kiln::samples_usage() << "\n";
                return 2;
            }
            std::string raw = argv[++i];
            int n = 0;
            std::string err;
            if (!kiln::parse_samples(raw, n, err)) {
                std::cerr << "kiln: bad " << kiln::kSamplesFlag << ": " << err << "\n"
                          << kiln::samples_usage() << "\n";
                return 2;
            }
            s.samples = n;
        }
        // --samples=4 / --ss=4 / --aa=4: the same value joined by '='. Kept in a
        // separate branch so the spaced spelling above keeps consuming the next
        // argv entry while this one never looks past the current argument.
        else if (kiln::samples_inline_value(a, samples_inline)) {
            int n = 0;
            std::string err;
            if (!kiln::parse_samples(samples_inline, n, err)) {
                std::cerr << "kiln: bad " << kiln::kSamplesFlag << ": " << err << "\n"
                          << kiln::samples_usage() << "\n";
                return 2;
            }
            s.samples = n;
        }
        // --contact-shadow and --ao set the two look passes for every preview
        // this run draws. Both default to 0 (off), so omitting them renders
        // exactly as this build did before the passes existed.
        else if (a == "--contact-shadow" || a == "--shadow") {
            if (i + 1 >= argc) {
                std::cerr << "kiln: --contact-shadow needs a value\n" << kiln::contact_shadow_usage() << "\n";
                return 2;
            }
            std::string raw = argv[++i];
            int n = 0;
            std::string err;
            if (!kiln::parse_contact_shadow(raw, n, err)) {
                std::cerr << "kiln: bad --contact-shadow: " << err << "\n" << kiln::contact_shadow_usage() << "\n";
                return 2;
            }
            s.contact_shadow = n;
        } else if (a == "--ao") {
            if (i + 1 >= argc) {
                std::cerr << "kiln: --ao needs a value\n" << kiln::ao_usage() << "\n";
                return 2;
            }
            std::string raw = argv[++i];
            int n = 0;
            std::string err;
            if (!kiln::parse_ao(raw, n, err)) {
                std::cerr << "kiln: bad --ao: " << err << "\n" << kiln::ao_usage() << "\n";
                return 2;
            }
            s.ao = n;
        } else if (a == "--prompt" || a == "-e") {
            if (i + 1 < argc) prompts.push_back(argv[++i]);
        } else if (a[0] != '-')
            prompts.push_back(a);
        else {
            std::cerr << "unknown flag " << a << "\n";
            return 2;
        }
    }

    if (!prompts.empty() || !export_path.empty() || !preview_path.empty()) headless = true;
    // A baked preview is drawn before anything is written, so a resolution and
    // sample count that together cost more than the preview budget is refused
    // here rather than crashing the renderer halfway through the export.
    if (s.glb_preview && !kiln::preview_cost_ok(s.glb_preview_w, s.glb_preview_h, s.samples)) {
        std::cerr << "kiln: " << kiln::preview_cost_message(s.glb_preview_w, s.glb_preview_h, s.samples)
                  << "\n"
                  << kiln::preview_usage() << "\n";
        return 2;
    }
    // Checked first: the machine interface owns stdin/stdout and prints no banner.
    if (rpc_mode) return kiln::run_rpc(s);
    if (repl) return kiln::run_repl(s);
    if (headless) {
        if (prompts.empty() && export_path.empty() && preview_path.empty()) return kiln::run_repl(s);
        return kiln::run_oneshot(s, prompts, export_path, preview_path);
    }

#ifdef KILN_GUI
    return kiln::run_gui(s);
#else
    return kiln::run_repl(s);
#endif
}
