#include "app.hpp"

#include <algorithm>
#include <cctype>
#include <cstdlib>
#include <iostream>
#include <sstream>

namespace kiln {
namespace {

std::string lower(std::string s) {
    for (char& c : s) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    return s;
}
std::string trim(std::string s) {
    auto n = [](int ch) { return !std::isspace(ch); };
    s.erase(s.begin(), std::find_if(s.begin(), s.end(), n));
    s.erase(std::find_if(s.rbegin(), s.rend(), n).base(), s.end());
    return s;
}
std::vector<std::string> split_ws(const std::string& s) {
    std::istringstream in(s);
    std::vector<std::string> t;
    std::string x;
    while (in >> x) t.push_back(x);
    return t;
}

bool meta(Session& s, const std::string& line, std::string& reply) {
    auto t = split_ws(line);
    if (t.empty()) return false;
    std::string c = lower(t[0]);
    if (c == "quit" || c == "exit") {
        s.running = false;
        reply = "bye";
        return true;
    }
    if (c == "list") {
        std::ostringstream os;
        if (s.doc.scene.nodes.empty()) os << "(empty scene)";
        for (int i = 0; i < static_cast<int>(s.doc.scene.nodes.size()); ++i) {
            const Node& n = s.doc.scene.nodes[i];
            os << (i == s.doc.scene.selection ? "* " : "  ") << n.name << "  " << node_type_name(n.type);
            os << "  (" << n.translation.x << "," << n.translation.y << "," << n.translation.z << ")\n";
        }
        reply = os.str();
        return true;
    }
    if (c == "info") {
        int i = s.doc.scene.selection;
        if (t.size() >= 2) i = s.doc.scene.find_node(t[1]);
        if (i < 0) {
            reply = "no selection";
            return true;
        }
        const Node& n = s.doc.scene.nodes[i];
        std::ostringstream os;
        os << n.name << " id=" << n.id << " type=" << node_type_name(n.type) << "\n"
           << "  t " << n.translation.x << " " << n.translation.y << " " << n.translation.z << "\n"
           << "  r " << n.rotation_deg.x << " " << n.rotation_deg.y << " " << n.rotation_deg.z << "\n"
           << "  s " << n.scale.x << " " << n.scale.y << " " << n.scale.z;
        if (n.mesh >= 0 && n.mesh < static_cast<int>(s.doc.scene.meshes.size())) {
            const Mesh& mesh = s.doc.scene.meshes[n.mesh];
            os << "\n  " << mesh.faces.size() << " faces, " << mesh.positions.size() << " verts";
            os << "  mode=" << edit_mode_name(s.doc.scene.edit_mode);
            if (!s.doc.scene.face_sel.empty()) {
                os << "  facesel=";
                for (int x : s.doc.scene.face_sel) os << x << ",";
            }
            os << "\n  faces:";
            int shown = std::min(12, static_cast<int>(mesh.faces.size()));
            for (int fi = 0; fi < shown; ++fi) {
                Vec3 nrm = face_normal(mesh, fi);
                os << " [" << fi << " n=" << nrm.x << "," << nrm.y << "," << nrm.z << "]";
            }
        }
        reply = os.str();
        return true;
    }
    if (c == "export") {
        if (t.size() < 2) {
            reply = "export <path.gltf|glb>";
            return true;
        }
        // Bake a preview into the GLB only when the session asks for one; with
        // it off the exporter is handed an empty preview and writes the file it
        // always wrote.
        GlbPreview pv;
        if (s.glb_preview) {
            if (!preview_cost_ok(s.glb_preview_w, s.glb_preview_h, s.samples)) {
                s.last_export_ok = false;
                reply = "export: " + preview_cost_message(s.glb_preview_w, s.glb_preview_h, s.samples);
                return true;
            }
            auto img = render_preview(s.doc.scene, s.glb_preview_w, s.glb_preview_h, s.samples, s.shading());
            std::string perr;
            if (!encode_png(img, pv.png, perr)) {
                reply = "export: " + perr;
                return true;
            }
            if (pv.png.size() > kGlbPreviewMaxBytes) {
                reply = "export: baked preview would be " + std::to_string(pv.png.size() / 1024) +
                        " KB, over the " + std::to_string(kGlbPreviewMaxBytes / 1024) +
                        " KB cap -- lower it with glbpreview <WxH> or --preview-size\n" + preview_usage();
                return true;
            }
            pv.enabled = true;
            pv.w = img.w;
            pv.h = img.h;
            pv.samples = s.samples;
        }
        std::string err;
        s.last_export_ok = export_gltf(s.doc.scene, t[1], pv, err);
        if (!s.last_export_ok) {
            reply = err;
            return true;
        }
        if (pv.enabled)
            reply = "wrote " + t[1] + " (baked " + std::to_string(pv.w) + "x" + std::to_string(pv.h) +
                    " preview, " + std::to_string(pv.png.size() / 1024) + " KB PNG, " +
                    std::to_string(pv.samples) + "x" + std::to_string(pv.samples) + " samples)";
        else
            reply = "wrote " + t[1];
        return true;
    }
    if (c == "preview") {
        std::string path = t.size() >= 2 ? t[1] : s.last_preview;
        s.last_preview = path;
        auto img = render_preview(s.doc.scene, 960, 540, s.samples, s.shading());
        std::string err;
        if (!write_png(img, path, err)) {
            reply = err;
            return true;
        }
        reply = "wrote " + path + " (" + std::to_string(img.w) + "x" + std::to_string(img.h) + ", " +
                std::to_string(s.samples) + "x" + std::to_string(s.samples) + " samples" +
                shading_suffix(s.contact_shadow, s.ao) + ")";
        return true;
    }
    // Supersampling quality for subsequent previews. Accepts the same range as
    // --samples so the REPL and the command line cannot disagree.
    if (c == "samples" || c == "aa") {
        if (t.size() < 2) {
            reply = "samples is " + std::to_string(s.samples) + " \u00b7 " + samples_usage();
            return true;
        }
        int n = 0;
        std::string err;
        if (!parse_samples(t[1], n, err)) {
            // Handled, not passed through: an invalid value must never fall
            // through to the LLM, which used to answer it as prose.
            reply = "samples rejected: " + err + "\n" + samples_usage();
            return true;
        }
        s.samples = n;
        reply = "samples set to " + std::to_string(n) + " (" + std::to_string(n) + "x" +
                std::to_string(n) + " supersampling)";
        return true;
    }
    // Whether the next export carries a preview, and at what resolution.
    // Same range and defaults as --glb-preview / --preview-size.
    if (c == "glbpreview" || c == "embedpreview") {
        if (t.size() < 2) {
            reply = std::string("glb preview is ") + (s.glb_preview ? "on" : "off") + " at " +
                    std::to_string(s.glb_preview_w) + "x" + std::to_string(s.glb_preview_h) + " \u00b7 " +
                    preview_usage();
            return true;
        }
        std::string arg = lower(t[1]);
        if (arg == "on" || arg == "off") {
            s.glb_preview = arg == "on";
            reply = std::string("glb preview ") + (s.glb_preview ? "on" : "off") + " at " +
                    std::to_string(s.glb_preview_w) + "x" + std::to_string(s.glb_preview_h);
            return true;
        }
        if (arg == "default") {
            s.glb_preview_w = kPreviewDefaultW;
            s.glb_preview_h = kPreviewDefaultH;
            reply = "glb preview size reset to " + std::to_string(s.glb_preview_w) + "x" +
                    std::to_string(s.glb_preview_h);
            return true;
        }
        int pw = 0, ph = 0;
        std::string err;
        if (!parse_preview_size(t[1], pw, ph, err)) {
            // Handled, not passed through: an invalid value must never fall
            // through to the LLM, which would answer it as prose.
            reply = "glbpreview rejected: " + err + "\n" + preview_usage();
            return true;
        }
        s.glb_preview_w = pw;
        s.glb_preview_h = ph;
        reply = "glb preview size set to " + std::to_string(pw) + "x" + std::to_string(ph) +
                (s.glb_preview ? "" : " (glb preview is off \u00b7 glbpreview on to bake it)");
        return true;
    }
    // The two look passes for subsequent previews. Same range and same one
    // authority as --contact-shadow / --ao, so the REPL and the command line
    // cannot disagree. 0 is accepted here and means "off", unlike samples.
    if (c == "shadow" || c == "contact-shadow" || c == "contact_shadow") {
        if (t.size() < 2) {
            reply = "contact shadow is " + std::to_string(s.contact_shadow) + " \u00b7 " +
                    contact_shadow_usage();
            return true;
        }
        int n = 0;
        std::string err;
        if (!parse_contact_shadow(t[1], n, err)) {
            // Handled, not passed through: an invalid value must never fall
            // through to the LLM, which would answer it as prose.
            reply = "contact shadow rejected: " + err + "\n" + contact_shadow_usage();
            return true;
        }
        s.contact_shadow = n;
        reply = n == 0 ? std::string("contact shadows off")
                       : "contact shadow set to " + std::to_string(n) + " of " +
                             std::to_string(kContactShadowMax);
        return true;
    }
    if (c == "ao" || c == "occlusion") {
        if (t.size() < 2) {
            reply = "ao is " + std::to_string(s.ao) + " \u00b7 " + ao_usage();
            return true;
        }
        int n = 0;
        std::string err;
        if (!parse_ao(t[1], n, err)) {
            reply = "ao rejected: " + err + "\n" + ao_usage();
            return true;
        }
        s.ao = n;
        reply = n == 0 ? std::string("ambient occlusion off")
                       : "ao set to " + std::to_string(n) + " of " + std::to_string(kAoMax);
        return true;
    }
    if (c == "shading") {
        reply = shading_note(s.contact_shadow, s.ao) + " \u00b7 " + shading_usage();
        return true;
    }
    if (c == "save") {
        if (t.size() < 2) {
            reply = "save <path.json>";
            return true;
        }
        std::string err;
        if (!save_scene(s.doc.scene, t[1], err)) {
            reply = err;
            return true;
        }
        reply = "saved " + t[1];
        return true;
    }
    if (c == "open" || c == "load") {
        if (t.size() < 2) {
            reply = "open <path.json>";
            return true;
        }
        std::string err;
        s.doc.push_undo();
        if (!load_scene(s.doc.scene, t[1], err)) {
            reply = err;
            return true;
        }
        reply = "opened " + t[1];
        return true;
    }
    return false;
}

}  // namespace

bool handle_line(Session& s, const std::string& line, std::string& reply) {
    std::string t = trim(line);
    if (t.empty()) {
        reply.clear();
        return true;
    }
    if (meta(s, t, reply)) return true;
    AgentReply r = interpret(t, s.doc, s.agent);
    reply = r.text;
    return r.ok;
}

int run_repl(Session& s) {
    std::cout << "kiln  ·  Linux 3D modeler  ·  type help\n";
    std::cout << "preview samples " << s.samples << "x" << s.samples << " (--samples 1.." << kSamplesMax
              << " to change)\n";
    std::cout << "shading " << shading_note(s.contact_shadow, s.ao)
              << "  \u00b7  --contact-shadow / --ao, or the shadow / ao commands\n";
    std::cout << "glb preview " << (s.glb_preview ? "on" : "off") << " at " << s.glb_preview_w << "x"
              << s.glb_preview_h << " (glbpreview on|off|<WxH> to change)\n";
    if (s.agent.api_key.empty())
        std::cout << "no KILN_API_KEY — built-in agent + DSL only\n";
    else
        std::cout << "LLM " << s.agent.model << " @ " << s.agent.api_base << "\n";
    std::string line;
    while (s.running && std::cout << "kiln> " && std::getline(std::cin, line)) {
        std::string reply;
        bool ok = handle_line(s, line, reply);
        if (!reply.empty()) std::cout << (ok ? "" : "error: ") << reply << "\n";
    }
    return 0;
}

int run_oneshot(Session& s, const std::vector<std::string>& prompts, const std::string& export_path,
                const std::string& preview_path) {
    for (const auto& p : prompts) {
        std::string reply;
        bool ok = handle_line(s, p, reply);
        std::cout << (ok ? "ok  " : "error  ") << p << "\n    " << reply << "\n";
        if (!ok) return 1;
    }
    if (!export_path.empty()) {
        std::string reply;
        handle_line(s, "export " + export_path, reply);
        std::cout << reply << "\n";
        // An export that could not write its file is a failure, not a note.
        // The error text goes to stderr as well so `--export a.glb >/dev/null`
        // still tells the caller what went wrong.
        if (!s.last_export_ok) {
            std::cerr << reply << "\n";
            return EXIT_FAILURE;
        }
    }
    if (!preview_path.empty()) {
        std::string reply;
        handle_line(s, "preview " + preview_path, reply);
        std::cout << reply << "\n";
    }
    return EXIT_SUCCESS;
}

}  // namespace kiln
