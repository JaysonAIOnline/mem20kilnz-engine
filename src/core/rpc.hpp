#pragma once

// Machine interface constants for kiln's JSON-RPC stdio mode (--rpc).
//
// This header owns the protocol version, the error-code table, the method
// names and the op-name list, so the driver in src/rpc/, the help text and the
// documented protocol cannot drift apart. It is header-only and depends on
// nothing but the standard library, matching the other src/core headers.

#include <string>
#include <vector>

namespace kiln {
namespace rpc {

// Bumped only for a breaking change to the envelope. Additive changes (a new
// method, a new op, a new result field) do not move it.
constexpr int kVersion = 1;

constexpr const char* kProtocolName = "kiln-rpc";

// ---------------------------------------------------------------------------
// Error codes.
//
// JSON-RPC 2.0 reserves -32768..-32000 (and -32000..-32099 for the server).
// The four standard codes are used verbatim; everything a *modelling* call can
// go wrong with is reported in the free application range -32000..-32099, which
// is this server's reserved slice.
// ---------------------------------------------------------------------------
namespace err {
constexpr int kParseError = -32700;      // frame was not JSON at all
constexpr int kInvalidRequest = -32600;  // JSON, but not a request object
constexpr int kMethodNotFound = -32601;  // unknown method; data lists what exists
constexpr int kInvalidParams = -32602;   // params wrong shape or invalid value
constexpr int kInternalError = -32603;   // kiln itself failed
// Application codes (server-reserved slice).
constexpr int kOpFailed = -32000;        // a scene op ran and reported a failure
constexpr int kNoScene = -32001;         // op needs a node/mesh; none is selected
constexpr int kNotFound = -32002;        // named node / file does not exist
constexpr int kNoCamera = -32003;        // nothing to render with
constexpr int kIoError = -32004;         // write/read of a file failed
constexpr int kBadSamples = -32005;      // samples outside the accepted range
constexpr int kUnsupported = -32006;     // a real feature kiln does not implement
}  // namespace err

// One-line meaning per code, for the error message and the help text.
inline std::string error_name(int code) {
    switch (code) {
        case err::kParseError: return "parse error";
        case err::kInvalidRequest: return "invalid request";
        case err::kMethodNotFound: return "method not found";
        case err::kInvalidParams: return "invalid params";
        case err::kInternalError: return "internal error";
        case err::kOpFailed: return "op failed";
        case err::kNoScene: return "no target";
        case err::kNotFound: return "not found";
        case err::kNoCamera: return "no camera";
        case err::kIoError: return "io error";
        case err::kBadSamples: return "bad samples";
        case err::kUnsupported: return "unsupported";
        default: return "error";
    }
}

// ---------------------------------------------------------------------------
// Methods.
// ---------------------------------------------------------------------------
inline const std::vector<std::string>& method_names() {
    static const std::vector<std::string> kMethods = {
        "op",        // apply one op object; params = the op itself
        "ops",       // apply an array of ops as one transaction
        "command",   // one DSL / English / JSON line, exactly like the REPL
        "describe",  // structured scene summary (same JSON as the describe op)
        "list",      // nodes, selection marker, transform
        "info",      // detail for one node, or the selection
        "samples",   // read (no param) or set the supersampling factor
        "render",    // render a preview PNG at a given quality
        "export",    // write glTF/GLB
        "save",      // write kiln scene JSON
        "open",      // read kiln scene JSON
        "reset",     // empty the scene (same as the new op)
        "ping",      // liveness / protocol version / op count
        "shutdown",  // end the session cleanly
    };
    return kMethods;
}

// True when `name` is a method this server implements.
inline bool is_method(const std::string& name) {
    for (const auto& m : method_names())
        if (m == name) return true;
    return false;
}

// ---------------------------------------------------------------------------
// Op names.
//
// A mirror of the op kinds apply_op() dispatches on in src/ops/ops.cpp. It is
// advisory only: the driver forwards every op object to apply_op() verbatim and
// lets it decide, so an op added to ops.cpp is reachable over this interface
// the moment it compiles. This list exists so `ping` can report coverage and so
// a typo can be answered with the list of names instead of a bare rejection.
//
// Keep in step with ops.cpp when adding an op; the [core] section of
// docs/RPC.md is generated from the same list.
// ---------------------------------------------------------------------------
inline const std::vector<std::string>& op_names() {
    static const std::vector<std::string> kOps = {
        // session / history
        "new", "undo", "redo",
        // objects
        "create", "delete", "duplicate", "rename", "select", "mode", "parent",
        // transforms and materials
        "move", "set_translation", "translate", "rotate", "set_rotation",
        "scale", "set_scale", "color", "set_material", "apply",
        // selection
        "select_faces", "select_edges", "select_verts", "select_vertices",
        "select_edge_loop", "select_edge_ring", "grow", "select_more",
        "invert", "linked",
        // mesh editing
        "extrude", "inset", "bevel", "loop_cut", "loopcut", "delete_faces",
        "subdivide", "subdiv", "merge", "weld", "weld_distance", "fill",
        "mirror", "solidify", "array", "spin", "bridge", "join", "ground",
        "to_ground", "origin", "triangulate", "poke", "flip", "flip_normals",
        "dissolve", "duplicate_faces", "separate", "smooth", "push", "sphere",
        "to_sphere", "flatten", "randomize", "bisect", "snap", "split",
        "connect",
        // rigging and animation
        "rig", "bind", "pose", "rest", "shape",
        // level of detail
        "lod",
        // import / export
        "import", "import_gltf", "import_obj", "export_obj",
        // mesh conditioning
        "decimate", "recalc_normals", "separate_parts", "separate_loose",
        // inspection
        "describe",
        // camera
        "lookat", "frame", "focus", "set_camera",
    };
    return kOps;
}

// True when `name` appears in the advisory op table.
inline bool is_op_name(const std::string& name) {
    for (const auto& o : op_names())
        if (o == name) return true;
    return false;
}

inline std::size_t op_count() { return op_names().size(); }

// The closest known op name to `name`, or "" when nothing is close enough.
// Deliberately tiny: a case difference or a dropped/extra character is what a
// hand-written call usually gets wrong. Used only to make a typo's error data
// helpful — it never changes whether an op runs.
inline std::string nearest_op(const std::string& name) {
    if (name.empty()) return "";
    auto folded = [](std::string s) {
        for (char& c : s)
            if (c >= 'A' && c <= 'Z') c = static_cast<char>(c - 'A' + 'a');
        return s;
    };
    std::string want = folded(name);
    for (const auto& o : op_names()) {
        if (folded(o) == want) return o;  // only a case difference
    }
    std::size_t best = std::string::npos;
    std::string hit;
    for (const auto& o : op_names()) {
        if (o.size() > want.size()) continue;
        // Everything the caller wrote must appear, in order, in the candidate.
        std::size_t i = 0, j = 0;
        while (i < want.size() && j < o.size()) {
            if (want[i] == o[j]) ++i;
            ++j;
        }
        if (i != want.size()) continue;
        std::size_t cost = o.size() - want.size();
        if (cost < best) {
            best = cost;
            hit = o;
        }
    }
    return best <= 3 ? hit : std::string();
}

}  // namespace rpc
}  // namespace kiln
