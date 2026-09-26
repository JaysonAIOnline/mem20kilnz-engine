#include "rpc.hpp"

#include "../core/rpc.hpp"
#include "../core/samples.hpp"
#include "../io/gltf.hpp"
#include "../render/software.hpp"

#include <nlohmann/json.hpp>

#include <cmath>
#include <iostream>
#include <string>
#include <vector>

namespace kiln {
namespace {

using nlohmann::json;

// ---------------------------------------------------------------------------
// Envelopes. Every response -- success or failure -- is one of these objects on
// one line, so a client never has to guess whether the server is still there.
// ---------------------------------------------------------------------------

json ok_envelope(const json& id, json result) {
    json r = json::object();
    r["jsonrpc"] = "2.0";
    r["id"] = id;
    r["result"] = std::move(result);
    return r;
}

json err_envelope(const json& id, int code, const std::string& message, json data = json()) {
    json e = json::object();
    e["code"] = code;
    e["message"] = message;
    if (!data.is_null() && data != json()) e["data"] = std::move(data);
    json r = json::object();
    r["jsonrpc"] = "2.0";
    r["id"] = id;
    r["error"] = std::move(e);
    return r;
}

// A result that reports success/failure *and* carries the op's own message.
// Applied ops use this rather than the error channel so a batch can be
// inspected call by call; the error channel is for protocol-level failures.
json op_result_json(bool ok, const std::string& message, json extra = json()) {
    json r = json::object();
    r["ok"] = ok;
    r["message"] = message;
    if (!extra.is_null() && extra != json()) r["data"] = std::move(extra);
    return r;
}

// ---------------------------------------------------------------------------
// Param helpers. Each one reports *why* rather than coercing silently -- a
// wrong type must never be quietly read as a default.
// ---------------------------------------------------------------------------

bool want_string(const json& j, const char* key, std::string& out, std::string& err) {
    if (!j.contains(key)) {
        err = std::string("missing required field \"") + key + "\"";
        return false;
    }
    if (!j[key].is_string()) {
        err = std::string("field \"") + key + "\" must be a string";
        return false;
    }
    out = j[key].get<std::string>();
    return true;
}

bool optional_int(const json& j, const char* key, int& out, std::string& err) {
    if (!j.contains(key) || j[key].is_null()) return true;
    if (j[key].is_number_integer()) {
        out = j[key].get<int>();
        return true;
    }
    if (j[key].is_number_float()) {
        err = std::string("field \"") + key + "\" must be a whole number";
        return false;
    }
    err = std::string("field \"") + key + "\" must be a number";
    return false;
}

bool optional_positive_int(const json& j, const char* key, int& out, std::string& err) {
    if (!optional_int(j, key, out, err)) return false;
    if (!j.contains(key) || j[key].is_null()) return true;
    if (out <= 0) {
        err = std::string("field \"") + key + "\" must be positive";
        return false;
    }
    if (out > 8192) {
        err = std::string("field \"") + key + "\" is above the maximum of 8192";
        return false;
    }
    return true;
}

// ---------------------------------------------------------------------------
// Methods. Each returns the `result` payload, or fills `code`/`message`/`data`
// and reports false. None of them terminate the loop -- a bad call is data.
// ---------------------------------------------------------------------------

bool m_op(Session& s, const json& params, json& result, int& code, std::string& message, json& data) {
    if (!params.is_object()) {
        code = rpc::err::kInvalidParams;
        message = "op expects a single op object";
        data = {{"received", params.type_name()}};
        return false;
    }
    OpResult r = apply_op(s.doc, params);
    result = op_result_json(r.ok, r.message, r.data);
    return true;
}

bool m_ops(Session& s, const json& params, json& result, int& code, std::string& message, json& data) {
    if (!params.is_array()) {
        code = rpc::err::kInvalidParams;
        message = "ops expects an array of op objects";
        data = {{"received", params.type_name()}};
        return false;
    }
    // Applied one at a time, stopping at the first failure, so the client can
    // see exactly which call in the batch went wrong.
    json applied = json::array();
    bool all_ok = true;
    for (std::size_t i = 0; i < params.size(); ++i) {
        OpResult r = apply_op(s.doc, params[i]);
        applied.push_back(op_result_json(r.ok, r.message,
                                           {{"index", static_cast<int>(i)},
                                            {"op_data", r.data}}));
        if (!r.ok) {
            all_ok = false;
            break;
        }
    }
    // The message is built before the move below: argument evaluation order is
    // unspecified in C++, so reading applied.size() inside the same call that
    // moves `applied` can observe it already emptied.
    const std::string summary = all_ok ? ("applied " + std::to_string(applied.size()) + " ops")
                                       : "batch stopped at the first failed op";
    result = op_result_json(all_ok, summary, {{"applied", std::move(applied)}});
    return true;
}

bool m_command(Session& s, const json& params, json& result, int& code, std::string& message, json& data) {
    std::string line;
    if (params.is_string()) {
        line = params.get<std::string>();
    } else if (params.is_object() && !want_string(params, "line", line, message)) {
        code = rpc::err::kInvalidParams;
        data = {{"hint", "command takes a string, or an object with a \"line\" string"}};
        return false;
    } else if (!params.is_object()) {
        code = rpc::err::kInvalidParams;
        message = "command expects a string or an object with \"line\"";
        data = {{"received", params.type_name()}};
        return false;
    }
    // Exactly the REPL path: DSL, English, raw JSON, meta commands, all of it.
    std::string reply;
    bool ok = handle_line(s, line, reply);
    result = op_result_json(ok, reply, {{"line", line}});
    return true;
}

bool m_describe(Session& s, const json&, json& result, int&, std::string&, json&) {
    result = describe_scene(s.doc.scene);
    return true;
}

bool m_list(Session& s, const json&, json& result, int&, std::string&, json&) {
    json nodes = json::array();
    for (int i = 0; i < static_cast<int>(s.doc.scene.nodes.size()); ++i) {
        const Node& n = s.doc.scene.nodes[i];
        json jn = json::object();
        jn["index"] = i;
        jn["name"] = n.name;
        jn["type"] = node_type_name(n.type);
        jn["selected"] = (i == s.doc.scene.selection);
        jn["t"] = json::array({n.translation.x, n.translation.y, n.translation.z});
        nodes.push_back(std::move(jn));
    }
    result = json::object();
    result["count"] = static_cast<int>(nodes.size());
    result["selection"] =
        s.doc.scene.selection >= 0 && s.doc.scene.selection < int(s.doc.scene.nodes.size())
            ? s.doc.scene.nodes[s.doc.scene.selection].name
            : "";
    result["nodes"] = std::move(nodes);
    return true;
}

/// The provenance record: every op applied in this session, in order, with its
/// verdict. `clear` empties it without touching the scene.
bool m_journal(Session& s, const json& params, json& result, int&, std::string&, json&) {
    if (params.is_object() && params.value("clear", false)) {
        int n = static_cast<int>(s.doc.journal.size());
        s.doc.journal.clear();
        result = json::object();
        result["cleared"] = n;
        result["count"] = 0;
        result["total"] = 0;
        result["failed"] = 0;
        result["entries"] = json::array();
        return true;
    }
    int since = 0;
    if (params.is_object() && params.contains("since")) {
        since = std::max(0, params["since"].get<int>());
    }
    if (since > static_cast<int>(s.doc.journal.size())) since = static_cast<int>(s.doc.journal.size());
    json entries = json::array();
    for (int i = since; i < static_cast<int>(s.doc.journal.size()); ++i) {
        entries.push_back(s.doc.journal[static_cast<std::size_t>(i)]);
    }
    int total = static_cast<int>(s.doc.journal.size());
    int failed = 0;
    for (const auto& e : s.doc.journal) {
        if (e.value("ok", true) == false) ++failed;
    }
    result = json::object();
    result["count"] = static_cast<int>(entries.size());
    result["total"] = total;
    result["failed"] = failed;
    result["entries"] = std::move(entries);
    return true;
}

bool m_info(Session& s, const json& params, json& result, int& code, std::string& message, json& data) {
    Scene& sc = s.doc.scene;
    int i = sc.selection;
    if (params.is_string()) {
        i = sc.find_node(params.get<std::string>());
        if (i < 0) {
            code = rpc::err::kNotFound;
            message = "no node named \"" + params.get<std::string>() + "\"";
            return false;
        }
    } else if (params.is_object() && params.contains("name")) {
        std::string nm;
        if (!want_string(params, "name", nm, message)) {
            code = rpc::err::kInvalidParams;
            return false;
        }
        i = sc.find_node(nm);
        if (i < 0) {
            code = rpc::err::kNotFound;
            message = "no node named \"" + nm + "\"";
            return false;
        }
    }
    if (i < 0) {
        code = rpc::err::kNoScene;
        message = "no selection and no name given";
        data = {{"hint", "pass a node name, or select one first"}};
        return false;
    }
    const Node& n = sc.nodes[i];
    json jn = json::object();
    jn["index"] = i;
    jn["id"] = n.id;
    jn["name"] = n.name;
    jn["type"] = node_type_name(n.type);
    jn["t"] = json::array({n.translation.x, n.translation.y, n.translation.z});
    jn["r"] = json::array({n.rotation_deg.x, n.rotation_deg.y, n.rotation_deg.z});
    jn["s"] = json::array({n.scale.x, n.scale.y, n.scale.z});
    if (n.type == NodeType::Camera) jn["fov"] = n.fov_deg;
    if (n.type == NodeType::Light) {
        jn["intensity"] = n.light_intensity;
        jn["light"] = (n.light == LightType::Point) ? "point" : "sun";
    }
    if (n.type == NodeType::Mesh && n.mesh >= 0 && n.mesh < int(sc.meshes.size())) {
        const Mesh& m = sc.meshes[n.mesh];
        jn["faces"] = static_cast<int>(m.faces.size());
        jn["verts"] = static_cast<int>(m.positions.size());
        jn["lod"] = m.lod;
        jn["lods"] = static_cast<int>(m.lods.size());
        jn["smooth"] = m.smooth;
        jn["face_sel"] = sc.face_sel;
        jn["edge_sel_count"] = static_cast<int>(sc.edge_sel.size());
        jn["vert_sel_count"] = static_cast<int>(sc.vert_sel.size());
    }
    jn["edit_mode"] = edit_mode_name(sc.edit_mode);
    result = std::move(jn);
    return true;
}

bool m_samples(Session& s, const json& params, json& result, int& code, std::string& message, json& data) {
    // Read form: no params at all.
    bool is_read = params.is_null() || (params.is_object() && !params.contains("value")) ||
                   (params.is_object() && params.contains("value") && params["value"].is_null());
    if (is_read) {
        result = json::object();
        result["samples"] = s.samples;
        result["min"] = kSamplesMin;
        result["max"] = kSamplesMax;
        result["default"] = kDefaultSamples;
        return true;
    }
    json raw = params.is_object() ? params["value"] : params;
    if (!raw.is_number_integer()) {
        code = rpc::err::kInvalidParams;
        message = "samples needs a whole number";
        data = {{"received", raw.type_name()}, {"usage", samples_usage()}};
        return false;
    }
    int n = raw.get<int>();
    if (!samples_in_range(n)) {
        code = rpc::err::kBadSamples;
        message = "samples " + std::to_string(n) + " is outside [" + std::to_string(kSamplesMin) + ", " +
                  std::to_string(kSamplesMax) + "]";
        data = {{"min", kSamplesMin}, {"max", kSamplesMax}, {"default", kDefaultSamples},
                {"usage", samples_usage()}};
        return false;
    }
    s.samples = n;
    result = json::object();
    result["samples"] = n;
    result["min"] = kSamplesMin;
    result["max"] = kSamplesMax;
    result["default"] = kDefaultSamples;
    return true;
}

bool m_render(Session& s, const json& params, json& result, int& code, std::string& message, json& data) {
    if (!params.is_object()) {
        code = rpc::err::kInvalidParams;
        message = "render expects an object with \"path\"";
        data = {{"received", params.type_name()}};
        return false;
    }
    std::string path;
    if (!want_string(params, "path", path, message)) {
        code = rpc::err::kInvalidParams;
        return false;
    }
    int w = 960, h = 540;
    if (!optional_positive_int(params, "width", w, message)) {
        code = rpc::err::kInvalidParams;
        return false;
    }
    if (!optional_positive_int(params, "height", h, message)) {
        code = rpc::err::kInvalidParams;
        return false;
    }
    if (w > 4096) {
        code = rpc::err::kInvalidParams;
        message = "width is above the maximum of 4096";
        return false;
    }
    if (h > 4096) {
        code = rpc::err::kInvalidParams;
        message = "height is above the maximum of 4096";
        return false;
    }
    // samples: use this render's own value when given, otherwise the session's,
    // which is the same field the --samples flag and the REPL both write.
    int ss = s.samples;
    if (params.contains("samples") && !params["samples"].is_null()) {
        if (!params["samples"].is_number_integer()) {
            code = rpc::err::kInvalidParams;
            message = "samples needs a whole number";
            data = {{"usage", samples_usage()}};
            return false;
        }
        int raw = params["samples"].get<int>();
        if (!samples_in_range(raw)) {
            code = rpc::err::kBadSamples;
            message = "samples " + std::to_string(raw) + " is outside [" + std::to_string(kSamplesMin) +
                      ", " + std::to_string(kSamplesMax) + "]";
            data = {{"min", kSamplesMin}, {"max", kSamplesMax}, {"default", kDefaultSamples},
                    {"usage", samples_usage()}};
            return false;
        }
        ss = raw;
    }
    if (s.doc.scene.nodes.empty()) {
        code = rpc::err::kNoCamera;
        message = "nothing to render: the scene is empty";
        return false;
    }
    auto img = render_preview(s.doc.scene, w, h, ss, s.shading());
    std::string err;
    if (!write_png(img, path, err)) {
        code = rpc::err::kIoError;
        message = err;
        data = {{"path", path}};
        return false;
    }
    s.last_preview = path;
    result = json::object();
    result["path"] = path;
    result["width"] = img.w;
    result["height"] = img.h;
    result["samples"] = ss;
    return true;
}

bool m_export(Session& s, const json& params, json& result, int& code, std::string& message, json& data) {
    if (!params.is_object()) {
        code = rpc::err::kInvalidParams;
        message = "export expects an object with \"path\"";
        data = {{"received", params.type_name()}};
        return false;
    }
    std::string path;
    if (!want_string(params, "path", path, message)) {
        code = rpc::err::kInvalidParams;
        return false;
    }
    std::string err;
    if (!export_gltf(s.doc.scene, path, err)) {
        code = rpc::err::kIoError;
        message = err;
        data = {{"path", path}};
        return false;
    }
    result = json::object();
    result["path"] = path;
    result["nodes"] = static_cast<int>(s.doc.scene.nodes.size());
    result["meshes"] = static_cast<int>(s.doc.scene.meshes.size());
    return true;
}

bool m_save(Session& s, const json& params, json& result, int& code, std::string& message, json& data) {
    if (!params.is_object()) {
        code = rpc::err::kInvalidParams;
        message = "save expects an object with \"path\"";
        data = {{"received", params.type_name()}};
        return false;
    }
    std::string path;
    if (!want_string(params, "path", path, message)) {
        code = rpc::err::kInvalidParams;
        return false;
    }
    std::string err;
    if (!save_scene(s.doc.scene, path, err)) {
        code = rpc::err::kIoError;
        message = err;
        data = {{"path", path}};
        return false;
    }
    result = json::object();
    result["path"] = path;
    return true;
}

bool m_open(Session& s, const json& params, json& result, int& code, std::string& message, json& data) {
    if (!params.is_object()) {
        code = rpc::err::kInvalidParams;
        message = "open expects an object with \"path\"";
        data = {{"received", params.type_name()}};
        return false;
    }
    std::string path;
    if (!want_string(params, "path", path, message)) {
        code = rpc::err::kInvalidParams;
        return false;
    }
    // Same as the `open` meta command: undoable, so a wrong file is recoverable.
    s.doc.push_undo();
    std::string err;
    if (!load_scene(s.doc.scene, path, err)) {
        code = rpc::err::kIoError;
        message = err;
        data = {{"path", path}};
        return false;
    }
    result = json::object();
    result["path"] = path;
    result["nodes"] = static_cast<int>(s.doc.scene.nodes.size());
    return true;
}

bool m_reset(Session& s, const json&, json& result, int&, std::string&, json&) {
    s.doc.reset();
    result = json::object();
    result["nodes"] = 0;
    return true;
}

bool m_ping(Session&, const json&, json& result, int&, std::string&, json&) {
    result = json::object();
    result["protocol"] = rpc::kProtocolName;
    result["version"] = rpc::kVersion;
    result["ops"] = static_cast<int>(rpc::op_count());
    result["methods"] = rpc::method_names();
    result["ready"] = true;
    return true;
}

// Renders one line of model detail into the help that `initialize` returns, so
// an agent that has never seen kiln can still send a valid first op.
json help_payload() {
    json j = json::object();
    j["name"] = "kiln";
    j["protocol"] = rpc::kProtocolName;
    j["version"] = rpc::kVersion;
    j["usage"] = "kiln --rpc  (one JSON-RPC 2.0 request per line on stdin)";
    j["methods"] = rpc::method_names();
    j["ops"] = rpc::op_names();
    j["samples"] = {{"min", kSamplesMin}, {"max", kSamplesMax}, {"default", kDefaultSamples}};
    j["notes"] = json::array({"Y is up; a unit cube resting on the ground sits at [0, 0.5, 0]",
                              "op results carry ok/message; protocol failures use the error object",
                              "each response is exactly one line of JSON"});
    return j;
}

}  // namespace

std::string rpc_usage() {
    return "--rpc runs the JSON-RPC 2.0 stdio session (one request per line; no banner, JSON only)";
}

int run_rpc(Session& s) {
    // stdout carries protocol traffic only, and must be flushed per line so a
    // client blocked on read() is never left waiting on a buffer.
    std::ios::sync_with_stdio(false);
    std::string line;
    while (std::getline(std::cin, line)) {
        if (line.empty()) continue;

        json req;
        try {
            req = json::parse(line);
        } catch (const std::exception& e) {
            // Not JSON at all. JSON-RPC wants id:null here, since no id could
            // be recovered from the frame.
            std::cout << err_envelope(nullptr, rpc::err::kParseError,
                                      std::string("request is not valid JSON: ") + e.what(),
                                      {{"received_bytes", static_cast<int>(line.size())}})
                             .dump()
                      << "\n";
            std::cout.flush();
            continue;
        }

        if (!req.is_object()) {
            std::cout << err_envelope(nullptr, rpc::err::kInvalidRequest,
                                      "request must be a JSON object")
                             .dump()
                      << "\n";
            std::cout.flush();
            continue;
        }

        json id = req.contains("id") ? req["id"] : json(nullptr);
        bool is_notification = !req.contains("id");
        std::string method;
        if (!req.contains("method") || !req["method"].is_string()) {
            if (!is_notification) {
                std::cout << err_envelope(id, rpc::err::kInvalidRequest,
                                          "request must have a string \"method\"")
                                 .dump()
                          << "\n";
                std::cout.flush();
            }
            continue;
        }
        method = req["method"].get<std::string>();
        json params = req.contains("params") ? req["params"] : json(nullptr);

        json result = json();
        int code = 0;
        std::string message;
        json data = json();
        bool ok = true;

        if (method == "op") ok = m_op(s, params, result, code, message, data);
        else if (method == "ops") ok = m_ops(s, params, result, code, message, data);
        else if (method == "command") ok = m_command(s, params, result, code, message, data);
        else if (method == "describe") ok = m_describe(s, params, result, code, message, data);
        else if (method == "list") ok = m_list(s, params, result, code, message, data);
        else if (method == "journal") ok = m_journal(s, params, result, code, message, data);
        else if (method == "info") ok = m_info(s, params, result, code, message, data);
        else if (method == "samples") ok = m_samples(s, params, result, code, message, data);
        else if (method == "render") ok = m_render(s, params, result, code, message, data);
        else if (method == "export") ok = m_export(s, params, result, code, message, data);
        else if (method == "save") ok = m_save(s, params, result, code, message, data);
        else if (method == "open") ok = m_open(s, params, result, code, message, data);
        else if (method == "reset") ok = m_reset(s, params, result, code, message, data);
        else if (method == "ping") ok = m_ping(s, params, result, code, message, data);
        else if (method == "initialize") {
            result = help_payload();
        } else if (method == "help") {
            result = help_payload();
        } else if (method == "shutdown") {
            result = json::object();
            result["bye"] = true;
            // A notification must not be answered; a request must be, then the
            // loop ends.
            if (!is_notification) {
                std::cout << ok_envelope(id, std::move(result)).dump() << "\n";
                std::cout.flush();
            }
            return 0;
        } else {
            ok = false;
            code = rpc::err::kMethodNotFound;
            message = "unknown method \"" + method + "\"";
            data = {{"methods", rpc::method_names()}};
        }

        // A notification gets no response at all, per JSON-RPC 2.0 -- except a
        // parse/invalid-request failure, which is always reported.
        if (is_notification) continue;

        if (ok) {
            std::cout << ok_envelope(id, std::move(result)).dump() << "\n";
        } else {
            std::cout << err_envelope(id, code, message, data).dump() << "\n";
        }
        std::cout.flush();
    }
    // EOF: the client closed the pipe. A clean end, not an error.
    return 0;
}

}  // namespace kiln
