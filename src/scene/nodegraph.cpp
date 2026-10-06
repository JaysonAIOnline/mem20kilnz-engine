// Procedural node graph.
//
// The graph exists to generate geometry, and every node that produces geometry
// calls the real mesh operation it stands for: `primitive` calls make_primitive,
// `union` calls boolean_mesh, `remesh` calls remesh_mesh, `displace` calls
// displace_mesh. Nothing here fabricates a stand-in, so a graph produces the same
// mesh the equivalent op sequence would, which is what makes the graph worth
// having rather than a second way to describe nothing.
//
// Evaluation is demand-driven from the output node with explicit cycle
// detection. A failure names the node that failed and why, because the caller is
// usually an agent that has to correct itself from the message.
//
// Node types and their sockets:
//
//   primitive(primitive:string, size:vec3, segments:int) -> mesh
//   translate(mesh, offset:vec3)                        -> mesh
//   rotate(mesh, degrees:vec3)                          -> mesh
//   scale(mesh, factor:vec3)                             -> mesh
//   join(a:mesh, b:mesh)                                -> mesh
//   union(a:mesh, b:mesh, resolution:int)                -> mesh
//   difference(a:mesh, b:mesh, resolution:int)           -> mesh
//   intersect(a:mesh, b:mesh, resolution:int)             -> mesh
//   remesh(mesh, resolution:int)                         -> mesh
//   displace(mesh, amplitude:float, frequency:float, seed:int, octaves:int) -> mesh
//   bbox_size(mesh)                                      -> vec3
//   math(a:float, b:float, op:string)                   -> float
//   output(mesh)                                         -> mesh

#include "../scene/scene.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <string>
#include <vector>

namespace kiln {

int NodeGraph::find_node(const std::string& n) const {
    for (size_t i = 0; i < nodes.size(); ++i) {
        if (nodes[i].name == n) return static_cast<int>(i);
    }
    return -1;
}

GraphNode* NodeGraph::node_at(int i) {
    if (i < 0 || i >= static_cast<int>(nodes.size())) return nullptr;
    return &nodes[static_cast<size_t>(i)];
}

const GraphNode* NodeGraph::node_at(int i) const {
    if (i < 0 || i >= static_cast<int>(nodes.size())) return nullptr;
    return &nodes[static_cast<size_t>(i)];
}

NodeGraph* Scene::graph_named(const std::string& n) {
    for (NodeGraph& g : graphs) {
        if (g.name == n) return &g;
    }
    return nullptr;
}

const NodeGraph* Scene::graph_named(const std::string& n) const {
    for (const NodeGraph& g : graphs) {
        if (g.name == n) return &g;
    }
    return nullptr;
}

int Scene::find_graph(const std::string& n) const {
    for (size_t i = 0; i < graphs.size(); ++i) {
        if (graphs[i].name == n) return static_cast<int>(i);
    }
    return -1;
}

namespace {

struct SocketSpec {
    const char* name;
    SockValue::Type type;
    SockValue def;
};

/// One node definition: what it takes and how it computes.
struct NodeDef {
    const char* type;
    std::vector<SocketSpec> inputs;
};

/// The node type registry.
///
/// Function-local static, not a value return: `find_def` hands out pointers into
/// this table, so it has to outlive every caller. Returning a vector by value gave
/// out pointers into a temporary that was destroyed the moment find_def returned,
/// and every reader of the socket spec was reading freed memory. That segfaulted
/// inside the vector growth itself, which is a long way from the actual mistake.
const std::vector<NodeDef>& registry() {
    static const std::vector<NodeDef> r = {
        {"primitive",
         {{"primitive", SockValue::Type::String, SockValue::of_string("cube")},
          {"size", SockValue::Type::Vec3, SockValue::of_vec3({1, 1, 1})},
          {"segments", SockValue::Type::Int, SockValue::of_int(24)}}},
        {"translate",
         {{"mesh", SockValue::Type::Mesh, SockValue{}},
          {"offset", SockValue::Type::Vec3, SockValue::of_vec3({0, 0, 0})}}},
        {"rotate",
         {{"mesh", SockValue::Type::Mesh, SockValue{}},
          {"degrees", SockValue::Type::Vec3, SockValue::of_vec3({0, 0, 0})}}},
        {"scale",
         {{"mesh", SockValue::Type::Mesh, SockValue{}},
          {"factor", SockValue::Type::Vec3, SockValue::of_vec3({1, 1, 1})}}},
        {"join",
         {{"a", SockValue::Type::Mesh, SockValue{}},
          {"b", SockValue::Type::Mesh, SockValue{}}}},
        {"union",
         {{"a", SockValue::Type::Mesh, SockValue{}},
          {"b", SockValue::Type::Mesh, SockValue{}},
          {"resolution", SockValue::Type::Int, SockValue::of_int(64)}}},
        {"difference",
         {{"a", SockValue::Type::Mesh, SockValue{}},
          {"b", SockValue::Type::Mesh, SockValue{}},
          {"resolution", SockValue::Type::Int, SockValue::of_int(64)}}},
        {"intersect",
         {{"a", SockValue::Type::Mesh, SockValue{}},
          {"b", SockValue::Type::Mesh, SockValue{}},
          {"resolution", SockValue::Type::Int, SockValue::of_int(64)}}},
        {"remesh",
         {{"mesh", SockValue::Type::Mesh, SockValue{}},
          {"resolution", SockValue::Type::Int, SockValue::of_int(48)}}},
        {"displace",
         {{"mesh", SockValue::Type::Mesh, SockValue{}},
          {"amplitude", SockValue::Type::Float, SockValue::of_float(0.1f)},
          {"frequency", SockValue::Type::Float, SockValue::of_float(2.0f)},
          {"seed", SockValue::Type::Int, SockValue::of_int(0)},
          {"octaves", SockValue::Type::Int, SockValue::of_int(3)}}},
        {"bbox_size", {{"mesh", SockValue::Type::Mesh, SockValue{}}}},
        {"math",
         {{"a", SockValue::Type::Float, SockValue::of_float(0.0f)},
          {"b", SockValue::Type::Float, SockValue::of_float(1.0f)},
          {"op", SockValue::Type::String, SockValue::of_string("add")}}},
        {"output", {{"mesh", SockValue::Type::Mesh, SockValue{}}}},
    };
    return r;
}

const NodeDef* find_def(const std::string& type) {
    for (const NodeDef& d : registry()) {
        if (type == d.type) return &d;
    }
    return nullptr;
}

const char* type_name(SockValue::Type t);

/// Coerce a value to the type a socket declares.
///
/// Numbers cross freely, because an agent that sets an integer where a float is
/// expected means the same thing and should not have to know the difference. A
/// string or a mesh where a number belongs is a real mistake and is reported,
/// rather than being silently read as zero.
bool coerce(const SockValue& in, SockValue::Type want, SockValue& out, std::string& err) {
    if (in.type == want) {
        out = in;
        return true;
    }
    const bool in_num = in.type == SockValue::Type::Float || in.type == SockValue::Type::Int;
    const bool want_num = want == SockValue::Type::Float || want == SockValue::Type::Int;
    if (in_num && want_num) {
        out = in;
        if (want == SockValue::Type::Float) {
            out.type = SockValue::Type::Float;
            out.f = in.type == SockValue::Type::Int ? static_cast<float>(in.i) : in.f;
        } else {
            out.type = SockValue::Type::Int;
            out.i = in.type == SockValue::Type::Float ? static_cast<int>(std::lround(in.f)) : in.i;
        }
        return true;
    }
    err = std::string("cannot use a ") + type_name(in.type) + " where a " +
          type_name(want) + " is expected";
    return false;
}

const char* type_name(SockValue::Type t) {
    switch (t) {
        case SockValue::Type::Float: return "float";
        case SockValue::Type::Int: return "int";
        case SockValue::Type::Vec3: return "vec3";
        case SockValue::Type::String: return "string";
        case SockValue::Type::Mesh: return "mesh";
    }
    return "value";
}

/// Evaluate one node from already-evaluated inputs.
bool run_node(const std::string& type, const std::vector<SockValue>& in, SockValue& out,
              std::string& err) {
    auto mesh_at = [&](size_t i) -> const Mesh* {
        if (i >= in.size() || in[i].type != SockValue::Type::Mesh) return nullptr;
        if (in[i].mesh.positions.empty() || in[i].mesh.faces.empty()) return nullptr;
        return &in[i].mesh;
    };

    if (type == "primitive") {
        PrimType pt;
        if (!parse_prim(in[0].s, pt)) {
            err = "primitive: unknown primitive '" + in[0].s + "'";
            return false;
        }
        Vec3 size = in[1].v;
        if (in[2].i < 3) {
            err = "primitive: segments must be at least 3, got " + std::to_string(in[2].i);
            return false;
        }
        if (pt == PrimType::Plane && size.z == 1 && size.x != 1) size.z = size.x;
        Mesh m = make_primitive(pt, size, in[2].i);
        if (m.faces.empty()) {
            err = "primitive: make_primitive produced no faces";
            return false;
        }
        // The same two steps the `create` op does, and both matter. Curved
        // primitives are smooth-shaded and flat ones are not, which is not only
        // cosmetic: mesh_sync_render welds on (position, normal, uv), so a
        // flat-shaded curved surface cannot share a vertex. And without the sync
        // the render mesh is never built, so displace_mesh, which needs vertex
        // normals, refused the output of this node.
        const bool curved = pt == PrimType::Sphere || pt == PrimType::Cylinder ||
                            pt == PrimType::Cone || pt == PrimType::Torus;
        m.smooth = curved;
        mesh_sync_render(m);
        out = SockValue::of_mesh(std::move(m));
        return true;
    }

    if (type == "translate" || type == "rotate" || type == "scale") {
        const Mesh* src = mesh_at(0);
        if (!src) {
            err = type + ": the mesh input is empty or not a mesh";
            return false;
        }
        Vec3 t{0, 0, 0}, r{0, 0, 0}, s{1, 1, 1};
        if (type == "translate") t = in[1].v;
        else if (type == "rotate") r = in[1].v;
        else s = in[1].v;
        if (type == "scale") {
            if (std::fabs(s.x) < 1e-6f || std::fabs(s.y) < 1e-6f || std::fabs(s.z) < 1e-6f) {
                err = "scale: factor must be non-zero on every axis";
                return false;
            }
        }
        Mesh out_m;
        mesh_append_transformed(out_m, *src, trs(t, r, s));
        out = SockValue::of_mesh(std::move(out_m));
        return true;
    }

    if (type == "join") {
        const Mesh* a = mesh_at(0);
        const Mesh* b = mesh_at(1);
        if (!a) {
            err = "join: input a is empty or not a mesh";
            return false;
        }
        Mesh out_m = *a;
        if (b) {
            mesh_append_transformed(out_m, *b, Mat4::identity());
        } else {
            err = "join: input b is empty or not a mesh";
            return false;
        }
        out = SockValue::of_mesh(std::move(out_m));
        return true;
    }

    if (type == "union" || type == "difference" || type == "intersect") {
        const Mesh* a = mesh_at(0);
        const Mesh* b = mesh_at(1);
        if (!a || !b) {
            err = type + ": both inputs must be non-empty meshes";
            return false;
        }
        const int mode = type == "union" ? 0 : (type == "difference" ? 1 : 2);
        const int res = in[2].i;
        Mesh out_m;
        if (!boolean_mesh(*a, *b, out_m, mode, res)) {
            err = type + ": the boolean produced no surface at resolution " +
                  std::to_string(res) +
                  ". Disjoint operands give an empty intersection, and a resolution "
                  "that fine may not resolve the overlap.";
            return false;
        }
        out = SockValue::of_mesh(std::move(out_m));
        return true;
    }

    if (type == "remesh") {
        const Mesh* src = mesh_at(0);
        if (!src) {
            err = "remesh: the mesh input is empty or not a mesh";
            return false;
        }
        Mesh out_m;
        if (!remesh_mesh(*src, out_m, in[1].i)) {
            err = "remesh: could not extract a surface at resolution " +
                  std::to_string(in[1].i) +
                  ". The input must be a closed surface; an open one has no interior.";
            return false;
        }
        out = SockValue::of_mesh(std::move(out_m));
        return true;
    }

    if (type == "displace") {
        const Mesh* src = mesh_at(0);
        if (!src) {
            err = "displace: the mesh input is empty or not a mesh";
            return false;
        }
        if (!(in[2].f > 0.0f)) {
            err = "displace: frequency must be greater than 0";
            return false;
        }
        if (in[4].i < 1 || in[4].i > 8) {
            err = "displace: octaves must be 1..8, got " + std::to_string(in[4].i);
            return false;
        }
        Mesh out_m = *src;
        if (!displace_mesh(out_m, in[1].f, in[2].f, in[3].i, in[4].i, true)) {
            err = "displace: could not displace the mesh";
            return false;
        }
        out = SockValue::of_mesh(std::move(out_m));
        return true;
    }

    if (type == "bbox_size") {
        const Mesh* src = mesh_at(0);
        if (!src) {
            err = "bbox_size: the mesh input is empty or not a mesh";
            return false;
        }
        Vec3 lo = src->positions[0], hi = src->positions[0];
        for (const Vec3& p : src->positions) {
            lo = Vec3{std::min(lo.x, p.x), std::min(lo.y, p.y), std::min(lo.z, p.z)};
            hi = Vec3{std::max(hi.x, p.x), std::max(hi.y, p.y), std::max(hi.z, p.z)};
        }
        out = SockValue::of_vec3(hi - lo);
        return true;
    }

    if (type == "math") {
        const float a = in[0].f, b = in[1].f;
        const std::string& op = in[2].s;
        float r = 0.0f;
        if (op == "add") r = a + b;
        else if (op == "sub") r = a - b;
        else if (op == "mul") r = a * b;
        else if (op == "div") r = b != 0.0f ? a / b : 0.0f;
        else if (op == "min") r = std::min(a, b);
        else if (op == "max") r = std::max(a, b);
        else if (op == "pow") r = std::pow(a, b);
        else {
            err = "math: unknown op '" + op +
                  "'; use add, sub, mul, div, min, max or pow";
            return false;
        }
        out = SockValue::of_float(r);
        return true;
    }

    if (type == "group") {
        out = in.empty() ? SockValue{} : in[0];
        return true;
    }

    if (type == "output") {
        if (in.empty() || in[0].type != SockValue::Type::Mesh) {
            err = "output: the mesh input is empty or not a mesh. Link a node that "
                  "produces geometry into it, such as primitive, join, union, "
                  "difference, intersect, remesh or displace";
            return false;
        }
        out = in[0];
        return true;
    }

    err = "unknown node type '" + type + "'";
    return false;
}

/// Demand-driven evaluation with cycle detection.
struct Evaluator {
    const NodeGraph& g;
    const std::vector<NodeGraph>* all;
    std::vector<SockValue> cache;
    std::vector<uint8_t> state;  // 0 unvisited, 1 in progress, 2 done
    std::vector<int> stack;
    std::string error;
    std::string failed;
    int count = 0;

    Evaluator(const NodeGraph& gr, const std::vector<NodeGraph>* a)
        : g(gr), all(a), cache(gr.nodes.size()), state(gr.nodes.size(), 0) {}

    /// Evaluate an instanced graph, applying this node's parameter overrides to a
    /// copy so the original graph is untouched.
    bool eval_group(int idx) {
        const GraphNode& n = g.nodes[static_cast<size_t>(idx)];
        if (!all) {
            error = "group '" + n.name + "': no graph set to resolve against";
            failed = n.name;
            return false;
        }
        const NodeGraph* target = nullptr;
        for (const NodeGraph& cand : *all) {
            if (cand.name == n.target_graph) {
                target = &cand;
                break;
            }
        }
        if (!target) {
            error = "group '" + n.name + "': no graph named '" + n.target_graph + "'";
            failed = n.name;
            return false;
        }
        // A copy, so an instance never mutates the graph it instantiates.
        NodeGraph inst = *target;
        int applied = 0;
        for (std::size_t k = 0; k < n.inputs.size() && k < inst.params.size(); ++k) {
            const SockIn& sock = n.inputs[k];
            if (sock.linked()) continue;  // a wired parameter is driven by the link
            const GraphParam& prm = inst.params[k];
            const int ti = inst.find_node(prm.node);
            GraphNode* tn = inst.node_at(ti);
            if (!tn) continue;
            for (SockIn& ts : tn->inputs) {
                if (ts.name == prm.socket) {
                    ts.literal = sock.literal;
                    ts.literal_set = true;
                    ++applied;
                    break;
                }
            }
        }
        if (applied == 0) {
            error = "group '" + n.name + "' overrode nothing: none of its parameters "
                    "matched a socket in '" + inst.name + "'";
            failed = n.name;
            return false;
        }
        const GraphResult r = evaluate_graph_in(*all, inst);
        if (!r.ok) {
            error = "group '" + n.name + "' instantiating '" + inst.name + "': " + r.error;
            failed = n.name;
            return false;
        }
        cache[static_cast<size_t>(idx)] = SockValue::of_mesh(r.mesh);
        state[static_cast<size_t>(idx)] = 2;
        ++count;
        return true;
    }

    bool eval(int idx) {
        if (idx < 0 || idx >= static_cast<int>(g.nodes.size())) {
            error = "no node at index " + std::to_string(idx);
            return false;
        }
        if (state[static_cast<size_t>(idx)] == 2) return true;
        if (state[static_cast<size_t>(idx)] == 1) {
            // Walk back to the node that closed the loop so the message names it.
            error = "cycle: node '" + g.nodes[static_cast<size_t>(idx)].name + "' is already "
                    "being evaluated";
            for (int s : stack) {
                if (s == idx) {
                    error += " (via '";
                    for (int t : stack) error += g.nodes[static_cast<size_t>(t)].name + "' -> '";
                    error += g.nodes[static_cast<size_t>(idx)].name + "')";
                    break;
                }
            }
            failed = g.nodes[static_cast<size_t>(idx)].name;
            return false;
        }
        const GraphNode& n = g.nodes[static_cast<size_t>(idx)];
        if (n.type == "group") {
            return eval_group(idx);
        }
        state[static_cast<size_t>(idx)] = 1;
        stack.push_back(idx);

        // A group node's signature comes from the graph it instantiates rather than
        // from the static registry.
        std::vector<std::pair<std::string, SockValue::Type>> spec;
        const NodeDef* def = nullptr;
        if (n.type == "group") {
            spec = graph_group_inputs(*all, n.target_graph);
        } else {
            def = find_def(n.type);
            if (!def) {
                error = "unknown node type '" + n.type + "'";
                failed = n.name;
                return false;
            }
            for (const SocketSpec& sp : def->inputs) spec.emplace_back(sp.name, sp.type);
        }
        std::vector<SockValue> ins;
        ins.reserve(spec.size());
        for (size_t k = 0; k < spec.size(); ++k) {
            const SockIn* sock = nullptr;
            for (const SockIn& candidate : n.inputs) {
                if (candidate.name == def->inputs[k].name) {
                    sock = &candidate;
                    break;
                }
            }
            SockValue got;
            if (sock && sock->linked()) {
                if (!eval(sock->link_node)) {
                    stack.pop_back();
                    return false;
                }
                got = cache[static_cast<size_t>(sock->link_node)];
            } else if (sock) {
                got = sock->literal;
            } else if (n.type == "group") {
                // Unwired group parameters take the instanced graph's own default,
                // which is what makes a group usable without setting every knob.
                got = SockValue{};
                const NodeGraph* tgt = nullptr;
                if (all) {
                    for (const NodeGraph& cand : *all) {
                        if (cand.name == n.target_graph) tgt = &cand;
                    }
                }
                if (tgt && k < tgt->params.size()) got = tgt->params[k].def;
            } else {
                got = def->inputs[k].def;
            }
            SockValue coerced;
            std::string cerr;
            if (spec[k].second == SockValue::Type::Mesh && got.type != SockValue::Type::Mesh) {
                // Left at its default, a mesh socket is legitimately empty and the
                // node reports that. Set to something else, it is a type error, and
                // saying "the output node did not produce a mesh" instead would hide
                // the actual mistake.
                if (sock && sock->literal_set) {
                    error = std::string("input '") + spec[k].first + "': cannot use a " +
                            type_name(got.type) + " where a mesh is expected";
                    failed = n.name;
                    stack.pop_back();
                    return false;
                }
                coerced = got;
            } else if (!coerce(got, spec[k].second, coerced, cerr)) {
                error = std::string("input '") + spec[k].first + "': " + cerr;
                failed = n.name;
                stack.pop_back();
                return false;
            }
            ins.push_back(std::move(coerced));
        }
        SockValue out;
        std::string err;
        if (!run_node(n.type, ins, out, err)) {
            error = err;
            failed = n.name;
            stack.pop_back();
            return false;
        }
        cache[static_cast<size_t>(idx)] = std::move(out);
        state[static_cast<size_t>(idx)] = 2;
        stack.pop_back();
        ++count;
        return true;
    }
};

}  // namespace

/// Serialize graphs by node and socket name rather than by index.
///
/// Names rather than indices, so a graph survives a rebuild that renumbers
/// nodes; indices would silently rewire on the next edit. Shared by the undo
/// snapshot, the json sidecar and the glTF export, so there is one format and one
/// implementation of it.
nlohmann::json graph_to_json(const NodeGraph& g) {
    nlohmann::json jg = nlohmann::json::object();
    jg["name"] = g.name;
    jg["output_node"] = g.output_node >= 0 &&
                                g.output_node < static_cast<int>(g.nodes.size())
                            ? g.nodes[static_cast<size_t>(g.output_node)].name
                            : std::string();
    jg["nodes"] = nlohmann::json::array();
    for (const GraphNode& n : g.nodes) {
        nlohmann::json jn = nlohmann::json::object();
        jn["name"] = n.name;
        jn["type"] = n.type;
        if (!n.target_graph.empty()) jn["target_graph"] = n.target_graph;
        jn["x"] = n.x;
        jn["y"] = n.y;
        jn["sockets"] = nlohmann::json::array();
        for (const SockIn& sk : n.inputs) {
            nlohmann::json js = nlohmann::json::object();
            js["name"] = sk.name;
            if (sk.linked() && sk.link_node >= 0 &&
                sk.link_node < static_cast<int>(g.nodes.size())) {
                js["from"] = g.nodes[static_cast<size_t>(sk.link_node)].name;
                js["from_socket"] = sk.link_sock;
            } else {
                js["set"] = sk.literal_set;
                if (sk.literal.type == SockValue::Type::Vec3) {
                    js["value"] = nlohmann::json::array(
                        {sk.literal.v.x, sk.literal.v.y, sk.literal.v.z});
                } else if (sk.literal.type == SockValue::Type::String) {
                    js["value"] = sk.literal.s;
                } else if (sk.literal.type == SockValue::Type::Int) {
                    js["value"] = sk.literal.i;
                } else {
                    js["value"] = sk.literal.f;
                }
            }
            jn["sockets"].push_back(std::move(js));
        }
        jg["nodes"].push_back(std::move(jn));
    }
    return jg;
}


/// Read graphs back, resolving links in a second pass because a node may be
/// linked to one that has not been created yet.
void graphs_from_json(const nlohmann::json& arr, std::vector<NodeGraph>& out) {
    out.clear();
    if (!arr.is_array()) return;
    for (const auto& jg : arr) {
        NodeGraph g;
        g.name = jg.value("name", "Graph");
        const std::string out_name = jg.value("output_node", std::string());
        if (jg.contains("params") && jg["params"].is_array()) {
            for (const auto& jp : jg["params"]) {
                GraphParam prm;
                prm.name = jp.value("name", "");
                prm.type = static_cast<SockValue::Type>(jp.value("type", 0));
                prm.node = jp.value("node", "");
                prm.socket = jp.value("socket", "");
                if (jp.contains("default")) {
                    const auto& jd = jp["default"];
                    if (jd.is_array() && jd.size() >= 3) {
                        prm.def = SockValue::of_vec3(
                            {jd[0].get<float>(), jd[1].get<float>(), jd[2].get<float>()});
                    } else if (jd.is_string()) {
                        prm.def = SockValue::of_string(jd.get<std::string>());
                    } else if (jd.is_number_integer()) {
                        prm.def = SockValue::of_int(jd.get<int>());
                    } else {
                        prm.def = SockValue::of_float(jd.get<float>());
                    }
                }
                if (!prm.name.empty() && !prm.node.empty()) g.params.push_back(std::move(prm));
            }
        }
        if (jg.contains("nodes") && jg["nodes"].is_array()) {
            for (const auto& jn : jg["nodes"]) {
                GraphNode n;
                n.name = jn.value("name", "node");
                n.type = jn.value("type", "");
                n.target_graph = jn.value("target_graph", "");
                n.x = jn.value("x", 0);
                n.y = jn.value("y", 0);
                // Sockets are seeded from the registry so a socket added by a
                // later version still exists, then the stored values are applied.
                for (const auto& sp : graph_node_inputs(n.type)) {
                    SockIn sk;
                    sk.name = sp.first;
                    sk.literal = graph_socket_default(n.type, sp.first);
                    n.inputs.push_back(sk);
                }
                if (jn.contains("sockets") && jn["sockets"].is_array()) {
                    for (const auto& js : jn["sockets"]) {
                        const std::string sn = js.value("name", "");
                        for (SockIn& sk : n.inputs) {
                            if (sk.name != sn) continue;
                            if (js.contains("from")) {
                                // Link target resolved by name in the second pass.
                            } else {
                                sk.literal_set = js.value("set", false);
                                if (js.contains("value")) {
                                    const auto& jv = js["value"];
                                    if (jv.is_array() && jv.size() >= 3) {
                                        sk.literal = SockValue::of_vec3(
                                            {jv[0].get<float>(), jv[1].get<float>(),
                                             jv[2].get<float>()});
                                    } else if (jv.is_string()) {
                                        sk.literal =
                                            SockValue::of_string(jv.get<std::string>());
                                    } else if (jv.is_number_integer()) {
                                        sk.literal = SockValue::of_int(jv.get<int>());
                                    } else {
                                        sk.literal = SockValue::of_float(jv.get<float>());
                                    }
                                }
                            }
                            break;
                        }
                    }
                }
                g.nodes.push_back(std::move(n));
            }
        }
        if (!out_name.empty()) g.output_node = g.find_node(out_name);
        out.push_back(std::move(g));
    }
    // Second pass for links, now that every node name is known.
    size_t gi = 0;
    for (const auto& jg : arr) {
        if (gi >= out.size()) break;
        NodeGraph& g = out[gi++];
        if (!(jg.contains("nodes") && jg["nodes"].is_array())) continue;
        for (const auto& jn : jg["nodes"]) {
            const int to_i = g.find_node(jn.value("name", ""));
            if (to_i < 0) continue;
            GraphNode* to = g.node_at(to_i);
            if (!(jn.contains("sockets") && jn["sockets"].is_array())) continue;
            for (const auto& js : jn["sockets"]) {
                if (!js.contains("from")) continue;
                const int from_i = g.find_node(js.value("from", ""));
                const std::string sn = js.value("name", "");
                if (from_i < 0) continue;
                for (SockIn& sk : to->inputs) {
                    if (sk.name == sn) {
                        sk.link_node = from_i;
                        sk.link_sock = js.value("from_socket", 0);
                        break;
                    }
                }
            }
        }
    }
}

nlohmann::json graphs_to_json(const std::vector<NodeGraph>& graphs) {
    nlohmann::json arr = nlohmann::json::array();
    for (const NodeGraph& g : graphs) arr.push_back(graph_to_json(g));
    return arr;
}

std::vector<std::string> graph_node_types() {
    std::vector<std::string> out;
    for (const NodeDef& d : registry()) out.push_back(d.type);
    return out;
}

SockValue graph_socket_default(const std::string& type, const std::string& input) {
    const NodeDef* d = find_def(type);
    if (!d) return SockValue{};
    for (const SocketSpec& s : d->inputs) {
        if (input == s.name) return s.def;
    }
    return SockValue{};
}

std::vector<std::pair<std::string, SockValue::Type>> graph_node_inputs(
    const std::string& type) {
    std::vector<std::pair<std::string, SockValue::Type>> out;
    const NodeDef* d = find_def(type);
    if (!d) return out;
    for (const SocketSpec& s : d->inputs) out.emplace_back(s.name, s.type);
    return out;
}

std::vector<std::pair<std::string, SockValue::Type>> graph_group_inputs(
    const std::vector<NodeGraph>& all, const std::string& target) {
    std::vector<std::pair<std::string, SockValue::Type>> out;
    for (const NodeGraph& g : all) {
        if (g.name != target) continue;
        for (const GraphParam& p : g.params) out.emplace_back(p.name, p.type);
        return out;
    }
    return out;
}

GraphResult evaluate_graph_in(const std::vector<NodeGraph>& all, const NodeGraph& g,
                              int forced_output) {
    GraphResult r;
    if (g.nodes.empty()) {
        r.error = "the graph has no nodes";
        return r;
    }
    int out_idx = forced_output;
    if (out_idx < 0) out_idx = g.output_node;
    if (out_idx < 0) {
        r.error =
            "the graph has no output node. Add one with "
            "graph_node type=output and link a mesh into it.";
        return r;
    }
    const GraphNode* outn = g.node_at(out_idx);
    if (!outn) {
        r.error = "output node index " + std::to_string(out_idx) + " is out of range";
        return r;
    }
    if (outn->type != "output") {
        r.error = "node '" + outn->name + "' is type '" + outn->type +
                  "', not an output node";
        r.failed_node = outn->name;
        return r;
    }
    // Cross-graph cycle check up front. Doing it here rather than with a stack in
    // the evaluator means the message can name the whole path, and it costs one
    // walk of the instantiation graph instead of a guard on every node visit.
    {
        std::vector<std::string> chain{g.name};
        std::function<bool(const std::string&)> walk = [&](const std::string& name) {
            for (const NodeGraph& cand : all) {
                if (cand.name != name) continue;
                for (const GraphNode& n : cand.nodes) {
                    if (n.type != "group") continue;
                    for (const std::string& open : chain) {
                        if (open == n.target_graph) {
                            r.error = "cycle: '" + n.name + "' in '" + name + "' instantiates '" +
                                      n.target_graph + "', which is already being evaluated";
                            r.failed_node = n.name;
                            return true;
                        }
                    }
                    chain.push_back(n.target_graph);
                    if (walk(n.target_graph)) return true;
                    chain.pop_back();
                }
            }
            return false;
        };
        if (walk(g.name)) return r;
    }

    Evaluator ev(g, &all);
    if (!ev.eval(out_idx)) {
        r.error = ev.error;
        r.failed_node = ev.failed;
        return r;
    }
    const SockValue& v = ev.cache[static_cast<size_t>(out_idx)];
    if (v.type != SockValue::Type::Mesh || v.mesh.faces.empty()) {
        r.error = "the output node did not produce a mesh. It must be linked to a node "
                  "that produces geometry, such as primitive, join, union, difference, "
                  "intersect, remesh or displace.";
        r.failed_node = outn->name;
        return r;
    }
    r.ok = true;
    r.nodes_evaluated = ev.count;
    r.mesh = v.mesh;
    r.stats = mesh_stats(r.mesh);
    return r;
}

GraphResult evaluate_graph(const NodeGraph& g, int forced_output) {
    // Single-graph convenience: the graph itself is the only catalog entry
    // that `group` nodes can resolve against.
    return evaluate_graph_in({g}, g, forced_output);
}

}  // namespace kiln
