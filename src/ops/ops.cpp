#include "ops.hpp"

#include "../io/assimp_io.hpp"
#include "../io/gltf.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <map>
#include <set>
#include <sstream>

namespace kiln {
namespace {

std::string require_str(const nlohmann::json& j, const char* k, const std::string& d = {}) {
    if (j.contains(k) && j[k].is_string()) return j[k].get<std::string>();
    return d;
}

Vec3 require_vec3(const nlohmann::json& j, const char* k, Vec3 d = {}) {
    if (!j.contains(k)) return d;
    const auto& v = j[k];
    if (v.is_array() && v.size() >= 3) return {v[0].get<float>(), v[1].get<float>(), v[2].get<float>()};
    return d;
}

bool finite(float x) { return x == x && x > -3.0e38f && x < 3.0e38f; }

float require_f(const nlohmann::json& j, const char* k, float d = 0) {
    if (!j.contains(k)) return d;
    if (j[k].is_number()) return j[k].get<float>();
    return d;
}

int resolve_node(Scene& s, const nlohmann::json& op) {
    if (op.contains("id") && op["id"].is_number_integer()) return s.find_node_id(op["id"].get<uint32_t>());
    if (op.contains("target") && op["target"].is_string()) return s.find_node(op["target"].get<std::string>());
    if (op.contains("name") && op["name"].is_string() && op.value("op", "") != "create")
        return s.find_node(op["name"].get<std::string>());
    return s.selection;
}

int add_material(Scene& s, Vec3 albedo) {
    Material m;
    m.name = "Material";
    m.albedo = albedo;
    s.materials.push_back(m);
    return static_cast<int>(s.materials.size()) - 1;
}

std::string lower_copy(std::string s) {
    for (char& c : s) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    return s;
}

OpResult fail(const std::string& m) { return {false, m}; }
OpResult ok(const std::string& m) { return {true, m}; }

bool parse_axis(const std::string& s, Vec3& out) {
    std::string k = s;
    for (char& c : k) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    if (k == "x" || k == "+x") out = {1, 0, 0};
    else if (k == "y" || k == "+y" || k == "vertical") out = {0, 1, 0};
    else if (k == "z" || k == "+z") out = {0, 0, 1};
    else if (k == "horizontal") out = {1, 0, 0};
    else return false;
    return true;
}

Edge edge_from_json(const nlohmann::json& j) {
    if (j.is_array() && j.size() >= 2 && j[0].is_number() && j[1].is_number())
        return make_edge(j[0].get<uint32_t>(), j[1].get<uint32_t>());
    return {};
}

std::vector<uint32_t> current_verts(const Scene& s, const Mesh& mesh) {
    // A selection can outlive an edit that shrank the mesh. Never hand a stale
    // index to a vertex op.
    auto live = [&](std::vector<uint32_t> v) {
        v.erase(std::remove_if(v.begin(), v.end(), [&](uint32_t i) { return i >= mesh.positions.size(); }), v.end());
        return v;
    };
    if (s.edit_mode == EditMode::Vert && !s.vert_sel.empty()) return live(s.vert_sel);
    if (!s.face_sel.empty()) return live(verts_of_faces(mesh, s.face_sel));
    if (!s.edge_sel.empty()) {
        std::set<uint32_t> u;
        for (Edge e : s.edge_sel) {
            u.insert(e.a);
            u.insert(e.b);
        }
        return live({u.begin(), u.end()});
    }
    std::vector<uint32_t> all(mesh.positions.size());
    for (size_t i = 0; i < all.size(); ++i) all[i] = uint32_t(i);
    return all;
}

}  // namespace

void Document::push_undo() {
    undo.push_back(scene_to_json(scene));
    if (static_cast<int>(undo.size()) > max_undo) undo.erase(undo.begin());
    redo.clear();
}

bool Document::undo_once() {
    if (undo.empty()) return false;
    redo.push_back(scene_to_json(scene));
    scene_from_json(undo.back(), scene);
    undo.pop_back();
    return true;
}

bool Document::redo_once() {
    if (redo.empty()) return false;
    undo.push_back(scene_to_json(scene));
    scene_from_json(redo.back(), scene);
    redo.pop_back();
    return true;
}

void Document::reset() {
    scene = Scene{};
    undo.clear();
    redo.clear();
    journal.clear();
}

void Document::record(const nlohmann::json& op, const OpResult& r) {
    nlohmann::json entry = nlohmann::json::object();
    entry["op"] = op;
    entry["ok"] = r.ok;
    entry["message"] = r.message;
    if (!r.data.is_null() && !(r.data.is_object() && r.data.empty())) entry["data"] = r.data;
    journal.push_back(std::move(entry));
    if (static_cast<int>(journal.size()) > max_journal) {
        journal.erase(journal.begin(),
                     journal.begin() + (static_cast<int>(journal.size()) - max_journal));
    }
}

/// Single funnel for every applied op, so early returns are journaled too.
OpResult apply_op(Document& doc, const nlohmann::json& op) {
    OpResult r = apply_op_impl(doc, op);
    doc.record(op, r);
    return r;
}

int ensure_default_camera(Scene& s) {
    if (s.active_camera >= 0 && s.active_camera < static_cast<int>(s.nodes.size())) return s.active_camera;
    for (int i = 0; i < static_cast<int>(s.nodes.size()); ++i) {
        if (s.nodes[i].type == NodeType::Camera) {
            s.active_camera = i;
            return i;
        }
    }
    Node cam;
    cam.type = NodeType::Camera;
    cam.name = "Camera";
    cam.translation = {4.2f, 2.8f, 5.4f};
    cam.rotation_deg = {-22.0f, 38.0f, 0};
    cam.fov_deg = 50;
    s.active_camera = s.add_node(cam);
    return s.active_camera;
}

int ensure_default_sun(Scene& s) {
    for (int i = 0; i < static_cast<int>(s.nodes.size()); ++i) {
        if (s.nodes[i].type == NodeType::Light) return i;
    }
    Node L;
    L.type = NodeType::Light;
    L.name = "Sun";
    L.light = LightType::Sun;
    L.rotation_deg = {-50, 35, 0};
    L.light_intensity = 1.15f;
    return s.add_node(L);
}

OpResult apply_op_impl(Document& doc, const nlohmann::json& op) {
    if (!op.is_object() || !op.contains("op")) return fail("op object requires 'op'");
    std::string kind = op["op"].get<std::string>();
    Scene& s = doc.scene;

    if (kind == "new") {
        doc.reset();
        return ok("new scene");
    }
    if (kind == "undo") return doc.undo_once() ? ok("undo") : fail("nothing to undo");
    if (kind == "redo") return doc.redo_once() ? ok("redo") : fail("nothing to redo");

    bool mutates = kind != "select" && kind != "select_faces" && kind != "select_edges" &&
                   kind != "select_edge_loop" && kind != "select_edge_ring" && kind != "select_verts" &&
                   kind != "grow" && kind != "invert" && kind != "linked" && kind != "select_more" &&
                   kind != "mode" && kind != "list" && kind != "info" && kind != "describe";
    if (mutates) doc.push_undo();

    if (kind == "create") {
        std::string prims = require_str(op, "primitive", "cube");
        PrimType prim;
        if (!parse_prim(prims, prim)) {
            if (prims == "camera") {
                Node n;
                n.type = NodeType::Camera;
                n.name = require_str(op, "name", "Camera");
                n.translation = require_vec3(op, "translation", {4, 3, 5});
                n.rotation_deg = require_vec3(op, "rotation", {-20, 35, 0});
                n.fov_deg = require_f(op, "fov", 50);
                int i = s.add_node(n);
                s.active_camera = i;
                s.selection = i;
                return ok("created camera " + s.nodes[i].name);
            }
            if (prims == "light" || prims == "sun" || prims == "point") {
                Node n;
                n.type = NodeType::Light;
                n.name = require_str(op, "name", prims == "point" ? "Point" : "Sun");
                n.light = (prims == "point") ? LightType::Point : LightType::Sun;
                n.translation = require_vec3(op, "translation", {0, 3, 0});
                n.rotation_deg = require_vec3(op, "rotation", {-50, 30, 0});
                n.light_color = require_vec3(op, "albedo", n.light_color);
                n.light_intensity = require_f(op, "intensity", 1);
                int i = s.add_node(n);
                s.selection = i;
                return ok("created light " + s.nodes[i].name);
            }
            if (prims == "empty") {
                Node n;
                n.type = NodeType::Empty;
                n.name = require_str(op, "name", "Empty");
                n.translation = require_vec3(op, "translation");
                int i = s.add_node(n);
                s.selection = i;
                return ok("created empty " + s.nodes[i].name);
            }
            if (prims == "bone") {
                // Bones previously only arrived by importing a rigged glTF or
                // FBX, which left `parent`, `pose`, `ik` and `skin` unusable on a
                // built scene and untestable without a fixture file.
                Node n;
                n.type = NodeType::Bone;
                n.name = require_str(op, "name", "Bone");
                n.translation = require_vec3(op, "translation");
                n.rotation_deg = require_vec3(op, "rotation");
                n.scale = require_vec3(op, "scale", {1, 1, 1});
                n.bone_length = require_f(op, "bone_length", 0.2f);
                int i = s.add_node(n);
                if (op.contains("parent")) {
                    const std::string pn = require_str(op, "parent", "");
                    const int pi = s.find_node(pn);
                    if (pi < 0 || static_cast<size_t>(pi) >= s.nodes.size() ||
                        s.nodes[static_cast<size_t>(pi)].type != NodeType::Bone) {
                        return fail("create bone: parent '" + pn + "' is not a bone node");
                    }
                    if (pi == i) return fail("create bone: a bone cannot be its own parent");
                    s.nodes[i].parent = pi;
                }
                s.selection = i;
                return ok("created bone " + s.nodes[i].name);
            }
            if (mutates) doc.undo_once();
            return fail("unknown primitive '" + prims + "'");
        }
        Vec3 size = require_vec3(op, "size", {1, 1, 1});
        if (prim == PrimType::Plane && size.z == 1 && size.x != 1) size.z = size.x;
        Mesh mesh = make_primitive(prim, size, static_cast<int>(require_f(op, "segments", 24)));
        // Curved primitives are smooth-shaded, flat ones are not. This is not
        // only cosmetic: mesh_sync_render welds on (position, normal, uv), so a
        // flat-shaded sphere cannot share a single vertex and exports as a
        // topologically open surface -- 18% of its edges shared when measured.
        // A sphere is smooth, and it should also be one connected mesh.
        const bool curved = prim == PrimType::Sphere || prim == PrimType::Cylinder ||
                            prim == PrimType::Cone || prim == PrimType::Torus;
        mesh.smooth = curved;
        s.meshes.push_back(std::move(mesh));
        int mi = static_cast<int>(s.meshes.size()) - 1;
        Vec3 albedo = require_vec3(op, "albedo", {0.72f, 0.73f, 0.76f});
        int mati = add_material(s, albedo);
        if (op.contains("roughness")) s.materials[mati].roughness = require_f(op, "roughness", 0.45f);
        if (op.contains("metallic")) s.materials[mati].metallic = require_f(op, "metallic", 0);
        Node n;
        n.type = NodeType::Mesh;
        n.name = require_str(op, "name", prim_name(prim));
        if (!n.name.empty()) n.name[0] = static_cast<char>(std::toupper(n.name[0]));
        n.mesh = mi;
        n.material = mati;
        n.translation = require_vec3(op, "translation");
        n.rotation_deg = require_vec3(op, "rotation");
        n.scale = require_vec3(op, "scale", {1, 1, 1});
        int i = s.add_node(n);
        s.selection = i;
        s.edit_mode = EditMode::Object;
        s.face_sel.clear();
        s.edge_sel.clear();
        return ok("created " + s.nodes[i].name);
    }

    if (kind == "delete") {
        int i = resolve_node(s, op);
        if (i < 0) return fail("delete: no target");
        std::string name = s.nodes[i].name;
        s.remove_node_at(i);
        return ok("deleted " + name);
    }

    if (kind == "duplicate") {
        int i = resolve_node(s, op);
        if (i < 0) return fail("duplicate: no target");
        Node n = s.nodes[i];
        n.id = 0;
        n.name = require_str(op, "name", n.name);
        n.translation.x += 1.0f;
        int ni = s.add_node(n);
        s.selection = ni;
        return ok("duplicated " + s.nodes[ni].name);
    }

    if (kind == "rename") {
        int i = resolve_node(s, op);
        if (i < 0) return fail("rename: no target");
        std::string nn = require_str(op, "to", require_str(op, "new_name"));
        if (nn.empty()) return fail("rename: missing to");
        s.nodes[i].name = s.unique_name(nn);
        return ok("renamed " + s.nodes[i].name);
    }

    if (kind == "select") {
        if (op.contains("target") && (op["target"] == "none" || op["target"] == "")) {
            s.selection = -1;
            return ok("cleared selection");
        }
        int i = resolve_node(s, op);
        if (i < 0) return fail("select: not found");
        s.selection = i;
        s.edit_mode = EditMode::Object;
        s.face_sel.clear();
        s.edge_sel.clear();
        s.vert_sel.clear();
        return ok("selected " + s.nodes[i].name);
    }

    if (kind == "mode") {
        std::string md = require_str(op, "mode", "object");
        if (md == "face" || md == "faces") s.edit_mode = EditMode::Face;
        else if (md == "edge" || md == "edges") s.edit_mode = EditMode::Edge;
        else if (md == "vert" || md == "vertex" || md == "verts" || md == "vertices")
            s.edit_mode = EditMode::Vert;
        else {
            s.edit_mode = EditMode::Object;
            s.face_sel.clear();
            s.edge_sel.clear();
            s.vert_sel.clear();
        }
        return ok(std::string("mode ") + edit_mode_name(s.edit_mode));
    }

    if (kind == "select_faces") {
        int i = resolve_node(s, op);
        if (i < 0) i = s.selection;
        if (i < 0) return fail("select_faces: no mesh");
        s.selection = i;
        Node& n = s.nodes[i];
        if (n.mesh < 0) return fail("select_faces: not a mesh");
        Mesh& mesh = s.meshes[n.mesh];
        mesh_ensure_faces(mesh);
        std::vector<int> faces;
        if (op.contains("region") && op["region"].is_string()) {
            faces = faces_matching(mesh, op["region"].get<std::string>());
        } else if (op.contains("faces") && op["faces"].is_array()) {
            for (const auto& x : op["faces"]) {
                if (x.is_number_integer()) faces.push_back(x.get<int>());
            }
        }
        s.edit_mode = EditMode::Face;
        s.face_sel = std::move(faces);
        s.edge_sel.clear();
        s.vert_sel.clear();
        return ok("selected " + std::to_string(s.face_sel.size()) + " faces on " + n.name);
    }

    if (kind == "select_edges") {
        Mesh* mesh = nullptr;
        int i = resolve_node(s, op);
        if (i < 0) i = s.selection;
        if (i >= 0) s.selection = i;
        mesh = s.selected_mesh();
        if (!mesh) return fail("select_edges: no mesh");
        mesh_ensure_faces(*mesh);
        std::vector<Edge> edges;
        if (op.contains("region") && op["region"].is_string()) {
            auto faces = faces_matching(*mesh, op["region"].get<std::string>());
            edges = edges_of_faces(*mesh, faces);
        } else if (op.contains("axis") && op["axis"].is_string()) {
            Vec3 ax;
            if (parse_axis(op["axis"].get<std::string>(), ax)) {
                Edge e = pick_edge_axis(*mesh, ax);
                edges = edge_loop(*mesh, e);
            }
        } else if (op.contains("edges") && op["edges"].is_array()) {
            for (const auto& x : op["edges"]) {
                Edge e = edge_from_json(x);
                if (e.a != e.b) edges.push_back(e);
            }
        }
        s.edit_mode = EditMode::Edge;
        s.edge_sel = std::move(edges);
        s.face_sel.clear();
        s.vert_sel.clear();
        return ok("selected " + std::to_string(s.edge_sel.size()) + " edges");
    }

    if (kind == "select_edge_loop" || kind == "select_edge_ring") {
        Mesh* mesh = s.selected_mesh();
        if (!mesh) return fail("edge loop: no mesh");
        mesh_ensure_faces(*mesh);
        Edge seed{};
        if (op.contains("edge")) seed = edge_from_json(op["edge"]);
        else if (!s.edge_sel.empty()) seed = s.edge_sel[0];
        else if (op.contains("region") && op["region"].is_string()) {
            auto faces = faces_matching(*mesh, op["region"].get<std::string>());
            auto es = edges_of_faces(*mesh, faces);
            if (!es.empty()) seed = es[0];
        } else if (op.contains("axis") && op["axis"].is_string()) {
            Vec3 ax;
            if (parse_axis(op["axis"].get<std::string>(), ax)) seed = pick_edge_axis(*mesh, ax);
        }
        if (seed.a == seed.b && seed.a == 0) {
            if (!mesh->faces.empty() && mesh->faces[0].v.size() >= 2)
                seed = make_edge(mesh->faces[0].v[0], mesh->faces[0].v[1]);
        }
        auto got = (kind == "select_edge_ring") ? edge_ring(*mesh, seed) : edge_loop(*mesh, seed);
        s.edit_mode = EditMode::Edge;
        s.edge_sel = std::move(got);
        s.face_sel.clear();
        s.vert_sel.clear();
        return ok(std::string(kind == "select_edge_ring" ? "ring " : "loop ") +
                  std::to_string(s.edge_sel.size()) + " edges");
    }

    if (kind == "select_verts" || kind == "select_vertices") {
        Mesh* mesh = s.selected_mesh();
        int i = resolve_node(s, op);
        if (i < 0) i = s.selection;
        if (i >= 0) s.selection = i;
        mesh = s.selected_mesh();
        if (!mesh) return fail("select_verts: no mesh");
        mesh_ensure_faces(*mesh);
        std::vector<uint32_t> verts;
        if (op.contains("region") && op["region"].is_string()) {
            verts = verts_matching(*mesh, op["region"].get<std::string>());
        } else if (op.contains("verts") && op["verts"].is_array()) {
            for (const auto& x : op["verts"]) {
                if (x.is_number()) {
                    if (x.is_number_float()) verts.push_back(uint32_t(x.get<float>()));
                    else verts.push_back(x.get<uint32_t>());
                }
            }
        } else if (!s.face_sel.empty()) {
            verts = verts_of_faces(*mesh, s.face_sel);
        } else if (!s.edge_sel.empty()) {
            std::set<uint32_t> u;
            for (Edge e : s.edge_sel) {
                u.insert(e.a);
                u.insert(e.b);
            }
            verts.assign(u.begin(), u.end());
        }
        s.edit_mode = EditMode::Vert;
        s.vert_sel = std::move(verts);
        s.face_sel.clear();
        s.edge_sel.clear();
        return ok("selected " + std::to_string(s.vert_sel.size()) + " verts");
    }

    if (kind == "extrude") {
        Mesh* mesh = s.selected_mesh();
        if (!mesh) return fail("extrude: no mesh");
        mesh_ensure_faces(*mesh);
        std::vector<int> faces = s.face_sel;
        if (faces.empty() && op.contains("region") && op["region"].is_string())
            faces = faces_matching(*mesh, op["region"].get<std::string>());
        if (faces.empty()) return fail("extrude: no faces selected");
        float amt = require_f(op, "amount", 0.35f);
        std::vector<int> neu;
        if (!extrude_faces(*mesh, faces, amt, neu)) {
            if (mutates) doc.undo_once();
            return fail("extrude failed");
        }
        s.edit_mode = EditMode::Face;
        s.face_sel = neu;
        return ok("extruded " + std::to_string(neu.size()) + " faces by " + std::to_string(amt));
    }

    if (kind == "inset") {
        Mesh* mesh = s.selected_mesh();
        if (!mesh) return fail("inset: no mesh");
        mesh_ensure_faces(*mesh);
        std::vector<int> faces = s.face_sel;
        if (faces.empty() && op.contains("region") && op["region"].is_string())
            faces = faces_matching(*mesh, op["region"].get<std::string>());
        if (faces.empty()) return fail("inset: no faces selected");
        float amt = require_f(op, "amount", 0.15f);
        std::vector<int> neu;
        if (!inset_faces(*mesh, faces, amt, neu)) {
            if (mutates) doc.undo_once();
            return fail("inset failed");
        }
        s.face_sel = neu;
        s.edit_mode = EditMode::Face;
        return ok("inset " + std::to_string(neu.size()) + " faces");
    }

    if (kind == "delete_faces") {
        Mesh* mesh = s.selected_mesh();
        if (!mesh) return fail("delete_faces: no mesh");
        mesh_ensure_faces(*mesh);
        std::vector<int> faces = s.face_sel;
        if (faces.empty() && op.contains("region") && op["region"].is_string())
            faces = faces_matching(*mesh, op["region"].get<std::string>());
        if (faces.empty()) return fail("delete_faces: no faces");
        if (!delete_faces_mesh(*mesh, faces)) {
            if (mutates) doc.undo_once();
            return fail("delete_faces failed");
        }
        s.face_sel.clear();
        return ok("deleted faces");
    }

    if (kind == "loop_cut" || kind == "loopcut") {
        Mesh* mesh = s.selected_mesh();
        if (!mesh) return fail("loop_cut: no mesh");
        mesh_ensure_faces(*mesh);
        Edge seed{};
        if (op.contains("edge")) seed = edge_from_json(op["edge"]);
        else if (!s.edge_sel.empty()) seed = s.edge_sel[0];
        else if (op.contains("axis") && op["axis"].is_string()) {
            Vec3 ax;
            if (parse_axis(op["axis"].get<std::string>(), ax)) seed = pick_edge_axis(*mesh, ax);
        } else if (op.contains("region") && op["region"].is_string()) {
            auto faces = faces_matching(*mesh, op["region"].get<std::string>());
            auto es = edges_of_faces(*mesh, faces);
            if (!es.empty()) seed = es[0];
        }
        if (seed.a == seed.b) seed = pick_edge_axis(*mesh, {0, 1, 0});
        int cuts = 1;
        if (op.contains("cuts") && op["cuts"].is_number()) cuts = std::max(1, int(op["cuts"].get<float>()));
        if (op.contains("count") && op["count"].is_number()) cuts = std::max(1, int(op["count"].get<float>()));
        std::vector<Edge> neu;
        if (!loop_cut(*mesh, seed, cuts, neu)) {
            if (mutates) doc.undo_once();
            return fail("loop_cut failed");
        }
        s.edit_mode = EditMode::Edge;
        s.edge_sel = std::move(neu);
        s.face_sel.clear();
        return ok("loop cut x" + std::to_string(cuts));
    }

    if (kind == "bevel") {
        Mesh* mesh = s.selected_mesh();
        if (!mesh) return fail("bevel: no mesh");
        mesh_ensure_faces(*mesh);
        std::vector<Edge> edges = s.edge_sel;
        if (edges.empty() && !s.face_sel.empty()) edges = edges_of_faces(*mesh, s.face_sel);
        if (edges.empty() && op.contains("region") && op["region"].is_string()) {
            auto faces = faces_matching(*mesh, op["region"].get<std::string>());
            edges = edges_of_faces(*mesh, faces);
        }
        if (edges.empty()) return fail("bevel: no edges");
        float amt = require_f(op, "amount", 0.12f);
        std::vector<Edge> neu;
        if (!bevel_edges(*mesh, edges, amt, neu)) {
            if (mutates) doc.undo_once();
            return fail("bevel failed");
        }
        s.edit_mode = EditMode::Edge;
        s.edge_sel = std::move(neu);
        s.face_sel.clear();
        return ok("bevelled " + std::to_string(edges.size()) + " edges");
    }

    if (kind == "subdivide" || kind == "subdiv") {
        Mesh* mesh = s.selected_mesh();
        if (!mesh) return fail("subdivide: no mesh");
        int cuts = 1;
        if (op.contains("cuts") && op["cuts"].is_number()) cuts = std::max(1, int(op["cuts"].get<float>()));
        if (op.contains("count") && op["count"].is_number()) cuts = std::max(1, int(op["count"].get<float>()));
        std::vector<int> faces = s.face_sel;
        if (faces.empty() && op.contains("region") && op["region"].is_string())
            faces = faces_matching(*mesh, op["region"].get<std::string>());
        std::vector<int> neu;
        if (!subdivide_faces(*mesh, faces, cuts, neu)) {
            if (mutates) doc.undo_once();
            return fail("subdivide failed");
        }
        s.edit_mode = EditMode::Face;
        s.face_sel = neu;
        s.edge_sel.clear();
        s.vert_sel.clear();
        return ok("subdivided to " + std::to_string(mesh->faces.size()) + " faces");
    }

    if (kind == "merge" || kind == "weld") {
        Mesh* mesh = s.selected_mesh();
        if (!mesh) return fail("merge: no mesh");
        std::vector<uint32_t> verts = s.vert_sel;
        if (verts.empty() && !s.face_sel.empty()) verts = verts_of_faces(*mesh, s.face_sel);
        if (verts.empty() && !s.edge_sel.empty()) {
            std::set<uint32_t> u;
            for (Edge e : s.edge_sel) {
                u.insert(e.a);
                u.insert(e.b);
            }
            verts.assign(u.begin(), u.end());
        }
        if (verts.empty() && op.contains("region") && op["region"].is_string())
            verts = verts_matching(*mesh, op["region"].get<std::string>());
        uint32_t survivor = 0;
        if (!merge_verts(*mesh, verts, survivor)) {
            if (mutates) doc.undo_once();
            return fail("merge: need 2+ verts");
        }
        s.edit_mode = EditMode::Vert;
        s.vert_sel = {survivor};
        s.face_sel.clear();
        s.edge_sel.clear();
        return ok("merged to vert " + std::to_string(survivor));
    }

    if (kind == "fill") {
        Mesh* mesh = s.selected_mesh();
        if (!mesh) return fail("fill: no mesh");
        std::vector<Edge> edges = s.edge_sel;
        if (edges.empty() && s.vert_sel.size() >= 3) {
            for (size_t i = 0; i < s.vert_sel.size(); ++i)
                edges.push_back(make_edge(s.vert_sel[i], s.vert_sel[(i + 1) % s.vert_sel.size()]));
        }
        std::vector<int> neu;
        if (!fill_loops(*mesh, edges, neu)) {
            if (mutates) doc.undo_once();
            return fail("fill: no loop");
        }
        s.edit_mode = EditMode::Face;
        s.face_sel = neu;
        s.edge_sel.clear();
        s.vert_sel.clear();
        return ok("filled " + std::to_string(neu.size()) + " faces");
    }

    if (kind == "mirror") {
        Mesh* mesh = s.selected_mesh();
        if (!mesh) return fail("mirror: no mesh");
        int axis = 0;
        Vec3 ax;
        std::string as = require_str(op, "axis", "x");
        if (parse_axis(as, ax)) {
            if (std::abs(ax.y) > 0.5f) axis = 1;
            else if (std::abs(ax.z) > 0.5f) axis = 2;
            else axis = 0;
        }
        if (!mirror_mesh(*mesh, axis)) {
            if (mutates) doc.undo_once();
            return fail("mirror failed");
        }
        s.face_sel.clear();
        s.edge_sel.clear();
        s.vert_sel.clear();
        s.edit_mode = EditMode::Object;
        return ok(std::string("mirrored ") + (axis == 1 ? "y" : axis == 2 ? "z" : "x"));
    }

    if (kind == "solidify") {
        Mesh* mesh = s.selected_mesh();
        if (!mesh) return fail("solidify: no mesh");
        float t = require_f(op, "thickness", require_f(op, "amount", 0.08f));
        if (!solidify_mesh(*mesh, t)) {
            if (mutates) doc.undo_once();
            return fail("solidify failed");
        }
        s.face_sel.clear();
        s.edge_sel.clear();
        s.vert_sel.clear();
        s.edit_mode = EditMode::Object;
        return ok("solidified " + std::to_string(t));
    }

    if (kind == "array") {
        Mesh* mesh = s.selected_mesh();
        if (!mesh) return fail("array: no mesh");
        int count = 3;
        if (op.contains("count") && op["count"].is_number()) count = std::max(2, int(op["count"].get<float>()));
        Vec3 off = require_vec3(op, "offset", {0, 0, 0});
        if (length(off) < 1e-6f) {
            Vec3 ax{1, 0, 0};
            parse_axis(require_str(op, "axis", "x"), ax);
            float sp = require_f(op, "spacing", 1.2f);
            off = ax * sp;
        }
        if (!array_mesh(*mesh, count, off)) {
            if (mutates) doc.undo_once();
            return fail("array failed");
        }
        s.face_sel.clear();
        s.edge_sel.clear();
        s.vert_sel.clear();
        s.edit_mode = EditMode::Object;
        return ok("array x" + std::to_string(count));
    }

    if (kind == "spin") {
        Mesh* mesh = s.selected_mesh();
        if (!mesh) return fail("spin: no mesh");
        int steps = 8;
        if (op.contains("steps") && op["steps"].is_number()) steps = std::max(2, int(op["steps"].get<float>()));
        if (op.contains("count") && op["count"].is_number()) steps = std::max(2, int(op["count"].get<float>()));
        float ang = require_f(op, "angle", 360.0f);
        Vec3 ax{0, 1, 0};
        parse_axis(require_str(op, "axis", "y"), ax);
        Vec3 center = require_vec3(op, "center", {0, 0, 0});
        std::vector<Edge> profile = s.edge_sel;
        if (!spin_mesh(*mesh, profile, steps, ang, ax, center)) {
            if (mutates) doc.undo_once();
            return fail("spin failed");
        }
        s.face_sel.clear();
        s.edge_sel.clear();
        s.vert_sel.clear();
        s.edit_mode = EditMode::Object;
        return ok("spun x" + std::to_string(steps));
    }

    if (kind == "bridge") {
        Mesh* mesh = s.selected_mesh();
        if (!mesh) return fail("bridge: no mesh");
        std::vector<int> neu;
        if (!bridge_loops(*mesh, s.face_sel, s.edge_sel, neu)) {
            if (mutates) doc.undo_once();
            return fail("bridge: need two loops");
        }
        s.edit_mode = EditMode::Face;
        s.face_sel = neu;
        s.edge_sel.clear();
        s.vert_sel.clear();
        return ok("bridged " + std::to_string(neu.size()) + " faces");
    }

    if (kind == "join") {
        int b = -1;
        if (op.contains("other") && op["other"].is_string()) b = s.find_node(op["other"].get<std::string>());
        else if (op.contains("with") && op["with"].is_string()) b = s.find_node(op["with"].get<std::string>());
        int a = -1;
        if (op.contains("target") && op["target"].is_string()) a = s.find_node(op["target"].get<std::string>());
        if (a < 0 && s.selection >= 0 && s.selection != b) a = s.selection;
        if (a < 0) {
            for (int i = 0; i < int(s.nodes.size()); ++i) {
                if (i != b && s.nodes[i].type == NodeType::Mesh && s.nodes[i].mesh >= 0) {
                    a = i;
                    break;
                }
            }
        }
        if (a < 0 || b < 0) return fail("join: need two meshes");
        if (a == b) return fail("join: same object");
        if (s.nodes[a].type != NodeType::Mesh || s.nodes[b].type != NodeType::Mesh)
            return fail("join: not meshes");
        if (s.nodes[a].mesh < 0 || s.nodes[b].mesh < 0) return fail("join: missing mesh");
        std::string an = s.nodes[a].name;
        std::string bn = s.nodes[b].name;
        int users = 0;
        for (const auto& n : s.nodes)
            if (n.mesh == s.nodes[a].mesh) ++users;
        if (users > 1) {
            s.meshes.push_back(s.meshes[s.nodes[a].mesh]);
            s.nodes[a].mesh = int(s.meshes.size()) - 1;
        }
        Mat4 xa = s.world_matrix(a);
        Mat4 xb = s.world_matrix(b);
        Mat4 xform = mul(invert_affine(xa), xb);
        mesh_append_transformed(s.meshes[s.nodes[a].mesh], s.meshes[s.nodes[b].mesh], xform);
        mesh_sync_render(s.meshes[s.nodes[a].mesh]);
        s.remove_node_at(b);
        s.selection = s.find_node(an);
        s.edit_mode = EditMode::Object;
        s.face_sel.clear();
        s.edge_sel.clear();
        s.vert_sel.clear();
        return ok("joined " + bn + " into " + an);
    }

    if (kind == "ground" || kind == "to_ground") {
        int i = resolve_node(s, op);
        if (i < 0) i = s.selection;
        if (i < 0) return fail("ground: no target");
        Node& n = s.nodes[i];
        if (n.type != NodeType::Mesh || n.mesh < 0) return fail("ground: not a mesh");
        Mesh& mesh = s.meshes[n.mesh];
        Mat4 W = s.world_matrix(i);
        float ymin = 1e9f;
        for (const auto& p : mesh.positions) ymin = std::min(ymin, transform_point(W, p).y);
        if (ymin > 1e8f) return fail("ground: empty");
        n.translation.y -= ymin;
        return ok("grounded " + n.name);
    }

    if (kind == "origin") {
        int i = resolve_node(s, op);
        if (i < 0) i = s.selection;
        if (i < 0) return fail("origin: no target");
        Node& n = s.nodes[i];
        if (n.type != NodeType::Mesh || n.mesh < 0) return fail("origin: not a mesh");
        Mesh& mesh = s.meshes[n.mesh];
        Vec3 c = mesh_centroid(mesh);
        for (auto& p : mesh.positions) p = p - c;
        mesh_sync_render(mesh);
        Vec3 world_c = transform_point(s.world_matrix(i), c);
        if (n.parent >= 0) {
            Mat4 invp = invert_affine(s.world_matrix(n.parent));
            n.translation = transform_point(invp, world_c);
        } else {
            n.translation = world_c;
        }
        return ok("origin to geometry " + n.name);
    }

    if (kind == "rig") {
        std::string k = require_str(op, "kind", require_str(op, "rig", "biped"));
        int root = -1;
        if (k == "quadruped" || k == "animal" || k == "dog" || k == "horse") root = rig_quadruped(s);
        else if (k == "face" || k == "facial") root = rig_face(s);
        else {
            bool apose = require_str(op, "pose", "") == "apose" || require_str(op, "pose", "") == "a";
            if (k == "apose") apose = true;
            root = rig_biped(s, apose);
        }
        if (root < 0) {
            if (mutates) doc.undo_once();
            return fail("rig failed");
        }
        return ok(std::string("rigged ") + k + " at " + s.nodes[root].name);
    }

    if (kind == "bind") {
        int i = resolve_node(s, op);
        if (i < 0) i = s.selection;
        if (i < 0 || s.nodes[i].type != NodeType::Mesh) {
            i = -1;
            for (int k = 0; k < int(s.nodes.size()); ++k)
                if (s.nodes[k].type == NodeType::Mesh && s.nodes[k].mesh >= 0) {
                    i = k;
                    break;
                }
        }
        if (i < 0) return fail("bind: no mesh");
        if (!bind_mesh(s, i)) {
            if (mutates) doc.undo_once();
            return fail("bind: need a mesh and bones");
        }
        return ok("bound skin");
    }

    if (kind == "pose") {
        int i = resolve_node(s, op);
        if (i < 0) i = s.selection;
        if (i < 0 || s.nodes[i].type != NodeType::Bone) return fail("pose: pick a bone");
        if (op.contains("rotation")) s.nodes[i].pose_rot = require_vec3(op, "rotation", s.nodes[i].pose_rot);
        if (op.contains("delta")) s.nodes[i].pose_rot += require_vec3(op, "delta");
        if (op.contains("translation")) s.nodes[i].pose_loc = require_vec3(op, "translation", s.nodes[i].pose_loc);
        return ok("posed " + s.nodes[i].name);
    }

    if (kind == "rest") {
        for (auto& n : s.nodes) {
            if (n.type != NodeType::Bone) continue;
            n.pose_rot = {0, 0, 0};
            n.pose_loc = {0, 0, 0};
        }
        return ok("rest pose");
    }

    if (kind == "shape") {
        Mesh* mesh = s.selected_mesh();
        if (!mesh) {
            for (auto& n : s.nodes)
                if (n.type == NodeType::Mesh && n.mesh >= 0) {
                    mesh = &s.meshes[n.mesh];
                    break;
                }
        }
        if (!mesh) return fail("shape: no mesh");
        std::string name = require_str(op, "name", require_str(op, "shape"));
        float v = require_f(op, "value", require_f(op, "amount", 1.0f));
        bool found = false;
        for (auto& sh : mesh->shapes) {
            if (name.empty() || sh.name == name) {
                sh.value = v;
                found = true;
                if (!name.empty()) break;
            }
        }
        if (!found) return fail("shape: no key '" + name + "'");
        return ok("shape " + name + " = " + std::to_string(v));
    }

    if (kind == "lod") {
        Mesh* mesh = s.selected_mesh();
        if (!mesh) return fail("lod: no mesh");
        int levels = 3;
        if (op.contains("levels") && op["levels"].is_number()) levels = std::max(1, int(op["levels"].get<float>()));
        if (op.contains("level") && op["level"].is_number()) {
            mesh->lod = std::max(0, int(op["level"].get<float>()));
            if (mesh->lod > int(mesh->lods.size())) mesh->lod = int(mesh->lods.size());
            return ok("lod " + std::to_string(mesh->lod));
        }
        if (!build_lods(*mesh, levels)) {
            if (mutates) doc.undo_once();
            return fail("lod failed");
        }
        mesh->lod = 0;
        return ok("built " + std::to_string(mesh->lods.size()) + " lods");
    }

    if (kind == "describe") {
        return ok(describe_scene(s).dump(2));
    }

    if (kind == "triangulate") {
        Mesh* mesh = s.selected_mesh();
        if (!mesh) return fail("triangulate: no mesh");
        if (!triangulate_faces(*mesh, s.face_sel)) {
            if (mutates) doc.undo_once();
            return fail("triangulate failed");
        }
        return ok("triangulated " + std::to_string(mesh->faces.size()) + " faces");
    }

    if (kind == "poke") {
        Mesh* mesh = s.selected_mesh();
        if (!mesh) return fail("poke: no mesh");
        std::vector<int> neu;
        if (!poke_faces(*mesh, s.face_sel, neu)) {
            if (mutates) doc.undo_once();
            return fail("poke failed");
        }
        s.edit_mode = EditMode::Face;
        s.face_sel = neu;
        return ok("poked " + std::to_string(neu.size()) + " faces");
    }

    if (kind == "flip" || kind == "flip_normals") {
        Mesh* mesh = s.selected_mesh();
        if (!mesh) return fail("flip: no mesh");
        if (!flip_faces(*mesh, s.face_sel)) {
            if (mutates) doc.undo_once();
            return fail("flip failed");
        }
        return ok("flipped faces");
    }

    if (kind == "dissolve") {
        Mesh* mesh = s.selected_mesh();
        if (!mesh) return fail("dissolve: no mesh");
        std::vector<int> neu;
        if (!dissolve_faces(*mesh, s.face_sel, neu)) {
            if (mutates) doc.undo_once();
            return fail("dissolve: select faces");
        }
        s.face_sel = neu;
        s.edit_mode = EditMode::Face;
        return ok("dissolved to " + std::to_string(neu.size()) + " faces");
    }

    if (kind == "duplicate_faces") {
        Mesh* mesh = s.selected_mesh();
        if (!mesh) return fail("duplicate_faces: no mesh");
        std::vector<int> faces = s.face_sel;
        if (faces.empty() && op.contains("region") && op["region"].is_string())
            faces = faces_matching(*mesh, op["region"].get<std::string>());
        float amt = require_f(op, "amount", 0.0f);
        std::vector<int> neu;
        if (!duplicate_faces_mesh(*mesh, faces, amt, neu)) {
            if (mutates) doc.undo_once();
            return fail("duplicate_faces: select faces");
        }
        s.edit_mode = EditMode::Face;
        s.face_sel = neu;
        return ok("duplicated " + std::to_string(neu.size()) + " faces");
    }

    if (kind == "separate") {
        Mesh* mesh = s.selected_mesh();
        if (!mesh) return fail("separate: no mesh");
        int ni = s.selection;
        std::vector<int> faces = s.face_sel;
        if (faces.empty()) return fail("separate: select faces");
        Mesh part;
        part.name = mesh->name + "_sep";
        std::set<uint32_t> used;
        for (int fi : faces)
            if (fi >= 0 && fi < int(mesh->faces.size()))
                for (uint32_t v : mesh->faces[fi].v) used.insert(v);
        std::map<uint32_t, uint32_t> remap;
        for (uint32_t v : used) {
            remap[v] = uint32_t(part.positions.size());
            part.positions.push_back(mesh->positions[v]);
        }
        for (int fi : faces) {
            if (fi < 0 || fi >= int(mesh->faces.size())) continue;
            Face nf;
            for (uint32_t v : mesh->faces[fi].v) nf.v.push_back(remap[v]);
            part.faces.push_back(std::move(nf));
        }
        mesh_sync_render(part);
        delete_faces_mesh(*mesh, faces);
        s.meshes.push_back(std::move(part));
        Node n = s.nodes[ni];
        n.id = 0;
        n.name = s.unique_name(n.name + "_sep");
        n.mesh = int(s.meshes.size()) - 1;
        n.translation.x += 1.2f;
        int pi = s.add_node(std::move(n));
        s.selection = pi;
        s.face_sel.clear();
        s.edit_mode = EditMode::Object;
        return ok("separated " + s.nodes[pi].name);
    }

    if (kind == "weld" || kind == "weld_distance") {
        Mesh* mesh = s.selected_mesh();
        if (!mesh) return fail("weld: no mesh");
        float d = require_f(op, "distance", require_f(op, "amount", 0.001f));
        if (!weld_distance(*mesh, d)) {
            if (mutates) doc.undo_once();
            return fail("weld: nothing in range");
        }
        s.vert_sel.clear();
        return ok("welded to " + std::to_string(mesh->positions.size()) + " verts");
    }

    if (kind == "smooth") {
        Mesh* mesh = s.selected_mesh();
        if (!mesh) return fail("smooth: no mesh");
        auto vs = current_verts(s, *mesh);
        float f = require_f(op, "factor", require_f(op, "amount", 0.5f));
        int it = 1;
        if (op.contains("iters") && op["iters"].is_number()) it = std::max(1, int(op["iters"].get<float>()));
        if (!smooth_verts(*mesh, vs, f, it)) {
            if (mutates) doc.undo_once();
            return fail("smooth failed");
        }
        return ok("smoothed " + std::to_string(vs.size()) + " verts");
    }

    if (kind == "push") {
        Mesh* mesh = s.selected_mesh();
        if (!mesh) return fail("push: no mesh");
        auto vs = current_verts(s, *mesh);
        float a = require_f(op, "amount", 0.1f);
        if (!push_verts(*mesh, vs, a)) {
            if (mutates) doc.undo_once();
            return fail("push failed");
        }
        return ok("pushed " + std::to_string(vs.size()) + " verts");
    }

    if (kind == "sphere" || kind == "to_sphere") {
        Mesh* mesh = s.selected_mesh();
        if (!mesh) return fail("sphere: no mesh");
        auto vs = current_verts(s, *mesh);
        float f = require_f(op, "factor", require_f(op, "amount", 1.0f));
        if (!to_sphere(*mesh, vs, f)) {
            if (mutates) doc.undo_once();
            return fail("sphere failed");
        }
        return ok("to sphere");
    }

    if (kind == "flatten") {
        Mesh* mesh = s.selected_mesh();
        if (!mesh) return fail("flatten: no mesh");
        auto vs = current_verts(s, *mesh);
        Vec3 ax{0, 1, 0};
        parse_axis(require_str(op, "axis", "y"), ax);
        if (!flatten_verts(*mesh, vs, ax)) {
            if (mutates) doc.undo_once();
            return fail("flatten failed");
        }
        return ok("flattened");
    }

    if (kind == "randomize") {
        Mesh* mesh = s.selected_mesh();
        if (!mesh) return fail("randomize: no mesh");
        auto vs = current_verts(s, *mesh);
        float a = require_f(op, "amount", 0.05f);
        if (!randomize_verts(*mesh, vs, a)) {
            if (mutates) doc.undo_once();
            return fail("randomize failed");
        }
        return ok("randomized");
    }

    if (kind == "apply") {
        int i = resolve_node(s, op);
        if (i < 0) i = s.selection;
        if (i < 0) return fail("apply: no target");
        Node& n = s.nodes[i];
        if (n.type != NodeType::Mesh || n.mesh < 0) return fail("apply: not a mesh");
        Mesh& mesh = s.meshes[n.mesh];
        Mat4 R = euler_xyz(n.rotation_deg);
        Mat4 S = scale(n.scale);
        Mat4 X = mul(R, S);
        for (auto& p : mesh.positions) p = transform_point(X, p);
        mesh_sync_render(mesh);
        n.rotation_deg = {0, 0, 0};
        n.scale = {1, 1, 1};
        return ok("applied transform " + n.name);
    }

    if (kind == "bisect") {
        Mesh* mesh = s.selected_mesh();
        if (!mesh) return fail("bisect: no mesh");
        Vec3 ax{0, 1, 0};
        parse_axis(require_str(op, "axis", "y"), ax);
        float d = require_f(op, "distance", require_f(op, "offset", 0.0f));
        if (!bisect_mesh(*mesh, ax, d)) {
            if (mutates) doc.undo_once();
            return fail("bisect failed");
        }
        s.face_sel.clear();
        return ok("bisected " + std::to_string(mesh->faces.size()) + " faces");
    }

    if (kind == "snap") {
        Mesh* mesh = s.selected_mesh();
        if (!mesh) return fail("snap: no mesh");
        auto vs = current_verts(s, *mesh);
        float g = require_f(op, "grid", require_f(op, "amount", 0.1f));
        if (!snap_verts(*mesh, vs, g)) {
            if (mutates) doc.undo_once();
            return fail("snap failed");
        }
        return ok("snapped");
    }

    if (kind == "split") {
        Mesh* mesh = s.selected_mesh();
        if (!mesh) return fail("split: no mesh");
        if (s.face_sel.empty()) return fail("split: select faces");
        if (!split_region(*mesh, s.face_sel)) {
            if (mutates) doc.undo_once();
            return fail("split failed");
        }
        return ok("split region");
    }

    if (kind == "connect") {
        Mesh* mesh = s.selected_mesh();
        if (!mesh) return fail("connect: no mesh");
        if (s.vert_sel.size() < 2) return fail("connect: select 2 verts");
        if (!connect_verts(*mesh, s.vert_sel[0], s.vert_sel[1])) {
            if (mutates) doc.undo_once();
            return fail("connect: verts not on a face");
        }
        return ok("connected verts");
    }

    if (kind == "grow" || kind == "select_more") {
        Mesh* mesh = s.selected_mesh();
        if (!mesh) return fail("grow: no mesh");
        if (s.edit_mode == EditMode::Face || !s.face_sel.empty()) {
            s.face_sel = grow_faces(*mesh, s.face_sel);
            s.edit_mode = EditMode::Face;
            return ok("grew to " + std::to_string(s.face_sel.size()) + " faces");
        }
        if (s.edit_mode == EditMode::Edge || !s.edge_sel.empty()) {
            s.edge_sel = grow_edges(*mesh, s.edge_sel);
            s.edit_mode = EditMode::Edge;
            return ok("grew to " + std::to_string(s.edge_sel.size()) + " edges");
        }
        s.vert_sel = grow_verts(*mesh, s.vert_sel);
        s.edit_mode = EditMode::Vert;
        return ok("grew to " + std::to_string(s.vert_sel.size()) + " verts");
    }

    if (kind == "invert") {
        Mesh* mesh = s.selected_mesh();
        if (!mesh) return fail("invert: no mesh");
        if (s.edit_mode == EditMode::Face || !s.face_sel.empty()) {
            std::set<int> cur(s.face_sel.begin(), s.face_sel.end());
            s.face_sel.clear();
            for (int i = 0; i < int(mesh->faces.size()); ++i)
                if (!cur.count(i)) s.face_sel.push_back(i);
            s.edit_mode = EditMode::Face;
            return ok("inverted " + std::to_string(s.face_sel.size()) + " faces");
        }
        if (s.edit_mode == EditMode::Vert || !s.vert_sel.empty()) {
            std::set<uint32_t> cur(s.vert_sel.begin(), s.vert_sel.end());
            s.vert_sel.clear();
            for (uint32_t i = 0; i < mesh->positions.size(); ++i)
                if (!cur.count(i)) s.vert_sel.push_back(i);
            s.edit_mode = EditMode::Vert;
            return ok("inverted " + std::to_string(s.vert_sel.size()) + " verts");
        }
        return fail("invert: nothing selected");
    }

    if (kind == "linked") {
        Mesh* mesh = s.selected_mesh();
        if (!mesh) return fail("linked: no mesh");
        s.face_sel = linked_faces(*mesh, s.face_sel);
        s.edit_mode = EditMode::Face;
        return ok("linked " + std::to_string(s.face_sel.size()) + " faces");
    }

    if (kind == "move" || kind == "set_translation") {
        int i = resolve_node(s, op);
        if (i < 0) return fail("move: no target");
        s.nodes[i].translation = require_vec3(op, "translation", s.nodes[i].translation);
        return ok("moved " + s.nodes[i].name);
    }

    if (kind == "translate") {
        Vec3 d = require_vec3(op, "delta");
        Mesh* mesh = s.selected_mesh();
        if (mesh && s.edit_mode != EditMode::Object) {
            std::vector<uint32_t> vs = s.vert_sel;
            if (vs.empty() && s.edit_mode == EditMode::Face) vs = verts_of_faces(*mesh, s.face_sel);
            if (vs.empty() && s.edit_mode == EditMode::Edge) {
                std::set<uint32_t> u;
                for (Edge e : s.edge_sel) {
                    u.insert(e.a);
                    u.insert(e.b);
                }
                vs.assign(u.begin(), u.end());
            }
            if (!vs.empty() && translate_verts(*mesh, vs, d))
                return ok("translated " + std::to_string(vs.size()) + " verts");
        }
        int i = resolve_node(s, op);
        if (i < 0) return fail("translate: no target");
        s.nodes[i].translation += d;
        return ok("translated " + s.nodes[i].name);
    }

    if (kind == "rotate" || kind == "set_rotation") {
        int i = resolve_node(s, op);
        if (i < 0) return fail("rotate: no target");
        if (op.contains("delta")) s.nodes[i].rotation_deg += require_vec3(op, "delta");
        else s.nodes[i].rotation_deg = require_vec3(op, "rotation", s.nodes[i].rotation_deg);
        return ok("rotated " + s.nodes[i].name);
    }

    if (kind == "scale" || kind == "set_scale") {
        int i = resolve_node(s, op);
        if (i < 0) return fail("scale: no target");
        s.nodes[i].scale = require_vec3(op, "scale", s.nodes[i].scale);
        return ok("scaled " + s.nodes[i].name);
    }

    if (kind == "color" || kind == "set_material") {
        int i = resolve_node(s, op);
        if (i < 0) return fail("material: no target");
        Node& n = s.nodes[i];
        if (n.material < 0) {
            n.material = add_material(s, {0.72f, 0.73f, 0.76f});
        }
        Material& m = s.materials[n.material];
        if (op.contains("albedo")) m.albedo = require_vec3(op, "albedo", m.albedo);
        if (op.contains("roughness")) m.roughness = require_f(op, "roughness", m.roughness);
        if (op.contains("metallic")) m.metallic = require_f(op, "metallic", m.metallic);
        if (op.contains("emission")) m.emission = require_f(op, "emission", m.emission);
        return ok("material " + n.name);
    }

    if (kind == "parent") {
        int i = resolve_node(s, op);
        if (i < 0) return fail("parent: no child");
        std::string p = require_str(op, "parent");
        if (p.empty() || p == "none") {
            s.nodes[i].parent = -1;
            return ok("unparented " + s.nodes[i].name);
        }
        int pi = s.find_node(p);
        if (pi < 0) return fail("parent not found");
        if (pi == i) return fail("cannot parent to self");
        s.nodes[i].parent = pi;
        return ok(s.nodes[i].name + " parented to " + s.nodes[pi].name);
    }

    if (kind == "lookat") {
        int i = resolve_node(s, op);
        if (i < 0 || s.nodes[i].type != NodeType::Camera) {
            i = ensure_default_camera(s);
        }
        Vec3 eye = require_vec3(op, "eye", s.nodes[i].translation);
        Vec3 center = require_vec3(op, "center", {0, 0, 0});
        s.nodes[i].type = NodeType::Camera;
        s.nodes[i].translation = eye;
        s.nodes[i].rotation_deg = camera_euler_from_forward(center - eye);
        s.active_camera = i;
        return ok("camera " + s.nodes[i].name + " aimed");
    }

    // frame: like lookat, but you name what to look at instead of doing the
    // arithmetic yourself. target=<name> | all | scene.
    if (kind == "frame" || kind == "focus") {
        int i = resolve_node(s, op);
        if (i < 0 || s.nodes[i].type != NodeType::Camera) i = ensure_default_camera(s);
        Vec3 lo{1e9f, 1e9f, 1e9f}, hi{-1e9f, -1e9f, -1e9f};
        bool any = false;
        auto add = [&](Vec3 p) {
            lo.x = std::min(lo.x, p.x); lo.y = std::min(lo.y, p.y); lo.z = std::min(lo.z, p.z);
            hi.x = std::max(hi.x, p.x); hi.y = std::max(hi.y, p.y); hi.z = std::max(hi.z, p.z);
            any = true;
        };
        std::string tgt = require_str(op, "target", "");
        int only = tgt.empty() ? -1 : s.find_node(tgt);
        if (!tgt.empty() && only < 0) return fail("frame: no node '" + tgt + "'");
        for (int ni = 0; ni < int(s.nodes.size()); ++ni) {
            if (ni == i || s.nodes[ni].type != NodeType::Mesh) continue;
            if (only >= 0 && ni != only) continue;
            const Node& nd = s.nodes[ni];
            Mat4 W = s.world_matrix(ni);
            // Use the real mesh bounds, not just the pivot: a single object framed
            // by its pivot alone would put the camera inside the object.
            if (nd.mesh >= 0 && nd.mesh < int(s.meshes.size()) && !s.meshes[nd.mesh].positions.empty()) {
                const Mesh& mesh = s.meshes[nd.mesh];
                int lod = mesh.lod;
                if (lod > 0 && lod <= int(mesh.lods.size())) {
                    for (Vec3 p : mesh.lods[lod - 1].positions) add(transform_point(W, p));
                } else {
                    for (Vec3 p : mesh.positions) add(transform_point(W, p));
                }
            } else {
                add(nd.translation);
            }
        }
        if (!any) return fail("frame: nothing to frame");
        Vec3 center = (lo + hi) * 0.5f;
        float radius = std::max(0.25f, length(hi - center));
        float fov = s.nodes[i].fov_deg > 1.0f ? s.nodes[i].fov_deg : 50.0f;
        float margin = require_f(op, "margin", 1.35f);
        if (!finite(margin) || margin < 1.0f) margin = 1.35f;
        float dist = radius / std::tan(deg2rad(fov) * 0.5f) * margin;
        std::string ax = require_str(op, "axis", "iso");
        Vec3 dir = ax == "front" ? Vec3{0, 0, 1}
                 : ax == "side"  ? Vec3{1, 0, 0}
                 : ax == "top"   ? Vec3{0, 1, 0.001f}
                                 : normalize(Vec3{0.78f, 0.52f, 1.0f});
        Vec3 eye = center + normalize(dir) * dist;
        s.nodes[i].translation = eye;
        s.nodes[i].rotation_deg = camera_euler_from_forward(center - eye);
        s.active_camera = i;
        return ok("framed " + (only >= 0 ? s.nodes[only].name : std::string("scene")));
    }

    if (kind == "set_camera") {
        int i = resolve_node(s, op);
        if (i < 0) return fail("camera not found");
        s.active_camera = i;
        if (op.contains("fov")) s.nodes[i].fov_deg = require_f(op, "fov", 50);
        return ok("active camera " + s.nodes[i].name);
    }

    if (kind == "subdivide_surface" || kind == "subsurf" || kind == "catmull_clark") {
        Mesh* mesh = s.selected_mesh();
        if (!mesh) return fail("subsurf: no mesh");
        int levels = int(require_f(op, "levels", require_f(op, "amount", 1.0f)));
        size_t before = mesh->faces.size();
        if (!catmull_clark(*mesh, levels)) {
            if (mutates) doc.undo_once();
            return fail("subsurf: level count must be 1-4, got " + std::to_string(levels));
        }
        return ok("subdivided " + std::to_string(before) + " faces to " +
                  std::to_string(mesh->faces.size()) + ", " +
                  std::to_string(mesh->positions.size()) + " verts");
    }

    if (kind == "wireframe") {
        Mesh* mesh = s.selected_mesh();
        if (!mesh) return fail("wireframe: no mesh");
        float t = require_f(op, "thickness", require_f(op, "amount", 0.02f));
        size_t before = mesh->faces.size();
        if (!wireframe_mesh(*mesh, t)) {
            if (mutates) doc.undo_once();
            return fail("wireframe: mesh has no edges, or thickness is not positive");
        }
        return ok("wireframe " + std::to_string(before) + " faces to " +
                  std::to_string(mesh->faces.size()) + ", thickness " + std::to_string(t));
    }

    if (kind == "shrinkwrap") {
        Mesh* mesh = s.selected_mesh();
        if (!mesh) return fail("shrinkwrap: no mesh");
        int ti = resolve_node(s, op);
        if (ti < 0 || ti >= static_cast<int>(s.nodes.size()) ||
            s.nodes[static_cast<size_t>(ti)].mesh < 0)
            return fail("shrinkwrap: name a target mesh node");
        const Mesh& target = s.meshes[static_cast<size_t>(s.nodes[static_cast<size_t>(ti)].mesh)];
        if (&target == mesh) return fail("shrinkwrap: target is the same mesh");
        float w = require_f(op, "weight", 1.0f);
        if (!shrinkwrap_mesh(*mesh, target, w)) {
            if (mutates) doc.undo_once();
            return fail("shrinkwrap: nothing to wrap, or weight is not in (0, 1]");
        }
        return ok("shrinkwrapped " + std::to_string(mesh->positions.size()) + " verts onto " +
                  s.nodes[static_cast<size_t>(ti)].name + " at weight " + std::to_string(w));
    }

    if (kind == "symmetrize" || kind == "mirror_symmetry") {
        Mesh* mesh = s.selected_mesh();
        if (!mesh) return fail("symmetrize: no mesh");
        std::string ax = lower_copy(require_str(op, "axis", "x"));
        int axis = ax == "x" ? 0 : (ax == "y" ? 1 : (ax == "z" ? 2 : -1));
        if (axis < 0) return fail("symmetrize: axis must be x, y or z");
        size_t before = mesh->faces.size();
        if (!symmetrize_mesh(*mesh, axis)) {
            if (mutates) doc.undo_once();
            return fail("symmetrize: no face lies wholly on one side of the " + ax + " plane");
        }
        return ok("symmetrized across " + ax + ": " + std::to_string(before) + " faces to " +
                  std::to_string(mesh->faces.size()) + ", " +
                  std::to_string(mesh->positions.size()) + " verts");
    }

    if (kind == "boundary") {
        Mesh* mesh = s.selected_mesh();
        if (!mesh) return fail("boundary: no mesh");
        std::vector<int> b = boundary_faces(*mesh);
        return {true, std::to_string(b.size()) + " boundary faces of " +
                         std::to_string(mesh->faces.size()),
                {{"boundary_faces", b.size()}, {"total_faces", mesh->faces.size()}}};
    }

    if (kind == "extrude_individual") {
        Mesh* mesh = s.selected_mesh();
        if (!mesh) return fail("extrude_individual: no mesh");
        std::vector<int> faces = s.face_sel;
        if (faces.empty()) return fail("extrude_individual: select faces first");
        float amount = require_f(op, "amount", 0.0f);
        std::vector<int> ns;
        if (!extrude_individual(*mesh, faces, amount, ns)) {
            if (mutates) doc.undo_once();
            return fail("extrude_individual: amount must be non-zero");
        }
        s.face_sel = ns;
        return ok("extruded " + std::to_string(ns.size()) + " faces individually by " +
                  std::to_string(amount));
    }

    if (kind == "rotate_verts") {
        Mesh* mesh = s.selected_mesh();
        if (!mesh) return fail("rotate_verts: no mesh");
        std::vector<uint32_t> vs = current_verts(s, *mesh);
        Vec3 e = require_vec3(op, "rotation", {0, 0, 0});
        if (length(e) < 1e-9f) e = require_vec3(op, "delta", {0, 0, 0});
        if (!rotate_verts(*mesh, vs, e)) {
            if (mutates) doc.undo_once();
            return fail("rotate_verts: no vertices to rotate");
        }
        return ok("rotated " + std::to_string(vs.size()) + " verts by " +
                  std::to_string(e.x) + "," + std::to_string(e.y) + "," + std::to_string(e.z));
    }

    if (kind == "scale_verts") {
        Mesh* mesh = s.selected_mesh();
        if (!mesh) return fail("scale_verts: no mesh");
        std::vector<uint32_t> vs = current_verts(s, *mesh);
        Vec3 s3 = require_vec3(op, "scale", {1, 1, 1});
        if (op.contains("factor")) {
            float f = require_f(op, "factor", 1.0f);
            s3 = {f, f, f};
        }
        if (!scale_verts(*mesh, vs, s3)) {
            if (mutates) doc.undo_once();
            return fail("scale_verts: no vertices, or a scale component is zero");
        }
        return ok("scaled " + std::to_string(vs.size()) + " verts by " +
                  std::to_string(s3.x) + "," + std::to_string(s3.y) + "," + std::to_string(s3.z));
    }

    if (kind == "shear_verts") {
        Mesh* mesh = s.selected_mesh();
        if (!mesh) return fail("shear_verts: no mesh");
        std::vector<uint32_t> vs = current_verts(s, *mesh);
        std::string ax = lower_copy(require_str(op, "axis", "x"));
        Vec3 axis = ax == "y" ? Vec3{0, 1, 0} : (ax == "z" ? Vec3{0, 0, 1} : Vec3{1, 0, 0});
        float factor = require_f(op, "factor", require_f(op, "amount", 0.0f));
        if (!shear_verts(*mesh, vs, axis, factor)) {
            if (mutates) doc.undo_once();
            return fail("shear_verts: axis must be x, y or z and factor non-zero");
        }
        return ok("sheared " + std::to_string(vs.size()) + " verts along " + ax + " by " +
                  std::to_string(factor));
    }

    if (kind == "frame_at" || kind == "set_frame") {
        int f = int(require_f(op, "frame", float(s.frame)));
        apply_frame(s, f);
        return ok("frame " + std::to_string(s.frame) + " applied to " +
                  std::to_string(s.keys.size()) + " keys");
    }

    if (kind == "ik" || kind == "ik_two_bone") {
        int i = resolve_node(s, op);
        if (i < 0 || i >= static_cast<int>(s.nodes.size()) ||
            s.nodes[static_cast<size_t>(i)].type != NodeType::Bone)
            return fail("ik: target must be a bone");
        Vec3 target = require_vec3(op, "target", {0, 0, 0});
        if (op.contains("at")) target = require_vec3(op, "at", target);
        if (length(target) < 1e-9f) return fail("ik: needs a 'target' or 'at' position");
        std::string tn = s.nodes[static_cast<size_t>(i)].name;
        if (!ik_two_bone(s, i, target)) {
            if (mutates) doc.undo_once();
            return fail("ik: " + tn + " needs a parent and grandparent bone, and non-zero lengths");
        }
        return ok("solved two-bone IK for " + tn + " toward " + std::to_string(target.x) + "," +
                  std::to_string(target.y) + "," + std::to_string(target.z));
    }

    if (kind == "import_obj") {
        std::string path = require_str(op, "path");
        if (path.empty()) return fail("import_obj needs 'path'");
        size_t nodes_before = s.nodes.size();
        size_t meshes_before = s.meshes.size();
        ObjImportNotes notes;
        std::string err;
        if (!import_obj(s, path, err, notes)) {
            s.nodes.resize(nodes_before);
            s.meshes.resize(meshes_before);
            return fail("import_obj failed: " + err);
        }
        if (op.value("replace", false)) {
            s.next_id = 1;
            s.selection = -1;
            s.active_camera = -1;
        }
        std::string msg = "imported " + path + ": " + std::to_string(notes.meshes) + " meshes, " +
                          std::to_string(notes.faces) + " faces, " +
                          std::to_string(notes.vertices) + " verts";
        nlohmann::json data = {{"meshes", notes.meshes},
                               {"faces", notes.faces},
                               {"vertices", notes.vertices}};
        std::vector<std::string> dropped;
        if (notes.had_uvs) dropped.push_back("vt");
        if (notes.had_normals) dropped.push_back("vn");
        if (!dropped.empty()) {
            msg += " (dropped:";
            for (const auto& d : dropped) msg += " " + d;
            msg += ")";
            data["dropped_attributes"] = dropped;
        }
        return {true, msg, data};
    }

    if (kind == "export_obj") {
        std::string path = require_str(op, "path");
        if (path.empty()) return fail("export_obj needs 'path'");
        std::string err;
        if (!export_obj(s, path, err)) return fail("export_obj failed: " + err);
        return ok("wrote " + path);
    }

    if (kind == "shade_smooth" || kind == "shade_flat") {
        Mesh* mesh = s.selected_mesh();
        if (!mesh) return fail(kind + ": no mesh");
        const bool want_smooth = (kind == "shade_smooth");
        if (mesh->smooth == want_smooth) {
            return ok(std::string(kind) + ": already " +
                      (want_smooth ? "smooth" : "flat"));
        }
        mesh->smooth = want_smooth;
        // Corner attributes carry per-corner normals, which would override the
        // shading choice, so they are dropped when the mode changes.
        mesh->corner_attr.clear();
        mesh_sync_render(*mesh);
        return ok(std::string(kind) + ": " + (want_smooth ? "smooth" : "flat") + " shading on " +
                  mesh->name);
    }

    if (kind == "displace") {
        Mesh* mesh = s.selected_mesh();
        if (!mesh) return fail("displace: no mesh selected");
        if (mesh->positions.empty()) return fail("displace: the mesh has no vertices");
        const float amplitude = require_f(op, "amplitude", 0.1f);
        const float frequency = require_f(op, "frequency", 2.0f);
        const int seed = int(require_f(op, "seed", 0.0f));
        const int octaves = int(require_f(op, "octaves", 3.0f));
        const bool along_normal =
            op.contains("direction")
                ? require_str(op, "direction", "normal") == "normal"
                : true;
        const size_t before = mesh->positions.size();
        if (!(frequency > 0.0f)) {
            return fail("displace: frequency must be greater than 0, got " +
                        std::to_string(frequency));
        }
        if (octaves < 1 || octaves > 8) {
            return fail("displace: octaves must be 1..8, got " + std::to_string(octaves));
        }
        if (!displace_mesh(*mesh, amplitude, frequency, seed, octaves, along_normal)) {
            return fail("displace: could not displace " + std::to_string(before) +
                        " verts (the render mesh is out of step with the positions)");
        }
        const MeshStats ms = mesh_stats(*mesh);
        return {true,
                "displaced " + std::to_string(before) + " verts by amplitude " +
                    std::to_string(amplitude) + " at frequency " +
                    std::to_string(frequency) + " (seed " + std::to_string(seed) +
                    ", " + std::to_string(octaves) + " octaves); volume now " +
                    std::to_string(ms.signed_volume),
                {{"verts", ms.verts},
                 {"faces", ms.faces},
                 {"amplitude", amplitude},
                 {"frequency", frequency},
                 {"seed", seed},
                 {"octaves", octaves},
                 {"degenerate_faces", ms.degenerate_faces},
                 {"non_finite_verts", ms.non_finite},
                 {"watertight", ms.watertight() && ms.non_finite == 0},
                 {"surface_area", ms.surface_area},
                 {"signed_volume", ms.signed_volume}}};
    }

    if (kind == "skin") {
        Mesh* mesh = s.selected_mesh();
        if (!mesh) return fail("skin: no mesh selected");
        if (mesh->positions.empty()) return fail("skin: the mesh has no vertices");
        std::vector<int> bones;
        std::string names;
        if (op.contains("bones")) {
            if (!op["bones"].is_array()) return fail("skin: 'bones' must be a list of node names");
            for (const auto& b : op["bones"]) {
                const std::string nm = b.is_string() ? b.get<std::string>() : std::string();
                const int idx = s.find_node(nm);
                if (idx < 0 || static_cast<size_t>(idx) >= s.nodes.size() ||
                    s.nodes[static_cast<size_t>(idx)].type != NodeType::Bone) {
                    return fail("skin: '" + nm + "' is not a bone node");
                }
                if (!names.empty()) names += ",";
                names += nm;
                bones.push_back(idx);
            }
        } else {
            for (size_t i = 0; i < s.nodes.size(); ++i) {
                if (s.nodes[i].type == NodeType::Bone) {
                    bones.push_back(static_cast<int>(i));
                    if (!names.empty()) names += ",";
                    names += s.nodes[i].name;
                }
            }
        }
        if (bones.empty()) return fail("skin: no bones found; create bone nodes or pass 'bones'");
        const float falloff = require_f(op, "falloff", 0.5f);
        // Bone world positions drive the weights, so the mesh ends up skinned to
        // the pose the rig is actually in.
        mesh->joint_positions.clear();
        for (const int b : bones) {
            mesh->joint_positions.push_back(
                transform_point(s.world_matrix(b), Vec3{0, 0, 0}));
        }
        if (!skin_mesh(*mesh, bones, falloff)) {
            return fail("skin: the mesh has no geometry to bind");
        }
        int unbound = 0;
        for (const SkinVert& sv : mesh->skin) {
            float sum = 0.0f;
            for (int k = 0; k < 4; ++k) sum += sv.w[k];
            if (std::fabs(sum - 1.0f) > 1e-3f) ++unbound;
        }
        const MeshStats ms = mesh_stats(*mesh);
        return {true,
                "skinned " + std::to_string(ms.verts) + " verts to " +
                    std::to_string(bones.size()) + " bones (" + names + ") with falloff " +
                    std::to_string(falloff) +
                    (unbound == 0 ? "" :
                        "; WARNING " + std::to_string(unbound) + " verts have weights that "
                        "do not sum to 1"),
                {{"verts", ms.verts},
                 {"bones", bones.size()},
                 {"bone_names", names},
                 {"falloff", falloff},
                 {"weights_not_normalised", unbound},
                 {"watertight", ms.watertight()}}};
    }

    if (kind == "boolean") {
        const std::string mode = require_str(op, "mode", "union");
        int bm = -1;
        if (mode == "union") bm = 0;
        else if (mode == "difference") bm = 1;
        else if (mode == "intersect") bm = 2;
        else {
            return fail("boolean: unknown mode '" + mode +
                        "'; use union, difference or intersect");
        }
        const int res = int(require_f(op, "resolution", 64.0f));
        const int ai = s.selection;
        if (ai < 0 || static_cast<size_t>(ai) >= s.nodes.size()) {
            return fail("boolean: select the first operand");
        }
        const Node& an = s.nodes[ai];
        if (an.type != NodeType::Mesh || an.mesh < 0 ||
            static_cast<size_t>(an.mesh) >= s.meshes.size()) {
            return fail("boolean: the selection is not a mesh");
        }
        const int bi = s.find_node(require_str(op, "other", ""));
        if (bi < 0 || static_cast<size_t>(bi) >= s.nodes.size()) {
            return fail("boolean: 'other' must name an existing node");
        }
        const Node& bn = s.nodes[bi];
        if (bn.type != NodeType::Mesh || bn.mesh < 0 ||
            static_cast<size_t>(bn.mesh) >= s.meshes.size()) {
            return fail("boolean: 'other' is not a mesh");
        }
        if (bi == ai) return fail("boolean: the two operands are the same object");

        // Both operands are baked to world space first. A boolean in local space
        // gives a wrong answer for any moved or rotated operand, and moving an
        // object is the most ordinary thing a user does.
        Mesh wa, wb;
        mesh_append_transformed(wa, s.meshes[an.mesh], s.world_matrix(ai));
        mesh_append_transformed(wb, s.meshes[bn.mesh], s.world_matrix(bi));
        wa.smooth = true;
        wb.smooth = true;

        Mesh out;
        if (!boolean_mesh(wa, wb, out, bm, res)) {
            return fail("boolean: the operation produced no surface. Disjoint operands "
                        "give an empty intersection; a resolution of " +
                        std::to_string(res) + " may also be too fine to resolve the "
                        "overlap.");
        }
        s.meshes.push_back(std::move(out));
        const int mi = static_cast<int>(s.meshes.size()) - 1;
        Mesh* nm = &s.meshes[mi];
        std::string base_name = s.meshes[an.mesh].name;
        if (base_name.empty()) base_name = "Mesh";
        Node n;
        n.type = NodeType::Mesh;
        n.name = base_name + "_" + mode;
        n.mesh = mi;
        n.material = an.material;
        const int i = s.add_node(n);
        s.selection = i;
        s.edit_mode = EditMode::Object;
        s.face_sel.clear();
        s.edge_sel.clear();
        s.vert_sel.clear();
        const MeshStats ms = mesh_stats(*nm);
        return {true,
                "boolean " + mode + " of " + base_name + " and " + bn.name + ": " +
                    std::to_string(ms.faces) + " faces, " +
                    std::to_string(ms.verts) + " verts, " +
                    (ms.watertight() && ms.non_finite == 0
                         ? "watertight"
                         : "NOT watertight: " + std::to_string(ms.boundary_edges) +
                               " boundary, " + std::to_string(ms.nonmanifold_edges) +
                               " non-manifold, " + std::to_string(ms.non_finite) +
                               " non-finite") +
                    ", volume " + std::to_string(ms.signed_volume) + " at resolution " +
                    std::to_string(res),
                {{"mode", mode},
                 {"faces", ms.faces},
                 {"verts", ms.verts},
                 {"boundary_edges", ms.boundary_edges},
                 {"nonmanifold_edges", ms.nonmanifold_edges},
                 {"degenerate_faces", ms.degenerate_faces},
                 {"non_finite_verts", ms.non_finite},
                 {"watertight", ms.watertight() && ms.non_finite == 0},
                 {"signed_volume", ms.signed_volume},
                 {"surface_area", ms.surface_area},
                 {"resolution", res}}};
    }

    if (kind == "remesh") {
        Mesh* mesh = s.selected_mesh();
        if (!mesh) return fail("remesh: no mesh selected");
        if (mesh->faces.empty()) return fail("remesh: the mesh has no faces");
        const size_t before = mesh->faces.size();

        // `target` is what a caller actually wants, so the resolution is searched
        // rather than exposed. Face count rises monotonically with resolution, so
        // a bisection finds the closest setting; the search is bounded to 7 steps
        // so a large mesh cannot make this unbounded.
        Mesh best;
        int best_res = 0;
        long long target = 0;
        bool use_target = false;
        if (op.contains("target") && op["target"].is_number()) {
            target = static_cast<long long>(op["target"].get<float>());
            use_target = target >= 8;
        }
        if (!use_target) {
            const int res = int(require_f(op, "resolution", 48.0f));
            if (!remesh_mesh(*mesh, best, res)) {
                return fail("remesh: could not extract a surface at resolution " +
                            std::to_string(res) + ". The input must be a closed surface.");
            }
            best_res = res;
        } else {
            int lo = 6, hi = 192;
            long long best_err = -1;
            for (int step = 0; step < 7 && lo <= hi; ++step) {
                const int mid = (lo + hi) / 2;
                Mesh probe;
                if (!remesh_mesh(*mesh, probe, mid)) break;
                best = std::move(probe);
                best_res = mid;
                const long long err = std::llabs(static_cast<long long>(best.faces.size()) -
                                                 target);
                if (best_err < 0 || err < best_err) best_err = err;
                if (best_err == 0) break;
                if (static_cast<long long>(best.faces.size()) < target) lo = mid + 1;
                else hi = mid - 1;
            }
            if (best.faces.empty()) {
                return fail("remesh: could not extract a surface. The input must be a "
                            "closed surface.");
            }
        }
        if (best.faces.size() < 4) return fail("remesh: extraction produced too few faces");
        // Saved before the move: the destination is the same object, so reading
        // the name afterwards would return the extracted mesh's empty name.
        const std::string keep_name = mesh->name;
        *mesh = std::move(best);
        mesh->name = keep_name;
        mesh_sync_render(*mesh);
        const MeshStats ms = mesh_stats(*mesh);
        return {true,
                "remeshed " + std::to_string(before) + " faces to " +
                    std::to_string(ms.faces) + " at resolution " + std::to_string(best_res) +
                    ", " + std::to_string(ms.verts) + " verts, " +
                    (ms.watertight() ? "watertight" : "NOT watertight: " +
                        std::to_string(ms.boundary_edges) + " boundary and " +
                        std::to_string(ms.nonmanifold_edges) + " non-manifold edges") +
                    ", volume " + std::to_string(ms.signed_volume),
                {{"faces_before", before},
                 {"faces_after", ms.faces},
                 {"verts", ms.verts},
                 {"boundary_edges", ms.boundary_edges},
                 {"nonmanifold_edges", ms.nonmanifold_edges},
                 {"degenerate_faces", ms.degenerate_faces},
                 {"non_finite_verts", ms.non_finite},
                 {"watertight", ms.watertight() && ms.non_finite == 0},
                 {"signed_volume", ms.signed_volume},
                 {"surface_area", ms.surface_area},
                 {"resolution", best_res}}};
    }

    if (kind == "decimate") {
        Mesh* mesh = s.selected_mesh();
        if (!mesh) return fail("decimate: no mesh");
        if (mesh->faces.size() < 4) return fail("decimate: mesh has fewer than 4 faces");
        size_t before_faces = mesh->faces.size();
        size_t before_verts = mesh->positions.size();
        float ratio = require_f(op, "ratio", 0.0f);
        if (!(ratio > 0.0f) || ratio >= 1.0f)
            return fail("decimate: ratio must be in (0, 1), got " + std::to_string(ratio));
        // `method` picks the algorithm. qem is the default: measured on a
        // watertight 528-triangle sphere it hits the requested ratio exactly,
        // keeps the surface closed with zero boundary and zero degenerate faces,
        // and drifts 6-20x less volume than clustering (-0.6% vs -13.2% at
        // ratio 0.5). Clustering cannot hit the target at all: it reached 33% of
        // 0.75 and 23% of 0.5.
        //
        // Both depend on a valid input. A mesh with holes or non-manifold edges
        // is refused rather than reduced into something that looks fine, because
        // a decimator run on a broken mesh returns confident nonsense.
        const std::string method = op.value("method", std::string("qem"));

        // Manifold precheck: every edge of a closed surface is used exactly
        // twice. Anything else means the input already has a hole.
        {
            std::map<std::pair<uint32_t, uint32_t>, int> counts;
            for (const Face& f : mesh->faces) {
                const size_t nv = f.v.size();
                for (size_t i = 0; i < nv; ++i) {
                    const uint32_t u = f.v[i], w = f.v[(i + 1) % nv];
                    if (u == w) continue;
                    counts[{std::min(u, w), std::max(u, w)}] += 1;
                }
            }
            int boundary = 0, over = 0;
            for (const auto& kv : counts) {
                if (kv.second == 1) ++boundary;
                else if (kv.second > 2) ++over;
            }
            if (boundary > 0 || over > 0) {
                return fail(
                    "decimate: the selected mesh is not a closed surface (" +
                    std::to_string(boundary) + " boundary edge(s), " +
                    std::to_string(over) + " non-manifold edge(s)). Fix the mesh "
                    "first; reducing a holed or non-manifold surface would give a "
                    "result that looks plausible and is not.");
            }
        }

        bool ok = false;
        if (method == "qem") {
            const int max_collapses =
                static_cast<int>(op.value("max_collapses", 20000));
            ok = decimate_qem(*mesh, ratio, max_collapses);
        } else if (method == "cluster") {
            ok = decimate_mesh(*mesh, ratio);
        } else {
            return fail("decimate: unknown method '" + method + "'; use qem or cluster");
        }
        if (!ok) {
            if (mutates) doc.undo_once();
            return fail("decimate: no valid collapse found at ratio " + std::to_string(ratio) +
                        " (method " + method + ")");
        }
        std::string msg = "decimated " + std::to_string(before_faces) + " faces to " +
                          std::to_string(mesh->faces.size()) + ", " +
                          std::to_string(before_verts) + " verts to " +
                          std::to_string(mesh->positions.size());
        return {true, msg, {{"faces_before", before_faces},
                            {"faces_after", mesh->faces.size()},
                            {"verts_before", before_verts},
                            {"verts_after", mesh->positions.size()},
                            {"ratio", ratio}}};
    }

    if (kind == "recalc_normals") {
        Mesh* mesh = s.selected_mesh();
        if (!mesh) return fail("recalc_normals: no mesh");
        if (!recalc_normals(*mesh)) {
            if (mutates) doc.undo_once();
            return fail("recalc_normals: mesh has no faces");
        }
        return ok("recalculated normals on " + std::to_string(mesh->positions.size()) + " verts");
    }

    if (kind == "separate_parts" || kind == "separate_loose") {
        Mesh* mesh = s.selected_mesh();
        if (!mesh) return fail("separate_parts: no mesh");
        int ni = s.selection;
        std::vector<Mesh> parts = separate_loose_meshes(*mesh);
        if (parts.size() < 2) return fail("separate_parts: mesh is already one connected piece");
        s.meshes[static_cast<size_t>(s.nodes[static_cast<size_t>(ni)].mesh)] = std::move(parts[0]);
        for (size_t i = 1; i < parts.size(); ++i) {
            Node child;
            child.name = s.unique_name(mesh->name + "_part" + std::to_string(i));
            child.type = NodeType::Mesh;
            child.parent = ni;
            child.material = s.nodes[static_cast<size_t>(ni)].material;
            child.translation = s.nodes[static_cast<size_t>(ni)].translation;
            child.rotation_deg = s.nodes[static_cast<size_t>(ni)].rotation_deg;
            child.scale = s.nodes[static_cast<size_t>(ni)].scale;
            child.mesh = static_cast<int>(s.meshes.size());
            s.meshes.push_back(std::move(parts[i]));
            s.add_node(child);
        }
        return ok("separated " + std::to_string(parts.size()) + " connected pieces");
    }

    if (kind == "import" || kind == "import_gltf" || kind == "import_fbx") {
        std::string path = require_str(op, "path");
        if (path.empty()) return fail("import needs 'path'");
        bool replace = op.value("replace", false);
        size_t nodes_before = s.nodes.size();
        size_t meshes_before = s.meshes.size();
        size_t materials_before = s.materials.size();
        GltfImportStats st;
        std::string err;
        // FBX and the other assimp formats go through assimp; glTF keeps its
        // own reader, which is kiln's native path and already tested.
        // assimp must not take over formats kiln already reads natively, or the
        // tested glTF path would be silently replaced by a second implementation.
        // assimp also cannot read a bare .gltf (it wants the .glb container), so
        // an explicit .gltf must stay on the native reader too.
        static const char* const kNative[] = {".gltf", ".glb"};
        bool native = false;
        for (const char* ext : kNative) {
            const std::string p = path;
            const std::size_t d = p.rfind('.');
            if (d != std::string::npos) {
                std::string e = p.substr(d);
                for (char& c : e) {
                    if (c >= 'A' && c <= 'Z') c = static_cast<char>(c - 'A' + 'a');
                }
                if (e == ext) native = true;
            }
        }
        const bool assimp_path = !native && ((kind == "import_fbx") || fbx_can_read(path));
        if (assimp_path) {
            FbxImportStats fst = import_fbx(s, path, err);
            if (fst.meshes == 0) {
                // Do not fall through to the glTF reader: assimp already said
                // why, and reporting a JSON error for a binary FBX file sent the
                // caller looking in entirely the wrong place.
                s.nodes.resize(nodes_before);
                s.meshes.resize(meshes_before);
                s.materials.resize(materials_before);
                return fail("import failed via assimp: " +
                            (err.empty() ? std::string("the file contains no mesh geometry")
                                         : err));
            }
            {
                if (replace) {
                    s.next_id = 1;
                    s.selection = -1;
                    s.active_camera = -1;
                }
                nlohmann::json d = nlohmann::json::object();
                d["nodes"] = fst.nodes;
                d["meshes"] = fst.meshes;
                d["materials"] = fst.materials;
                d["faces"] = fst.faces;
                d["vertices"] = fst.vertices;
                d["cameras"] = fst.cameras;
                d["lights"] = fst.lights;
                d["reader"] = "assimp";
                if (!fst.dropped_attributes.empty()) {
                    d["dropped_attributes"] = fst.dropped_attributes;
                }
                return {true, "imported " + std::to_string(fst.meshes) + " mesh(es), " +
                                 std::to_string(fst.vertices) + " vertices via assimp",
                        d};
            }
        }
        st = import_gltf(s, path, err);
        if (!err.empty()) {
            s.nodes.resize(nodes_before);
            s.meshes.resize(meshes_before);
            s.materials.resize(materials_before);
            return fail("import failed: " + err);
        }
        if (st.meshes == 0 && st.cameras == 0 && st.materials == 0) {
            s.nodes.resize(nodes_before);
            s.meshes.resize(meshes_before);
            s.materials.resize(materials_before);
            return fail("import read no geometry from " + path);
        }
        if (replace) {
            s.next_id = 1;
            s.selection = -1;
            s.active_camera = -1;
        }
        nlohmann::json data = nlohmann::json::object();
        data["nodes"] = st.nodes;
        data["meshes"] = st.meshes;
        data["primitives"] = st.primitives;
        data["materials"] = st.materials;
        data["faces"] = st.faces;
        data["vertices"] = st.vertices;
        data["cameras"] = st.cameras;
        data["skins"] = st.skins;
        data["bones"] = st.bones;
        data["skinned_vertices"] = st.skinned_vertices;
        data["skipped_modes"] = st.skipped_modes;
        if (!st.dropped_attributes.empty()) data["dropped_attributes"] = st.dropped_attributes;
        if (!st.unsupported_extensions.empty())
            data["unsupported_extensions"] = st.unsupported_extensions;
        std::string msg = "imported " + path + ": " + std::to_string(st.meshes) + " meshes, " +
                          std::to_string(st.faces) + " faces, " +
                          std::to_string(st.vertices) + " verts, " +
                          std::to_string(st.materials) + " materials";
        if (st.skins > 0)
            msg += ", " + std::to_string(st.skins) + " skins/" + std::to_string(st.bones) +
                   " bones";
        if (st.skipped_modes > 0)
            msg += ", " + std::to_string(st.skipped_modes) + " primitives skipped";
        if (!st.dropped_attributes.empty()) {
            msg += " (dropped:";
            for (const auto& a : st.dropped_attributes) msg += " " + a;
            msg += ")";
        }
        if (!st.unsupported_extensions.empty()) {
            msg += " (unhandled extensions:";
            for (const auto& e : st.unsupported_extensions) msg += " " + e;
            msg += ")";
        }
        return {true, msg, data};
    }

    if (mutates) {
        doc.undo.pop_back();
    }
    return fail("unknown op '" + kind + "'");
}

OpResult apply_ops(Document& doc, const nlohmann::json& ops) {
    if (ops.is_object()) return apply_op(doc, ops);
    if (!ops.is_array()) return fail("ops must be object or array");
    std::ostringstream msg;
    bool any = false;
    int applied = 0;
    for (const auto& op : ops) {
        OpResult r = apply_op(doc, op);
        if (!r.ok) {
            // A batch that got partway through has still changed the document.
            // Reporting only the failure made a caller throw away real work, so
            // say how far it got and carry the count in `data`.
            if (applied > 0) {
                OpResult partial;
                partial.ok = false;
                partial.message = std::to_string(applied) + " of " +
                                  std::to_string(ops.size()) +
                                  " ops applied, then failed: " + r.message +
                                  (any ? ("; applied: " + msg.str()) : std::string());
                partial.data = nlohmann::json::object();
                partial.data["partial"] = true;
                partial.data["applied"] = applied;
                partial.data["total"] = static_cast<int>(ops.size());
                return partial;
            }
            return r;
        }
        ++applied;
        if (any) msg << "; ";
        msg << r.message;
        any = true;
    }
    if (!any) return ok("no ops");
    return ok(msg.str());
}

}  // namespace kiln
