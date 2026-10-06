#include "scene.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <initializer_list>
#include <sstream>

namespace kiln {
namespace {

std::string lower(std::string s) {
    for (char& c : s) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    return s;
}

uint32_t add_pos(Mesh& m, Vec3 p) {
    m.positions.push_back(p);
    return static_cast<uint32_t>(m.positions.size() - 1);
}

void add_face(Mesh& m, std::initializer_list<uint32_t> vs) {
    m.faces.push_back({std::vector<uint32_t>(vs)});
}

Mesh finish(Mesh m) {
    mesh_sync_render(m);
    return m;
}

Mesh cube_mesh(Vec3 size) {
    Mesh m;
    m.name = "Cube";
    Vec3 h = size * 0.5f;
    add_pos(m, {-h.x, -h.y, -h.z});
    add_pos(m, {h.x, -h.y, -h.z});
    add_pos(m, {h.x, -h.y, h.z});
    add_pos(m, {-h.x, -h.y, h.z});
    add_pos(m, {-h.x, h.y, -h.z});
    add_pos(m, {h.x, h.y, -h.z});
    add_pos(m, {h.x, h.y, h.z});
    add_pos(m, {-h.x, h.y, h.z});
    add_face(m, {3, 2, 6, 7});
    add_face(m, {1, 0, 4, 5});
    add_face(m, {0, 3, 7, 4});
    add_face(m, {2, 1, 5, 6});
    add_face(m, {7, 6, 5, 4});
    add_face(m, {0, 1, 2, 3});
    return finish(m);
}

Mesh plane_mesh(Vec3 size) {
    Mesh m;
    m.name = "Plane";
    float hx = size.x * 0.5f, hz = size.z * 0.5f;
    add_pos(m, {-hx, 0, hz});
    add_pos(m, {hx, 0, hz});
    add_pos(m, {hx, 0, -hz});
    add_pos(m, {-hx, 0, -hz});
    add_face(m, {0, 1, 2, 3});
    return finish(m);
}

Mesh sphere_mesh(Vec3 size, int seg) {
    Mesh m;
    m.name = "Sphere";
    int slices = std::max(8, seg);
    int stacks = std::max(6, seg / 2);
    Vec3 r = size * 0.5f;
    auto idx = [&](int i, int j) -> uint32_t {
        if (j == 0) return 0;
        if (j == stacks) return 1;
        return 2 + uint32_t((j - 1) * slices + (i % slices));
    };
    add_pos(m, {0, r.y, 0});
    add_pos(m, {0, -r.y, 0});
    for (int j = 1; j < stacks; ++j) {
        float v = float(j) / stacks;
        float theta = v * 3.14159265f;
        for (int i = 0; i < slices; ++i) {
            float phi = (float(i) / slices) * 6.2831853f;
            Vec3 n{std::sin(theta) * std::cos(phi), std::cos(theta), std::sin(theta) * std::sin(phi)};
            add_pos(m, {n.x * r.x, n.y * r.y, n.z * r.z});
        }
    }
    for (int i = 0; i < slices; ++i) {
        add_face(m, {0, idx(i, 1), idx(i + 1, 1)});
        add_face(m, {idx(i, stacks - 1), 1, idx(i + 1, stacks - 1)});
    }
    for (int j = 1; j < stacks - 1; ++j) {
        for (int i = 0; i < slices; ++i) {
            add_face(m, {idx(i, j), idx(i, j + 1), idx(i + 1, j + 1), idx(i + 1, j)});
        }
    }
    return finish(m);
}

Mesh cylinder_mesh(Vec3 size, int seg) {
    Mesh m;
    m.name = "Cylinder";
    int n = std::max(8, seg);
    float hx = size.x * 0.5f, hy = size.y * 0.5f, hz = size.z * 0.5f;
    uint32_t top = add_pos(m, {0, hy, 0});
    uint32_t bot = add_pos(m, {0, -hy, 0});
    std::vector<uint32_t> ring0, ring1;
    for (int i = 0; i < n; ++i) {
        float a = (float(i) / n) * 6.2831853f;
        ring0.push_back(add_pos(m, {std::cos(a) * hx, -hy, std::sin(a) * hz}));
        ring1.push_back(add_pos(m, {std::cos(a) * hx, hy, std::sin(a) * hz}));
    }
    for (int i = 0; i < n; ++i) {
        int j = (i + 1) % n;
        add_face(m, {ring0[i], ring0[j], ring1[j], ring1[i]});
        add_face(m, {top, ring1[i], ring1[j]});
        add_face(m, {bot, ring0[j], ring0[i]});
    }
    return finish(m);
}

Mesh cone_mesh(Vec3 size, int seg) {
    Mesh m;
    m.name = "Cone";
    int n = std::max(8, seg);
    float hx = size.x * 0.5f, hy = size.y * 0.5f, hz = size.z * 0.5f;
    uint32_t tip = add_pos(m, {0, hy, 0});
    uint32_t bot = add_pos(m, {0, -hy, 0});
    std::vector<uint32_t> ring;
    for (int i = 0; i < n; ++i) {
        float a = (float(i) / n) * 6.2831853f;
        ring.push_back(add_pos(m, {std::cos(a) * hx, -hy, std::sin(a) * hz}));
    }
    for (int i = 0; i < n; ++i) {
        int j = (i + 1) % n;
        add_face(m, {ring[i], ring[j], tip});
        add_face(m, {bot, ring[j], ring[i]});
    }
    return finish(m);
}

Mesh torus_mesh(Vec3 size, int seg) {
    Mesh m;
    m.name = "Torus";
    int segs = std::max(8, seg);
    int tubes = std::max(8, seg / 2);
    float R = size.x * 0.35f;
    float r = size.y * 0.18f;
    auto idx = [&](int i, int j) -> uint32_t {
        return uint32_t((i % segs) * tubes + (j % tubes));
    };
    for (int i = 0; i < segs; ++i) {
        float a = (float(i) / segs) * 6.2831853f;
        for (int j = 0; j < tubes; ++j) {
            float b = (float(j) / tubes) * 6.2831853f;
            add_pos(m, {(R + r * std::cos(b)) * std::cos(a), r * std::sin(b), (R + r * std::cos(b)) * std::sin(a)});
        }
    }
    for (int i = 0; i < segs; ++i) {
        for (int j = 0; j < tubes; ++j) {
            add_face(m, {idx(i, j), idx(i + 1, j), idx(i + 1, j + 1), idx(i, j + 1)});
        }
    }
    return finish(m);
}

nlohmann::json vec3_json(Vec3 v) { return nlohmann::json::array({v.x, v.y, v.z}); }
Vec3 vec3_from(const nlohmann::json& j, Vec3 d = {}) {
    if (!j.is_array() || j.size() < 3) return d;
    return {j[0].get<float>(), j[1].get<float>(), j[2].get<float>()};
}

}  // namespace

Mesh make_primitive(PrimType type, Vec3 size, int segments) {
    switch (type) {
        case PrimType::Cube: return cube_mesh(size);
        case PrimType::Plane: return plane_mesh(size);
        case PrimType::Sphere: return sphere_mesh(size, segments);
        case PrimType::Cylinder: return cylinder_mesh(size, segments);
        case PrimType::Cone: return cone_mesh(size, segments);
        case PrimType::Torus: return torus_mesh(size, segments);
    }
    return cube_mesh(size);
}

const char* prim_name(PrimType t) {
    switch (t) {
        case PrimType::Cube: return "cube";
        case PrimType::Plane: return "plane";
        case PrimType::Sphere: return "sphere";
        case PrimType::Cylinder: return "cylinder";
        case PrimType::Cone: return "cone";
        case PrimType::Torus: return "torus";
    }
    return "cube";
}

bool parse_prim(const std::string& s, PrimType& out) {
    std::string k = lower(s);
    if (k == "cube" || k == "box") out = PrimType::Cube;
    else if (k == "plane" || k == "ground" || k == "floor" || k == "quad") out = PrimType::Plane;
    else if (k == "sphere" || k == "ball" || k == "ico") out = PrimType::Sphere;
    else if (k == "cylinder" || k == "tube") out = PrimType::Cylinder;
    else if (k == "cone") out = PrimType::Cone;
    else if (k == "torus" || k == "donut" || k == "doughnut") out = PrimType::Torus;
    else return false;
    return true;
}

const char* node_type_name(NodeType t) {
    switch (t) {
        case NodeType::Mesh: return "mesh";
        case NodeType::Camera: return "camera";
        case NodeType::Light: return "light";
        case NodeType::Empty: return "empty";
        case NodeType::Bone: return "bone";
    }
    return "empty";
}

int Scene::find_node(const std::string& name) const {
    std::string k = lower(name);
    for (int i = 0; i < static_cast<int>(nodes.size()); ++i) {
        if (lower(nodes[i].name) == k) return i;
    }
    return -1;
}

int Scene::find_node_id(uint32_t id) const {
    for (int i = 0; i < static_cast<int>(nodes.size()); ++i) {
        if (nodes[i].id == id) return i;
    }
    return -1;
}

Node* Scene::node_by_name(const std::string& name) {
    int i = find_node(name);
    return i < 0 ? nullptr : &nodes[i];
}
const Node* Scene::node_by_name(const std::string& name) const {
    int i = find_node(name);
    return i < 0 ? nullptr : &nodes[i];
}

std::string Scene::unique_name(std::string base) const {
    if (base.empty()) base = "Object";
    if (find_node(base) < 0) return base;
    for (int i = 1; i < 10000; ++i) {
        std::ostringstream os;
        os << base << "." << (i < 10 ? "00" : i < 100 ? "0" : "") << i;
        if (find_node(os.str()) < 0) return os.str();
    }
    return base + ".x";
}

int Scene::add_node(Node n) {
    if (n.id == 0) n.id = next_id++;
    else if (n.id >= next_id) next_id = n.id + 1;
    if (n.name.empty()) n.name = unique_name("Object");
    else n.name = unique_name(n.name);
    nodes.push_back(std::move(n));
    return static_cast<int>(nodes.size()) - 1;
}

bool Scene::remove_node_at(int index) {
    if (index < 0 || index >= static_cast<int>(nodes.size())) return false;
    uint32_t id = nodes[index].id;
    for (auto& n : nodes) {
        if (n.parent >= 0 && n.parent < static_cast<int>(nodes.size()) && nodes[n.parent].id == id)
            n.parent = -1;
    }
    nodes.erase(nodes.begin() + index);
    if (selection == index) {
        selection = -1;
        face_sel.clear();
        edge_sel.clear();
        vert_sel.clear();
        edit_mode = EditMode::Object;
    } else if (selection > index)
        --selection;
    if (active_camera == index) active_camera = -1;
    else if (active_camera > index) --active_camera;
    for (auto& n : nodes) {
        if (n.parent > index) --n.parent;
        else if (n.parent == index) n.parent = -1;
    }
    return true;
}

Mat4 Scene::rest_matrix(int index) const {
    if (index < 0 || index >= static_cast<int>(nodes.size())) return Mat4::identity();
    const Node& n = nodes[index];
    Mat4 local = trs(n.translation, n.rotation_deg, n.scale);
    if (n.parent >= 0 && n.parent != index) return mul(rest_matrix(n.parent), local);
    return local;
}

Mat4 Scene::world_matrix(int index) const {
    if (index < 0 || index >= static_cast<int>(nodes.size())) return Mat4::identity();
    const Node& n = nodes[index];
    Mat4 local = trs(n.translation, n.rotation_deg, n.scale);
    if (n.type == NodeType::Bone) local = mul(local, trs(n.pose_loc, n.pose_rot, {1, 1, 1}));
    if (n.parent >= 0 && n.parent != index) return mul(world_matrix(n.parent), local);
    return local;
}

nlohmann::json scene_to_json(const Scene& s) {
    nlohmann::json j;
    j["name"] = s.name;
    j["next_id"] = s.next_id;
    j["selection"] = s.selection;
    j["active_camera"] = s.active_camera;
    j["edit_mode"] = edit_mode_name(s.edit_mode);
    j["face_sel"] = s.face_sel;
    j["vert_sel"] = s.vert_sel;
    // Clips are serialized so a journal can replay a build that animated
    // something. They were not before, which meant a replay silently produced a
    // static scene even though every op had been recorded.
    j["frame"] = s.frame;
    j["active_clip"] = s.active_clip;
    j["graphs"] = graphs_to_json(s.graphs);
    j["clips"] = nlohmann::json::array();
    for (const Clip& c : s.clips) {
        nlohmann::json jc = nlohmann::json::object();
        jc["name"] = c.name;
        jc["fps"] = c.fps;
        jc["tracks"] = nlohmann::json::array();
        for (const Track& t : c.tracks) {
            nlohmann::json jt = nlohmann::json::object();
            jt["node"] = t.node;
            jt["channel"] = static_cast<int>(t.channel);
            jt["keys"] = nlohmann::json::array();
            for (const Keyframe& k : t.keys) {
                nlohmann::json jk = nlohmann::json::object();
                jk["frame"] = k.frame;
                jk["value"] = nlohmann::json::array({k.value.x, k.value.y, k.value.z});
                jk["interp"] = static_cast<int>(k.interp);
                jk["in_tan"] = nlohmann::json::array({k.in_tan.x, k.in_tan.y, k.in_tan.z});
                jk["out_tan"] =
                    nlohmann::json::array({k.out_tan.x, k.out_tan.y, k.out_tan.z});
                jt["keys"].push_back(std::move(jk));
            }
            jc["tracks"].push_back(std::move(jt));
        }
        j["clips"].push_back(std::move(jc));
    }
    j["edge_sel"] = nlohmann::json::array();
    for (auto e : s.edge_sel) {
        nlohmann::json je = nlohmann::json::array();
        je.push_back(e.a);
        je.push_back(e.b);
        j["edge_sel"].push_back(std::move(je));
    }
    j["materials"] = nlohmann::json::array();
    for (const auto& m : s.materials) {
        j["materials"].push_back({
            {"name", m.name},
            {"albedo", vec3_json(m.albedo)},
            {"roughness", m.roughness},
            {"metallic", m.metallic},
            {"emission", m.emission}
        });
    }
    j["meshes"] = nlohmann::json::array();
    for (const auto& mesh : s.meshes) {
        nlohmann::json jm = nlohmann::json::object();
        jm["name"] = mesh.name;
        jm["positions"] = nlohmann::json::array();
        for (const auto& p : mesh.positions) jm["positions"].push_back(vec3_json(p));
        jm["faces"] = nlohmann::json::array();
        for (const auto& f : mesh.faces) jm["faces"].push_back(f.v);
        jm["lod"] = mesh.lod;
        jm["joints"] = mesh.joints;
        if (!mesh.skin.empty()) {
            jm["skin"] = nlohmann::json::array();
            for (const auto& sv : mesh.skin) {
                nlohmann::json js = nlohmann::json::object();
                js["j"] = nlohmann::json::array({sv.j[0], sv.j[1], sv.j[2], sv.j[3]});
                js["w"] = nlohmann::json::array({sv.w[0], sv.w[1], sv.w[2], sv.w[3]});
                jm["skin"].push_back(std::move(js));
            }
        }
        if (!mesh.inv_bind.empty()) {
            jm["inv_bind"] = nlohmann::json::array();
            for (const auto& M : mesh.inv_bind) {
                nlohmann::json a = nlohmann::json::array();
                for (int i = 0; i < 16; ++i) a.push_back(M.m[i]);
                jm["inv_bind"].push_back(std::move(a));
            }
        }
        jm["shapes"] = nlohmann::json::array();
        for (const auto& sh : mesh.shapes) {
            nlohmann::json js = nlohmann::json::object();
            js["name"] = sh.name;
            js["value"] = sh.value;
            js["delta"] = nlohmann::json::array();
            for (const auto& d : sh.delta) js["delta"].push_back(vec3_json(d));
            jm["shapes"].push_back(std::move(js));
        }
        j["meshes"].push_back(std::move(jm));
    }
    j["nodes"] = nlohmann::json::array();
    for (const auto& n : s.nodes) {
        j["nodes"].push_back({
            {"id", n.id},
            {"name", n.name},
            {"type", node_type_name(n.type)},
            {"translation", vec3_json(n.translation)},
            {"rotation_deg", vec3_json(n.rotation_deg)},
            {"scale", vec3_json(n.scale)},
            {"parent", n.parent},
            {"mesh", n.mesh},
            {"material", n.material},
            {"fov_deg", n.fov_deg},
            {"light", n.light == LightType::Point ? "point" : "sun"},
            {"light_color", vec3_json(n.light_color)},
            {"light_intensity", n.light_intensity},
            {"pose_rot", vec3_json(n.pose_rot)},
            {"pose_loc", vec3_json(n.pose_loc)},
            {"bone_length", n.bone_length}
        });
    }
    return j;
}

bool scene_from_json(const nlohmann::json& j, Scene& s) {
    s = Scene{};
    s.name = j.value("name", "untitled");
    s.next_id = j.value("next_id", 1u);
    s.selection = j.value("selection", -1);
    s.active_camera = j.value("active_camera", -1);
    {
        std::string em = j.value("edit_mode", "object");
        s.edit_mode = em == "face" ? EditMode::Face : em == "edge" ? EditMode::Edge
                       : (em == "vert" || em == "vertex" || em == "verts") ? EditMode::Vert
                       : EditMode::Object;
    }
    if (j.contains("face_sel") && j["face_sel"].is_array()) s.face_sel = j["face_sel"].get<std::vector<int>>();
    if (j.contains("vert_sel") && j["vert_sel"].is_array()) s.vert_sel = j["vert_sel"].get<std::vector<uint32_t>>();
    if (j.contains("edge_sel") && j["edge_sel"].is_array()) {
        for (const auto& je : j["edge_sel"]) {
            if (je.is_array() && je.size() >= 2) s.edge_sel.push_back(make_edge(je[0].get<uint32_t>(), je[1].get<uint32_t>()));
        }
    }
    if (j.contains("materials")) {
        for (const auto& jm : j["materials"]) {
            Material m;
            m.name = jm.value("name", "Material");
            m.albedo = vec3_from(jm["albedo"], m.albedo);
            m.roughness = jm.value("roughness", 0.45f);
            m.metallic = jm.value("metallic", 0.0f);
            m.emission = jm.value("emission", 0.0f);
            s.materials.push_back(m);
        }
    }
    if (j.contains("meshes")) {
        for (const auto& jm : j["meshes"]) {
            Mesh mesh;
            mesh.name = jm.value("name", "Mesh");
            if (jm.contains("positions")) {
                for (const auto& p : jm["positions"]) mesh.positions.push_back(vec3_from(p));
            }
            if (jm.contains("faces")) {
                for (const auto& f : jm["faces"]) {
                    Face face;
                    if (f.is_array()) face.v = f.get<std::vector<uint32_t>>();
                    mesh.faces.push_back(std::move(face));
                }
            }
            if (jm.contains("vertices")) {
                for (const auto& v : jm["vertices"]) {
                    Vertex vx;
                    vx.p = vec3_from(v["p"]);
                    vx.n = vec3_from(v["n"], {0, 1, 0});
                    if (v.contains("uv") && v["uv"].is_array() && v["uv"].size() >= 2)
                        vx.uv = {v["uv"][0].get<float>(), v["uv"][1].get<float>()};
                    mesh.vertices.push_back(vx);
                }
            }
            if (jm.contains("indices")) mesh.indices = jm["indices"].get<std::vector<uint32_t>>();
            mesh.lod = jm.value("lod", 0);
            if (jm.contains("joints") && jm["joints"].is_array()) mesh.joints = jm["joints"].get<std::vector<int>>();
            if (jm.contains("skin") && jm["skin"].is_array()) {
                for (const auto& js : jm["skin"]) {
                    SkinVert sv;
                    if (js.contains("j") && js["j"].is_array())
                        for (int i = 0; i < 4 && i < int(js["j"].size()); ++i) sv.j[i] = js["j"][i].get<uint16_t>();
                    if (js.contains("w") && js["w"].is_array())
                        for (int i = 0; i < 4 && i < int(js["w"].size()); ++i) sv.w[i] = js["w"][i].get<float>();
                    mesh.skin.push_back(sv);
                }
            }
            if (jm.contains("inv_bind") && jm["inv_bind"].is_array()) {
                for (const auto& a : jm["inv_bind"]) {
                    Mat4 M = Mat4::identity();
                    if (a.is_array())
                        for (int i = 0; i < 16 && i < int(a.size()); ++i) M.m[i] = a[i].get<float>();
                    mesh.inv_bind.push_back(M);
                }
            }
            if (jm.contains("shapes") && jm["shapes"].is_array()) {
                for (const auto& js : jm["shapes"]) {
                    ShapeKey sh;
                    sh.name = js.value("name", "key");
                    sh.value = js.value("value", 0.0f);
                    if (js.contains("delta"))
                        for (const auto& d : js["delta"]) sh.delta.push_back(vec3_from(d));
                    mesh.shapes.push_back(std::move(sh));
                }
            }
            mesh_ensure_faces(mesh);
            s.meshes.push_back(std::move(mesh));
        }
    }
    graphs_from_json(j.contains("graphs") ? j["graphs"] : nlohmann::json::array(),
                       s.graphs);
    // Read clips back. Without this a replay of a journal that contained a clip
    // produced a static scene.
    if (j.contains("clips") && j["clips"].is_array()) {
        s.clips.clear();
        for (const auto& jc : j["clips"]) {
            Clip c;
            c.name = jc.value("name", "Clip");
            c.fps = jc.value("fps", 24.0f);
            if (!(c.fps > 0.0f)) c.fps = 24.0f;
            if (jc.contains("tracks") && jc["tracks"].is_array()) {
                for (const auto& jt : jc["tracks"]) {
                    Track t;
                    t.node = jt.value("node", "");
                    t.channel = static_cast<Channel>(jt.value("channel", 0));
                    if (jt.contains("keys") && jt["keys"].is_array()) {
                        for (const auto& jk : jt["keys"]) {
                            Keyframe k;
                            k.frame = jk.value("frame", 1);
                            if (jk.contains("value")) k.value = vec3_from(jk["value"]);
                            k.interp = static_cast<Interp>(jk.value("interp", 1));
                            if (jk.contains("in_tan")) k.in_tan = vec3_from(jk["in_tan"]);
                            if (jk.contains("out_tan")) k.out_tan = vec3_from(jk["out_tan"]);
                            t.keys.push_back(k);
                        }
                    }
                    if (!t.node.empty() && !t.keys.empty()) c.tracks.push_back(std::move(t));
                }
            }
            c.sort_keys();
            s.clips.push_back(std::move(c));
        }
        s.active_clip = j.value("active_clip", s.clips.empty() ? -1 : 0);
        if (s.active_clip >= static_cast<int>(s.clips.size())) {
            s.active_clip = s.clips.empty() ? -1 : 0;
        }
    }
    if (j.contains("nodes")) {
        for (const auto& jn : j["nodes"]) {
            Node n;
            n.id = jn.value("id", 0u);
            n.name = jn.value("name", "Object");
            std::string t = jn.value("type", "empty");
            if (t == "mesh") n.type = NodeType::Mesh;
            else if (t == "camera") n.type = NodeType::Camera;
            else if (t == "light") n.type = NodeType::Light;
            else if (t == "bone") n.type = NodeType::Bone;
            else n.type = NodeType::Empty;
            n.translation = vec3_from(jn["translation"]);
            n.rotation_deg = vec3_from(jn["rotation_deg"]);
            n.scale = vec3_from(jn["scale"], {1, 1, 1});
            n.parent = jn.value("parent", -1);
            n.mesh = jn.value("mesh", -1);
            n.material = jn.value("material", -1);
            n.fov_deg = jn.value("fov_deg", 50.0f);
            n.light = jn.value("light", "sun") == "point" ? LightType::Point : LightType::Sun;
            n.light_color = vec3_from(jn["light_color"], n.light_color);
            n.light_intensity = jn.value("light_intensity", 1.0f);
            n.pose_rot = vec3_from(jn.contains("pose_rot") ? jn["pose_rot"] : nlohmann::json{}, {});
            n.pose_loc = vec3_from(jn.contains("pose_loc") ? jn["pose_loc"] : nlohmann::json{}, {});
            n.bone_length = jn.value("bone_length", 0.2f);
            s.nodes.push_back(n);
        }
    }
    return true;
}

}  // namespace kiln
