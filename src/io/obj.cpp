#include "gltf.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <set>
#include <cstdio>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

namespace kiln {
namespace {

struct ObjIndex {
    int v = 0;
    int vt = 0;
    int vn = 0;
    bool operator==(const ObjIndex& o) const { return v == o.v && vt == o.vt && vn == o.vn; }
};

bool parse_face_token(const std::string& tok, ObjIndex& out) {
    int parts[3] = {0, 0, 0};
    int field = 0;
    size_t i = 0;
    while (i <= tok.size() && field < 3) {
        size_t start = i;
        while (i < tok.size() && tok[i] != '/') ++i;
        if (i > start) {
            std::string num = tok.substr(start, i - start);
            char* endp = nullptr;
            long val = std::strtol(num.c_str(), &endp, 10);
            if (endp == num.c_str()) return false;
            parts[field] = static_cast<int>(val);
        }
        ++field;
        if (i < tok.size() && tok[i] == '/') ++i;
        else break;
    }
    out.v = parts[0];
    out.vt = parts[1];
    out.vn = parts[2];
    return out.v != 0;
}

int resolve(int raw, size_t count) {
    if (raw > 0) return raw - 1;
    if (raw < 0) return static_cast<int>(count) + raw;
    return -1;
}

std::string trim(const std::string& s) {
    size_t a = s.find_first_not_of(" \t\r\n");
    if (a == std::string::npos) return "";
    size_t b = s.find_last_not_of(" \t\r\n");
    return s.substr(a, b - a + 1);
}

}  // namespace

bool import_obj(Scene& scene, const std::string& path, std::string& err) {
    ObjImportNotes notes;
    return import_obj(scene, path, err, notes);
}

bool import_obj(Scene& scene, const std::string& path, std::string& err,
                ObjImportNotes& notes) {
    notes = ObjImportNotes{};
    std::ifstream f(path);
    if (!f) {
        err = "cannot read " + path;
        return false;
    }

    std::vector<Vec3> positions;
    std::vector<Vec3> normals;
    std::vector<Vec2> uvs;
    struct RawFace {
        std::vector<ObjIndex> idx;
        std::string object;
    };
    std::vector<RawFace> faces;
    std::string current_object;
    std::string line;
    size_t line_no = 0;
    bool saw_normal = false;
    bool saw_uv = false;

    while (std::getline(f, line)) {
        ++line_no;
        if (!line.empty() && line.back() == '\r') line.pop_back();
        if (line.empty() || line[0] == '#') continue;
        std::istringstream is(line);
        std::string tag;
        is >> tag;
        if (tag == "v") {
            float x = 0, y = 0, z = 0, w = 1;
            if (!(is >> x >> y >> z)) {
                err = path + ":" + std::to_string(line_no) + ": malformed vertex";
                return false;
            }
            is >> w;
            if (w != 0.0f && std::isfinite(w) && std::abs(w - 1.0f) > 1e-6f) {
                x /= w;
                y /= w;
                z /= w;
            }
            positions.push_back({x, y, z});
        } else if (tag == "vn") {
            float x = 0, y = 0, z = 0;
            if (is >> x >> y >> z) {
                normals.push_back(normalize(Vec3{x, y, z}));
                saw_normal = true;
            }
        } else if (tag == "vt") {
            float u = 0, v = 0;
            if (is >> u >> v) {
                uvs.push_back(Vec2{u, v});
                saw_uv = true;
            }
        } else if (tag == "f") {
            std::vector<ObjIndex> poly;
            std::string tok;
            while (is >> tok) {
                ObjIndex oi;
                if (!parse_face_token(tok, oi)) {
                    err = path + ":" + std::to_string(line_no) + ": malformed face index '" + tok + "'";
                    return false;
                }
                poly.push_back(oi);
            }
            if (poly.size() >= 3) faces.push_back(RawFace{std::move(poly), current_object});
        } else if (tag == "o" || tag == "g") {
            std::string rest;
            std::getline(is, rest);
            current_object = trim(rest);
        }
    }

    if (positions.empty() || faces.empty()) {
        err = path + " has no faces";
        return false;
    }

    struct Group {
        std::string name;
        std::vector<std::vector<uint32_t>> faces;
    };
    std::vector<Group> groups;
    size_t cur = 0;
    for (size_t i = 0; i < faces.size(); ++i) {
        const std::string& want = faces[i].object.empty() ? std::string("mesh") : faces[i].object;
        if (groups.empty() || groups.back().name != want) {
            groups.push_back(Group{want, {}});
            cur = groups.size() - 1;
        }
        std::vector<uint32_t> tri;
        const std::vector<ObjIndex>& poly = faces[i].idx;
        for (size_t k = 1; k + 1 < poly.size(); ++k) {
            int a = resolve(poly[0].v, positions.size());
            int b = resolve(poly[k].v, positions.size());
            int c = resolve(poly[k + 1].v, positions.size());
            if (a < 0 || b < 0 || c < 0 || a >= static_cast<int>(positions.size()) ||
                b >= static_cast<int>(positions.size()) || c >= static_cast<int>(positions.size())) {
                err = path + ": face references vertex " +
                      std::to_string(std::max(poly[0].v, std::max(poly[k].v, poly[k + 1].v))) +
                      " but only " + std::to_string(positions.size()) + " vertices exist";
                return false;
            }
            tri.push_back(static_cast<uint32_t>(a));
            tri.push_back(static_cast<uint32_t>(b));
            tri.push_back(static_cast<uint32_t>(c));
        }
        if (tri.empty()) continue;
        for (size_t t = 0; t + 2 < tri.size(); t += 3)
            groups[cur].faces.push_back({tri[t], tri[t + 1], tri[t + 2]});
    }
    if (groups.empty() || groups[0].faces.empty()) {
        err = path + " has no usable faces";
        return false;
    }

    Material mat;
    mat.name = current_object.empty() ? "Material" : current_object;
    int mat_index = static_cast<int>(scene.materials.size());
    scene.materials.push_back(mat);

    int added = 0;
    for (const Group& g : groups) {
        if (g.faces.empty()) continue;
        Mesh mesh;
        mesh.name = g.name;
        mesh.faces.reserve(g.faces.size());
        for (const auto& t : g.faces) mesh.faces.push_back(Face{t});
        mesh.smooth = saw_normal;
        std::set<uint32_t> used;
        for (const auto& t : g.faces)
            for (uint32_t v : t) used.insert(v);
        for (uint32_t v : used) mesh.positions.push_back(positions[v]);
        std::vector<uint32_t> remap(positions.size(), 0);
        {
            uint32_t next = 0;
            for (uint32_t v : used) remap[v] = next++;
        }
        for (auto& fc : mesh.faces)
            for (uint32_t& v : fc.v) v = remap[v];
        mesh_sync_render(mesh);
        Node node;
        node.name = scene.unique_name(g.name);
        node.type = NodeType::Mesh;
        node.mesh = static_cast<int>(scene.meshes.size());
        node.material = mat_index;
        scene.meshes.push_back(std::move(mesh));
        scene.add_node(node);
        added++;
    }
    if (added == 0) {
        err = path + " produced no meshes";
        return false;
    }
    notes.had_uvs = saw_uv;
    notes.had_normals = saw_normal;
    notes.meshes = static_cast<size_t>(added);
    notes.faces = 0;
    notes.vertices = 0;
    for (const Group& g : groups) {
        if (g.faces.empty()) continue;
        notes.faces += g.faces.size();
        std::set<uint32_t> used;
        for (const auto& t : g.faces)
            for (uint32_t v : t) used.insert(v);
        notes.vertices += used.size();
    }
    return true;
}

bool export_obj(const Scene& scene, const std::string& path, std::string& err) {
    std::ofstream f(path);
    if (!f) {
        err = "cannot write " + path;
        return false;
    }
    f << "# exported by kiln\n";
    int base = 0;
    size_t written = 0;
    for (const Node& n : scene.nodes) {
        if (n.type != NodeType::Mesh || n.mesh < 0 || n.mesh >= static_cast<int>(scene.meshes.size()))
            continue;
        const Mesh& m = scene.meshes[static_cast<size_t>(n.mesh)];
        if (m.positions.empty()) continue;
        f << "o " << n.name << "\n";
        if (n.material >= 0 && n.material < static_cast<int>(scene.materials.size()))
            f << "usemtl " << scene.materials[static_cast<size_t>(n.material)].name << "\n";
        for (const Vec3& p : m.positions)
            f << "v " << p.x << " " << p.y << " " << p.z << "\n";
        for (const Vertex& vtx : m.vertices)
            f << "vn " << vtx.n.x << " " << vtx.n.y << " " << vtx.n.z << "\n";
        for (const Face& fc : m.faces) {
            if (fc.v.size() < 3) continue;
            f << "f";
            for (uint32_t v : fc.v) f << " " << (base + static_cast<int>(v) + 1) << "//";
            f << "\n";
        }
        base += static_cast<int>(m.positions.size());
        written++;
    }
    if (written == 0) {
        err = "scene has no meshes to write";
        return false;
    }
    f.flush();
    if (!f) {
        err = "write failed for " + path;
        return false;
    }
    return true;
}

}  // namespace kiln
