#include "scene.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>

namespace kiln {
namespace {

int bone(Scene& s, const std::string& name, int parent, Vec3 local, Vec3 rest, float length) {
    Node n;
    n.type = NodeType::Bone;
    n.name = name;
    n.translation = local;
    n.rotation_deg = rest;
    n.parent = parent;
    n.bone_length = length;
    return s.add_node(std::move(n));
}

}  // namespace

int add_bone(Scene& s, const std::string& name, int parent, Vec3 local, Vec3 rest_rot, float length) {
    return bone(s, name, parent, local, rest_rot, length);
}

int rig_biped(Scene& s, bool apose) {
    float arm_z = apose ? 40.0f : 0.0f;
    int hips = bone(s, "Hips", -1, {0, 0.95f, 0}, {0, 0, 0}, 0.18f);
    int spine = bone(s, "Spine", hips, {0, 0.16f, 0}, {0, 0, 0}, 0.16f);
    int chest = bone(s, "Chest", spine, {0, 0.18f, 0}, {0, 0, 0}, 0.16f);
    int neck = bone(s, "Neck", chest, {0, 0.20f, 0}, {0, 0, 0}, 0.08f);
    bone(s, "Head", neck, {0, 0.12f, 0}, {0, 0, 0}, 0.14f);
    int sl = bone(s, "Shoulder.L", chest, {0.08f, 0.16f, 0}, {0, 0, 0}, 0.08f);
    int sr = bone(s, "Shoulder.R", chest, {-0.08f, 0.16f, 0}, {0, 0, 0}, 0.08f);
    int ual = bone(s, "UpperArm.L", sl, {0.12f, 0, 0}, {0, 0, arm_z}, 0.26f);
    int uar = bone(s, "UpperArm.R", sr, {-0.12f, 0, 0}, {0, 0, -arm_z}, 0.26f);
    int fal = bone(s, "Forearm.L", ual, {0.26f, 0, 0}, {0, 0, 0}, 0.24f);
    int far = bone(s, "Forearm.R", uar, {-0.26f, 0, 0}, {0, 0, 0}, 0.24f);
    bone(s, "Hand.L", fal, {0.24f, 0, 0}, {0, 0, 0}, 0.10f);
    bone(s, "Hand.R", far, {-0.24f, 0, 0}, {0, 0, 0}, 0.10f);
    int ull = bone(s, "UpperLeg.L", hips, {0.10f, -0.04f, 0}, {0, 0, 0}, 0.40f);
    int ulr = bone(s, "UpperLeg.R", hips, {-0.10f, -0.04f, 0}, {0, 0, 0}, 0.40f);
    int lll = bone(s, "LowerLeg.L", ull, {0, -0.40f, 0}, {0, 0, 0}, 0.38f);
    int llr = bone(s, "LowerLeg.R", ulr, {0, -0.40f, 0}, {0, 0, 0}, 0.38f);
    bone(s, "Foot.L", lll, {0, -0.38f, 0.06f}, {0, 0, 0}, 0.14f);
    bone(s, "Foot.R", llr, {0, -0.38f, 0.06f}, {0, 0, 0}, 0.14f);
    s.selection = hips;
    return hips;
}

int rig_quadruped(Scene& s) {
    int hips = bone(s, "Hips", -1, {0, 0.55f, -0.25f}, {0, 0, 0}, 0.16f);
    int chest = bone(s, "Chest", hips, {0, 0.08f, 0.40f}, {0, 0, 0}, 0.18f);
    int neck = bone(s, "Neck", chest, {0, 0.10f, 0.22f}, {0, 0, 0}, 0.16f);
    bone(s, "Head", neck, {0, 0.06f, 0.16f}, {0, 0, 0}, 0.14f);
    int tail0 = bone(s, "Tail", hips, {0, 0.04f, -0.16f}, {0, 0, 0}, 0.14f);
    bone(s, "TailTip", tail0, {0, -0.02f, -0.16f}, {0, 0, 0}, 0.12f);
    auto leg = [&](const char* side, int parent, Vec3 at) {
        std::string S = side;
        int u = bone(s, "UpperLeg." + S, parent, at, {0, 0, 0}, 0.22f);
        int l = bone(s, "LowerLeg." + S, u, {0, -0.22f, 0}, {0, 0, 0}, 0.20f);
        bone(s, "Foot." + S, l, {0, -0.20f, 0.04f}, {0, 0, 0}, 0.08f);
    };
    leg("FL", chest, {0.10f, -0.06f, 0.08f});
    leg("FR", chest, {-0.10f, -0.06f, 0.08f});
    leg("BL", hips, {0.10f, -0.04f, 0.02f});
    leg("BR", hips, {-0.10f, -0.04f, 0.02f});
    s.selection = hips;
    return hips;
}

int rig_face(Scene& s) {
    int head = s.find_node("Head");
    Vec3 at = head >= 0 ? Vec3{0, 0.10f, 0.10f} : Vec3{0, 1.55f, 0.08f};
    int root = bone(s, "Face", head, at, {0, 0, 0}, 0.08f);
    bone(s, "Jaw", root, {0, -0.06f, 0.02f}, {0, 0, 0}, 0.06f);
    bone(s, "Eye.L", root, {0.035f, 0.03f, 0.06f}, {0, 0, 0}, 0.02f);
    bone(s, "Eye.R", root, {-0.035f, 0.03f, 0.06f}, {0, 0, 0}, 0.02f);
    bone(s, "Brow.L", root, {0.04f, 0.055f, 0.055f}, {0, 0, 0}, 0.02f);
    bone(s, "Brow.R", root, {-0.04f, 0.055f, 0.055f}, {0, 0, 0}, 0.02f);
    bone(s, "Mouth.L", root, {0.03f, -0.02f, 0.06f}, {0, 0, 0}, 0.02f);
    bone(s, "Mouth.R", root, {-0.03f, -0.02f, 0.06f}, {0, 0, 0}, 0.02f);
    bone(s, "Cheek.L", root, {0.05f, 0.0f, 0.04f}, {0, 0, 0}, 0.02f);
    bone(s, "Cheek.R", root, {-0.05f, 0.0f, 0.04f}, {0, 0, 0}, 0.02f);
    bone(s, "Nose", root, {0, 0.01f, 0.08f}, {0, 0, 0}, 0.03f);
    Mesh* mesh = s.selected_mesh();
    if (!mesh) {
        for (auto& n : s.nodes)
            if (n.type == NodeType::Mesh && n.mesh >= 0 && n.mesh < int(s.meshes.size())) {
                mesh = &s.meshes[n.mesh];
                break;
            }
    }
    if (mesh && !mesh->positions.empty()) {
        auto add_shape = [&](const std::string& name, auto fn) {
            ShapeKey sh;
            sh.name = name;
            sh.delta.resize(mesh->positions.size());
            Vec3 c = mesh_centroid(*mesh);
            for (size_t i = 0; i < mesh->positions.size(); ++i) sh.delta[i] = fn(mesh->positions[i], c);
            mesh->shapes.push_back(std::move(sh));
        };
        add_shape("smile", [](Vec3 p, Vec3 c) {
            float f = std::max(0.0f, p.z - c.z) * std::max(0.0f, 0.15f - std::abs(p.y - c.y));
            return Vec3{(p.x - c.x) * 0.4f, 0.35f, 0.1f} * f * 8;
        });
        add_shape("frown", [](Vec3 p, Vec3 c) {
            float f = std::max(0.0f, p.z - c.z) * std::max(0.0f, 0.15f - std::abs(p.y - c.y));
            return Vec3{0, -0.4f, 0} * f * 8;
        });
        add_shape("jaw_open", [](Vec3 p, Vec3 c) {
            float f = std::max(0.0f, c.y - p.y) * std::max(0.0f, p.z - c.z + 0.1f);
            return Vec3{0, -0.8f, 0.2f} * f * 4;
        });
        add_shape("blink.L", [](Vec3 p, Vec3 c) {
            float f = std::max(0.0f, p.x - c.x) * std::max(0.0f, p.y - c.y) * std::max(0.0f, p.z - c.z);
            return Vec3{0, -1.2f, 0} * f * 10;
        });
        add_shape("blink.R", [](Vec3 p, Vec3 c) {
            float f = std::max(0.0f, c.x - p.x) * std::max(0.0f, p.y - c.y) * std::max(0.0f, p.z - c.z);
            return Vec3{0, -1.2f, 0} * f * 10;
        });
        add_shape("brows_up", [](Vec3 p, Vec3 c) {
            float f = std::max(0.0f, p.y - c.y) * std::max(0.0f, p.z - c.z);
            return Vec3{0, 0.6f, 0} * f * 6;
        });
    }
    s.selection = root;
    return root;
}

bool bind_mesh(Scene& s, int mesh_node) {
    if (mesh_node < 0 || mesh_node >= int(s.nodes.size())) return false;
    Node& n = s.nodes[mesh_node];
    if (n.type != NodeType::Mesh || n.mesh < 0) return false;
    Mesh& mesh = s.meshes[n.mesh];
    mesh.joints.clear();
    for (int i = 0; i < int(s.nodes.size()); ++i)
        if (s.nodes[i].type == NodeType::Bone) mesh.joints.push_back(i);
    if (mesh.joints.empty()) return false;
    mesh.inv_bind.resize(mesh.joints.size());
    std::vector<Vec3> jpos(mesh.joints.size());
    for (size_t i = 0; i < mesh.joints.size(); ++i) {
        mesh.inv_bind[i] = invert_affine(s.rest_matrix(mesh.joints[i]));
        Mat4 W = s.rest_matrix(mesh.joints[i]);
        jpos[i] = transform_point(W, {0, 0, 0});
    }
    Mat4 MW = s.world_matrix(mesh_node);
    mesh.skin.assign(mesh.positions.size(), {});
    for (size_t vi = 0; vi < mesh.positions.size(); ++vi) {
        Vec3 p = transform_point(MW, mesh.positions[vi]);
        float d[4] = {1e9f, 1e9f, 1e9f, 1e9f};
        int id[4] = {0, 0, 0, 0};
        for (size_t j = 0; j < mesh.joints.size(); ++j) {
            float dist = length(p - jpos[j]);
            for (int k = 0; k < 4; ++k)
                if (dist < d[k]) {
                    for (int u = 3; u > k; --u) {
                        d[u] = d[u - 1];
                        id[u] = id[u - 1];
                    }
                    d[k] = dist;
                    id[k] = int(j);
                    break;
                }
        }
        float wsum = 0;
        SkinVert sv;
        for (int k = 0; k < 4; ++k) {
            sv.j[k] = uint16_t(id[k]);
            float w = 1.0f / std::max(1e-4f, d[k] * d[k]);
            sv.w[k] = w;
            wsum += w;
        }
        if (wsum > 0)
            for (int k = 0; k < 4; ++k) sv.w[k] /= wsum;
        mesh.skin[vi] = sv;
    }
    return true;
}

void deform_positions(const Scene& s, int mesh_node, std::vector<Vec3>& out) {
    if (mesh_node < 0 || mesh_node >= int(s.nodes.size())) return;
    int mi = s.nodes[mesh_node].mesh;
    if (mi < 0 || mi >= int(s.meshes.size())) return;
    const Mesh& mesh = s.meshes[mi];
    const std::vector<Vec3>* src = &mesh.positions;
    std::vector<Vec3> lodpos;
    if (mesh.lod > 0 && mesh.lod <= int(mesh.lods.size())) {
        lodpos = mesh.lods[mesh.lod - 1].positions;
        src = &lodpos;
    }
    out = *src;
    for (const auto& sh : mesh.shapes) {
        if (std::abs(sh.value) < 1e-6f) continue;
        size_t n = std::min(out.size(), sh.delta.size());
        for (size_t i = 0; i < n; ++i) out[i] += sh.delta[i] * sh.value;
    }
    if (mesh.skin.size() != src->size() || mesh.joints.empty() || mesh.inv_bind.size() != mesh.joints.size())
        return;
    Mat4 MW = s.world_matrix(mesh_node);
    Mat4 invM = invert_affine(MW);
    std::vector<Mat4> pal(mesh.joints.size());
    for (size_t j = 0; j < mesh.joints.size(); ++j)
        pal[j] = mul(s.world_matrix(mesh.joints[j]), mesh.inv_bind[j]);
    for (size_t i = 0; i < out.size(); ++i) {
        Vec3 p = transform_point(MW, out[i]);
        const SkinVert& sv = mesh.skin[i];
        Vec3 acc{0, 0, 0};
        for (int k = 0; k < 4; ++k) {
            if (sv.w[k] < 1e-6f) continue;
            uint16_t ji = sv.j[k];
            if (ji >= pal.size()) continue;
            acc += transform_point(pal[ji], p) * sv.w[k];
        }
        out[i] = transform_point(invM, acc);
    }
}

bool build_lods(Mesh& m, int levels) {
    mesh_ensure_faces(m);
    if (m.faces.empty()) return false;
    if (levels < 1) levels = 1;
    if (levels > 4) levels = 4;
    m.lods.clear();
    Mesh cur = m;
    cur.lods.clear();
    cur.skin.clear();
    cur.shapes.clear();
    cur.joints.clear();
    float dist = 0.02f;
    for (int i = 0; i < levels; ++i) {
        weld_distance(cur, dist);
        LodLevel lv;
        lv.positions = cur.positions;
        lv.faces = cur.faces;
        m.lods.push_back(std::move(lv));
        dist *= 2.2f;
    }
    return !m.lods.empty();
}

nlohmann::json describe_scene(const Scene& s) {
    nlohmann::json j = nlohmann::json::object();
    j["selection"] = s.selection >= 0 && s.selection < int(s.nodes.size()) ? s.nodes[s.selection].name : "";
    j["edit_mode"] = edit_mode_name(s.edit_mode);
    j["nodes"] = nlohmann::json::array();
    int bones = 0;
    for (int i = 0; i < int(s.nodes.size()); ++i) {
        const Node& n = s.nodes[i];
        if (n.type == NodeType::Bone) ++bones;
        nlohmann::json jn = nlohmann::json::object();
        jn["name"] = n.name;
        jn["type"] = node_type_name(n.type);
        jn["t"] = nlohmann::json::array({n.translation.x, n.translation.y, n.translation.z});
        if (n.type == NodeType::Bone) {
            jn["pose"] = nlohmann::json::array({n.pose_rot.x, n.pose_rot.y, n.pose_rot.z});
            jn["parent"] = n.parent >= 0 && n.parent < int(s.nodes.size()) ? s.nodes[n.parent].name : "";
        }
        // Parentage for every node, not just bones: a text-only agent needs to
        // trace a part back to whatever it belongs to.
        if (n.type != NodeType::Bone) {
            jn["parent"] = n.parent >= 0 && n.parent < int(s.nodes.size()) ? s.nodes[n.parent].name : "";
        }
        jn["scale"] = nlohmann::json::array({n.scale.x, n.scale.y, n.scale.z});
        if (n.type == NodeType::Mesh && n.mesh >= 0 && n.mesh < int(s.meshes.size())) {
            const Mesh& m = s.meshes[n.mesh];
            jn["faces"] = int(m.faces.size());
            jn["verts"] = int(m.positions.size());
            jn["skinned"] = !m.skin.empty();
            jn["lod"] = m.lod;
            jn["lods"] = int(m.lods.size());
            // Measured bounds, so a caller can answer "how big is this part"
            // without re-deriving them from the transform chain.
            bool have = false;
            float lo[3] = {0, 0, 0}, hi[3] = {0, 0, 0};
            for (const Vec3& p : m.positions) {
                const float c[3] = {p.x, p.y, p.z};
                if (!have) {
                    for (int k = 0; k < 3; ++k) lo[k] = hi[k] = c[k];
                    have = true;
                } else {
                    for (int k = 0; k < 3; ++k) {
                        if (c[k] < lo[k]) lo[k] = c[k];
                        if (c[k] > hi[k]) hi[k] = c[k];
                    }
                }
            }
            jn["bounds_measured"] = have;
            if (have) {
                jn["local_min"] = nlohmann::json::array({lo[0], lo[1], lo[2]});
                jn["local_max"] = nlohmann::json::array({hi[0], hi[1], hi[2]});
            }
            // Material, so colour and finish are available to a caller that
            // cannot see them.
            if (n.material >= 0 && n.material < int(s.materials.size())) {
                const Material& mat = s.materials[n.material];
                jn["material"] = mat.name;
                jn["albedo"] = nlohmann::json::array({mat.albedo.x, mat.albedo.y, mat.albedo.z});
                jn["roughness"] = mat.roughness;
                jn["metallic"] = mat.metallic;
            }
            jn["shapes"] = nlohmann::json::array();
            for (const auto& sh : m.shapes) {
                nlohmann::json js = nlohmann::json::object();
                js["name"] = sh.name;
                js["value"] = sh.value;
                jn["shapes"].push_back(std::move(js));
            }
        }
        j["nodes"].push_back(std::move(jn));
    }
    j["bone_count"] = bones;
    return j;
}

}  // namespace kiln
