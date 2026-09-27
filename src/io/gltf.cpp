#include "gltf.hpp"

#include "../core/preview.hpp"

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <sstream>

namespace kiln {
namespace {

void append_f32(std::vector<uint8_t>& b, float v) {
    uint8_t tmp[4];
    std::memcpy(tmp, &v, 4);
    b.insert(b.end(), tmp, tmp + 4);
}
void append_u16(std::vector<uint8_t>& b, uint16_t v) {
    b.push_back(uint8_t(v));
    b.push_back(uint8_t(v >> 8));
}
void append_u32(std::vector<uint8_t>& b, uint32_t v) {
    b.push_back(uint8_t(v));
    b.push_back(uint8_t(v >> 8));
    b.push_back(uint8_t(v >> 16));
    b.push_back(uint8_t(v >> 24));
}

void pad4(std::vector<uint8_t>& b, uint8_t fill = 0x20) {
    while (b.size() % 4) b.push_back(fill);
}

bool ends_with(const std::string& s, const std::string& e) {
    return s.size() >= e.size() && s.compare(s.size() - e.size(), e.size(), e) == 0;
}

// Append one preview PNG to a finished GLB buffer as the standard glTF image
// chain: bufferView -> image -> sampler + texture -> material.
//
// It is referenced through a second material rather than the mesh's own, so a
// viewer that ignores it renders exactly the model kiln wrote before, and a
// viewer that shows it has the thumbnail without needing a vendor extension.
void append_preview_image(nlohmann::json& g, std::vector<uint8_t>& bin, const GlbPreview& preview) {
    if (!g.contains("bufferViews")) g["bufferViews"] = nlohmann::json::array();
    if (!g.contains("images")) g["images"] = nlohmann::json::array();
    if (!g.contains("samplers")) g["samplers"] = nlohmann::json::array();
    if (!g.contains("textures")) g["textures"] = nlohmann::json::array();
    if (!g.contains("materials")) g["materials"] = nlohmann::json::array();

    pad4(bin, 0);
    uint32_t off = static_cast<uint32_t>(bin.size());
    bin.insert(bin.end(), preview.png.begin(), preview.png.end());
    pad4(bin, 0);

    nlohmann::json bv = nlohmann::json::object();
    bv["buffer"] = 0;
    bv["byteOffset"] = off;
    bv["byteLength"] = preview.png.size();  // the PNG only; padding is not part of it
    int bv_index = static_cast<int>(g["bufferViews"].size());
    g["bufferViews"].push_back(std::move(bv));

    nlohmann::json img = nlohmann::json::object();
    img["bufferView"] = bv_index;
    img["mimeType"] = "image/png";
    img["name"] = "kiln preview";
    int img_index = static_cast<int>(g["images"].size());
    g["images"].push_back(std::move(img));

    nlohmann::json smp = nlohmann::json::object();
    smp["magFilter"] = 9729;  // LINEAR
    smp["minFilter"] = 9987;  // LINEAR_MIPMAP_LINEAR
    smp["wrapS"] = 33071;     // CLAMP_TO_EDGE
    smp["wrapT"] = 33071;
    int smp_index = static_cast<int>(g["samplers"].size());
    g["samplers"].push_back(std::move(smp));

    nlohmann::json tex = nlohmann::json::object();
    tex["sampler"] = smp_index;
    tex["source"] = img_index;
    tex["name"] = "kiln preview";
    int tex_index = static_cast<int>(g["textures"].size());
    g["textures"].push_back(std::move(tex));

    nlohmann::json mat = nlohmann::json::object();
    mat["name"] = "kiln preview";
    nlohmann::json pbr = nlohmann::json::object();
    pbr["baseColorTexture"] = nlohmann::json::object({{"index", tex_index}});
    pbr["metallicFactor"] = 0.0;
    pbr["roughnessFactor"] = 1.0;
    mat["pbrMetallicRoughness"] = std::move(pbr);
    mat["doubleSided"] = true;

    // Non-standard but explicitly legal (glTF 2.0 permits extra properties) and
    // ignored by viewers that do not know them. Recorded here because kiln's own
    // exporter does not write the info.thumbnail extension.
    nlohmann::json ex = nlohmann::json::object();
    ex["kilnPreview"] = nlohmann::json::object({{"width", preview.w}, {"height", preview.h}});
    mat["extras"] = std::move(ex);

    g["materials"].push_back(std::move(mat));
}

}  // namespace

bool export_gltf(const Scene& scene, const std::string& path, const GlbPreview& preview,
                 std::string& err) {
  try {
    nlohmann::json g;
    g["asset"] = nlohmann::json::object();
    g["asset"]["version"] = "2.0";
    g["asset"]["generator"] = "kiln";
    // glTF stores animation times in seconds and says nothing about the rate they
    // were authored at, so a 30fps clip reimported at the default 24fps came back
    // with its keys on the wrong frames. The rate is carried in asset extras, which
    // is the spec's designated place for tool-specific data.
    if (!scene.clips.empty()) {
        g["asset"]["extras"] = nlohmann::json::object();
        const float rate = scene.clips.front().fps > 0.0f ? scene.clips.front().fps : 24.0f;
        g["asset"]["extras"]["kiln_fps"] = rate;
    }
    g["scene"] = 0;
    nlohmann::json sc = nlohmann::json::object();
    sc["name"] = scene.name;
    g["skins"] = nlohmann::json::array();
    sc["nodes"] = nlohmann::json::array();
    g["scenes"] = nlohmann::json::array({sc});

    std::vector<uint8_t> bin;
    g["buffers"] = nlohmann::json::array();
    g["bufferViews"] = nlohmann::json::array();
    g["accessors"] = nlohmann::json::array();
    g["meshes"] = nlohmann::json::array();
    g["materials"] = nlohmann::json::array();
    g["nodes"] = nlohmann::json::array();
    g["cameras"] = nlohmann::json::array();

    for (const auto& mat : scene.materials) {
        nlohmann::json pbr = nlohmann::json::object();
        pbr["baseColorFactor"] = nlohmann::json::array({mat.albedo.x, mat.albedo.y, mat.albedo.z, 1.0});
        pbr["metallicFactor"] = mat.metallic;
        pbr["roughnessFactor"] = mat.roughness;
        nlohmann::json jm = nlohmann::json::object();
        jm["name"] = mat.name;
        jm["pbrMetallicRoughness"] = pbr;
        jm["emissiveFactor"] = nlohmann::json::array(
            {mat.albedo.x * mat.emission, mat.albedo.y * mat.emission, mat.albedo.z * mat.emission});
        g["materials"].push_back(std::move(jm));
    }

    std::vector<int> mesh_gltf(scene.meshes.size(), -1);

    // Hoisted out of the mesh loop so the animation pass can build samplers with
    // the same accessors, rather than duplicating the schema here.
    auto buffer_view = [](uint32_t off, size_t len, int target) {
        nlohmann::json bv = nlohmann::json::object();
        bv["buffer"] = 0;
        bv["byteOffset"] = off;
        bv["byteLength"] = len;
        bv["target"] = target;
        return bv;
    };
    auto accessor = [](int view, int ctype, size_t count, const char* type) {
        nlohmann::json ac = nlohmann::json::object();
        ac["bufferView"] = view;
        ac["componentType"] = ctype;
        ac["count"] = count;
        ac["type"] = type;
        return ac;
    };

    // glTF skin index per mesh, -1 when the mesh is not skinned.
    std::vector<int> mesh_skin_gltf(scene.meshes.size(), -1);
    for (size_t mi = 0; mi < scene.meshes.size(); ++mi) {
        Mesh mesh = scene.meshes[mi];
        int ni = -1;
        for (int i = 0; i < int(scene.nodes.size()); ++i)
            if (scene.nodes[i].mesh == int(mi)) {
                ni = i;
                break;
            }
        if (ni >= 0) {
            std::vector<Vec3> def;
            deform_positions(scene, ni, def);
            if (def.size() == mesh.positions.size()) {
                mesh.positions = std::move(def);
                mesh_sync_render(mesh);
            }
        }
        if (mesh.indices.empty() || mesh.vertices.empty()) continue;

        uint32_t pos_off = static_cast<uint32_t>(bin.size());
        Vec3 bmin = mesh.vertices[0].p, bmax = mesh.vertices[0].p;
        for (const auto& v : mesh.vertices) {
            append_f32(bin, v.p.x);
            append_f32(bin, v.p.y);
            append_f32(bin, v.p.z);
            bmin.x = std::min(bmin.x, v.p.x);
            bmin.y = std::min(bmin.y, v.p.y);
            bmin.z = std::min(bmin.z, v.p.z);
            bmax.x = std::max(bmax.x, v.p.x);
            bmax.y = std::max(bmax.y, v.p.y);
            bmax.z = std::max(bmax.z, v.p.z);
        }
        uint32_t nrm_off = static_cast<uint32_t>(bin.size());
        for (const auto& v : mesh.vertices) {
            append_f32(bin, v.n.x);
            append_f32(bin, v.n.y);
            append_f32(bin, v.n.z);
        }
        uint32_t uv_off = static_cast<uint32_t>(bin.size());
        if (mesh.has_uv) {
            for (const auto& v : mesh.vertices) {
                append_f32(bin, v.uv.x);
                append_f32(bin, v.uv.y);
            }
            pad4(bin, 0);
        }
        // Skinning attributes. Written only when the mesh is actually skinned and
        // the weight count matches the render vertices, because a JOINTS_0 set of
        // the wrong length makes the file invalid rather than merely wrong.
        const bool skinned = !mesh.skin.empty() && mesh.skin.size() == mesh.vertices.size();
        uint32_t joints_off = 0, weights_off = 0;
        if (skinned) {
            joints_off = static_cast<uint32_t>(bin.size());
            for (const SkinVert& sv : mesh.skin) {
                for (int k = 0; k < 4; ++k) append_u16(bin, sv.j[k]);
            }
            pad4(bin, 0);
            weights_off = static_cast<uint32_t>(bin.size());
            for (const SkinVert& sv : mesh.skin) {
                for (int k = 0; k < 4; ++k) append_f32(bin, sv.w[k]);
            }
            pad4(bin, 0);
        }
        uint32_t idx_off = static_cast<uint32_t>(bin.size());
        bool u32 = mesh.vertices.size() > 65535;
        for (uint32_t ix : mesh.indices) {
            if (u32) append_u32(bin, ix);
            else append_u16(bin, static_cast<uint16_t>(ix));
        }
        pad4(bin, 0);


        int bv_pos = static_cast<int>(g["bufferViews"].size());
        g["bufferViews"].push_back(buffer_view(pos_off, mesh.vertices.size() * 12, 34962));
        int bv_nrm = static_cast<int>(g["bufferViews"].size());
        g["bufferViews"].push_back(buffer_view(nrm_off, mesh.vertices.size() * 12, 34962));
        int bv_uv = -1;
        if (mesh.has_uv) {
            bv_uv = static_cast<int>(g["bufferViews"].size());
            g["bufferViews"].push_back(buffer_view(uv_off, mesh.vertices.size() * 8, 34962));
        }
        int bv_joints = -1, bv_weights = -1;
        if (skinned) {
            bv_joints = static_cast<int>(g["bufferViews"].size());
            g["bufferViews"].push_back(
                buffer_view(joints_off, mesh.skin.size() * 8, 34962));
            bv_weights = static_cast<int>(g["bufferViews"].size());
            g["bufferViews"].push_back(
                buffer_view(weights_off, mesh.skin.size() * 16, 34962));
        }
        int bv_idx = static_cast<int>(g["bufferViews"].size());
        g["bufferViews"].push_back(buffer_view(idx_off, mesh.indices.size() * (u32 ? 4 : 2), 34963));

        int ac_pos = static_cast<int>(g["accessors"].size());
        auto acp = accessor(bv_pos, 5126, mesh.vertices.size(), "VEC3");
        acp["min"] = nlohmann::json::array({bmin.x, bmin.y, bmin.z});
        acp["max"] = nlohmann::json::array({bmax.x, bmax.y, bmax.z});
        g["accessors"].push_back(std::move(acp));
        int ac_nrm = static_cast<int>(g["accessors"].size());
        g["accessors"].push_back(accessor(bv_nrm, 5126, mesh.vertices.size(), "VEC3"));
        int ac_uv = -1;
        if (mesh.has_uv) {
            ac_uv = static_cast<int>(g["accessors"].size());
            g["accessors"].push_back(accessor(bv_uv, 5126, mesh.vertices.size(), "VEC2"));
        }
        int ac_joints = -1, ac_weights = -1;
        if (skinned) {
            ac_joints = static_cast<int>(g["accessors"].size());
            g["accessors"].push_back(accessor(bv_joints, 5123, mesh.skin.size(), "VEC4"));
            ac_weights = static_cast<int>(g["accessors"].size());
            g["accessors"].push_back(accessor(bv_weights, 5126, mesh.skin.size(), "VEC4"));
        }
        int ac_idx = static_cast<int>(g["accessors"].size());
        g["accessors"].push_back(accessor(bv_idx, u32 ? 5125 : 5123, mesh.indices.size(), "SCALAR"));

        nlohmann::json attrs = nlohmann::json::object();
        attrs["POSITION"] = ac_pos;
        attrs["NORMAL"] = ac_nrm;
        // Only advertise a UV set when the mesh really has one. Writing a
        // placeholder attribute here is how files ended up claiming uvs=y while
        // carrying the per-triangle ramp from mesh_sync_render.
        if (ac_uv >= 0) attrs["TEXCOORD_0"] = ac_uv;
        if (ac_joints >= 0) {
            attrs["JOINTS_0"] = ac_joints;
            attrs["WEIGHTS_0"] = ac_weights;
        }
        nlohmann::json prim = nlohmann::json::object();
        prim["attributes"] = attrs;
        prim["indices"] = ac_idx;
        if (ac_joints >= 0) {
            // inverseBindMatrices, one mat4 per joint, in mesh joint order.
            const size_t nj = mesh.inv_bind.size();
            const size_t ibm_off = static_cast<size_t>(bin.size());
            for (size_t b = 0; b < nj; ++b) {
                const Mat4& m = mesh.inv_bind[b];
                for (int c = 0; c < 16; ++c) append_f32(bin, m.m[c]);
            }
            pad4(bin, 0);
            const int bv_ibm = static_cast<int>(g["bufferViews"].size());
            g["bufferViews"].push_back(buffer_view(
                static_cast<uint32_t>(ibm_off), nj * 64, 34962));
            const int ac_ibm = static_cast<int>(g["accessors"].size());
            g["accessors"].push_back(accessor(bv_ibm, 5126, nj, "MAT4"));
            mesh_skin_gltf[mi] = static_cast<int>(g["skins"].size());
            nlohmann::json sk = nlohmann::json::object();
            sk["name"] = mesh.name + "_skin";
            sk["inverseBindMatrices"] = ac_ibm;
            sk["joints"] = nlohmann::json::array();
            g["skins"].push_back(std::move(sk));
        }
        mesh_gltf[mi] = static_cast<int>(g["meshes"].size());
        nlohmann::json jm = nlohmann::json::object();
        jm["name"] = mesh.name;
        jm["primitives"] = nlohmann::json::array({prim});
        g["meshes"].push_back(std::move(jm));
    }

    std::vector<int> node_index(scene.nodes.size(), -1);
    for (size_t i = 0; i < scene.nodes.size(); ++i) {
        const Node& n = scene.nodes[i];
        nlohmann::json jn = nlohmann::json::object();
        jn["name"] = n.name;
        jn["translation"] = nlohmann::json::array({n.translation.x, n.translation.y, n.translation.z});
        jn["scale"] = nlohmann::json::array({n.scale.x, n.scale.y, n.scale.z});
        // glTF rotation is quaternion; convert XYZ euler
        float sx = std::sin(deg2rad(n.rotation_deg.x) * 0.5f);
        float cx = std::cos(deg2rad(n.rotation_deg.x) * 0.5f);
        float sy = std::sin(deg2rad(n.rotation_deg.y) * 0.5f);
        float cy = std::cos(deg2rad(n.rotation_deg.y) * 0.5f);
        float sz = std::sin(deg2rad(n.rotation_deg.z) * 0.5f);
        float cz = std::cos(deg2rad(n.rotation_deg.z) * 0.5f);
        float qx = sx * cy * cz - cx * sy * sz;
        float qy = cx * sy * cz + sx * cy * sz;
        float qz = cx * cy * sz - sx * sy * cz;
        float qw = cx * cy * cz + sx * sy * sz;
        jn["rotation"] = nlohmann::json::array({qx, qy, qz, qw});
        if (n.type == NodeType::Mesh && n.mesh >= 0 && n.mesh < static_cast<int>(mesh_gltf.size()) &&
            mesh_gltf[n.mesh] >= 0) {
            jn["mesh"] = mesh_gltf[n.mesh];
            if (n.material >= 0 && n.material < static_cast<int>(scene.materials.size())) {
                int gi = mesh_gltf[n.mesh];
                g["meshes"].at(gi)["primitives"].at(0)["material"] = n.material;
            }
            if (mesh_skin_gltf[n.mesh] >= 0) {
                jn["skin"] = mesh_skin_gltf[n.mesh];
            }
        }
        if (n.type == NodeType::Camera) {
            int ci = static_cast<int>(g["cameras"].size());
            nlohmann::json persp = nlohmann::json::object();
            persp["yfov"] = deg2rad(n.fov_deg);
            persp["znear"] = 0.05;
            persp["zfar"] = 500.0;
            nlohmann::json cam = nlohmann::json::object();
            cam["type"] = "perspective";
            cam["perspective"] = persp;
            g["cameras"].push_back(std::move(cam));
            jn["camera"] = ci;
        }
        node_index[i] = static_cast<int>(g["nodes"].size());
        g["nodes"].push_back(std::move(jn));
        g["scenes"][0]["nodes"].push_back(node_index[i]);
    }

    // -- animations ---------------------------------------------------------
    //
    // Written as glTF animation channels, one sampler per track. Times go in
    // seconds (frame / fps) because that is what glTF specifies, and rotation
    // tracks become quaternions because that is the only rotation form glTF
    // accepts. CUBICSPLINE is used for bezier tracks, whose tangents are stored
    // here as value-per-frame and have to be scaled by the time delta.
    if (!scene.clips.empty()) {
        g["animations"] = nlohmann::json::array();
        auto euler_deg_to_quat = [](Vec3 d, float out[4]) {
            const float hx = deg2rad(d.x) * 0.5f, hy = deg2rad(d.y) * 0.5f,
                        hz = deg2rad(d.z) * 0.5f;
            const float sx = std::sin(hx), cx = std::cos(hx);
            const float sy = std::sin(hy), cy = std::cos(hy);
            const float sz = std::sin(hz), cz = std::cos(hz);
            out[0] = sx * cy * cz - cx * sy * sz;
            out[1] = cx * sy * cz + sx * cy * sz;
            out[2] = cx * sy * sz - sx * cy * cz;
            out[3] = cx * cy * cz + sx * sy * sz;
        };
        for (const Clip& clip : scene.clips) {
            const float fps = clip.fps > 0.0f ? clip.fps : 24.0f;
            nlohmann::json janim = nlohmann::json::object();
            janim["name"] = clip.name;
            // Per-animation rate, so a file with clips at different rates survives.
            janim["extras"] = nlohmann::json::object();
            janim["extras"]["kiln_fps"] = fps;
            janim["samplers"] = nlohmann::json::array();
            janim["channels"] = nlohmann::json::array();
            int written = 0;
            for (const Track& t : clip.tracks) {
                if (t.keys.size() < 2) continue;  // glTF needs at least two keys
                // Track names are node names; resolve to a scene node index.
                int ni = -1;
                for (size_t q = 0; q < scene.nodes.size(); ++q) {
                    if (scene.nodes[q].name == t.node) {
                        ni = static_cast<int>(q);
                        break;
                    }
                }
                if (ni < 0 || node_index[static_cast<size_t>(ni)] < 0) continue;
                const bool cubic = t.keys.front().interp == Interp::Bezier ||
                                   t.keys.back().interp == Interp::Bezier;
                const bool is_rot = t.channel == Channel::Rotation;
                const int comps = is_rot ? 4 : 3;

                // input: key times in seconds, required to be strictly increasing
                const uint32_t t_off = static_cast<uint32_t>(bin.size());
                for (const Keyframe& k : t.keys) append_f32(bin, float(k.frame) / fps);
                pad4(bin, 0);
                const int bv_t = static_cast<int>(g["bufferViews"].size());
                g["bufferViews"].push_back(buffer_view(
                    t_off, t.keys.size() * 4, 34962));
                const int ac_t = static_cast<int>(g["accessors"].size());
                nlohmann::json acc_t = nlohmann::json::object();
                acc_t["bufferView"] = bv_t;
                acc_t["componentType"] = 5126;
                acc_t["count"] = t.keys.size();
                acc_t["type"] = "SCALAR";
                // glTF requires min/max on animation input samplers.
                acc_t["min"] = nlohmann::json::array({float(t.keys.front().frame) / fps});
                acc_t["max"] = nlohmann::json::array({float(t.keys.back().frame) / fps});
                g["accessors"].push_back(std::move(acc_t));

                // output: value, or in/value/out for CUBICSPLINE
                const uint32_t o_off = static_cast<uint32_t>(bin.size());
                for (std::size_t i = 0; i < t.keys.size(); ++i) {
                    const Keyframe& k = t.keys[i];
                    auto push = [&](Vec3 v, const Vec3& tan) {
                        if (is_rot) {
                            float q[4];
                            euler_deg_to_quat(v, q);
                            for (int c = 0; c < 4; ++c) append_f32(bin, q[c]);
                        } else {
                            append_f32(bin, v.x);
                            append_f32(bin, v.y);
                            append_f32(bin, v.z);
                        }
                        if (cubic) {
                            const float dt = float(i + 1 < t.keys.size()
                                                       ? t.keys[i + 1].frame - k.frame
                                                       : k.frame - t.keys[i - 1].frame);
                            if (is_rot) {
                                float qt[4];
                                euler_deg_to_quat(tan * dt, qt);
                                for (int c = 0; c < 4; ++c) append_f32(bin, qt[c]);
                            } else {
                                append_f32(bin, tan.x * dt);
                                append_f32(bin, tan.y * dt);
                                append_f32(bin, tan.z * dt);
                            }
                        }
                    };
                    if (cubic) {
                        const Vec3 in_t = k.in_tan;
                        const Vec3 out_t = k.out_tan;
                        push(in_t, Vec3{0, 0, 0});
                        push(k.value, Vec3{0, 0, 0});
                        push(out_t, Vec3{0, 0, 0});
                    } else {
                        push(k.value, Vec3{0, 0, 0});
                    }
                }
                pad4(bin, 0);
                const size_t olen = t.keys.size() * (cubic ? 3u : 1u) * comps * 4u;
                const int bv_o = static_cast<int>(g["bufferViews"].size());
                g["bufferViews"].push_back(buffer_view(o_off, olen, 34962));
                const int ac_o = static_cast<int>(g["accessors"].size());
                g["accessors"].push_back(
                    accessor(bv_o, 5126, t.keys.size() * (cubic ? 3u : 1u),
                             is_rot ? "VEC4" : "VEC3"));

                nlohmann::json smp = nlohmann::json::object();
                smp["input"] = ac_t;
                smp["output"] = ac_o;
                smp["interpolation"] = cubic ? "CUBICSPLINE" : (t.keys.front().interp ==
                                                               Interp::Step ? "STEP"
                                                                            : "LINEAR");
                const int si = static_cast<int>(janim["samplers"].size());
                janim["samplers"].push_back(std::move(smp));
                nlohmann::json ch = nlohmann::json::object();
                ch["sampler"] = si;
                ch["target"] = nlohmann::json::object();
                ch["target"]["node"] = node_index[static_cast<size_t>(ni)];
                ch["target"]["path"] = is_rot ? "rotation"
                                              : (t.channel == Channel::Scale ? "scale"
                                                                          : "translation");
                janim["channels"].push_back(std::move(ch));
                ++written;
            }
            if (written == 0) continue;  // do not emit an animation with no channels
            g["animations"].push_back(std::move(janim));
        }
        if (g["animations"].empty()) g["animations"] = nlohmann::json::array();
    }

    // Joints can only be resolved once every node has an index, so the skin
    // joint lists are filled here rather than during the mesh pass.
    for (size_t i = 0; i < scene.meshes.size(); ++i) {
        const int si = mesh_skin_gltf[i];
        if (si < 0) continue;
        const Mesh& m = scene.meshes[i];
        nlohmann::json joints = nlohmann::json::array();
        for (const int j : m.joints) {
            if (j >= 0 && j < static_cast<int>(scene.nodes.size()) &&
                node_index[j] >= 0) {
                joints.push_back(node_index[j]);
            }
        }
        g["skins"][si]["joints"] = std::move(joints);
    }

    // The scene itself carries the supersampling factor the run rendered with,
    // so a file found later says how its preview was drawn. Absent unless a
    // preview was asked for, which keeps the no-preview path byte-identical.
    if (preview.enabled) {
        nlohmann::json sx = nlohmann::json::object();
        sx["samples"] = preview.samples;
        g["scenes"][0]["extras"] = std::move(sx);
    }

    for (size_t i = 0; i < scene.nodes.size(); ++i) {
        int p = scene.nodes[i].parent;
        if (p >= 0 && p < static_cast<int>(scene.nodes.size())) {
            if (!g["nodes"][node_index[p]].contains("children"))
                g["nodes"][node_index[p]]["children"] = nlohmann::json::array();
            g["nodes"][node_index[p]]["children"].push_back(node_index[i]);
        }
    }

    // Only a GLB has a binary chunk a plain bufferView image can live in; the
    // separate .gltf + .bin pair the branch below writes is left untouched.
    bool glb_target = ends_with(path, ".glb") || ends_with(path, ".GLB");
    if (preview.enabled && glb_target) append_preview_image(g, bin, preview);

    nlohmann::json buf = nlohmann::json::object();
    buf["byteLength"] = bin.size();
    g["buffers"] = nlohmann::json::array({buf});

    bool glb = glb_target;
    if (!glb) {
        std::string bin_path = path;
        auto dot = bin_path.find_last_of('.');
        if (dot != std::string::npos) bin_path = bin_path.substr(0, dot);
        bin_path += ".bin";
        auto slash = bin_path.find_last_of('/');
        std::string uri = slash == std::string::npos ? bin_path : bin_path.substr(slash + 1);
        g["buffers"][0]["uri"] = uri;
        std::ofstream bf(bin_path, std::ios::binary);
        if (!bf) {
            err = "cannot write " + bin_path;
            return false;
        }
        bf.write(reinterpret_cast<const char*>(bin.data()), static_cast<std::streamsize>(bin.size()));
        std::ofstream jf(path);
        if (!jf) {
            err = "cannot write " + path;
            return false;
        }
        jf << g.dump(2);
        return true;
    }

    std::string json = g.dump();
    std::vector<uint8_t> json_bytes(json.begin(), json.end());
    pad4(json_bytes, 0x20);
    pad4(bin, 0);

    std::vector<uint8_t> out;
    uint32_t total = 12 + 8 + static_cast<uint32_t>(json_bytes.size()) + 8 + static_cast<uint32_t>(bin.size());
    append_u32(out, 0x46546C67);
    append_u32(out, 2);
    append_u32(out, total);
    append_u32(out, static_cast<uint32_t>(json_bytes.size()));
    append_u32(out, 0x4E4F534A);
    out.insert(out.end(), json_bytes.begin(), json_bytes.end());
    append_u32(out, static_cast<uint32_t>(bin.size()));
    append_u32(out, 0x004E4942);
    out.insert(out.end(), bin.begin(), bin.end());

    std::ofstream f(path, std::ios::binary);
    if (!f) {
        err = "cannot write " + path;
        return false;
    }
    f.write(reinterpret_cast<const char*>(out.data()), static_cast<std::streamsize>(out.size()));
    return true;
  } catch (const std::exception& e) {
    err = std::string("gltf: ") + e.what();
    return false;
  }
}

bool export_gltf(const Scene& scene, const std::string& path, std::string& err) {
    // The historical call, kept so every existing caller keeps its behaviour.
    return export_gltf(scene, path, GlbPreview{}, err);
}

bool save_scene(const Scene& scene, const std::string& path, std::string& err) {
    std::ofstream f(path);
    if (!f) {
        err = "cannot write " + path;
        return false;
    }
    f << scene_to_json(scene).dump(2);
    return true;
}

bool load_scene(Scene& scene, const std::string& path, std::string& err) {
    std::ifstream f(path);
    if (!f) {
        err = "cannot read " + path;
        return false;
    }
    std::ostringstream ss;
    ss << f.rdbuf();
    try {
        auto j = nlohmann::json::parse(ss.str());
        return scene_from_json(j, scene);
    } catch (const std::exception& e) {
        err = e.what();
        return false;
    }
}

}  // namespace kiln
