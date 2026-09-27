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
    g["scene"] = 0;
    nlohmann::json sc = nlohmann::json::object();
    sc["name"] = scene.name;
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
        uint32_t idx_off = static_cast<uint32_t>(bin.size());
        bool u32 = mesh.vertices.size() > 65535;
        for (uint32_t ix : mesh.indices) {
            if (u32) append_u32(bin, ix);
            else append_u16(bin, static_cast<uint16_t>(ix));
        }
        pad4(bin, 0);

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

        int bv_pos = static_cast<int>(g["bufferViews"].size());
        g["bufferViews"].push_back(buffer_view(pos_off, mesh.vertices.size() * 12, 34962));
        int bv_nrm = static_cast<int>(g["bufferViews"].size());
        g["bufferViews"].push_back(buffer_view(nrm_off, mesh.vertices.size() * 12, 34962));
        int bv_uv = -1;
        if (mesh.has_uv) {
            bv_uv = static_cast<int>(g["bufferViews"].size());
            g["bufferViews"].push_back(buffer_view(uv_off, mesh.vertices.size() * 8, 34962));
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
        int ac_idx = static_cast<int>(g["accessors"].size());
        g["accessors"].push_back(accessor(bv_idx, u32 ? 5125 : 5123, mesh.indices.size(), "SCALAR"));

        nlohmann::json attrs = nlohmann::json::object();
        attrs["POSITION"] = ac_pos;
        attrs["NORMAL"] = ac_nrm;
        // Only advertise a UV set when the mesh really has one. Writing a
        // placeholder attribute here is how files ended up claiming uvs=y while
        // carrying the per-triangle ramp from mesh_sync_render.
        if (ac_uv >= 0) attrs["TEXCOORD_0"] = ac_uv;
        nlohmann::json prim = nlohmann::json::object();
        prim["attributes"] = attrs;
        prim["indices"] = ac_idx;
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
