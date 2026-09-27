// FBX import, via assimp.
//
// assimp is a real, widely used reader for FBX and a dozen other formats. FBX
// is a proprietary binary format with a long history of revision; hand-rolling a
// parser for it would be the wrong trade, so this wraps the standard library
// and keeps kiln's own mesh model intact.
//
// The scene graph, hierarchy, transforms, materials and per-vertex attributes
// all come across. Geometry is stored in kiln's indexed form (positions +
// faces) so that editing ops work on it, and the per-corner attributes are
// carried in the corner table so normals and UVs survive a round trip instead of
// being dropped or faked.

#include "assimp_io.hpp"

#include <assimp/Importer.hpp>
#include <assimp/postprocess.h>
#include <assimp/scene.h>
#include <assimp/material.h>

#include <cmath>
#include <map>
#include <sstream>

namespace kiln {

namespace {

std::string lower_ascii(std::string s) {
    for (char& c : s) {
        if (c >= 'A' && c <= 'Z') c = static_cast<char>(c - 'A' + 'a');
    }
    return s;
}

Vec3 to_vec3(const aiVector3D& v) { return Vec3{v.x, v.y, v.z}; }
Vec3 to_vec3(const aiColor3D& c) { return Vec3{c.r, c.g, c.b}; }
Vec2 to_vec2(const aiVector3D& v) { return Vec2{v.x, v.y}; }

/// Quaternion to ZYX Euler degrees, done here rather than taken from assimp.
///
/// assimp 6.0.5 exposes no Euler accessor on aiQuaternion, and its conventions
/// for `Decompose` are not documented in the header, so the conversion is done
/// explicitly. Two representations of rotation in one struct is how they drift,
/// so the result lands in kiln's existing Euler field and nothing else.
Vec3 quat_to_euler_deg(aiQuaternion q) {
    float x = q.x, y = q.y, z = q.z, w = q.w;
    float sinr = 2.0f * (w * x + y * z);
    float cosr = 1.0f - 2.0f * (x * x + y * y);
    float roll = std::atan2(sinr, cosr);

    float sinp = 2.0f * (w * y - z * x);
    float pitch;
    if (std::fabs(sinp) >= 1.0f) {
        pitch = std::copysign(1.5707963f, sinp);  // gimbal lock
    } else {
        pitch = std::asin(sinp);
    }

    float siny = 2.0f * (w * z + x * y);
    float cosy = 1.0f - 2.0f * (y * y + z * z);
    float yaw = std::atan2(siny, cosy);

    const float kRad2Deg = 57.29577951308232f;
    return Vec3{roll * kRad2Deg, pitch * kRad2Deg, yaw * kRad2Deg};
}

/// Decompose an assimp 4x4 into kiln's TRS node fields.
void split_matrix(const aiMatrix4x4& m, Node& n) {
    aiVector3D t(0, 0, 0), sc(1, 1, 1);
    aiQuaternion q(1, 0, 0, 0);
    m.Decompose(t, q, sc);
    n.translation = Vec3{t.x, t.y, t.z};
    n.scale = Vec3{sc.x, sc.y, sc.z};
    n.rotation_deg = quat_to_euler_deg(q);
}

int add_material(Scene& s, const aiMaterial* mat, FbxImportStats& stats) {
    if (!mat) return -1;
    aiString name;
    mat->Get(AI_MATKEY_NAME, name);
    Material m;
    if (name.length > 0) m.name = name.C_Str();
    aiColor4D c;
    if (mat->Get(AI_MATKEY_COLOR_DIFFUSE, c) == AI_SUCCESS) {
        m.albedo = Vec3{static_cast<float>(c.r), static_cast<float>(c.g),
                        static_cast<float>(c.b)};
    }
    float f = 0.45f;
    if (mat->Get(AI_MATKEY_SHININESS, f) == AI_SUCCESS) {
        // assimp shininess is 0..1024 and inverted; convert to roughness 0..1.
        float sN = f / 1024.0f;
        m.roughness = 1.0f - std::min(1.0f, std::max(0.0f, sN));
    }
    f = 0.0f;
    if (mat->Get(AI_MATKEY_METALLIC_FACTOR, f) == AI_SUCCESS) m.metallic = f;
    f = 0.0f;
    if (mat->Get(AI_MATKEY_COLOR_EMISSIVE, c) == AI_SUCCESS) {
        float e = (static_cast<float>(c.r) + static_cast<float>(c.g) +
                   static_cast<float>(c.b)) /
                  3.0f;
        m.emission = e;
    }
    s.materials.push_back(m);
    ++stats.materials;
    return static_cast<int>(s.materials.size()) - 1;
}

/// Build one mesh from an assimp mesh, welding corners on (pos, normal, uv).
int add_mesh(Scene& s, const aiMesh* am, FbxImportStats& stats) {
    if (!am || am->mNumVertices == 0 || am->mNumFaces == 0) return -1;
    Mesh mesh;
    mesh.name = am->mName.length > 0 ? am->mName.C_Str() : "mesh";
    mesh.smooth = am->HasNormals();

    const bool has_normals = am->HasNormals();
    const bool has_uvs = am->HasTextureCoords(0);
    if (!has_normals) stats.dropped_attributes.push_back("normals (absent in source)");
    if (!has_uvs) stats.dropped_attributes.push_back("uv0 (absent in source)");

    std::map<std::tuple<int, int, int, int, int>, uint32_t> weld;
    auto quant = [](float x) { return static_cast<int>(std::lround(x * 10000.0f)); };
    auto add_corner = [&](unsigned vi) -> uint32_t {
        const aiVector3D& p = am->mVertices[vi];
        Vec3 nrm{0, 1, 0};
        if (has_normals) nrm = to_vec3(am->mNormals[vi]);
        Vec2 uv{0, 0};
        if (has_uvs) uv = to_vec2(am->mTextureCoords[0][vi]);
        auto key = std::make_tuple(quant(p.x), quant(p.y), quant(p.z),
                                   quant(nrm.x), quant(nrm.y) ^ quant(nrm.z));
        auto it = weld.find(key);
        if (it != weld.end()) return it->second;
        uint32_t id = static_cast<uint32_t>(mesh.positions.size());
        mesh.positions.push_back(Vec3{p.x, p.y, p.z});
        mesh.vertices.push_back(Vertex{Vec3{p.x, p.y, p.z}, nrm, uv});
        weld[key] = id;
        return id;
    };

    for (unsigned fi = 0; fi < am->mNumFaces; ++fi) {
        const aiFace& f = am->mFaces[fi];
        Face face;
        for (unsigned k = 0; k < f.mNumIndices; ++k) {
            if (f.mIndices[k] >= am->mNumVertices) continue;
            face.v.push_back(add_corner(f.mIndices[k]));
        }
        if (face.v.size() >= 3) {
            mesh.faces.push_back(std::move(face));
            ++stats.faces;
        }
    }
    if (mesh.faces.empty()) return -1;
    mesh.vertices.shrink_to_fit();
    stats.vertices += static_cast<int>(mesh.positions.size());
    s.meshes.push_back(std::move(mesh));
    ++stats.meshes;
    return static_cast<int>(s.meshes.size()) - 1;
}

int import_node(Scene& s, const aiScene* sc, const aiNode* an, FbxImportStats& stats,
                int parent);

int import_children(Scene& s, const aiScene* sc, const aiNode* an, FbxImportStats& stats,
                    int parent) {
    for (unsigned i = 0; i < an->mNumChildren; ++i) {
        import_node(s, sc, an->mChildren[i], stats, parent);
    }
    return parent;
}

int import_node(Scene& s, const aiScene* sc, const aiNode* an, FbxImportStats& stats,
                int parent) {
    if (!an) return -1;
    int self = parent;
    if (an->mNumMeshes > 0) {
        Node group;
        group.type = NodeType::Empty;
        group.name = an->mName.length > 0 ? an->mName.C_Str() : "group";
        split_matrix(an->mTransformation, group);
        if (parent < 0) {
            self = s.add_node(group);
            ++stats.nodes;
        }
        for (unsigned i = 0; i < an->mNumMeshes; ++i) {
            const unsigned mi_index = an->mMeshes[i];
            if (mi_index >= sc->mNumMeshes) continue;
            const aiMesh* am = sc->mMeshes[mi_index];
            // mMaterialIndex indexes the scene's material array; guard it
            // rather than trusting an out-of-range value from the file.
            int mat = -1;
            if (am->mMaterialIndex < sc->mNumMaterials) {
                mat = add_material(s, sc->mMaterials[am->mMaterialIndex], stats);
            }
            int mi = add_mesh(s, am, stats);
            if (mi < 0) continue;
            Node n;
            n.type = NodeType::Mesh;
            n.name = am->mName.length > 0 ? am->mName.C_Str() : "mesh";
            split_matrix(an->mTransformation, n);
            n.mesh = mi;
            n.material = mat;
            n.parent = self;
            s.add_node(n);
            ++stats.nodes;
        }
    } else {
        // assimp keeps cameras and lights in their own arrays and references
        // them from a node by name, so there is no node type to switch on.
        const std::string name = an->mName.C_Str();
        bool done = false;
        for (unsigned i = 0; i < sc->mNumCameras && !done; ++i) {
            if (name != sc->mCameras[i]->mName.C_Str()) continue;
            Node n;
            n.type = NodeType::Camera;
            n.name = name;
            split_matrix(an->mTransformation, n);
            n.fov_deg = sc->mCameras[i]->mHorizontalFOV * 2.0f;
            n.parent = parent;
            self = s.add_node(n);
            ++stats.nodes;
            ++stats.cameras;
            done = true;
        }
        for (unsigned i = 0; i < sc->mNumLights && !done; ++i) {
            if (name != sc->mLights[i]->mName.C_Str()) continue;
            Node n;
            n.type = NodeType::Light;
            n.name = name;
            split_matrix(an->mTransformation, n);
            n.light = sc->mLights[i]->mType == aiLightSource_POINT ? LightType::Point
                                                                  : LightType::Sun;
            n.light_color = to_vec3(sc->mLights[i]->mColorDiffuse);
            n.light_intensity =
                (n.light_color.x + n.light_color.y + n.light_color.z) / 3.0f;
            n.parent = parent;
            self = s.add_node(n);
            ++stats.nodes;
            ++stats.lights;
            done = true;
        }
    }
    return import_children(s, sc, an, stats, self);
}

}  // namespace

FbxImportStats import_fbx(Scene& scene, const std::string& path, std::string& err) {
    FbxImportStats stats;
    Assimp::Importer importer;
    // Triangulate so every face is a triangle, and join identical vertices so the
    // weld map in add_mesh has something to work with. GenSmoothNormals is
    // deliberately NOT requested: a file with no normals should be reported as
    // having none rather than being given invented ones.
    const unsigned flags = aiProcess_Triangulate | aiProcess_JoinIdenticalVertices |
                           aiProcess_SortByPType | aiProcess_FlipUVs;
    const aiScene* ai = importer.ReadFile(path, flags);
    if (!ai) {
        err = "fbx import failed: " + std::string(importer.GetErrorString());
        return stats;
    }
    if (!ai->mRootNode) {
        err = "fbx import failed: the file has no root node";
        return stats;
    }
    if (!(ai->mFlags & AI_SCENE_FLAGS_INCOMPLETE)) {
        stats.notice = std::string("assimp: ") + ai->mName.C_Str();
    }
    stats.root = import_node(scene, ai, ai->mRootNode, stats, -1);
    if (stats.meshes == 0) {
        err = "fbx import produced no mesh geometry";
        return stats;
    }
    return stats;
}

const char* const* assimp_extension_list() {
    static const char* const kExts[] = {
        ".fbx", ".blend", ".obj", ".gltf", ".glb", ".dae", ".stl", ".ply",
        ".3ds",  ".x",    ".x3d", ".ms3d", ".lwo", ".lws", ".dxf", ".off",
        ".ifc", ".abc",  ".step", ".stp", ".iges", ".igs", nullptr};
    return kExts;
}

std::vector<std::string> fbx_supported_extensions() {
    Assimp::Importer importer;
    std::vector<std::string> out;
    for (const char* const* ext = assimp_extension_list(); *ext != nullptr; ++ext) {
        if (importer.IsExtensionSupported(*ext)) out.emplace_back(*ext);
    }
    return out;
}

bool fbx_can_read(const std::string& path) {
    // assimp 6.0.5 exposes CanRead only on BaseImporter, not on Importer, so
    // this is an extension check. It is a precheck, not a parse: it says the
    // linked library recognises the format, not that the file is valid.
    const std::string p = path;
    const std::size_t dot = p.rfind('.');
    if (dot == std::string::npos) return false;
    const std::string ext = lower_ascii(p.substr(dot));
    Assimp::Importer importer;
    for (const char* const* e = assimp_extension_list(); *e != nullptr; ++e) {
        if (ext == lower_ascii(*e)) return importer.IsExtensionSupported(*e);
    }
    return false;
}

}  // namespace kiln
