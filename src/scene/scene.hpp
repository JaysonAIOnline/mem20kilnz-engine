#pragma once

#include "../core/math.hpp"

#include <nlohmann/json.hpp>
#include <string>
#include <vector>

namespace kiln {

enum class NodeType { Mesh, Camera, Light, Empty, Bone };
enum class LightType { Sun, Point };
enum class PrimType { Cube, Plane, Sphere, Cylinder, Cone, Torus };
enum class EditMode { Object, Face, Edge, Vert };

struct Edge {
    uint32_t a = 0, b = 0;
};

inline Edge make_edge(uint32_t u, uint32_t v) { return u < v ? Edge{u, v} : Edge{v, u}; }
inline bool operator==(Edge x, Edge y) { return x.a == y.a && x.b == y.b; }

struct Vertex {
    Vec3 p;
    Vec3 n;
    Vec2 uv;
};

struct Face {
    std::vector<uint32_t> v;
};

struct SkinVert {
    uint16_t j[4] = {0, 0, 0, 0};
    float w[4] = {1, 0, 0, 0};
};

struct ShapeKey {
    std::string name;
    std::vector<Vec3> delta;
    float value = 0;
};

/// Per-face-corner attributes, the standard mesh "loop" representation.
///
/// `positions` + `faces` are what the editing ops work on. Normals and UVs are
/// per corner, because that is the only way a UV seam or a split normal can be
/// expressed at all. Stored as a flat array of three entries per face, in face
/// order, matching the triangulated fan order used by mesh_sync_render.
struct CornerAttr {
    Vec3 n{0, 1, 0};
    Vec2 uv{0, 0};
    bool has_uv = false;
};

struct LodLevel {
    std::vector<Vec3> positions;
    std::vector<Face> faces;
};

struct Mesh {
    std::string name;
    std::vector<Vec3> positions;
    std::vector<Face> faces;
    std::vector<Vertex> vertices;
    std::vector<uint32_t> indices;
    std::vector<int> tri_face;
    std::vector<SkinVert> skin;
    std::vector<int> joints;
    std::vector<Mat4> inv_bind;
    std::vector<ShapeKey> shapes;
    std::vector<LodLevel> lods;
    int lod = 0;
    bool smooth = false;
    /// Three entries per face. Empty means "no authored corner attributes",
    /// in which case normals are computed and no UVs exist.
    std::vector<CornerAttr> corner_attr;
    /// True only when at least one corner carried a real UV. The exporter omits
    /// TEXCOORD_0 when this is false rather than writing placeholder coordinates.
    bool has_uv = false;
};

struct AnimKey {
    int frame = 1;
    std::string node;
    Vec3 pose_rot;
    Vec3 pose_loc;
};

struct Material {
    std::string name = "Material";
    Vec3 albedo{0.72f, 0.73f, 0.76f};
    float roughness = 0.45f;
    float metallic = 0.0f;
    float emission = 0.0f;
};

struct Node {
    uint32_t id = 0;
    std::string name;
    NodeType type = NodeType::Empty;
    Vec3 translation{0, 0, 0};
    Vec3 rotation_deg{0, 0, 0};
    Vec3 scale{1, 1, 1};
    int parent = -1;
    int mesh = -1;
    int material = -1;
    float fov_deg = 50.0f;
    LightType light = LightType::Sun;
    Vec3 light_color{1, 0.96f, 0.90f};
    float light_intensity = 1.0f;
    Vec3 pose_rot{0, 0, 0};
    Vec3 pose_loc{0, 0, 0};
    float bone_length = 0.2f;
};

struct Scene {
    std::string name = "untitled";
    std::vector<Node> nodes;
    std::vector<Mesh> meshes;
    std::vector<Material> materials;
    uint32_t next_id = 1;
    int selection = -1;
    int active_camera = -1;
    EditMode edit_mode = EditMode::Object;
    std::vector<int> face_sel;
    std::vector<Edge> edge_sel;
    std::vector<uint32_t> vert_sel;
    int frame = 1;
    std::vector<AnimKey> keys;

    int find_node(const std::string& name) const;
    int find_node_id(uint32_t id) const;
    Node* node_by_name(const std::string& name);
    const Node* node_by_name(const std::string& name) const;
    std::string unique_name(std::string base) const;
    int add_node(Node n);
    bool remove_node_at(int index);
    Mat4 world_matrix(int index) const;
    Mat4 rest_matrix(int index) const;
    Mesh* selected_mesh();
};

Mesh make_primitive(PrimType type, Vec3 size, int segments = 24);
const char* prim_name(PrimType t);
bool parse_prim(const std::string& s, PrimType& out);
const char* node_type_name(NodeType t);
const char* edit_mode_name(EditMode m);

void mesh_sync_render(Mesh& m);
void mesh_ensure_faces(Mesh& m);
void mesh_compact(Mesh& m, std::vector<uint32_t>* remap_out = nullptr);
Vec3 face_normal(const Mesh& m, int fi);
Vec3 face_center(const Mesh& m, int fi);
std::vector<int> faces_matching(const Mesh& m, const std::string& region);
std::vector<Edge> edges_of_faces(const Mesh& m, const std::vector<int>& faces);
std::vector<Edge> edge_loop(const Mesh& m, Edge seed);
std::vector<Edge> edge_ring(const Mesh& m, Edge seed);
Edge pick_edge_axis(const Mesh& m, Vec3 axis);
bool extrude_faces(Mesh& m, const std::vector<int>& faces, float amount, std::vector<int>& new_sel);
bool inset_faces(Mesh& m, const std::vector<int>& faces, float amount, std::vector<int>& new_sel);
bool delete_faces_mesh(Mesh& m, const std::vector<int>& faces);
bool loop_cut(Mesh& m, Edge seed, int cuts, std::vector<Edge>& new_sel);
bool bevel_edges(Mesh& m, const std::vector<Edge>& edges, float amount, std::vector<Edge>& new_sel);
std::vector<uint32_t> verts_of_faces(const Mesh& m, const std::vector<int>& faces);
std::vector<uint32_t> verts_matching(const Mesh& m, const std::string& region);
bool translate_verts(Mesh& m, const std::vector<uint32_t>& verts, Vec3 delta);
bool subdivide_faces(Mesh& m, const std::vector<int>& faces, int cuts, std::vector<int>& new_sel);
bool merge_verts(Mesh& m, const std::vector<uint32_t>& verts, uint32_t& survivor);
bool fill_loops(Mesh& m, const std::vector<Edge>& edges, std::vector<int>& new_sel);
bool mirror_mesh(Mesh& m, int axis);
bool solidify_mesh(Mesh& m, float thickness);
bool array_mesh(Mesh& m, int count, Vec3 offset);
bool spin_mesh(Mesh& m, const std::vector<Edge>& profile, int steps, float angle_deg, Vec3 axis, Vec3 center);
bool bridge_loops(Mesh& m, const std::vector<int>& faces, const std::vector<Edge>& edges, std::vector<int>& new_sel);
void mesh_append_transformed(Mesh& dst, const Mesh& src, const Mat4& xform);
Vec3 mesh_centroid(const Mesh& m);
std::vector<Vec3> vertex_normals(const Mesh& m);
bool triangulate_faces(Mesh& m, const std::vector<int>& faces);
bool poke_faces(Mesh& m, const std::vector<int>& faces, std::vector<int>& new_sel);
bool flip_faces(Mesh& m, const std::vector<int>& faces);
bool dissolve_faces(Mesh& m, const std::vector<int>& faces, std::vector<int>& new_sel);
bool duplicate_faces_mesh(Mesh& m, const std::vector<int>& faces, float amount, std::vector<int>& new_sel);
bool split_region(Mesh& m, const std::vector<int>& faces);
bool weld_distance(Mesh& m, float dist);
bool smooth_verts(Mesh& m, const std::vector<uint32_t>& verts, float factor, int iters);
bool push_verts(Mesh& m, const std::vector<uint32_t>& verts, float amount);
bool to_sphere(Mesh& m, const std::vector<uint32_t>& verts, float factor);
bool flatten_verts(Mesh& m, const std::vector<uint32_t>& verts, Vec3 axis);
bool randomize_verts(Mesh& m, const std::vector<uint32_t>& verts, float amount);
bool snap_verts(Mesh& m, const std::vector<uint32_t>& verts, float grid);
bool bisect_mesh(Mesh& m, Vec3 plane_n, float plane_d);
bool connect_verts(Mesh& m, uint32_t a, uint32_t b);
std::vector<int> grow_faces(const Mesh& m, const std::vector<int>& faces);
std::vector<Edge> grow_edges(const Mesh& m, const std::vector<Edge>& edges);
std::vector<uint32_t> grow_verts(const Mesh& m, const std::vector<uint32_t>& verts);
std::vector<int> linked_faces(const Mesh& m, const std::vector<int>& seeds);

nlohmann::json scene_to_json(const Scene& s);
bool scene_from_json(const nlohmann::json& j, Scene& s);

int add_bone(Scene& s, const std::string& name, int parent, Vec3 local, Vec3 rest_rot, float length);
int rig_biped(Scene& s, bool apose);
int rig_quadruped(Scene& s);
int rig_face(Scene& s);
bool bind_mesh(Scene& s, int mesh_node);
void deform_positions(const Scene& s, int mesh_node, std::vector<Vec3>& out);
bool build_lods(Mesh& m, int levels);
nlohmann::json describe_scene(const Scene& s);
bool catmull_clark(Mesh& m, int levels);
bool extrude_individual(Mesh& m, const std::vector<int>& faces, float amount, std::vector<int>& new_sel);
bool symmetrize_mesh(Mesh& m, int axis);
bool shrinkwrap_mesh(Mesh& m, const Mesh& target, float weight);
bool recalc_normals(Mesh& m);
bool decimate_mesh(Mesh& m, float ratio);
bool wireframe_mesh(Mesh& m, float thickness);
bool shear_verts(Mesh& m, const std::vector<uint32_t>& verts, Vec3 axis, float factor);
bool rotate_verts(Mesh& m, const std::vector<uint32_t>& verts, Vec3 euler_deg);
bool scale_verts(Mesh& m, const std::vector<uint32_t>& verts, Vec3 scl);
std::vector<int> boundary_faces(const Mesh& m);
std::vector<Mesh> separate_loose_meshes(Mesh& m);
bool ik_two_bone(Scene& s, int tip, Vec3 target_world);
void apply_frame(Scene& s, int frame);
bool export_obj(const Scene& scene, const std::string& path, std::string& err);
bool import_obj(Scene& scene, const std::string& path, std::string& err);

}  // namespace kiln
