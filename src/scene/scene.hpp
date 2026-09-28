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
    /// World-space bone positions matching `joints`, used when authoring weights.
    std::vector<Vec3> joint_positions;
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

/// How a keyframe interpolates towards the next one.
enum class Interp { Step, Linear, Bezier };

/// One channel of one node over time.
///
/// `in_tan` and `out_tan` are Hermite slopes in value units per frame, which is
/// what makes them convertible to glTF CUBICSPLINE tangents without guessing. A
/// zero slope is a corner, not a flat handle: the auto-tangent path fills them in
/// when they are left unset, so a key that was never touched still curves.
struct Keyframe {
    int frame = 1;
    Vec3 value;
    Interp interp = Interp::Linear;
    Vec3 in_tan{0, 0, 0};
    Vec3 out_tan{0, 0, 0};
};

/// Which transform a track drives.
enum class Channel { Location = 0, Rotation = 1, Scale = 2 };

struct Track {
    std::string node;   ///< node name, so a clip survives a scene rebuild
    Channel channel = Channel::Location;
    std::vector<Keyframe> keys;

    int start() const { return keys.empty() ? 1 : keys.front().frame; }
    int end() const { return keys.empty() ? 1 : keys.back().frame; }
};

/// A named animation: a set of tracks plus a frame range and rate.
///
/// The previous representation was a flat list of bone poses with no name, no
/// channels, no rate and no interpolation choice, so nothing could be authored,
/// two clips could not coexist, and no clip could be exported.
struct Clip {
    std::string name;
    float fps = 24.0f;
    std::vector<Track> tracks;

    int start() const;
    int end() const;
    /// Keys sorted by frame, per track, so evaluation is a bracket search.
    void sort_keys();
    Track* track_for(const std::string& node, Channel channel);
    const Track* track_for(const std::string& node, Channel channel) const;
    int track_count() const { return static_cast<int>(tracks.size()); }
    int key_count() const;
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

/// A value on a graph socket.
///
/// Deliberately a small explicit sum rather than a variant: a node that cannot be
/// fed a real number has to say so, and an agent reading an error needs the type
/// it actually wanted. Mesh sockets are the heavy case and are only populated for
/// them.
struct SockValue {
    enum class Type { Float, Int, Vec3, String, Mesh };
    Type type = Type::Float;
    float f = 0.0f;
    int i = 0;
    Vec3 v{0, 0, 0};
    std::string s;
    Mesh mesh;

    static SockValue of_float(float x) { SockValue r; r.type = Type::Float; r.f = x; return r; }
    static SockValue of_int(int x) { SockValue r; r.type = Type::Int; r.i = x; return r; }
    static SockValue of_vec3(Vec3 x) { SockValue r; r.type = Type::Vec3; r.v = x; return r; }
    static SockValue of_string(std::string x) { SockValue r; r.type = Type::String; r.s = std::move(x); return r; }
    static SockValue of_mesh(Mesh x) { SockValue r; r.type = Type::Mesh; r.mesh = std::move(x); return r; }
};

/// One input of a node instance: either a literal or a link to another node's
/// output. A link always wins over the literal, and the literal is kept so
/// disconnecting a link restores the previous value.
struct SockIn {
    std::string name;
    SockValue literal;
    /// True once a caller has set this literal. Needed to tell "left at its
    /// default, so the input is simply empty" from "explicitly set to something
    /// of the wrong type", which are different mistakes and need different
    /// messages.
    bool literal_set = false;
    int link_node = -1;
    int link_sock = 0;
    bool linked() const { return link_node >= 0; }
};

struct GraphNode {
    std::string type;              ///< registry key
    std::string name;              ///< unique within the graph, for readable ops
    std::vector<SockIn> inputs;
    int x = 0, y = 0;              ///< layout, so a non-visual agent can place it
};

/// A procedural graph. Evaluation is demand-driven from the output node and
/// calls the real mesh operations, so a graph produces the same geometry the
/// equivalent op sequence would.
struct NodeGraph {
    std::string name;
    std::vector<GraphNode> nodes;
    int output_node = -1;
    int find_node(const std::string& n) const;
    GraphNode* node_at(int i);
    const GraphNode* node_at(int i) const;
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
    /// Procedural node graphs.
    std::vector<NodeGraph> graphs;

    NodeGraph* graph_named(const std::string& n);
    const NodeGraph* graph_named(const std::string& n) const;
    int find_graph(const std::string& n) const;

    /// Animation clips. Serialized so the journal can replay a build that
    /// animated something, which it previously could not.
    std::vector<Clip> clips;
    int active_clip = -1;

    Clip* active() {
        if (active_clip < 0 || active_clip >= static_cast<int>(clips.size())) return nullptr;
        return &clips[static_cast<size_t>(active_clip)];
    }
    const Clip* active() const {
        if (active_clip < 0 || active_clip >= static_cast<int>(clips.size())) return nullptr;
        return &clips[static_cast<size_t>(active_clip)];
    }
    Clip* clip_named(const std::string& n);
    /// Index of a named clip, or -1. Separate from clip_named because a null
    /// clip is ambiguous between "absent" and "not active".
    int find_clip(const std::string& n) const;
    const Clip* clip_named(const std::string& n) const;

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

/// Measured quality of a mesh: topology and size, so an operation can report
/// what it produced instead of leaving the caller to export and guess.
struct MeshStats {
    int faces = 0;
    int verts = 0;
    int boundary_edges = 0;      ///< edges used by exactly one face: a hole
    int nonmanifold_edges = 0;   ///< edges used by more than two faces
    int degenerate_faces = 0;
    int non_finite = 0;
    double surface_area = 0.0;
    double signed_volume = 0.0;
    /// A closed surface has no boundary and no non-manifold edges.
    bool watertight() const { return boundary_edges == 0 && nonmanifold_edges == 0; }
};

MeshStats mesh_stats(const Mesh& m);

/// Move vertices along their normals by layered value noise. Deterministic for a
/// given seed, so a displaced mesh replays byte-identically. `octaves` is 1..8.
bool displace_mesh(Mesh& m, float amplitude, float frequency, int seed, int octaves,
                   bool along_normal);

/// Bind every vertex to up to four of `bone_nodes` with smooth falloff weights,
/// filling `skin`, `joints` and `inv_bind` the way the importers do. A vertex
/// beyond every bone's falloff binds fully to the nearest, so none is left
/// unbound. `falloff` is the bone radius in world units, clamped to (0, 1].
bool skin_mesh(Mesh& m, const std::vector<int>& bone_nodes, float falloff);

/// What a graph evaluation produced, with the measurements an agent needs to
/// check it without rendering anything.
struct GraphResult {
    bool ok = false;
    std::string error;      ///< empty when ok
    std::string failed_node;///< node that failed, for a readable error
    int nodes_evaluated = 0;
    Mesh mesh;
    MeshStats stats;
};

/// Names of every registered node type, for discovery.
std::vector<std::string> graph_node_types();
/// Input socket names and types for a node type, empty if the type is unknown.
std::vector<std::pair<std::string, SockValue::Type>> graph_node_inputs(
    const std::string& type);
/// Default value for one input of a node type, so a caller can set one socket and
/// leave the rest at their defaults.
SockValue graph_socket_default(const std::string& type, const std::string& input);

/// Evaluate `g` from its output node (or `forced_output` when >= 0).
/// Reports the failing node and the reason rather than returning a partial mesh.
GraphResult evaluate_graph(const NodeGraph& g, int forced_output = -1);

/// Resample a closed mesh onto an isosurface extracted from a voxel field.
/// `res` is the target cell count along the longest axis (clamped to 6..192).
/// The result is closed by construction; the extracted surface sits within one
/// cell of the input, so volume error falls as O(h^2).
bool remesh_mesh(const Mesh& in, Mesh& out, int res);

/// Boolean of two closed meshes on a shared voxel grid. `mode`: 0 union,
/// 1 difference (a - b), 2 intersection. Both operands are sampled on one grid
/// over the union of their bounds, so a moved or rotated operand is handled
/// correctly. Requires closed inputs; a holed input has no well-defined inside.
bool boolean_mesh(const Mesh& a, const Mesh& b, Mesh& out, int mode, int res);

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

/// Quadric error metric decimation. Returns false when no legal collapse
/// remains, which is a real answer: forcing the target is what cost the previous
/// implementation 20-79% of its volume.
bool decimate_qem(Mesh& m, float ratio, int max_collapses);
bool wireframe_mesh(Mesh& m, float thickness);
bool shear_verts(Mesh& m, const std::vector<uint32_t>& verts, Vec3 axis, float factor);
bool rotate_verts(Mesh& m, const std::vector<uint32_t>& verts, Vec3 euler_deg);
bool scale_verts(Mesh& m, const std::vector<uint32_t>& verts, Vec3 scl);
std::vector<int> boundary_faces(const Mesh& m);
std::vector<Mesh> separate_loose_meshes(Mesh& m);
bool ik_two_bone(Scene& s, int tip, Vec3 target_world);
/// Evaluate the active clip at `frame` and write the result onto the nodes.
/// Returns the number of channels driven, or 0 when there is no active clip.
int apply_frame(Scene& s, int frame);

/// Evaluate `clip` at `frame` into a map of node name -> (loc, rot, scale).
/// Separated from apply_frame so a clip can be retargeted or inspected without
/// mutating the scene.
struct ClipSample {
    Vec3 location{0, 0, 0};
    Vec3 rotation_deg{0, 0, 0};
    Vec3 scale{1, 1, 1};
};
void sample_clip(const Clip& c, int frame, std::vector<std::pair<std::string, ClipSample>>& out);

/// Re-map a clip's tracks onto the bones present in `target`, matching by bone
/// name. Tracks with no counterpart are dropped and counted in `dropped_out`
/// rather than being attached to a similar-looking bone. Returns false when the
/// target has no bones or no track survived.
bool retarget_clip(Clip& clip, const Scene& target, int* dropped_out);
bool export_obj(const Scene& scene, const std::string& path, std::string& err);
bool import_obj(Scene& scene, const std::string& path, std::string& err);

}  // namespace kiln
