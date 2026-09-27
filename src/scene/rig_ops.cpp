#include "../scene/scene.hpp"

#include <algorithm>
#include <cmath>
#include <map>

namespace kiln {
namespace {

// Pose a bone so its local +X points along `world_dir`, expressed as the pose
// delta that the bone's own rest rotation already implies. Bones are built
// pointing down +X (see rig.cpp), so +X is the bone's forward axis.
//
// The bone's local matrix is rest * pose, and the parent world already includes
// every ancestor's pose. Inverting the parent's world and the bone's own rest
// rotation leaves exactly the delta to store.
bool aim_bone(Scene& s, int bone, Vec3 world_dir) {
    if (bone < 0 || bone >= static_cast<int>(s.nodes.size())) return false;
    if (length(world_dir) < 1e-9f) return false;
    const Node& n = s.nodes[static_cast<size_t>(bone)];
    int parent = n.parent;
    Mat4 P = parent >= 0 && parent != bone ? s.world_matrix(parent) : Mat4::identity();
    Mat4 W = s.world_matrix(bone);
    Vec3 cur_up = mat_column(W, 1);
    Mat4 R_new = basis_from_x(world_dir, cur_up);

    Mat4 L_new = mul(invert_affine(P), R_new);
    Mat4 R_rest_local = mat_rotation_part(trs(n.translation, n.rotation_deg, n.scale));
    Mat4 R_pose = mul(invert_affine(R_rest_local), mat_rotation_part(L_new));
    s.nodes[static_cast<size_t>(bone)].pose_rot = mat_to_euler_xyz(R_pose);
    return true;
}

}  // namespace


bool ik_two_bone(Scene& s, int tip, Vec3 target_world) {
    if (tip < 0 || tip >= static_cast<int>(s.nodes.size())) return false;
    if (s.nodes[static_cast<size_t>(tip)].type != NodeType::Bone) return false;
    int mid = s.nodes[static_cast<size_t>(tip)].parent;
    if (mid < 0 || mid >= static_cast<int>(s.nodes.size())) return false;
    if (s.nodes[static_cast<size_t>(mid)].type != NodeType::Bone) return false;
    int root = s.nodes[static_cast<size_t>(mid)].parent;
    if (root < 0 || root >= static_cast<int>(s.nodes.size())) return false;
    if (s.nodes[static_cast<size_t>(root)].type != NodeType::Bone) return false;

    Vec3 p_root = transform_point(s.world_matrix(root), {0, 0, 0});
    Vec3 p_mid = transform_point(s.world_matrix(mid), {0, 0, 0});
    Vec3 p_tip = transform_point(s.world_matrix(tip), {0, 0, 0});

    Vec3 u = p_mid - p_root;
    Vec3 v = p_tip - p_mid;
    float L1 = length(u), L2 = length(v);
    if (L1 < 1e-6f || L2 < 1e-6f) return false;

    Vec3 to_target = target_world - p_root;
    float d = length(to_target);
    const float kEps = 1e-4f;
    float dmin = std::abs(L1 - L2) + kEps;
    float dmax = L1 + L2 - kEps;
    if (d < dmin) d = dmin;
    if (d > dmax) d = dmax;
    if (dmax <= dmin) return false;

    float cos_a1 = (L1 * L1 + d * d - L2 * L2) / (2.0f * L1 * d);
    float a1 = std::acos(std::max(-1.0f, std::min(1.0f, cos_a1)));

    Vec3 axis = cross(normalize(u), normalize(v));
    if (length(axis) < 1e-5f) {
        Vec3 alt{0, 0, 1};
        if (std::abs(dot(normalize(u), alt)) > 0.9f) alt = Vec3{0, 1, 0};
        axis = normalize(cross(normalize(u), alt));
    }
    axis = normalize(axis);

    Vec3 probe = normalize(u) * L1;
    Vec3 cand_a = rotate_around(p_root + probe, p_root, axis, a1) - p_root;
    Vec3 cand_b = rotate_around(p_root + probe, p_root, axis, -a1) - p_root;
    Vec3 new_dir = length(cand_a - to_target) <= length(cand_b - to_target) ? cand_a : cand_b;
    if (length(new_dir) < 1e-9f) return false;
    new_dir = normalize(new_dir);

    if (!aim_bone(s, root, new_dir)) return false;

    Vec3 p_mid_new = p_root + new_dir * L1;
    Vec3 want_mid_to_tip = target_world - p_mid_new;
    if (length(want_mid_to_tip) < 1e-6f) {
        Vec3 alt{0, 0, 1};
        if (std::abs(dot(new_dir, alt)) > 0.9f) alt = Vec3{0, 1, 0};
        want_mid_to_tip = cross(new_dir, alt);
    }
    return aim_bone(s, mid, normalize(want_mid_to_tip));
}

}  // namespace kiln
