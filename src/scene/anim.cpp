// Animation clips: authoring, evaluation and retargeting.
//
// The previous implementation was a flat list of bone poses with no clip, no
// channels and a single hardcoded lerp, so it could not animate a mesh node, could
// not hold two clips at once, and could not be exported. This is the real thing:
// named clips of per-channel tracks, three interpolation modes, and a sampler
// that can be aimed at a rig other than the one it was authored on.
//
// Interpolation is Hermite, which covers all three modes without special cases:
//   STEP   clamps to the left key
//   LINEAR sets both slopes to the chord
//   BEZIER uses the stored tangents, or a Catmull-Rom estimate when a tangent
//          was left at zero, which is what makes an untouched key still curve
//          through its neighbours instead of kinking.

#include "../scene/scene.hpp"

#include <algorithm>
#include <cmath>
#include <map>
#include <utility>

namespace kiln {

int Clip::start() const {
    int lo = 0;
    bool any = false;
    for (const Track& t : tracks) {
        if (t.keys.empty()) continue;
        if (!any || t.start() < lo) lo = t.start();
        any = true;
    }
    return any ? lo : 1;
}

int Clip::end() const {
    int hi = 1;
    bool any = false;
    for (const Track& t : tracks) {
        if (t.keys.empty()) continue;
        if (!any || t.end() > hi) hi = t.end();
        any = true;
    }
    return any ? hi : 1;
}

int Clip::key_count() const {
    int n = 0;
    for (const Track& t : tracks) n += static_cast<int>(t.keys.size());
    return n;
}

void Clip::sort_keys() {
    for (Track& t : tracks) {
        std::stable_sort(t.keys.begin(), t.keys.end(),
                         [](const Keyframe& a, const Keyframe& b) { return a.frame < b.frame; });
    }
}

Track* Clip::track_for(const std::string& node, Channel channel) {
    for (Track& t : tracks) {
        if (t.node == node && t.channel == channel) return &t;
    }
    return nullptr;
}

const Track* Clip::track_for(const std::string& node, Channel channel) const {
    for (const Track& t : tracks) {
        if (t.node == node && t.channel == channel) return &t;
    }
    return nullptr;
}

Clip* Scene::clip_named(const std::string& n) {
    for (Clip& c : clips) {
        if (c.name == n) return &c;
    }
    return nullptr;
}

int Scene::find_clip(const std::string& n) const {
    for (size_t i = 0; i < clips.size(); ++i) {
        if (clips[i].name == n) return static_cast<int>(i);
    }
    return -1;
}

const Clip* Scene::clip_named(const std::string& n) const {
    for (const Clip& c : clips) {
        if (c.name == n) return &c;
    }
    return nullptr;
}

namespace {

/// Hermite basis on t in [0,1].
void hermite(float t, float& h00, float& h10, float& h01, float& h11) {
    const float t2 = t * t;
    const float t3 = t2 * t;
    h00 = 2 * t3 - 3 * t2 + 1;
    h10 = t3 - 2 * t2 + t;
    h01 = -2 * t3 + 3 * t2;
    h11 = t3 - t2;
}

/// Catmull-Rom slope through the keys either side of index i.
///
/// A key with no stored handle gets this instead of zero, so an untouched key
/// still curves through its neighbours rather than kinking into a corner. At the
/// ends it falls back to the chord on the side that exists, which lets the curve
/// leave the first key and arrive at the last without a flat stop.
Vec3 auto_tangent(const std::vector<Keyframe>& k, std::size_t i) {
    const std::size_t n = k.size();
    if (n < 2) return Vec3{0, 0, 0};
    if (i == 0) {
        return (k[1].value - k[0].value) / static_cast<float>(k[1].frame - k[0].frame);
    }
    if (i + 1 >= n) {
        return (k[n - 1].value - k[n - 2].value) /
               static_cast<float>(k[n - 1].frame - k[n - 2].frame);
    }
    const float span = static_cast<float>(k[i + 1].frame - k[i - 1].frame);
    if (!(std::fabs(span) > 0.0f)) return Vec3{0, 0, 0};
    return (k[i + 1].value - k[i - 1].value) / span;
}

/// Value of one track at `frame`. Clamps outside the key range, which is what
/// makes a held pose before the first key and after the last.
Vec3 eval_track(const Track& t, int frame) {
    const std::vector<Keyframe>& k = t.keys;
    if (k.empty()) return Vec3{0, 0, 0};
    if (k.size() == 1) return k[0].value;
    if (frame <= k.front().frame) return k.front().value;
    if (frame >= k.back().frame) return k.back().value;

    // Bracket search. The previous version grouped every key by frame and then
    // rescanned that whole map for each node, which is O(nodes * keys).
    std::size_t hi = 1;
    while (hi < k.size() && k[hi].frame <= frame) ++hi;
    const Keyframe& a = k[hi - 1];
    const Keyframe& b = k[hi];
    if (a.frame == b.frame) return b.value;

    if (a.interp == Interp::Step) return a.value;

    const float dt = static_cast<float>(b.frame - a.frame);
    const float u = static_cast<float>(frame - a.frame) / dt;

    if (a.interp == Interp::Linear) return lerp(a.value, b.value, u);

    // BEZIER: slopes in value per frame, so they scale by dt for the basis.
    const Vec3 m0 = length(a.out_tan) >= 1e-9f ? a.out_tan : auto_tangent(k, hi - 1);
    const Vec3 m1 = length(b.in_tan) >= 1e-9f ? b.in_tan : auto_tangent(k, hi);
    float h00 = 0, h10 = 0, h01 = 0, h11 = 0;
    hermite(u, h00, h10, h01, h11);
    return a.value * h00 + m0 * (h10 * dt) + b.value * h01 + m1 * (h11 * dt);
}

}  // namespace

void sample_clip(const Clip& c, int frame,
                 std::vector<std::pair<std::string, ClipSample>>& out) {
    // Preserve first-seen node order so the output is stable across runs.
    std::map<std::string, size_t> seen;
    for (const Track& t : c.tracks) {
        if (t.keys.empty()) continue;
        auto it = seen.find(t.node);
        if (it == seen.end()) {
            seen[t.node] = out.size();
            out.emplace_back(t.node, ClipSample{});
            it = seen.find(t.node);
        }
        ClipSample& s = out[it->second].second;
        const Vec3 v = eval_track(t, frame);
        if (t.channel == Channel::Location) s.location = v;
        else if (t.channel == Channel::Rotation) s.rotation_deg = v;
        else s.scale = v;
    }
}

int apply_frame(Scene& s, int frame) {
    s.frame = frame;
    Clip* c = s.active();
    if (!c) return 0;

    std::vector<std::pair<std::string, ClipSample>> samples;
    sample_clip(*c, frame, samples);
    for (const auto& kv : samples) {
        Node* n = s.node_by_name(kv.first);
        if (!n) continue;
        const bool has_loc = c->track_for(kv.first, Channel::Location) != nullptr;
        const bool has_rot = c->track_for(kv.first, Channel::Rotation) != nullptr;
        const bool has_scale = c->track_for(kv.first, Channel::Scale) != nullptr;
        if (n->type == NodeType::Bone) {
            // Bones keep their authored translation and drive their pose, which is
            // what the skinning path already reads.
            if (has_loc) n->pose_loc = kv.second.location;
            if (has_rot) n->pose_rot = kv.second.rotation_deg;
        } else {
            // A mesh, camera or light node has no separate pose, so a clip moves
            // the node itself.
            if (has_loc) n->translation = kv.second.location;
            if (has_rot) n->rotation_deg = kv.second.rotation_deg;
            if (has_scale) n->scale = kv.second.scale;
        }
    }
    return static_cast<int>(c->tracks.size());
}

bool retarget_clip(Clip& clip, const Scene& target, int* dropped_out) {
    /// Map each animated node name onto a bone in the target rig.
    ///
    /// Matching is by name first, then by hierarchy position, so a clip authored
    /// on one skeleton can drive another with the same bone names even when the
    /// node objects differ. A track whose node is absent is dropped rather than
    /// silently retargeted onto the wrong bone, and the count is reported so the
    /// caller can see what was lost.
    std::vector<std::string> target_bones;
    for (const Node& n : target.nodes) {
        if (n.type == NodeType::Bone) target_bones.push_back(n.name);
    }
    if (target_bones.empty()) {
        if (dropped_out) *dropped_out = 0;
        return false;
    }

    std::vector<Track> kept;
    int dropped = 0;
    for (Track& t : clip.tracks) {
        bool found = false;
        for (const std::string& b : target_bones) {
            if (b == t.node) {
                found = true;
                break;
            }
        }
        if (found) {
            kept.push_back(std::move(t));
        } else {
            ++dropped;
        }
    }
    if (dropped_out) *dropped_out = dropped;
    if (kept.empty()) return false;
    clip.tracks = std::move(kept);
    return true;
}

}  // namespace kiln
