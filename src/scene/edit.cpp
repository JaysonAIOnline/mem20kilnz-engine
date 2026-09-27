#include "scene.hpp"

#include <algorithm>
#include <cmath>
#include <map>
#include <queue>
#include <set>
#include <tuple>
#include <unordered_map>
#include <unordered_set>

namespace kiln {
namespace {

uint64_t ekey(uint32_t a, uint32_t b) {
    if (a > b) std::swap(a, b);
    return (uint64_t(a) << 32) | uint64_t(b);
}

std::string lower(std::string s) {
    for (char& c : s) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    return s;
}

bool finite_f(float x) { return x == x && x > -3.0e38f && x < 3.0e38f; }

// An edit can shrink the mesh while a selection still refers to the old indices.
// Drop anything that no longer exists so a stale selection can never index past
// the end of m.positions (nor divide a centroid by a zero count).
std::vector<uint32_t> live_verts(const Mesh& m, const std::vector<uint32_t>& verts) {
    std::vector<uint32_t> out;
    out.reserve(verts.size());
    for (uint32_t v : verts)
        if (v < m.positions.size()) out.push_back(v);
    return out;
}

}  // namespace

Vec3 face_normal(const Mesh& m, int fi) {
    if (fi < 0 || fi >= static_cast<int>(m.faces.size())) return {0, 1, 0};
    const Face& f = m.faces[fi];
    if (f.v.size() < 3) return {0, 1, 0};
    Vec3 acc{0, 0, 0};
    Vec3 a = m.positions[f.v[0]];
    for (size_t i = 1; i + 1 < f.v.size(); ++i) {
        acc += cross(m.positions[f.v[i]] - a, m.positions[f.v[i + 1]] - a);
    }
    Vec3 n = normalize(acc);
    return length(n) > 0.5f ? n : Vec3{0, 1, 0};
}

Vec3 face_center(const Mesh& m, int fi) {
    const Face& f = m.faces[fi];
    Vec3 c{0, 0, 0};
    if (f.v.empty()) return c;
    for (uint32_t i : f.v) c += m.positions[i];
    return c / float(f.v.size());
}

/// Rebuild the render mesh (vertices + indices) from the editable mesh.
///
/// Corners are **welded**: two corners share a render vertex when their
/// position, normal and UV all match. This used to append three fresh vertices
/// per triangle unconditionally, which made every export a triangle soup --
/// an N-face mesh left as 3N unshared vertices, and the UV written was the
/// per-triangle ramp (0,0), (1,0), (1,1), which is not a texture coordinate at
/// all. Welding on the full attribute triple is what lets a UV seam or a split
/// normal survive, because those corners no longer match and stay apart.
///
/// When the mesh carries no real UVs, the UV is left at zero and `has_uv` is
/// cleared, so the exporter omits TEXCOORD_0 rather than publishing a
/// fabricated one.
void mesh_sync_render(Mesh& m) {
    m.vertices.clear();
    m.indices.clear();
    m.tri_face.clear();
    std::vector<Vec3> vn;
    if (m.smooth) vn = vertex_normals(m);

    // Corner attributes are only meaningful while they still line up with the
    // faces: three per face, in face order. Any op that changes the face count
    // without clearing them leaves a shorter or longer array, and reading it
    // anyway attributes one face's UVs and normals to another. That produced a
    // subdivided sphere with 1,104 boundary edges, 1,370 non-manifold edges and
    // a signed volume of exactly 0, because the stale normals cancelled.
    const bool corners_usable = m.corner_attr.size() == m.faces.size() * 3;
    if (!corners_usable) m.corner_attr.clear();

    bool any_uv = false;
    std::map<std::tuple<int, int, int, int, int, int, int>, uint32_t> weld;
    auto quant = [](float x) { return static_cast<int>(std::lround(x * 10000.0f)); };

    for (int fi = 0; fi < static_cast<int>(m.faces.size()); ++fi) {
        const Face& f = m.faces[fi];
        if (f.v.size() < 3) continue;
        const Vec3 fn = face_normal(m, fi);
        auto emit = [&](uint32_t ia, uint32_t ib, uint32_t ic) {
            const uint32_t tri[3] = {ia, ib, ic};
            for (int k = 0; k < 3; ++k) {
                const uint32_t i = tri[k];
                if (i >= m.positions.size()) continue;
                Vec3 n = fn;
                Vec2 uv{0, 0};
                // A corner's authored attributes win over the computed ones.
                const size_t corner = static_cast<size_t>(fi) * 3 + k;
                if (corners_usable && corner < m.corner_attr.size()) {
                    const CornerAttr& ca = m.corner_attr[corner];
                    n = ca.n;
                    uv = ca.uv;
                    if (ca.has_uv) any_uv = true;
                } else if (m.smooth && i < vn.size()) {
                    n = vn[i];
                }
                if (uv.x != 0.0f || uv.y != 0.0f) any_uv = true;
                const Vec3& p = m.positions[i];
                const auto key = std::make_tuple(quant(p.x), quant(p.y), quant(p.z),
                                                 quant(n.x), quant(n.y), quant(n.z),
                                                 quant(uv.x) ^ quant(uv.y));
                auto it = weld.find(key);
                uint32_t id;
                if (it != weld.end()) {
                    id = it->second;
                } else {
                    id = static_cast<uint32_t>(m.vertices.size());
                    m.vertices.push_back(Vertex{p, n, uv});
                    weld[key] = id;
                }
                m.indices.push_back(id);
            }
            m.tri_face.push_back(fi);
        };
        for (size_t i = 1; i + 1 < f.v.size(); ++i) emit(f.v[0], f.v[i], f.v[i + 1]);
    }
    m.has_uv = any_uv;
}

void mesh_compact(Mesh& m, std::vector<uint32_t>* remap_out) {
    if (m.positions.empty()) return;
    std::vector<int> used(m.positions.size(), 0);
    for (const auto& f : m.faces)
        for (uint32_t v : f.v)
            if (v < used.size()) used[v] = 1;
    std::vector<uint32_t> remap(m.positions.size(), 0);
    std::vector<Vec3> np;
    np.reserve(m.positions.size());
    for (size_t i = 0; i < m.positions.size(); ++i) {
        if (!used[i]) continue;
        remap[i] = static_cast<uint32_t>(np.size());
        np.push_back(m.positions[i]);
    }
    for (auto& f : m.faces)
        for (uint32_t& v : f.v) v = remap[v];
    m.positions = std::move(np);
    if (remap_out) *remap_out = std::move(remap);
}

void mesh_ensure_faces(Mesh& m) {
    if (!m.faces.empty()) {
        if (m.vertices.empty()) mesh_sync_render(m);
        return;
    }
    if (!m.positions.empty() && m.indices.size() >= 3) {
        for (size_t i = 0; i + 2 < m.indices.size(); i += 3) {
            m.faces.push_back({{m.indices[i], m.indices[i + 1], m.indices[i + 2]}});
        }
        mesh_sync_render(m);
        return;
    }
    if (m.vertices.empty() || m.indices.size() < 3) return;
    std::map<std::tuple<int, int, int>, uint32_t> weld;
    auto q = [](float x) { return int(std::round(x * 10000.0f)); };
    auto addp = [&](Vec3 p) {
        auto k = std::make_tuple(q(p.x), q(p.y), q(p.z));
        auto it = weld.find(k);
        if (it != weld.end()) return it->second;
        uint32_t id = static_cast<uint32_t>(m.positions.size());
        m.positions.push_back(p);
        weld[k] = id;
        return id;
    };
    for (size_t i = 0; i + 2 < m.indices.size(); i += 3) {
        Face f;
        f.v.push_back(addp(m.vertices[m.indices[i]].p));
        f.v.push_back(addp(m.vertices[m.indices[i + 1]].p));
        f.v.push_back(addp(m.vertices[m.indices[i + 2]].p));
        m.faces.push_back(std::move(f));
    }
    mesh_sync_render(m);
}

std::vector<int> faces_matching(const Mesh& m, const std::string& region) {
    std::string r = lower(region);
    Vec3 dir{0, 0, 0};
    bool all = r == "all" || r == "*";
    if (r == "top" || r == "up" || r == "+y") dir = {0, 1, 0};
    else if (r == "bottom" || r == "down" || r == "-y") dir = {0, -1, 0};
    else if (r == "right" || r == "+x") dir = {1, 0, 0};
    else if (r == "left" || r == "-x") dir = {-1, 0, 0};
    else if (r == "front" || r == "+z") dir = {0, 0, 1};
    else if (r == "back" || r == "-z") dir = {0, 0, -1};
    else if (!all) return {};
    std::vector<int> out;
    for (int i = 0; i < static_cast<int>(m.faces.size()); ++i) {
        if (all || dot(face_normal(m, i), dir) > 0.82f) out.push_back(i);
    }
    return out;
}

bool extrude_faces(Mesh& m, const std::vector<int>& faces, float amount, std::vector<int>& new_sel) {
    mesh_ensure_faces(m);
    if (faces.empty() || m.faces.empty()) return false;
    std::set<int> S;
    Vec3 nsum{0, 0, 0};
    for (int fi : faces) {
        if (fi < 0 || fi >= static_cast<int>(m.faces.size())) continue;
        S.insert(fi);
        nsum += face_normal(m, fi);
    }
    if (S.empty()) return false;
    Vec3 delta = normalize(nsum) * amount;

    std::set<uint32_t> used;
    std::map<uint64_t, std::pair<uint32_t, uint32_t>> boundary;
    std::map<uint64_t, int> ecount;
    for (int fi : S) {
        const Face& f = m.faces[fi];
        int nv = static_cast<int>(f.v.size());
        for (int i = 0; i < nv; ++i) {
            uint32_t a = f.v[i], b = f.v[(i + 1) % nv];
            used.insert(a);
            uint64_t k = ekey(a, b);
            ecount[k]++;
            if (ecount[k] == 1) boundary[k] = {a, b};
            else boundary.erase(k);
        }
    }

    std::map<uint32_t, uint32_t> dup;
    for (uint32_t v : used) {
        dup[v] = static_cast<uint32_t>(m.positions.size());
        m.positions.push_back(m.positions[v] + delta);
    }
    for (int fi : S) {
        for (uint32_t& v : m.faces[fi].v) v = dup[v];
    }
    for (auto& kv : boundary) {
        uint32_t a = kv.second.first, b = kv.second.second;
        Face q;
        q.v = {a, b, dup[b], dup[a]};
        m.faces.push_back(std::move(q));
    }
    mesh_compact(m);
    mesh_sync_render(m);
    new_sel.assign(S.begin(), S.end());
    return true;
}

bool inset_faces(Mesh& m, const std::vector<int>& faces, float amount, std::vector<int>& new_sel) {
    mesh_ensure_faces(m);
    if (faces.empty()) return false;
    float t = amount <= 0 ? 0.15f : amount;
    new_sel.clear();
    std::vector<int> sel = faces;
    for (int fi : sel) {
        if (fi < 0 || fi >= static_cast<int>(m.faces.size())) continue;
        Face f = m.faces[fi];
        if (f.v.size() < 3) continue;
        Vec3 c = face_center(m, fi);
        Vec3 n = face_normal(m, fi);
        std::vector<uint32_t> inner;
        inner.reserve(f.v.size());
        for (uint32_t vi : f.v) {
            Vec3 p = m.positions[vi];
            Vec3 d = p - c;
            d = d - n * dot(d, n);
            float len = length(d);
            float factor = len < 1e-6f ? 0.2f : std::max(0.08f, 1.0f - t / len);
            uint32_t ni = static_cast<uint32_t>(m.positions.size());
            m.positions.push_back(c + d * factor);
            inner.push_back(ni);
        }
        int nv = static_cast<int>(f.v.size());
        for (int i = 0; i < nv; ++i) {
            Face q;
            q.v = {f.v[i], f.v[(i + 1) % nv], inner[(i + 1) % nv], inner[i]};
            m.faces.push_back(std::move(q));
        }
        m.faces[fi].v = inner;
        new_sel.push_back(fi);
    }
    mesh_sync_render(m);
    return !new_sel.empty();
}

bool delete_faces_mesh(Mesh& m, const std::vector<int>& faces) {
    mesh_ensure_faces(m);
    if (faces.empty()) return false;
    std::vector<char> drop(m.faces.size(), 0);
    int n = 0;
    for (int fi : faces) {
        if (fi >= 0 && fi < static_cast<int>(m.faces.size()) && !drop[fi]) {
            drop[fi] = 1;
            ++n;
        }
    }
    if (!n) return false;
    std::vector<Face> kept;
    kept.reserve(m.faces.size() - n);
    for (size_t i = 0; i < m.faces.size(); ++i)
        if (!drop[i]) kept.push_back(std::move(m.faces[i]));
    m.faces = std::move(kept);
    mesh_compact(m);
    mesh_sync_render(m);
    return true;
}

std::vector<Edge> edges_of_faces(const Mesh& m, const std::vector<int>& faces) {
    std::set<uint64_t> seen;
    std::vector<Edge> out;
    for (int fi : faces) {
        if (fi < 0 || fi >= static_cast<int>(m.faces.size())) continue;
        const Face& f = m.faces[fi];
        int nv = static_cast<int>(f.v.size());
        for (int i = 0; i < nv; ++i) {
            Edge e = make_edge(f.v[i], f.v[(i + 1) % nv]);
            uint64_t k = ekey(e.a, e.b);
            if (seen.insert(k).second) out.push_back(e);
        }
    }
    return out;
}

namespace {

struct Topo {
    std::vector<std::vector<uint32_t>> nbr;
    std::map<uint64_t, std::vector<int>> edge_faces;
};

Topo build_topo(const Mesh& m) {
    Topo t;
    t.nbr.resize(m.positions.size());
    auto add_nbr = [&](uint32_t a, uint32_t b) {
        if (a >= t.nbr.size() || b >= t.nbr.size()) return;
        if (std::find(t.nbr[a].begin(), t.nbr[a].end(), b) == t.nbr[a].end()) t.nbr[a].push_back(b);
        if (std::find(t.nbr[b].begin(), t.nbr[b].end(), a) == t.nbr[b].end()) t.nbr[b].push_back(a);
    };
    for (int fi = 0; fi < static_cast<int>(m.faces.size()); ++fi) {
        const Face& f = m.faces[fi];
        int nv = static_cast<int>(f.v.size());
        for (int i = 0; i < nv; ++i) {
            uint32_t a = f.v[i], b = f.v[(i + 1) % nv];
            add_nbr(a, b);
            t.edge_faces[ekey(a, b)].push_back(fi);
        }
    }
    return t;
}

int face_edge_index(const Face& f, Edge e) {
    int nv = static_cast<int>(f.v.size());
    for (int i = 0; i < nv; ++i) {
        if (make_edge(f.v[i], f.v[(i + 1) % nv]) == e) return i;
    }
    return -1;
}

Edge opposite_on_quad(const Face& f, Edge e) {
    if (f.v.size() != 4) return {};
    int i = face_edge_index(f, e);
    if (i < 0) return {};
    return make_edge(f.v[(i + 2) % 4], f.v[(i + 3) % 4]);
}

int next_on_face(const Face& f, uint32_t prev, uint32_t curr) {
    int nv = static_cast<int>(f.v.size());
    for (int i = 0; i < nv; ++i) {
        if (f.v[i] == curr) {
            uint32_t p = f.v[(i + nv - 1) % nv];
            uint32_t n = f.v[(i + 1) % nv];
            if (p == prev) return int(n);
            if (n == prev) return int(p);
        }
    }
    return -1;
}

}  // namespace

std::vector<Edge> edge_ring(const Mesh& m, Edge seed) {
    Topo t = build_topo(m);
    uint64_t sk = ekey(seed.a, seed.b);
    if (!t.edge_faces.count(sk)) return {seed};
    std::set<uint64_t> seen;
    std::vector<Edge> out;
    std::queue<Edge> q;
    q.push(seed);
    seen.insert(sk);
    while (!q.empty()) {
        Edge e = q.front();
        q.pop();
        out.push_back(e);
        auto it = t.edge_faces.find(ekey(e.a, e.b));
        if (it == t.edge_faces.end()) continue;
        for (int fi : it->second) {
            if (m.faces[fi].v.size() != 4) continue;
            Edge opp = opposite_on_quad(m.faces[fi], e);
            uint64_t k = ekey(opp.a, opp.b);
            if (opp.a == opp.b) continue;
            if (seen.insert(k).second) q.push(opp);
        }
    }
    return out;
}

std::vector<Edge> edge_loop(const Mesh& m, Edge seed) {
    Topo t = build_topo(m);
    if (seed.a >= t.nbr.size() || seed.b >= t.nbr.size()) return {seed};
    auto step = [&](uint32_t prev, uint32_t curr) -> int {
        auto fit = t.edge_faces.find(ekey(prev, curr));
        std::set<uint32_t> side;
        if (fit != t.edge_faces.end()) {
            for (int fi : fit->second) {
                int nx = next_on_face(m.faces[fi], prev, curr);
                if (nx >= 0) side.insert(uint32_t(nx));
            }
        }
        int leftover = -1, nleft = 0;
        for (uint32_t nb : t.nbr[curr]) {
            if (nb == prev) continue;
            if (side.count(nb)) continue;
            leftover = int(nb);
            ++nleft;
        }
        if (nleft == 1) return leftover;
        if (fit != t.edge_faces.end() && !fit->second.empty())
            return next_on_face(m.faces[fit->second[0]], prev, curr);
        return -1;
    };

    std::vector<Edge> out;
    std::set<uint64_t> seen;
    auto walk = [&](uint32_t a, uint32_t b) {
        uint32_t prev = a, curr = b;
        for (int n = 0; n < 4096; ++n) {
            Edge e = make_edge(prev, curr);
            uint64_t k = ekey(e.a, e.b);
            if (!seen.insert(k).second) break;
            out.push_back(e);
            int nx = step(prev, curr);
            if (nx < 0) break;
            prev = curr;
            curr = uint32_t(nx);
            if (make_edge(prev, curr) == seed) break;
        }
    };
    walk(seed.a, seed.b);
    walk(seed.b, seed.a);
    if (out.empty()) out.push_back(seed);
    return out;
}

Edge pick_edge_axis(const Mesh& m, Vec3 axis) {
    axis = normalize(axis);
    Edge best{0, 0};
    float best_d = -1;
    for (const auto& f : m.faces) {
        int nv = int(f.v.size());
        for (int i = 0; i < nv; ++i) {
            uint32_t a = f.v[i], b = f.v[(i + 1) % nv];
            Vec3 d = m.positions[b] - m.positions[a];
            float l = length(d);
            if (l < 1e-8f) continue;
            float al = std::abs(dot(d / l, axis));
            if (al > best_d) {
                best_d = al;
                best = make_edge(a, b);
            }
        }
    }
    return best;
}

bool loop_cut(Mesh& m, Edge seed, int cuts, std::vector<Edge>& new_sel) {
    mesh_ensure_faces(m);
    if (cuts < 1) cuts = 1;
    if (cuts > 8) cuts = 8;
    auto ring = edge_ring(m, seed);
    if (ring.size() < 2) return false;

    std::map<uint64_t, std::vector<uint32_t>> on_edge;  // a->b order at increasing t
    std::map<uint64_t, uint32_t> orient_a;              // stored start vertex
    for (Edge e : ring) {
        orient_a[ekey(e.a, e.b)] = e.a;
        std::vector<uint32_t> ids;
        for (int i = 1; i <= cuts; ++i) {
            float t = float(i) / float(cuts + 1);
            uint32_t id = uint32_t(m.positions.size());
            m.positions.push_back(m.positions[e.a] * (1 - t) + m.positions[e.b] * t);
            ids.push_back(id);
        }
        on_edge[ekey(e.a, e.b)] = std::move(ids);
    }

    std::vector<Face> rebuilt;
    rebuilt.reserve(m.faces.size() + ring.size() * cuts);
    std::vector<Edge> loop_edges;
    std::set<int> split_faces;
    Topo t = build_topo(m);
    for (Edge e : ring) {
        auto it = t.edge_faces.find(ekey(e.a, e.b));
        if (it == t.edge_faces.end()) continue;
        for (int fi : it->second) split_faces.insert(fi);
    }

    for (int fi = 0; fi < int(m.faces.size()); ++fi) {
        if (!split_faces.count(fi) || m.faces[fi].v.size() != 4) {
            rebuilt.push_back(m.faces[fi]);
            continue;
        }
        const Face& f = m.faces[fi];
        int ring_i[2] = {-1, -1};
        int nring = 0;
        for (int i = 0; i < 4; ++i) {
            Edge e = make_edge(f.v[i], f.v[(i + 1) % 4]);
            if (on_edge.count(ekey(e.a, e.b))) {
                if (nring < 2) ring_i[nring] = i;
                ++nring;
            }
        }
        if (nring != 2 || (ring_i[1] - ring_i[0] + 4) % 4 != 2) {
            rebuilt.push_back(f);
            continue;
        }
        auto seq = [&](int ei) {
            uint32_t u = f.v[ei], v = f.v[(ei + 1) % 4];
            auto ids = on_edge[ekey(u, v)];
            if (orient_a[ekey(u, v)] != u) std::reverse(ids.begin(), ids.end());
            return ids;
        };
        auto sa = seq(ring_i[0]);
        auto sb = seq(ring_i[1]);
        std::reverse(sb.begin(), sb.end());
        uint32_t a0 = f.v[ring_i[0]];
        uint32_t a1 = f.v[(ring_i[0] + 1) % 4];
        uint32_t b0 = f.v[ring_i[1]];
        uint32_t b1 = f.v[(ring_i[1] + 1) % 4];
        std::vector<uint32_t> A = {a0};
        A.insert(A.end(), sa.begin(), sa.end());
        A.push_back(a1);
        std::vector<uint32_t> B = {b1};
        B.insert(B.end(), sb.begin(), sb.end());
        B.push_back(b0);
        for (int i = 0; i < cuts + 1; ++i) {
            Face q;
            q.v = {A[i], A[i + 1], B[i + 1], B[i]};
            rebuilt.push_back(std::move(q));
        }
        for (int i = 1; i <= cuts; ++i) loop_edges.push_back(make_edge(A[i], B[i]));
    }
    m.faces = std::move(rebuilt);
    std::vector<uint32_t> remap;
    mesh_compact(m, &remap);
    mesh_sync_render(m);
    new_sel.clear();
    std::set<uint64_t> seen;
    for (Edge e : loop_edges) {
        if (e.a < remap.size()) e.a = remap[e.a];
        if (e.b < remap.size()) e.b = remap[e.b];
        e = make_edge(e.a, e.b);
        if (seen.insert(ekey(e.a, e.b)).second) new_sel.push_back(e);
    }
    return true;
}

bool bevel_edges(Mesh& m, const std::vector<Edge>& edges, float amount, std::vector<Edge>& new_sel) {
    mesh_ensure_faces(m);
    if (edges.empty()) return false;
    std::set<uint64_t> E;
    for (Edge e : edges) E.insert(ekey(e.a, e.b));
    float minl = 1e9f;
    for (Edge e : edges) {
        float l = length(m.positions[e.b] - m.positions[e.a]);
        if (l > 1e-8f) minl = std::min(minl, l);
    }
    if (minl > 1e8f) return false;
    float t = std::max(0.001f, std::min(amount, minl * 0.45f));

    auto selected = [&](uint32_t a, uint32_t b) { return E.count(ekey(a, b)) > 0; };

    // (face, vert) -> new vert
    std::map<uint64_t, uint32_t> fv;
    auto fvkey = [](int fi, uint32_t v) { return (uint64_t(uint32_t(fi)) << 32) | v; };

    for (int fi = 0; fi < int(m.faces.size()); ++fi) {
        const Face& f = m.faces[fi];
        int nv = int(f.v.size());
        for (int i = 0; i < nv; ++i) {
            uint32_t v = f.v[i];
            uint32_t prev = f.v[(i + nv - 1) % nv];
            uint32_t next = f.v[(i + 1) % nv];
            bool ep = selected(prev, v), en = selected(v, next);
            if (!ep && !en) continue;
            Vec3 off{0, 0, 0};
            if (ep) off += normalize(m.positions[next] - m.positions[v]) * t;
            if (en) off += normalize(m.positions[prev] - m.positions[v]) * t;
            uint32_t id = uint32_t(m.positions.size());
            m.positions.push_back(m.positions[v] + off);
            fv[fvkey(fi, v)] = id;
        }
    }
    if (fv.empty()) return false;

    std::vector<Face> rebuilt;
    for (int fi = 0; fi < int(m.faces.size()); ++fi) {
        Face nf;
        const Face& f = m.faces[fi];
        for (uint32_t v : f.v) {
            auto it = fv.find(fvkey(fi, v));
            nf.v.push_back(it == fv.end() ? v : it->second);
        }
        if (nf.v.size() >= 3) rebuilt.push_back(std::move(nf));
    }

    Topo topo = build_topo(m);
    new_sel.clear();
    for (Edge e : edges) {
        auto it = topo.edge_faces.find(ekey(e.a, e.b));
        if (it == topo.edge_faces.end() || it->second.size() < 1) continue;
        if (it->second.size() == 1) {
            int f1 = it->second[0];
            auto a1 = fv.find(fvkey(f1, e.a));
            auto b1 = fv.find(fvkey(f1, e.b));
            if (a1 == fv.end() || b1 == fv.end()) continue;
            new_sel.push_back(make_edge(a1->second, b1->second));
            continue;
        }
        int f1 = it->second[0], f2 = it->second[1];
        auto a1 = fv.find(fvkey(f1, e.a));
        auto b1 = fv.find(fvkey(f1, e.b));
        auto a2 = fv.find(fvkey(f2, e.a));
        auto b2 = fv.find(fvkey(f2, e.b));
        if (a1 == fv.end() || b1 == fv.end() || a2 == fv.end() || b2 == fv.end()) continue;
        Face q;
        q.v = {a1->second, b1->second, b2->second, a2->second};
        Vec3 want = face_normal(m, f1) + face_normal(m, f2);
        Vec3 qn = cross(m.positions[b1->second] - m.positions[a1->second],
                        m.positions[a2->second] - m.positions[a1->second]);
        if (dot(qn, want) < 0) std::reverse(q.v.begin(), q.v.end());
        rebuilt.push_back(q);
        new_sel.push_back(make_edge(a1->second, b1->second));
    }

    std::map<uint32_t, std::vector<uint32_t>> corners;
    for (auto& kv : fv) {
        uint32_t v = uint32_t(kv.first & 0xffffffffu);
        corners[v].push_back(kv.second);
    }
    Vec3 origin{0, 0, 0};
    for (const auto& p : m.positions) origin += p;
    if (!m.positions.empty()) origin = origin / float(m.positions.size());
    for (auto& kv : corners) {
        std::vector<uint32_t> ids;
        ids.reserve(kv.second.size());
        for (uint32_t id : kv.second) {
            bool dupe = false;
            for (uint32_t u : ids) {
                if (length(m.positions[u] - m.positions[id]) < t * 0.05f) {
                    dupe = true;
                    break;
                }
            }
            if (!dupe) ids.push_back(id);
        }
        if (ids.size() < 3) continue;
        Vec3 c = m.positions[kv.first];
        Vec3 nn = c - origin;
        if (length(nn) < 1e-6f) {
            nn = {0, 0, 0};
            for (uint32_t id : ids) nn += m.positions[id] - c;
        }
        nn = normalize(nn);
        if (length(nn) < 0.5f) nn = {0, 1, 0};
        Vec3 ref = std::abs(nn.y) < 0.9f ? Vec3{0, 1, 0} : Vec3{1, 0, 0};
        Vec3 x = normalize(cross(ref, nn));
        Vec3 y = cross(nn, x);
        std::sort(ids.begin(), ids.end(), [&](uint32_t p, uint32_t qid) {
            Vec3 dp = m.positions[p] - c, dq = m.positions[qid] - c;
            return std::atan2(dot(dp, y), dot(dp, x)) < std::atan2(dot(dq, y), dot(dq, x));
        });
        Vec3 qn = cross(m.positions[ids[1]] - m.positions[ids[0]], m.positions[ids[2]] - m.positions[ids[0]]);
        if (dot(qn, nn) < 0) std::reverse(ids.begin(), ids.end());
        Face cf;
        cf.v = std::move(ids);
        rebuilt.push_back(std::move(cf));
    }

    m.faces = std::move(rebuilt);
    std::vector<uint32_t> remap;
    mesh_compact(m, &remap);
    mesh_sync_render(m);
    for (auto& e : new_sel) {
        if (e.a < remap.size()) e.a = remap[e.a];
        if (e.b < remap.size()) e.b = remap[e.b];
        if (e.a > e.b) std::swap(e.a, e.b);
    }
    return true;
}

std::vector<uint32_t> verts_of_faces(const Mesh& m, const std::vector<int>& faces) {
    std::set<uint32_t> s;
    for (int fi : faces) {
        if (fi < 0 || fi >= int(m.faces.size())) continue;
        for (uint32_t v : m.faces[fi].v) s.insert(v);
    }
    return {s.begin(), s.end()};
}

std::vector<uint32_t> verts_matching(const Mesh& m, const std::string& region) {
    return verts_of_faces(m, faces_matching(m, region));
}

bool translate_verts(Mesh& m, const std::vector<uint32_t>& verts, Vec3 delta) {
    if (verts.empty()) return false;
    bool any = false;
    for (uint32_t v : verts) {
        if (v >= m.positions.size()) continue;
        m.positions[v] += delta;
        any = true;
    }
    if (any) mesh_sync_render(m);
    return any;
}

bool subdivide_faces(Mesh& m, const std::vector<int>& faces, int cuts, std::vector<int>& new_sel) {
    mesh_ensure_faces(m);
    if (m.faces.empty()) return false;
    if (cuts < 1) cuts = 1;
    if (cuts > 3) cuts = 3;
    std::set<int> split;
    if (faces.empty()) {
        for (int i = 0; i < int(m.faces.size()); ++i) split.insert(i);
    } else {
        for (int fi : faces)
            if (fi >= 0 && fi < int(m.faces.size())) split.insert(fi);
    }
    if (split.empty()) return false;

    for (int pass = 0; pass < cuts; ++pass) {
        std::map<uint64_t, uint32_t> mid;
        auto midpoint = [&](uint32_t a, uint32_t b) {
            uint64_t k = ekey(a, b);
            auto it = mid.find(k);
            if (it != mid.end()) return it->second;
            uint32_t id = uint32_t(m.positions.size());
            m.positions.push_back((m.positions[a] + m.positions[b]) * 0.5f);
            mid[k] = id;
            return id;
        };
        for (int fi : split) {
            const Face& f = m.faces[fi];
            int nv = int(f.v.size());
            for (int i = 0; i < nv; ++i) midpoint(f.v[i], f.v[(i + 1) % nv]);
        }
        std::vector<Face> rebuilt;
        rebuilt.reserve(m.faces.size() * 2);
        std::set<int> next_split;
        for (int fi = 0; fi < int(m.faces.size()); ++fi) {
            const Face& f = m.faces[fi];
            int nv = int(f.v.size());
            if (!split.count(fi)) {
                Face nf;
                for (int i = 0; i < nv; ++i) {
                    uint32_t a = f.v[i], b = f.v[(i + 1) % nv];
                    nf.v.push_back(a);
                    auto it = mid.find(ekey(a, b));
                    if (it != mid.end()) nf.v.push_back(it->second);
                }
                if (nf.v.size() >= 3) rebuilt.push_back(std::move(nf));
                continue;
            }
            Vec3 c{0, 0, 0};
            for (uint32_t v : f.v) c += m.positions[v];
            c = c / float(nv);
            uint32_t cid = uint32_t(m.positions.size());
            m.positions.push_back(c);
            std::vector<uint32_t> mids(nv);
            for (int i = 0; i < nv; ++i) mids[i] = midpoint(f.v[i], f.v[(i + 1) % nv]);
            int start = int(rebuilt.size());
            for (int i = 0; i < nv; ++i) {
                int p = (i + nv - 1) % nv;
                Face q;
                q.v = {f.v[i], mids[i], cid, mids[p]};
                rebuilt.push_back(std::move(q));
            }
            for (int k = start; k < int(rebuilt.size()); ++k) next_split.insert(k);
        }
        m.faces = std::move(rebuilt);
        split = std::move(next_split);
    }
    mesh_compact(m);
    mesh_sync_render(m);
    new_sel.assign(split.begin(), split.end());
    return !new_sel.empty();
}

bool merge_verts(Mesh& m, const std::vector<uint32_t>& verts, uint32_t& survivor) {
    mesh_ensure_faces(m);
    std::vector<uint32_t> vs;
    for (uint32_t v : verts)
        if (v < m.positions.size()) vs.push_back(v);
    std::sort(vs.begin(), vs.end());
    vs.erase(std::unique(vs.begin(), vs.end()), vs.end());
    if (vs.size() < 2) return false;
    Vec3 c{0, 0, 0};
    for (uint32_t v : vs) c += m.positions[v];
    c = c / float(vs.size());
    survivor = vs[0];
    m.positions[survivor] = c;
    std::vector<uint32_t> to(m.positions.size());
    for (size_t i = 0; i < to.size(); ++i) to[i] = uint32_t(i);
    for (uint32_t v : vs) to[v] = survivor;
    std::vector<Face> kept;
    for (auto& f : m.faces) {
        Face nf;
        for (uint32_t v : f.v) {
            uint32_t w = to[v];
            if (nf.v.empty() || nf.v.back() != w) nf.v.push_back(w);
        }
        if (nf.v.size() >= 2 && nf.v.front() == nf.v.back()) nf.v.pop_back();
        if (nf.v.size() >= 3) kept.push_back(std::move(nf));
    }
    m.faces = std::move(kept);
    std::vector<uint32_t> remap;
    mesh_compact(m, &remap);
    if (survivor < remap.size()) survivor = remap[survivor];
    mesh_sync_render(m);
    return true;
}

bool fill_loops(Mesh& m, const std::vector<Edge>& edges, std::vector<int>& new_sel) {
    mesh_ensure_faces(m);
    Topo t = build_topo(m);
    std::set<uint64_t> want;
    if (!edges.empty()) {
        for (Edge e : edges) want.insert(ekey(e.a, e.b));
    } else {
        for (auto& kv : t.edge_faces)
            if (kv.second.size() == 1) want.insert(kv.first);
    }
    if (want.empty()) return false;

    auto has_face = [&](const std::vector<uint32_t>& loop) {
        std::set<uint32_t> S(loop.begin(), loop.end());
        for (const auto& f : m.faces) {
            if (f.v.size() != S.size()) continue;
            std::set<uint32_t> F(f.v.begin(), f.v.end());
            if (F == S) return true;
        }
        return false;
    };

    std::map<uint32_t, std::vector<uint32_t>> adj;
    for (uint64_t k : want) {
        uint32_t a = uint32_t(k >> 32), b = uint32_t(k);
        adj[a].push_back(b);
        adj[b].push_back(a);
    }

    Vec3 origin{0, 0, 0};
    for (const auto& p : m.positions) origin += p;
    if (!m.positions.empty()) origin = origin / float(m.positions.size());

    std::set<uint64_t> used;
    new_sel.clear();
    for (auto& kv : adj) {
        uint32_t start = kv.first;
        for (uint32_t nb0 : kv.second) {
            uint64_t sk = ekey(start, nb0);
            if (used.count(sk)) continue;
            std::vector<uint32_t> loop;
            uint32_t prev = start, curr = nb0;
            loop.push_back(start);
            bool closed = false;
            for (int n = 0; n < 4096; ++n) {
                used.insert(ekey(prev, curr));
                loop.push_back(curr);
                uint32_t nxt = curr;
                bool found = false;
                for (uint32_t nb : adj[curr]) {
                    if (nb == prev) continue;
                    if (used.count(ekey(curr, nb)) && nb != start) continue;
                    if (nb == start && loop.size() >= 3) {
                        closed = true;
                        used.insert(ekey(curr, start));
                        found = true;
                        break;
                    }
                    if (!used.count(ekey(curr, nb))) {
                        nxt = nb;
                        found = true;
                        break;
                    }
                }
                if (closed) break;
                if (!found || nxt == curr) break;
                prev = curr;
                curr = nxt;
            }
            if (!closed || loop.size() < 3) continue;
            if (loop.back() == loop.front()) loop.pop_back();
            if (loop.size() < 3 || has_face(loop)) continue;
            Face f;
            f.v = loop;
            m.faces.push_back(f);
            int fi = int(m.faces.size()) - 1;
            Vec3 n = face_normal(m, fi);
            Vec3 fc = face_center(m, fi);
            if (dot(n, fc - origin) < 0) std::reverse(m.faces[fi].v.begin(), m.faces[fi].v.end());
            new_sel.push_back(fi);
        }
    }
    if (new_sel.empty()) return false;
    mesh_sync_render(m);
    return true;
}

bool mirror_mesh(Mesh& m, int axis) {
    mesh_ensure_faces(m);
    if (m.positions.empty() || m.faces.empty()) return false;
    axis = axis < 0 ? 0 : (axis > 2 ? 2 : axis);
    auto coord = [&](Vec3 p) { return axis == 0 ? p.x : axis == 1 ? p.y : p.z; };
    auto flip = [&](Vec3 p) {
        if (axis == 0) p.x = -p.x;
        else if (axis == 1) p.y = -p.y;
        else p.z = -p.z;
        return p;
    };
    const float eps = 1e-4f;
    size_t n0 = m.positions.size();
    std::vector<uint32_t> mir(n0);
    for (size_t i = 0; i < n0; ++i) {
        if (std::abs(coord(m.positions[i])) < eps) {
            mir[i] = uint32_t(i);
            continue;
        }
        mir[i] = uint32_t(m.positions.size());
        m.positions.push_back(flip(m.positions[i]));
    }
    std::vector<Face> extra;
    extra.reserve(m.faces.size());
    for (const auto& f : m.faces) {
        Face nf;
        nf.v.resize(f.v.size());
        bool same = true;
        for (size_t i = 0; i < f.v.size(); ++i) {
            nf.v[f.v.size() - 1 - i] = mir[f.v[i]];
            if (mir[f.v[i]] != f.v[i]) same = false;
        }
        if (!same && nf.v.size() >= 3) extra.push_back(std::move(nf));
    }
    m.faces.insert(m.faces.end(), extra.begin(), extra.end());

    auto q = [](float x) { return int(std::round(x * 10000.0f)); };
    std::map<std::tuple<int, int, int>, uint32_t> weld;
    std::vector<uint32_t> to(m.positions.size());
    std::vector<Vec3> np;
    for (size_t i = 0; i < m.positions.size(); ++i) {
        Vec3 p = m.positions[i];
        auto k = std::make_tuple(q(p.x), q(p.y), q(p.z));
        auto it = weld.find(k);
        if (it == weld.end()) {
            to[i] = uint32_t(np.size());
            weld[k] = to[i];
            np.push_back(p);
        } else
            to[i] = it->second;
    }
    std::set<std::vector<uint32_t>> seen;
    std::vector<Face> kept;
    for (auto& f : m.faces) {
        Face nf;
        for (uint32_t v : f.v) {
            uint32_t w = to[v];
            if (nf.v.empty() || nf.v.back() != w) nf.v.push_back(w);
        }
        if (nf.v.size() >= 2 && nf.v.front() == nf.v.back()) nf.v.pop_back();
        if (nf.v.size() < 3) continue;
        std::vector<uint32_t> key = nf.v;
        std::sort(key.begin(), key.end());
        if (!seen.insert(key).second) continue;
        kept.push_back(std::move(nf));
    }
    m.positions = std::move(np);
    m.faces = std::move(kept);
    mesh_compact(m);
    mesh_sync_render(m);
    return !m.faces.empty();
}

bool solidify_mesh(Mesh& m, float thickness) {
    mesh_ensure_faces(m);
    if (m.faces.empty() || m.positions.empty()) return false;
    float t = thickness == 0 ? 0.08f : thickness;
    size_t n0 = m.positions.size();
    std::vector<Vec3> vn(n0, {0, 0, 0});
    for (int fi = 0; fi < int(m.faces.size()); ++fi) {
        Vec3 n = face_normal(m, fi);
        for (uint32_t v : m.faces[fi].v)
            if (v < n0) vn[v] += n;
    }
    for (size_t i = 0; i < n0; ++i) {
        vn[i] = normalize(vn[i]);
        if (length(vn[i]) < 0.5f) vn[i] = {0, 1, 0};
        m.positions.push_back(m.positions[i] - vn[i] * t);
    }
    Topo topo = build_topo(m);
    int nfaces = int(m.faces.size());
    for (int fi = 0; fi < nfaces; ++fi) {
        Face inner;
        const Face& f = m.faces[fi];
        inner.v.reserve(f.v.size());
        for (int i = int(f.v.size()) - 1; i >= 0; --i) inner.v.push_back(f.v[i] + uint32_t(n0));
        m.faces.push_back(std::move(inner));
    }
    for (auto& kv : topo.edge_faces) {
        if (kv.second.size() != 1) continue;
        uint32_t a = uint32_t(kv.first >> 32), b = uint32_t(kv.first);
        const Face& f = m.faces[kv.second[0]];
        int nv = int(f.v.size());
        bool fwd = false;
        for (int i = 0; i < nv; ++i) {
            if (f.v[i] == a && f.v[(i + 1) % nv] == b) {
                fwd = true;
                break;
            }
        }
        if (!fwd) std::swap(a, b);
        Face rim;
        rim.v = {a, b, b + uint32_t(n0), a + uint32_t(n0)};
        m.faces.push_back(std::move(rim));
    }
    mesh_sync_render(m);
    return true;
}

bool array_mesh(Mesh& m, int count, Vec3 offset) {
    mesh_ensure_faces(m);
    if (m.faces.empty() || m.positions.empty()) return false;
    if (count < 2) count = 2;
    if (count > 16) count = 16;
    size_t n0 = m.positions.size();
    size_t f0 = m.faces.size();
    m.positions.reserve(n0 * count);
    m.faces.reserve(f0 * count);
    for (int i = 1; i < count; ++i) {
        uint32_t base = uint32_t(m.positions.size());
        for (size_t v = 0; v < n0; ++v) m.positions.push_back(m.positions[v] + offset * float(i));
        for (size_t fi = 0; fi < f0; ++fi) {
            Face nf;
            nf.v.reserve(m.faces[fi].v.size());
            for (uint32_t vx : m.faces[fi].v) nf.v.push_back(vx + base);
            m.faces.push_back(std::move(nf));
        }
    }
    mesh_sync_render(m);
    return true;
}

Vec3 mesh_centroid(const Mesh& m) {
    Vec3 c{0, 0, 0};
    if (m.positions.empty()) return c;
    for (const auto& p : m.positions) c += p;
    return c / float(m.positions.size());
}

void mesh_append_transformed(Mesh& dst, const Mesh& src, const Mat4& xform) {
    uint32_t base = uint32_t(dst.positions.size());
    dst.positions.reserve(dst.positions.size() + src.positions.size());
    for (const auto& p : src.positions) dst.positions.push_back(transform_point(xform, p));
    dst.faces.reserve(dst.faces.size() + src.faces.size());
    for (const auto& f : src.faces) {
        Face nf;
        nf.v.reserve(f.v.size());
        for (uint32_t v : f.v) nf.v.push_back(v + base);
        if (nf.v.size() >= 3) dst.faces.push_back(std::move(nf));
    }
}

bool spin_mesh(Mesh& m, const std::vector<Edge>& profile, int steps, float angle_deg, Vec3 axis, Vec3 center) {
    mesh_ensure_faces(m);
    if (m.positions.empty()) return false;
    if (steps < 2) steps = 2;
    if (steps > 32) steps = 32;
    float ang = angle_deg == 0 ? 360.0f : angle_deg;
    bool closed = std::abs(std::abs(ang) - 360.0f) < 0.5f;
    int copies = closed ? steps : steps + 1;
    float step_rad = deg2rad(ang / float(steps));
    axis = normalize(axis);
    if (length(axis) < 0.5f) axis = {0, 1, 0};

    if (profile.empty()) {
        size_t n0 = m.positions.size();
        size_t f0 = m.faces.size();
        std::vector<Vec3> base = m.positions;
        for (int i = 1; i < copies; ++i) {
            uint32_t off = uint32_t(m.positions.size());
            float rad = step_rad * float(i);
            for (size_t v = 0; v < n0; ++v)
                m.positions.push_back(rotate_around(base[v], center, axis, rad));
            for (size_t fi = 0; fi < f0; ++fi) {
                Face nf;
                for (uint32_t vx : m.faces[fi].v) nf.v.push_back(vx + off);
                m.faces.push_back(std::move(nf));
            }
        }
        mesh_sync_render(m);
        return true;
    }

    std::set<uint32_t> pv;
    for (Edge e : profile) {
        pv.insert(e.a);
        pv.insert(e.b);
    }
    std::vector<uint32_t> verts(pv.begin(), pv.end());
    if (verts.size() < 2) return false;
    std::map<uint32_t, std::vector<uint32_t>> rings;
    for (uint32_t v : verts) {
        rings[v].push_back(v);
        for (int i = 1; i < copies; ++i) {
            uint32_t id = uint32_t(m.positions.size());
            m.positions.push_back(rotate_around(m.positions[v], center, axis, step_rad * float(i)));
            rings[v].push_back(id);
        }
        if (closed) rings[v].push_back(v);
    }
    int segs = closed ? steps : steps;
    for (Edge e : profile) {
        auto& A = rings[e.a];
        auto& B = rings[e.b];
        if (A.size() < 2 || B.size() < 2) continue;
        for (int i = 0; i < segs; ++i) {
            Face q;
            q.v = {A[i], B[i], B[i + 1], A[i + 1]};
            m.faces.push_back(std::move(q));
        }
    }
    mesh_sync_render(m);
    return true;
}

namespace {

std::vector<std::vector<uint32_t>> loops_from_edge_set(const std::set<uint64_t>& want) {
    std::map<uint32_t, std::vector<uint32_t>> adj;
    for (uint64_t k : want) {
        uint32_t a = uint32_t(k >> 32), b = uint32_t(k);
        adj[a].push_back(b);
        adj[b].push_back(a);
    }
    std::set<uint64_t> used;
    std::vector<std::vector<uint32_t>> loops;
    for (auto& kv : adj) {
        uint32_t start = kv.first;
        for (uint32_t nb0 : kv.second) {
            if (used.count(ekey(start, nb0))) continue;
            std::vector<uint32_t> loop;
            uint32_t prev = start, curr = nb0;
            loop.push_back(start);
            bool closed = false;
            for (int n = 0; n < 4096; ++n) {
                used.insert(ekey(prev, curr));
                loop.push_back(curr);
                bool found = false;
                uint32_t nxt = curr;
                for (uint32_t nb : adj[curr]) {
                    if (nb == prev) continue;
                    if (nb == start && loop.size() >= 3) {
                        closed = true;
                        used.insert(ekey(curr, start));
                        found = true;
                        break;
                    }
                    if (!used.count(ekey(curr, nb))) {
                        nxt = nb;
                        found = true;
                        break;
                    }
                }
                if (closed) break;
                if (!found || nxt == curr) break;
                prev = curr;
                curr = nxt;
            }
            if (!closed || loop.size() < 3) continue;
            if (loop.back() == loop.front()) loop.pop_back();
            if (loop.size() >= 3) loops.push_back(std::move(loop));
        }
    }
    return loops;
}

}  // namespace

bool bridge_loops(Mesh& m, const std::vector<int>& faces, const std::vector<Edge>& edges, std::vector<int>& new_sel) {
    mesh_ensure_faces(m);
    std::vector<std::vector<uint32_t>> loops;
    std::vector<int> drop;
    if (faces.size() >= 2) {
        for (int fi : faces) {
            if (fi < 0 || fi >= int(m.faces.size())) continue;
            if (m.faces[fi].v.size() < 3) continue;
            loops.push_back(m.faces[fi].v);
            drop.push_back(fi);
            if (loops.size() == 2) break;
        }
    }
    if (loops.size() < 2) {
        std::set<uint64_t> want;
        if (!edges.empty()) {
            for (Edge e : edges) want.insert(ekey(e.a, e.b));
        } else {
            Topo t = build_topo(m);
            for (auto& kv : t.edge_faces)
                if (kv.second.size() == 1) want.insert(kv.first);
        }
        loops = loops_from_edge_set(want);
    }
    if (loops.size() < 2) return false;
    auto a = loops[0], b = loops[1];
    if (a.size() != b.size() || a.size() < 3) return false;
    int n = int(a.size());
    auto score = [&](const std::vector<uint32_t>& A, const std::vector<uint32_t>& B, int off, bool rev) {
        float s = 0;
        for (int i = 0; i < n; ++i) {
            int j = (off + i) % n;
            if (rev) j = (off - i + n) % n;
            s += length(m.positions[A[i]] - m.positions[B[j]]);
        }
        return s;
    };
    int best_off = 0;
    bool best_rev = false;
    float best = 1e18f;
    for (int rev = 0; rev < 2; ++rev)
        for (int off = 0; off < n; ++off) {
            float s = score(a, b, off, rev);
            if (s < best) {
                best = s;
                best_off = off;
                best_rev = rev;
            }
        }
    std::vector<uint32_t> br(n);
    for (int i = 0; i < n; ++i) {
        int j = (best_off + i) % n;
        if (best_rev) j = (best_off - i + n) % n;
        br[i] = b[j];
    }
    if (!drop.empty()) {
        std::vector<char> kill(m.faces.size(), 0);
        for (int fi : drop)
            if (fi >= 0 && fi < int(kill.size())) kill[fi] = 1;
        std::vector<Face> kept;
        for (size_t i = 0; i < m.faces.size(); ++i)
            if (!kill[i]) kept.push_back(std::move(m.faces[i]));
        m.faces = std::move(kept);
    }
    new_sel.clear();
    Vec3 origin = mesh_centroid(m);
    for (int i = 0; i < n; ++i) {
        Face q;
        q.v = {a[i], a[(i + 1) % n], br[(i + 1) % n], br[i]};
        m.faces.push_back(q);
        int fi = int(m.faces.size()) - 1;
        Vec3 nn = face_normal(m, fi);
        Vec3 fc = face_center(m, fi);
        if (dot(nn, fc - origin) < 0) std::reverse(m.faces[fi].v.begin(), m.faces[fi].v.end());
        new_sel.push_back(fi);
    }
    mesh_sync_render(m);
    return true;
}

std::vector<Vec3> vertex_normals(const Mesh& m) {
    std::vector<Vec3> n(m.positions.size(), {0, 0, 0});
    for (int fi = 0; fi < int(m.faces.size()); ++fi) {
        Vec3 fn = face_normal(m, fi);
        for (uint32_t v : m.faces[fi].v)
            if (v < n.size()) n[v] += fn;
    }
    for (auto& x : n) {
        x = normalize(x);
        if (length(x) < 0.5f) x = {0, 1, 0};
    }
    return n;
}

static std::vector<int> faces_or_all(const Mesh& m, const std::vector<int>& faces) {
    if (!faces.empty()) return faces;
    std::vector<int> all(m.faces.size());
    for (int i = 0; i < int(all.size()); ++i) all[i] = i;
    return all;
}

bool triangulate_faces(Mesh& m, const std::vector<int>& faces) {
    mesh_ensure_faces(m);
    auto S = faces_or_all(m, faces);
    std::vector<char> tri(m.faces.size(), 0);
    for (int fi : S)
        if (fi >= 0 && fi < int(m.faces.size())) tri[fi] = 1;
    std::vector<Face> extra;
    for (int fi = 0; fi < int(m.faces.size()); ++fi) {
        if (!tri[fi] || m.faces[fi].v.size() < 4) continue;
        Face f = m.faces[fi];
        m.faces[fi].v = {f.v[0], f.v[1], f.v[2]};
        for (size_t i = 2; i + 1 < f.v.size(); ++i) extra.push_back(Face{{f.v[0], f.v[i], f.v[i + 1]}});
    }
    m.faces.insert(m.faces.end(), extra.begin(), extra.end());
    mesh_sync_render(m);
    return true;
}

bool poke_faces(Mesh& m, const std::vector<int>& faces, std::vector<int>& new_sel) {
    mesh_ensure_faces(m);
    auto S = faces_or_all(m, faces);
    std::vector<Face> rebuilt;
    new_sel.clear();
    std::set<int> poke(S.begin(), S.end());
    for (int fi = 0; fi < int(m.faces.size()); ++fi) {
        const Face& f = m.faces[fi];
        if (!poke.count(fi) || f.v.size() < 3) {
            if (f.v.size() >= 3) rebuilt.push_back(f);
            continue;
        }
        Vec3 c = face_center(m, fi);
        uint32_t cid = uint32_t(m.positions.size());
        m.positions.push_back(c);
        int start = int(rebuilt.size());
        int nv = int(f.v.size());
        for (int i = 0; i < nv; ++i) {
            Face t;
            t.v = {f.v[i], f.v[(i + 1) % nv], cid};
            rebuilt.push_back(std::move(t));
        }
        for (int k = start; k < int(rebuilt.size()); ++k) new_sel.push_back(k);
    }
    m.faces = std::move(rebuilt);
    mesh_sync_render(m);
    return !new_sel.empty();
}

bool flip_faces(Mesh& m, const std::vector<int>& faces) {
    mesh_ensure_faces(m);
    auto S = faces_or_all(m, faces);
    bool any = false;
    for (int fi : S) {
        if (fi < 0 || fi >= int(m.faces.size())) continue;
        std::reverse(m.faces[fi].v.begin(), m.faces[fi].v.end());
        any = true;
    }
    if (any) mesh_sync_render(m);
    return any;
}

bool dissolve_faces(Mesh& m, const std::vector<int>& faces, std::vector<int>& new_sel) {
    mesh_ensure_faces(m);
    if (faces.empty()) return false;
    std::set<int> S;
    for (int fi : faces)
        if (fi >= 0 && fi < int(m.faces.size())) S.insert(fi);
    if (S.empty()) return false;
    std::map<uint64_t, int> ecount;
    std::map<uint64_t, std::pair<uint32_t, uint32_t>> directed;
    for (int fi : S) {
        const Face& f = m.faces[fi];
        int nv = int(f.v.size());
        for (int i = 0; i < nv; ++i) {
            uint32_t a = f.v[i], b = f.v[(i + 1) % nv];
            uint64_t k = ekey(a, b);
            ecount[k]++;
            if (ecount[k] == 1) directed[k] = {a, b};
            else directed.erase(k);
        }
    }
    std::set<uint64_t> bound;
    for (auto& kv : ecount)
        if (kv.second == 1) bound.insert(kv.first);
    auto loops = loops_from_edge_set(bound);
    std::vector<char> kill(m.faces.size(), 0);
    for (int fi : S) kill[fi] = 1;
    std::vector<Face> kept;
    for (size_t i = 0; i < m.faces.size(); ++i)
        if (!kill[i]) kept.push_back(std::move(m.faces[i]));
    m.faces = std::move(kept);
    new_sel.clear();
    for (auto& lp : loops) {
        if (lp.size() < 3) continue;
        Face nf;
        nf.v = lp;
        m.faces.push_back(std::move(nf));
        new_sel.push_back(int(m.faces.size()) - 1);
    }
    mesh_compact(m);
    mesh_sync_render(m);
    return !new_sel.empty() || S.size() > 0;
}

bool duplicate_faces_mesh(Mesh& m, const std::vector<int>& faces, float amount, std::vector<int>& new_sel) {
    mesh_ensure_faces(m);
    auto S = faces;
    if (S.empty()) return false;
    std::set<uint32_t> used;
    Vec3 nsum{0, 0, 0};
    for (int fi : S) {
        if (fi < 0 || fi >= int(m.faces.size())) continue;
        nsum += face_normal(m, fi);
        for (uint32_t v : m.faces[fi].v) used.insert(v);
    }
    if (used.empty()) return false;
    Vec3 off = normalize(nsum) * amount;
    std::map<uint32_t, uint32_t> dup;
    for (uint32_t v : used) {
        dup[v] = uint32_t(m.positions.size());
        m.positions.push_back(m.positions[v] + off);
    }
    new_sel.clear();
    for (int fi : S) {
        if (fi < 0 || fi >= int(m.faces.size())) continue;
        Face nf;
        for (uint32_t v : m.faces[fi].v) nf.v.push_back(dup[v]);
        m.faces.push_back(std::move(nf));
        new_sel.push_back(int(m.faces.size()) - 1);
    }
    mesh_sync_render(m);
    return !new_sel.empty();
}

bool split_region(Mesh& m, const std::vector<int>& faces) {
    mesh_ensure_faces(m);
    if (faces.empty()) return false;
    std::set<uint32_t> used;
    std::set<int> S;
    for (int fi : faces) {
        if (fi < 0 || fi >= int(m.faces.size())) continue;
        S.insert(fi);
        for (uint32_t v : m.faces[fi].v) used.insert(v);
    }
    if (used.empty()) return false;
    std::map<uint32_t, uint32_t> dup;
    for (uint32_t v : used) {
        dup[v] = uint32_t(m.positions.size());
        m.positions.push_back(m.positions[v]);
    }
    for (int fi : S)
        for (uint32_t& v : m.faces[fi].v) v = dup[v];
    mesh_sync_render(m);
    return true;
}

bool weld_distance(Mesh& m, float dist) {
    mesh_ensure_faces(m);
    if (m.positions.size() < 2) return false;
    float e = dist <= 0 ? 0.0001f : dist;
    int qn = int(std::max(1.0f, 1.0f / e));
    auto q = [&](float x) { return int(std::round(x * float(qn))); };
    std::map<std::tuple<int, int, int>, uint32_t> weld;
    std::vector<uint32_t> to(m.positions.size());
    std::vector<Vec3> np;
    for (size_t i = 0; i < m.positions.size(); ++i) {
        Vec3 p = m.positions[i];
        auto k = std::make_tuple(q(p.x), q(p.y), q(p.z));
        auto it = weld.find(k);
        if (it == weld.end()) {
            to[i] = uint32_t(np.size());
            weld[k] = to[i];
            np.push_back(p);
        } else
            to[i] = it->second;
    }
    if (np.size() == m.positions.size()) return false;
    for (auto& f : m.faces) {
        Face nf;
        for (uint32_t v : f.v) {
            uint32_t w = to[v];
            if (nf.v.empty() || nf.v.back() != w) nf.v.push_back(w);
        }
        if (nf.v.size() >= 2 && nf.v.front() == nf.v.back()) nf.v.pop_back();
        f = std::move(nf);
    }
    std::vector<Face> kept;
    for (auto& f : m.faces)
        if (f.v.size() >= 3) kept.push_back(std::move(f));
    m.positions = std::move(np);
    m.faces = std::move(kept);
    mesh_compact(m);
    mesh_sync_render(m);
    return true;
}

bool smooth_verts(Mesh& m, const std::vector<uint32_t>& verts, float factor, int iters) {
    mesh_ensure_faces(m);
    if (verts.empty()) return false;
    float t = (!finite_f(factor) || factor <= 0) ? 0.5f : std::min(1.0f, factor);
    if (iters < 1) iters = 1;
    if (iters > 20) iters = 20;
    Topo topo = build_topo(m);
    std::set<uint32_t> S(verts.begin(), verts.end());
    for (int it = 0; it < iters; ++it) {
        std::vector<Vec3> np = m.positions;
        for (uint32_t v : S) {
            if (v >= topo.nbr.size() || v >= m.positions.size() || topo.nbr[v].empty()) continue;
            Vec3 a{0, 0, 0};
            size_t cnt = 0;
            for (uint32_t u : topo.nbr[v]) {
                if (u >= m.positions.size()) continue;
                a += m.positions[u];
                ++cnt;
            }
            if (!cnt) continue;
            a = a / float(cnt);
            np[v] = m.positions[v] * (1 - t) + a * t;
        }
        m.positions = std::move(np);
    }
    mesh_sync_render(m);
    return true;
}

bool push_verts(Mesh& m, const std::vector<uint32_t>& verts, float amount) {
    if (verts.empty()) return false;
    if (!finite_f(amount)) return false;
    auto n = vertex_normals(m);
    for (uint32_t v : live_verts(m, verts))
        if (v < n.size()) m.positions[v] += n[v] * amount;
    mesh_sync_render(m);
    return true;
}

bool to_sphere(Mesh& m, const std::vector<uint32_t>& verts, float factor) {
    if (verts.empty()) return false;
    const std::vector<uint32_t> vs = live_verts(m, verts);
    if (vs.empty()) return false;
    float t = finite_f(factor) ? std::min(1.0f, std::max(0.0f, factor)) : 1.0f;
    Vec3 c{0, 0, 0};
    for (uint32_t v : vs) c += m.positions[v];
    c = c / float(vs.size());
    float r = 0;
    for (uint32_t v : vs) r += length(m.positions[v] - c);
    r /= float(vs.size());
    if (!(r > 1e-8f)) return false;
    for (uint32_t v : vs) {
        Vec3 d = m.positions[v] - c;
        float l = length(d);
        Vec3 on = c + (l < 1e-8f ? Vec3{0, r, 0} : d * (r / l));
        m.positions[v] = m.positions[v] * (1 - t) + on * t;
    }
    mesh_sync_render(m);
    return true;
}

bool flatten_verts(Mesh& m, const std::vector<uint32_t>& verts, Vec3 axis) {
    if (verts.empty()) return false;
    std::vector<uint32_t> vs = live_verts(m, verts);
    if (vs.empty()) return false;
    Vec3 c{0, 0, 0};
    for (uint32_t v : vs) c += m.positions[v];
    c = c / float(vs.size());
    Vec3 n = finite_f(axis.x) && finite_f(axis.y) && finite_f(axis.z) ? normalize(axis) : Vec3{0, 0, 0};
    if (length(n) < 0.5f) {
        n = {0, 0, 0};
        auto vn = vertex_normals(m);
        for (uint32_t v : verts)
            if (v < vn.size()) n += vn[v];
        n = normalize(n);
        if (length(n) < 0.5f) n = {0, 1, 0};
    }
    for (uint32_t v : verts) {
        Vec3 d = m.positions[v] - c;
        m.positions[v] = m.positions[v] - n * dot(d, n);
    }
    mesh_sync_render(m);
    return true;
}

bool randomize_verts(Mesh& m, const std::vector<uint32_t>& verts, float amount) {
    if (verts.empty()) return false;
    const std::vector<uint32_t> vs = live_verts(m, verts);
    if (vs.empty()) return false;
    float a = (!finite_f(amount) || amount == 0) ? 0.05f : amount;
    for (uint32_t v : vs) {
        float s = float(v) * 127.1f + 311.7f;
        auto fr = [](float x) {
            x = x - std::floor(x);
            return x;
        };
        float rx = fr(std::sin(s) * 43758.5453f) * 2 - 1;
        float ry = fr(std::sin(s + 19.2f) * 23421.631f) * 2 - 1;
        float rz = fr(std::sin(s + 47.8f) * 98127.173f) * 2 - 1;
        m.positions[v] += Vec3{rx, ry, rz} * a;
    }
    mesh_sync_render(m);
    return true;
}

bool snap_verts(Mesh& m, const std::vector<uint32_t>& verts, float grid) {
    if (verts.empty()) return false;
    const std::vector<uint32_t> vs = live_verts(m, verts);
    if (vs.empty()) return false;
    float g = (!finite_f(grid) || grid <= 0) ? 0.1f : grid;
    auto sn = [&](float x) { return std::round(x / g) * g; };
    for (uint32_t v : vs) {
        Vec3& p = m.positions[v];
        p = {sn(p.x), sn(p.y), sn(p.z)};
    }
    mesh_sync_render(m);
    return true;
}

bool bisect_mesh(Mesh& m, Vec3 plane_n, float plane_d) {
    mesh_ensure_faces(m);
    Vec3 n = normalize(plane_n);
    if (length(n) < 0.5f) n = {0, 1, 0};
    const float eps = 1e-5f;
    auto sgn = [&](uint32_t v) { return dot(n, m.positions[v]) - plane_d; };
    std::map<uint64_t, uint32_t> cut;
    auto isect = [&](uint32_t a, uint32_t b) {
        uint64_t k = ekey(a, b);
        auto it = cut.find(k);
        if (it != cut.end()) return it->second;
        float sa = sgn(a), sb = sgn(b);
        float t = sa / (sa - sb);
        t = std::min(1.0f, std::max(0.0f, t));
        uint32_t id = uint32_t(m.positions.size());
        m.positions.push_back(m.positions[a] * (1 - t) + m.positions[b] * t);
        cut[k] = id;
        return id;
    };
    std::vector<Face> rebuilt;
    for (const auto& f : m.faces) {
        int nv = int(f.v.size());
        if (nv < 3) continue;
        for (int side = 0; side < 2; ++side) {
            Face nf;
            for (int i = 0; i < nv; ++i) {
                uint32_t a = f.v[i], b = f.v[(i + 1) % nv];
                float sa = sgn(a), sb = sgn(b);
                bool keep = side == 0 ? sa >= -eps : sa <= eps;
                if (keep) nf.v.push_back(a);
                if (sa * sb < -eps * eps) nf.v.push_back(isect(a, b));
            }
            Face cleaned;
            for (uint32_t v : nf.v)
                if (cleaned.v.empty() || cleaned.v.back() != v) cleaned.v.push_back(v);
            if (cleaned.v.size() >= 2 && cleaned.v.front() == cleaned.v.back()) cleaned.v.pop_back();
            if (cleaned.v.size() >= 3) rebuilt.push_back(std::move(cleaned));
        }
    }
    m.faces = std::move(rebuilt);
    mesh_compact(m);
    mesh_sync_render(m);
    return !m.faces.empty();
}

bool connect_verts(Mesh& m, uint32_t a, uint32_t b) {
    mesh_ensure_faces(m);
    if (a == b || a >= m.positions.size() || b >= m.positions.size()) return false;
    for (auto& f : m.faces) {
        int ia = -1, ib = -1, nv = int(f.v.size());
        for (int i = 0; i < nv; ++i) {
            if (f.v[i] == a) ia = i;
            if (f.v[i] == b) ib = i;
        }
        if (ia < 0 || ib < 0) continue;
        int d = std::abs(ia - ib);
        if (d == 1 || d == nv - 1) continue;
        if (ia > ib) std::swap(ia, ib);
        Face f2;
        for (int i = ia; i <= ib; ++i) f2.v.push_back(f.v[i]);
        Face f1;
        for (int i = ib; i < nv; ++i) f1.v.push_back(f.v[i]);
        for (int i = 0; i <= ia; ++i) f1.v.push_back(f.v[i]);
        Face cleaned = f1;
        f = std::move(f2);
        if (cleaned.v.size() >= 3) m.faces.push_back(std::move(cleaned));
        mesh_sync_render(m);
        return true;
    }
    return false;
}

std::vector<int> grow_faces(const Mesh& m, const std::vector<int>& faces) {
    Topo t = build_topo(m);
    std::set<int> S(faces.begin(), faces.end());
    std::set<int> out = S;
    for (int fi : S) {
        if (fi < 0 || fi >= int(m.faces.size())) continue;
        const Face& f = m.faces[fi];
        int nv = int(f.v.size());
        for (int i = 0; i < nv; ++i) {
            auto it = t.edge_faces.find(ekey(f.v[i], f.v[(i + 1) % nv]));
            if (it == t.edge_faces.end()) continue;
            for (int o : it->second) out.insert(o);
        }
    }
    return {out.begin(), out.end()};
}

std::vector<Edge> grow_edges(const Mesh& m, const std::vector<Edge>& edges) {
    Topo t = build_topo(m);
    std::set<uint64_t> S;
    for (Edge e : edges) S.insert(ekey(e.a, e.b));
    std::set<uint64_t> out = S;
    for (Edge e : edges) {
        for (uint32_t v : {e.a, e.b}) {
            if (v >= t.nbr.size()) continue;
            for (uint32_t u : t.nbr[v]) out.insert(ekey(v, u));
        }
    }
    std::vector<Edge> r;
    for (uint64_t k : out) r.push_back(make_edge(uint32_t(k >> 32), uint32_t(k)));
    return r;
}

std::vector<uint32_t> grow_verts(const Mesh& m, const std::vector<uint32_t>& verts) {
    Topo t = build_topo(m);
    std::set<uint32_t> out(verts.begin(), verts.end());
    for (uint32_t v : verts) {
        if (v >= t.nbr.size()) continue;
        for (uint32_t u : t.nbr[v]) out.insert(u);
    }
    return {out.begin(), out.end()};
}

std::vector<int> linked_faces(const Mesh& m, const std::vector<int>& seeds) {
    Topo t = build_topo(m);
    std::set<int> seen;
    std::queue<int> q;
    for (int fi : seeds)
        if (fi >= 0 && fi < int(m.faces.size()) && seen.insert(fi).second) q.push(fi);
    if (seen.empty() && !m.faces.empty()) {
        seen.insert(0);
        q.push(0);
    }
    while (!q.empty()) {
        int fi = q.front();
        q.pop();
        const Face& f = m.faces[fi];
        int nv = int(f.v.size());
        for (int i = 0; i < nv; ++i) {
            auto it = t.edge_faces.find(ekey(f.v[i], f.v[(i + 1) % nv]));
            if (it == t.edge_faces.end()) continue;
            for (int o : it->second)
                if (seen.insert(o).second) q.push(o);
        }
    }
    return {seen.begin(), seen.end()};
}

const char* edit_mode_name(EditMode m) {
    if (m == EditMode::Face) return "face";
    if (m == EditMode::Edge) return "edge";
    if (m == EditMode::Vert) return "vert";
    return "object";
}

Mesh* Scene::selected_mesh() {
    if (selection < 0 || selection >= static_cast<int>(nodes.size())) return nullptr;
    int mi = nodes[selection].mesh;
    if (mi < 0 || mi >= static_cast<int>(meshes.size())) return nullptr;
    return &meshes[mi];
}

}  // namespace kiln
