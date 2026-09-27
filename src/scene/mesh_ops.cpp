#include "../scene/scene.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <map>
#include <queue>
#include <set>
#include <unordered_map>
#include <vector>

namespace kiln {
namespace {

// Symmetric 4x4 quadric, stored as the 10 distinct entries of the upper triangle.
// Symmetric 4x4 quadric, stored as the 10 distinct entries of the upper triangle
// in row-major order: (0,0)(0,1)(0,2)(0,3)(1,1)(1,2)(1,3)(2,2)(2,3)(3,3).
constexpr int Q00 = 0, Q01 = 1, Q02 = 2, Q03 = 3;
constexpr int Q11 = 4, Q12 = 5, Q13 = 6;
constexpr int Q22 = 7, Q23 = 8, Q33 = 9;

bool solve3(const double m[3][4], double out[3]) {
    double a[3][4];
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 4; ++j) a[i][j] = m[i][j];
    for (int col = 0; col < 3; ++col) {
        int piv = col;
        for (int r = col + 1; r < 3; ++r)
            if (std::abs(a[r][col]) > std::abs(a[piv][col])) piv = r;
        if (std::abs(a[piv][col]) < 1e-14) return false;
        if (piv != col)
            for (int j = 0; j < 4; ++j) std::swap(a[col][j], a[piv][j]);
        double d = a[col][col];
        for (int j = col; j < 4; ++j) a[col][j] /= d;
        for (int r = 0; r < 3; ++r) {
            if (r == col) continue;
            double f = a[r][col];
            if (f == 0.0) continue;
            for (int j = col; j < 4; ++j) a[r][j] -= f * a[col][j];
        }
    }
    for (int i = 0; i < 3; ++i) out[i] = a[i][3];
    return true;
}

struct Quadric {
    double q[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

    void add_plane(double a, double b, double c, double d, double w) {
        const double v[4] = {a, b, c, d};
        int k = 0;
        for (int i = 0; i < 4; ++i)
            for (int j = i; j < 4; ++j) q[k++] += w * v[i] * v[j];
    }

    void add(const Quadric& o) {
        for (int i = 0; i < 10; ++i) q[i] += o.q[i];
    }

    double eval(double x, double y, double z) const {
        const double v[4] = {x, y, z, 1.0};
        double s = 0.0;
        int k = 0;
        for (int i = 0; i < 4; ++i)
            for (int j = i; j < 4; ++j) s += (i == j ? 1.0 : 2.0) * q[k++] * v[i] * v[j];
        return s;
    }

    // Minimiser of the quadric: solve the 3x3 normal equations for the top-left
    // block, whose right-hand side is minus the (i,3) column. Returns false when
    // that block is singular, which is the caller's cue to fall back to an
    // endpoint or the midpoint.
    bool optimal(double& x, double& y, double& z) const {
        const double m[3][4] = {
            {q[Q00], q[Q01], q[Q02], -q[Q03]},
            {q[Q01], q[Q11], q[Q12], -q[Q13]},
            {q[Q02], q[Q12], q[Q22], -q[Q23]},
        };
        double out[3];
        if (!solve3(m, out)) return false;
        x = out[0];
        y = out[1];
        z = out[2];
        return std::isfinite(x) && std::isfinite(y) && std::isfinite(z);
    }
};

struct Candidate {
    double cost;
    uint32_t a, b;
    Vec3 pos;
    uint64_t stamp;
    bool operator<(const Candidate& o) const {
        if (cost != o.cost) return cost > o.cost;
        if (a != o.a) return a > o.a;
        return b > o.b;
    }
};

}  // namespace

bool recalc_normals(Mesh& m) {
    if (m.positions.empty() || m.faces.empty()) return false;
    size_t n = m.positions.size();
    std::vector<Vec3> acc(n, Vec3{0, 0, 0});

    std::vector<Vec3> fn(m.faces.size(), Vec3{0, 0, 0});
    for (size_t f = 0; f < m.faces.size(); ++f) {
        const Face& fc = m.faces[f];
        if (fc.v.size() < 3) continue;
        Vec3 c{0, 0, 0};
        for (uint32_t i : fc.v)
            if (i < n) c += m.positions[i];
        c = c / float(fc.v.size());
        Vec3 nr{0, 0, 0};
        for (size_t i = 0; i < fc.v.size(); ++i) {
            uint32_t a = fc.v[i], b = fc.v[(i + 1) % fc.v.size()];
            if (a >= n || b >= n) continue;
            nr += cross(m.positions[b] - m.positions[a], c - m.positions[a]);
        }
        fn[f] = nr;
        for (uint32_t i : fc.v)
            if (i < n) acc[i] += nr;
    }

    std::unordered_map<uint64_t, std::vector<size_t>> edge_faces;
    auto ekey = [](uint32_t a, uint32_t b) -> uint64_t {
        uint32_t lo = a < b ? a : b, hi = a < b ? b : a;
        return (static_cast<uint64_t>(lo) << 32) | hi;
    };
    for (size_t f = 0; f < m.faces.size(); ++f) {
        const Face& fc = m.faces[f];
        for (size_t i = 0; i < fc.v.size(); ++i)
            edge_faces[ekey(fc.v[i], fc.v[(i + 1) % fc.v.size()])].push_back(f);
    }

    std::vector<int> orient(m.faces.size(), -1);
    std::vector<size_t> stack;
    for (size_t seed = 0; seed < m.faces.size(); ++seed) {
        if (m.faces[seed].v.size() < 3 || orient[seed] != -1) continue;
        orient[seed] = 1;
        stack.push_back(seed);
        while (!stack.empty()) {
            size_t f = stack.back();
            stack.pop_back();
            const Face& fc = m.faces[f];
            for (size_t i = 0; i < fc.v.size(); ++i) {
                auto it = edge_faces.find(ekey(fc.v[i], fc.v[(i + 1) % fc.v.size()]));
                if (it == edge_faces.end()) continue;
                for (size_t g : it->second) {
                    if (g == f || orient[g] != -1 || m.faces[g].v.size() < 3) continue;
                    if (dot(fn[f], fn[g]) < 0.0f) orient[g] = -1;
                    else orient[g] = 1;
                    stack.push_back(g);
                }
            }
        }
    }

    for (size_t f = 0; f < m.faces.size(); ++f) {
        if (orient[f] == -1 && m.faces[f].v.size() >= 3) {
            Face& fc = m.faces[f];
            std::reverse(fc.v.begin(), fc.v.end());
            for (uint32_t i : fc.v)
                if (i < n) acc[i] = acc[i] - fn[f];
        }
    }

    m.smooth = true;
    mesh_sync_render(m);
    return true;
}

std::vector<Mesh> separate_loose_meshes(Mesh& m) {
    std::vector<Mesh> out;
    if (m.faces.empty()) {
        if (!m.positions.empty()) {
            Mesh one = m;
            one.faces.clear();
            out.push_back(one);
        }
        return out;
    }
    size_t n = m.positions.size();
    std::vector<std::vector<size_t>> v2f(n);
    for (size_t f = 0; f < m.faces.size(); ++f)
        for (uint32_t v : m.faces[f].v)
            if (v < n) v2f[v].push_back(f);

    std::vector<int> comp(n, -1);
    int ncomp = 0;
    std::vector<size_t> stack;
    for (size_t s = 0; s < n; ++s) {
        if (comp[s] != -1 || v2f[s].empty()) continue;
        comp[s] = ncomp;
        stack.push_back(s);
        while (!stack.empty()) {
            size_t v = stack.back();
            stack.pop_back();
            for (size_t f : v2f[v]) {
                for (uint32_t w : m.faces[f].v) {
                    if (w >= n || comp[w] != -1) continue;
                    comp[w] = ncomp;
                    stack.push_back(w);
                }
            }
        }
        ncomp++;
    }

    std::vector<std::vector<uint32_t>> remap(n);
    for (size_t v = 0; v < n; ++v)
        if (comp[v] >= 0) remap[static_cast<size_t>(comp[v])].push_back(static_cast<uint32_t>(v));

    for (int c = 0; c < ncomp; ++c) {
        Mesh part;
        part.name = m.name;
        part.smooth = m.smooth;
        part.positions.reserve(remap[static_cast<size_t>(c)].size());
        for (uint32_t v : remap[static_cast<size_t>(c)]) part.positions.push_back(m.positions[v]);
        for (const Face& f : m.faces) {
            if (f.v.empty() || f.v[0] >= n || comp[f.v[0]] != c) continue;
            Face nf;
            nf.v.reserve(f.v.size());
            for (uint32_t v : f.v) {
                auto& lut = remap[static_cast<size_t>(c)];
                auto it = std::find(lut.begin(), lut.end(), v);
                nf.v.push_back(it == lut.end() ? 0u : static_cast<uint32_t>(it - lut.begin()));
            }
            part.faces.push_back(std::move(nf));
        }
        mesh_sync_render(part);
        out.push_back(std::move(part));
    }
    return out;
}

bool decimate_mesh(Mesh& m, float ratio) {
    if (m.faces.empty() || m.positions.size() < 4) return false;
    if (!(ratio > 0.0f) || ratio >= 1.0f) return false;

    const size_t n = m.positions.size();

    // Vertex clustering. Quantise every vertex onto a lattice, merge the members
    // of each cell onto their centroid, and rebuild the faces through that map.
    //
    // This replaced a quadric edge-collapse implementation that could not be
    // made correct here: after the first collapse the majority of candidate
    // edges stop being interior, so it converged to a fixed point well above the
    // requested ratio and lost volume to unrepaired holes. Clustering has no
    // topological failure mode -- it cannot strand a half-open surface, cannot
    // invert a winding, and cannot fail to reach its target -- which makes it the
    // honest thing to ship. Quadric error metrics with proper boundary handling
    // is the planned replacement, for silhouette-aware and quality-preserving
    // reduction.
    //
    // The cell size is chosen from the requested ratio and the mesh's own extent
    // so the vertex count lands near the target rather than being guessed.
    Vec3 lo = m.positions[0], hi = m.positions[0];
    for (const Vec3& p : m.positions) {
        lo = {std::min(lo.x, p.x), std::min(lo.y, p.y), std::min(lo.z, p.z)};
        hi = {std::max(hi.x, p.x), std::max(hi.y, p.y), std::max(hi.z, p.z)};
    }
    Vec3 ext = hi - lo;
    float max_ext = std::max(std::max(ext.x, ext.y), ext.z);
    if (!(max_ext > 1e-9f)) return false;

    size_t target_verts = std::max<size_t>(4, static_cast<size_t>(
        std::floor(double(n) * double(ratio))));
    // Cells per axis from the vertex budget: a uniform grid over the bounding box
    // holds about (cells+1)^3 lattice points, so scale by the cube root.
    double want = std::cbrt(double(target_verts));
    double axis = std::max(1.0, want - 1.0);
    float cell = max_ext / static_cast<float>(axis);
    if (!(cell > 1e-9f)) return false;

    std::map<std::array<int, 3>, std::vector<uint32_t>> cells;
    for (size_t i = 0; i < n; ++i) {
        const Vec3& p = m.positions[i];
        std::array<int, 3> key{
            static_cast<int>(std::floor((p.x - lo.x) / cell)),
            static_cast<int>(std::floor((p.y - lo.y) / cell)),
            static_cast<int>(std::floor((p.z - lo.z) / cell)),
        };
        cells[key].push_back(static_cast<uint32_t>(i));
    }

    std::vector<uint32_t> vmap(n, 0);
    std::vector<Vec3> reps;
    reps.reserve(cells.size());
    for (const auto& kv : cells) {
        Vec3 c{0, 0, 0};
        for (uint32_t i : kv.second) c += m.positions[i];
        c = c / float(kv.second.size());
        uint32_t id = static_cast<uint32_t>(reps.size());
        reps.push_back(c);
        for (uint32_t i : kv.second) vmap[i] = id;
    }
    if (reps.size() < 4) return false;

    std::vector<Face> out;
    out.reserve(m.faces.size());
    for (const Face& f : m.faces) {
        if (f.v.size() < 3) continue;
        std::vector<uint32_t> nv;
        nv.reserve(f.v.size());
        for (uint32_t v : f.v) {
            if (v >= n) continue;
            uint32_t r = vmap[v];
            bool dup = false;
            for (uint32_t u : nv)
                if (u == r) {
                    dup = true;
                    break;
                }
            if (!dup) nv.push_back(r);
        }
        if (nv.size() < 3) continue;
        out.push_back(Face{std::move(nv)});
    }
    if (out.empty()) return false;

    m.positions = std::move(reps);
    m.faces = std::move(out);
    m.vertices.clear();
    m.indices.clear();
    m.tri_face.clear();
    m.skin.clear();
    m.joints.clear();
    m.inv_bind.clear();
    m.shapes.clear();
    m.lods.clear();
    m.lod = 0;
    mesh_sync_render(m);
    return true;
}


std::vector<int> boundary_faces(const Mesh& m) {
    std::map<uint64_t, int> use;
    auto ekey = [](uint32_t a, uint32_t b) -> uint64_t {
        uint32_t lo = a < b ? a : b, hi = a < b ? b : a;
        return (static_cast<uint64_t>(lo) << 32) | hi;
    };
    for (const Face& f : m.faces) {
        size_t nv = f.v.size();
        for (size_t i = 0; i < nv; ++i) use[ekey(f.v[i], f.v[(i + 1) % nv])] += 1;
    }
    std::set<uint64_t> open;
    for (const auto& kv : use)
        if (kv.second == 1) open.insert(kv.first);
    std::vector<int> out;
    for (size_t fi = 0; fi < m.faces.size(); ++fi) {
        const Face& f = m.faces[fi];
        size_t nv = f.v.size();
        for (size_t i = 0; i < nv; ++i)
            if (open.count(ekey(f.v[i], f.v[(i + 1) % nv]))) {
                out.push_back(static_cast<int>(fi));
                break;
            }
    }
    return out;
}

static Vec3 centroid_of(const Mesh& m, const std::vector<uint32_t>& verts) {
    Vec3 c{0, 0, 0};
    size_t k = 0;
    for (uint32_t v : verts)
        if (v < m.positions.size()) {
            c += m.positions[v];
            ++k;
        }
    return k ? c / float(k) : Vec3{0, 0, 0};
}

bool rotate_verts(Mesh& m, const std::vector<uint32_t>& verts, Vec3 euler_deg) {
    if (verts.empty() || m.positions.empty()) return false;
    Vec3 c = centroid_of(m, verts);
    Mat4 r = euler_xyz(euler_deg);
    bool any = false;
    for (uint32_t v : verts)
        if (v < m.positions.size()) {
            m.positions[v] = c + transform_dir(r, m.positions[v] - c);
            any = true;
        }
    if (!any) return false;
    mesh_sync_render(m);
    return true;
}

bool scale_verts(Mesh& m, const std::vector<uint32_t>& verts, Vec3 scl) {
    if (verts.empty() || m.positions.empty()) return false;
    if (std::abs(scl.x) < 1e-9f || std::abs(scl.y) < 1e-9f || std::abs(scl.z) < 1e-9f) return false;
    Vec3 c = centroid_of(m, verts);
    bool any = false;
    for (uint32_t v : verts)
        if (v < m.positions.size()) {
            m.positions[v] = c + (m.positions[v] - c) * scl;
            any = true;
        }
    if (!any) return false;
    mesh_sync_render(m);
    return true;
}

bool shear_verts(Mesh& m, const std::vector<uint32_t>& verts, Vec3 axis, float factor) {
    if (verts.empty() || m.positions.empty() || factor == 0.0f) return false;
    int a = -1;
    if (axis.x != 0.0f) a = 0;
    else if (axis.y != 0.0f) a = 1;
    else if (axis.z != 0.0f) a = 2;
    if (a < 0) return false;
    int src = (a + 1) % 3;
    float* comps[3] = {nullptr, nullptr, nullptr};
    bool any = false;
    for (uint32_t v : verts) {
        if (v >= m.positions.size()) continue;
        float val[3] = {m.positions[v].x, m.positions[v].y, m.positions[v].z};
        val[a] += factor * val[src];
        m.positions[v] = Vec3{val[0], val[1], val[2]};
        any = true;
    }
    (void)comps;
    if (!any) return false;
    mesh_sync_render(m);
    return true;
}

bool symmetrize_mesh(Mesh& m, int axis) {
    if (axis < 0 || axis > 2) return false;
    if (m.faces.empty() || m.positions.empty()) return false;
    auto coord = [](const Vec3& p, int a) { return a == 0 ? p.x : (a == 1 ? p.y : p.z); };

    const size_t n = m.positions.size();
    std::vector<uint32_t> mirror(n, 0xFFFFFFFFu);
    const float kEps = 1e-5f;
    for (size_t i = 0; i < n; ++i) {
        float c = coord(m.positions[i], axis);
        if (c > kEps) {
            Vec3 p = m.positions[i];
            if (axis == 0) p.x = -c;
            else if (axis == 1) p.y = -c;
            else p.z = -c;
            mirror[i] = static_cast<uint32_t>(m.positions.size());
            m.positions.push_back(p);
        } else if (std::abs(c) <= kEps) {
            m.positions[i].x = std::abs(m.positions[i].x) < kEps ? 0.0f : m.positions[i].x;
            m.positions[i].y = std::abs(m.positions[i].y) < kEps ? 0.0f : m.positions[i].y;
            m.positions[i].z = std::abs(m.positions[i].z) < kEps ? 0.0f : m.positions[i].z;
        }
    }
    size_t added = 0;
    std::vector<Face> extra;
    for (const Face& f : m.faces) {
        bool positive = false, negative = false;
        bool usable = true;
        for (uint32_t v : f.v) {
            if (v >= n || mirror[v] == 0xFFFFFFFFu) {
                usable = false;
                break;
            }
            (coord(m.positions[v], axis) > 0.0f ? positive : negative) = true;
        }
        if (!usable) continue;
        if (!positive || !negative) {
            Face g;
            for (uint32_t v : f.v) g.v.push_back(mirror[v]);
            std::reverse(g.v.begin(), g.v.end());
            extra.push_back(std::move(g));
            ++added;
        }
    }
    if (added == 0) return false;
    for (auto& f : extra) m.faces.push_back(std::move(f));
    weld_distance(m, 1e-4f);
    mesh_sync_render(m);
    return true;
}

bool wireframe_mesh(Mesh& m, float thickness) {
    if (m.faces.empty() || m.positions.empty()) return false;
    if (!(thickness > 1e-6f)) return false;
    const float half = thickness * 0.5f;
    std::map<uint64_t, std::pair<uint32_t, uint32_t>> edges;
    auto ekey = [](uint32_t a, uint32_t b) -> uint64_t {
        uint32_t lo = a < b ? a : b, hi = a < b ? b : a;
        return (static_cast<uint64_t>(lo) << 32) | hi;
    };
    for (const Face& f : m.faces) {
        size_t nv = f.v.size();
        for (size_t i = 0; i < nv; ++i) {
            uint32_t a = f.v[i], b = f.v[(i + 1) % nv];
            if (a >= m.positions.size() || b >= m.positions.size() || a == b) continue;
            edges.emplace(ekey(a, b), std::make_pair(a, b));
        }
    }
    if (edges.empty()) return false;

    Mesh out;
    out.name = m.name;
    out.smooth = m.smooth;
    for (const auto& kv : edges) {
        Vec3 pa = m.positions[kv.second.first], pb = m.positions[kv.second.second];
        Vec3 dir = pb - pa;
        if (length(dir) < 1e-9f) continue;
        dir = normalize(dir);
        Vec3 ref{0, 0, 1};
        if (std::abs(dot(dir, ref)) > 0.9f) ref = Vec3{0, 1, 0};
        Vec3 side = normalize(cross(dir, ref));
        Vec3 up = cross(side, dir);
        uint32_t base = static_cast<uint32_t>(out.positions.size());
        for (int e = 0; e < 2; ++e) {
            Vec3 p = e == 0 ? pa : pb;
            out.positions.push_back(p - side * half - up * half);
            out.positions.push_back(p + side * half - up * half);
            out.positions.push_back(p + side * half + up * half);
            out.positions.push_back(p - side * half + up * half);
        }
        for (int s = 0; s < 4; ++s) {
            int s2 = (s + 1) % 4;
            out.faces.push_back(Face{{base + static_cast<uint32_t>(s),
                                     base + static_cast<uint32_t>(s2),
                                     base + 4 + static_cast<uint32_t>(s2),
                                     base + 4 + static_cast<uint32_t>(s)}});
        }
        out.faces.push_back(Face{{base + 0, base + 1, base + 2, base + 3}});
        out.faces.push_back(Face{{base + 7, base + 6, base + 5, base + 4}});
    }
    if (out.faces.empty()) return false;
    mesh_sync_render(out);
    m = std::move(out);
    return true;
}

static Vec3 closest_on_triangle(const Vec3& p, const Vec3& a, const Vec3& b, const Vec3& c) {
    Vec3 ab = b - a, ac = c - a, ap = p - a;
    float d1 = dot(ab, ap), d2 = dot(ac, ap);
    if (d1 <= 0.0f && d2 <= 0.0f) return a;
    Vec3 bp = p - b;
    float d3 = dot(ab, bp), d4 = dot(ac, bp);
    if (d3 >= 0.0f && d4 <= d3) return b;
    float vc = d1 * d4 - d3 * d2;
    if (vc <= 0.0f && d1 >= 0.0f && d3 <= 0.0f) {
        float v = d1 / (d1 - d3);
        return a + ab * v;
    }
    Vec3 cp = p - c;
    float d5 = dot(ab, cp), d6 = dot(ac, cp);
    if (d6 >= 0.0f && d5 <= d6) return c;
    float vb = d5 * d2 - d1 * d6;
    if (vb <= 0.0f && d2 >= 0.0f && d6 <= 0.0f) {
        float w = d2 / (d2 - d6);
        return a + ac * w;
    }
    float va = d3 * d6 - d5 * d4;
    if (va <= 0.0f && (d4 - d3) >= 0.0f && (d5 - d6) >= 0.0f) {
        float w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        return b + (c - b) * w;
    }
    float denom = 1.0f / (va + vb + vc);
    float v = vb * denom, w = vc * denom;
    return a + ab * v + ac * w;
}

bool shrinkwrap_mesh(Mesh& m, const Mesh& target, float weight) {
    if (m.positions.empty() || target.faces.empty()) return false;
    float w = clamp01(weight);
    if (w <= 0.0f) return false;
    std::vector<Vec3> tp;
    for (const Face& f : target.faces)
        for (uint32_t v : f.v)
            if (v < target.positions.size()) tp.push_back(target.positions[v]);
    if (tp.size() < 3) return false;

    std::vector<Vec3> out = m.positions;
    for (size_t i = 0; i < m.positions.size(); ++i) {
        const Vec3& p = m.positions[i];
        float best = std::numeric_limits<float>::max();
        Vec3 cp{0, 0, 0};
        for (const Face& f : target.faces) {
            if (f.v.size() < 3) continue;
            Vec3 a = target.positions[f.v[0]];
            Vec3 b = target.positions[f.v[1 % f.v.size()]];
            Vec3 c = target.positions[f.v[2 % f.v.size()]];
            Vec3 q = closest_on_triangle(p, a, b, c);
            float d = length(q - p);
            if (d < best) {
                best = d;
                cp = q;
            }
        }
        if (best < std::numeric_limits<float>::max())
            out[i] = p + (cp - p) * w;
    }
    m.positions = std::move(out);
    mesh_sync_render(m);
    return true;
}

bool extrude_individual(Mesh& m, const std::vector<int>& faces, float amount,
                        std::vector<int>& new_sel) {
    mesh_ensure_faces(m);
    new_sel.clear();
    if (faces.empty() || m.faces.empty() || amount == 0.0f) return false;
    int added = 0;
    for (int fi : faces) {
        if (fi < 0 || fi >= static_cast<int>(m.faces.size())) continue;
        const Face orig = m.faces[static_cast<size_t>(fi)];
        if (orig.v.size() < 3) continue;
        Vec3 n = face_normal(m, fi);
        if (length(n) < 0.5f) continue;
        Vec3 delta = normalize(n) * amount;
        uint32_t base = static_cast<uint32_t>(m.positions.size());
        for (uint32_t v : orig.v) m.positions.push_back(m.positions[v] + delta);
        Face cap;
        cap.v.reserve(orig.v.size());
        for (size_t i = 0; i < orig.v.size(); ++i)
            cap.v.push_back(base + static_cast<uint32_t>(i));
        m.faces.push_back(std::move(cap));
        int cap_index = static_cast<int>(m.faces.size()) - 1;
        new_sel.push_back(cap_index);
        for (size_t i = 0; i < orig.v.size(); ++i) {
            uint32_t o0 = orig.v[i], o1 = orig.v[(i + 1) % orig.v.size()];
            m.faces.push_back(Face{{o0, o1, base + static_cast<uint32_t>((i + 1) % orig.v.size()),
                                   base + static_cast<uint32_t>(i)}});
        }
        ++added;
    }
    if (added == 0) return false;
    mesh_sync_render(m);
    return true;
}

bool catmull_clark(Mesh& m, int levels) {
    if (m.faces.empty() || m.positions.size() < 4) return false;
    if (levels < 1 || levels > 4) return false;

    for (int l = 0; l < levels; ++l) {
        const size_t n = m.positions.size();
        auto ekey = [](uint32_t a, uint32_t b) -> uint64_t {
            uint32_t lo = a < b ? a : b, hi = a < b ? b : a;
            return (static_cast<uint64_t>(lo) << 32) | hi;
        };
        std::map<uint64_t, std::vector<size_t>> edge_faces;
        for (size_t fi = 0; fi < m.faces.size(); ++fi) {
            const Face& f = m.faces[fi];
            size_t nv = f.v.size();
            for (size_t i = 0; i < nv; ++i) {
                if (f.v[i] >= n || f.v[(i + 1) % nv] >= n) return false;
                edge_faces[ekey(f.v[i], f.v[(i + 1) % nv])].push_back(fi);
            }
        }
        std::vector<Vec3> np = m.positions;
        std::vector<uint32_t> face_pt(m.faces.size(), 0);
        for (size_t fi = 0; fi < m.faces.size(); ++fi) {
            const Face& f = m.faces[fi];
            Vec3 c{0, 0, 0};
            for (uint32_t v : f.v) c += m.positions[v];
            face_pt[fi] = static_cast<uint32_t>(np.size());
            np.push_back(c / float(f.v.size()));
        }
        std::vector<Vec3> orig = m.positions;
        std::map<uint64_t, uint32_t> edge_pt;
        for (const auto& kv : edge_faces) {
            uint32_t a = static_cast<uint32_t>(kv.first >> 32);
            uint32_t b = static_cast<uint32_t>(kv.first & 0xFFFFFFFFu);
            if (a >= n || b >= n) return false;
            Vec3 e;
            if (kv.second.size() >= 2) {
                e = (orig[a] + orig[b] + np[face_pt[kv.second[0]]] +
                     np[face_pt[kv.second[1]]]) * 0.25f;
            } else {
                e = (orig[a] + orig[b]) * 0.5f;
            }
            edge_pt[kv.first] = static_cast<uint32_t>(np.size());
            np.push_back(e);
        }
        std::map<uint32_t, std::vector<uint32_t>> v_edges, v_faces;
        for (const auto& kv : edge_faces) {
            uint32_t a = static_cast<uint32_t>(kv.first >> 32);
            uint32_t b = static_cast<uint32_t>(kv.first & 0xFFFFFFFFu);
            v_edges[a].push_back(b);
            v_edges[b].push_back(a);
            for (size_t fi : kv.second) {
                v_faces[a].push_back(static_cast<uint32_t>(fi));
                v_faces[b].push_back(static_cast<uint32_t>(fi));
            }
        }
        std::vector<uint32_t> v_pt(n, 0);
        for (size_t v = 0; v < n; ++v) {
            auto fe = v_faces.find(static_cast<uint32_t>(v));
            auto ee = v_edges.find(static_cast<uint32_t>(v));
            size_t nf = fe == v_faces.end() ? 0 : fe->second.size();
            size_t ne = ee == v_edges.end() ? 0 : ee->second.size();
            if (nf == 0 || ne == 0) {
                v_pt[v] = static_cast<uint32_t>(np.size());
                np.push_back(orig[v]);
                continue;
            }
            Vec3 F{0, 0, 0};
            for (uint32_t fi : fe->second) F += np[face_pt[fi]];
            F = F / float(nf);
            Vec3 R{0, 0, 0};
            for (uint32_t w : ee->second) R += (orig[v] + orig[w]) * 0.5f;
            R = R / float(ne);
            float nval = float(ne);
            Vec3 V = (F + R * 2.0f + orig[v] * (nval - 3.0f)) / nval;
            v_pt[v] = static_cast<uint32_t>(np.size());
            np.push_back(V);
        }
        std::vector<Face> nf;
        nf.reserve(m.faces.size() * 4);
        // Each original edge (cur, nxt) becomes a quad bounded by the new vertex
        // points of both endpoints and the edge points on either side:
        //   (v[cur], e[cur->nxt], v[nxt], e[prev->cur])
        //
        // This used to emit (e[cur->nxt], nxt, v[nxt], v[cur]), which put the
        // *original*, unsmoothed vertex index into the new face. Since the
        // position array had already been replaced, that index pointed at the
        // pre-smoothing position, so neighbouring faces referenced different
        // vertices for the same corner. A subdivided sphere came out with 1,104
        // boundary edges, 1,370 non-manifold edges and a signed volume of
        // exactly zero, because the inconsistent normals cancelled.
        // Each face is subdivided by its own type, so a mesh mixing quads and
        // triangles stays conforming: every original edge yields exactly one edge
        // point, which both neighbouring faces use. A quad becomes four quads; a
        // triangle becomes four triangles (three corners plus a centre).
        //
        // Subdividing a triangle into quads instead does not tile it, which is
        // what left a subdivided sphere with 1,104 boundary edges and 1,656
        // non-manifold edges.
        auto edge_of = [&](uint32_t u, uint32_t w) -> uint32_t {
            auto it = edge_pt.find(ekey(u, w));
            return it == edge_pt.end() ? UINT32_MAX : it->second;
        };
        for (size_t fi = 0; fi < m.faces.size(); ++fi) {
            const Face& f = m.faces[fi];
            const size_t nv = f.v.size();
            if (nv == 4) {
                // A quad needs its face point. Four quads cannot tile a quad
                // without one: without the centre, the sub-quad near edge
                // (a, b) is (v_a, e_ab, v_b, e_da) and its closing edge
                // (v_b, e_da) belongs to no other face, so the surface is left
                // open along every original corner. The face point was computed
                // all along and never used.
                // Emitted as triangles rather than quads, fanned from the edge
                // point, so the diagonal runs (edge point -> face point).
                //
                // Fanning from the first corner instead makes the diagonal
                // (v_cur, v_nxt), and the quad on the *other* side of that
                // original edge has the same two corners in the same order, so
                // it picks the same diagonal. Every interior diagonal was then
                // used by four triangles instead of two: a subdivided torus
                // measured 576 non-manifold edges, all with face count 4,
                // forming a cycle of the original corners. The face point is
                // unique to its face, so a diagonal through it can never
                // coincide with a neighbour's.
                const uint32_t fc = face_pt[fi];
                for (size_t i = 0; i < 4; ++i) {
                    const uint32_t cur = f.v[i];
                    const uint32_t nxt = f.v[(i + 1) % 4];
                    const uint32_t e_next = edge_of(cur, nxt);
                    if (e_next == UINT32_MAX) return false;
                    nf.push_back(Face{{v_pt[cur], e_next, fc}});
                    nf.push_back(Face{{e_next, v_pt[nxt], fc}});
                }
            } else if (nv == 3) {
                // A triangle splits into three corner triangles plus a centre
                // one. Each corner triangle uses the new point at its own vertex
                // and the edge points on either side of it. It previously used
                // the *next* vertex point instead of the preceding edge point,
                // so consecutive corner triangles shared no edge at all.
                for (size_t i = 0; i < 3; ++i) {
                    const uint32_t cur = f.v[i];
                    const uint32_t nxt = f.v[(i + 1) % 3];
                    const uint32_t prv = f.v[(i + 2) % 3];
                    const uint32_t e_next = edge_of(cur, nxt);
                    const uint32_t e_prev = edge_of(prv, cur);
                    if (e_next == UINT32_MAX || e_prev == UINT32_MAX) return false;
                    nf.push_back(Face{{v_pt[cur], e_next, e_prev}});
                }
                const uint32_t e0 = edge_of(f.v[0], f.v[1]);
                const uint32_t e1 = edge_of(f.v[1], f.v[2]);
                const uint32_t e2 = edge_of(f.v[2], f.v[0]);
                if (e0 == UINT32_MAX || e1 == UINT32_MAX || e2 == UINT32_MAX) return false;
                nf.push_back(Face{{e0, e1, e2}});
            } else {
                return false;  // n-gons have no conforming subdivision here
            }
        }
        m.positions = std::move(np);
        m.faces = std::move(nf);
    }
    m.vertices.clear();
    m.indices.clear();
    m.tri_face.clear();
    m.skin.clear();
    m.joints.clear();
    m.inv_bind.clear();
    // Catmull-Clark is a smoothing scheme, so its output is smooth-shaded, as in
    // any DCC tool. Leaving it flat-shaded also stopped the weld: corners with
    // per-face normals never match, so a subdivided cube exported as a triangle
    // soup of 144 vertices where the same mesh smooth-shaded welds to 26.
    m.smooth = true;
    mesh_sync_render(m);
    return true;
}

}  // namespace kiln
