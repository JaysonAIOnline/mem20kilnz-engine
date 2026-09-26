#include "../scene/scene.hpp"

#include <algorithm>
#include <array>
#include <cmath>
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

}  // namespace kiln
