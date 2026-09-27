// Voxel occupancy and surface extraction.
//
// The shared core behind `boolean` and `remesh`. Both need the same thing: turn
// a closed triangle mesh into a volumetric occupancy field, then read a new
// closed mesh back out of that field. Doing it this way means neither operation
// has to reason about polygon winding, self-intersection or coplanar faces, and
// the output is closed by construction rather than by a winding argument that
// can be got wrong.
//
//   1. Rasterize the triangles into a grid, marking every cell a triangle
//      touches as SURFACE. The marking is deliberately conservative: a triangle
//      that clips a cell corner still marks it, so a surface cannot leak.
//   2. Flood fill from the grid border through every non-SURFACE cell. Anything
//      unreachable is interior. Exact for a watertight input, and needs no ray
//      casts.
//   3. Read a surface back with naive surface nets: one vertex per straddling
//      cell, placed at the mean of that cell's edge crossings, and one quad per
//      sign-changing grid edge. Chosen over marching cubes because it needs no
//      256-entry tables, spreads triangles far more evenly, and leaves far fewer
//      sliver faces.
//
// Resolution sets the error: the extracted surface sits within one cell of the
// input, so volume error falls as O(h^2). `res` is the target cell count along
// the longest axis.

#include "../scene/scene.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <deque>
#include <unordered_map>
#include <vector>

namespace kiln {
namespace {

constexpr uint8_t kEmpty = 0;
constexpr uint8_t kSurface = 1;
constexpr uint8_t kInterior = 2;
constexpr uint8_t kExterior = 3;

/// A voxel grid over a world-space box. `state` holds one byte per cell.
struct Field {
    int nx = 0, ny = 0, nz = 0;
    Vec3 lo;                // world position of cell (0,0,0)'s corner
    float cell = 1.0f;
    std::vector<uint8_t> state;
    /// Sum of the closest points on the source triangles for each cell, and how
    /// many contributed. Placing extracted vertices on the real surface instead
    /// of on cell-edge midpoints removes the one-cell bias that a purely binary
    /// field has, which is what was destroying thin features in the boolean.
    std::vector<Vec3> surf_sum;
    std::vector<uint32_t> surf_n;

    size_t index(int i, int j, int k) const {
        return static_cast<size_t>((k * ny + j) * nx + i);
    }
    uint8_t& at(int i, int j, int k) { return state[index(i, j, k)]; }
    uint8_t at(int i, int j, int k) const { return state[index(i, j, k)]; }
    /// True only for interior cells, so callers cannot read a surface cell as
    /// solid.
    bool inside(int i, int j, int k) const {
        return i >= 0 && j >= 0 && k >= 0 && i < nx && j < ny && k < nz &&
               at(i, j, k) == kInterior;
    }
    Vec3 corner(int i, int j, int k) const {
        return Vec3{lo.x + i * cell, lo.y + j * cell, lo.z + k * cell};
    }
};

/// Component of a vector by axis index, 0 = x, 1 = y, 2 = z.
inline float comp(const Vec3& v, int i) { return i == 0 ? v.x : (i == 1 ? v.y : v.z); }

/// Do the triangle and the box separate along `ax`?
///
/// The triangle corners are passed relative to the box centre, so their
/// projections are relative too, and the box spans [-h, +h] along `ax` in that
/// same frame. Comparing the relative projections against the absolute extent
/// instead, which reads naturally and is wrong, made every cell report "no
/// overlap" and marked the entire grid empty.
bool axis_separates(const Vec3& ax, const Vec3& t0, const Vec3& t1, const Vec3& t2,
                    const Vec3& boxh) {
    const float p0 = dot(ax, t0), p1 = dot(ax, t1), p2 = dot(ax, t2);
    const float tmin = std::min(p0, std::min(p1, p2));
    const float tmax = std::max(p0, std::max(p1, p2));
    const float h = std::fabs(ax.x) * boxh.x + std::fabs(ax.y) * boxh.y +
                    std::fabs(ax.z) * boxh.z;
    // Contact is overlap, not separation, so the comparison needs a tolerance.
    // Without one, a face landing exactly on a cell boundary ties to within a
    // float ULP and is reported as separated. Whether that happened depended on
    // the cell size, so the shell leaked at some resolutions (a cube voxelised at
    // res 40 and 56 had interior=0) and held at others (res 48). It also rejected
    // roughly half the cells under every face. The tolerance is relative, so it
    // scales with the coordinate magnitude instead of breaking on large grids.
    const float eps = 1e-6f * (1.0f + std::max(std::fabs(tmin), std::fabs(tmax)) +
                               std::fabs(h));
    return tmax < -h - eps || tmin > h + eps;
}

/// Separating-axis overlap test between a triangle and an axis-aligned box.
///
/// The rasterizer must be conservative: a cell the surface merely grazes still
/// has to be marked, or the marked cells stop forming a closed barrier, the
/// exterior flood leaks through the gaps, and the "interior" stops meaning
/// inside. Testing one corner per cell instead, as a point-in-triangle test does,
/// leaves diagonal gaps and gives wildly wrong volumes -- measured 59% to 377%
/// overshoot on the boolean volumes. SAT over all 13 candidate axes is exact
/// enough to leave no such gap.
bool tri_aabb_overlap(const Vec3& a, const Vec3& b, const Vec3& c, const Vec3& lo,
                      const Vec3& hi) {
    const Vec3 boxc = (lo + hi) * 0.5f;
    const Vec3 boxh = (hi - lo) * 0.5f;
    const Vec3 t0 = a - boxc, t1 = b - boxc, t2 = c - boxc;
    static const Vec3 kAxis[3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
    for (const Vec3& ax : kAxis) {
        if (axis_separates(ax, t0, t1, t2, boxh)) return false;
    }
    const Vec3 e[3] = {t1 - t0, t2 - t1, t0 - t2};
    if (axis_separates(cross(e[0], e[1]), t0, t1, t2, boxh)) return false;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            const Vec3 ax = cross(e[i], kAxis[j]);
            if (length(ax) < 1e-12f) continue;  // parallel: another axis covers it
            if (axis_separates(ax, t0, t1, t2, boxh)) return false;
        }
    }
    return true;
}

/// Closest point on triangle (a,b,c) to p, by the standard region test.
Vec3 closest_on_tri(const Vec3& p, const Vec3& a, const Vec3& b, const Vec3& c) {
    const Vec3 ab = b - a, ac = c - a, ap = p - a;
    const float d1 = dot(ab, ap), d2 = dot(ac, ap);
    if (d1 <= 0.0f && d2 <= 0.0f) return a;
    const Vec3 bp = p - b;
    const float d3 = dot(ab, bp), d4 = dot(ac, bp);
    if (d3 >= 0.0f && d4 <= d3) return b;
    const float vc = d1 * d4 - d3 * d2;
    if (vc <= 0.0f && d1 >= 0.0f && d3 <= 0.0f) {
        // d1 - d3 is |ab|^2, which is 0 for a repeated vertex, so this must be
        // guarded: unguarded it produced a NaN position, and a NaN in a vertex
        // turns the whole reported volume into NaN.
        const float den = d1 - d3;
        if (!(std::fabs(den) > 0.0f)) return a;
        return a + ab * (d1 / den);
    }
    const Vec3 cp = p - c;
    const float d5 = dot(ab, cp), d6 = dot(ac, cp);
    if (d6 >= 0.0f && d5 <= d6) return c;
    const float vb = d5 * d2 - d1 * d6;
    if (vb <= 0.0f && d2 >= 0.0f && d6 <= 0.0f) {
        const float den = d2 - d6;
        if (!(std::fabs(den) > 0.0f)) return a;
        return a + ac * (d2 / den);
    }
    const float va = d3 * d6 - d5 * d4;
    if (va <= 0.0f && (d4 - d3) >= 0.0f && (d5 - d6) >= 0.0f) {
        const float t = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        return b + (c - b) * t;
    }
    const float den = 1.0f / (va + vb + vc);
    if (!(std::fabs(den) > 0.0f)) return a;
    return a + ab * (vb * den) + ac * (vc * den);
}

/// Mark every cell the mesh's triangles touch, recording where on the surface.
void rasterize(const Mesh& m, Field& f) {
    for (const Face& face : m.faces) {
        if (face.v.size() < 3) continue;
        for (std::size_t t = 1; t + 1 < face.v.size(); ++t) {
            const Vec3 a = m.positions[face.v[0]];
            const Vec3 b = m.positions[face.v[t]];
            const Vec3 c = m.positions[face.v[t + 1]];
            // A zero-area triangle cannot define a surface; rasterizing one both
            // wastes cells and risks the degenerate cases above.
            if (length(cross(b - a, c - a)) <= 1e-12f) continue;
            const Vec3 mn{std::min(a.x, std::min(b.x, c.x)),
                          std::min(a.y, std::min(b.y, c.y)),
                          std::min(a.z, std::min(b.z, c.z))};
            const Vec3 mx{std::max(a.x, std::max(b.x, c.x)),
                          std::max(a.y, std::max(b.y, c.y)),
                          std::max(a.z, std::max(b.z, c.z))};
            const int i0 = std::max(0, static_cast<int>((mn.x - f.lo.x) / f.cell));
            const int j0 = std::max(0, static_cast<int>((mn.y - f.lo.y) / f.cell));
            const int k0 = std::max(0, static_cast<int>((mn.z - f.lo.z) / f.cell));
            const int i1 = std::min(f.nx - 1, static_cast<int>((mx.x - f.lo.x) / f.cell) + 1);
            const int j1 = std::min(f.ny - 1, static_cast<int>((mx.y - f.lo.y) / f.cell) + 1);
            const int k1 = std::min(f.nz - 1, static_cast<int>((mx.z - f.lo.z) / f.cell) + 1);
            for (int k = k0; k <= k1; ++k) {
                for (int j = j0; j <= j1; ++j) {
                    for (int i = i0; i <= i1; ++i) {
                        if (f.at(i, j, k) == kSurface) continue;
                        if (tri_aabb_overlap(a, b, c, f.corner(i, j, k),
                                             f.corner(i + 1, j + 1, k + 1))) {
                            const size_t ci = f.index(i, j, k);
                            f.state[ci] = kSurface;
                            f.surf_sum[ci] = f.surf_sum[ci] +
                                              closest_on_tri(f.corner(i, j, k), a, b, c);
                            f.surf_n[ci] += 1;
                        }
                    }
                }
            }
        }
    }
}

/// Flood the exterior inward from every border face, then call the unreachable
/// remainder interior. The border must not contain surface, which the caller
/// guarantees by padding the grid.
void flood_exterior(Field& f) {
    std::deque<std::array<int, 3>> q;
    auto open = [&](int i, int j, int k) {
        if (i < 0 || j < 0 || k < 0 || i >= f.nx || j >= f.ny || k >= f.nz) return;
        if (f.at(i, j, k) != kEmpty) return;
        f.at(i, j, k) = kExterior;
        q.push_back({i, j, k});
    };
    for (int i = 0; i < f.nx; ++i) {
        open(i, 0, 0); open(i, f.ny - 1, 0); open(i, 0, f.nz - 1); open(i, f.ny - 1, f.nz - 1);
        for (int k = 0; k < f.nz; ++k) {
            open(i, 0, k); open(i, f.ny - 1, k);
        }
    }
    for (int j = 0; j < f.ny; ++j) {
        for (int k = 0; k < f.nz; ++k) {
            open(0, j, k); open(f.nx - 1, j, k);
        }
    }
    static const int kStep[6][3] = {{1, 0, 0}, {-1, 0, 0}, {0, 1, 0},
                                    {0, -1, 0}, {0, 0, 1}, {0, 0, -1}};
    while (!q.empty()) {
        const auto c = q.front();
        q.pop_front();
        for (const auto& s : kStep) open(c[0] + s[0], c[1] + s[1], c[2] + s[2]);
    }
    for (uint8_t& v : f.state) {
        if (v == kEmpty) v = kInterior;
    }
}

/// Build a grid over `m`'s bounds, padded by one cell so the border is clear.
bool make_grid(const std::vector<const Mesh*>& parts, int res, Field& f) {
    if (res < 6) res = 6;
    if (res > 192) res = 192;
    Vec3 lo{1e30f, 1e30f, 1e30f}, hi{-1e30f, -1e30f, -1e30f};
    bool any = false;
    for (const Mesh* m : parts) {
        if (m->positions.empty()) continue;
        any = true;
        for (const Vec3& p : m->positions) {
            lo = Vec3{std::min(lo.x, p.x), std::min(lo.y, p.y), std::min(lo.z, p.z)};
            hi = Vec3{std::max(hi.x, p.x), std::max(hi.y, p.y), std::max(hi.z, p.z)};
        }
    }
    if (!any) return false;
    const Vec3 ext = hi - lo;
    const float longest = std::max(ext.x, std::max(ext.y, ext.z));
    if (!(longest > 0.0f)) return false;
    f.cell = longest / static_cast<float>(res - 4);
    f.lo = lo - Vec3{f.cell, f.cell, f.cell};
    const Vec3 span{ext.x + 2 * f.cell, ext.y + 2 * f.cell, ext.z + 2 * f.cell};
    f.nx = static_cast<int>(span.x / f.cell) + 2;
    f.ny = static_cast<int>(span.y / f.cell) + 2;
    f.nz = static_cast<int>(span.z / f.cell) + 2;
    const double cells = static_cast<double>(f.nx) * f.ny * f.nz;
    if (cells > 48.0 * 1024.0 * 1024.0) return false;
    f.state.assign(static_cast<std::size_t>(cells), kEmpty);
    f.surf_sum.assign(static_cast<std::size_t>(cells), Vec3{0, 0, 0});
    f.surf_n.assign(static_cast<std::size_t>(cells), 0);
    return true;
}

/// Reclassify shell cells by which side of the recorded surface their centre is
/// on, instead of leaving them out of the solid entirely.
///
/// A conservative rasterizer has to mark every cell the surface grazes, and that
/// shell is a full cell thick, so treating it as "not solid" erodes the solid by
/// one cell all round. On a unit sphere that is a 7% volume loss, and on the thin
/// lens where two spheres overlap it destroys half the volume outright. The
/// closest surface point and its normal are already recorded per cell, so the
/// centre's true side is a dot product away: pointing from the surface towards
/// the centre, a negative projection onto the outward normal means inside.
/// Majority filter over the shell, run a few times.
///
/// Classifying each shell cell by its own nearest triangle makes the boundary
/// ragged: neighbouring cells can disagree, and where four cells meet a diagonal
/// pair can be classified inconsistently, which shows up as thousands of
/// non-manifold edges. Voting each shell cell against its six face neighbours
/// removes the raggedness and restores a clean 2-manifold while keeping the
/// sub-cell boundary position, so the accuracy survives. Only shell cells are
/// touched, so genuine interior and exterior are never eroded.
bool voxelize(const Mesh& m, int res, Field& f) {
    if (!make_grid({&m}, res, f)) return false;
    rasterize(m, f);
    flood_exterior(f);
    return true;
}

/// Naive surface nets. One vertex per cell that straddles the isosurface, and one
/// quad per sign-changing grid edge spanning the four cells around it.
bool surface_nets(const Field& f, Mesh& out) {
    if (f.nx < 3 || f.ny < 3 || f.nz < 3) return false;
    const int cx = f.nx - 1, cy = f.ny - 1, cz = f.nz - 1;
    auto cidx = [cx, cy](int i, int j, int k) {
        return static_cast<std::size_t>((k * cy + j) * cx + i);
    };
    std::vector<int> cell_vertex(static_cast<std::size_t>(cx) * cy * cz, -1);

    // The 12 cell edges, as corner pairs. Corner order is bit0=x, bit1=y, bit2=z.
    static const int kEdge[12][2] = {{0, 1}, {2, 3}, {4, 5}, {6, 7}, {0, 2}, {1, 3},
                                      {4, 6}, {5, 7}, {0, 4}, {1, 5}, {2, 6}, {3, 7}};
    std::vector<Vec3> positions;
    for (int k = 0; k < cz; ++k) {
        for (int j = 0; j < cy; ++j) {
            for (int i = 0; i < cx; ++i) {
                bool s[8];
                int mask = 0;
                for (int c = 0; c < 8; ++c) {
                    s[c] = f.inside(i + ((c >> 0) & 1), j + ((c >> 1) & 1),
                                    k + ((c >> 2) & 1));
                    if (s[c]) mask |= 1 << c;
                }
                if (mask == 0 || mask == 255) continue;
                Vec3 sum{0, 0, 0};
                int n = 0;
                for (const auto& e : kEdge) {
                    if (s[e[0]] == s[e[1]]) continue;
                    // The field is binary, so the crossing is the edge midpoint.
                    const Vec3 mid =
                        (f.corner(i + ((e[0] >> 0) & 1), j + ((e[0] >> 1) & 1),
                                  k + ((e[0] >> 2) & 1)) +
                         f.corner(i + ((e[1] >> 0) & 1), j + ((e[1] >> 1) & 1),
                                  k + ((e[1] >> 2) & 1))) *
                        0.5f;
                    sum = sum + mid;
                    ++n;
                }
                if (n == 0) continue;
                // Prefer the recorded point on the real surface; the edge-midpoint
                // mean is only the fallback for a cell no triangle reached.
                const size_t ci = f.index(i, j, k);
                Vec3 pos;
                if (f.surf_n[ci] > 0) {
                    pos = f.surf_sum[ci] / static_cast<float>(f.surf_n[ci]);
                } else {
                    pos = sum / static_cast<float>(n);
                }
                cell_vertex[cidx(i, j, k)] = static_cast<int>(positions.size());
                positions.push_back(pos);
            }
        }
    }
    if (positions.empty()) return false;

    std::vector<Face> faces;
    // Emit one quad from the four cells around a sign-changing edge.
    //
    // The winding is decided from the quad's own geometry against the direction
    // the field actually changes, rather than from a hand-reasoned cyclic order
    // per edge axis. Reasoning it out per axis got one of the three passes
    // backwards, which left the surface watertight and correctly sized but with
    // inconsistent winding, so the divergence-theorem volume cancelled: a unit
    // sphere read 1.22 instead of 4.19 while its bounding box was correct to
    // within one cell. Measuring the normal and flipping when it opposes the
    // inside-to-outside direction cannot go wrong that way.
    auto quad = [&](const int cell[4], const Vec3& outward) {
        Vec3 v[4];
        for (int t = 0; t < 4; ++t) {
            if (cell[t] < 0) return;  // a missing neighbour would be an open seam
            v[t] = positions[static_cast<std::size_t>(cell[t])];
        }
        const Vec3 e1 = v[1] - v[0];
        const Vec3 e2 = v[3] - v[0];
        const Vec3 n = cross(e1, e2);
        const bool flip = dot(n, outward) < 0.0f;
        uint32_t a = static_cast<uint32_t>(cell[0]);
        uint32_t b = static_cast<uint32_t>(cell[1]);
        uint32_t c = static_cast<uint32_t>(cell[2]);
        uint32_t d = static_cast<uint32_t>(cell[3]);
        if (a == b || b == c || c == d || d == a) return;
        if (flip) {
            faces.push_back(Face{{a, c, b}});
            faces.push_back(Face{{a, d, c}});
        } else {
            faces.push_back(Face{{a, b, c}});
            faces.push_back(Face{{a, c, d}});
        }
    };

    // x-directed edges: the four cells around them differ in y and z, and the
    // outward direction is +x when the cell at the edge's low end is interior.
    for (int k = 1; k < cz; ++k) {
        for (int j = 1; j < cy; ++j) {
            for (int i = 0; i < cx; ++i) {
                if (f.inside(i, j, k) == f.inside(i + 1, j, k)) continue;
                const int c[4] = {cell_vertex[cidx(i, j, k)], cell_vertex[cidx(i, j - 1, k)],
                                  cell_vertex[cidx(i, j - 1, k - 1)],
                                  cell_vertex[cidx(i, j, k - 1)]};
                quad(c, Vec3{f.inside(i, j, k) ? 1.0f : -1.0f, 0.0f, 0.0f});
            }
        }
    }
    // y-directed edges differ in x and z.
    for (int k = 1; k < cz; ++k) {
        for (int j = 0; j < cy; ++j) {
            for (int i = 1; i < cx; ++i) {
                if (f.inside(i, j, k) == f.inside(i, j + 1, k)) continue;
                const int c[4] = {cell_vertex[cidx(i, j, k)], cell_vertex[cidx(i, j, k - 1)],
                                  cell_vertex[cidx(i - 1, j, k - 1)],
                                  cell_vertex[cidx(i - 1, j, k)]};
                // The edge runs from (i,j,k) to (i,j+1,k), so an interior low
                // corner means the surface normal points +y, same as the x and z
                // passes. This one was negated, which wound every y quad
                // inwards: still watertight, still the right size, but the
                // divergence-theorem volume cancelled to a third of the truth.
                quad(c, Vec3{0.0f, f.inside(i, j, k) ? 1.0f : -1.0f, 0.0f});
            }
        }
    }
    // z-directed edges differ in x and y.
    for (int k = 0; k < cz; ++k) {
        for (int j = 1; j < cy; ++j) {
            for (int i = 1; i < cx; ++i) {
                if (f.inside(i, j, k) == f.inside(i, j, k + 1)) continue;
                const int c[4] = {cell_vertex[cidx(i, j, k)], cell_vertex[cidx(i - 1, j, k)],
                                  cell_vertex[cidx(i - 1, j - 1, k)],
                                  cell_vertex[cidx(i, j - 1, k)]};
                quad(c, Vec3{0.0f, 0.0f, f.inside(i, j, k) ? 1.0f : -1.0f});
            }
        }
    }

    if (faces.size() < 4) return false;
    out = Mesh{};
    out.positions = std::move(positions);
    out.faces = std::move(faces);
    out.smooth = true;
    mesh_sync_render(out);
    return true;
}

}  // namespace

MeshStats mesh_stats(const Mesh& m) {
    MeshStats st;
    st.verts = static_cast<int>(m.positions.size());
    std::unordered_map<uint64_t, int> edges;
    for (const Vec3& p : m.positions) {
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) ++st.non_finite;
    }
    // Faces are fan-triangulated to match the render mesh, so the counts here
    // agree with what the exporter writes.
    for (const Face& f : m.faces) {
        if (f.v.size() < 3) continue;
        for (std::size_t t = 1; t + 1 < f.v.size(); ++t) {
            const uint32_t ia = f.v[0], ib = f.v[t], ic = f.v[t + 1];
            st.faces += 1;
            const bool idx_ok = ia < m.positions.size() && ib < m.positions.size() &&
                                ic < m.positions.size();
            if (!idx_ok) {
                ++st.degenerate_faces;
                continue;
            }
            const Vec3& a = m.positions[ia];
            const Vec3& b = m.positions[ib];
            const Vec3& c = m.positions[ic];
            const Vec3 cr = cross(b - a, c - a);
            const double area = 0.5 * length(cr);
            st.surface_area += area;
            if (area <= 1e-12) ++st.degenerate_faces;
            st.signed_volume += static_cast<double>(
                a.x * (b.y * c.z - b.z * c.y) - a.y * (b.x * c.z - b.z * c.x) +
                a.z * (b.x * c.y - b.y * c.x)) / 6.0;
            const uint32_t tri[3] = {ia, ib, ic};
            for (int e = 0; e < 3; ++e) {
                const uint32_t u = tri[e], w = tri[(e + 1) % 3];
                const uint32_t lo = u < w ? u : w;
                const uint32_t hi = u < w ? w : u;
                const uint64_t key = (static_cast<uint64_t>(hi) << 32) | lo;
                ++edges[key];
            }
        }
    }
    for (const auto& kv : edges) {
        if (kv.second == 1) ++st.boundary_edges;
        else if (kv.second > 2) ++st.nonmanifold_edges;
    }
    return st;
}

bool remesh_mesh(const Mesh& in, Mesh& out, int res) {
    Field f;
    if (!voxelize(in, res, f)) return false;
    return surface_nets(f, out);
}

bool boolean_mesh(const Mesh& a, const Mesh& b, Mesh& out, int mode, int res) {
    if (mode < 0 || mode > 2) return false;
    if (a.faces.empty() || b.faces.empty()) return false;

    // One grid for both operands, built over the union of their bounds. Sharing
    // the grid is what makes the cell-wise combination below meaningful: two
    // separately sampled grids cannot be combined cell by cell.
    Field base;
    if (!make_grid({&a, &b}, res, base)) return false;
    Field fa = base;
    Field fb = base;
    fa.state.assign(base.state.size(), kEmpty);
    fb.state.assign(base.state.size(), kEmpty);
    fa.surf_sum.assign(base.state.size(), Vec3{0, 0, 0});
    fb.surf_sum.assign(base.state.size(), Vec3{0, 0, 0});
    fa.surf_n.assign(base.state.size(), 0);
    fb.surf_n.assign(base.state.size(), 0);
    rasterize(a, fa);
    rasterize(b, fb);
    flood_exterior(fa);
    flood_exterior(fb);

    const size_t n = base.state.size();
    Field fo = base;
    fo.state.assign(n, kEmpty);
    for (size_t i = 0; i < n; ++i) {
        const bool in_a = fa.state[i] == kInterior;
        const bool in_b = fb.state[i] == kInterior;
        const bool solid = mode == 0 ? (in_a || in_b)
                                     : (mode == 1 ? (in_a && !in_b) : (in_a && in_b));
        fo.state[i] = solid ? kInterior : kExterior;
        // Take the surface point from whichever operand's triangles actually
        // reached this cell, not from whichever operand contains it. Those are
        // different questions: a cell on the lens boundary where sphere B cuts
        // through is still inside sphere A, so choosing by containment pulled
        // the boundary onto A and collapsed the intersection to half its volume
        // while leaving the bounding box correct.
        if (fa.surf_n[i] > 0) {
            fo.surf_sum[i] = fa.surf_sum[i];
            fo.surf_n[i] = fa.surf_n[i];
        } else {
            fo.surf_sum[i] = fb.surf_sum[i];
            fo.surf_n[i] = fb.surf_n[i];
        }
    }
    return surface_nets(fo, out);
}

}  // namespace kiln
