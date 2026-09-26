#include "software.hpp"

#include "../core/ambient_occlusion.hpp"
#include "../core/contact_shadow.hpp"
#include "../core/samples.hpp"

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include <stb_image_write.h>

#include <algorithm>
#include <cmath>
#include <fstream>

namespace kiln {
namespace {

struct Cam {
    Mat4 view;
    Mat4 proj;
    Vec3 eye;
    Vec3 sun_dir;
    Vec3 sun_col;
    // Kept so the shading pass can rebuild a pixel's view ray: the projective
    // view-projection is never inverted wholesale, we go back through view
    // space instead, and that needs the same fov/aspect the projection used.
    float fov = 50.0f;
    float aspect = 1.0f;
};

Cam make_cam(const Scene& scene, int w, int h) {
    Cam c;
    int ci = scene.active_camera;
    Vec3 eye{4.2f, 2.8f, 5.4f};
    Vec3 center{0, 0.4f, 0};
    float fov = 50;
    if (ci >= 0 && ci < static_cast<int>(scene.nodes.size()) && scene.nodes[ci].type == NodeType::Camera) {
        const Node& n = scene.nodes[ci];
        eye = n.translation;
        center = eye + camera_forward(n.rotation_deg);
        fov = n.fov_deg;
    } else {
        Vec3 lo{1e9, 1e9, 1e9}, hi{-1e9, -1e9, -1e9};
        bool any = false;
        for (const auto& n : scene.nodes) {
            if (n.type != NodeType::Mesh) continue;
            lo.x = std::min(lo.x, n.translation.x);
            lo.y = std::min(lo.y, n.translation.y);
            lo.z = std::min(lo.z, n.translation.z);
            hi.x = std::max(hi.x, n.translation.x);
            hi.y = std::max(hi.y, n.translation.y);
            hi.z = std::max(hi.z, n.translation.z);
            any = true;
        }
        if (any) center = (lo + hi) * 0.5f;
        eye = center + Vec3{4.2f, 2.6f, 5.2f};
    }
    c.eye = eye;
    c.fov = fov;
    c.aspect = float(w) / float(h);
    c.view = look_at(eye, center, {0, 1, 0});
    c.proj = perspective(fov, c.aspect, 0.05f, 200.0f);
    c.sun_dir = normalize(Vec3{0.45f, 0.82f, 0.35f});
    c.sun_col = {1.0f, 0.96f, 0.90f};
    for (const auto& n : scene.nodes) {
        if (n.type != NodeType::Light) continue;
        c.sun_col = n.light_color * n.light_intensity;
        if (n.light == LightType::Sun) {
            Mat4 R = euler_xyz(n.rotation_deg);
            c.sun_dir = normalize(transform_dir(R, {0, 0, -1}));
        }
    }
    return c;
}

void put(Preview& img, int x, int y, Vec3 col) {
    if (x < 0 || y < 0 || x >= img.w || y >= img.h) return;
    size_t i = (static_cast<size_t>(y) * img.w + x) * 4;
    auto srgb = [](float x) {
        x = clamp01(x);
        return static_cast<uint8_t>(std::pow(x, 1.0f / 2.2f) * 255.0f + 0.5f);
    };
    img.rgba[i] = srgb(col.x);
    img.rgba[i + 1] = srgb(col.y);
    img.rgba[i + 2] = srgb(col.z);
    img.rgba[i + 3] = 255;
}

Vec3 shade(Vec3 n, Vec3 albedo, Vec3 sun_dir, Vec3 sun_col, float metallic) {
    n = normalize(n);
    float ndl = std::max(0.0f, dot(n, sun_dir));
    Vec3 ambient = albedo * Vec3{0.12f, 0.13f, 0.16f};
    Vec3 diff = albedo * (1.0f - metallic * 0.7f) * sun_col * ndl;
    float spec = std::pow(std::max(0.0f, ndl), 32.0f) * (0.08f + metallic * 0.5f);
    return ambient + diff + sun_col * spec;
}

// ---------------------------------------------------------------------------
// Screen-space shading pass.
//
// Contact shadows and ambient occlusion both answer one question per pixel: for
// the surface this pixel shows, how much of the world around it stands in the
// way? Both read the sampled depth buffer the colour pass has just written, so
// both are pure software -- no GPU, no second geometry pass, no change to how
// the scene is rasterised.
//
// The depth buffer holds NDC z, which is nonlinear, so both passes first turn it
// back into w, the linear camera distance. That is what lets one thickness
// threshold mean the same thing near the camera and far from it.
// ---------------------------------------------------------------------------

// What the colour pass leaves behind for the shading pass to read. A pixel with
// hit == 0 is backdrop, a ground grid line or a selection marker: the pass
// leaves it exactly as it was drawn.
struct GBuffer {
    std::vector<float> depth;   // NDC z of the nearest triangle; smaller = nearer
    std::vector<Vec3> nrm;      // world-space normal of that triangle
    std::vector<uint8_t> hit;   // 1 = a triangle wrote this pixel
    void reset(int w, int h) {
        const size_t n = static_cast<size_t>(w) * h;
        depth.assign(n, 1e9f);
        nrm.assign(n, Vec3{});
        hit.assign(n, 0);
    }
};

// NDC z -> linear camera distance w. The perspective matrix maps view-space z to
// clip.z = m22*z + m23 with clip.w = -z, so z = m23 / (-ndc_z - m22) and the
// distance along the view axis is w = -z.
inline float ndc_z_to_w(float ndc_z, const Mat4& proj) {
    const float m22 = proj.at(2, 2);
    const float m23 = proj.at(2, 3);
    const float denom = -ndc_z - m22;
    if (std::abs(denom) < 1e-9f) return 1e9f;
    return -m23 / denom;
}

// World position of a pixel, rebuilt from its linear depth by going back through
// view space. The view half of the transform is affine, which invert_affine
// already covers, so the projective matrix is never inverted directly.
inline Vec3 world_from_pixel(int x, int y, float w_lin, const Cam& cam, const Mat4& inv_view, int w,
                             int h) {
    const float nx = 2.0f * (float(x) + 0.5f) / float(w) - 1.0f;
    const float ny = 1.0f - 2.0f * (float(y) + 0.5f) / float(h);
    const float f = 1.0f / std::tan(deg2rad(cam.fov) * 0.5f);
    const Vec3 view_p{nx * w_lin * cam.aspect / f, ny * w_lin / f, -w_lin};
    return transform_point(inv_view, view_p);
}

// Screen position and linear depth of a world point, or false when it is behind
// the camera and cannot be compared against the depth buffer at all.
inline bool to_screen(Vec3 p, const Mat4& vp, int w, int h, float& sx, float& sy, float& w_lin) {
    Vec4 c = mul(vp, Vec4{p, 1});
    if (c.w <= 1e-5f) return false;
    sx = (c.x / c.w * 0.5f + 0.5f) * float(w);
    sy = (1.0f - (c.y / c.w * 0.5f + 0.5f)) * float(h);
    w_lin = c.w;
    return true;
}

// March one screen-space segment and report the first depth-buffer sample that
// swallows it. Returns 1 when the ray is blocked on its first step, falling to
// ~0 when it is blocked only far away or never blocked at all -- so a blocker
// touching the surface reads far stronger than one floating above it, which is
// exactly what distinguishes a contact shadow from a plain drop shadow.
inline float march_occlusion(const GBuffer& gb, const Mat4& vp, const Mat4& proj, Vec3 from, Vec3 to,
                             int w, int h, int steps, float thickness) {
    float ax, ay, aw, bx, by, bw;
    if (!to_screen(from, vp, w, h, ax, ay, aw)) return 0.0f;
    if (!to_screen(to, vp, w, h, bx, by, bw)) return 0.0f;
    const float iw0 = 1.0f / aw;
    const float iw1 = 1.0f / bw;
    for (int s = 1; s <= steps; ++s) {
        const float t = float(s) / float(steps);
        const int ix = int(ax + (bx - ax) * t);
        const int iy = int(ay + (by - ay) * t);
        if (ix < 0 || iy < 0 || ix >= w || iy >= h) continue;
        const size_t id = static_cast<size_t>(iy) * w + ix;
        if (!gb.hit[id]) continue;
        const float ray_w = 1.0f / (iw0 + (iw1 - iw0) * t);  // perspective-correct w along the ray
        const float scene_w = ndc_z_to_w(gb.depth[id], proj);
        if (scene_w < ray_w - thickness) return 1.0f - t;    // nearer along the ray = stronger
    }
    return 0.0f;
}

// Contact shadow for one pixel: how much of the sun does the nearby geometry
// take away? A surface already turned away from the sun is already dark, so it
// is skipped rather than shadowed twice.
inline float sun_occlusion(const GBuffer& gb, const Mat4& vp, const Mat4& proj, Vec3 p, Vec3 n, Vec3 sun,
                           int w, int h) {
    if (dot(n, sun) <= 0.02f) return 0.0f;
    const Vec3 from = p + n * kContactShadowBias;
    const Vec3 to = from + sun * kContactShadowReach;
    return march_occlusion(gb, vp, proj, from, to, w, h, kContactShadowSteps, kContactShadowBias * 8.0f);
}

// Ambient occlusion for one pixel: probe a short hemisphere around the surface
// and count how many probes are swallowed. A crease or a cavity blocks more of
// its own hemisphere than a flat expanse does, so it darkens and the form reads.
inline float ambient_occlusion(const GBuffer& gb, const Mat4& vp, const Mat4& proj, Vec3 p, Vec3 n,
                               int w, int h) {
    // A fixed probe set, each reflected into the surface's hemisphere so the
    // probe count stays constant and the result does not jump as a normal turns.
    static const Vec3 kProbe[kAoDirections] = {
        {1, 0, 0}, {-1, 0, 0}, {0, 1, 0}, {0, 0, 1}, {0, 0, -1}, {0.577f, 0.577f, 0.577f}};
    const Vec3 base = p + n * kAoBias;
    int blocked = 0;
    for (int i = 0; i < kAoDirections; ++i) {
        Vec3 d = kProbe[i];
        const float dn = dot(d, n);
        if (dn < 0.0f) d = d - n * (2.0f * dn);  // mirror into the hemisphere
        d = normalize(d + n * 0.4f);             // and lift off the surface plane
        const Vec3 to = base + d * kAoReach;
        if (march_occlusion(gb, vp, proj, base, to, w, h, kAoSteps, kAoBias * 6.0f) > 0.0f) ++blocked;
    }
    return float(blocked) / float(kAoDirections);
}

// One tightening pass over an occlusion buffer so the shadow edge reads as a
// soft penumbra instead of a hard screen-space cutout. Neighbours that are not
// geometry are skipped rather than blended in, which keeps edges clean.
inline void soften_occlusion(std::vector<float>& v, const std::vector<uint8_t>& hit, int w, int h) {
    std::vector<float> out(v.size(), 0.0f);
    for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x) {
            const size_t id = static_cast<size_t>(y) * w + x;
            if (!hit[id]) continue;
            float acc = v[id] * 2.0f;
            float wsum = 2.0f;
            if (x - 1 >= 0 && hit[id - 1]) { acc += v[id - 1]; wsum += 1.0f; }
            if (x + 1 < w && hit[id + 1]) { acc += v[id + 1]; wsum += 1.0f; }
            if (y - 1 >= 0 && hit[id - w]) { acc += v[id - w]; wsum += 1.0f; }
            if (y + 1 < h && hit[id + w]) { acc += v[id + w]; wsum += 1.0f; }
            out[id] = acc / wsum;
        }
    }
    v.swap(out);
}

// Apply both passes to a finished preview. Returns immediately when neither dial
// is set, so a render with the defaults never enters this code at all and is
// byte-identical to one made before the passes existed.
void apply_shading(Preview& img, const GBuffer& gb, const Scene& scene, int w, int h,
                   const Shading& sh) {
    if (sh.contact_shadow <= 0 && sh.ao <= 0) return;
    const Cam cam = make_cam(scene, w, h);
    const Mat4 vp = mul(cam.proj, cam.view);
    const Mat4 inv_view = invert_affine(cam.view);
    const size_t n = static_cast<size_t>(w) * h;
    const bool want_shadow = sh.contact_shadow > 0;
    const bool want_ao = sh.ao > 0;

    std::vector<float> shadow(n, 0.0f);
    std::vector<float> ao(n, 0.0f);
    for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x) {
            const size_t id = static_cast<size_t>(y) * w + x;
            if (!gb.hit[id]) continue;
            const float w_lin = ndc_z_to_w(gb.depth[id], cam.proj);
            if (w_lin >= 1e8f) continue;
            const Vec3 p = world_from_pixel(x, y, w_lin, cam, inv_view, w, h);
            const Vec3 nrm = gb.nrm[id];
            if (want_shadow) shadow[id] = sun_occlusion(gb, vp, cam.proj, p, nrm, cam.sun_dir, w, h);
            if (want_ao) ao[id] = ambient_occlusion(gb, vp, cam.proj, p, nrm, w, h);
        }
    }
    if (want_shadow && kContactShadowSoften > 0) soften_occlusion(shadow, gb.hit, w, h);

    // The dials scale the computed occlusion. The per-pixel cost is fixed, so 1
    // and 4 differ only in how much of the accumulated occlusion is applied --
    // the dial changes the look, not the render time.
    const float shadow_amt = 0.18f * float(sh.contact_shadow);
    const float ao_amt = 0.16f * float(sh.ao);
    for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x) {
            const size_t id = static_cast<size_t>(y) * w + x;
            if (!gb.hit[id]) continue;
            float f = 1.0f;
            if (shadow_amt > 0.0f) f *= std::max(0.0f, 1.0f - shadow[id] * shadow_amt);
            if (ao_amt > 0.0f) f *= std::max(0.0f, 1.0f - ao[id] * ao_amt);
            if (f >= 1.0f) continue;
            const size_t i = id * 4;
            img.rgba[i] = static_cast<uint8_t>(img.rgba[i] * f + 0.5f);
            img.rgba[i + 1] = static_cast<uint8_t>(img.rgba[i + 1] * f + 0.5f);
            img.rgba[i + 2] = static_cast<uint8_t>(img.rgba[i + 2] * f + 0.5f);
        }
    }
}

}  // namespace

// `gb` is filled only when the caller is going to run the shading pass; passing
// nullptr skips every write to it, so a render with the passes off stays on
// exactly the work it did before they existed.
static Preview render_preview_raw(const Scene& scene, int w, int h, GBuffer* gb) {
    Preview img;
    img.w = w;
    img.h = h;
    img.rgba.assign(static_cast<size_t>(w) * h * 4, 0);
    if (gb) gb->reset(w, h);
    std::vector<float> zbuf(static_cast<size_t>(w) * h, 1e9f);
    Cam cam = make_cam(scene, w, h);
    Mat4 vp = mul(cam.proj, cam.view);

    auto project = [&](Vec3 p, Vec3& ndc, float& wz) -> bool {
        Vec4 c = mul(vp, Vec4{p, 1});
        if (c.w <= 1e-5f) return false;
        ndc = {c.x / c.w, c.y / c.w, c.z / c.w};
        wz = c.w;
        return ndc.z > -1.2f && ndc.z < 1.2f;
    };

    // backdrop
    for (int y = 0; y < h; ++y) {
        float t = float(y) / float(h);
        Vec3 top{0.10f, 0.11f, 0.13f};
        Vec3 bot{0.16f, 0.15f, 0.14f};
        Vec3 col = lerp(bot, top, t);
        for (int x = 0; x < w; ++x) put(img, x, y, col);
    }

    // ground grid
    {
        const int lines = 21;
        float ext = 8;
        for (int i = 0; i < lines; ++i) {
            float t = (float(i) / (lines - 1)) * 2 * ext - ext;
            Vec3 a1{t, 0, -ext}, b1{t, 0, ext};
            Vec3 a2{-ext, 0, t}, b2{ext, 0, t};
            auto draw_line = [&](Vec3 a, Vec3 b, Vec3 col) {
                Vec3 na, nb;
                float wa, wb;
                if (!project(a, na, wa) || !project(b, nb, wb)) return;
                int x0 = int((na.x * 0.5f + 0.5f) * w);
                int y0 = int((1.0f - (na.y * 0.5f + 0.5f)) * h);
                int x1 = int((nb.x * 0.5f + 0.5f) * w);
                int y1 = int((1.0f - (nb.y * 0.5f + 0.5f)) * h);
                int dx = std::abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
                int dy = -std::abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
                int err = dx + dy;
                for (;;) {
                    if (x0 >= 0 && y0 >= 0 && x0 < w && y0 < h) {
                        size_t id = size_t(y0) * w + x0;
                        if (0.99f < zbuf[id]) put(img, x0, y0, col);
                    }
                    if (x0 == x1 && y0 == y1) break;
                    int e2 = 2 * err;
                    if (e2 >= dy) {
                        err += dy;
                        x0 += sx;
                    }
                    if (e2 <= dx) {
                        err += dx;
                        y0 += sy;
                    }
                }
            };
            Vec3 gc = (i == lines / 2) ? Vec3{0.42f, 0.28f, 0.18f} : Vec3{0.22f, 0.22f, 0.24f};
            draw_line(a1, b1, gc);
            draw_line(a2, b2, gc);
        }
    }

    auto raster = [&](Vec3 p0, Vec3 p1, Vec3 p2, Vec3 n0, Vec3 n1, Vec3 n2, Vec3 albedo, float metallic) {
        Vec3 q0, q1, q2;
        float w0, w1, w2;
        if (!project(p0, q0, w0) || !project(p1, q1, w1) || !project(p2, q2, w2)) return;
        auto to_s = [&](Vec3 q) {
            return Vec3{(q.x * 0.5f + 0.5f) * w, (1.0f - (q.y * 0.5f + 0.5f)) * h, q.z};
        };
        Vec3 s0 = to_s(q0), s1 = to_s(q1), s2 = to_s(q2);
        float area = (s1.x - s0.x) * (s2.y - s0.y) - (s1.y - s0.y) * (s2.x - s0.x);
        if (area > 0) {
            std::swap(s1, s2);
            std::swap(n1, n2);
            std::swap(w1, w2);
            area = -area;
        }
        if (area > -1e-4f) return;
        int minx = std::max(0, int(std::floor(std::min({s0.x, s1.x, s2.x}))));
        int maxx = std::min(w - 1, int(std::ceil(std::max({s0.x, s1.x, s2.x}))));
        int miny = std::max(0, int(std::floor(std::min({s0.y, s1.y, s2.y}))));
        int maxy = std::min(h - 1, int(std::ceil(std::max({s0.y, s1.y, s2.y}))));
        auto edge = [](Vec3 a, Vec3 b, float x, float y) {
            return (b.x - a.x) * (y - a.y) - (b.y - a.y) * (x - a.x);
        };
        for (int y = miny; y <= maxy; ++y) {
            for (int x = minx; x <= maxx; ++x) {
                float px = x + 0.5f, py = y + 0.5f;
                float e0 = edge(s1, s2, px, py);
                float e1 = edge(s2, s0, px, py);
                float e2 = edge(s0, s1, px, py);
                if (e0 > 0 || e1 > 0 || e2 > 0) continue;
                float a = e0 / area, b = e1 / area, c = e2 / area;
                float z = a * s0.z + b * s1.z + c * s2.z;
                size_t id = size_t(y) * w + x;
                if (z >= zbuf[id]) continue;
                zbuf[id] = z;
                Vec3 n = normalize(n0 * a + n1 * b + n2 * c);
                if (gb) {
                    gb->depth[id] = z;
                    gb->nrm[id] = n;
                    gb->hit[id] = 1;
                }
                put(img, x, y, shade(n, albedo, cam.sun_dir, cam.sun_col, metallic));
            }
        }
    };

    for (int ni = 0; ni < static_cast<int>(scene.nodes.size()); ++ni) {
        const Node& n = scene.nodes[ni];
        if (n.type != NodeType::Mesh || n.mesh < 0 || n.mesh >= static_cast<int>(scene.meshes.size())) continue;
        const Mesh& mesh0 = scene.meshes[n.mesh];
        Mesh mesh = mesh0;
        std::vector<Vec3> def;
        deform_positions(scene, ni, def);
        if (def.size() == mesh.positions.size()) mesh.positions = def;
        if (mesh0.lod > 0 && mesh0.lod <= int(mesh0.lods.size())) {
            mesh.positions = def.size() == mesh0.lods[mesh0.lod - 1].positions.size() ? def : mesh0.lods[mesh0.lod - 1].positions;
            mesh.faces = mesh0.lods[mesh0.lod - 1].faces;
        }
        mesh_sync_render(mesh);
        Mat4 M = scene.world_matrix(ni);
        Vec3 albedo{0.7f, 0.7f, 0.72f};
        float metallic = 0;
        if (n.material >= 0 && n.material < static_cast<int>(scene.materials.size())) {
            albedo = scene.materials[n.material].albedo;
            metallic = scene.materials[n.material].metallic;
        }
        for (size_t i = 0; i + 2 < mesh.indices.size(); i += 3) {
            const Vertex& a = mesh.vertices[mesh.indices[i]];
            const Vertex& b = mesh.vertices[mesh.indices[i + 1]];
            const Vertex& c = mesh.vertices[mesh.indices[i + 2]];
            Vec3 p0 = transform_point(M, a.p);
            Vec3 p1 = transform_point(M, b.p);
            Vec3 p2 = transform_point(M, c.p);
            Vec3 n0 = normalize(transform_dir(M, a.n));
            Vec3 n1 = normalize(transform_dir(M, b.n));
            Vec3 n2 = normalize(transform_dir(M, c.n));
            Vec3 alb = albedo;
            size_t ti = i / 3;
            if (ni == scene.selection && scene.edit_mode == EditMode::Face && ti < mesh.tri_face.size()) {
                int fi = mesh.tri_face[ti];
                for (int sfi : scene.face_sel) {
                    if (sfi == fi) {
                        alb = lerp(alb, Vec3{0.95f, 0.48f, 0.18f}, 0.62f);
                        break;
                    }
                }
            }
            raster(p0, p1, p2, n0, n1, n2, alb, metallic);
        }
        if (ni == scene.selection && scene.edit_mode == EditMode::Vert) {
            Vec3 vcol{0.95f, 0.48f, 0.16f};
            for (uint32_t vi : scene.vert_sel) {
                if (vi >= mesh.positions.size()) continue;
                Vec3 p = transform_point(M, mesh.positions[vi]);
                Vec3 ndc;
                float wz;
                if (!project(p, ndc, wz)) continue;
                int x = int((ndc.x * 0.5f + 0.5f) * w);
                int y = int((1.0f - (ndc.y * 0.5f + 0.5f)) * h);
                for (int dy = -2; dy <= 2; ++dy)
                    for (int dx = -2; dx <= 2; ++dx)
                        if (std::abs(dx) + std::abs(dy) <= 2) put(img, x + dx, y + dy, vcol);
            }
        }
        if (ni == scene.selection && scene.edit_mode == EditMode::Edge) {
            Vec3 ecol{0.95f, 0.48f, 0.16f};
            for (const Edge& e : scene.edge_sel) {
                if (e.a >= mesh.positions.size() || e.b >= mesh.positions.size()) continue;
                Vec3 a = transform_point(M, mesh.positions[e.a]);
                Vec3 b = transform_point(M, mesh.positions[e.b]);
                Vec3 na, nb;
                float wa, wb;
                if (!project(a, na, wa) || !project(b, nb, wb)) continue;
                int x0 = int((na.x * 0.5f + 0.5f) * w);
                int y0 = int((1.0f - (na.y * 0.5f + 0.5f)) * h);
                int x1 = int((nb.x * 0.5f + 0.5f) * w);
                int y1 = int((1.0f - (nb.y * 0.5f + 0.5f)) * h);
                int dx = std::abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
                int dy = -std::abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
                int err = dx + dy;
                for (;;) {
                    if (x0 >= 0 && y0 >= 0 && x0 < w && y0 < h) put(img, x0, y0, ecol);
                    if (x0 == x1 && y0 == y1) break;
                    int e2 = 2 * err;
                    if (e2 >= dy) {
                        err += dy;
                        x0 += sx;
                    }
                    if (e2 <= dx) {
                        err += dx;
                        y0 += sy;
                    }
                }
            }
        }
    }
    auto draw_seg = [&](Vec3 a, Vec3 b, Vec3 col) {
        Vec3 na, nb;
        float wa, wb;
        if (!project(a, na, wa) || !project(b, nb, wb)) return;
        int x0 = int((na.x * 0.5f + 0.5f) * w);
        int y0 = int((1.0f - (na.y * 0.5f + 0.5f)) * h);
        int x1 = int((nb.x * 0.5f + 0.5f) * w);
        int y1 = int((1.0f - (nb.y * 0.5f + 0.5f)) * h);
        int dx = std::abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
        int dy = -std::abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
        int err = dx + dy;
        for (;;) {
            put(img, x0, y0, col);
            if (x0 == x1 && y0 == y1) break;
            int e2 = 2 * err;
            if (e2 >= dy) {
                err += dy;
                x0 += sx;
            }
            if (e2 <= dx) {
                err += dx;
                y0 += sy;
            }
        }
    };
    for (int ni = 0; ni < int(scene.nodes.size()); ++ni) {
        const Node& n = scene.nodes[ni];
        if (n.type != NodeType::Bone) continue;
        Mat4 W = scene.world_matrix(ni);
        Vec3 a = transform_point(W, {0, 0, 0});
        Vec3 b = transform_point(W, {0, n.bone_length, 0});
        if (n.parent >= 0 && n.parent < int(scene.nodes.size()) && scene.nodes[n.parent].type == NodeType::Bone) {
            Vec3 p = transform_point(scene.world_matrix(n.parent), {0, 0, 0});
            draw_seg(p, a, Vec3{0.95f, 0.72f, 0.22f});
        }
        draw_seg(a, b, ni == scene.selection ? Vec3{0.95f, 0.45f, 0.16f} : Vec3{0.85f, 0.78f, 0.35f});
    }
    return img;
}

// Supersample: render SSxSS and box-filter down. The scene is small enough that
// a 2x2 grid is cheap and it removes the jaggies on every edge and grid line.
// ss is clamped to [kSamplesMin, kSamplesMax]; the caller validates the user
// value first, so this is a backstop rather than the range check.
//
// The shading passes run on the finished, already-filtered image, so the
// antialiasing they sit on is bit-for-bit what it was. With both dials at their
// defaults the function reduces to the original path with no extra work.
Preview render_preview(const Scene& scene, int w, int h, int ss, const Shading& sh) {
    if (ss < kSamplesMin) ss = kSamplesMin;
    if (ss > kSamplesMax) ss = kSamplesMax;
    const bool shade = sh.contact_shadow > 0 || sh.ao > 0;

    if (ss == 1) {
        GBuffer gb;
        Preview img = render_preview_raw(scene, w, h, shade ? &gb : nullptr);
        if (shade) apply_shading(img, gb, scene, w, h, sh);
        return img;
    }

    GBuffer big_gb;
    Preview big = render_preview_raw(scene, w * ss, h * ss, shade ? &big_gb : nullptr);
    Preview out;
    out.w = w;
    out.h = h;
    out.rgba.assign(static_cast<size_t>(w) * h * 4, 0);
    GBuffer gb;
    if (shade) gb.reset(w, h);
    const float inv = 1.0f / float(ss * ss);
    for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x) {
            int acc[3] = {0, 0, 0};
            // The nearest sample in the block also supplies the shading inputs:
            // averaging a normal or a depth across a silhouette would invent a
            // surface that is not there and leak shadow onto the backdrop.
            float best_z = 1e9f;
            size_t best_s = 0;
            bool have_best = false;
            for (int sy = 0; sy < ss; ++sy) {
                for (int sx = 0; sx < ss; ++sx) {
                    const size_t bi = static_cast<size_t>(y * ss + sy) * big.w + (x * ss + sx);
                    const size_t i = bi * 4;
                    acc[0] += big.rgba[i];
                    acc[1] += big.rgba[i + 1];
                    acc[2] += big.rgba[i + 2];
                    if (shade && big_gb.hit[bi] && big_gb.depth[bi] < best_z) {
                        best_z = big_gb.depth[bi];
                        best_s = bi;
                        have_best = true;
                    }
                }
            }
            const size_t o = static_cast<size_t>(y) * w + x;
            out.rgba[o * 4] = static_cast<uint8_t>(acc[0] * inv + 0.5f);
            out.rgba[o * 4 + 1] = static_cast<uint8_t>(acc[1] * inv + 0.5f);
            out.rgba[o * 4 + 2] = static_cast<uint8_t>(acc[2] * inv + 0.5f);
            out.rgba[o * 4 + 3] = 255;
            if (shade && have_best) {
                gb.depth[o] = big_gb.depth[best_s];
                gb.nrm[o] = big_gb.nrm[best_s];
                gb.hit[o] = 1;
            }
        }
    }
    if (shade) apply_shading(out, gb, scene, w, h, sh);
    return out;
}

bool write_png(const Preview& img, const std::string& path, std::string& err) {
    if (path.size() >= 4 && path.substr(path.size() - 4) == ".ppm") {
        std::ofstream f(path);
        if (!f) {
            err = "cannot write " + path;
            return false;
        }
        f << "P3\n" << img.w << " " << img.h << "\n255\n";
        for (int y = 0; y < img.h; ++y) {
            for (int x = 0; x < img.w; ++x) {
                size_t i = (size_t(y) * img.w + x) * 4;
                f << int(img.rgba[i]) << " " << int(img.rgba[i + 1]) << " " << int(img.rgba[i + 2]) << "\n";
            }
        }
        return true;
    }
    if (!stbi_write_png(path.c_str(), img.w, img.h, 4, img.rgba.data(), img.w * 4)) {
        err = "png write failed";
        return false;
    }
    return true;
}

bool encode_png(const Preview& img, std::vector<uint8_t>& out, std::string& err) {
    if (img.w <= 0 || img.h <= 0) {
        err = "cannot encode an empty preview";
        return false;
    }
    if (img.rgba.size() < static_cast<size_t>(img.w) * static_cast<size_t>(img.h) * 4u) {
        err = "preview pixel buffer is short";
        return false;
    }
    out.clear();
    // Same encoder, same 4-channel input and same stride as write_png, so the
    // baked preview is byte-for-byte the PNG the preview command writes.
    stbi_write_png_to_func(
        [](void* ctx, void* data, int size) {
            auto* sink = static_cast<std::vector<uint8_t>*>(ctx);
            const uint8_t* p = static_cast<const uint8_t*>(data);
            sink->insert(sink->end(), p, p + size);
        },
        &out, img.w, img.h, 4, img.rgba.data(), img.w * 4);
    if (out.empty()) {
        err = "png encode failed";
        return false;
    }
    return true;
}

}  // namespace kiln
