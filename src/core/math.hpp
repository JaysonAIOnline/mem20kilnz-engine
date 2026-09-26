#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace kiln {

struct Vec2 {
    float x = 0, y = 0;
    Vec2() = default;
    Vec2(float x, float y) : x(x), y(y) {}
};

struct Vec3 {
    float x = 0, y = 0, z = 0;
    Vec3() = default;
    Vec3(float x, float y, float z) : x(x), y(y), z(z) {}
    Vec3 operator+(Vec3 o) const { return {x + o.x, y + o.y, z + o.z}; }
    Vec3 operator-(Vec3 o) const { return {x - o.x, y - o.y, z - o.z}; }
    Vec3 operator-() const { return {-x, -y, -z}; }
    Vec3 operator*(float s) const { return {x * s, y * s, z * s}; }
    Vec3 operator*(Vec3 o) const { return {x * o.x, y * o.y, z * o.z}; }
    Vec3 operator/(float s) const { return {x / s, y / s, z / s}; }
    Vec3& operator+=(Vec3 o) { x += o.x; y += o.y; z += o.z; return *this; }
};

inline Vec3 operator*(float s, Vec3 v) { return v * s; }

struct Vec4 {
    float x = 0, y = 0, z = 0, w = 0;
    Vec4() = default;
    Vec4(float x, float y, float z, float w) : x(x), y(y), z(z), w(w) {}
    Vec4(Vec3 v, float w) : x(v.x), y(v.y), z(v.z), w(w) {}
};

struct Mat4 {
    float m[16] = {
        1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1
    };
    static Mat4 identity() { return {}; }
    float& at(int r, int c) { return m[c * 4 + r]; }
    float at(int r, int c) const { return m[c * 4 + r]; }
};

inline float dot(Vec3 a, Vec3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
inline Vec3 cross(Vec3 a, Vec3 b) {
    return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}
inline float length(Vec3 v) { return std::sqrt(dot(v, v)); }
inline Vec3 normalize(Vec3 v) {
    float l = length(v);
    return l > 1e-12f ? v / l : Vec3{0, 0, 0};
}
inline Vec3 lerp(Vec3 a, Vec3 b, float t) { return a * (1 - t) + b * t; }
inline float clamp01(float x) { return x < 0 ? 0 : (x > 1 ? 1 : x); }
inline float deg2rad(float d) { return d * 0.017453292519943295f; }
inline float rad2deg(float r) { return r * 57.29577951308232f; }

inline Mat4 mul(const Mat4& a, const Mat4& b) {
    Mat4 r{};
    for (int c = 0; c < 4; ++c) {
        for (int row = 0; row < 4; ++row) {
            r.at(row, c) = a.at(row, 0) * b.at(0, c) + a.at(row, 1) * b.at(1, c) +
                           a.at(row, 2) * b.at(2, c) + a.at(row, 3) * b.at(3, c);
        }
    }
    return r;
}

inline Vec4 mul(const Mat4& a, Vec4 v) {
    return {
        a.at(0, 0) * v.x + a.at(0, 1) * v.y + a.at(0, 2) * v.z + a.at(0, 3) * v.w,
        a.at(1, 0) * v.x + a.at(1, 1) * v.y + a.at(1, 2) * v.z + a.at(1, 3) * v.w,
        a.at(2, 0) * v.x + a.at(2, 1) * v.y + a.at(2, 2) * v.z + a.at(2, 3) * v.w,
        a.at(3, 0) * v.x + a.at(3, 1) * v.y + a.at(3, 2) * v.z + a.at(3, 3) * v.w
    };
}

inline Vec3 transform_point(const Mat4& m, Vec3 p) {
    Vec4 r = mul(m, Vec4{p, 1});
    float w = r.w != 0 ? r.w : 1;
    return {r.x / w, r.y / w, r.z / w};
}

inline Vec3 transform_dir(const Mat4& m, Vec3 d) {
    Vec4 r = mul(m, Vec4{d, 0});
    return {r.x, r.y, r.z};
}

inline Mat4 translate(Vec3 t) {
    Mat4 m = Mat4::identity();
    m.at(0, 3) = t.x;
    m.at(1, 3) = t.y;
    m.at(2, 3) = t.z;
    return m;
}

inline Mat4 scale(Vec3 s) {
    Mat4 m{};
    m.at(0, 0) = s.x;
    m.at(1, 1) = s.y;
    m.at(2, 2) = s.z;
    m.at(3, 3) = 1;
    return m;
}

inline Mat4 rotate_x(float rad) {
    float c = std::cos(rad), s = std::sin(rad);
    Mat4 m = Mat4::identity();
    m.at(1, 1) = c;
    m.at(2, 1) = s;
    m.at(1, 2) = -s;
    m.at(2, 2) = c;
    return m;
}

inline Mat4 rotate_y(float rad) {
    float c = std::cos(rad), s = std::sin(rad);
    Mat4 m = Mat4::identity();
    m.at(0, 0) = c;
    m.at(2, 0) = -s;
    m.at(0, 2) = s;
    m.at(2, 2) = c;
    return m;
}

inline Mat4 rotate_z(float rad) {
    float c = std::cos(rad), s = std::sin(rad);
    Mat4 m = Mat4::identity();
    m.at(0, 0) = c;
    m.at(1, 0) = s;
    m.at(0, 1) = -s;
    m.at(1, 1) = c;
    return m;
}

inline Mat4 euler_xyz(Vec3 deg) {
    return mul(rotate_z(deg2rad(deg.z)), mul(rotate_y(deg2rad(deg.y)), rotate_x(deg2rad(deg.x))));
}

inline Mat4 trs(Vec3 t, Vec3 euler_deg, Vec3 s) {
    return mul(translate(t), mul(euler_xyz(euler_deg), scale(s)));
}

inline Mat4 invert_affine(const Mat4& m) {
    float a00 = m.at(0, 0), a01 = m.at(0, 1), a02 = m.at(0, 2);
    float a10 = m.at(1, 0), a11 = m.at(1, 1), a12 = m.at(1, 2);
    float a20 = m.at(2, 0), a21 = m.at(2, 1), a22 = m.at(2, 2);
    float c00 = a11 * a22 - a12 * a21;
    float c01 = a12 * a20 - a10 * a22;
    float c02 = a10 * a21 - a11 * a20;
    float c10 = a02 * a21 - a01 * a22;
    float c11 = a00 * a22 - a02 * a20;
    float c12 = a01 * a20 - a00 * a21;
    float c20 = a01 * a12 - a02 * a11;
    float c21 = a02 * a10 - a00 * a12;
    float c22 = a00 * a11 - a01 * a10;
    float det = a00 * c00 + a01 * c01 + a02 * c02;
    Mat4 r = Mat4::identity();
    if (std::abs(det) < 1e-12f) return r;
    float inv = 1.0f / det;
    r.at(0, 0) = c00 * inv;
    r.at(0, 1) = c10 * inv;
    r.at(0, 2) = c20 * inv;
    r.at(1, 0) = c01 * inv;
    r.at(1, 1) = c11 * inv;
    r.at(1, 2) = c21 * inv;
    r.at(2, 0) = c02 * inv;
    r.at(2, 1) = c12 * inv;
    r.at(2, 2) = c22 * inv;
    float tx = m.at(0, 3), ty = m.at(1, 3), tz = m.at(2, 3);
    r.at(0, 3) = -(r.at(0, 0) * tx + r.at(0, 1) * ty + r.at(0, 2) * tz);
    r.at(1, 3) = -(r.at(1, 0) * tx + r.at(1, 1) * ty + r.at(1, 2) * tz);
    r.at(2, 3) = -(r.at(2, 0) * tx + r.at(2, 1) * ty + r.at(2, 2) * tz);
    return r;
}

inline Vec3 rotate_around(Vec3 p, Vec3 center, Vec3 axis, float rad) {
    Vec3 v = p - center;
    axis = normalize(axis);
    if (length(axis) < 0.5f) axis = {0, 1, 0};
    float cs = std::cos(rad), sn = std::sin(rad);
    return center + v * cs + cross(axis, v) * sn + axis * (dot(axis, v) * (1.0f - cs));
}

inline Mat4 look_at(Vec3 eye, Vec3 center, Vec3 up) {
    Vec3 f = normalize(center - eye);
    Vec3 r = normalize(cross(f, up));
    Vec3 u = cross(r, f);
    Mat4 m = Mat4::identity();
    m.at(0, 0) = r.x;
    m.at(0, 1) = r.y;
    m.at(0, 2) = r.z;
    m.at(1, 0) = u.x;
    m.at(1, 1) = u.y;
    m.at(1, 2) = u.z;
    m.at(2, 0) = -f.x;
    m.at(2, 1) = -f.y;
    m.at(2, 2) = -f.z;
    m.at(0, 3) = -dot(r, eye);
    m.at(1, 3) = -dot(u, eye);
    m.at(2, 3) = dot(f, eye);
    return m;
}

inline Vec3 camera_forward(Vec3 euler_deg) {
    float pitch = deg2rad(euler_deg.x);
    float yaw = deg2rad(euler_deg.y);
    float cp = std::cos(pitch);
    return normalize(Vec3{std::sin(yaw) * cp, std::sin(pitch), -std::cos(yaw) * cp});
}

inline Vec3 camera_euler_from_forward(Vec3 fwd) {
    fwd = normalize(fwd);
    float pitch = rad2deg(std::asin(std::max(-1.0f, std::min(1.0f, fwd.y))));
    float yaw = rad2deg(std::atan2(fwd.x, -fwd.z));
    return {pitch, yaw, 0};
}

inline Mat4 perspective(float fov_deg, float aspect, float zn, float zf) {
    float f = 1.0f / std::tan(deg2rad(fov_deg) * 0.5f);
    Mat4 m{};
    for (int i = 0; i < 16; ++i) m.m[i] = 0;
    m.at(0, 0) = f / aspect;
    m.at(1, 1) = f;
    m.at(2, 2) = zf / (zn - zf);
    m.at(3, 2) = -1;
    m.at(2, 3) = (zn * zf) / (zn - zf);
    return m;
}

}  // namespace kiln
