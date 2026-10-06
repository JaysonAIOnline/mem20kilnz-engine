#include "gltf.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <fstream>
#include <functional>
#include <set>

namespace kiln {
namespace {

constexpr uint32_t kChunkJson = 0x4E4F534A;
constexpr uint32_t kChunkBin = 0x004E4942;

constexpr int kModePoints = 0;
constexpr int kModeLineLoop = 2;
constexpr int kModeTriangles = 4;

struct Doc {
    nlohmann::json g;
    std::vector<uint8_t> bin;
    std::string base_dir;
};

// nlohmann's const operator[] asserts when the key is absent, and glTF omits
// keys freely. Every optional lookup in this file goes through here.
const nlohmann::json& jget(const nlohmann::json& j, const char* key) {
    static const nlohmann::json empty = nlohmann::json::object();
    if (j.is_object()) {
        auto it = j.find(key);
        if (it != j.end()) return *it;
    }
    return empty;
}

bool jis_array(const nlohmann::json& j, const char* key) {
    const nlohmann::json& v = jget(j, key);
    return v.is_array();
}

int jint(const nlohmann::json& j, const char* key, int dflt) {
    const nlohmann::json& v = jget(j, key);
    return v.is_number_integer() ? v.get<int>() : dflt;
}

size_t jsize(const nlohmann::json& j, const char* key, size_t dflt) {
    const nlohmann::json& v = jget(j, key);
    return v.is_number_integer() ? static_cast<size_t>(v.get<long long>()) : dflt;
}

std::string jstr(const nlohmann::json& j, const char* key, const std::string& dflt) {
    const nlohmann::json& v = jget(j, key);
    return v.is_string() ? v.get<std::string>() : dflt;
}

bool jbool(const nlohmann::json& j, const char* key, bool dflt) {
    const nlohmann::json& v = jget(j, key);
    return v.is_boolean() ? v.get<bool>() : dflt;
}

bool read_file(const std::string& path, std::vector<uint8_t>& out) {
    std::ifstream f(path, std::ios::binary);
    if (!f) return false;
    f.seekg(0, std::ios::end);
    std::streamoff n = f.tellg();
    if (n < 0) return false;
    f.seekg(0, std::ios::beg);
    out.resize(static_cast<size_t>(n));
    if (n > 0) f.read(reinterpret_cast<char*>(out.data()), n);
    return static_cast<bool>(f);
}

std::string dir_of(const std::string& path) {
    size_t p = path.find_last_of('/');
    if (p == std::string::npos) return ".";
    if (p == 0) return "/";
    return path.substr(0, p);
}

int b64_val(char c) {
    if (c >= 'A' && c <= 'Z') return c - 'A';
    if (c >= 'a' && c <= 'z') return c - 'a' + 26;
    if (c >= '0' && c <= '9') return c - '0' + 52;
    if (c == '+') return 62;
    if (c == '/') return 63;
    return -1;
}

bool decode_data_uri(const std::string& uri, std::vector<uint8_t>& out) {
    size_t comma = uri.find(',');
    if (comma == std::string::npos) return false;
    if (uri.rfind("data:", 0) != 0) return false;
    std::string meta = uri.substr(5, comma - 5);
    std::string b64 = uri.substr(comma + 1);
    if (meta.find("base64") == std::string::npos) {
        out.assign(b64.begin(), b64.end());
        return true;
    }
    int acc = 0, bits = 0;
    for (char c : b64) {
        if (c == '=' || c == '\n' || c == '\r') continue;
        int v = b64_val(c);
        if (v < 0) continue;
        acc = (acc << 6) | v;
        bits += 6;
        if (bits >= 8) {
            bits -= 8;
            out.push_back(static_cast<uint8_t>((acc >> bits) & 0xFF));
        }
    }
    return true;
}

bool load_doc(const std::string& path, Doc& d, std::string& err) {
    std::vector<uint8_t> raw;
    if (!read_file(path, raw)) {
        err = "cannot read " + path;
        return false;
    }
    d.base_dir = dir_of(path);

    if (raw.size() >= 12 && std::memcmp(raw.data(), "glTF", 4) == 0) {
        uint32_t version = 0, total = 0;
        std::memcpy(&version, raw.data() + 4, 4);
        std::memcpy(&total, raw.data() + 8, 4);
        if (version != 2) {
            err = "glb version " + std::to_string(version) + " is not 2";
            return false;
        }
        if (total > raw.size()) {
            err = "glb header length " + std::to_string(total) + " exceeds file size " +
                  std::to_string(raw.size());
            return false;
        }
        size_t off = 12;
        const char* json_ptr = nullptr;
        size_t json_len = 0;
        while (off + 8 <= raw.size()) {
            uint32_t clen = 0, ctype = 0;
            std::memcpy(&clen, raw.data() + off, 4);
            std::memcpy(&ctype, raw.data() + off + 4, 4);
            size_t body = off + 8;
            if (body + clen > raw.size()) {
                err = "glb chunk at offset " + std::to_string(off) + " overruns the file";
                return false;
            }
            if (ctype == kChunkJson && !json_ptr) {
                json_ptr = reinterpret_cast<const char*>(raw.data()) + body;
                json_len = clen;
            } else if (ctype == kChunkBin && d.bin.empty()) {
                d.bin.assign(raw.begin() + static_cast<long>(body),
                             raw.begin() + static_cast<long>(body + clen));
            }
            off = body + clen;
            off += (4 - (clen % 4)) % 4;
        }
        if (!json_ptr) {
            err = "glb has no JSON chunk";
            return false;
        }
        try {
            d.g = nlohmann::json::parse(json_ptr, json_ptr + json_len);
        } catch (const std::exception& e) {
            err = std::string("glb JSON chunk is invalid: ") + e.what();
            return false;
        }
        return true;
    }

    std::string text(reinterpret_cast<const char*>(raw.data()), raw.size());
    try {
        d.g = nlohmann::json::parse(text);
    } catch (const std::exception& e) {
        err = std::string("gltf JSON is invalid: ") + e.what();
        return false;
    }
    if (d.g.contains("buffers") && d.g["buffers"].is_array() && !d.g["buffers"].empty()) {
        const auto& b0 = d.g["buffers"][0];
        if (b0.contains("uri") && b0["uri"].is_string()) {
            std::string uri = b0["uri"].get<std::string>();
            if (uri.rfind("data:", 0) == 0) {
                if (!decode_data_uri(uri, d.bin)) {
                    err = "buffer 0 data URI could not be decoded";
                    return false;
                }
            } else {
                std::string full = (!uri.empty() && uri[0] == '/') ? uri : d.base_dir + "/" + uri;
                if (!read_file(full, d.bin)) {
                    err = "cannot read external buffer " + full;
                    return false;
                }
            }
        }
    }
    return true;
}

int type_components(const std::string& t) {
    if (t == "SCALAR") return 1;
    if (t == "VEC2") return 2;
    if (t == "VEC3") return 3;
    if (t == "VEC4") return 4;
    if (t == "MAT2") return 4;
    if (t == "MAT3") return 9;
    if (t == "MAT4") return 16;
    return 0;
}

int component_size(int component_type) {
    switch (component_type) {
        case 5120:
        case 5121: return 1;
        case 5122:
        case 5123: return 2;
        case 5125:
        case 5126: return 4;
        default: return 0;
    }
}

double read_component(const uint8_t* p, int component_type) {
    switch (component_type) {
        case 5120: { int8_t v; std::memcpy(&v, p, 1); return static_cast<double>(v); }
        case 5121: { uint8_t v; std::memcpy(&v, p, 1); return static_cast<double>(v); }
        case 5122: { int16_t v; std::memcpy(&v, p, 2); return static_cast<double>(v); }
        case 5123: { uint16_t v; std::memcpy(&v, p, 2); return static_cast<double>(v); }
        case 5125: { uint32_t v; std::memcpy(&v, p, 4); return static_cast<double>(v); }
        case 5126: { float v; std::memcpy(&v, p, 4); return static_cast<double>(v); }
        default: return 0.0;
    }
}

float denormalize(double v, int component_type, bool normalized) {
    if (!normalized) return static_cast<float>(v);
    switch (component_type) {
        case 5120: return std::max(static_cast<float>(v) / 127.0f, -1.0f);
        case 5121: return static_cast<float>(v) / 255.0f;
        case 5122: return std::max(static_cast<float>(v) / 32767.0f, -1.0f);
        case 5123: return static_cast<float>(v) / 65535.0f;
        default: return static_cast<float>(v);
    }
}

struct Accessor {
    bool ok = false;
    int component_type = 0;
    int components = 0;
    size_t count = 0;
    size_t elem_size = 0;
    std::vector<double> v;
};

Accessor read_accessor(const Doc& d, int index) {
    Accessor a;
    if (!d.g.contains("accessors") || index < 0 || index >= static_cast<int>(d.g["accessors"].size()))
        return a;
    const auto& ja = d.g["accessors"][static_cast<size_t>(index)];
    a.component_type = jint(ja, "componentType", 0);
    a.components = type_components(ja.value("type", std::string("SCALAR")));
    a.count = jsize(ja, "count", 0);
    int csz = component_size(a.component_type);
    if (a.components <= 0 || csz == 0) return a;
    a.elem_size = static_cast<size_t>(csz) * static_cast<size_t>(a.components);
    const size_t nc = static_cast<size_t>(a.components);
    a.v.assign(a.count * nc, 0.0);

    if (jget(ja, "bufferView").is_number_integer()) {
        int bv = jget(ja, "bufferView").get<int>();
        if (!d.g.contains("bufferViews") || bv < 0 || bv >= static_cast<int>(d.g["bufferViews"].size()))
            return a;
        const auto& jbv = d.g["bufferViews"][static_cast<size_t>(bv)];
        if (jint(jbv, "buffer", 0) != 0 || d.bin.empty()) return a;
        size_t bv_off = jsize(jbv, "byteOffset", 0);
        size_t acc_off = jsize(ja, "byteOffset", 0);
        size_t stride = jsize(jbv, "byteStride", 0);
        if (stride == 0) stride = a.elem_size;
        size_t span = a.count ? (a.count - 1) * stride + a.elem_size : 0;
        if (bv_off + acc_off + span > d.bin.size()) return a;
        const uint8_t* base = d.bin.data();
        for (size_t e = 0; e < a.count; ++e) {
            const uint8_t* p = base + bv_off + acc_off + e * stride;
            for (int c = 0; c < a.components; ++c)
                a.v[e * nc + static_cast<size_t>(c)] =
                    read_component(p + static_cast<size_t>(csz) * static_cast<size_t>(c), a.component_type);
        }
    }

    if (ja.contains("sparse")) {
        const auto& sp = ja["sparse"];
        size_t n = jsize(sp, "count", 0);
        std::vector<size_t> idx(n, 0);
        std::vector<double> vals(n * nc, 0.0);
        if (jget(sp, "indices").is_object() && jis_array(d.g, "bufferViews")) {
            const auto& si = sp["indices"];
            int bv = jint(si, "bufferView", -1);
            int ict = jint(si, "componentType", 5125);
            int isz = component_size(ict);
            if (bv >= 0 && isz > 0 && bv < static_cast<int>(d.g["bufferViews"].size()) && !d.bin.empty()) {
                const auto& jbv = d.g["bufferViews"][static_cast<size_t>(bv)];
                size_t off = jsize(jbv, "byteOffset", 0) +
                             jsize(si, "byteOffset", 0);
                if (off + n * static_cast<size_t>(isz) <= d.bin.size())
                    for (size_t i = 0; i < n; ++i)
                        idx[i] = static_cast<size_t>(read_component(
                            d.bin.data() + off + i * static_cast<size_t>(isz), ict));
            }
        }
        if (jget(sp, "values").is_object() && jis_array(d.g, "bufferViews")) {
            int bv = jint(jget(sp, "values"), "bufferView", -1);
            if (bv >= 0 && bv < static_cast<int>(d.g["bufferViews"].size()) && !d.bin.empty()) {
                const auto& jbv = d.g["bufferViews"][static_cast<size_t>(bv)];
                size_t off = jsize(jbv, "byteOffset", 0) +
                             jsize(jget(sp, "values"), "byteOffset", 0);
                if (off + n * a.elem_size <= d.bin.size())
                    for (size_t i = 0; i < n; ++i) {
                        const uint8_t* p = d.bin.data() + off + i * a.elem_size;
                        for (int c = 0; c < a.components; ++c)
                            vals[i * nc + static_cast<size_t>(c)] = read_component(
                                p + static_cast<size_t>(csz) * static_cast<size_t>(c), a.component_type);
                    }
            }
        }
        for (size_t i = 0; i < idx.size(); ++i) {
            if (idx[i] >= a.count) continue;
            for (int c = 0; c < a.components; ++c)
                a.v[idx[i] * nc + static_cast<size_t>(c)] = vals[i * nc + static_cast<size_t>(c)];
        }
    }

    a.ok = true;
    return a;
}

float acc_f(const Accessor& a, size_t e, int c, bool normalized) {
    if (!a.ok || c >= a.components) return 0.0f;
    size_t i = e * static_cast<size_t>(a.components) + static_cast<size_t>(c);
    if (i >= a.v.size()) return 0.0f;
    return denormalize(a.v[i], a.component_type, normalized);
}

uint32_t acc_u(const Accessor& a, size_t e) {
    if (!a.ok || e >= a.count || e >= a.v.size()) return 0;
    return static_cast<uint32_t>(a.v[e]);
}

struct Trs {
    Vec3 t{0, 0, 0};
    Vec3 r{0, 0, 0};
    Vec3 s{1, 1, 1};
};

Trs decompose(const Mat4& m) {
    Trs out;
    out.t = {m.at(0, 3), m.at(1, 3), m.at(2, 3)};
    Vec3 c0{m.at(0, 0), m.at(1, 0), m.at(2, 0)};
    Vec3 c1{m.at(0, 1), m.at(1, 1), m.at(2, 1)};
    Vec3 c2{m.at(0, 2), m.at(1, 2), m.at(2, 2)};
    float sx = length(c0), sy = length(c1), sz = length(c2);
    if (sx < 1e-12f) sx = 1e-12f;
    if (sy < 1e-12f) sy = 1e-12f;
    if (sz < 1e-12f) sz = 1e-12f;
    c0 = c0 / sx;
    c1 = c1 / sy;
    c2 = c2 / sz;
    float det = c0.x * (c1.y * c2.z - c1.z * c2.y) - c1.x * (c0.y * c2.z - c0.z * c2.y) +
               c2.x * (c0.y * c1.z - c0.z * c1.y);
    if (det < 0.0f) sx = -sx;
    Mat4 rot = Mat4::identity();
    rot.at(0, 0) = c0.x; rot.at(1, 0) = c0.y; rot.at(2, 0) = c0.z;
    rot.at(0, 1) = c1.x; rot.at(1, 1) = c1.y; rot.at(2, 1) = c1.z;
    rot.at(0, 2) = c2.x; rot.at(1, 2) = c2.y; rot.at(2, 2) = c2.z;
    out.r = mat_to_euler_xyz(rot);
    out.s = {sx, sy, sz};
    return out;
}

struct Report {
    int faces = 0;
    int vertices = 0;
    int cameras = 0;
    int skins = 0;
    int bones = 0;
    int skinned_vertices = 0;
    int skipped_modes = 0;
    std::set<std::string> dropped;
    std::set<std::string> exts;
};

void collect_exts(const nlohmann::json& j, Report& rep) {
    if (!j.is_object() && !j.is_array()) return;
    if (j.is_object()) {
        auto it = j.find("extensions");
        if (it != j.end() && it->is_object())
            for (auto e = it->begin(); e != it->end(); ++e) rep.exts.insert(e.key());
    }
    if (j.is_array()) {
        for (const auto& c : j) collect_exts(c, rep);
    } else {
        for (auto c = j.begin(); c != j.end(); ++c) collect_exts(c.value(), rep);
    }
}

bool read_primitive(const Doc& d, const nlohmann::json& jp, Mesh& mesh, bool& smooth,
                    int& material, Report& rep, std::string& err) {
    int mode = jp.value("mode", kModeTriangles);
    if (mode < kModeTriangles) {
        rep.skipped_modes++;
        return false;
    }
    if (!jp.contains("attributes") || !jp["attributes"].is_object()) return false;
    const auto& attrs = jp["attributes"];
    if (!attrs.contains("POSITION") || !attrs["POSITION"].is_number_integer()) {
        err = "primitive has no POSITION attribute";
        return false;
    }

    if (!jis_array(d.g, "accessors")) {
        err = "primitive has no accessors array";
        return false;
    }
    int pos_index = attrs["POSITION"].get<int>();
    if (pos_index < 0 || pos_index >= static_cast<int>(d.g["accessors"].size())) {
        err = "POSITION accessor index out of range";
        return false;
    }
    const auto& ja = d.g["accessors"][static_cast<size_t>(pos_index)];
    bool pos_norm = jbool(ja, "normalized", false);
    Accessor pos = read_accessor(d, attrs["POSITION"].get<int>());
    if (!pos.ok || pos.components < 3) {
        err = "POSITION accessor could not be read";
        return false;
    }
    mesh.positions.reserve(pos.count);
    for (size_t i = 0; i < pos.count; ++i)
        mesh.positions.push_back({acc_f(pos, i, 0, pos_norm), acc_f(pos, i, 1, pos_norm),
                                 acc_f(pos, i, 2, pos_norm)});

    Accessor nrm;
    bool has_nrm = false;
    if (attrs.contains("NORMAL") && attrs["NORMAL"].is_number_integer()) {
        nrm = read_accessor(d, attrs["NORMAL"].get<int>());
        has_nrm = nrm.ok && nrm.count == pos.count;
    }
    if (has_nrm) smooth = true;
    // UVs and normals are read, not dropped: they are per-corner attributes and
    // kiln now has somewhere to keep them.
    Accessor uv;
    bool has_uv = false;
    if (attrs.contains("TEXCOORD_0") && attrs["TEXCOORD_0"].is_number_integer()) {
        uv = read_accessor(d, attrs["TEXCOORD_0"].get<int>());
        has_uv = uv.ok && uv.count == pos.count && uv.components >= 2;
        if (!has_uv) rep.dropped.insert("TEXCOORD_0");
    }
    if (attrs.contains("TANGENT")) rep.dropped.insert("TANGENT");
    if (attrs.contains("COLOR_0")) rep.dropped.insert("COLOR_0");
    if (attrs.contains("TEXCOORD_1")) rep.dropped.insert("TEXCOORD_1");
    if (attrs.contains("JOINTS_1")) rep.dropped.insert("JOINTS_1");
    if (attrs.contains("WEIGHTS_1")) rep.dropped.insert("WEIGHTS_1");
    if (has_nrm && attrs.contains("NORMAL")) {
        bool flat = false;
        for (size_t i = 1; i < nrm.count && !flat; ++i)
            if (nrm.v[i] != nrm.v[static_cast<size_t>(i) -
                                     static_cast<size_t>(std::max(1, nrm.components))])
                flat = true;
        if (flat) rep.dropped.insert("SPLIT_NORMALS");
    }

    std::vector<uint32_t> idx;
    if (jp.contains("indices") && jp["indices"].is_number_integer()) {
        Accessor ia = read_accessor(d, jp["indices"].get<int>());
        if (!ia.ok) {
            err = "index accessor could not be read";
            return false;
        }
        idx.reserve(ia.count);
        for (size_t i = 0; i < ia.count; ++i) idx.push_back(acc_u(ia, i));
    } else {
        idx.reserve(pos.count);
        for (size_t i = 0; i < pos.count; ++i) idx.push_back(static_cast<uint32_t>(i));
    }

    size_t tri_count = 0;
    if (mode == kModeTriangles) tri_count = idx.size() / 3;
    else if (mode == 5) tri_count = idx.size() >= 3 ? idx.size() - 2 : 0;
    else if (mode == 6) tri_count = idx.size() >= 3 ? idx.size() - 2 : 0;
    if (tri_count == 0) {
        rep.dropped.insert("MODE_" + std::to_string(mode));
        return false;
    }

    mesh.faces.reserve(tri_count);
    mesh.corner_attr.reserve(tri_count * 3);
    // The corner attributes are appended in lockstep with the faces, because
    // mesh_sync_render reads them as three entries per face.
    auto push = [&](uint32_t a, uint32_t b, uint32_t c) {
        if (a >= mesh.positions.size() || b >= mesh.positions.size() || c >= mesh.positions.size())
            return;
        if (a == b || b == c || a == c) return;
        mesh.faces.push_back(Face{{a, b, c}});
        const uint32_t tri[3] = {a, b, c};
        for (uint32_t v : tri) {
            CornerAttr ca;
            if (has_nrm) {
                const size_t o = static_cast<size_t>(v) * nrm.components;
                if (o + 2 < nrm.v.size()) {
                    ca.n = Vec3{static_cast<float>(nrm.v[o]), static_cast<float>(nrm.v[o + 1]),
                                static_cast<float>(nrm.v[o + 2])};
                }
            }
            if (has_uv) {
                const size_t o = static_cast<size_t>(v) * uv.components;
                if (o + 1 < uv.v.size()) {
                    ca.uv = Vec2{static_cast<float>(uv.v[o]), static_cast<float>(uv.v[o + 1])};
                    ca.has_uv = true;
                    mesh.has_uv = true;
                }
            }
            mesh.corner_attr.push_back(ca);
        }
    };
    if (mode == kModeTriangles) {
        for (size_t i = 0; i + 2 < idx.size(); i += 3) push(idx[i], idx[i + 1], idx[i + 2]);
    } else if (mode == 5) {
        for (size_t i = 0; i + 2 < idx.size(); ++i) {
            if (i % 2 == 0) push(idx[i], idx[i + 1], idx[i + 2]);
            else push(idx[i + 1], idx[i], idx[i + 2]);
        }
    } else {
        for (size_t i = 1; i + 1 < idx.size(); ++i) push(idx[0], idx[i], idx[i + 1]);
    }
    if (mesh.faces.empty()) return false;

    if (attrs.contains("JOINTS_0") && attrs["JOINTS_0"].is_number_integer()) {
        Accessor ja0 = read_accessor(d, attrs["JOINTS_0"].get<int>());
        Accessor wa0;
        bool has_w = attrs.contains("WEIGHTS_0") && attrs["WEIGHTS_0"].is_number_integer();
        if (has_w) wa0 = read_accessor(d, attrs["WEIGHTS_0"].get<int>());
        if (ja0.ok && has_w && wa0.ok && ja0.count == pos.count && wa0.count == pos.count) {
            const auto& jraw = d.g["accessors"][static_cast<size_t>(attrs["JOINTS_0"].get<int>())];
            bool jnorm = jbool(jraw, "normalized", false);
            const auto& wraw = d.g["accessors"][static_cast<size_t>(attrs["WEIGHTS_0"].get<int>())];
            bool wnorm = jbool(wraw, "normalized", false);
            mesh.skin.assign(pos.count, SkinVert{});
            for (size_t i = 0; i < pos.count; ++i) {
                SkinVert sv;
                for (int k = 0; k < 4; ++k) {
                    uint32_t raw = static_cast<uint32_t>(acc_f(ja0, i, k, jnorm));
                    sv.j[k] = static_cast<uint16_t>(std::min<uint32_t>(raw, 65535));
                    sv.w[k] = acc_f(wa0, i, k, wnorm);
                }
                float sum = sv.w[0] + sv.w[1] + sv.w[2] + sv.w[3];
                if (sum > 1e-6f)
                    for (int k = 0; k < 4; ++k) sv.w[k] /= sum;
                mesh.skin[i] = sv;
            }
        } else {
            rep.dropped.insert("SKIN_WEIGHTS");
        }
    }

    material = jp.value("material", -1);
    mesh_sync_render(mesh);
    return true;
}

}  // namespace

namespace {
/// Index of a clip by name, or -1. Used so a re-import does not duplicate.
int s_find_clip(const Scene& s, const std::string& n) {
    for (size_t i = 0; i < s.clips.size(); ++i) {
        if (s.clips[i].name == n) return static_cast<int>(i);
    }
    return -1;
}
}  // namespace

GltfImportStats import_gltf(Scene& scene, const std::string& path, std::string& err) {
    GltfImportStats stats;
    Doc d;
    if (!load_doc(path, d, err)) return stats;
    if (!d.g.contains("asset")) {
        err = "gltf has no 'asset' block";
        return stats;
    }
    std::string ver = jstr(d.g["asset"], "version", "");
    if (ver.rfind("2.", 0) != 0) {
        err = "gltf asset.version is '" + ver + "', expected 2.x";
        return stats;
    }
    Report rep;
    collect_exts(d.g, rep);

    std::vector<int> mat_index;
    if (d.g.contains("materials") && d.g["materials"].is_array()) {
        for (const auto& jm : d.g["materials"]) {
            Material m;
            m.name = jstr(jm, "name", "Material");
            if (jm.contains("pbrMetallicRoughness")) {
                const auto& p = jm["pbrMetallicRoughness"];
                const auto& bc = jget(p, "baseColorFactor");
                if (bc.is_array() && bc.size() >= 3)
                    m.albedo = {bc[0].get<float>(), bc[1].get<float>(), bc[2].get<float>()};
                m.metallic = p.value("metallicFactor", 1.0f);
                m.roughness = p.value("roughnessFactor", 1.0f);
            }
            const auto& ef = jget(jm, "emissiveFactor");
            if (ef.is_array() && ef.size() >= 3) {
                float e0 = ef[0].get<float>(), e1 = ef[1].get<float>(), e2 = ef[2].get<float>();
                float emax = std::max(std::max(e0, e1), e2);
                if (emax > 0.0f) {
                    float amax = std::max(std::max(m.albedo.x, m.albedo.y), m.albedo.z);
                    m.emission = amax > 1e-6f ? emax / amax : 1.0f;
                }
            }
            mat_index.push_back(static_cast<int>(scene.materials.size()));
            scene.materials.push_back(m);
        }
    }
    stats.materials = static_cast<int>(mat_index.size());

    if (!d.g.contains("nodes") || !d.g["nodes"].is_array()) {
        err = "gltf has no 'nodes' array";
        return stats;
    }
    const auto& jnodes = d.g["nodes"];
    std::vector<int> node_slot(jnodes.size(), -1);
    std::vector<std::vector<int>> node_meshes(jnodes.size());
    std::vector<int> node_skin(jnodes.size(), -1);

    std::function<void(size_t, int)> walk = [&](size_t gi, int parent_slot) {
        if (gi >= jnodes.size()) return;
        const auto& jn = jnodes[gi];
        Node node;
        std::string raw_name = jstr(jn, "name", "");
        node.name = raw_name.empty() ? ("Node" + std::to_string(gi)) : raw_name;
        node.parent = parent_slot;
        node.type = NodeType::Empty;

        Mat4 local = Mat4::identity();
        if (jget(jn, "matrix").is_array() && jn["matrix"].size() == 16) {
            for (int i = 0; i < 16; ++i) local.m[i] = jn["matrix"][static_cast<size_t>(i)].get<float>();
        } else {
            Vec3 t{0, 0, 0}, s{1, 1, 1};
            Vec4 q{0, 0, 0, 1};
            if (jget(jn, "translation").is_array() && jn["translation"].size() >= 3)
                t = {jn["translation"][0].get<float>(), jn["translation"][1].get<float>(),
                     jn["translation"][2].get<float>()};
            if (jget(jn, "scale").is_array() && jn["scale"].size() >= 3)
                s = {jn["scale"][0].get<float>(), jn["scale"][1].get<float>(),
                     jn["scale"][2].get<float>()};
            if (jget(jn, "rotation").is_array() && jn["rotation"].size() >= 4)
                q = {jn["rotation"][0].get<float>(), jn["rotation"][1].get<float>(),
                     jn["rotation"][2].get<float>(), jn["rotation"][3].get<float>()};
            local = mul(translate(t), mul(quat_to_mat(q.x, q.y, q.z, q.w), scale(s)));
        }
        Trs trs = decompose(local);
        node.translation = trs.t;
        node.rotation_deg = trs.r;
        node.scale = trs.s;

        int prim_counter = 0;
        struct Built {
            Mesh mesh;
            int material = -1;
        };
        std::vector<Built> built;

        if (jget(jn, "mesh").is_number_integer()) {
            int mi = jn["mesh"].get<int>();
            if (d.g["meshes"].is_array() && mi >= 0 && mi < static_cast<int>(d.g["meshes"].size())) {
                const auto& jm = d.g["meshes"][static_cast<size_t>(mi)];
                std::string mesh_name = jstr(jm, "name", "");
                if (mesh_name.empty()) mesh_name = node.name;
                const auto& prims = jget(jm, "primitives");
                for (const auto& jp : prims) {
                    Built b;
                    b.mesh.name = mesh_name + (prims.size() > 1 ? "_" + std::to_string(prim_counter)
                                                                : std::string());
                    bool smooth = false;
                    if (!read_primitive(d, jp, b.mesh, smooth, b.material, rep, err)) {
                        if (!err.empty()) return;
                        continue;
                    }
                    built.push_back(std::move(b));
                    prim_counter++;
                }
            }
        }

        if (built.size() == 1) {
            node.type = NodeType::Mesh;
            node.mesh = static_cast<int>(scene.meshes.size());
            if (built[0].material >= 0 && built[0].material < static_cast<int>(mat_index.size()))
                node.material = mat_index[static_cast<size_t>(built[0].material)];
            scene.meshes.push_back(std::move(built[0].mesh));
            rep.faces += static_cast<int>(scene.meshes.back().faces.size());
            rep.vertices += static_cast<int>(scene.meshes.back().positions.size());
            node_meshes[gi].push_back(static_cast<int>(scene.meshes.size()) - 1);
        } else if (built.size() > 1) {
            node.type = NodeType::Empty;
        }

        if (jget(jn, "camera").is_number_integer()) {
            int ci = jn["camera"].get<int>();
            if (d.g["cameras"].is_array() && ci >= 0 && ci < static_cast<int>(d.g["cameras"].size())) {
                const auto& jc = d.g["cameras"][static_cast<size_t>(ci)];
                if (jstr(jc, "type", "") == "perspective" &&
                    jget(jc, "perspective").is_object()) {
                    node.type = NodeType::Camera;
                    node.fov_deg = rad2deg(jc["perspective"].value("yfov", deg2rad(50.0f)));
                    rep.cameras++;
                } else {
                    rep.dropped.insert("CAMERA_ORTHOGRAPHIC");
                }
            }
        }

        if (jget(jn, "skin").is_number_integer()) node_skin[gi] = jn["skin"].get<int>();
        node.name = scene.unique_name(node.name);
        int slot = scene.add_node(node);
        node_slot[gi] = slot;

        if (built.size() > 1) {
            for (auto& b : built) {
                Node pn;
                pn.name = scene.unique_name(b.mesh.name);
                pn.type = NodeType::Mesh;
                pn.parent = slot;
                pn.mesh = static_cast<int>(scene.meshes.size());
                if (b.material >= 0 && b.material < static_cast<int>(mat_index.size()))
                    pn.material = mat_index[static_cast<size_t>(b.material)];
                rep.faces += static_cast<int>(b.mesh.faces.size());
                rep.vertices += static_cast<int>(b.mesh.positions.size());
                scene.meshes.push_back(std::move(b.mesh));
                int child = scene.add_node(pn);
                node_meshes[gi].push_back(child);
            }
        }

        if (jget(jn, "children").is_array()) {
            for (const auto& c : jn["children"])
                if (c.is_number_integer()) walk(static_cast<size_t>(c.get<int>()), node_slot[gi]);
        }
    };

    std::vector<size_t> roots;
    if (d.g["scenes"].is_array() && !d.g["scenes"].empty()) {
        int si = jint(d.g, "scene", 0);
        if (si < 0 || si >= static_cast<int>(d.g["scenes"].size())) si = 0;
        const auto& sc = d.g["scenes"][static_cast<size_t>(si)];
        if (sc["nodes"].is_array())
            for (const auto& c : sc["nodes"])
                if (c.is_number_integer()) roots.push_back(static_cast<size_t>(c.get<int>()));
    }
    if (roots.empty())
        for (size_t i = 0; i < jnodes.size(); ++i) roots.push_back(i);
    for (size_t r : roots) walk(r, -1);

    if (d.g["skins"].is_array()) {
        for (size_t gi = 0; gi < node_skin.size(); ++gi) {
            int si = node_skin[gi];
            if (si < 0 || si >= static_cast<int>(d.g["skins"].size())) continue;
            const auto& js = d.g["skins"][static_cast<size_t>(si)];
            if (!jget(js, "joints").is_array() || js["joints"].empty()) continue;
            std::vector<int> joint_slots;
            for (const auto& jv : js["joints"]) {
                if (!jv.is_number_integer()) continue;
                size_t jg = static_cast<size_t>(jv.get<int>());
                if (jg >= node_slot.size() || node_slot[jg] < 0) continue;
                int js_slot = node_slot[jg];
                scene.nodes[static_cast<size_t>(js_slot)].type = NodeType::Bone;
                if (scene.nodes[static_cast<size_t>(js_slot)].bone_length <= 0.0f)
                    scene.nodes[static_cast<size_t>(js_slot)].bone_length = 0.2f;
                joint_slots.push_back(js_slot);
                rep.bones++;
            }
            if (joint_slots.empty()) continue;
            rep.skins++;

            std::vector<Mat4> ibm(joint_slots.size(), Mat4::identity());
            if (jget(js, "inverseBindMatrices").is_number_integer()) {
                Accessor ia = read_accessor(d, js["inverseBindMatrices"].get<int>());
                for (size_t i = 0; i < ibm.size() && ia.ok; ++i) {
                    Mat4 m = Mat4::identity();
                    for (int c = 0; c < 4; ++c)
                        for (int r = 0; r < 4; ++r) m.at(r, c) = acc_f(ia, i * 16 + static_cast<size_t>(c * 4 + r), 0, false);
                    ibm[i] = m;
                }
            } else {
                for (size_t i = 0; i < ibm.size(); ++i)
                    ibm[i] = invert_affine(scene.rest_matrix(joint_slots[i]));
            }
            std::map<uint32_t, uint16_t> gltf_to_local;
            for (size_t i = 0; i < joint_slots.size(); ++i) {
                const auto& jv = js["joints"][i];
                if (jv.is_number_integer())
                    gltf_to_local[static_cast<uint32_t>(jv.get<int>())] = static_cast<uint16_t>(i);
            }
            for (int mesh_slot : node_meshes[gi]) {
                if (mesh_slot < 0) continue;
                Node& mn = scene.nodes[static_cast<size_t>(mesh_slot)];
                if (mn.mesh < 0 || mn.mesh >= static_cast<int>(scene.meshes.size())) continue;
                Mesh& mesh = scene.meshes[static_cast<size_t>(mn.mesh)];
                if (mesh.skin.empty()) continue;
                mesh.joints = joint_slots;
                mesh.inv_bind = ibm;
                uint32_t remapped = 0;
                for (size_t vi = 0; vi < mesh.skin.size(); ++vi) {
                    SkinVert& sv = mesh.skin[vi];
                    uint16_t jidx[4] = {0, 0, 0, 0};
                    for (int k = 0; k < 4; ++k) {
                        auto it = gltf_to_local.find(sv.j[k]);
                        jidx[k] = it == gltf_to_local.end() ? 0 : it->second;
                        if (it != gltf_to_local.end()) remapped++;
                    }
                    for (int k = 0; k < 4; ++k) sv.j[k] = jidx[k];
                }
                rep.skinned_vertices += static_cast<int>(remapped);
            }
        }
    }

    // -- animations ---------------------------------------------------------
    //
    // Read back into clips so a file written by this exporter round-trips. Times
    // are seconds and are converted to frames using the rate, which is taken from
    // the glTF asset extras when present and otherwise assumed 24fps. CUBICSPLINE
    // tangents arrive pre-multiplied by their time delta, so they are divided back
    // out to recover the value-per-frame slopes the rest of the kernel uses.
    {
        float fps = 24.0f;
        if (d.g.contains("asset") && d.g["asset"].contains("extras")) {
            const auto& ex = d.g["asset"]["extras"];
            if (ex.contains("kiln_fps")) fps = ex.value("kiln_fps", 24.0f);
        }
        if (!(fps > 0.0f)) fps = 24.0f;

        // node_slot, built by the node walk above, maps a glTF node index to the
        // scene node it became, so a channel target resolves to a real node.
        const std::vector<int>& node_map = node_slot;

        if (d.g.contains("animations") && d.g["animations"].is_array()) {
            for (const auto& ja : d.g["animations"]) {
                Clip clip;
                clip.name = jstr(ja, "name", "Clip");
                // A per-animation rate wins over the file-level default.
                if (ja.contains("extras") && ja["extras"].contains("kiln_fps")) {
                    const float r = ja["extras"].value("kiln_fps", fps);
                    if (r > 0.0f) fps = r;
                }
                clip.fps = fps;
                bool usable = false;
                if (ja.contains("channels") && ja["channels"].is_array() &&
                    ja.contains("samplers") && ja["samplers"].is_array()) {
                    for (const auto& jch : ja["channels"]) {
                        if (!jch.contains("target")) continue;
                        const auto& jt = jch["target"];
                        const int gnode = jint(jt, "node", -1);
                        if (gnode < 0 || gnode >= static_cast<int>(node_map.size())) continue;
                        const int scene_node = node_map[static_cast<size_t>(gnode)];
                        if (scene_node < 0 ||
                            scene_node >= static_cast<int>(scene.nodes.size())) {
                            continue;
                        }
                        const int si = jint(jch, "sampler", -1);
                        if (si < 0 || si >= static_cast<int>(ja["samplers"].size())) continue;
                        const auto& js = ja["samplers"][static_cast<size_t>(si)];
                        const int in_acc = jint(js, "input", -1);
                        const int out_acc = jint(js, "output", -1);
                        if (in_acc < 0 || out_acc < 0) continue;
                        const Accessor ia = read_accessor(d, in_acc);
                        const Accessor oa = read_accessor(d, out_acc);
                        if (!ia.ok || !oa.ok || ia.count == 0) continue;
                        const std::string interp = jstr(js, "interpolation", "LINEAR");
                        const bool cubic = interp == "CUBICSPLINE";
                        const size_t stride = cubic ? 3 : 1;

                        const std::string path = jstr(jt, "path", "translation");
                        Channel ch = Channel::Location;
                        if (path == "rotation") ch = Channel::Rotation;
                        else if (path == "scale") ch = Channel::Scale;
                        else if (path != "translation") continue;  // weights/morph ignored

                        Track track;
                        track.node = scene.nodes[static_cast<size_t>(scene_node)].name;
                        track.channel = ch;
                        for (size_t k = 0; k < ia.count; ++k) {
                            const float secs = acc_f(ia, k, 0, false);
                            Keyframe kf;
                            kf.frame = static_cast<int>(std::lround(secs * fps));
                            if (k < oa.count / stride) {
                                // CUBICSPLINE stores in/value/out per key, so the
                                // value is the middle of the triplet; LINEAR and STEP
                                // store one value per key. Reading k*stride+1 for a
                                // non-cubic sampler walked a key ahead and read
                                // nothing at all past the end.
                                const size_t o = cubic ? k * 3 + 1 : k;
                                if (ch == Channel::Rotation) {
                                    const float qx = acc_f(oa, o, 0, false);
                                    const float qy = acc_f(oa, o, 1, false);
                                    const float qz = acc_f(oa, o, 2, false);
                                    const float qw = acc_f(oa, o, 3, false);
                                    kf.value = mat_to_euler_xyz(
                                        quat_to_mat(qx, qy, qz, qw));
                                } else {
                                    kf.value = {acc_f(oa, o, 0, false), acc_f(oa, o, 1, false),
                                                acc_f(oa, o, 2, false)};
                                }
                            }
                            kf.interp = cubic ? Interp::Bezier
                                              : (interp == "STEP" ? Interp::Step
                                                                   : Interp::Linear);
                            // Convert the pre-multiplied glTF tangents back to
                            // value-per-frame so evaluation matches the exporter.
                            if (cubic && k + 1 < ia.count) {
                                const float dt = acc_f(ia, k + 1, 0, false) - secs;
                                if (dt > 1e-6f) {
                                    const size_t tin = k * stride;
                                    const size_t tout = k * stride + 2;
                                    auto read_tan = [&](size_t idx) {
                                        if (ch == Channel::Rotation) {
                                            const float qx = acc_f(oa, idx, 0, false);
                                            const float qy = acc_f(oa, idx, 1, false);
                                            const float qz = acc_f(oa, idx, 2, false);
                                            const float qw = acc_f(oa, idx, 3, false);
                                            return mat_to_euler_xyz(
                                                quat_to_mat(qx, qy, qz, qw)) /
                                                   dt;
                                        }
                                        return Vec3{acc_f(oa, idx, 0, false),
                                                    acc_f(oa, idx, 1, false),
                                                    acc_f(oa, idx, 2, false)} /
                                               dt;
                                    };
                                    kf.in_tan = read_tan(tin);
                                    kf.out_tan = read_tan(tout);
                                }
                            }
                            track.keys.push_back(kf);
                        }
                        if (track.keys.size() < 2) continue;
                        clip.tracks.push_back(std::move(track));
                        usable = true;
                    }
                }
                if (!usable) continue;
                clip.sort_keys();
                if (s_find_clip(scene, clip.name) < 0) {
                    scene.clips.push_back(std::move(clip));
                }
            }
            if (!scene.clips.empty() && scene.active_clip < 0) {
                scene.active_clip = static_cast<int>(scene.clips.size()) - 1;
            }
        }
    }


    // -- procedural graphs -------------------------------------------------
    //
    // Read the recipes back so an imported file is re-parameterisable rather than
    // just a mesh. Graphs already in the scene win: a re-import must not silently
    // replace a graph the caller has been editing.
    if (d.g.contains("extensions") && d.g["extensions"].is_array()) {
        for (const auto& jg : d.g["extensions"]) {
            if (!jg.contains("KHR_kiln_procedural")) continue;
            const auto& ext = jg["KHR_kiln_procedural"];
            if (!ext.contains("graphs")) continue;
            std::vector<NodeGraph> incoming;
            graphs_from_json(ext["graphs"], incoming);
            for (const NodeGraph& g : incoming) {
                if (scene.find_graph(g.name) < 0) scene.graphs.push_back(g);
            }
        }
    }
    if (d.g.contains("extensions") && d.g["extensions"].is_object()) {
        const auto& je = d.g["extensions"];
        if (je.contains("KHR_kiln_procedural") &&
            je["KHR_kiln_procedural"].contains("graphs")) {
            std::vector<NodeGraph> incoming;
            graphs_from_json(je["KHR_kiln_procedural"]["graphs"], incoming);
            for (const NodeGraph& g : incoming) {
                if (scene.find_graph(g.name) < 0) scene.graphs.push_back(g);
            }
        }
    }

    stats.nodes = static_cast<int>(scene.nodes.size());
    stats.meshes = static_cast<int>(scene.meshes.size());
    stats.primitives = stats.meshes;
    stats.faces = rep.faces;
    stats.vertices = rep.vertices;
    stats.cameras = rep.cameras;
    stats.skins = rep.skins;
    stats.bones = rep.bones;
    stats.skinned_vertices = rep.skinned_vertices;
    stats.skipped_modes = rep.skipped_modes;
    for (const auto& a : rep.dropped) stats.dropped_attributes.push_back(a);
    for (const auto& e : rep.exts) stats.unsupported_extensions.push_back(e);
    return stats;
}

}  // namespace kiln
