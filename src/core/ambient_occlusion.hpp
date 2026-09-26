#pragma once

#include "int_parse.hpp"

#include <string>

namespace kiln {

// Ambient occlusion for the software preview path. The shading pass probes a
// short hemisphere around the surface each pixel shows and darkens the pixel by
// how much of that hemisphere is blocked. Creases, cavities and the seams where
// two surfaces meet therefore go darker than flat areas, so the form reads in
// depth even where the sun is not raking across it.
//
// The dial is INTENSITY, exactly like --contact-shadow: 0 turns the pass off, 4
// is a deep, obvious cavity darkening. The probe count is fixed
// (kAoDirections rays x kAoSteps steps) and does not grow with the dial, so the
// cost is the same at 1 and at 4.
//
// The accepted range lives here and only here, so the CLI flag, the REPL command
// and the renderer cannot drift apart. The pass reads the same level-of-detail
// geometry the colour pass does, so AO on a decimated mesh follows the LOD.
constexpr int kAoMin = 0;
constexpr int kAoMax = 4;
constexpr int kDefaultAo = 0;  // off, so existing renders are unchanged

// Tuning. Not user-facing, for the same reason as the contact-shadow ones: they
// are the cost of the pass rather than the look.
constexpr int kAoDirections = 6;        // hemisphere probe directions per pixel
constexpr int kAoSteps = 6;             // steps marched along each probe
constexpr float kAoReach = 0.6f;        // world units a probe is marched
constexpr float kAoBias = 0.005f;       // depth bias, kills self-occlusion

// True when v is inside the accepted range.
inline bool ao_in_range(int v) { return v >= kAoMin && v <= kAoMax; }

// Parse one --ao / "ao" argument. Returns true and writes `out` on success; on
// failure returns false and fills `err` with a reason phrased for the user.
// Accepts a whole decimal number only. 0 is accepted: it is a meaningful "off".
inline bool parse_ao(const std::string& text, int& out, std::string& err) {
    long long v = 0;
    if (!scan_int(text, v, err)) return false;
    long long r = 0;
    if (!check_int_range(v, kAoMin, kAoMax, text, std::string(), r, err)) return false;
    out = static_cast<int>(r);
    return true;
}

// One-line hint appended to every rejection so the range is always visible.
inline std::string ao_usage() {
    return "--ao must be a whole number in [" + std::to_string(kAoMin) + ", " + std::to_string(kAoMax) +
           "] (" + std::to_string(kDefaultAo) + " = off, the default; 1 = subtle, 4 = strong)";
}

}  // namespace kiln
