#pragma once

#include "int_parse.hpp"

#include <string>

namespace kiln {

// Contact shadows for the software preview path. The shading pass asks, for the
// surface each pixel shows, how far it is from the first thing that blocks the
// sun. An object touching the ground blocks immediately, so the seam under it
// goes dark; an object floating above the ground blocks only after a gap, so its
// shadow is lighter and detaches -- which is what stops a floating mesh from
// reading as pasted on.
//
// The dial is INTENSITY, matching how --samples reads as one whole number: 0
// turns the pass off, 1 is a hint, 4 is a deep, obvious grounding. The sample
// cost is fixed (kContactShadowSteps marches per pixel) and does not grow with
// the dial, so the cost is the same at 1 and at 4 -- only the amount changes.
//
// The accepted range lives here and only here, so the CLI flag, the REPL command
// and the renderer cannot drift apart.
constexpr int kContactShadowMin = 0;
constexpr int kContactShadowMax = 4;
constexpr int kDefaultContactShadow = 0;  // off, so existing renders are unchanged

// Tuning. These are deliberately not user-facing: they are the quality/cost of
// the pass, not the look, so exposing them would multiply the surface area of
// the feature for no gain. They are still named here so the renderer and the
// documented performance numbers quote the same numbers.
constexpr int kContactShadowSteps = 16;   // ray-march steps from surface to sun
constexpr float kContactShadowReach = 2.5f;  // world units the ray is marched
constexpr float kContactShadowBias = 0.006f; // depth bias, kills surface acne
constexpr int kContactShadowSoften = 1;   // 1 = 3-tap separable blur on the result

// True when v is inside the accepted range.
inline bool contact_shadow_in_range(int v) {
    return v >= kContactShadowMin && v <= kContactShadowMax;
}

// Parse one --contact-shadow / "shadow" argument. Returns true and writes `out`
// on success; on failure returns false and fills `err` with a reason phrased for
// the user. Accepts a whole decimal number only, so "2.5", "2x", "1e3" and "two"
// are all rejected rather than silently truncated. 0 is accepted: unlike
// --samples, 0 here is a meaningful "off".
inline bool parse_contact_shadow(const std::string& text, int& out, std::string& err) {
    long long v = 0;
    if (!scan_int(text, v, err)) return false;
    long long r = 0;
    if (!check_int_range(v, kContactShadowMin, kContactShadowMax, text, std::string(), r, err)) return false;
    out = static_cast<int>(r);
    return true;
}

// One-line hint appended to every rejection so the range is always visible.
inline std::string contact_shadow_usage() {
    return "--contact-shadow must be a whole number in [" + std::to_string(kContactShadowMin) + ", " +
           std::to_string(kContactShadowMax) + "] (" + std::to_string(kDefaultContactShadow) +
           " = off, the default; 1 = subtle, 4 = strong)";
}

}  // namespace kiln
