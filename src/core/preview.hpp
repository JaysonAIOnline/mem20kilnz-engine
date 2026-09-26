#pragma once

#include <string>

namespace kiln {

// --- preview images baked into an export -------------------------------------
//
// `export` can bake one preview PNG into the GLB it writes, so a file that
// leaves kiln carries its own thumbnail. Three decisions live here and only
// here, so the CLI, the REPL and the exporter cannot drift apart:
//
//   * embedding is OFF by default. An export that does not ask for a preview
//     stays byte-identical to what kiln wrote before the option existed, which
//     is the whole reason this is opt-in rather than opt-out.
//   * the preview resolution, and the range it may be set to.
//   * the size cap that keeps a GLB a model file rather than a screenshot.
//
// The preview is drawn by the same software renderer as any other preview and
// honours the same supersampling factor (core/samples.hpp), so `--samples 8`
// sharpens the embedded thumbnail and the on-disk PNG together.

// Opt-in. `false` means "write exactly the GLB this build wrote before".
constexpr bool kGlbPreviewDefault = false;

// 16:9 and small enough that the encoded PNG lands in the tens of kilobytes:
// a thumbnail, not a still. Override per export with --preview-size, or per
// REPL session with `glbpreview WxH`.
constexpr int kPreviewDefaultW = 512;
constexpr int kPreviewDefaultH = 288;

// Accepted per-axis range. The floor keeps a preview from being an unreadable
// smudge; the ceiling keeps one export from becoming a multi-second render and
// a megabyte of PNG.
constexpr int kPreviewMinDim = 16;
constexpr int kPreviewMaxDim = 2048;

// Cap on the encoded PNG payload. Flat-shaded geometry at the maximum
// resolution compresses far below this, so hitting the cap means the size was
// chosen wrong for a thumbnail -- which is what the rejection says out loud.
constexpr size_t kGlbPreviewMaxBytes = 4u * 1024u * 1024u;  // 4 MiB

// Ceiling on what the preview render may cost, in supersampled pixels
// (width * height * samples * samples). This is the bound that actually keeps a
// baked preview a thumbnail: at the maximum resolution and the maximum sample
// count the renderer would need a one-gigabyte buffer, and an export that dies
// mid-render is a worse failure than one that refuses up front. 32M admits
// 1920x1080 at 4x4 and 2048x2048 at 2x2, and refuses the corner where both
// dials are maxed at once.
constexpr long long kPreviewMaxSupersampledPixels = 32LL * 1024 * 1024;

inline bool preview_dim_in_range(int v) { return v >= kPreviewMinDim && v <= kPreviewMaxDim; }
inline bool preview_size_in_range(int w, int h) { return preview_dim_in_range(w) && preview_dim_in_range(h); }

inline long long preview_supersampled_pixels(int w, int h, int samples) {
    return static_cast<long long>(w) * h * samples * samples;
}

inline bool preview_cost_ok(int w, int h, int samples) {
    return preview_supersampled_pixels(w, h, samples) <= kPreviewMaxSupersampledPixels;
}

inline std::string preview_cost_message(int w, int h, int samples) {
    return "a " + std::to_string(w) + "x" + std::to_string(h) + " preview at " + std::to_string(samples) +
           "x" + std::to_string(samples) + " samples needs " +
           std::to_string(preview_supersampled_pixels(w, h, samples)) +
           " supersampled pixels, over the cap of " +
           std::to_string(kPreviewMaxSupersampledPixels) + " -- lower --samples or --preview-size";
}

// One whole decimal number, saturating past the point where the value can no
// longer be in range. Same contract as parse_samples: no sign, no fraction, no
// exponent, no trailing unit.
inline bool parse_preview_dim(const std::string& text, long long& out) {
    if (text.empty()) return false;
    long long v = 0;
    for (std::size_t i = 0; i < text.size(); ++i) {
        char ch = text[i];
        if (ch < '0' || ch > '9') return false;
        if (v <= 1000000000LL) v = v * 10 + (ch - '0');
    }
    out = v;
    return true;
}

// Parse a preview resolution written "WxH" (also "WXH" and "W*H"). Returns true
// and writes both sides on success; on failure returns false and fills `err`
// with a reason phrased for the user.
inline bool parse_preview_size(const std::string& text, int& w, int& h, std::string& err) {
    if (text.empty()) {
        err = "missing size";
        return false;
    }
    std::size_t sep = text.find_first_of("xX*");
    if (sep == std::string::npos) {
        err = "\"" + text + "\" is not WxH (for example " + std::to_string(kPreviewDefaultW) + "x" +
              std::to_string(kPreviewDefaultH) + ")";
        return false;
    }
    long long lw = 0, lh = 0;
    if (!parse_preview_dim(text.substr(0, sep), lw) || !parse_preview_dim(text.substr(sep + 1), lh)) {
        err = "\"" + text + "\" is not two whole numbers separated by x";
        return false;
    }
    // Compared as long long so no narrowing cast can turn an absurd value into
    // an in-range one before the check.
    if (lw < kPreviewMinDim || lw > kPreviewMaxDim || lh < kPreviewMinDim || lh > kPreviewMaxDim) {
        err = "\"" + text + "\" is outside the accepted range (" + std::to_string(kPreviewMinDim) + " to " +
              std::to_string(kPreviewMaxDim) + " on each side)";
        return false;
    }
    w = static_cast<int>(lw);
    h = static_cast<int>(lh);
    return true;
}

// One-line hint describing the option, appended to every rejection and shown in
// `help` so both halves of the dial are always visible together.
inline std::string preview_usage() {
    return "--glb-preview bakes a preview PNG into the exported GLB (off by default); "
           "--preview-size WxH sets its resolution (" + std::to_string(kPreviewDefaultW) + "x" +
           std::to_string(kPreviewDefaultH) + " by default, each side " + std::to_string(kPreviewMinDim) +
           " to " + std::to_string(kPreviewMaxDim) + "); --samples N sets its anti-aliasing";
}

}  // namespace kiln
