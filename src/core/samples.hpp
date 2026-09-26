#pragma once

#include <string>

namespace kiln {

// Supersampling factor for the software preview path. The renderer draws at
// N x N times the target resolution and box-filters back down to it, so the
// factor is the quality/speed dial: 1 disables AA, 2 is the historical 2x2
// default, higher numbers keep smoothing silhouettes and the ground grid.
//
// The accepted range lives here and only here, so the CLI flag, the REPL
// command and the renderer cannot drift apart.
constexpr int kSamplesMin = 1;
constexpr int kSamplesMax = 16;
constexpr int kDefaultSamples = 2;

// The accepted spellings of the flag, so the CLI parser, the usage hint and the
// docs cannot drift apart. --ss is the short alias: same values, same parser,
// same error text. Acronym flags keep their double dash -- a single-dash -ss
// would collide with short-flag clustering, so it is deliberately not offered.
constexpr const char* kSamplesFlag = "--samples";
constexpr const char* kSamplesFlagAlias = "--ss";

// True when v is inside the accepted range.
inline bool samples_in_range(int v) { return v >= kSamplesMin && v <= kSamplesMax; }

// Parse one --samples / "samples" argument. Returns true and writes `out` on
// success; on failure returns false and fills `err` with a reason phrased for
// the user. Accepts a whole decimal number only, so "2.5", "2x", "1e3" and
// "two" are all rejected rather than silently truncated.
inline bool parse_samples(const std::string& text, int& out, std::string& err) {
    if (text.empty()) {
        err = "missing value";
        return false;
    }

    std::size_t i = 0;
    bool negative = false;
    if (text[0] == '+' || text[0] == '-') {
        negative = text[0] == '-';
        i = 1;
    }
    if (i >= text.size()) {
        err = "\"" + text + "\" is not a number";
        return false;
    }

    long long v = 0;
    for (; i < text.size(); ++i) {
        char ch = text[i];
        if (ch < '0' || ch > '9') {
            err = "\"" + text + "\" is not a whole number";
            return false;
        }
        // Saturate rather than overflow: every value past the maximum is
        // rejected identically, so the exact magnitude does not matter.
        if (v <= 1000000000LL) v = v * 10 + (ch - '0');
    }
    if (negative) v = -v;

    if (v == 0) {
        err = "sample count must be at least " + std::to_string(kSamplesMin) + " (0 disables nothing)";
        return false;
    }
    if (v < 0) {
        err = "sample count must be positive, not " + text;
        return false;
    }
    if (v > static_cast<long long>(kSamplesMax)) {
        err = "\"" + text + "\" is above the maximum of " + std::to_string(kSamplesMax);
        return false;
    }

    out = static_cast<int>(v);
    return true;
}

// One-line hint appended to every rejection so the range is always visible.
inline std::string samples_usage() {
    return "--samples must be a whole number in [" + std::to_string(kSamplesMin) + ", " +
           std::to_string(kSamplesMax) + "] (" + std::to_string(kDefaultSamples) +
           " = 2x2 grid, the default; 1 = off)";
}

// Recognise the --flag=VALUE spelling for every accepted spelling of the flag,
// and hand back just the value. Kept in a separate branch by the CLI so the
// spaced spelling above keeps consuming the next argv entry while this one never
// looks past the current argument.
inline bool samples_inline_value(const std::string& arg, std::string& out) {
    static const char* const kFlags[] = {kSamplesFlag, kSamplesFlagAlias, "--aa"};
    for (const char* f : kFlags) {
        std::string prefix = std::string(f) + "=";
        if (arg.size() >= prefix.size() && arg.compare(0, prefix.size(), prefix) == 0) {
            out = arg.substr(prefix.size());
            return true;
        }
    }
    return false;
}

}  // namespace kiln
