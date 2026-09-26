#pragma once

// Shared whole-number scanner for kiln's numeric command-line / REPL options.
//
// Every dial that takes a whole number (--samples, --contact-shadow, --ao, and
// any future one) needs the same three behaviours, and getting them subtly
// different per option is how two surfaces drift apart:
//
//   * a strict decimal scan, so "2.5", "2x", "1e3", "two" and "" are rejected
//     rather than silently truncated to a prefix;
//   * an explicit sign, so a negative is diagnosed as a negative rather than
//     as a malformed number;
//   * saturation instead of overflow, so an absurd run of digits is rejected
//     identically to any other out-of-range value instead of wrapping back
//     into range.
//
// The range itself is NOT here: it belongs to the header that owns the dial
// (core/samples.hpp, core/contact_shadow.hpp, core/ambient_occlusion.hpp), so
// each option keeps one authority for its own bounds.

#include <string>

namespace kiln {

// One whole-number scan. Returns true and writes `out` on success; on failure
// returns false and fills `err` with a reason phrased for the user. `err` is
// set only on failure.
inline bool scan_int(const std::string& text, long long& out, std::string& err) {
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

    out = v;
    return true;
}

// Reject a value below `lo` or above `hi` with a message naming the range.
// `reject_zero_hint` is appended when the value is zero, so a dial that would
// silently disable the feature can say so instead of accepting it.
inline bool check_int_range(long long v, long long lo, long long hi, const std::string& text,
                            const std::string& reject_zero_hint, long long& out, std::string& err) {
    if (v == 0 && !reject_zero_hint.empty()) {
        err = reject_zero_hint;
        return false;
    }
    if (v < lo) {
        err = "\"" + text + "\" is below the minimum of " + std::to_string(lo);
        return false;
    }
    if (v > hi) {
        err = "\"" + text + "\" is above the maximum of " + std::to_string(hi);
        return false;
    }
    out = v;
    return true;
}

}  // namespace kiln
