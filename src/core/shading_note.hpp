#pragma once

#include "ambient_occlusion.hpp"
#include "contact_shadow.hpp"

#include <string>

namespace kiln {

// The two shading passes are separate dials with separate owners, but every
// surface that shows them wants the same sentence: the REPL banner, the reply to
// `preview`, and the GUI HUD. Building that sentence here -- from the values
// rather than from a Session -- keeps the agent's help text and the REPL from
// printing two different things, and keeps agent.cpp from having to know what a
// Session is.

// Compact state readout, e.g. "contact shadow 2/4, ao 1/4".
inline std::string shading_note(int contact_shadow, int ao) {
    return "contact shadow " + std::to_string(contact_shadow) + "/" +
           std::to_string(kContactShadowMax) + ", ao " + std::to_string(ao) + "/" +
           std::to_string(kAoMax);
}

// Both usage lines together, for a rejection or a help block.
inline std::string shading_usage() {
    return contact_shadow_usage() + "\n" + ao_usage();
}

// Suffix for the `preview` reply. Empty when both passes are off, so the default
// message is byte-for-byte what it was before the passes existed.
inline std::string shading_suffix(int contact_shadow, int ao) {
    if (contact_shadow <= 0 && ao <= 0) return "";
    return ", " + shading_note(contact_shadow, ao);
}

}  // namespace kiln
