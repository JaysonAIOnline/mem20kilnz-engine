#pragma once

// Machine-drivable interface: JSON-RPC 2.0 over stdio.
//
// Additive to the existing paths. kiln's op pipeline is already JSON-shaped
// (apply_op takes an object, returns OpResult), so the cleanest machine
// interface is a line-delimited JSON-RPC loop that hands op objects straight to
// that same function -- no allowlist, no translation layer, nothing that can
// fall out of step with the DSL the REPL uses.
//
// The CLI, the REPL and the GUI are untouched: --rpc is a fourth entry point
// into the same Session and the same Document.

#include "../app.hpp"

#include <string>

namespace kiln {

// Run the JSON-RPC session loop over stdin/stdout until a `shutdown` request,
// EOF, or a stream failure. One request per line, one response per line.
//
// Nothing but protocol JSON is ever written to stdout. Returns 0 on a clean
// end, 3 if the streams themselves failed.
int run_rpc(Session& s);

// One-line hint for the --rpc flag, shown by --help and on a bad --rpc value.
std::string rpc_usage();

}  // namespace kiln
