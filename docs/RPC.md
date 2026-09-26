# kiln machine interface — JSON-RPC 2.0 over stdio

An external agent process can drive kiln end to end: author geometry, edit meshes,
frame the camera, set render quality, render and export — and get a structured,
parseable result back from every single call.

```bash
./kiln --rpc        # one JSON-RPC 2.0 request per line on stdin, one response per line on stdout
```

## Why this shape

kiln's op pipeline is already JSON-shaped: `apply_op(Document&, const json& op)`
takes one op object and returns `OpResult{ok, message}`. The machine interface is
therefore a thin JSON-RPC loop that hands op objects **straight to that same
function**, with no allowlist and no translation layer — so it can never fall out
of step with the DSL the REPL accepts, and an op added to `ops.cpp` is reachable
here the moment it compiles.

A Python binding is included as well (`bindings/kiln_rpc.py`), but it is a
*client* of this protocol rather than a second implementation: standard library
only, no compiled extension, no dependencies to install. That keeps one source of
truth for what an op means.

Additive by construction: `--rpc` is a fourth entry point into the same `Session`
and the same `Document`. The CLI, the REPL and the GUI are untouched.

## Framing

- One request per line, one response per line. No server-initiated messages.
- No banner, no prompt, no logging on stdout: **stdout carries protocol JSON and
  nothing else**. Anything diagnostic goes to stderr.
- Trailing `\n` after every response; flushed per line, so a client blocked on
  `read()` is never left waiting on a buffer.
- Requests are JSON-RPC 2.0 objects. A notification (no `id`) gets no response,
  per the spec — except a parse/invalid-request failure, which is always reported.

## Message shapes

### Request

```json
{"jsonrpc": "2.0", "id": 1, "method": "op", "params": {"op": "create", "primitive": "cube", "name": "Box"}}
```

`id` may be any JSON scalar; it is echoed back verbatim.

### Success response

```json
{"jsonrpc": "2.0", "id": 1, "result": {"ok": true, "message": "created Box"}}
```

### Failure response

```json
{"jsonrpc": "2.0", "id": 1, "error": {"code": -32005, "message": "samples 99 is outside [1, 16]",
  "data": {"min": 1, "max": 16, "default": 2}}}
```

Two failure channels, deliberately:

| Channel | Use | Shape |
|---|---|---|
| `error` | protocol-level failure — bad frame, unknown method, bad param, io | `{code, message, data?}` |
| `result.ok = false` | an op *ran* and reported failure | `{ok: false, message, data?}` |

An op that fails does not raise a protocol error, so a client can send a batch and
read an honest success/failure per call. Nothing is ever a silent success.

## Methods

| Method | `params` | `result` |
|---|---|---|
| `op` | one op object | `{ok, message}` |
| `ops` | array of op objects | `{ok, message, data.applied[]}` — stops at the first failure; each entry carries `index` |
| `command` | `"create cube Box"` or `{"line": "..."}` | `{ok, message, data.line}` — the full REPL path: DSL, English, raw JSON, meta commands |
| `describe` | — | the scene summary: `selection`, `edit_mode`, `nodes[]`, `bone_count` |
| `list` | — | `{count, selection, nodes[{index,name,type,selected,t}]}` |
| `info` | `"Box"` or `{"name": "Box"}` or omitted (uses selection) | one node's detail: transform, `faces`, `verts`, `lod`, `lods`, `face_sel`, `edit_mode` |
| `samples` | — to read, or `{"value": N}` | `{samples, min, max, default}` |
| `render` | `{path, width?, height?, samples?}` | `{path, width, height, samples}` |
| `export` | `{path}` | `{path, nodes, meshes}` — glTF/GLB |
| `save` | `{path}` | `{path}` — kiln scene JSON |
| `open` | `{path}` | `{path, nodes}` — undoable, like the REPL's `open` |
| `reset` | — | `{nodes: 0}` |
| `ping` | — | `{protocol, version, ops, methods[], ready}` |
| `initialize` / `help` | — | full capability payload: method list, all op names, samples range, notes |
| `shutdown` | — | `{bye: true}`, then the loop ends |

A client that has never seen kiln should start with `initialize`: it returns every
op name, every method and the samples range, so a first valid call needs no
out-of-band documentation.

## Op reference

Every op is an object whose `"op"` field names it. The names below are exactly
what `apply_op` dispatches on; `ping`/`initialize` report the same list, and
kiln answers a typo with the list of valid names rather than a bare rejection.

**Session & history**

| op | arguments | example |
|---|---|---|
| `new` | — | `{"op":"new"}` |
| `undo` / `redo` | — | `{"op":"undo"}` |

**Objects**

| op | arguments | example |
|---|---|---|
| `create` | `primitive` (`cube\|plane\|sphere\|cylinder\|cone\|torus\|camera\|light\|sun\|point\|empty`), `name`, `size`, `translation`, `rotation`, `scale`, `albedo`, `roughness`, `metallic`, `fov`, `intensity` | `{"op":"create","primitive":"cube","name":"Box","size":[1,1,1],"translation":[0,0.5,0]}` |
| `delete` | `target` | `{"op":"delete","target":"Box"}` |
| `duplicate` | `target` | `{"op":"duplicate","target":"Box"}` |
| `rename` | `target`, `to` | `{"op":"rename","target":"Box","to":"Hero"}` |
| `select` | `target` or `id` | `{"op":"select","target":"Box"}` |
| `mode` | `mode` (`object\|face\|edge\|vert`) | `{"op":"mode","mode":"face"}` |
| `parent` | `target`, `parent` | `{"op":"parent","target":"Child","parent":"Root"}` |

**Transforms & materials**

| op | arguments | example |
|---|---|---|
| `move` / `set_translation` | `target`, `translation` | `{"op":"move","target":"Box","translation":[0,1,0]}` |
| `translate` | `target`, `delta` (or `delta` alone in vertex mode) | `{"op":"translate","target":"Box","delta":[0,0.2,0]}` |
| `rotate` / `set_rotation` | `target`, `rotation` (degrees) or `delta` | `{"op":"rotate","target":"Box","rotation":[0,45,0]}` |
| `scale` / `set_scale` | `target`, `scale` or `delta` | `{"op":"scale","target":"Box","scale":[1.5,1,1]}` |
| `color` / `set_material` | `target`, `albedo`, `roughness`, `metallic`, `emission` | `{"op":"color","target":"Box","albedo":[0.88,0.42,0.18]}` |
| `apply` | — | `{"op":"apply"}` — bake the transform |

**Selection**

| op | arguments | example |
|---|---|---|
| `select_faces` | `target`, `region` (`top\|bottom\|left\|right\|front\|back\|all`) or `faces[]` | `{"op":"select_faces","target":"Box","region":"top"}` |
| `select_edges` | `region`, `axis`, or `edges[[a,b]]` | `{"op":"select_edges","axis":"y"}` |
| `select_edge_loop` / `select_edge_ring` | `region` / `axis` | `{"op":"select_edge_ring","axis":"y"}` |
| `select_verts` / `select_vertices` | `region` or `verts[]` | `{"op":"select_verts","region":"top"}` |
| `grow` / `select_more` / `invert` / `linked` | — | `{"op":"grow"}` |

**Mesh editing**

| op | arguments | example |
|---|---|---|
| `extrude` | `amount`, `region?`, `target?` | `{"op":"extrude","amount":0.5,"region":"top"}` |
| `inset` | `amount`, `region?` | `{"op":"inset","amount":0.15}` |
| `bevel` | `amount`, `region?` | `{"op":"bevel","amount":0.12}` |
| `loop_cut` / `loopcut` | `cuts`, `axis` | `{"op":"loop_cut","cuts":1,"axis":"y"}` |
| `delete_faces` | `region` | `{"op":"delete_faces","region":"bottom"}` |
| `subdivide` / `subdiv` | `cuts`, `region` | `{"op":"subdivide","cuts":1}` |
| `merge` / `weld` | `distance?` | `{"op":"weld","distance":0.001}` |
| `weld_distance` | `distance` | `{"op":"weld_distance","distance":0.002}` |
| `fill` | — | `{"op":"fill"}` |
| `mirror` | `axis` | `{"op":"mirror","axis":"x"}` |
| `solidify` | `thickness` | `{"op":"solidify","thickness":0.08}` |
| `array` | `count`, `axis`, `spacing` or `offset` | `{"op":"array","count":4,"axis":"x","spacing":1.2}` |
| `spin` | `steps`, `angle`, `axis` | `{"op":"spin","steps":8,"angle":360,"axis":"y"}` |
| `bridge` | — | `{"op":"bridge"}` |
| `join` | `other` | `{"op":"join","other":"Sphere"}` |
| `ground` / `to_ground` | — | `{"op":"ground"}` — drop onto y=0 |
| `origin` | — | `{"op":"origin"}` |
| `triangulate` / `poke` / `flip` / `dissolve` | — | `{"op":"triangulate"}` |
| `duplicate_faces` | `amount` | `{"op":"duplicate_faces","amount":0.2}` |
| `separate` | — | `{"op":"separate"}` |
| `smooth` | `factor`, `iters` | `{"op":"smooth","factor":0.5,"iters":2}` |
| `push` | `amount` | `{"op":"push","amount":0.1}` |
| `sphere` / `to_sphere` | `factor` | `{"op":"sphere","factor":1}` |
| `flatten` | `axis` | `{"op":"flatten","axis":"y"}` |
| `randomize` | `amount` | `{"op":"randomize","amount":0.05}` |
| `bisect` | `axis`, `distance` | `{"op":"bisect","axis":"y","distance":0}` |
| `snap` | `grid` | `{"op":"snap","grid":0.1}` |
| `split` / `connect` | — | `{"op":"split"}` |

**Rigging & animation**

| op | arguments | example |
|---|---|---|
| `rig` | `kind` (`biped\|quadruped\|face`), `pose` (`tpose\|apose`) | `{"op":"rig","kind":"biped","pose":"tpose"}` |
| `bind` | — | `{"op":"bind"}` |
| `pose` | `target` (bone name), `rotation` | `{"op":"pose","target":"UpperArm.L","rotation":[0,0,40]}` |
| `rest` | — | `{"op":"rest"}` |
| `shape` | `name`, `value` | `{"op":"shape","name":"smile","value":1}` |

**Level of detail**

| op | arguments | example |
|---|---|---|
| `lod` | omit `level` to build; `level` to switch | `{"op":"lod"}` · `{"op":"lod","level":1}` |

**Inspection**

| op | arguments | example |
|---|---|---|
| `describe` | — | `{"op":"describe"}` — returns the scene JSON as the op's message |

**Camera**

| op | arguments | example |
|---|---|---|
| `lookat` | `eye`, `center` | `{"op":"lookat","eye":[6,4,7],"center":[0,0.5,0]}` |
| `frame` / `focus` | `target` (node name, or omit for everything), `axis` (`iso\|front\|side\|top`), `margin` | `{"op":"frame","target":"Box","axis":"iso"}` |
| `set_camera` | `target`, `fov` | `{"op":"set_camera","target":"Camera","fov":50}` |

That is **79 op names**. `ping` and `initialize` report the live count, so an
agent can assert coverage rather than trusting this table.

## Render quality: `--samples`

Supersampling is a *session setting*, not a scene op, so it has its own method —
the same dial the `--samples` flag and the REPL's `samples` command write:

```json
{"jsonrpc":"2.0","id":1,"method":"samples"}                  -> {"samples":2,"min":1,"max":16,"default":2}
{"jsonrpc":"2.0","id":2,"method":"samples","params":{"value":4}}  -> {"samples":4,"min":1,"max":16,"default":2}
```

`render` also accepts a per-call `samples`, which overrides the session value for
that render only:

```json
{"jsonrpc":"2.0","id":3,"method":"render","params":{"path":"hero.png","width":1920,"height":1080,"samples":8}}
```

The accepted range is owned by `src/core/samples.hpp` — `kSamplesMin = 1`,
`kSamplesMax = 16`, `kDefaultSamples = 2` — one place, so the flag, the REPL
command, this protocol and the Python binding cannot drift apart. The renderer
draws at $$N \times N$$ the target resolution and box-filters back down, so cost
grows with $$N^2$$: 16×16 is 256 samples per output pixel. Reach for it on a
final still, not on an animation.

## Error codes

| Code | Name | Raised when | `data` |
|---|---|---|---|
| `-32700` | parse error | the line is not valid JSON | `received_bytes` |
| `-32600` | invalid request | JSON, but not a request object (or no string `method`) | — |
| `-32601` | method not found | unknown `method` | `methods[]` — what does exist |
| `-32602` | invalid params | wrong param type or a required field missing | `received`, `hint` / `usage` |
| `-32603` | internal error | kiln itself failed | — |
| `-32000` | op failed | an op ran and reported failure (also returned as `result.ok=false`) | the op's message |
| `-32001` | no target | an op needs a node or mesh and none is selected | `hint` |
| `-32002` | not found | a named node does not exist | — |
| `-32003` | no camera | nothing to render with | — |
| `-32004` | io error | reading or writing a file failed | `path` |
| `-32005` | bad samples | samples outside `[1, 16]` | `min`, `max`, `default`, `usage` |
| `-32006` | unsupported | a real feature kiln does not implement | — |

`-32768..-32000` is JSON-RPC's reserved range; `-32000..-32099` is the server's
slice, and `-32000..-32006` is what this server defines. Nothing below `-32006`
is used.

Every one of these leaves the process running: the session stays usable after a
bad call, which is what makes a long agent loop safe.

## Worked example

A complete session — geometry, a selection-sensitive edit chain, framing, quality,
render, export, the scene summary, and a handled failure. This is
`examples/rpc_session.py`, verbatim and runnable:

```python
import sys, os
sys.path.insert(0, "bindings")
from kiln_rpc import Kiln, OpFailed

with Kiln("./kiln") as k:
    print(f"connected: {k}")            # <Kiln ./kiln protocol='kiln-rpc' v1 ops=79>

    # create -> extrude -> re-select -> bevel ->
    # re-select -> inset. bevel clears the face selection, so the
    # selection has to be re-made before each op that needs one.
    for op in [
        {"op": "create", "primitive": "plane", "name": "Ground", "size": [12, 1, 12]},
        {"op": "color", "target": "Ground", "albedo": [0.30, 0.32, 0.36], "roughness": 0.9},
        {"op": "create", "primitive": "cube", "name": "Box", "size": [1.4, 1.4, 1.4],
         "translation": [0, 0.7, 0], "albedo": [0.88, 0.42, 0.18]},
        {"op": "select_faces", "target": "Box", "region": "top"},
        {"op": "extrude", "amount": 0.55},
        {"op": "select_faces", "target": "Box", "region": "top"},
        {"op": "bevel", "amount": 0.10},
        {"op": "select_faces", "target": "Box", "region": "top"},
        {"op": "inset", "amount": 0.18},
        {"op": "mode", "mode": "object"},
        {"op": "create", "primitive": "sphere", "name": "Ball", "size": [0.5, 0.5, 0.5],
         "translation": [1.9, 0.5, 0.3], "albedo": [0.95, 0.78, 0.28]},
        {"op": "move", "target": "Ball", "translation": [1.9, 0.5, 0.3]},
        {"op": "frame", "axis": "iso"},          # no target = frame everything
    ]:
        print(" ", op["op"], "->", k.op(op)["message"])

    # a batch is one request; it stops at the first failure
    k.ops([{"op": "color", "target": "Ball", "albedo": [0.95, 0.78, 0.28], "roughness": 0.25},
           {"op": "select", "target": "Box"}])

    k.samples(4)                                  # session render quality
    k.render("hero.png", 960, 540, samples=4)     # or per-call
    k.export("hero.glb")

    desc = k.describe()
    print("nodes:", [(n["name"], n["type"]) for n in desc["nodes"]])

    try:
        k.op({"op": "frame", "target": "NoSuchNode"})
    except OpFailed as exc:
        print(f"handled: [{exc.code}] {exc.message}")   # [-32000] frame: no node 'NoSuchNode'
    print("still alive:", k.describe() is not None)      # True — the session survived
```

Run it:

```bash
cd kiln
make                      # builds ./kiln
python3 examples/rpc_session.py ./kiln
```

### The same thing as raw protocol lines

```bash
printf '%s\n' \
  '{"jsonrpc":"2.0","id":1,"method":"op","params":{"op":"create","primitive":"cube","name":"Box"}}' \
  '{"jsonrpc":"2.0","id":2,"method":"op","params":{"op":"select_faces","target":"Box","region":"top"}}' \
  '{"jsonrpc":"2.0","id":3,"method":"op","params":{"op":"extrude","amount":0.6}}' \
  '{"jsonrpc":"2.0","id":4,"method":"samples","params":{"value":4}}' \
  '{"jsonrpc":"2.0","id":5,"method":"render","params":{"path":"out.png"}}' \
  '{"jsonrpc":"2.0","id":6,"method":"shutdown"}' \
  | ./kiln --rpc
```

```
{"id":1,"jsonrpc":"2.0","result":{"message":"created Box","ok":true}}
{"id":2,"jsonrpc":"2.0","result":{"message":"selected 1 faces on Box","ok":true}}
{"id":3,"jsonrpc":"2.0","result":{"message":"extruded 1 faces by 0.600000","ok":true}}
{"id":4,"jsonrpc":"2.0","result":{"default":2,"max":16,"min":1,"samples":4}}
{"id":5,"jsonrpc":"2.0","result":{"height":540,"path":"out.png","samples":4,"width":960}}
{"id":6,"jsonrpc":"2.0","result":{"bye":true}}
```

## Client, not a second implementation

`bindings/kiln_rpc.py` is ~30 lines of plumbing over `subprocess` and `json`.
Everything it can do maps to one of the methods above, so the protocol stays the
only thing that has to be learned.

It raises on failure rather than returning sentinels, and the two classes carry
the structured payload so a caller branches on `code` instead of parsing text:

- `KilnError` — a JSON-RPC refusal. Attributes: `.code`, `.message`, `.data`.
- `OpFailed` (a `KilnError`) — the op ran and said no.

`try_op()` / `try_command()` return the failure instead of raising, for callers
that want to inspect it. `Kiln()` as a context manager guarantees the subprocess
is reaped.

## Verification

The interface was checked end to end, and the evidence is reproducible:

- **REPL parity** — the same 10 op-arrays run through the REPL and through the
  RPC produce a **byte-identical scene** (`save` → md5 match) and a
  **byte-identical render** (`preview` → md5 `9925399be6dd98da38ecb97d1e6be57a`).
- **No regression** — a fixed scene rendered by the pre-change binary and by this
  one gives an identical PNG (`89ffa9631bb2d1ac57c8baf0a07fcf34`) and an
  identical glTF export (`460ed423a44d39371df21e7cc5b9baf0`). Omitting `--samples`
  and passing `--samples 2` are byte-identical, so the default is preserved.
- **Errors are data** — 14 deliberately malformed frames (bad JSON, non-object
  frame, no method, unknown method, wrong param type, unknown op, missing node,
  `samples 0`, `samples 99`, non-numeric samples, missing file, unwritable path,
  frame on an empty scene) each produced a structured error naming the field or
  the range, the process **exited 0**, wrote nothing to stderr, and a fresh
  session started normally afterwards.
- **Clean build** — `make -j2` with `-Wall -Wextra`, zero warnings.
