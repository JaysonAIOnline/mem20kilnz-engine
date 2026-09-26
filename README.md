# kiln

Linux-only, AI-native 3D modeler. The agent is the app: you talk, the scene graph changes, the viewport follows.

Not a Blender clone. v1 is a modeler — primitives, transforms, materials, cameras, lights, glTF export — with every mutation expressed as an operation the agent can emit.

## Why this shape

Blender’s power is a thousand tools. kiln’s power is one language of ops (`create`, `move`, `color`, …) plus a viewport. English, a REPL, JSON, or an OpenAI-compatible model all compile to that language.

Y is up. A unit cube sitting on the ground lives at `(0, 0.5, 0)`.

## Quick start (headless, any Linux)

```bash
make
./kiln --repl
```

```
kiln> add a red cube
kiln> create plane Ground size 12 1 12
kiln> color Ground gray
kiln> lookat 5 3 6  0 0.5 0
kiln> preview out.png
kiln> export out.glb
```

One shot:

```bash
./kiln --headless --export scene.glb --preview scene.png \
  "add a red cube" "create sphere Ball size 0.6 0.6 0.6 at 1.4 0.3 0"
```

## Anti-aliasing: `--samples N`

The software preview supersamples: it draws at **N x N** the target resolution and
box-filters back down, which is what smooths silhouettes and the ground grid.
`--samples N` sets N for every preview the run draws.

Accepted range is **1 to 16** inclusive. **The default is 2**, the historical 2x2
grid, so leaving the flag off renders exactly as before.

```bash
./kiln --headless --samples 1  --preview fast.png   "frame all"   # off — fastest
./kiln --headless --samples 4  --preview nice.png   "frame all"   # 4x4
./kiln --headless --samples 16 --preview final.png  "frame all"   # 16x16 — smoothest
```

Cost grows with N squared (16x16 renders 256 samples per output pixel), so reach for
16 on a final still, not on an animation. Values outside 1-16, non-numbers, and
zero/negative counts are rejected up front with a message naming the range and
exit code 2.

The same dial is available at runtime from the REPL, and it is what the GUI reads:

```
kiln> samples 4      # set supersampling for subsequent previews
kiln> samples        # report the current value
```

It composes with everything else — `frame`/`focus`, LOD switches and material or
selection state are unaffected by the sample count.

`--ss` is a short alias for the same flag, and the value may be joined with `=`
as well as spaced:

```bash
./kiln --headless --ss 8        --preview a.png "frame all"   # same as --samples 8
./kiln --headless --samples=8   --preview b.png "frame all"   # inline spelling
./kiln --headless --ss=8        --preview c.png "frame all"   # inline, short alias
```

All of these reach one parser in `src/core/samples.hpp`, so the spellings cannot
disagree about the accepted range or the error text. `--aa` remains as the older
long alias.

## Contact shadows and ambient occlusion

Two look passes for the software preview. Both run on the finished, already
average-filtered image, using the depth buffer the colour pass just wrote — no
GPU, no second geometry pass, nothing added to how the scene is rasterised.

**Contact shadows** (`--contact-shadow N`, 0-4, default **0**) ask, for the surface
each pixel shows, how far it is from the first thing that blocks the sun. An object
touching the ground blocks immediately, so the seam under it goes dark; an object
floating above the ground blocks only after a gap, so its shadow is lighter and
detaches. That difference is why a floating mesh stops reading as pasted on.

**Ambient occlusion** (`--ao N`, 0-4, default **0**) probes a short hemisphere around
the surface each pixel shows and darkens the pixel by how much of that hemisphere
is blocked. Creases, cavities and the seams where two surfaces meet therefore go
darker than flat areas, so the form reads in depth even where the sun is not raking
across it. It reads the same level-of-detail geometry the colour pass does, so AO on
a decimated mesh follows the LOD.

```bash
./kiln --headless --contact-shadow 2 --preview grounded.png  "frame all"   # hint of a shadow
./kiln --headless --ao 3              --preview deep.png      "frame all"   # cavities darken
./kiln --headless --contact-shadow 4 --ao 4 --preview hero.png "frame all" # both, strong
```

**Both default to 0 (off).** A render that does not name them is byte-identical to
one made before the passes existed — `md5sum` on the PNG matches. **0 is accepted
here and means "off"**, which is the one way these two differ from `--samples`,
where 0 is rejected because it would disable nothing.

Accepted range is **0 to 4** inclusive on each. Values outside 0-4, non-numbers,
zero-prefixed fractions and negative counts are rejected up front with a message
naming the range and exit code 2.

### Cost

The dial changes the look, **not** the render time. Each pass marches a fixed
number of samples per pixel regardless of the value — 16 steps for the sun ray, 6
probes x 6 steps for the hemisphere — so `--contact-shadow 1` and
`--contact-shadow 4` cost the same. What grows with the dial is only how much of
the accumulated occlusion is applied.

The passes run once per output pixel, *after* the supersampled buffer has been
box-filtered down, so their cost is fixed while `--samples` multiplies the cost of
everything under them. That makes the relative overhead fall as samples rise: on
the 960x540 QA scene both passes at strength 4 add roughly **0.2-0.3 s**, about
**+70 % at `--samples 1`**, **+50 % at `--samples 2`**, and **+13 % at `--samples 8`**.

```
kiln> shadow 2       # set contact-shadow strength for subsequent previews
kiln> shadow 0       # turn them off again
kiln> shadow         # report the current value and the range
kiln> ao 3           # set ambient-occlusion strength
kiln> shading        # report both passes at once
```

The same two dials are what the GUI HUD reads, so `--contact-shadow` and the
readout cannot disagree about what the renderer is running.

## Baked previews in an export: `--glb-preview`

`export` can bake one preview PNG into the GLB it writes, so a model file that
leaves kiln carries its own thumbnail. It is **off by default** and additive when
on: an export that does not ask for a preview stays byte-identical to what kiln
wrote before the option existed.

```bash
./kiln --headless --glb-preview --export hero.glb "frame all"                 # 512x288 thumbnail
./kiln --headless --glb-preview --preview-size 1024x576 --export hero.glb     # larger
./kiln --headless --glb-preview --ss 8 --export hero.glb "frame all"          # sharpens it too
```

The preview is drawn by the same software renderer as any other preview and
honours the same supersampling factor, so `--samples 8` sharpens the embedded
thumbnail and the on-disk PNG together. It is attached through a **second
material named `kiln preview`** rather than the mesh's own, so a viewer that
ignores it renders exactly the model kiln always wrote.

`--preview-size WxH` (also `--glb-preview-size`) sets the resolution: default
**512x288**, each side **16 to 2048**. The encoded PNG is capped at **4 MiB**, and
the render itself is capped at **32M supersampled pixels** -- at the maximum
resolution and the maximum sample count the renderer would need a one-gigabyte
buffer, so that combination is refused up front rather than dying mid-export.

From the REPL, the same setting is the `glbpreview` command:

```
kiln> glbpreview on           # bake a preview into the next export
kiln> glbpreview 1024x576     # set its resolution
kiln> glbpreview off          # back to a plain GLB
```

## Machine interface: `kiln --rpc`

`--rpc` runs a **JSON-RPC 2.0** session over stdin/stdout, so an external agent
process can drive kiln directly. One request per line, one response per line, and
nothing but protocol JSON on stdout. It is a fourth entry point into the same
`Session` and the same op pipeline the REPL uses -- no allowlist, no translation
layer.

```bash
printf '%s\n' \
  '{"jsonrpc":"2.0","id":1,"method":"op","params":{"op":"create","primitive":"cube","name":"Box"}}' \
  '{"jsonrpc":"2.0","id":2,"method":"render","params":{"path":"out.png","samples":4}}' \
  '{"jsonrpc":"2.0","id":3,"method":"shutdown"}' \
  | ./kiln --rpc
```

Methods: `op`, `ops`, `command`, `describe`, `list`, `info`, `samples`,
`render`, `export`, `save`, `open`, `reset`, `ping`, `initialize`, `help`,
`shutdown`. The protocol constants and the error table live in
`src/core/rpc.hpp`; the driver is `src/rpc/rpc.cpp`. See `docs/RPC.md` for the
full protocol.

Two Python entry points come with it: `bindings/kiln_rpc.py` is a small client
class, and `examples/rpc_session.py` drives a complete session end to end.

## GUI (Vulkan + Dear ImGui)

On a machine with Vulkan:

```bash
./scripts/bootstrap-ubuntu.sh   # Ubuntu/Debian
cmake -S . -B build -G Ninja
cmake --build build
./build/kiln
```

Or `./build/kiln --gui`. Headless agents use `preview out.png` (pixels) or `describe` (JSON). Vision models should take the PNG; non-vision models should take `describe`.

Export is **glTF/GLB** (posed/skinned meshes are baked). There is no FBX exporter.

Shaders compile with `glslc` or `glslangValidator`. If CMake cannot find them:

```bash
./scripts/compile_shaders.sh
```

## Language model

Without a key, kiln uses a small built-in agent plus the ops DSL.

```bash
export KILN_API_KEY=...          # or OPENAI_API_KEY
export KILN_API_BASE=https://api.openai.com/v1
export KILN_MODEL=gpt-4.1-mini
```

Any OpenAI-compatible `/chat/completions` endpoint works. The model is instructed to emit JSON ops only.

## Ops DSL

| command | example |
|---|---|
| create | `create cube Box size 1 1 1 at 0 0.5 0 color red` |
| move / translate | `move Box 0 1 0` · `translate Box 0 0.2 0` |
| rotate / scale | `rotate Box 0 45 0` · `scale Box 1.5` |
| color | `color Box kiln` |
| camera | `lookat 6 4 7  0 0.5 0` · `frame Box` · `frame all top` |
| duplicate / delete | `duplicate Box` · `delete Box` |
| undo / redo / new | |
| select faces | `select faces top` · `select faces 0 1` |
| extrude / inset | `extrude 0.5` · `extrude top 0.4` · `inset 0.12` |
| delete faces | `delete faces bottom` |
| edges / loop | `select edges top` · `select edge loop` · `select edge ring y` |
| verts | `select verts top` · `grab 0 0.2 0` |
| loop cut | `loopcut 1` · `loopcut y` |
| bevel | `bevel 0.12` · `bevel top 0.1` |
| subdivide / merge / fill | `subdivide` · `merge` · `fill` |
| mirror / solidify / array | `mirror x` · `solidify 0.08` · `array 4 x 1.4` |
| spin / bridge / join | `spin 8` · `bridge` · `join Sphere` |
| ground / origin | `ground` · `origin` |
| triangulate / poke / flip / dissolve | `triangulate` · `poke` · `flip` · `dissolve` |
| duplicate faces / separate | `duplicate faces 0.2` · `separate` |
| weld / smooth / push | `weld 0.001` · `smooth 0.5` · `push 0.1` |
| sphere / flatten / randomize | `sphere` · `flatten y` · `randomize 0.05` |
| apply / bisect / snap | `apply` · `bisect y` · `snap 0.1` |
| split / connect | `split` · `connect` |
| grow / invert / linked | `grow` · `invert` · `linked` |
| rig | `rig biped tpose` · `rig biped apose` · `rig quadruped` · `rig face` |
| skin / pose | `bind` · `pose UpperArm.L 0 0 40` · `rest` · `shape smile 1` |
| lod | `lod` (build) · `lod 1` (switch) |
| describe | `describe` — JSON for non-vision agents |
| export / preview / save / open | `export hero.glb` · `preview out.png` |
| samples | `samples 4` — supersampling for previews, 1–16, default 2 (`--ss` alias) |
| shadow / ao | `shadow 2` · `ao 3` — contact shadows / ambient occlusion, 0–4, default 0 = off |
| glbpreview | `glbpreview on` · `glbpreview 1024x576` — bake a preview PNG into the next export, off by default |

JSON is also valid input:

```json
[{"op":"create","primitive":"cube","name":"Box","albedo":[0.88,0.42,0.18]}]
```

## Layout

```
src/scene     scene graph, primitives
src/ops       undoable operations
src/agent     DSL + English + LLM → ops
src/io        glTF/GLB + kiln JSON
src/render    CPU preview (and Vulkan in src/gui)
src/rpc       JSON-RPC 2.0 stdio driver (--rpc)
src/gui       GLFW + Vulkan + Dear ImGui
src/core      shared constants/validators (samples range, shading dials, preview caps, rpc protocol, math)
bindings      Python client for --rpc
docs          RPC.md -- the machine-interface protocol
```

Headless `make` needs only g++ 9+. The GUI needs Vulkan, GLFW (fetched), and ImGui (fetched).

## License

MIT
