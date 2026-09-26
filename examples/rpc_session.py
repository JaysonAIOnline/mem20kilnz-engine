#!/usr/bin/env python3
"""A complete kiln session driven from an external process over JSON-RPC.

This is the worked example from docs/RPC.md, runnable as-is:

    cd kiln
    make
    python3 examples/rpc_session.py ./kiln

It builds a scene with geometry, transforms, mesh edits, framing and materials;
renders a preview at an explicit quality; exports glTF; prints the structured
scene summary; and finally shows what a rejected op looks like -- so it doubles
as a check that failures come back as data rather than killing the process.
"""

from __future__ import annotations

import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "bindings"))

from kiln_rpc import Kiln, OpFailed  # noqa: E402

OUT_DIR = os.path.dirname(os.path.abspath(__file__))


def main() -> int:
    binary = sys.argv[1] if len(sys.argv) > 1 else None
    out_png = os.path.join(OUT_DIR, "rpc_session.png")
    out_glb = os.path.join(OUT_DIR, "rpc_session.glb")

    with Kiln(binary) as k:
        print(f"connected: {k}")
        print(f"methods:   {', '.join(k.methods)}\n")

        # --- geometry: the built-in ops, one request each -------------------
        steps = [
            {"op": "create", "primitive": "plane", "name": "Ground", "size": [12, 1, 12]},
            {"op": "color", "target": "Ground", "albedo": [0.30, 0.32, 0.36], "roughness": 0.9},
            {"op": "create", "primitive": "cube", "name": "Box", "size": [1.4, 1.4, 1.4],
             "translation": [0, 0.7, 0], "albedo": [0.88, 0.42, 0.18]},
            # select -> extrude -> re-select -> bevel: bevel clears the face
            # selection, so the selection has to be re-made before inset.
            {"op": "select_faces", "target": "Box", "region": "top"},
            {"op": "extrude", "amount": 0.55},
            {"op": "select_faces", "target": "Box", "region": "top"},
            {"op": "bevel", "amount": 0.10},
            {"op": "select_faces", "target": "Box", "region": "top"},
            {"op": "inset", "amount": 0.18},
            {"op": "mode", "mode": "object"},
            # a second object, moved and rotated
            {"op": "create", "primitive": "sphere", "name": "Ball", "size": [0.5, 0.5, 0.5],
             "translation": [1.9, 0.5, 0.3], "albedo": [0.95, 0.78, 0.28]},
            {"op": "scale", "target": "Ball", "scale": [1.0, 1.0, 1.0]},
            {"op": "move", "target": "Ball", "translation": [1.9, 0.5, 0.3]},
            # frame the whole scene: omitting "target" means everything, the
            # same as `frame all` in the REPL.
            {"op": "frame", "axis": "iso"},
        ]
        for op in steps:
            r = k.op(op)
            print(f"  {op['op']:<14} -> {r['message']}")

        # --- a batch, applied as one request --------------------------------
        # `ops` takes an array and stops at the first failure, reporting every
        # outcome by index. Supersampling is a session setting, not a scene op,
        # so it goes through the `samples` method instead.
        batch = k.ops([
            {"op": "color", "target": "Ball", "albedo": [0.95, 0.78, 0.28], "roughness": 0.25},
            {"op": "select", "target": "Box"},
        ])
        print(f"\n  batch          -> {batch['message']}")

        # --- quality, render, export ---------------------------------------
        q = k.samples(4)
        print(f"  samples        -> {q['samples']} of {q['min']}..{q['max']} (default {q['default']})")

        info = k.info("Box")
        print(f"  Box            -> {info['faces']} faces, {info['verts']} verts")

        r = k.render(out_png, width=960, height=540, samples=4)
        print(f"  render         -> {r['path']} {r['width']}x{r['height']} at {r['samples']}x")

        r = k.export(out_glb)
        print(f"  export         -> {r['path']} ({r['nodes']} nodes, {r['meshes']} meshes)")

        # --- the structured scene summary ----------------------------------
        desc = k.describe()
        print(f"\n  scene: selection={desc['selection']!r} edit_mode={desc['edit_mode']!r}")
        for n in desc["nodes"]:
            extra = f" faces={n['faces']} verts={n['verts']}" if n["type"] == "mesh" else ""
            print(f"    {n['type']:<7} {n['name']}{extra}")

        # --- a failure is data, not a crash --------------------------------
        print("\n  deliberate failure:")
        try:
            k.op({"op": "frame", "target": "NoSuchNode"})
        except OpFailed as exc:
            print(f"    OpFailed       -> [{exc.code}] {exc.message}")

        still_alive = k.try_op({"op": "describe"})
        print(f"    session still up: {still_alive['ok']}")

    print("\nsession closed cleanly")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
