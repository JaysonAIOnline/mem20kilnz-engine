#!/usr/bin/env python3
"""Python bindings for kiln's JSON-RPC stdio interface.

Standard library only -- no pip install, no compiled extension. It spawns
``kiln --rpc`` as a subprocess and speaks the line-delimited JSON-RPC 2.0
protocol documented in ``docs/RPC.md``.

    from kiln_rpc import Kiln

    with Kiln("./kiln") as k:
        k.op({"op": "create", "primitive": "cube", "name": "Box"})
        k.op({"op": "select_faces", "target": "Box", "region": "top"})
        k.op({"op": "extrude", "amount": 0.6})
        k.render("out.png", samples=4)
        print(k.describe()["nodes"])

Every call returns the ``result`` payload as a dict. A JSON-RPC level failure
raises :class:`KilnError`; an op that ran and reported failure raises
:class:`OpFailed`. Both carry the structured error/result, so a caller can
branch on ``code`` instead of parsing text.
"""

from __future__ import annotations

import json
import os
import shutil
import subprocess
import sys
from typing import Any, Dict, Iterable, List, Optional

__all__ = [
    "Kiln",
    "KilnError",
    "OpFailed",
    "ERR_PARSE",
    "ERR_INVALID_REQUEST",
    "ERR_METHOD_NOT_FOUND",
    "ERR_INVALID_PARAMS",
    "ERR_INTERNAL",
    "ERR_OP_FAILED",
    "ERR_NO_SCENE",
    "ERR_NOT_FOUND",
    "ERR_NO_CAMERA",
    "ERR_IO",
    "ERR_BAD_SAMPLES",
    "ERR_UNSUPPORTED",
    "SAMPLES_MIN",
    "SAMPLES_MAX",
    "SAMPLES_DEFAULT",
]

# ---------------------------------------------------------------------------
# Error codes. Mirrors src/core/rpc.hpp -- keep the two in step; docs/RPC.md
# lists what each one means.
# ---------------------------------------------------------------------------
ERR_PARSE = -32700
ERR_INVALID_REQUEST = -32600
ERR_METHOD_NOT_FOUND = -32601
ERR_INVALID_PARAMS = -32602
ERR_INTERNAL = -32603
ERR_OP_FAILED = -32000
ERR_NO_SCENE = -32001
ERR_NOT_FOUND = -32002
ERR_NO_CAMERA = -32003
ERR_IO = -32004
ERR_BAD_SAMPLES = -32005
ERR_UNSUPPORTED = -32006

# The supersampling range, from src/core/samples.hpp.
SAMPLES_MIN = 1
SAMPLES_MAX = 16
SAMPLES_DEFAULT = 2


class KilnError(RuntimeError):
    """A JSON-RPC level failure: bad request, unknown method, bad params, io."""

    def __init__(self, code: int, message: str, data: Any = None) -> None:
        super().__init__(f"[{code}] {message}")
        self.code = code
        self.message = message
        self.data = data


class OpFailed(KilnError):
    """An op ran and reported failure. Carries the op's own message in ``data``."""

    def __init__(self, message: str, data: Any = None) -> None:
        super().__init__(ERR_OP_FAILED, message, data)


def _find_binary(explicit: Optional[str]) -> str:
    """Resolve the kiln executable: as given, else on PATH, else next to this file."""
    if explicit:
        if os.path.isfile(explicit) and os.access(explicit, os.X_OK):
            return explicit
        found = shutil.which(explicit)
        if found:
            return found
        raise FileNotFoundError(f"kiln binary not found at {explicit!r}")

    found = shutil.which("kiln")
    if found:
        return found

    # Running from the source tree: bindings/ sits next to the built binary.
    here = os.path.dirname(os.path.abspath(__file__))
    for candidate in ("kiln", os.path.join("..", "kiln"), os.path.join("..", "build", "kiln")):
        path = os.path.normpath(os.path.join(here, candidate))
        if os.path.isfile(path) and os.access(path, os.X_OK):
            return path
    raise FileNotFoundError("kiln binary not found; pass its path to Kiln(binary=...)")


class Kiln:
    """A kiln modelling session driven over JSON-RPC.

    Use as a context manager so the subprocess is always reaped::

        with Kiln() as k:
            ...

    ``env`` is merged over the current environment, which is how you point a
    session at a model endpoint for the ``command`` method's English path.
    """

    def __init__(
        self,
        binary: Optional[str] = None,
        env: Optional[Dict[str, str]] = None,
        timeout: Optional[float] = 60.0,
    ) -> None:
        self.binary = _find_binary(binary)
        self.timeout = timeout
        run_env = dict(os.environ)
        if env:
            run_env.update(env)
        self._proc = subprocess.Popen(
            [self.binary, "--rpc"],
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            bufsize=1,
            env=run_env,
        )
        self._next_id = 1
        self._closed = False
        # Confirm the server is actually up before the caller sends real work.
        info = self.ping()
        if not info.get("ready"):
            raise KilnError(ERR_INTERNAL, "kiln --rpc started but did not report ready")
        self.protocol = info.get("protocol", "")
        self.version = info.get("version", 0)
        # Named op_count, not ops: `ops` is a method and must not be shadowed.
        self.op_count = info.get("ops", 0)
        self.methods = info.get("methods", [])

    # -- plumbing ----------------------------------------------------------

    def _write(self, payload: Dict[str, Any]) -> None:
        if self._closed or self._proc.stdin is None:
            raise KilnError(ERR_INTERNAL, "session is closed")
        try:
            self._proc.stdin.write(json.dumps(payload) + "\n")
            self._proc.stdin.flush()
        except BrokenPipeError as exc:  # server died
            raise KilnError(ERR_INTERNAL, f"kiln closed the pipe: {exc}") from exc

    def _readline(self) -> str:
        if self._proc.stdout is None:
            raise KilnError(ERR_INTERNAL, "session has no stdout")
        try:
            line = self._proc.stdout.readline()
        except Exception as exc:  # pragma: no cover - defensive
            raise KilnError(ERR_INTERNAL, f"read failed: {exc}") from exc
        if not line:
            # EOF where a response was due: surface whatever kiln said on stderr.
            err = ""
            if self._proc.stderr is not None:
                try:
                    err = self._proc.stderr.read() or ""
                except Exception:
                    err = ""
            raise KilnError(ERR_INTERNAL, f"kiln ended the session early. stderr: {err.strip()}")
        return line

    def call(self, method: str, params: Any = None) -> Any:
        """Send one request and return its ``result``.

        Raises :class:`KilnError` for a JSON-RPC error object.
        """
        req_id = self._next_id
        self._next_id += 1
        payload: Dict[str, Any] = {"jsonrpc": "2.0", "id": req_id, "method": method}
        if params is not None:
            payload["params"] = params
        self._write(payload)

        line = self._readline()
        try:
            reply = json.loads(line)
        except json.JSONDecodeError as exc:
            raise KilnError(ERR_PARSE, f"kiln sent a non-JSON line: {line[:200]!r}") from exc

        if reply.get("id") != req_id:
            raise KilnError(
                ERR_INVALID_REQUEST,
                f"response id {reply.get('id')!r} does not match request {req_id}",
            )

        if "error" in reply:
            err = reply["error"]
            code = err.get("code", ERR_INTERNAL)
            message = err.get("message", "unknown error")
            data = err.get("data")
            if code == ERR_OP_FAILED:
                raise OpFailed(message, data)
            raise KilnError(code, message, data)

        return reply.get("result")

    def notify(self, method: str, params: Any = None) -> None:
        """Send a notification (no id, no reply expected)."""
        payload: Dict[str, Any] = {"jsonrpc": "2.0", "method": method}
        if params is not None:
            payload["params"] = params
        self._write(payload)

    # -- ops and commands --------------------------------------------------

    def op(self, op: Dict[str, Any]) -> Dict[str, Any]:
        """Apply one op. The dict is the op itself, e.g. ``{"op": "bevel", "amount": 0.1}``.

        Raises :class:`OpFailed` when the op ran and reported failure, so a bad
        edit can never pass silently.
        """
        result = self.call("op", op)
        if not isinstance(result, dict) or not result.get("ok", False):
            message = result.get("message", "op failed") if isinstance(result, dict) else "op failed"
            raise OpFailed(message, result)
        return result

    def ops(self, ops: Iterable[Dict[str, Any]]) -> Dict[str, Any]:
        """Apply a batch. Stops at the first failure and reports every outcome."""
        result = self.call("ops", list(ops))
        if not isinstance(result, dict) or not result.get("ok", False):
            message = result.get("message", "batch failed") if isinstance(result, dict) else "batch failed"
            raise OpFailed(message, result)
        return result

    def command(self, line: str) -> Dict[str, Any]:
        """Run one DSL / English / JSON line, exactly as the REPL would."""
        result = self.call("command", line)
        if not isinstance(result, dict) or not result.get("ok", False):
            message = result.get("message", "command failed") if isinstance(result, dict) else "command failed"
            raise OpFailed(message, result)
        return result

    def try_op(self, op: Dict[str, Any]) -> Dict[str, Any]:
        """Like :meth:`op` but returns the failure instead of raising it."""
        return self.call("op", op)

    def try_command(self, line: str) -> Dict[str, Any]:
        """Like :meth:`command` but returns the failure instead of raising it."""
        return self.call("command", line)

    # -- scene inspection --------------------------------------------------

    def describe(self) -> Dict[str, Any]:
        """Structured scene summary: selection, edit mode, one entry per node."""
        return self.call("describe")

    def list_nodes(self) -> Dict[str, Any]:
        """Nodes with index, name, type, transform and a selection marker."""
        return self.call("list")

    def info(self, name: Optional[str] = None) -> Dict[str, Any]:
        """Detail for a named node, or the current selection."""
        return self.call("info", name)

    def node_names(self) -> List[str]:
        return [n["name"] for n in self.list_nodes()["nodes"]]

    # -- render / export ---------------------------------------------------

    def samples(self, value: Optional[int] = None) -> Dict[str, Any]:
        """Read the supersampling factor, or set it for subsequent renders."""
        if value is None:
            return self.call("samples")
        return self.call("samples", {"value": value})

    def render(
        self,
        path: str,
        width: int = 960,
        height: int = 540,
        samples: Optional[int] = None,
    ) -> Dict[str, Any]:
        """Render a preview PNG. ``samples`` 1-16 overrides the session value."""
        params: Dict[str, Any] = {"path": path, "width": width, "height": height}
        if samples is not None:
            params["samples"] = samples
        return self.call("render", params)

    def export(self, path: str) -> Dict[str, Any]:
        """Write the scene as glTF/GLB."""
        return self.call("export", {"path": path})

    def save(self, path: str) -> Dict[str, Any]:
        """Write the scene as kiln JSON."""
        return self.call("save", {"path": path})

    def open(self, path: str) -> Dict[str, Any]:
        """Load a kiln JSON scene (undoable, like the REPL's ``open``)."""
        return self.call("open", {"path": path})

    def reset(self) -> Dict[str, Any]:
        """Empty the scene."""
        return self.call("reset")

    def ping(self) -> Dict[str, Any]:
        """Liveness, protocol version, op count and the method list."""
        return self.call("ping")

    # -- lifecycle ---------------------------------------------------------

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        try:
            if self._proc.stdin is not None and not self._proc.stdin.closed:
                try:
                    self._proc.stdin.write(json.dumps({"jsonrpc": "2.0", "id": 0, "method": "shutdown"}) + "\n")
                    self._proc.stdin.flush()
                except (BrokenPipeError, ValueError):
                    pass
                self._proc.stdin.close()
        finally:
            try:
                self._proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self._proc.kill()
            if self._proc.stdout is not None:
                self._proc.stdout.close()
            if self._proc.stderr is not None:
                self._proc.stderr.close()

    def __enter__(self) -> "Kiln":
        return self

    def __exit__(self, *exc: Any) -> None:
        self.close()

    def __repr__(self) -> str:
        return f"<Kiln {self.binary} protocol={self.protocol!r} v{self.version} ops={self.op_count}>"


def _demo() -> int:  # pragma: no cover - manual smoke run
    binary = sys.argv[1] if len(sys.argv) > 1 else None
    with Kiln(binary) as k:
        print(k)
        k.op({"op": "create", "primitive": "cube", "name": "Box"})
        k.op({"op": "select_faces", "target": "Box", "region": "top"})
        k.op({"op": "extrude", "amount": 0.6})
        print(json.dumps(k.describe(), indent=2))
        try:
            k.op({"op": "inset", "amount": 0.1})
        except OpFailed as exc:
            print(f"expected structured failure: {exc}")
    return 0


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(_demo())
