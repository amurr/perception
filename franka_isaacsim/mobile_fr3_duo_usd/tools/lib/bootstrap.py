"""Ensure tooling root is importable when scripts are launched by Isaac python.sh."""

from __future__ import annotations

import os
import sys
from pathlib import Path


def ensure_tooling_on_path(tooling_root: Path | None = None) -> Path:
    """Insert the isaacsim6 tooling root on ``sys.path`` and return it.

    Entrypoints that cannot ``import lib`` yet should load this file by path::

        import runpy
        from pathlib import Path
        _TOOLING = runpy.run_path(
            str(Path(__file__).resolve().parents[1] / "lib" / "bootstrap.py")
        )["ensure_tooling_on_path"]()
    """
    if tooling_root is None:
        env = os.environ.get("ISAACSIM6_TOOLING_ROOT")
        tooling_root = Path(env) if env else Path(__file__).resolve().parents[1]
    root = Path(tooling_root).resolve()
    root_str = str(root)
    if root_str not in sys.path:
        sys.path.insert(0, root_str)
    return root
