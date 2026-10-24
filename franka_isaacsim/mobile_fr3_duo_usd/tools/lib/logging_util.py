"""Append-only file + stdout logging helpers."""

from __future__ import annotations

from pathlib import Path


class FileLogger:
    def __init__(self, path: Path, truncate: bool = True) -> None:
        self.path = path
        self.path.parent.mkdir(parents=True, exist_ok=True)
        if truncate:
            self.path.write_text("", encoding="utf-8")

    def log(self, message: str) -> None:
        print(message, flush=True)
        with self.path.open("a", encoding="utf-8") as handle:
            handle.write(message + "\n")
