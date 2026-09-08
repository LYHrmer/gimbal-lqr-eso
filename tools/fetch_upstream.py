"""Fetch and verify the exact upstream comparison source into ignored build/.

The upstream source is not vendored into this project's published source tree.
Use --source-dir for a verified offline copy; neither mode edits the upstream.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import tempfile
from urllib.request import Request, urlopen

ROOT = Path(__file__).resolve().parents[1]
UPSTREAM_REPOSITORY = "https://github.com/LamdaDay/YAW_Auto_Controller"
UPSTREAM_COMMIT = "665c5b4ab1067d6cb63122c120822f27502953e5"
UPSTREAM_SHA256 = {
    "yaw_auto_lqr_eso_controller.c":
        "e4ae2e11cc7ec425aed152dba6ce43d0dad4c490086253464d33e912475bfa61",
    "yaw_auto_lqr_eso_controller.h":
        "ce70e33074ffb5708a72074bf027cebb507fc0f251f16ebdd6c17f6ee708fe3b",
}
DEFAULT_DESTINATION = ROOT / "build" / "upstream" / UPSTREAM_COMMIT / "source"


def sha256(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def atomic_write(path: Path, data: bytes) -> None:
    """Readers see either the complete previous file or the complete new one."""
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(dir=path.parent, delete=False) as stream:
            temporary = Path(stream.name)
            stream.write(data)
        os.replace(temporary, path)
    finally:
        if temporary is not None and temporary.exists():
            temporary.unlink()


def verify_source(source_dir: Path | str) -> dict:
    source_dir = Path(source_dir).expanduser().resolve()
    files = {}
    for name, expected in UPSTREAM_SHA256.items():
        path = source_dir / name
        digest = sha256(path.read_bytes())
        if digest != expected:
            raise ValueError(f"Upstream checksum mismatch for {path}: "
                             f"expected {expected}, got {digest}")
        files[name] = {"sha256": digest, "path": str(path)}
    return {"repository": UPSTREAM_REPOSITORY, "commit": UPSTREAM_COMMIT,
            "source_dir": str(source_dir), "files": files}


def fetch_upstream(source_dir: Path | str | None = None,
                   destination: Path | str | None = None) -> Path:
    """Return an exact, checksum-verified cached source directory.

    Existing cached files with a wrong checksum fail closed. With source_dir,
    verify those files before copying; its git branch or working-tree status is
    irrelevant because the complete expected source bytes are pinned here.
    """
    destination = Path(destination or DEFAULT_DESTINATION).expanduser().resolve()
    if source_dir is None and all((destination / name).is_file()
                                  for name in UPSTREAM_SHA256):
        verify_source(destination)
        return destination

    origin = Path(source_dir).expanduser().resolve() if source_dir is not None else None
    if origin is not None:
        verify_source(origin)
    payloads = {}
    for name, expected in UPSTREAM_SHA256.items():
        if origin is not None:
            payload = (origin / name).read_bytes()
        else:
            url = ("https://raw.githubusercontent.com/LamdaDay/YAW_Auto_Controller/"
                   f"{UPSTREAM_COMMIT}/{name}")
            request = Request(url, headers={"User-Agent": "yaw-upstream-reproducibility/1"})
            try:
                with urlopen(request, timeout=30) as response:
                    payload = response.read()
            except OSError as exc:
                raise RuntimeError("Could not download the pinned upstream source. "
                                   "Use --source-dir with an offline checkout.") from exc
        actual = sha256(payload)
        if actual != expected:
            raise ValueError(f"Upstream checksum mismatch for {name}: "
                             f"expected {expected}, got {actual}")
        payloads[name] = payload

    for name, payload in payloads.items():
        atomic_write(destination / name, payload)
    metadata = verify_source(destination)
    metadata["acquisition"] = {"mode": "offline_copy" if origin is not None else "https",
                               "source_dir": str(origin) if origin is not None else None}
    atomic_write(destination / "source_manifest.json",
                 (json.dumps(metadata, indent=2, sort_keys=True) + "\n").encode())
    return destination


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source-dir", type=Path,
                        help="Offline source directory; exact pinned hashes are required")
    parser.add_argument("--destination", type=Path, default=DEFAULT_DESTINATION,
                        help="Source cache directory (default: build/upstream/<commit>/source)")
    args = parser.parse_args()
    directory = fetch_upstream(args.source_dir, args.destination)
    print(json.dumps(verify_source(directory), indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
