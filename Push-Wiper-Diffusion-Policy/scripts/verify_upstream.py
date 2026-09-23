#!/usr/bin/env python3
"""Verify every original upstream file against its pinned SHA-256 digest."""

import argparse
import hashlib
import json
from pathlib import Path


def verify(project_root: Path) -> dict:
    manifest = json.loads((project_root / "third_party/UPSTREAM.json").read_text())
    source_root = project_root / "third_party/diffusion_policy"
    failures = []
    for relative, expected in manifest["files"].items():
        path = source_root / relative
        if not path.is_file():
            failures.append(f"missing: {relative}")
        elif hashlib.sha256(path.read_bytes()).hexdigest() != expected:
            failures.append(f"modified: {relative}")
    if failures:
        raise RuntimeError("Upstream verification failed:\n" + "\n".join(failures))
    return {"revision": manifest["revision"], "verified_files": len(manifest["files"])}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--project-root", type=Path, default=Path(__file__).resolve().parents[1])
    args = parser.parse_args()
    print(json.dumps(verify(args.project_root), indent=2))


if __name__ == "__main__":
    main()
