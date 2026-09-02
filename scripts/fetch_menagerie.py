#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""fetch_menagerie.py -- fetch ONE MuJoCo Menagerie robot into assets/menagerie/.

The full google-deepmind/mujoco_menagerie repo is large; this script pulls
only the requested model directory (default: unitree_h1) via a shallow
clone + sparse checkout, then copies it into assets/menagerie/<model>/ and
records provenance + license (Apache-2.0) next to it. Idempotent: re-running
refreshes the target in place.

Usage:
    python scripts/fetch_menagerie.py                # unitree_h1
    python scripts/fetch_menagerie.py --model unitree_g1
    python scripts/fetch_menagerie.py --keep-tmp     # keep the temp clone

No files outside assets/menagerie/ are touched. Requires: git on PATH.
"""

import argparse
import os
import shutil
import subprocess
import sys
import tempfile

REPO_URL = "https://github.com/google-deepmind/mujoco_menagerie"
REVISION = "main"           # shallow clone of the default branch tip
LICENSE_NAME = "LICENSE"
# The Menagerie repo is Apache-2.0, but its single root LICENSE is a
# per-model digest: each model directory carries its OWN license section
# (e.g. unitree_h1 is BSD-3-Clause by Unitree Robotics). We extract the
# model's own section into the copied LICENSE (with the repo-level license
# noted in the header) so the attribution that ships with the assets is
# exactly the one that applies to them.
REPO_LICENSE_NOTE = (
    "The MuJoCo Menagerie REPOSITORY (packaging, scene tooling) is licensed "
    "under the Apache License 2.0 (https://github.com/google-deepmind/"
    "mujoco_menagerie). The MODEL CONTENT in this directory is licensed "
    "under the terms below, reproduced from the repository's root LICENSE "
    "digest for this directory.\n\n"
)

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEST_PARENT = os.path.join(ROOT, "assets", "menagerie")

PROVENANCE_TMPL = """# MuJoCo Menagerie -- {model}

Source:      {repo}
Directory:   {model}/ (sparse checkout, branch '{rev}')
Fetched:     {stamp} (scripts/fetch_menagerie.py)
License:     see LICENSE in this directory -- the model's own license as
             extracted from the repo LICENSE digest (Menagerie repo itself
             is Apache-2.0; e.g. unitree_h1 model content is BSD-3-Clause
             by Unitree Robotics)
Citation:    Menagerie model by the Menagerie contributors; see the upstream
             {model}/README.md for model-specific credits and documentation.

Re-fetch / update at any time:
    python scripts/fetch_menagerie.py --model {model}
"""


def run(cmd, cwd=None):
    print("+ %s" % " ".join(cmd), flush=True)
    proc = subprocess.run(cmd, cwd=cwd, capture_output=True, text=True,
                          encoding="utf-8", errors="replace")
    if proc.returncode != 0:
        raise RuntimeError("command failed (%d): %s\n%s"
                           % (proc.returncode, " ".join(cmd),
                              (proc.stderr or proc.stdout)[:2000]))
    return proc.stdout


def run_bytes(cmd, cwd=None):
    """Binary-safe variant (Windows console encoding must not mangle data)."""
    print("+ %s" % " ".join(cmd), flush=True)
    proc = subprocess.run(cmd, cwd=cwd, capture_output=True)
    if proc.returncode != 0:
        raise RuntimeError("command failed (%d): %s\n%s"
                           % (proc.returncode, " ".join(cmd),
                              (proc.stderr or proc.stdout)[:2000]))
    return proc.stdout


def fetch(model, keep_tmp=False):
    dest = os.path.join(DEST_PARENT, model)
    tmp = tempfile.mkdtemp(prefix="menagerie_")
    clone_dir = os.path.join(tmp, "mjmenagerie")
    try:
        # shallow + blobless + sparse: minimal bytes for one directory
        # (transient network resets are retried a couple of times)
        clone_cmd = ["git", "clone", "--depth", "1", "--filter=blob:none",
                     "--sparse", "--branch", REVISION, REPO_URL, clone_dir]
        for attempt in range(3):
            try:
                run(clone_cmd)
                break
            except RuntimeError:
                if attempt == 2:
                    raise
                import shutil as _shutil
                _shutil.rmtree(clone_dir, ignore_errors=True)
                import time as _time
                _time.sleep(2.0 * (attempt + 1))
        run(["git", "sparse-checkout", "set", model], cwd=clone_dir)

        src = os.path.join(clone_dir, model)
        if not os.path.isdir(src):
            raise RuntimeError("sparse checkout produced no %s/ directory"
                               % model)
        # the sparse filter may leave blobs unmaterialized on some git
        # versions; force-checkout the files so the meshes really exist
        run(["git", "checkout", "HEAD", "--", model], cwd=clone_dir)
        # the repo LICENSE is a file (cone sparse-checkout only accepts
        # directories) -> fetch that single blob on demand
        lic = run_bytes(["git", "show", "HEAD:" + LICENSE_NAME],
                        cwd=clone_dir)
        with open(os.path.join(clone_dir, LICENSE_NAME), "wb") as fh:
            fh.write(lic)

        if os.path.isdir(dest):
            shutil.rmtree(dest)
        os.makedirs(DEST_PARENT, exist_ok=True)
        shutil.copytree(src, dest)
        lic_src = os.path.join(clone_dir, LICENSE_NAME)
        if not os.path.isfile(lic_src):
            raise RuntimeError("LICENSE not found in the cloned repo")
        with open(lic_src, encoding="utf-8", errors="replace") as fh:
            lic_all = fh.read()
        # digest layout per section: "=====" line, "License for contents in
        # the directory '<name>/'" line, "=====" line, then the license body
        import re
        pat = re.compile(r"License for contents in the directory '([^']+)/'")
        matches = list(pat.finditer(lic_all))
        section = None
        for i, mm in enumerate(matches):
            if mm.group(1) != model:
                continue
            start = lic_all.rfind("\n", 0, lic_all.rfind("=" * 20, 0,
                                                         mm.start())) + 1
            if i + 1 < len(matches):
                end = lic_all.rfind("\n", 0,
                                    lic_all.rfind("=" * 20, 0,
                                                  matches[i + 1].start())) + 1
            else:
                end = len(lic_all)
            section = lic_all[start:end]
            break
        if section is not None:
            lic_out = REPO_LICENSE_NOTE + section.rstrip() + "\n"
        else:  # unknown model layout: keep the whole digest
            lic_out = REPO_LICENSE_NOTE + lic_all
        with open(os.path.join(dest, LICENSE_NAME), "w", encoding="utf-8",
                  newline="\n") as fh:
            fh.write(lic_out)

        import datetime
        stamp = datetime.date.today().isoformat()
        with open(os.path.join(DEST_PARENT, model + ".md"), "w",
                  encoding="utf-8") as fh:
            fh.write(PROVENANCE_TMPL.format(model=model, repo=REPO_URL,
                                            rev=REVISION, stamp=stamp))

        total = 0
        nfiles = 0
        for base, _dirs, files in os.walk(dest):
            for f in files:
                total += os.path.getsize(os.path.join(base, f))
                nfiles += 1
        print("FETCHED %s -> %s  (%d files, %.2f MB)"
              % (model, os.path.relpath(dest, ROOT), nfiles,
                 total / 1e6))
        return dest
    finally:
        if keep_tmp:
            print("temp clone kept at: %s" % tmp)
        else:
            shutil.rmtree(tmp, ignore_errors=True)


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--model", default="unitree_h1",
                   help="Menagerie model directory (default unitree_h1)")
    p.add_argument("--keep-tmp", action="store_true",
                   help="keep the temporary clone directory")
    args = p.parse_args(argv)
    try:
        fetch(args.model, keep_tmp=args.keep_tmp)
    except Exception as exc:  # noqa: BLE001
        print("FETCH FAILED: %s" % exc, file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
