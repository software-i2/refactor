#!/usr/bin/env python
# Copyright by BeeX [2026]

"""Capture pairs dropped into a folder by the camera side."""

from __future__ import print_function

import os

PLY = "_depth_scene.ply"
JSON = "_depth_poses.json"


def _mtime(path):
    try:
        return os.path.getmtime(path)
    except OSError:
        return None


def drop(path):
    try:
        os.remove(path)
    except OSError:
        pass


def newest(folder):
    found = {}
    for name in os.listdir(folder):
        if name.startswith("."):
            continue
        for suffix in (PLY, JSON):
            if name.endswith(suffix):
                found.setdefault(name[:-len(suffix)], {})[suffix] = os.path.join(folder, name)

    ready = []
    for fid, files in found.items():
        t = _mtime(files[JSON]) if len(files) == 2 else None
        if t is not None:
            ready.append((t, fid))
    if not ready:
        return None

    ready.sort()
    t, fid = ready[-1]
    stale = []
    for other, files in found.items():
        if other != fid:
            stale.extend(path for path in files.values() if (_mtime(path) or t) <= t)
    return fid, found[fid][PLY], found[fid][JSON], stale, len(ready) - 1
