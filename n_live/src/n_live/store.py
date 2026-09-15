#!/usr/bin/env python
# Copyright by BeeX [2026]

"""Built frames on disk: a folder each, a latest link, and only the newest kept."""

from __future__ import print_function

import os
import shutil

LATEST = "latest"
MARK = "frame.json"


def clear(out):
    for name in os.listdir(out):
        if name.startswith(".") and name.endswith(".part"):
            path = os.path.join(out, name)
            if os.path.islink(path) or not os.path.isdir(path):
                os.remove(path)
            else:
                shutil.rmtree(path, ignore_errors=True)


def stage(out, fid):
    path = os.path.join(out, ".%s.part" % fid)
    shutil.rmtree(path, ignore_errors=True)
    os.makedirs(path)
    return path


def commit(out, fid, staged):
    final = os.path.join(out, fid)
    shutil.rmtree(final, ignore_errors=True)
    os.rename(staged, final)
    link = os.path.join(out, ".%s.part" % LATEST)
    if os.path.lexists(link):
        os.remove(link)
    os.symlink(fid, link)
    os.rename(link, os.path.join(out, LATEST))
    return final


def keep(src, out):
    fid = os.path.basename(os.path.normpath(src))
    part = os.path.join(out, ".%s.part" % fid)
    shutil.rmtree(part, ignore_errors=True)
    shutil.copytree(src, part)
    return commit(out, fid, part)


def prune(out, keep):
    frames = []
    for name in os.listdir(out):
        path = os.path.join(out, name)
        mark = os.path.join(path, MARK)
        if name.startswith(".") or os.path.islink(path) or not os.path.isfile(mark):
            continue
        frames.append((os.path.getmtime(mark), name))
    frames.sort()
    for _t, name in frames[:max(0, len(frames) - keep)]:
        shutil.rmtree(os.path.join(out, name), ignore_errors=True)
