#!/usr/bin/env python3
"""
SPIRE data storage location.

All test and flight data go to the USB drive mounted at DATA_ROOT
(/mnt/spire_data, ext4, mounted by UUID with `nofail` in /etc/fstab).
Scripts call data_dir() to get their output folder; if the drive is not
mounted they stop with a clear message instead of silently writing to the
SD card. For development without the drive, pass allow_local=True (the
scripts expose this as --allow-local) to fall back to ./data/<sub>.
"""

import logging
import os
import sys
import tempfile

DATA_ROOT = "/mnt/spire_data"
LOCAL_ROOT = "data"

log = logging.getLogger("spire.storage")


def _writable(path):
    try:
        os.makedirs(path, exist_ok=True)
        with tempfile.NamedTemporaryFile(dir=path, prefix=".write_test_"):
            pass
        return True
    except OSError:
        return False


def data_dir(sub, root=DATA_ROOT, allow_local=False):
    """Return <root>/<sub>, created and checked writable.

    Exits the program if the USB drive is not mounted (unless allow_local),
    or if the folder cannot be written.
    """
    if os.path.ismount(root):
        path = os.path.join(root, sub)
    elif allow_local:
        path = os.path.join(LOCAL_ROOT, sub)
        log.warning(f"USB data drive not mounted at {root}; "
                    f"writing to local {path} (--allow-local)")
    else:
        sys.exit(f"USB data drive not mounted at {root}. Mount it "
                 f"(sudo mount -a) or pass --allow-local for a dev run.")
    if not _writable(path):
        sys.exit(f"Cannot write to {path}")
    return path


def find_sessions(sub="bench", root=DATA_ROOT):
    """Session folders on the USB drive and in ./data, oldest first."""
    import glob
    found = []
    for base in (os.path.join(root, sub), os.path.join(LOCAL_ROOT, sub)):
        found += glob.glob(os.path.join(base, "session_*"))
    return sorted(found, key=os.path.basename)
