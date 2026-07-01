#!/usr/bin/env python3
"""Shared utility to resolve adam_assets paths at runtime.

Example usage (from a launch file or node):

    from get_asset_path import get_asset_path

    map_yaml = get_asset_path("maps_2d", "my_map.yaml")
    mesh_file = get_asset_path("meshes", "collision.stl")

The function searches:
  1. ROS2 installed share (via ament_index_python)
  2. Source tree fallback (common during development)
"""

import os
from pathlib import Path
from typing import Optional

try:
    from ament_index_python.packages import get_package_share_directory

    HAVE_AMENT = True
except ImportError:
    HAVE_AMENT = False

_PACKAGE_NAME = "adam_assets"


def _find_share_dir() -> Optional[Path]:
    """Return the share directory of adam_assets, or None if not found."""
    if HAVE_AMENT:
        try:
            return Path(get_package_share_directory(_PACKAGE_NAME))
        except (PackageNotFoundError, ValueError):
            pass

    # Source-tree fallback: walk up from this file's location or CWD
    for anchor in (Path(__file__).resolve(), Path.cwd()):
        for parent in [anchor] + list(anchor.parents):
            candidate = parent / "src" / "navigation_ai" / "adam_assets" / "share"
            if candidate.is_dir():
                return candidate
    return None


def get_asset_path(*subdirs: str) -> str:
    """Return the absolute filesystem path to an adam_assets resource.

    Arguments are joined as sub-paths inside the package share directory.

    Raises FileNotFoundError if the asset directory cannot be located or the
    resolved path does not exist.
    """
    share_dir = _find_share_dir()
    if share_dir is None:
        raise FileNotFoundError(
            f"Could not locate {_PACKAGE_NAME} share directory. "
            "Is the package built and sourced?"
        )
    path = share_dir.joinpath(*subdirs).resolve()
    if not path.exists():
        raise FileNotFoundError(f"adam_assets resource not found: {path}")
    return str(path)
