"""Resolve data paths from the workspace, independently of the launch directory."""

from pathlib import Path


def project_root(anchor):
    """Find the source workspace above a package's share directory or source file."""
    anchor = Path(anchor).resolve()
    for candidate in (anchor, *anchor.parents):
        if (candidate / 'src/realflight_modules/px4ctrl/package.xml').is_file():
            return candidate
    raise RuntimeError(
        f'Cannot locate the controller workspace above {anchor}; '
        'keep install/ inside the workspace or configure an absolute data path')


def project_path(value, anchor):
    """Absolute paths stay absolute; relative paths start at the project root."""
    path = Path(value).expanduser()
    if not path.is_absolute():
        path = project_root(anchor) / path
    return path.resolve()
