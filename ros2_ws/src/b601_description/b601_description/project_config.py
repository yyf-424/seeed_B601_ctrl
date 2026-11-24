"""Load the project configuration from the installed description package."""

from pathlib import Path

import yaml


def load_project_config(path=None):
    if path is None:
        from ament_index_python.packages import get_package_share_directory

        path = Path(get_package_share_directory("b601_description")) / "config" / "b601_project.yaml"
    with Path(path).open(encoding="utf-8") as stream:
        return yaml.safe_load(stream)
