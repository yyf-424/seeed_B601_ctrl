#!/usr/bin/env python3
"""Read source configuration for checkout tools and shell scripts."""

import argparse
import json
from decimal import Decimal
from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parent.parent
TRUTH = ROOT / "ros2_ws" / "src" / "b601_description" / "config" / "b601_project.yaml"


def load_project_config():
    return yaml.safe_load(TRUTH.read_text(encoding="utf-8"))


def expanded_position_range(measured, fraction):
    lower, upper = (Decimal(str(value)) for value in measured)
    margin = (upper - lower) * Decimal(str(fraction))
    return [float(lower - margin), float(upper + margin)]


def safety_position_limits(project):
    angle = project["safety"]["angle"]
    limits = {name: expanded_position_range(measured, angle["position_expansion_fraction"])
              for name, measured in angle["measured_position_ranges"].items()}
    robot = project["robot"]
    lower, upper = limits[robot["gripper_joint"]]
    limits[robot["mimic_joint"]] = [-upper, -lower]
    return limits


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("key", help="配置路径，例如 hardware.dm.baudrate")
    parser.add_argument("--pairs", action="store_true", help="映射输出为制表符分隔的键和值")
    parser.add_argument("--lines", action="store_true", help="列表按元素输出，每行一个")
    args = parser.parse_args()

    value = load_project_config()
    for name in args.key.split("."):
        value = value[name]
    if args.pairs:
        for key, item in value.items():
            rendered = item if isinstance(item, str) else json.dumps(item, ensure_ascii=False)
            print(f"{key}\t{rendered}")
    elif args.lines:
        for item in value:
            print(json.dumps(item, ensure_ascii=False) if not isinstance(item, str) else item)
    else:
        print(value if isinstance(value, str) else json.dumps(value, ensure_ascii=False))


if __name__ == "__main__":
    main()
