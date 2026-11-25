"""Describe the running URDF for the browser."""

import os
from pathlib import Path
from xml.etree import ElementTree as ET

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory


DESCRIPTION_FOLDER = Path("description/DM")
MESH_URI = "package://rebotarm_bringup/description/DM/"


def find_vendor_model():
    configured = os.environ.get("B601_VENDOR_DESCRIPTION")
    if configured:
        return Path(configured) / DESCRIPTION_FOLDER
    try:
        installed = Path(get_package_share_directory("rebotarm_bringup")) / DESCRIPTION_FOLDER
        if installed.is_dir():
            return installed
    except PackageNotFoundError:
        pass
    for root in Path(__file__).resolve().parents:
        reference = root / "rebotarm_ros2/src/rebotarm_bringup" / DESCRIPTION_FOLDER
        if reference.is_dir():
            return reference
    return None


def vector(value):
    return [float(component) for component in value.split()]


def origin(element):
    frame = element.find("origin")
    attributes = {} if frame is None else frame.attrib
    return {key: vector(attributes.get(key, "0 0 0")) for key in ("xyz", "rpy")}


def visual_color(visual, colors):
    default = [0.65, 0.68, 0.72]
    material = visual.find("material")
    if material is None:
        return default
    local = material.find("color")
    if local is not None:
        return vector(local.attrib["rgba"])[:3]
    return colors.get(material.get("name"), default)


def mesh_geometry(geometry, vendor, assets, mesh_prefix):
    uri = geometry.attrib["filename"]
    mesh_root = (vendor / "meshes").resolve()
    path = (vendor / uri.removeprefix(MESH_URI)).resolve()
    if not (uri.startswith(MESH_URI) and path.is_relative_to(mesh_root)
            and path.suffix.lower() == ".stl" and path.is_file()):
        raise ValueError("Unavailable B601 visual mesh")
    key = f"{mesh_prefix}/{len(assets)}-{path.stat().st_mtime_ns}.stl"
    assets[key] = path
    return {"kind": "mesh", "url": key, "scale": vector(geometry.get("scale", "1 1 1"))}


def link_visuals(robot, vendor=None, assets=None, mesh_prefix="/robot-mesh"):
    colors = {material.attrib["name"]: vector(color.attrib["rgba"])[:3]
              for material in robot.findall("material")
              if (color := material.find("color")) is not None}
    links = {}
    for link in robot.findall("link"):
        visuals = []
        for visual in link.findall("visual"):
            geometry = visual.find("geometry/*")
            if geometry is None:
                continue
            if geometry.tag == "mesh" and vendor is not None:
                shape = mesh_geometry(geometry, vendor, assets, mesh_prefix)
            elif geometry.tag == "box":
                shape = {"kind": "box", "size": vector(geometry.attrib["size"])}
            elif geometry.tag in ("sphere", "cylinder"):
                shape = {"kind": geometry.tag, **{
                    name: float(value) for name, value in geometry.attrib.items()}}
            else:
                continue
            visuals.append({**origin(visual), "color": visual_color(visual, colors), **shape})
        links[link.attrib["name"]] = visuals
    return links


def joint_model(joint):
    axis = joint.find("axis")
    values = {
        "name": joint.attrib["name"],
        "type": joint.attrib["type"],
        "parent": joint.find("parent").attrib["link"],
        "child": joint.find("child").attrib["link"],
        "axis": vector("1 0 0" if axis is None else axis.attrib["xyz"]),
        **origin(joint),
    }
    mimic = joint.find("mimic")
    if mimic is not None:
        values["mimic"] = {"joint": mimic.attrib["joint"],
                           "multiplier": float(mimic.get("multiplier", 1)),
                           "offset": float(mimic.get("offset", 0))}
    return values


def build_model(description, vendor, mesh_prefix="/robot-mesh"):
    robot = ET.fromstring(description)
    visuals = link_visuals(robot)
    assets = {}
    asset_source = "urdf"
    if vendor is not None:
        try:
            vendor_robot = ET.parse(vendor / "urdf/ReBot_Arm_DM.urdf").getroot()
            vendor_visuals = link_visuals(vendor_robot, vendor, assets, mesh_prefix)
            if not all(vendor_visuals.get(link) for link, parts in visuals.items() if parts):
                raise ValueError("Incomplete B601 visual model")
            visuals = {name: vendor_visuals.get(name, parts) for name, parts in visuals.items()}
            asset_source = "vendor"
        except (OSError, ValueError, KeyError, ET.ParseError):
            assets.clear()
    return {
        "asset_source": asset_source,
        "links": visuals,
        "joints": [joint_model(joint) for joint in robot.findall("joint")],
    }, assets
