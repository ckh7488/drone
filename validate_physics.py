"""Run CAD clearance and four-rotor rigid-body checks in MuJoCo on a CPU."""

import argparse
import hashlib
import json
import math
from itertools import combinations
from pathlib import Path
from xml.etree import ElementTree as ET

import mujoco as mj
import numpy as np


ROOT = Path(__file__).resolve().parent
MM = 0.001
G = 9.80665
DT = 0.001
DECORATION = ("HeadingMark", "HoleMarker", "FrontPad", "RearPad", "RatingLabel")
ELECTRONICS = {"fc", "camera", "wifi", "flow", "compass", "receiver", "battery"}
RING_SEGMENTS = 52


def vec(values):
    return " ".join(f"{value:.12g}" for value in values)


def rotation(angles_deg):
    quaternion = np.empty(4)
    mj.mju_euler2Quat(quaternion, np.deg2rad(angles_deg), "XYZ")
    matrix = np.empty(9)
    mj.mju_quat2Mat(matrix, quaternion)
    return quaternion, matrix.reshape(3, 3)


def solids(assembly):
    for part in assembly["parts"]:
        group_quat, group_axes = rotation([0, 0, part["yaw_deg"]])
        for shape in part["shapes"]:
            if shape["name"].startswith(DECORATION):
                continue
            local_quat, _ = rotation(shape["rotation"])
            quat = np.empty(4)
            mj.mju_mulQuat(quat, group_quat, local_quat)
            yield dict(shape, part=part["id"], quat=quat,
                       center=np.asarray(part["origin"]) + group_axes @ shape["center"])


def append_geom(parent, geom_id, shape, *, contact=False):
    size = (np.asarray(shape["size"]) / 2 if shape["kind"] == "box"
            else np.array([shape["radius"], shape["height"] / 2]))
    ET.SubElement(parent, "geom", name=geom_id, type=shape["kind"],
                  pos=vec(np.asarray(shape["center"]) * MM), quat=vec(shape["quat"]),
                  size=vec(size * MM), contype=str(int(contact)), conaffinity=str(int(contact)))


def solid_id(shape):
    return shape["part"] + "/" + shape["name"]


def guard_walls(shape):
    half_angle = math.pi / RING_SEGMENTS
    inner = shape["inner"] * math.cos(half_angle)
    outer = shape["outer"]
    for index in range(RING_SEGMENTS):
        angle = (2 * index + 1) * half_angle
        quat, axes = rotation([0, 0, math.degrees(angle)])
        yield dict(shape, name=f"{shape['name']}.{index:02}", kind="box",
                   size=[outer - inner, 2 * outer * math.sin(half_angle), shape["height"]],
                   quat=quat, center=shape["center"] + axes[:, 0] * ((outer + inner) / 2))


def rotor_sweeps(assembly, shapes):
    by_id = {solid_id(shape): shape for shape in shapes}
    radius = assembly["propeller_diameter_mm"] / 2
    for index in range(1, 5):
        blade = by_id[f"props/Blade{index}"]
        yield dict(blade, name=f"Sweep{index}", kind="cylinder", radius=radius,
                   height=blade["size"][2])
        yield by_id[f"props/Hub{index}"]


def minimum_distance(model, data, pairs):
    limit = 0.2
    best = limit
    result = None
    for a, b in pairs:
        distance = mj.mj_geomDistance(model, data, model.geom(a).id, model.geom(b).id, limit, None)
        if distance < best:
            best = distance
            result = {"a": a, "b": b, "distance_mm": distance / MM}
    if result is None:
        raise ValueError("No comparable pair within the clearance search.")
    return result


def check_clearances(assembly, shapes):
    xml = ET.Element("mujoco")
    ET.SubElement(xml, "option", ccd_tolerance="1e-9", ccd_iterations="100")
    world = ET.SubElement(xml, "worldbody")
    groups = {}
    for shape in shapes:
        if shape["part"] == "props":
            continue
        for geom in guard_walls(shape) if shape["kind"] == "ring" else (shape,):
            name = solid_id(geom)
            append_geom(world, name, geom)
            groups.setdefault(shape["part"], []).append(name)
    rotors = list(rotor_sweeps(assembly, shapes))
    for shape in rotors:
        append_geom(world, solid_id(shape), shape)
    model = mj.MjModel.from_xml_string(ET.tostring(xml, encoding="unicode"))
    data = mj.MjData(model)
    mj.mj_forward(model, data)

    def group_pairs(first, second):
        return ((a, b) for a in groups[first] for b in groups[second])

    layout = {}
    for first, second in combinations(groups, 2):
        if not (first in ELECTRONICS or second in ELECTRONICS):
            continue
        key = "--".join(sorted([first, second]))
        layout[key] = minimum_distance(model, data, group_pairs(first, second))

    rotor_clearances = {}
    for name in groups:
        pairs = ((solid_id(shape), target) for shape in rotors for target in groups[name]
                 if target != f"motors/Shaft{shape['name'][-1]}" and "/Guard" not in target)
        rotor_clearances[name] = minimum_distance(model, data, pairs)
    flow_guard = minimum_distance(model, data, (
        (guard, sensor) for guard in groups["frame"] if "/Guard" in guard
        for sensor in groups["flow"]))

    radius = assembly["propeller_diameter_mm"] / 2
    centers = np.asarray(assembly["rotor_centers_mm"])
    guards = {s["name"]: s for s in shapes if s["kind"] == "ring"}
    guard_clearance = []
    for index, point in enumerate(centers, 1):
        guard = guards[f"Guard{index}"]
        sweep = rotors[(index - 1) * 2]
        if abs(guard["center"][2] - sweep["center"][2]) + sweep["height"] / 2 >= guard["height"] / 2:
            raise ValueError("Rotor sweep is outside its modeled guard.")
        offset = np.linalg.norm(point - guard["center"][:2])
        guard_clearance.append(guard["inner"] * math.cos(math.pi / RING_SEGMENTS) - radius - offset)
    separation = min(np.linalg.norm(a - b) - 2 * radius for a, b in combinations(centers, 2))
    neighbor_guard = min(np.linalg.norm(point - guard["center"][:2]) - radius - guard["outer"]
                         for index, point in enumerate(centers, 1)
                         for name, guard in guards.items() if name != f"Guard{index}")
    return {"layout_pairs": layout,
            "layout_interference": sorted(
                (entry for entry in layout.values() if entry["distance_mm"] < -0.025),
                key=lambda entry: entry["distance_mm"]),
            "rotor_clearances_mm": rotor_clearances,
            "flow_to_guard": flow_guard,
            "rotor_to_neighbor_rotor_mm": separation,
            "rotor_to_neighbor_guard_mm": neighbor_guard,
            "rotor_to_own_guard_mm": guard_clearance,
            "guard_margin_with_1mm_lateral_offset_mm": min(guard_clearance) - 1}


def volume_inertia(shape):
    """Return volume and the axis moments per unit mass in mm units."""
    if shape["kind"] == "box":
        x, y, z = shape["size"]
        return x*y*z, np.array([y*y+z*z, x*x+z*z, x*x+y*y]) / 12
    outer = shape.get("outer", shape.get("radius"))
    inner = shape.get("inner", 0)
    height = shape["height"]
    r2 = outer**2 + inner**2
    cross = r2 / 4 + height**2 / 12
    return math.pi * (outer**2 - inner**2) * height, np.array([cross, cross, r2/2])


def mass_properties(assembly, shapes):
    elements = []
    for part in assembly["parts"]:
        group = [shape for shape in shapes if shape["part"] == part["id"]]
        physical = [volume_inertia(shape) for shape in group]
        total_volume = sum(volume for volume, _ in physical)
        for shape, (volume, moments) in zip(group, physical):
            axes = np.empty(9)
            mj.mju_quat2Mat(axes, shape["quat"])
            axes = axes.reshape(3, 3)
            mass = part["mass_g"] * volume / total_volume / 1000
            tensor = axes @ np.diag(moments) @ axes.T
            elements.append((mass, shape["center"], tensor))
    elements.append((assembly["small_parts_estimate_g"] / 1000,
                     np.array([0, 0, 24]), np.eye(3) * 20))
    mass = sum(weight for weight, _, _ in elements)
    center = sum(weight * point for weight, point, _ in elements) / mass
    inertia = np.zeros((3, 3))
    for weight, point, tensor in elements:
        offset = point - center
        inertia += weight * (tensor + np.eye(3) * (offset @ offset) - np.outer(offset, offset))
    return mass, center, inertia * MM**2


def contact_model(shapes, mass, center, tensor):
    xml = ET.Element("mujoco")
    ET.SubElement(xml, "option", timestep=str(DT), gravity=vec([0, 0, -G]),
                  solver="Newton", iterations="60", tolerance="1e-12")
    world = ET.SubElement(xml, "worldbody")
    ET.SubElement(world, "geom", name="floor", type="plane", size="2 2 .01")
    drone = ET.SubElement(world, "body", name="drone", pos="0 0 .002")
    ET.SubElement(drone, "freejoint")
    xx, yy, zz = tensor.diagonal()
    ET.SubElement(drone, "inertial", pos=vec(center * MM), mass=f"{mass:.12g}",
                  fullinertia=vec([xx, yy, zz, tensor[0, 1], tensor[0, 2], tensor[1, 2]]))
    for shape in shapes:
        foot = shape["part"] == "frame" and shape["name"].startswith("LandingFoot")
        if foot or shape["part"] == "flow":
            append_geom(drone, solid_id(shape), shape, contact=foot)
    return mj.MjModel.from_xml_string(ET.tostring(xml, encoding="unicode"))


def loaded_feet(model, data):
    names = set()
    wrench = np.empty(6)
    for index in range(data.ncon):
        contact = data.contact[index]
        mj.mj_contactForce(model, data, index, wrench)
        if wrench[0] > 1e-6:
            names.update(model.geom(i).name for i in contact.geom if i != model.geom("floor").id)
    return sorted(names)


def check_motion(assembly, shapes):
    mass, center, tensor = mass_properties(assembly, shapes)
    model = contact_model(shapes, mass, center, tensor)
    data = mj.MjData(model)
    for _ in range(round(0.7 / DT)):
        mj.mj_step(model, data)
    mj.mj_forward(model, data)
    settled_qpos, settled_qvel = data.qpos.copy(), data.qvel.copy()
    settled_feet = loaded_feet(model, data)
    body = model.body("drone").id
    axes = data.xmat[body].reshape(3, 3)
    settled_tilt = math.degrees(math.acos(float(np.clip(axes[2, 2], -1, 1))))
    sensor_gap = minimum_distance(model, data, (
        ("floor", solid_id(shape)) for shape in shapes if shape["part"] == "flow"))
    centers = np.asarray(assembly["rotor_centers_mm"])
    motor_points = np.column_stack((centers, np.full(4, assembly["rotor_plane_mm"]))) * MM
    x, y = centers.T
    mix = np.array([[1, 1, 1, 1], x, y, [1, -1, -1, 1]])
    trim = np.linalg.solve(mix, mass * G * np.array([1, center[0], center[1], 0]))
    runs = []
    for scale in (0.9, 1.0, 1.1):
        mj.mj_resetData(model, data)
        data.qpos[:] = settled_qpos
        data.qvel[:] = settled_qvel
        mj.mj_forward(model, data)
        airborne_steps = 0
        highest = 0.0
        for _ in range(round(0.5 / DT)):
            axes = data.xmat[body].reshape(3, 3)
            axis_up = axes[:, 2]
            data.qfrc_applied[:] = 0
            for local_point, thrust in zip(motor_points, trim * scale):
                point = data.xpos[body] + axes @ local_point
                mj.mj_applyFT(model, data, axis_up * thrust, np.zeros(3), point, body,
                              data.qfrc_applied)
            mj.mj_step(model, data)
            rise = data.qpos[2] - settled_qpos[2]
            highest = max(highest, float(rise))
            airborne_steps += int(rise > 0.001 and data.ncon == 0)
        runs.append({"assumed_thrust_weight_ratio": scale,
                     "assumed_total_thrust_gf": scale * mass * 1000,
                     "rise_after_0_5s_mm": (data.qpos[2] - settled_qpos[2]) / MM,
                     "max_rise_mm": highest / MM, "airborne_s": airborne_steps * DT})
    return {"mass_estimate_g": mass * 1000, "com_estimate_mm": center.tolist(),
            "inertia_estimate_kg_m2": tensor.tolist(),
            "principal_inertia_kg_m2": model.body_inertia[body].tolist(),
            "principal_axes_quat_wxyz": model.body_iquat[body].tolist(),
            "settled_foot_contacts": settled_feet, "settled_tilt_degrees": settled_tilt,
            "settled_floor_to_sensor_mm": sensor_gap,
            "rangefinder_min_mm_from_vendor": 20, "optical_flow_min_mm_from_vendor": 80,
            "hover_trim_gf_per_rotor": (trim * 1000 / G).tolist(),
            "vertical_test_synthetic_thrust": runs}


def run(assembly):
    shapes = list(solids(assembly))
    return {"clearance": check_clearances(assembly, shapes),
            "motion": check_motion(assembly, shapes)}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--assembly", type=Path, default=ROOT / "assembly.json")
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    assembly_bytes = args.assembly.read_bytes()
    report = {"engine": f"MuJoCo {mj.__version__}", "timestep_s": DT,
              "assembly_sha256": hashlib.sha256(assembly_bytes).hexdigest(),
              "geometry_basis": "millimeter layout envelopes, partial vendor dimensions",
              "thrust_basis": "synthetic multiples of weight, installed thrust unavailable",
              "screening_overlap_threshold_mm": 0.025,
              "current": run(json.loads(assembly_bytes))}
    payload = json.dumps(report, indent=2, ensure_ascii=False, allow_nan=False) + "\n"
    if args.output:
        args.output.write_text(payload, encoding="utf-8")
    else:
        print(payload, end="")


if __name__ == "__main__":
    main()
