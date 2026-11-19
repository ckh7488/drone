"""Build the meter-scale assembly and scene from assembly.json using OpenUSD."""

import hashlib
import json
import math
from pathlib import Path

from pxr import Gf, Sdf, Usd, UsdGeom, UsdLux, UsdPhysics, UsdShade


ROOT = Path(__file__).resolve().parent
MM = 0.001


def vec3(mm):
    return Gf.Vec3d(*(n * MM for n in mm))


def ring_mesh(stage, path, shape):
    mesh = UsdGeom.Mesh.Define(stage, path)
    count = 52
    half = shape["height"] * MM / 2
    angles = [2 * math.pi * i / count for i in range(count)]
    points = [
        Gf.Vec3f(radius * math.cos(angle), radius * math.sin(angle), z)
        for radius, z in [
            (shape["outer"] * MM, -half), (shape["outer"] * MM, half),
            (shape["inner"] * MM, -half), (shape["inner"] * MM, half),
        ]
        for angle in angles
    ]
    faces = []
    for i in range(count):
        j = (i + 1) % count
        faces.extend([
            [i, j, count + j, count + i],
            [2 * count + i, 3 * count + i, 3 * count + j, 2 * count + j],
            [count + i, count + j, 3 * count + j, 3 * count + i],
            [i, 2 * count + i, 2 * count + j, j],
        ])
    mesh.CreatePointsAttr(points)
    mesh.CreateFaceVertexCountsAttr([4] * len(faces))
    mesh.CreateFaceVertexIndicesAttr([j for face in faces for j in face])
    mesh.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
    outer = shape["outer"] * MM
    mesh.CreateExtentAttr([Gf.Vec3f(-outer, -outer, -half), Gf.Vec3f(outer, outer, half)])
    return mesh


def shape_prim(stage, path, shape):
    if shape["kind"] == "box":
        geom = UsdGeom.Cube.Define(stage, path)
        geom.CreateSizeAttr(1.0)
    elif shape["kind"] == "cylinder":
        geom = UsdGeom.Cylinder.Define(stage, path)
        geom.CreateAxisAttr(UsdGeom.Tokens.z)
        geom.CreateRadiusAttr(shape["radius"] * MM)
        geom.CreateHeightAttr(shape["height"] * MM)
        radius = shape["radius"] * MM
        half = shape["height"] * MM / 2
        geom.CreateExtentAttr([Gf.Vec3f(-radius, -radius, -half), Gf.Vec3f(radius, radius, half)])
    else:
        geom = ring_mesh(stage, path, shape)
    geom.AddTranslateOp().Set(vec3(shape["center"]))
    if any(shape["rotation"]):
        geom.AddRotateXYZOp().Set(Gf.Vec3f(*shape["rotation"]))
    if shape["kind"] == "box":
        geom.AddScaleOp().Set(vec3(shape["size"]))
    return geom


def color_material(stage, color, materials):
    if color in materials:
        return materials[color]
    rgb = Gf.Vec3f(*(int(color[i:i + 2], 16) / 255 for i in (1, 3, 5)))
    path = "/World/Materials/C_" + color[1:]
    material = UsdShade.Material.Define(stage, path)
    shader = UsdShade.Shader.Define(stage, path + "/Surface")
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(rgb)
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.55)
    shader.CreateOutput("surface", Sdf.ValueTypeNames.Token)
    material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
    materials[color] = (material, rgb)
    return materials[color]


def bind(stage, geometry, color, materials):
    material, rgb = color_material(stage, color, materials)
    geometry.CreateDisplayColorAttr([rgb])
    UsdShade.MaterialBindingAPI.Apply(geometry.GetPrim()).Bind(material)


def camera(stage, path, eye, target, focal_length):
    cam = UsdGeom.Camera.Define(stage, path)
    cam.CreateFocalLengthAttr(focal_length)
    cam.CreateClippingRangeAttr(Gf.Vec2f(0.003, 10.0))
    matrix = Gf.Matrix4d().SetLookAt(eye, target, Gf.Vec3d(0, 0, 1)).GetInverse()
    cam.AddTransformOp().Set(matrix)
    return cam


def main():
    assembly_bytes = (ROOT / "assembly.json").read_bytes()
    assembly = json.loads(assembly_bytes)
    mass_g = sum(part["mass_g"] for part in assembly["parts"]) + assembly["small_parts_estimate_g"]
    stage = Usd.Stage.CreateInMemory()
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdPhysics.SetStageKilogramsPerUnit(stage, 1.0)
    stage.SetTimeCodesPerSecond(60)
    stage.SetEndTimeCode(120)
    world = UsdGeom.Xform.Define(stage, "/World").GetPrim()
    stage.SetDefaultPrim(world)
    world.SetDocumentation("FlyLens 85 assembly envelope. Vendor dimensions and preliminary placement.")
    UsdGeom.Scope.Define(stage, "/World/Materials")
    physics = UsdPhysics.Scene.Define(stage, "/World/Physics")
    physics.CreateGravityDirectionAttr(Gf.Vec3f(0, 0, -1))
    physics.CreateGravityMagnitudeAttr(9.80665)
    drone = UsdGeom.Xform.Define(stage, "/World/Drone")
    drone.AddTranslateOp().Set(Gf.Vec3d(0, 0, 0.001))
    drone.GetPrim().SetDocumentation("Layout model. The default mass is a parts estimate.")
    UsdPhysics.RigidBodyAPI.Apply(drone.GetPrim()).CreateRigidBodyEnabledAttr(True)
    mass_api = UsdPhysics.MassAPI.Apply(drone.GetPrim())
    mass_api.CreateMassAttr(mass_g / 1000)
    report_path = ROOT / "validation_report.json"
    if report_path.is_file():
        report = json.loads(report_path.read_bytes())
        if report["assembly_sha256"] == hashlib.sha256(assembly_bytes).hexdigest():
            motion = report["current"]["motion"]
            mass_api.CreateCenterOfMassAttr(Gf.Vec3f(*(pos * MM for pos in motion["com_estimate_mm"])))
            mass_api.CreateDiagonalInertiaAttr(Gf.Vec3f(*motion["principal_inertia_kg_m2"]))
            w, x, y, z = motion["principal_axes_quat_wxyz"]
            mass_api.CreatePrincipalAxesAttr(Gf.Quatf(w, Gf.Vec3f(x, y, z)))
    materials = {}

    for part in assembly["parts"]:
        group = UsdGeom.Xform.Define(stage, "/World/Drone/" + part["id"])
        group.GetPrim().SetDisplayName(part["label"])
        group.GetPrim().SetDocumentation(part["model"])
        group.AddTranslateOp().Set(vec3(part["origin"]))
        if part["yaw_deg"]:
            group.AddRotateZOp().Set(part["yaw_deg"])
        for shape in part["shapes"]:
            geometry = shape_prim(stage, group.GetPath().AppendChild(shape["name"]), shape)
            bind(stage, geometry, shape["color"], materials)
            if part["id"] == "frame" and shape["name"].startswith("LandingFoot"):
                UsdPhysics.CollisionAPI.Apply(geometry.GetPrim())

    ground = shape_prim(stage, "/World/Floor", {
        "kind": "box", "size": [4000, 4000, 40], "center": [0, 0, -20], "rotation": [0, 0, 0],
    })
    bind(stage, ground, "#a4b7bc", materials)
    UsdPhysics.CollisionAPI.Apply(ground.GetPrim())
    light = UsdLux.DomeLight.Define(stage, "/World/Ambient")
    light.CreateIntensityAttr(650)
    key = UsdLux.DistantLight.Define(stage, "/World/KeyLight")
    key.CreateIntensityAttr(950)
    key.AddRotateXYZOp().Set(Gf.Vec3f(-36, -28, 0))

    camera(stage, "/World/CameraOverview", Gf.Vec3d(.16, .21, .17), Gf.Vec3d(0, 0, .028), 28)
    camera(stage, "/World/CameraLift", Gf.Vec3d(.44, .52, .47), Gf.Vec3d(0, 0, .18), 23)
    lens = vec3(assembly["camera_mm"])
    front = camera(stage, "/World/Drone/FrontCamera", lens, lens + Gf.Vec3d(0, 1, 0), 10)
    front.GetPrim().SetDocumentation("Approximate lens placement and focal length for layout. Use measured intrinsics for training.")
    if not stage.GetRootLayer().Export(str(ROOT / "flylens85.usda")):
        raise RuntimeError("Could not export the USD assembly")


if __name__ == "__main__":
    main()
