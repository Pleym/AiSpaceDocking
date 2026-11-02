"""Renders the docking video views with Blender EEVEE: onboard camera + third-person chase camera.

    blender -b your_scene.blend --python render_video.py -- video_trajectory_<mode>.csv OUT_DIR [step] [start_s] [end_s]

step: render one trajectory row out of `step` (1 = 30 fps, 2 = 15 fps, frames are held when composing).
Outputs OUT_DIR/onboard/NNNNN.jpg (540 x 540, same field of view as the CNN camera) and
OUT_DIR/exterior/NNNNN.jpg (1920 x 1080), NNNNN = trajectory row.

Scene requirements (set the names with environment variables if yours differ):
  DOCKING_PORT_OBJECT  mesh whose origin/axes define the port frame (+Z out of the port)  [DockingPort]
  DOCKING_CAMERA       perspective camera whose lens is copied for the onboard view       [scene camera]
  ORION_STL            NASA Orion capsule STL, downloaded from
                       https://science.nasa.gov/3d-resources/orion-capsule/  ("no fbc" version)
"""
import csv
import math
import os
import sys
import time
from pathlib import Path

import bpy
from mathutils import Matrix, Quaternion, Vector

PORT_OBJECT = os.environ.get("DOCKING_PORT_OBJECT", "DockingPort")
CAMERA_OBJECT = os.environ.get("DOCKING_CAMERA", "")
ORION_STL = os.environ.get("ORION_STL", "assets/orion_capsule_no_fbc.stl")
# NASA STL: +Y towards the docking tip (Y = 8.03), heat shield at Y = -2.6; 16.3 units = 5.02 m.
ORION_SCALE = 5.02 / 16.3
ORION_TIP_Y, ORION_SHIELD_Y = 8.03, -2.6
CHASE_DIRECTION = Vector((0.9, 0.3, 0.15)).normalized()   # side view, slightly above


def rigid_frame(obj):
    """World matrix of the port without scale (the port frame must be rigid)."""
    location, rotation, _ = obj.matrix_world.decompose()
    return Matrix.LocRotScale(location, rotation, (1, 1, 1))


def material(name, color, metallic=0.0, roughness=0.5):
    mat = bpy.data.materials.new(name)
    mat.use_nodes = True
    bsdf = mat.node_tree.nodes["Principled BSDF"]
    bsdf.inputs["Base Color"].default_value = (*color, 1.0)
    bsdf.inputs["Metallic"].default_value = metallic
    bsdf.inputs["Roughness"].default_value = roughness
    return mat


def part(operator, name, parent, mat, location, rotation=(0, 0, 0), scale=(1, 1, 1), **options):
    operator(location=(0, 0, 0), **options)
    obj = bpy.context.active_object
    obj.name, obj.parent = name, parent
    obj.location, obj.rotation_euler, obj.scale = location, rotation, scale
    obj.data.materials.append(mat)
    for face in obj.data.polygons:
        face.use_smooth = abs(face.normal.z) < 0.9   # smooth sides, flat caps
    return obj


def orion_capsule(vehicle, hull, shield):
    """NASA capsule: docking tip just behind the onboard camera (z = 0.12 m), base towards +Z."""
    bpy.ops.wm.stl_import(filepath=str(Path(bpy.data.filepath).parent / ORION_STL))
    capsule = bpy.context.active_object
    capsule.name, capsule.parent = "Orion_Capsule", vehicle
    capsule.rotation_euler = (-math.pi / 2, 0, 0)          # STL +Y (tip) -> local -Z (forward)
    capsule.scale = (ORION_SCALE,) * 3
    capsule.location = (0, 0, 0.12 + ORION_TIP_Y * ORION_SCALE)
    capsule.data.materials.append(hull)
    capsule.data.materials.append(shield)
    for face in capsule.data.polygons:                     # heat shield: wide base facing -Y
        if face.center.y < ORION_SHIELD_Y + 0.6 and face.normal.y < -0.5:
            face.material_index = 1
    try:
        bpy.ops.object.shade_smooth_by_angle(angle=math.radians(35))
    except (AttributeError, RuntimeError):
        pass
    return (ORION_TIP_Y - ORION_SHIELD_Y) * ORION_SCALE + 0.12   # local z of the base (~3.4 m)


def build_vehicle(scene):
    """Local frame = Blender camera: -Z forward (towards the port), the vehicle sits behind (+Z)."""
    vehicle = bpy.data.objects.new("Vehicle", None)
    scene.collection.objects.link(vehicle)
    hull = material("Orion_Hull", (0.72, 0.73, 0.75), metallic=0.3, roughness=0.5)
    shield = material("Orion_Heat_Shield", (0.33, 0.21, 0.12), roughness=0.8)
    mli = material("Service_Module_MLI", (0.85, 0.6, 0.25), metallic=0.7, roughness=0.6)
    grey = material("Grey", (0.35, 0.36, 0.38), metallic=0.8, roughness=0.3)
    panel = material("Solar_Panel", (0.10, 0.18, 0.45), metallic=0.3, roughness=0.35)
    dark = material("Nozzle", (0.08, 0.08, 0.09), metallic=0.9, roughness=0.5)
    ops = bpy.ops.mesh
    base = orion_capsule(vehicle, hull, shield)
    # Generic service module at Orion scale (~4.1 m x 4 m), solar arrays in an X.
    part(ops.primitive_cylinder_add, "Service_Module", vehicle, mli, (0, 0, base + 2.05), radius=2.05, depth=4.0, vertices=64)
    part(ops.primitive_cone_add, "Nozzle", vehicle, dark, (0, 0, base + 4.45), radius1=0.3, radius2=0.8, depth=0.9, vertices=48)
    for k in range(4):
        angle = math.radians(45 + 90 * k)
        direction = Vector((math.cos(angle), math.sin(angle), 0))
        part(ops.primitive_cylinder_add, f"Boom_{k}", vehicle, grey, direction * 2.4 + Vector((0, 0, base + 3.0)),
             rotation=(math.pi / 2, 0, angle - math.pi / 2), radius=0.06, depth=0.7)
        part(ops.primitive_cube_add, f"Solar_Array_{k}", vehicle, panel, direction * 6.25 + Vector((0, 0, base + 3.0)),
             rotation=(0, 0, angle), scale=(3.5, 0.015, 1.0))
    # Docking floodlight: without it the vehicle's own shadow blacks out the target in the last centimetres.
    light = bpy.data.lights.new("Docking_Light", "SPOT")
    light.energy, light.spot_size, light.spot_blend = 6.0, math.radians(110), 0.5
    spot = bpy.data.objects.new("Docking_Light", light)
    scene.collection.objects.link(spot)
    spot.parent, spot.location = vehicle, (0, 0.15, 0.02)   # next to the camera, aiming -Z like it
    return vehicle


def look_at(position, target, up):
    forward = (target - position).normalized()
    right = forward.cross(up).normalized()
    top = right.cross(forward)
    rotation = Matrix((right, top, -forward)).transposed()  # columns: camera X, Y, Z
    return Matrix.LocRotScale(position, rotation.to_quaternion(), (1, 1, 1))


def main():
    args = sys.argv[sys.argv.index("--") + 1:]
    trajectory, out_dir = Path(args[0]), Path(args[1])
    step = int(args[2]) if len(args) > 2 else 1
    start = float(args[3]) if len(args) > 3 else 0.0
    end = float(args[4]) if len(args) > 4 else math.inf
    rows = list(csv.DictReader(trajectory.open()))
    for sub in ("onboard", "exterior"):
        (out_dir / sub).mkdir(parents=True, exist_ok=True)

    scene = bpy.context.scene
    deps = bpy.context.evaluated_depsgraph_get()
    port_world = rigid_frame(scene.objects[PORT_OBJECT].evaluated_get(deps))
    unit_m = scene.unit_settings.scale_length
    vehicle = build_vehicle(scene)
    source = scene.objects.get(CAMERA_OBJECT) or scene.camera
    onboard_data = source.evaluated_get(deps).data.copy()
    onboard_data.animation_data_clear()
    onboard_data.dof.use_dof = False
    onboard = bpy.data.objects.new("Onboard_Camera", onboard_data)
    scene.collection.objects.link(onboard)
    onboard.parent = vehicle                      # on the docking tip, same field of view as the CNN
    chase_data = bpy.data.cameras.new("Chase_Camera")
    chase_data.angle, chase_data.clip_end = math.radians(55), 500
    chase = bpy.data.objects.new("Chase_Camera", chase_data)
    scene.collection.objects.link(chase)

    render = scene.render
    render.engine = "BLENDER_EEVEE"
    scene.eevee.taa_render_samples = 16
    render.resolution_percentage = 100
    render.use_border = render.use_compositing = render.use_sequencer = render.film_transparent = False
    render.image_settings.file_format, render.image_settings.color_mode = "JPEG", "RGB"
    render.image_settings.quality = 90           # ~0.5 MB per frame pair instead of ~5 MB in PNG
    up = (port_world.to_3x3() @ Vector((0, 1, 0))).normalized()   # +Y port is "up" in the images

    clock, rendered = time.perf_counter(), 0
    for i in range(0, len(rows), step):
        row = rows[i]
        t = float(row["time_s"])
        if not start <= t <= end:
            continue
        p = Vector((float(row["x_m"]), float(row["y_m"]), float(row["z_m"])))
        q = Quaternion((float(row["qw"]), float(row["qx"]), float(row["qy"]), float(row["qz"])))
        blender_rotation = q.to_matrix() @ Matrix.Diagonal((1.0, -1.0, -1.0))   # vision camera -> Blender camera
        vehicle.matrix_world = port_world @ Matrix.LocRotScale(p / unit_m, blender_rotation.to_quaternion(), (1, 1, 1))
        # Aim between the port and the vehicle centre (~4 m behind its camera); back off with the gap.
        middle = Vector((0.5 * p.x, 0.5 * p.y, 0.5 * (p.z + 4.0)))
        distance = 18.0 + 1.2 * p.z
        chase.matrix_world = look_at(port_world @ ((middle + distance * CHASE_DIRECTION) / unit_m),
                                     port_world @ (middle / unit_m), up)
        for camera, folder, width, height in ((onboard, "onboard", 540, 540), (chase, "exterior", 1920, 1080)):
            scene.camera = camera
            render.resolution_x, render.resolution_y = width, height
            render.filepath = str(out_dir / folder / f"{i:05d}.jpg")
            bpy.ops.render.render(write_still=True)
        rendered += 1
        print(f"VIDEO {i} t={t:.1f}s z={p.z:.2f}m ({(time.perf_counter() - clock) / rendered:.2f} s/frame)", flush=True)


main()
