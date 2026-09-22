"""Crop the inspected RealityScan export to the cart footprint, then export Y-up."""
import json
import math
from pathlib import Path

import bmesh
import bpy
from mathutils import Vector

root = Path(r'C:\Users\DANHA\DigitalTwinWorkspace')
bpy.ops.object.select_all(action='SELECT')
bpy.ops.object.delete(use_global=False)
bpy.ops.wm.obj_import(filepath=str(root / 'reconstruction' / 'filtered.obj'), forward_axis='Y', up_axis='Z')
meshes = [obj for obj in bpy.context.scene.objects if obj.type == 'MESH']
# Bounds are in uncalibrated scan coordinates, selected from the top-view inspection.
center = Vector((-0.35, -3.16, 12.5))
angle = math.radians(-17.5)
axes = (Vector((math.cos(angle), math.sin(angle), 0)),
        Vector((-math.sin(angle), math.cos(angle), 0)), Vector((0, 0, 1)))
extents = (5.9, 9.8, 12.5)
for obj in meshes:
    bpy.context.view_layer.objects.active = obj
    obj.select_set(True)
    bpy.ops.object.transform_apply(location=True, rotation=True, scale=True)
    mesh = bmesh.new()
    mesh.from_mesh(obj.data)
    for axis, extent in zip(axes, extents):
        for sign in (-1, 1):
            normal = axis * sign
            bmesh.ops.bisect_plane(mesh, geom=list(mesh.verts) + list(mesh.edges) + list(mesh.faces),
                                  dist=0.00001, plane_co=center + normal * extent,
                                  plane_no=normal, clear_outer=True, clear_inner=False)
    # Remove low, upward-facing floor remnants without replacing missing cart parts.
    mesh.normal_update()
    floor_faces = [face for face in mesh.faces
                   if face.calc_center_median().z < 2.2 and abs(face.normal.z) > 0.65]
    bmesh.ops.delete(mesh, geom=floor_faces, context='FACES')
    loose = [vertex for vertex in mesh.verts if not vertex.link_faces]
    bmesh.ops.delete(mesh, geom=loose, context='VERTS')
    mesh.to_mesh(obj.data)
    mesh.free()
    obj.data.update()
    smooth = obj.modifiers.new('Reduce scan surface noise', 'SMOOTH')
    smooth.factor = 0.8
    smooth.iterations = 8
    bpy.ops.object.modifier_apply(modifier=smooth.name)
    for polygon in obj.data.polygons:
        polygon.use_smooth = True
    obj.select_set(False)

output = root / 'cart-only'
output.mkdir(exist_ok=True)
for obj in meshes:
    obj.select_set(True)
bpy.ops.wm.obj_export(filepath=str(output / 'cart.obj'), export_selected_objects=True,
                      forward_axis='NEGATIVE_Z', up_axis='Y', path_mode='COPY')
bpy.ops.wm.save_as_mainfile(filepath=str(output / 'cart.blend'))
stats = {'crop_center': list(center), 'crop_half_extents': extents,
         'crop_yaw_degrees': -17.5, 'scale_calibrated': False,
         'vertices': sum(len(obj.data.vertices) for obj in meshes),
         'polygons': sum(len(obj.data.polygons) for obj in meshes)}
(output / 'crop.json').write_text(json.dumps(stats, indent=2))
print(json.dumps(stats))
