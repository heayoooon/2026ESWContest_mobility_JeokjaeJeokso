"""Run with Blender --background --python prepare_scan.py -- source.obj output_dir."""
import json
import math
import sys
from pathlib import Path

import bpy
from mathutils import Vector

source, destination = sys.argv[sys.argv.index('--') + 1:]
destination = Path(destination)
destination.mkdir(parents=True, exist_ok=True)
bpy.ops.object.select_all(action='SELECT')
bpy.ops.object.delete(use_global=False)
if Path(source).suffix == '.blend':
    bpy.ops.wm.open_mainfile(filepath=source)
else:
    bpy.ops.wm.obj_import(filepath=source, forward_axis='Y', up_axis='Z')
meshes = [obj for obj in bpy.context.scene.objects if obj.type == 'MESH']
if not meshes:
    raise RuntimeError('No meshes imported')
points = [obj.matrix_world @ Vector(corner) for obj in meshes for corner in obj.bound_box]
low = Vector([min(point[i] for point in points) for i in range(3)])
high = Vector([max(point[i] for point in points) for i in range(3)])
center = (low + high) / 2
radius = (high - low).length / 2
stats = {'source': source, 'bounds_min': list(low), 'bounds_max': list(high),
         'vertices': sum(len(obj.data.vertices) for obj in meshes),
         'polygons': sum(len(obj.data.polygons) for obj in meshes),
         'scale_calibrated': False}
(destination / 'mesh-stats.json').write_text(json.dumps(stats, indent=2))
print(json.dumps(stats))

scene = bpy.context.scene
scene.render.engine = 'BLENDER_WORKBENCH'
scene.display.shading.light = 'STUDIO'
scene.display.shading.color_type = 'TEXTURE'
scene.display.shading.show_shadows = True
scene.display.shading.show_cavity = True
scene.display.shading.background_type = 'WORLD'
scene.world.color = (0.08, 0.09, 0.10)
scene.render.resolution_x = 1200
scene.render.resolution_y = 900
scene.render.resolution_percentage = 100
bpy.ops.object.camera_add()
camera = bpy.context.object
scene.camera = camera
camera.data.type = 'ORTHO'
camera.data.ortho_scale = radius * 2.4
camera.data.clip_end = radius * 100
for index, direction in enumerate([(1,-1,0.6), (-1,1,0.6), (0,0,1), (0,-1,0)]):
    camera.location = center + Vector(direction).normalized() * radius * 3
    camera.rotation_euler = (center - camera.location).to_track_quat('-Z', 'Y').to_euler()
    scene.render.filepath = str(destination / f'inspection-{index}.png')
    bpy.ops.render.render(write_still=True)
bpy.ops.wm.save_as_mainfile(filepath=str(destination / 'cart-inspection.blend'))
