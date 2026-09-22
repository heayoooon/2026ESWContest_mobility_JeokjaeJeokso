"""Package the original scan and derive a separate cargo-only asset."""
import json
import math
import shutil
from pathlib import Path

import bmesh
import bpy
from mathutils import Vector

root = Path(r'C:\Users\DANHA\DigitalTwinWorkspace')
delivery = Path(r'C:\Users\DANHA\Downloads\DigitalTwin_20260920')
bpy.ops.wm.open_mainfile(filepath=str(root / 'cart-only/cart.blend'))
bpy.ops.file.pack_all()
bpy.ops.wm.save_as_mainfile(filepath=str(delivery / 'CartAndCargo/Model/cart.blend'))
bpy.ops.export_scene.gltf(filepath=str(delivery / 'CartAndCargo/Model/cart.glb'), export_format='GLB')

angle = math.radians(-17.5)
depth_axis = Vector((-math.sin(angle), math.cos(angle), 0))
center = Vector((-0.35, -3.16, 0))
min_depth = -7.1
min_height = 6.15
for obj in list(bpy.context.scene.objects):
    if obj.type != 'MESH':
        continue
    mesh = bmesh.new()
    mesh.from_mesh(obj.data)
    # The handle is behind the cargo; the deck and wheels are below it.
    for plane_point, normal in [
        (Vector((0, 0, min_height)), Vector((0, 0, -1))),
        (center + depth_axis * min_depth, -depth_axis),
    ]:
        bmesh.ops.bisect_plane(mesh, geom=list(mesh.verts) + list(mesh.edges) + list(mesh.faces),
                              dist=0.00001, plane_co=plane_point, plane_no=normal,
                              clear_outer=True, clear_inner=False)
    bmesh.ops.delete(mesh, geom=[v for v in mesh.verts if not v.link_faces], context='VERTS')
    remaining = set(mesh.verts)
    components = []
    while remaining:
        seed = remaining.pop()
        component, queue = [seed], [seed]
        while queue:
            vertex = queue.pop()
            for edge in vertex.link_edges:
                neighbor = edge.other_vert(vertex)
                if neighbor in remaining:
                    remaining.remove(neighbor)
                    component.append(neighbor)
                    queue.append(neighbor)
        components.append(component)
    # Inspection confirmed the cargo is the largest component; two smaller ones are the handle.
    largest = max(components, key=len)
    keep = set(largest)
    bmesh.ops.delete(mesh, geom=[v for v in mesh.verts if v not in keep], context='VERTS')
    mesh.to_mesh(obj.data)
    mesh.free()
    obj.data.update()
    obj.name = 'Cargo Only - Uncalibrated Scan'

output = root / 'cargo-only'
output.mkdir(exist_ok=True)
bpy.ops.file.pack_all()
bpy.ops.wm.save_as_mainfile(filepath=str(output / 'cargo.blend'))
for image in bpy.data.images:
    if image.type == 'IMAGE':
        image.filepath = str(root / 'cart-only' / 'filtered_u1_v1_diffuse.png')
bpy.ops.wm.obj_export(filepath=str(output / 'cargo.obj'), forward_axis='NEGATIVE_Z', up_axis='Y', path_mode='COPY')
shutil.copy2(root / 'cart-only/filtered_u1_v1_diffuse.png', output / 'filtered_u1_v1_diffuse.png')
bpy.ops.export_scene.gltf(filepath=str(output / 'cargo.glb'), export_format='GLB')
(output / 'extraction.json').write_text(json.dumps({
    'source': str(root / 'cart-only/cart.blend'), 'min_height': min_height,
    'min_depth': min_depth, 'scale_calibrated': False,
    'note': 'Cart removed. Hidden contact surfaces remain unobserved, not synthesized.'
}, indent=2))
