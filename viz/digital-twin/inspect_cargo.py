import bpy
import bmesh
import math
from mathutils import Vector
bpy.ops.wm.open_mainfile(filepath=r'C:\Users\DANHA\DigitalTwinWorkspace\cargo-only\cargo.blend')
axis = Vector((-math.sin(math.radians(-17.5)), math.cos(math.radians(-17.5)), 0))
origin = Vector((-0.35, -3.16, 0))
for obj in bpy.context.scene.objects:
    if obj.type != 'MESH':
        continue
    mesh = bmesh.new()
    mesh.from_mesh(obj.data)
    remaining = set(mesh.verts)
    groups = []
    while remaining:
        seed = remaining.pop()
        group, queue = [seed], [seed]
        while queue:
            vertex = queue.pop()
            for edge in vertex.link_edges:
                neighbor = edge.other_vert(vertex)
                if neighbor in remaining:
                    remaining.remove(neighbor)
                    queue.append(neighbor)
                    group.append(neighbor)
        if len(group) > 30:
            groups.append((len(group), min(v.co.z for v in group), max(v.co.z for v in group),
                           min((v.co-origin).dot(axis) for v in group), max((v.co-origin).dot(axis) for v in group)))
    print('COMPONENTS count,zmin,zmax,depthmin,depthmax', sorted(groups, reverse=True))
    mesh.free()
