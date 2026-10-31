"""MDL materials from Kit's core library (OmniPBR, OmniPBR_ClearCoat, OmniGlass) and the collected
NVIDIA Base materials, with sensible presets for vehicle paint, chrome, rubber, glass and lamps."""
from pathlib import Path

from pxr import Gf, Sdf, UsdShade

from forest_scene import base_mdl

C3, F, B, F2 = Sdf.ValueTypeNames.Color3f, Sdf.ValueTypeNames.Float, Sdf.ValueTypeNames.Bool, Sdf.ValueTypeNames.Float2


def mdl(stage, name, module, inputs=None, path=None):
    """Material whose surface is an MDL module (core library name such as 'OmniPBR.mdl' or a file path)."""
    mat = UsdShade.Material.Define(stage, path or '/World/Materials/' + name)
    shader = UsdShade.Shader.Define(stage, mat.GetPath().AppendChild('Shader'))
    shader.SetSourceAsset(Sdf.AssetPath(str(module).replace('\\', '/')), 'mdl')
    shader.SetSourceAssetSubIdentifier(Path(str(module)).stem, 'mdl')
    for key, (type_name, value) in (inputs or {}).items():
        shader.CreateInput(key, type_name).Set(value)
    out = shader.CreateOutput('out', Sdf.ValueTypeNames.Token)
    mat.CreateSurfaceOutput('mdl').ConnectToSource(out)
    mat.CreateSurfaceOutput().ConnectToSource(out)
    return mat


def car_paint(stage, name, colour, roughness=.32, metallic=.15, round_edges=.025):
    return mdl(stage, name, 'OmniPBR_ClearCoat.mdl', {
        'diffuse_color_constant': (C3, Gf.Vec3f(*colour)), 'reflection_roughness_constant': (F, roughness),
        'metallic_constant': (F, metallic), 'specular_level': (F, .6), 'enable_clearcoat': (B, True),
        'clearcoat_weight': (F, 1.), 'clearcoat_reflection_roughness': (F, .04), 'clearcoat_ior': (F, 1.56),
        'round_edges_radius': (F, round_edges), 'round_edges_roundness': (F, 1.)})


def pbr(stage, name, colour, roughness=.5, metallic=0., specular=.5, round_edges=0.):
    inputs = {'diffuse_color_constant': (C3, Gf.Vec3f(*colour)), 'reflection_roughness_constant': (F, roughness),
              'metallic_constant': (F, metallic), 'specular_level': (F, specular)}
    if round_edges:
        inputs.update({'round_edges_radius': (F, round_edges), 'round_edges_roundness': (F, 1.)})
    return mdl(stage, name, 'OmniPBR.mdl', inputs)


def chrome(stage, name='Chrome', roughness=.12, tint=(.85, .86, .88)):
    return pbr(stage, name, tint, roughness, 1., .8, round_edges=.01)


def glass(stage, name='Glass', colour=(.30, .40, .48), opacity=.30, roughness=.04):
    """Window glass as a blended (translucent) UsdPreviewSurface: in the real-time RTX mode this shows the
    interior and reflections, whereas refractive OmniGlass turns a closed cab black (no light gets inside)."""
    mat = UsdShade.Material.Define(stage, '/World/Materials/' + name)
    shader = UsdShade.Shader.Define(stage, mat.GetPath().AppendChild('Shader'))
    shader.CreateIdAttr('UsdPreviewSurface')
    shader.CreateInput('diffuseColor', C3).Set(Gf.Vec3f(*colour))
    shader.CreateInput('roughness', F).Set(roughness)
    shader.CreateInput('metallic', F).Set(0.)
    shader.CreateInput('opacity', F).Set(opacity)
    shader.CreateInput('ior', F).Set(1.5)
    mat.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), 'surface')
    return mat


def refractive_glass(stage, name='GlassRefractive', colour=(.80, .88, .92), ior=1.5, roughness=.02, thin=True):
    """Physically refractive OmniGlass (for path-traced stills or open glass such as lamp covers)."""
    return mdl(stage, name, 'OmniGlass.mdl', {
        'glass_color': (C3, Gf.Vec3f(*colour)), 'glass_ior': (F, ior), 'frosting_roughness': (F, roughness),
        'thin_walled': (B, thin), 'depth': (F, .004)})


def translucent(stage, name, colour, opacity, roughness=.8):
    """Blended overlay (wear marks, shadows) using fractional cutout opacity."""
    mat = UsdShade.Material.Define(stage, '/World/Materials/' + name)
    shader = UsdShade.Shader.Define(stage, mat.GetPath().AppendChild('Shader'))
    shader.CreateIdAttr('UsdPreviewSurface')
    shader.CreateInput('diffuseColor', C3).Set(Gf.Vec3f(*colour))
    shader.CreateInput('roughness', F).Set(roughness)
    shader.CreateInput('opacity', F).Set(opacity)
    mat.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), 'surface')
    return mat


def emissive(stage, name, colour, intensity=20., base=(.6, .6, .6)):
    mat = mdl(stage, name, 'OmniPBR.mdl', {
        'diffuse_color_constant': (C3, Gf.Vec3f(*base)), 'reflection_roughness_constant': (F, .15),
        'enable_emission': (B, True), 'emissive_color': (C3, Gf.Vec3f(*colour)), 'emissive_intensity': (F, intensity)})
    shader = UsdShade.Shader(stage.GetPrimAtPath(mat.GetPath().AppendChild('Shader')))
    return mat, shader


def base_material(stage, name, group, mdl_name, tint=None, roughness=None, scale=None, extra=None):
    """One of the collected NVIDIA Base materials (assets/nvidia/materials) with optional overrides."""
    inputs = dict(extra or {})
    if tint is not None:
        inputs['diffuse_tint'] = (C3, Gf.Vec3f(*tint))
    if roughness is not None:
        inputs['reflection_roughness_constant'] = (F, roughness)
    if scale is not None:
        inputs['texture_scale'] = (F2, Gf.Vec2f(scale, scale))
    return mdl(stage, name, base_mdl(group, mdl_name), inputs)


def available(group, mdl_name):
    return base_mdl(group, mdl_name).exists()
