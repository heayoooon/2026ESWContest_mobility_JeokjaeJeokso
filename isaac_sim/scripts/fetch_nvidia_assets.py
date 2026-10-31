"""Download the NVIDIA Omniverse assets used by the realistic forest scene into assets/nvidia.

Run once (needs internet):
    python scripts\\fetch_nvidia_assets.py
Afterwards the forest twin loads everything from disk.
"""
import asyncio
import json
import sys
import urllib.request
from pathlib import Path

from isaacsim import SimulationApp

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / 'assets' / 'nvidia'
S3 = 'http://omniverse-content-production.s3-us-west-2.amazonaws.com'
PEOPLE = S3 + '/Assets/Isaac/4.2/Isaac/People'

BUNDLES = {
    'vegetation': [S3 + '/Assets/Vegetation/Trees/' + n + '.usd' for n in
                   ('Red_Oak', 'Sugar_Maple', 'White_Ash', 'Red_Maple', 'Yellow_Pine')] +
                  [S3 + '/Assets/Vegetation/Shrub/' + n + '.usd' for n in
                   ('Grass_Short_A', 'Grass_Short_B', 'Grass_Short_C', 'Switchgrass', 'Barberry')] +
                  [S3 + '/Assets/Vegetation/Debris/' + n + '.usd' for n in ('oakfall1', 'oakfall2', 'maplefall1')] +
                  [S3 + f'/Assets/Vegetation/Rocks/rock_small_{i:02d}.usda' for i in (1, 2, 3, 4, 5)],
    'people': [PEOPLE + '/Characters/male_adult_construction_03/male_adult_construction_03.usd',
               PEOPLE + '/Animations/stand_walk_loop_in_place.skelanim.usd',
               PEOPLE + '/Animations/stand_idle_loop.skelanim.usd'],
}
MATERIALS = [S3 + '/Materials/2023_1/Base/Natural/' + n + '.mdl' for n in
             ('Asphalt', 'Grass_Countryside', 'Soil_Rocky', 'Dirt', 'Mulch_Brown')] + \
            [S3 + '/Materials/2023_1/Base/Masonry/Concrete_Rough.mdl'] + \
            [S3 + '/Materials/2023_1/Base/Metals/' + n + '.mdl' for n in
             ('Steel_Carbon', 'Steel_Stainless', 'Aluminum_Cast', 'RustedMetal')] + \
            [S3 + '/Materials/2023_1/Base/Plastics/' + n + '.mdl' for n in ('Rubber_Textured', 'Plastic_ABS')] + \
            [S3 + '/Materials/2023_1/Base/Stone/Gravel.mdl']
SKY = S3 + '/Assets/Skies/Cloudy/kloofendal_48d_partly_cloudy_4k.hdr'
SKY_CLEAR = S3 + '/Assets/Skies/Clear/noon_grass_4k.hdr'

kit = Path(sys.executable).parent / 'Lib/site-packages/isaacsim/apps/isaac_twin_light.kit'
app = SimulationApp({'headless': True}, experience=str(kit))

from omni.isaac.core.utils.extensions import enable_extension  # noqa: E402

enable_extension('omni.kit.usd.collect')
app.update()

from omni.kit.usd.collect import Collector  # noqa: E402
from pxr import Sdf, Usd, UsdShade  # noqa: E402


def wrapper_stage(name, urls):
    """A local stage that references every remote asset so one collect pulls all dependencies."""
    path = OUT / '_staging' / f'{name}.usda'
    path.parent.mkdir(parents=True, exist_ok=True)
    stage = Usd.Stage.CreateNew(str(path))
    for i, url in enumerate(urls):
        prim = stage.DefinePrim(f'/Bundle/Item{i:02d}', 'Xform')
        prim.GetReferences().AddReference(url)
    stage.GetRootLayer().Save()
    return path


def material_stage(urls):
    path = OUT / '_staging' / 'materials.usda'
    path.parent.mkdir(parents=True, exist_ok=True)
    stage = Usd.Stage.CreateNew(str(path))
    for url in urls:
        name = Path(url).stem
        mat = UsdShade.Material.Define(stage, f'/Looks/{name}')
        shader = UsdShade.Shader.Define(stage, mat.GetPath().AppendChild('Shader'))
        shader.SetSourceAsset(Sdf.AssetPath(url), 'mdl')
        shader.SetSourceAssetSubIdentifier(name, 'mdl')
        mat.CreateSurfaceOutput('mdl').ConnectToSource(shader.ConnectableAPI(), 'out')
    stage.GetRootLayer().Save()
    return path


async def collect(src, dest):
    def progress(done, total):
        print(f'  {dest.name}: {done}/{total}', end='\r', flush=True)
    collector = Collector(str(src), str(dest), skip_existing=True)
    await collector.collect(progress, None)
    print(f'\n  collected -> {dest}', flush=True)


async def main():
    OUT.mkdir(parents=True, exist_ok=True)
    jobs = [] if '--materials-only' in sys.argv else [(wrapper_stage(name, urls), OUT / name) for name, urls in BUNDLES.items()]
    jobs.append((material_stage(MATERIALS), OUT / 'materials'))
    for src, dest in jobs:
        print('Collecting', src.name, flush=True)
        await collect(src, dest)
    for url, folder in ((SKY, 'sky'), (SKY_CLEAR, 'sky_clear')):
        sky = OUT / folder / Path(url).name
        if not sky.exists():
            sky.parent.mkdir(parents=True, exist_ok=True)
            print('Downloading', sky.name, flush=True)
            urllib.request.urlretrieve(url, sky)
    files = [p for p in OUT.rglob('*') if p.is_file()]
    size = sum(p.stat().st_size for p in files)
    summary = {'files': len(files), 'megabytes': round(size / 1e6, 1)}
    (OUT / 'fetch_summary.json').write_text(json.dumps(summary, indent=2), encoding='utf-8')
    print('FETCH_DONE', summary, flush=True)


task = asyncio.ensure_future(main())
while not task.done():
    app.update()
error = task.exception()
app.close()
if error:
    raise error
sys.exit(0)
