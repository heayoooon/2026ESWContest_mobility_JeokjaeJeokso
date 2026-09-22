# Cart digital twin

Status: first cart-only textured reconstruction and Unity scene generated.
Surrounding people, furniture and most floor geometry are cropped away.
This is a visual scan, not a calibrated physics or live-sensor twin.

## Local outputs

Both versions are packaged in `C:\Users\DANHA\Downloads\DigitalTwin_20260920`:
`CartAndCargo` preserves the original, and `CargoOnly` removes the cart using
height/depth clipping and inspected connected-component selection. Each has
a viewer plus packed Blender, GLB and OBJ/MTL/PNG assets. Cargo-only remains
one scanned arrangement, with unobserved contact surfaces left incomplete.
The separate cargo Unity project is `C:\Users\DANHA\DigitalTwinWorkspace\UnityCargoOnly`.

- Unity project: `C:\Users\DANHA\DigitalTwinWorkspace\UnityCartTwin`
- Scene: `Assets/Scenes/CartTwin.unity` in that project
- Windows viewer: `UnityCartTwin/Builds/CartTwin.exe`
- Blender asset: `C:\Users\DANHA\DigitalTwinWorkspace\cart-only\cart.blend`
- Y-up OBJ and texture: `C:\Users\DANHA\DigitalTwinWorkspace\cart-only`
- RealityScan project: `C:\Users\DANHA\DigitalTwinWorkspace\reconstruction\cart-filtered.rsproj`
- Preview: `UnityCartTwin/Previews/cart.png`

Viewer: left-drag rotates, right-drag pans, wheel zooms, Home resets, Escape exits.
Keep the EXE next to its generated Data folder and DLLs.

## Observed quality

The initial largest component contained 30/79 images. Higher feature limits and
component rematching increased it to 46/79 (mean reprojection error 0.82 pixels).
The normal-quality mesh was filtered, simplified to approximately 300k faces,
textured at 4096 pixels, cropped in Blender and lightly smoothed. The delivered
crop contains 191626 polygons. Camera reprojection error is not a physical
measurement-accuracy guarantee.

The top handle connection and parts of the wheels/underside are incomplete.
Box surfaces retain some video reconstruction distortion. Missing geometry has
not been fabricated. Actual scale, mass, collisions and sensor data are unset.

## Source

Two local KakaoTalk videos, sampled at 2 FPS into 79 frames under
`C:\Users\DANHA\DigitalTwinWorkspace\frames`.
The videos show a cart with stacked cardboard boxes. Frames and generated scan
assets stay outside version control.

## Reconstruction

Run `reconstruct.ps1 -Stage Align` after RealityScan is installed. Inspect aligned
cameras and components before continuing. Moving people, hands, cart motion,
reflections and occlusions can degrade or split reconstruction. Do not assume
both videos depict an identical static scene until alignment is inspected.

Set a reconstruction region around the cart in RealityScan, save the project,
then run `reconstruct.ps1 -Stage Mesh`. Texture and export the inspected model
as `cart.obj` with its MTL and texture files. Retain an untouched high-resolution
export before mesh cleanup in Blender.

CLI reference: https://rshelp.capturingreality.com/en-US/appbasics/allcommands.htm

## Unity

`Unity/Assets` contains the source of the viewer/import helper. The generated
complete project is at the local output path above.
Use Unity 6000.0.84f1 with an activated license. Copy these Assets into a standard
3D project with the legacy Input Manager enabled (or Active Input Handling Both).
Put the OBJ, MTL and textures in `Assets/Scan`, then use
`Digital Twin > Build Scan Scene`. The generated scene frames the imported mesh;
left-drag orbits and the mouse wheel zooms. `TwinSceneBuilder.Complete` builds the
scene, renders its preview and builds the Windows player in one batch run.

`crop_cart.py` records the manually inspected crop in scan coordinates and
exports a Unity-compatible Y-up OBJ. `prepare_scan.py` renders inspection views
from an original Z-up RealityScan OBJ or a Blender file.

## Calibration required

Photogrammetry does not establish physical scale or box masses by itself.
Measure at least one visible dimension to calibrate scale. Physics, center of
mass and tipping analysis need measured cart dimensions, support points and
load masses. No physical accuracy or live sensor integration is claimed yet.
