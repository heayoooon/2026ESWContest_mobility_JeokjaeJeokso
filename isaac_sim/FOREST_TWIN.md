# Forest Road Digital Twin - 2026-10-05 (truck version 2026-10-18)

Launch: `Start-ForestTwin.bat` (Isaac Sim 4.2.0, existing conda environment) - Hyundai Porter-class truck.
`Start-CartTwin.bat` runs the original hand-cart look (`--vehicle cart`); `Start-ForestTwin-Share.bat` streams.
The older `Start-IsaacTwin.bat` and original scripts are preserved.

## Truck twin - 2026-10-18

- Data: 14 recordings from the sensor Pi `~/scss_live/runs` (2026-10-14/15) copied to `data/live_runs/`.
  `scripts/prepare_live_runs.py` -> `live_data.json` (all CSV channels + team judgement unchanged) and
  `data/live_cache/<id>.npz` (per-frame D435i depth 424x240, colour JPEG, 1 cm height map, stats, camera calibration).
  Run titles are translated for the UI (`title_en`) and the Pi's Korean risk text is keyword-translated:
  [superseded 2026-11-01: the panel is Korean now, see below] omni.ui's glyph atlas had no Hangul, so the
  Korean originals stay only in the data, exports and this folder's file names.
- Vehicle (2026-10-18, second pass): procedural Hyundai XCIENT-class 8x4 cargo truck (`scripts/truck.py`:
  ~11.5 x 2.5 x 3.65 m, two steer axles, rear tandem with dual wheels, 520 mm wheels, flatbed 1.35 m high,
  red body, chrome grille, LED strips, high-roof sleeper cab). Decals come from the Hyundai logo image the user
  supplied (`assets/truck/hyundai_emblem_chrome.png`, `hyundai_wordmark_white.png`, derived by masking the navy
  pixels): emblem on the grille, tailgate and sleeper walls, HYUNDAI on the tailgate and cab sides, XCIENT on the
  sun visor, "L540 8x4" on the doors. Decal orientation: a quad's texture-right axis must be the viewer's right
  (+y for a face looking +x, -y for -x, -side*x for a side face). The earlier 1-ton Porter is `truck_porter.py`.
  The hand-cart's instrumented plate (`ForestScene.rig`) sits on the bed behind the cab, rotated so its N (front,
  belt c1) row faces the cab, drawn `RIG['scale']` = 3.2 times larger; cargo physics unchanged (kinematic plate
  collider + dynamic cargo body in the sandbox). The driver is the skinned character in a delivery outfit, posed
  by IK on the seat with hands on the steering wheel; the wheel and hands follow the yaw rate.
- Measured cargo surface (replay): `measured_surface.DepthSurface` rebuilds a textured mesh every camera frame
  from the depth image (decimated 2x -> 212x120 vertices) with the aligned colour JPEG; pixels without depth
  take the nearest cargo pixel's x/y at plate height (vertical sides). Camera grid axes: grid y -> rig +x (N->S),
  grid x -> rig +y; camera at the N end looking toward S (verified from COG/centroid correlation and box layout).
  The belts are re-fitted to the measured cargo top each frame.
- Motion (replay, toggle "Replay: move by recorded IMU"): speed = base speed (0 for runs that start at rest, else
  the cruise slider, 20 km/h default) + recorded speed change (imu1 ax integrated, bias from the first 0.5 s,
  linear drift removed); heading = recorded gz x "turn scale" (0.25 default: the hand cart spun a full circle in
  the sharp-turn runs, a truck cannot) + a lane-keeping correction; the road loops every 60 m (scenery is periodic).
  Recorded roll/pitch are accelerometer tilt (they track ax/ay at -0.8..-0.9 correlation), so the body tilts
  from acceleration (suspension model, "Body tilt deg per G") and the recorded angles are shown as data.
- Judgement: replay shows the Pi's 4-level results (overall, load, COG zone, cargo shift, belts N/S) with colours,
  the risk text, belt colours, load-cell bars (red over 5 kg), COG marker (height from h_y) and a roof beacon.
  The sandbox applies the same rules to estimated readings (team/analysis thresholds; cargo shift not estimated).
- Road: `scripts/road_path.py` is a closed four-lane loop (14 m asphalt + 1.5 m gravel shoulders, double
  yellow centre line, dashed lane lines, delineator posts): a 200 x 120 m rounded rectangle (40 m corners) with
  an S-bend on the top straight, 573 m per lap, driven counter-clockwise in the outer right lane. The vehicle
  moves in road coordinates (s along the loop, d lateral, psi relative heading): the recorded IMU yaw (scaled)
  plus lane keeping steer psi, the loop provides the curves, and the body roll follows the simulated lateral
  acceleration. Terrain, tree rows, ground cover and litter are laid out along the loop by distance to the
  centreline (plus a sparse scatter inside/outside). Camera presets are relative to the heading and placed
  over the road so the first view is not inside the roadside trees.
- Plate scale on the truck: `truck.RIG['scale']` (3.2) enlarges the drawn plate, belts, cells, cargo surface and
  the sandbox cargo body to suit the bed; `rig_to_cart` / `cart_to_rig` apply and remove it, so every load,
  COG, belt and judgement computation still uses the hand cart's real 250 x 392 mm geometry and masses.
- Wheels roll about the axle (USD rotateXYZ applies X, Y, Z about the parent axes: X=90 tilt, Y=roll,
  Z=steer for the front wheels); the earlier version rotated them about the vertical axis.
- Realism pass (2026-10-18, third pass): `truck_geom.py` builds filleted boxes (`rounded_box`), lathe solids
  (tyres with modelled tread blocks and sidewall profile, chrome rims and hubs, fuel tank, poles) and swept
  profiles (W-beam guardrails); `materials_pbr.py` wraps Kit's core MDL library (OmniPBR_ClearCoat car paint
  with round-edge shading, chrome, OmniPBR emissive lamps) and the collected NVIDIA Base materials (textured
  rubber, carbon/stainless steel). Window glass stays a blended UsdPreviewSurface: refractive OmniGlass on a
  closed cab rendered black in the real-time mode. Road furniture (`ForestScene.road_furniture`): darker wheel
  tracks in every lane (opaque asphalt strips - translucent overlays reflected the sky), guardrails with posts
  on both shoulders, utility poles with sagging wires inside the loop, speed-limit and curve-warning signs
  (drawn procedurally in assets/truck). Render settings add GI, sampled lighting, DLSS quality and a light haze.
  "HQ photo" switches to path tracing (8 spp/frame, 90 frames) for a still in `exports/<time>_hq/photo.png`
  and returns to real-time; `--verify` takes one at frame 420.
- Chase camera: with "Follow vehicle" on, the view keeps its offset in the vehicle frame (it turns with the
  truck) and always looks at the truck, so a chosen angle stays clear of the cargo around the loop; the earlier
  translation-only follow let the camera drift behind/ahead of the truck in the corners, where the cargo or the
  cab blocked the view. Presets Front (default, high front quarter over the road), Rear, Side, Aerial, Depth
  (D435i), Cab; an Alt-orbit is adopted as the new offset on mouse release; a position inside the truck falls
  back to the default. The per-frame camera update writes the camera transform directly (no undo commands).
- Verified 2026-10-18 (`--verify`): delete protection, Brake/Go, follow camera, measured surface animating,
  IMU-driven motion along the loop, wheel roll axis, judgement levels equal to the CSV, sandbox release.
  Re-run after the realism pass (same day, exports/20261018_2152*): all checks pass again - camera offset
  drift 0.09 m over 45 frames, surface motion 0.385 m while advancing 6.5 m at 6.2 m/s, wheel roll about
  the axle only, 240 x 320 depth, cargo slides 2.6 m after strap release on the 12 % grade; real-time
  ~40 FPS in the GUI with the full forest, HQ path-traced still written to exports/20261018_215240_hq.
- Shimmer fix (2026-10-18, evening): share mode had disabled DLSS (NGX) and fallen back to Kit's TAA, copied
  from NVIDIA's headless webrtc app. Measured with scratch harnesses on the realistic scene: TAA rendered a
  frame in 61 ms vs 32 ms with DLSS quality, with twice the static frame-to-frame noise, and toggling the
  individual effects (sampled lighting, GI, AO, dithered glass, rough reflections) changed the grain by
  < 5 %, so the effects stay on. Share mode now keeps DLSS: 47 FPS streamed instead of 30. The WebRTC
  encoder has no bitrate setting in this Kit build (NvStreamServer adapts it); the signalling server can
  be checked with a curl WebSocket handshake on port 49100 (expects 101 Switching Protocols).
- Controls (2026-10-18): panel title reads HYUNDAI XCIENT; a `Turn left` button (+25 deg/s) joins Go / Brake /
  Turn right / Straight so remote viewers can steer both ways without the slider. The Korean operator guide
  for teammates is `CONTROLS_KO.md`.
- Recording (2026-10-19): nothing is saved at start-up. `Start recording` (green, under Play/Reset) writes
  `recordings/<time>/twin_log.csv` (Excel, UTF-8 BOM) and `twin_log.jsonl` at 10 Hz - replay/vehicle state
  (mode, run, replay time, km/h, lap position, lane, heading, slope, bank, cargo mass, straps) followed by
  every reading the panel shows (cells, belts, IMUs, COG, judgement levels); `Stop recording` (red) closes
  the files and writes `summary.json`. Closing the app stops a running recording cleanly. Verified with
  `--verify` (recorder off at start, 10 rows in 1 s, summary written).
- Team download (2026-10-19, share mode): `scripts/recordings_web.py` mounts `/recordings/` on Kit's HTTP server
  (port 8211, no new port): a Korean index of recordings with a zip per recording and per-file links; only
  YYYYMMDD_HHMMSS folders and the three recorder files are served, a folder still being recorded is listed but
  not downloadable (409). The panel shows the address after Stop recording. Tested with curl over Tailscale and
  LAN (index, zip, csv 200; traversal attempts 404).
- Cargo shift (2026-10-20, `scripts/cargo_shift.py`): the sensor Pi's lv_move (algorithm 2026-10-08) misses a
  shift that stops - its reference follows the measured COG - so the "no rear belt, cargo shift" run is level 0
  throughout although the COG settles 28-30 mm forward. New rule: reference = COG/weight of the first steady 0.5 s
  (IMU bias from the same window, so a slope is not acceleration); judgement only after 0.7 s below 0.06 G; offset
  12/25/50 mm = caution/warning/danger; >= 20 % weight loss or one plate end newly below -0.5 kg = danger (tip);
  latched until reset/seek/new run. All 14 recorded runs as expected (shift/tip/unbelted hard stop >= warning,
  drives/curves/vibration/COG-offset 0, collision caution at 21 mm). Replay keeps the Pi value as lv_move_pi; the
  overall level is recombined with the team rule and never drops below the recorded one. Verified in the twin:
  replay of 20261015_212927 -> level 2 (Pi 0); sandbox strap release on 12 deg -> level 3 (64 mm rear).
- Remote control (2026-10-20, `scripts/twin_api.py`, share mode): /twin/ guide, /twin/state, /twin/commands,
  POST /twin/command (20 actions mirroring the panel; sliders clamped to their ranges), /twin/screenshot (JPEG),
  /twin/mcp. Actions are queued and run by the main loop. `scripts/isaac_twin_mcp.py` is a stdlib-only stdio MCP
  server for teammates' Claude Code / Desktop (twin_state, twin_commands, twin_command, twin_screenshot,
  list_recordings, download_recording); tested end to end over JSON-RPC.
- Display Pi (2026-10-20, `scripts/display_link.py`): "Send to display" streams a saved recording to the display
  kiosk (display/twin in heayoooon/2026ESWContest_mobility_JeokjaeJeokso dev; TCP 5010, frames as in its README)
  at the recorded pace: sensors/judgement from the recording, camera bodies from the recorded run's twin.tws at the
  same replay time; header rewritten (new session, run isaac_<name>). Default host 100.90.251.42 (env
  TWIN_DISPLAY). Tested against a local receiver that frames and decodes like the kiosk (all frames decoded,
  JPEG valid, move level and Korean reason present); not yet on the real display (it was offline).
- Fixes found by inspecting the running twin (2026-10-20, via /twin/screenshot and /twin/state):
  * D435i view / simulated depth: the lens sat inside the camera housing with the mast top in its field of view,
    which blacked out the lower half of the Depth view and of depth_m.npy. Housing and mast moved behind the lens.
  * Road slope / bank turned the whole world about the origin; ~100 m from it a 12 deg change moved the deck
    ~20 m in one frame and threw the strapped cargo off (cells 0, belts 7500 N), also on the way back to 0 deg.
    `retilt()` now turns the world about the deck and carries the cargo along; strapped cargo stays on 12 deg
    (belts 16 N, plate load cos 12 deg), with friction 0.15 it slides off when released (shift -> danger).
  * Cargo-shift reference: learned only while the horizontal acceleration is near the bias (a steady but
    non-zero acceleration from rest had become the "loaded position"); `rebias()` re-learns the IMU bias after a
    slope / bank change (the twin knows the slope; the real Pi assumes level ground), ignoring the change
    transient (the IMU spikes to -25 G in the tilt frame). 14 recorded runs unchanged.
  * --verify adds a sandbox drive-off / cruise / 12 deg strapped / release sequence (SANDBOX_SLOPE_CHECK).
- 2026-10-22 (user report + QA):
  * Display link streams until stopped: "Send to display" = the live twin at 10 Hz (replay or sandbox; strap
    preload / stiffness / slope / mass changes reach the display at once - verified on the real display: preload
    8 / 40 / 75 N -> belts 19 / 48 / 83 N, levels OK / caution / danger), reconnecting if the kiosk restarts; a
    recording sent from the download page or the API loops until stop_display. Sandbox frames move the cargo cells
    of the run's first camera frame by the simulated displacement (cleared when the cargo leaves the plate) so the
    display's 3D boxes follow the cargo (display_link.move_cargo, unit-tested).
  * Truck sinking into the road: the whole vehicle (wheels included) pitched up to 6 deg (10 deg/G) about its
    ground point, putting the front or rear axle ~0.25-0.5 m into the road under hard braking / acceleration.
    Now 4 deg/G, max 3 deg, and Truck.suspend moves every wheel and axle along the body's vertical so the tyres
    stay on the road (checked in a low road-side render at 3 deg). Relative heading limited to 30 deg and the lane
    offset to the asphalt minus 1.3 m so the 10 m truck no longer swings over the rising verge.
  * Sandbox load: judge() applies the team rule (measured weight minus 2T/g per belt) but the simulated cells
    carried only the cargo weight, so the load came out 4T/g too low (7.7 kg at 16 N belts, 0 kg from ~35 N).
    The cells now carry the belt push 4T/g (two belts, both legs) through the cargo like the real cart, so the
    judged load stays at the cargo mass for any preload (verified 8 / 40 / 75 N preload -> 14.26 kg; cells
    22.1 / 33.9 / 48.2 kg). COG unchanged (the push goes through the cargo footprint).
- Team judgement (2026-10-22): every judgement in the twin now comes from the team's sensor-Pi code.
  `scripts/sync_team_analysis.py` copies pi/analysis/*.py and pi/acquisition/{config,frame}.py + env.sh from
  heayoooon/2026ESWContest_mobility_JeokjaeJeokso (default branch dev; --branch / --repo for others) into team/
  with TEAM_VERSION.json; `scripts/team_judge.py` builds Frame / ImuReading from the twin's channels and runs
  analysis.analyzer.Analyzer with the Pi's env.sh, returning the sensor_feed.py columns. Replay re-judges every
  recording from its raw cells / belts / IMU at 10 Hz (the recording-time judgement stays in
  recorded_judgement); the sandbox feeds its simulated channels every 0.1 s. The panel shows the algorithm
  (e.g. "team dev@d4fe723"); the twin's own judge() and cargo_shift.py remain only as fallback / comparison
  (lv_move_twin). Re-judging the 14 runs with dev: load_kg identical to the Pi's, levels follow dev's newer
  thresholds (e.g. COG 15/8/3, COG_RESID 7: "no rear belt, cargo shift" -> cargo shift caution, was 0).
- Sandbox made to look like the real sensors to the team analyzer (each found by driving the sandbox and reading
  recordings): belt push 2T per belt on its own cell row (team model BELT_N_Y=0 / BELT_S_Y=BED_W); teleport spikes
  of the simulated IMU (up to 1e4 G at reset / slope change) replaced by the last plausible sample - one had left
  the analyzer's static COG at -3.2 km; IMU channels averaged over the 0.1 s frame and the cells' load transfer
  lagged 0.2 s (COG_LAG); load transfer h x a with the real cargo's h = 190 mm; the truck waits 1.5 s after a
  reset so the cargo settles; the commanded turn rate ramps over 0.6 s and fades near the heading / road-edge
  limits (a step yawed the deck and slid the strapped cargo 20-35 mm, which the analyzer rightly flagged).
  Probe: 46 s strapped (drive-off, 3 left/right turns, braking) -> all levels OK, cargo offset <= 13 mm;
  strap release with friction 0.15 in a turn -> cargo shift caution -> danger in 0.3 s, COG danger.
- COG X was mirrored: the marker and the sandbox cells put +COG X on the vehicle's left, while the D435i surface,
  the display convention (plate X = camera x) and the real cart (+ay in left turns loads +COG X, +110 mm/G) have
  it on the right. Marker, sandbox cells and display offset now use +X = right.
- Lashing belts as ropes (2026-10-22, `scripts/belt_rope.py`): the belts were three rigid boxes per belt (an
  inverted-U frame that only changed height) and, in the sandbox, an invisible 3D spring that pulled the cargo
  centre to its seat in every direction, with damping on the world velocity (a constant backwards drag while
  driving). Now each belt is webbing anchored beside the plate (tension-cell side and opposite edge) and pulled
  over the cargo: its path is the over-the-top arc of the convex hull of the anchors and the cargo's cross-section
  in the belt plane, its tension preload + stiffness x stretch + damping on the length rate, never negative
  (slack belts carry nothing, belts only pull), applied at the points where it leaves the cargo along the free
  legs (force + torque). Front c1 and rear c2 are computed separately and press their own cell rows. Belts are
  tightened while the cargo settles after a reset; the tension may change by at most 3 kN/s and is capped at
  400 N (a belt catching a bottom corner had thrown the cargo 5 m). The drawn webbing is re-draped every frame on
  the visible scanned cargo (sandbox) or on the D435i surface (replay). Strap stiffness default 1500 N/m
  (100-5000). Probe: at rest 8 / 8 N; turns 9-14 N; preload 40 -> 40 N with the load still 14.26 kg; friction
  0.15 on 12 deg -> the cargo slides out from under the front belt (c1 -> 0 N), then off the plate.
- Truck bed colliders (floor, side gates, tailgate, cab guard; invisible) so cargo that leaves the plate lands on
  the bed and stops at the tailgate instead of falling through the truck.
- Mouse moves of the cargo (2026-10-24): in the sandbox, moving the cargo with the viewport Move / Rotate tool -
  usually the scanned boxes, a child of the PhysX body - is applied to the PhysX body on mouse release
  (user_moved_cargo / teleport_cargo), so cells, belts and the team judgement follow. PhysX does not write the
  body pose back to USD here (USD stays at the creation pose), so the true body transform is taken from PhysX.
  API action move_cargo [right_mm, rear_mm] does the same for Claude. Belt anchors lowered to z .20 (below the
  plate top): with them at the plate top a cargo pushed 4 cm sideways put its bottom corner past an anchor and
  the belt dropped to 0 N; now tension rises with the push (4 cm 12.8 N, 7 cm 45.9 N, then belt caution).
- Sandbox IMU like the real MPU-6050 (2026-10-24): Isaac's IMU sensor on the kinematically moved truck missed
  the centripetal acceleration (turns read ~5 % of v x yaw rate), shook 70x more than the real sensor when parked
  and had no road vibration. The specific force is now computed from the truck's motion (forward acceleration,
  v x yaw rate, gravity through slope / bank / body attitude), gyro from heading and body roll / pitch rates, plus
  the real sensor's mounting bias (ax -0.006, ay +0.022 G), 0.1 s-frame noise when parked and road vibration when
  driving (ax / ay / az 0.035 / 0.015 / 0.022 G, gz 3.7 deg/s, AR(1) tau 0.08 s), from the sensor Pi recordings;
  roll / pitch with the Pi's complementary filter. Load cells keep the bias- and noise-free force. Measured:
  parked ax -0.0058 +- 0.0004, ay +0.0219 +- 0.0004 (real -0.007..-0.009 / +0.023..+0.025, sd 0.0003-0.0007);
  cruise sd ax 0.029, ay 0.011, gz 4.3 (real 0.031-0.046, 0.013, 3.7); turns ay 0.115 G (was 0.004); roll
  12.7 deg at 0.27 G (real 14.4 deg at 0.40 G); team analyzer h learnt 235 / 208 mm (was ~100 lateral), no false
  alarms; --verify passes.
- "Body must be non-kinematic!" PhysX errors (since 2026-10-20): retilt / teleport / reset set velocities, and the
  belt ropes applied forces, on the cargo while it was kinematic (replay, or the frame of a mode switch). All of
  them now check cargo_dynamic() first; --verify logs 0 [Error] lines.
- Lashed cargo held by static friction (2026-10-24, user report "the cargo moves although the belts are tight"):
  * secure_cargo: while demand m g |f_plane| < capacity mu (m g f_n + 2 T_front + 2 T_rear) the cargo is a kinematic
    body fixed to the plate (re-placing a dynamic body every frame left it 8 mm high on 12 deg and stretched the
    belts to 55 N); beyond it the cargo turns dynamic with the deck velocity, slides, and is held again where it
    rests. CSV: cargo_secured, friction, grip_demand_N, grip_capacity_N.
  * Vehicle orientation is now road tilt x heading x body attitude (vehicle_rotation). The Euler form put the heading
    after the tilt, so on a slope the truck was not parallel to the road once it turned (6.7 deg off at 6.7 deg
    slope, heading 60) and a held cargo read as sliding ~10 cm in curves.
  * Cargo position (cells, cargo_offset_mm) is taken in the real plate frame; a free cargo is compared with the deck
    at the same time step (it read v x dt = 52 mm ahead at 36 km/h).
  * The mouse-move detector forgets the cargo's USD pose on reset / mode switch (replay rewrites it every frame,
    which read as a mouse move and teleported the cargo when coming back to the sandbox).
  Result: friction 0.6, belts 24 N, 36 km/h, slope 0 -> 6.7 deg, curves, braking: cargo offset 0.0 mm, team cargo
  shift 0, h learnt 188 / 183 mm; friction 0.15 on 12 deg slides (front belt slack, cargo-shift danger); --verify
  passes with 0 PhysX errors.
- Mouse moves in measured replay (2026-10-24, user report "moving the cargo does not change the COG"): replay plays
  the recorded cells, so the move only moved the picture. replay_cargo_moved now switches to the physics sandbox on
  mouse release and puts the cargo at the same place on the plate (scanned parts or the body itself; the API
  move_cargo does the same in replay). A held cargo moved in the sandbox is held again at once at its new place:
  let go as a dynamic body for one frame it lost the deck speed and slid 17 mm rearward on a driving truck.
  Checked: +19 mm right -> COG x +19, +40 mm rear -> COG y +40, -31 / +31 mm body drags in replay and at 20 km/h,
  cargo_offset_mm unchanged afterwards (the ~24 mm COG y change when driving off is load transfer).
- Cargo moved while the sandbox is paused (user report: "the COG does not change in physics mode either"): the
  move was detected (CARGO_MOVED) but the team analyzer, which supplies cog_x / cog_y, only judged when the sim
  clock ran, so a paused twin kept the last judged COG; the moves showed up only after Play. The team clock now runs
  on while paused (the Pi keeps judging a parked truck) and PhysX steps for 1 s after a move while paused. Checked
  paused: +19 mm -> COG x 144, body drag +25 mm -> 169, paused mid-drive +19 mm -> 188; 0 PhysX errors.
- Belts measured one by one (2026-11-01, user request "the two tensions are measured at once"): the sandbox had one preload
  slider and one hook box for both belts, so c1 and c2 always read the same. Now each belt has its own preload
  (`pretension_c1` / `pretension_c2`) and hook (`straps_c1` / `straps_c2`); the old `pretension` / `straps` controls set
  both (the API, Release cargo and --verify keep working). Each tension cell reads its own belt (0 N when that belt is
  unhooked) with its own HX711 noise (sigma 0.01 N, the real cells at rest); static friction, the drawn webbing and the
  recorder follow each belt (CSV: belt_c1_connected, belt_c2_connected, preload_c1_N, preload_c2_N; straps_connected =
  both hooked).
- Korean panel (2026-11-01, user request): Kit's glyph manager takes extra glyphs from `/app/font/customFontPath` +
  `/app/font/customRegionFiles` (undocumented, found in omni.ui.glyphmanager). isaac_twin_light.kit (the site-packages copy is
  the one used, both copies updated) loads Malgun Gothic for `assets/fonts/korean_glyphs.txt` (2350 KS X 1001 syllables +
  jamo + every Hangul in scripts/, team/ and live_data.json; the file must be 64 characters per line like Kit's
  japanese.txt - one long line rendered only one glyph). Panel buttons, sliders, sections, readouts, run titles and the
  team's Korean warning text are Korean; API keys / names, recordings, log and API messages stay English (status_ko()
  translates the status line for the panel). New Hangul in panel text must be added to the glyph file.
- Render quality presets (2026-11-01, user report "the truck flickers"): the viewport fills its window (~1060 x 814) and
  DLSS quality rendered ~0.67 of that and upscaled, so the truck's thin parts shimmered. Panel 시점 > 화질 / API
  render_quality: fast (old: DLSS quality, 1 light sample), balanced (default: DLAA = /rtx/post/aa/op 4, 2 samples),
  high (DLAA + viewport resolution_scale 1.5). Measured on a still truck: frame-to-frame change 2.07 / 1.66 / 1.24,
  sharpness (Laplacian variance) 503 / 820 / 1013, 46 / 33 / 21.5 FPS. 4 light samples changed nothing. Env
  TWIN_QUALITY picks the start-up preset. The viewport `resolution` setting has no effect while fill_frame is on.
- Grass everywhere + sunny weather (2026-11-01, user request "bare patches beside the road, make it sunny"): the
  terrain beyond ROAD_EDGE + 8 m used the beige forest-floor material, so gaps between trees looked like dirt. It is
  now a second Grass_Countryside subset (GrassFloorPBR, tint .58/.70/.46, darker than the verge). Sun 4300 warm white,
  dome 1050 slightly blue, fog lighter and farther (colour .80/.87/.96, start 250 m, end 1500 m, density .03). The sky
  texture is now the clear noon HDRI `assets/nvidia/sky_clear/noon_grass_4k.hdr` (NVIDIA content server
  Assets/Skies/Clear, 25.3 MB, downloaded with the user's OK; fetch_nvidia_assets.py gets it too); the partly-cloudy
  one is the fallback when that file is missing. The 1500 leaf-litter mats (oakfall/maplefall, 1.5-3 m, up to 20 m
  from the road) are no longer placed: they showed as brown bare patches along the guardrail.
  The real cause of the brown triangles beside the road was the gravel shoulder ribbon showing where the coarse
  terrain dips into the ditch; the shoulders now use the verge grass material.
- Corner logos (2026-11-01, user request): contest logo top-left (assets/branding/contest_logo_white.svg = the
  user's contest_logo.svg with white text) and Hyundai logo
  (assets/branding/hyundai_logo.png) top-right of the viewport, small (30 / 34 px high) with no backing (the user's
  files are transparent), just below
  the viewport toolbar. They are an omni.ui overlay frame (`LOGOS`, `add_logos`), so the window and the share stream
  show them but /twin/screenshot and exports (render captures) do not.
- Replay cargo less warped (2026-11-01, user request "fix the cargo distortion a little"): measured_surface.DepthSurface
  now uses median 5 (was 3), drops flying pixels (3x3 depth span > 25 mm), fills holes from the nearest pixel above
  the deck (not a plate pixel) and blends consecutive frames (50 %, only where they differ < 20 mm; a seek starts
  afresh). Slanted 'sheet' faces between the cargo top and the plate, mean over the 14 cached runs: 256 -> 102 cm2;
  frame-to-frame depth change 1.62 -> 1.33 mm. The shape still comes from the recorded D435i frames.
- Live link (2026-11-01, user request "follow the bed in real time like the display Pi"): `scripts/live_link.py`
  listens on TCP 5010 for the sensor Pi's display frames (same format as twin.tws: header with the sensor row and
  camera calibration, depth row deltas, JPEG). Panel 실시간 받기 / API `live`: measured mode then draws the newest frame's
  surface (DepthSurface, re-calibrated when the Pi's camera / ref_h change), shows the Pi's sensor row and judges each
  new row with the team code on the Pi's own clock (t_s); the Pi's own levels are kept as recorded_judgement. The
  truck follows the driving sliders. The display Pi cannot forward frames, so the sensor Pi has to mirror them:
  live_twin.py --mirror <PC>:5010 with its own sender thread (sending to both through one ReliableSender would block
  the display for 3 s x 2 per frame whenever this PC is off). Tested end to end with a recorded run sent at 10 Hz
  (9.2 Hz received, team judgement and panel follow); the Pi reaches this PC's port 5010 over the tailnet in 10 ms.
  Sensor Pi patched with the user's OK (backups ~/scss_live/*.bak_mirror_20261101_045221) and restarted as
  `run_live.sh 100.90.251.42 100.109.1.106:5010`: display still 10 Hz, the twin receives 9.8-10 Hz with 0.01-0.05 s delay.
- Display-style cargo boxes (2026-11-01, user request "make the live cargo 3D, boxes like the display Pi are fine"):
  `scripts/cargo_boxes.py` runs the display Pi's own pure-numpy code (team display/twin/server cargo_model, stable_twin,
  stack_layers, surface_core; copied by `sync_team_analysis.py --part display` into team_display/) the way
  twin_view.StableBoxMesh + twin_server do (fixed reference from the first second, changes lasting > 0.5 s, boxes on the
  top planes, stacked layers, cargo floor) and draws up to 12 USD cubes / cylinders on the rig. Panel toggle
  `cargo_boxes` (default on) swaps them with the raw depth surface, in replay and live. The lowest boxes stand on the
  deck (the display's fixed .365 m foundation is its own camera reference). 13-24 ms per new frame (10 Hz); the
  twin stays at ~30 FPS. A higher-resolution depth stream was not pursued: the Pi already decimates 848x480 to 424x240
  at ~26-33 ms per frame and the display Pi gets the same frames.
  Review fixes (PR #34): StackLayers gets the frame as received (as twin_server does) and one StackLayers lives for
  good (its own reset keeps the cargo floor and the last layers); a stale live link (> 2 s) feeds no sensor row and
  clears the cargo instead of judging / recording the last reading; switching live on clears the replay cargo; live
  off restarts the recording at 0 s; recorder rows say mode 'live'. cargo_boxes falls back to ../display/twin/server
  inside the team repo when team_display/ was not synced.

## Realistic update - 2026-10-05

- NVIDIA Omniverse assets were collected once into `assets/nvidia/` by `scripts/fetch_nvidia_assets.py`
  (~500 MB): five tree species, grass, leaf litter, rocks, the `male_adult_construction_03` character,
  Base MDL road/ground materials and the `kloofendal_48d_partly_cloudy_4k` HDRI sky. When that folder
  is missing, `forest_scene.py` falls back to the original procedural primitives.
- Terrain: asphalt road mesh, gravel shoulders, a ditch and rolling verge/forest floor with world-space UVs.
  Invisible box colliders keep the earlier contact behaviour.
- Trees are instanceable references (their internal branch instancers lose leaves inside another
  PointInstancer). Barberry is excluded because it turns RTX frames black. Densities in `forest_scene.py`
  (`TREE_ROWS`, `COVER_NEAR`, ...) are tuned for about 55 FPS on the RTX 3080.
- Operator: skinned character dressed as in the reference video (black top, grey trousers, black cap).
  The skeleton is posed every frame: leg gait from the cart speed, two-bone IK keeping both hands on the grip,
  curled fingers. The NVIDIA walk clips use a different skeleton, so they are not used.
- First launch compiles the vegetation shaders; the trees can look grey for a short while.
- Viewport performance: the follow camera now carries the user's own view with the cart instead of
  re-setting the view every frame (that fought Alt-orbit). While a mouse button is held the camera is
  left to the user; the accumulated cart motion is caught up after release. Presets/zoom snap once.
  Video/depth panels decode forward and upload numpy arrays; physics properties are written only on change.
  The panel shows the viewport FPS; `--verify` also checks that a user view keeps its offset while following.
- Backup of the previous scripts: `backup_20261005_before_realism/`.

## Remote sharing - 2026-10-10

`Start-ForestTwin-Share.bat` runs the twin with Isaac Sim's WebRTC livestream
(`omni.services.streamclient.webrtc`). Only the Isaac Sim window is streamed; the viewer can control it.
Ports: 8211 (web page) and 49100 (signalling). The start-up log prints `SHARE_URL` lines, e.g.
`http://100.109.1.106:8211/streaming/webrtc-demo/?server=100.109.1.106` for Tailscale members.
Limits: one remote viewer at a time; DLSS is replaced by TAA while streaming. The Isaac Sim UI includes
the Script Editor and file dialogs, so a connected viewer can reach files on this PC - share only with
trusted people. The firewall rule for this python.exe allows Private and Public networks, so the stream
is also reachable from the local Ethernet network.

Isaac Sim 4.2's stream only advertises the LAN address (192.168.0.17) and 127.0.0.1 as WebRTC candidates,
not the Tailscale address (there is no publicEndpointAddress setting in this version). This PC therefore
advertises `192.168.0.17/32` as a Tailscale subnet route (`tailscale set --advertise-routes=192.168.0.17/32`,
approved by the tailnet admin alaskaa-yjh). Viewers need "Use Tailscale subnets" on. If the LAN address
changes, update the route.

All scene prims are marked `no_delete` at start-up: a remote viewer deleted `/World/Cart/Deck`, which
invalidated PhysX and ended the app. Unexpected errors now reset the simulation instead of closing the app.

## Imported evidence

- 35 sensor CSV recordings / 5,277 samples, including all nine 2026-10-04 runs.
- Camera CSV summaries matched to sensor timestamps within 250 ms.
- Seven external videos supplied on 2026-10-05. Select and compare them in the video panel.
- Original cargo photogrammetry USD and texture are retained.
- Pi acquisition configuration confirms NW-NE support spacing 250 mm and NW-SW 392 mm.
- All raw CSV values are retained. Missing values stay unavailable; overloads are not clipped.
- No new Oct 4 RGB/depth image recording was found on the Pi repository. The existing Oct 3 bags/surfaces remain in the older project.

## Controls

- **Measured replay**: choose any recording, play/pause, seek, adjust playback rate and loop.
- **Physics sandbox**: PhysX gravity, road/deck contact, adjustable friction and dynamic cargo.
- **Road / movement**: slope, bank, speed, acceleration, turn rate, lane position and pull/push pose.
- **Cargo / physics**: cargo/cart mass, cargo offsets, friction, strap preload/stiffness, release/reset, scan/proxy visibility.
- **View**: orbit, side, aerial, simulated depth-camera view, follow-camera switch and distance.
- **Reference video / depth**: seven original videos, manual time offset, simulated depth image.
- **Export**: current USD stage, settings and sensor JSON, CSV snapshot, depth array in metres, viewport PNG.

Exports are in `exports/YYYYMMDD_HHMMSS/`. USD exports reference local textures; keep the asset directory with the project.

## Sensor interpretation

The scene contains four compression cells rated 5 kg each, two tension cells rated 10 kgf (98.0665 N) each, one Isaac IMU, and one simulated RGB/depth camera. The physical support layout uses the measured spacing.

Replay values come from CSV, including c1 and c2 when present. CSV rows without c2 display N/A. Real load-cell COG is the centre of pressure under the current loading, not an independently measured 3-D mass centre.

In sandbox mode, IMU and depth are generated by Isaac Sim. The four load reactions use a quasi-static support-distribution approximation and strap readings use an elastic-force approximation. They are labelled estimated in the UI. They are not individual calibrated force-transducer simulations. The photogrammetry cargo uses one aggregate box collision proxy, so it does not simulate independent boxes colliding with each other.

## Reconstruction limits

- The road/trees are a procedural scene based on the supplied wooded-road references, not a survey of that exact location.
- The operator is an articulated clothed avatar with the video's black top, grey trousers and cap, hand-to-handle placement, and procedural gait. Exact human shape and motion capture have not been recovered.
- External video / sensor run pairing and clock offset are not verified. Videos remain independently selectable; no filename-based pairing is claimed.
- Cart world trajectory and walking speed are user-controlled approximations. The CSV does not supply an observed global trajectory.
- Cart outer dimensions, mass, cargo collider, friction and strap properties need physical calibration before using this model for load safety or quantitative prediction.
- Changing slope in replay is a visual experiment; historical sensor values remain the measurements recorded on the original run.

## Implementation

- `scripts/prepare_forest_data.py`: reads raw channels and generates `forest_data.json`.
- `scripts/inspect_videos.py`: creates video contact sheets and metadata.
- `scripts/forest_scene.py`: USD forest, road, cart hardware and articulated operator.
- `scripts/run_forest_twin.py`: simulation, controls, replay, video, depth and export.

## Validation

Run `scripts/run_forest_twin.py --verify` to exercise measured replay, a 12 degree slope, dynamic cargo and strap release. It writes `verification.json` and a viewport export, then returns to replay and keeps the GUI open. A running process alone is not evidence of a correct scene; inspect the viewport and exported image as well.

Verified locally on 2026-10-05: compilation, 35-run/7-video import counts, timestamp ordering, exact preservation of sample channels, unavailable historical c2, 320x240 depth output, finite cargo positions, and movement after strap release. Native UI tests covered recording selection, replay-to-physics-to-replay switching, slope adjustment and aerial/orbit camera switching. The final run produced no stderr errors during these checks. Rendering was inspected in the actual Isaac window; the textured scan, avatar, forest and telemetry panel were visible.

Mode transitions and exports are queued between frames. Calling World.reset inside an omni.ui callback caused renderer re-entry and a native crash during development; the queued implementation fixes that tested path. Kinematic reset avoids writing invalid rigid-body velocities.

API references: [Isaac 4.2 sensors](https://docs.isaacsim.omniverse.nvidia.com/4.2.0/py/source/extensions/omni.isaac.sensor/docs/index.html), [Omniverse image provider](https://docs.omniverse.nvidia.com/kit/docs/omni.ui/latest/omni.ui/omni.ui.ByteImageProvider.html).
