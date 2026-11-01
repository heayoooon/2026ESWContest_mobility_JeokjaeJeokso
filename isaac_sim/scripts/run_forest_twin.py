"""Interactive Isaac Sim 4.2 digital twin of the JeokjaeJeokso cargo rig.

A Hyundai Porter-class truck (or, with --vehicle cart, the original hand cart) carries the instrumented
plate. Measured replay plays the 2026-10-14/15 sensor-Pi recordings (live_data.json): load cells, belts,
IMU, the team's 4-level judgement, and the D435i depth/colour frames as an animated cargo surface; the
vehicle motion follows the recorded IMU. Physics sandbox drives a PhysX cargo body with sliders.
"""
import argparse
import bisect
import csv
import json
import math
import os
import sys
import time
import traceback
import collections
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
parser = argparse.ArgumentParser()
parser.add_argument('--verify', action='store_true')
parser.add_argument('--share', action='store_true',
                    help='stream this Isaac Sim window to a browser (WebRTC, one remote viewer with control)')
parser.add_argument('--vehicle', choices=('truck', 'cart'), default='truck')
parser.add_argument('--stream-res', default=os.environ.get('TWIN_STREAM_RES', '1920x1080'),
                    help='share mode: window and renderer size WxH = the streamed picture (e.g. 2560x1440)')
args, kit_args = parser.parse_known_args()
sys.argv = [sys.argv[0]] + kit_args
os.environ['OMNI_KIT_ACCEPT_EULA'] = 'YES'
from isaacsim import SimulationApp

STREAM_PORT = 8211  # HTTP port of the Kit services server (set in isaac_twin_light.kit)
kit = Path(sys.executable).parent / 'Lib/site-packages/isaacsim/apps/isaac_twin_light.kit'
config = {'headless': False, 'renderer': 'RayTracedLighting', 'width': 1440,
          'height': 900, 'window_width': 1760, 'window_height': 1080, 'anti_aliasing': 3}
if args.share:
    # Livestream encodes the app window with NVENC. DLSS stays on: Kit's TAA (the earlier choice,
    # copied from NVIDIA's headless webrtc app) left visible shimmer on foliage and thin edges and
    # rendered the realistic scene about twice as slowly (61 ms vs 32 ms per frame).
    # SimulationApp 4.2 forwards unknown command-line arguments to Kit.
    # While a viewer is connected the window is held at the renderer resolution (/app/renderer/resolution, the
    # 'width'/'height' here) and that is what gets encoded: at the old 1280x720 the 1080p stream was an upscale of a
    # ~840x630 viewport. Window and renderer are both the stream size (--stream-res); /twin/view asks for the same.
    sw, sh = (int(v) for v in args.stream_res.lower().split('x'))
    config.update(window_width=sw, window_height=sh, width=sw, height=sh)
    sys.argv += ['--/app/window/drawMouse=true',
                 '--enable', 'omni.services.streamclient.webrtc']
app = SimulationApp(config, experience=str(kit))

import cv2
import carb
import carb.input
import carb.settings
import omni.appwindow
import omni.ui as ui
import omni.usd
from omni.isaac.core import World
from omni.isaac.core.objects import DynamicCuboid
from omni.isaac.core.materials import PhysicsMaterial
from omni.isaac.core.utils.rotations import euler_angles_to_quat, quat_to_euler_angles
from omni.isaac.core.utils.viewports import set_camera_view
from omni.isaac.core.prims.xform_prim_view import XFormPrimView
from omni.isaac.sensor import Camera, IMUSensor
from omni.kit.viewport.utility import get_active_viewport, capture_viewport_to_file
from omni.kit.viewport.utility.camera_state import ViewportCameraState
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade
from forest_scene import ForestScene, add_scan, bind, material, pose, shape
from measured_surface import DECK_TOP, DepthSurface, jpeg_bytes, load_cache
from live_link import LiveReceiver
from measured_surface import GRID_Y_TO_RIG_X, PLATE_ABOVE_FLOOR, vertex_colour_material
from cargo_boxes import BoxDrawer, DisplayBoxes, frame_from_parts, team_display_version
import road_path
from cargo_shift import CargoShift
from display_link import DisplaySender
import twin_api
import belt_rope
import truck as T

G = 9.80665
RATED_KG = 15.
BED_MM = (250., 392.)
LEVEL_NAMES = ('OK', 'CAUTION', 'WARNING', 'DANGER')
LEVEL_UI = (0xff60d060, 0xff30d0ff, 0xff2080ff, 0xff3030ff, 0xff909090)   # omni.ui 0xAABBGGRR; [4] = n/a
LEVEL_RGB = ((.08, .8, .25), (1., .8, .1), (1., .4, .05), (1., .06, .04))
TENSION_MAX_N = 300.   # belt tension upper limit (N) in the twin; team_judge.TWIN_ENV sets the same for the team code
DISCRETE = {'risk', 'lv_load', 'lv_cog', 'lv_move', 'lv_c1', 'lv_c2', 'rated_kg', 'tension_max_N'}
TRUCK = args.vehicle == 'truck'


def rotation(roll=0, pitch=0, yaw=0):
    return euler_angles_to_quat(np.radians([roll,pitch,yaw]))


def kinematic_pose(obj, position, orientation):
    obj.prim.GetAttribute('xformOp:translate').Set(Gf.Vec3d(*map(float,position)))
    obj.prim.GetAttribute('xformOp:orient').Set(Gf.Quatd(float(orientation[0]),Gf.Vec3d(*map(float,orientation[1:]))))


class SwitchableCuboid(DynamicCuboid):
    def post_reset(self):
        if UsdPhysics.RigidBodyAPI(self.prim).GetKinematicEnabledAttr().Get():
            XFormPrimView.post_reset(self._rigid_prim_view)
        else:
            super().post_reset()


def stream_urls():
    """Browser addresses for the WebRTC stream, Tailscale (100.64.0.0/10) first."""
    import ipaddress
    import socket
    ips = {info[4][0] for info in socket.getaddrinfo(socket.gethostname(), None, socket.AF_INET)}
    tailnet = ipaddress.ip_network('100.64.0.0/10')
    ordered = sorted(ips, key=lambda ip: (ipaddress.ip_address(ip) not in tailnet, ip))
    return [f'http://{ip}:{STREAM_PORT}/twin/view?server={ip}' for ip in ordered      # high-quality viewer page
            if not ip.startswith('127.')]


def interpolate(case, t):
    frames=case['frames']; times=case['_times']
    index=max(0,min(bisect.bisect_right(times,t)-1,len(frames)-1))
    a=frames[index]; b=frames[min(index+1,len(frames)-1)]
    u=np.clip((t-a['t'])/max(b['t']-a['t'],1e-6),0,1)
    out={}
    for k,v in a.items():
        w=b.get(k)
        if k not in DISCRETE and isinstance(v,(int,float)) and isinstance(w,(int,float)):
            out[k]=v+(w-v)*u
        else:
            out[k]=v
    out['_index']=index
    return out


RISK_WORDS = (('가감속으로 설명 안 되는 이동', 'shift not explained by accel.'), ('장력 보정 못 함', 'no belt correction'),
              ('값 없음', 'no value'), ('많이 쏠림', 'far off-centre'), ('쏠림', 'off-centre'), ('무게중심', 'COG'),
              ('화물 이동', 'cargo shift'), ('맡은 무게', 'share'), ('화물', 'cargo'), ('장력', 'belt'), ('적재', 'load'),
              ('과적', 'overload'), ('적정', 'rated'), ('상한 넘음', 'over limit'), ('과조임', 'over-tight'),
              ('하한', 'min'), ('상한', 'max'), ('부족', 'low'), ('풀림', 'slack'), ('앞쪽', 'front'), ('뒤쪽', 'rear'),
              ('왼쪽', 'left'), ('오른쪽', 'right'), ('끝', 'edge'), ('넘음', 'above'), ('밑', 'below'), ('확정', 'confirmed'),
              ('의심', 'suspected'), ('허용', 'allowed'), ('잔차', 'resid'), ('정상', 'OK'), ('주의', 'CAUTION'),
              ('경고', 'WARNING'), ('위험', 'DANGER'), ('초', 's'), ('의', 'of'), ('·', '/'))


def english_risk(text):
    """Keyword translation of the Pi's Korean risk text for the (Latin-only) panel font."""
    out = text or ''
    for ko, en in RISK_WORDS:
        out = out.replace(ko, en)
    return ' '.join(out.split())


KO_LEVELS = ('정상', '주의', '경고', '위험')


def level_of(value):
    try:
        return None if value is None else int(round(float(value)))
    except (TypeError, ValueError):
        return None


def level_colour(level):
    return LEVEL_UI[level] if level in (0, 1, 2, 3) else LEVEL_UI[4]


def level_name(level):
    return LEVEL_NAMES[level] if level in (0, 1, 2, 3) else 'n/a'


# Viewport render quality presets (see Twin.set_quality). aa/op: 3 = DLSS, 4 = DLAA (DLSS anti-aliasing, native res).
# Measured 2026-11-01 (still truck, RTX 3080, share mode): fast 46 FPS / flicker 2.07 / sharpness 503, balanced 33 FPS /
# 1.66 / 820, high 21.5 FPS / 1.24 / 1013. 4 light samples instead of 2 changed nothing.
QUALITY = {
    'fast': {'scale': 1.0, 'settings': {'/rtx/post/aa/op': 3, '/rtx/post/dlss/execMode': 2,
                                        '/rtx/directLighting/sampledLighting/samplesPerPixel': 1}},
    'balanced': {'scale': 1.0, 'settings': {'/rtx/post/aa/op': 4,
                                            '/rtx/directLighting/sampledLighting/samplesPerPixel': 2}},
    'high': {'scale': 1.5, 'settings': {'/rtx/post/aa/op': 4,
                                        '/rtx/directLighting/sampledLighting/samplesPerPixel': 2}},
}
DEFAULT_QUALITY = os.environ.get('TWIN_QUALITY', 'balanced')
# Small logos in the top corners of the viewport (2026-11-01, user request): contest logo left, Hyundai right.
LOGOS = ((ROOT / 'assets' / 'branding' / 'contest_logo_white.svg', 143, 30),   # text white (user request)
         (ROOT / 'assets' / 'branding' / 'hyundai_logo.png', 57, 34))


def level_ko(level):
    """Panel text (Korean). The API and recordings keep the English names."""
    return KO_LEVELS[level] if level in (0, 1, 2, 3) else '판정없음'


# Panel status lines: the messages stay English in the code / log / API, the panel shows them in Korean.
STATUS_KO = (('Live: listening on port ', '실시간: 센서 파이를 기다리는 중, 포트 '), ('Live: off', '실시간: 꺼짐'),
             ('Live: port ', '실시간: 포트 '), ('Render quality: fast', '화질: 빠름'), ('Render quality: balanced', '화질: 보통'),
             ('Render quality: high', '화질: 고화질'), ('render quality -> fast', '화질 -> 빠름'),
             ('render quality -> balanced', '화질 -> 보통'), ('render quality -> high', '화질 -> 고화질'),
             ('Remote control: ', '원격 조종: '), ('Recording saved: ', '기록 저장: '), ('Recording: ', '기록 중: '),
             ('Team download: ', '팀원 다운로드: '), ('HQ photo: ', '고화질 사진: '), ('Saved: ', '내보냄: '),
             ('Sharing (1 viewer): ', '공유 중 (1명 접속): '), ('Cargo moved with the mouse', '마우스로 화물을 옮김'),
             ('Display: idle', '디스플레이: 꺼짐'), ('Display: live', '디스플레이: 실시간 보내는 중'),
             ('Display: waiting for', '디스플레이: 연결 기다리는 중'), ('is the display app on?', '디스플레이 앱이 켜져 있나요?'),
             ('Display: not reading - is the sensor Pi streaming live? (stop_live.sh)',
              '디스플레이가 받지 않음 - 센서 파이가 실시간으로 보내는 중인가요? (stop_live.sh)'),
             ('Display: connection lost - reconnecting...', '디스플레이: 연결 끊김 - 다시 연결 중'),
             ('Display: stopped after', '디스플레이: 멈춤, 보낸 프레임'), ('Display: failed', '디스플레이: 실패'),
             ('frames sent (until Stop display)', '프레임 보냄 (멈출 때까지)'), ('Display: ', '디스플레이: '))


def status_ko(text):
    out = text or ''
    for en, ko in STATUS_KO:
        out = out.replace(en, ko)
    return out


class Twin:
    def __init__(self):
        self.data=json.loads((ROOT/'live_data.json').read_text(encoding='utf-8'))
        self.cases=self.data['cases']
        for case in self.cases:
            case['_times']=[f['t'] for f in case['frames']]
        # Judgement = the team's sensor-Pi code (team/, synced by sync_team_analysis.py): every recording is
        # re-judged from its raw cells / belts / IMU, the sandbox feeds its simulated channels at 10 Hz.
        self.team=self.team_live=None;self.team_t=-1.;self.team_out={};self.team_clock=0.
        try:
            import team_judge
            self.team=team_judge.TeamJudge();self.team_live=team_judge.TeamJudge()
            for case in self.cases:
                case['_team']=team_judge.judge_recording(case['frames'],self.team)
            print('TEAM_JUDGE',self.team.algo,flush=True)
        except Exception:
            traceback.print_exc()
            print('TEAM_JUDGE unavailable - using the twin rules (run scripts/sync_team_analysis.py)',flush=True)
        self.caches={}
        self.world=World(stage_units_in_meters=1.,physics_dt=1/60,rendering_dt=1/60)
        self.stage=self.world.stage
        self.scene=ForestScene(self.stage)
        if self.scene.realistic:
            settings=carb.settings.get_settings()
            for key,value in ForestScene.render_settings().items():
                settings.set(key,value)
        self.contact=PhysicsMaterial('/World/Materials/Contact',static_friction=.6,dynamic_friction=.48,restitution=.03)
        # Vehicle motion (2026-11-01): a free world pose (xy, yaw) on the paved test area round the loop, so a turn
        # happens the moment it is asked for. s / d / psi are the road coordinates of that pose (d + = inside the
        # loop, psi = heading relative to the travel direction dir: +1 counter-clockwise, -1 clockwise). Right-hand
        # traffic: outer right lane by default.
        self.road=getattr(self.scene,'road',None) or road_path.Road()
        self.lane_default=road_path.RIGHT_LANE
        self.s=self.road.start_s;self.d=self.lane_default;self.psi=0.
        self.dir=1;self.turn_target=None;self.gz_bias=None;self.edge_stop=False
        start,heading=self.road.frenet_to_xy(self.s,self.d)
        self.xy=np.array(start);self.yaw=math.degrees(heading)
        self.cart=self.world.scene.add(SwitchableCuboid('/World/Cart',name='instrumented_cart',
            position=np.array([start[0],start[1],0]),scale=np.ones(3)*.01,mass=7.,visible=False))
        # Root is a small invisible kinematic collider; the full-size plate deck underneath has collision.
        self.cart_prim=self.stage.GetPrimAtPath('/World/Cart')
        self.cart.set_visibility(True)
        UsdGeom.Imageable(self.cart_prim).MakeVisible()
        self.cart.set_local_scale(np.ones(3))
        UsdGeom.Cube(self.cart_prim).GetSizeAttr().Set(.001)
        UsdPhysics.RigidBodyAPI(self.cart_prim).CreateKinematicEnabledAttr(True)
        # Rig (instrumented plate) frame relative to the vehicle root.
        if TRUCK:
            # The plate is drawn RIG['scale'] times larger to suit the bed; sensor arithmetic stays in
            # the hand-cart's real geometry (rig_to_cart / cart_to_rig apply and remove the scale).
            self.rig_scale=float(T.RIG.get('scale',1.))
            self.truck=T.Truck(self.stage,'/World/Cart/Truck',self.scene.mats)
            rig_root='/World/Cart/Rig'
            rig=UsdGeom.Xform.Define(self.stage,rig_root).GetPrim()
            pose(rig,(T.RIG['x'],T.RIG['y'],T.RIG['z']),(0,0,T.RIG['yaw']),(self.rig_scale,)*3)
            self.scene.rig(rig_root)
            self.rig_t=np.array([T.RIG['x'],T.RIG['y'],T.RIG['z']]);self.rig_yaw=T.RIG['yaw']
            self.driver=self.scene.driver('/World/Cart/Driver',T.DRIVER)
            cam_pos=np.array(T.CAMERA['pos']);cam_rot=rotation(0,T.CAMERA['pitch_deg'],180)
        else:
            self.rig_scale=1.
            self.truck=None;self.driver=None
            rig_root='/World/Cart'
            self.scene.cart(rig_root)
            self.rig_t=np.zeros(3);self.rig_yaw=0.
            self.scene.operator()
            cam_pos=np.array([.14,0,1.04]);cam_rot=rotation(0,90,180)
        self.rig_root=rig_root
        a=math.radians(self.rig_yaw);self.rig_cs=(math.cos(a),math.sin(a))
        self.cart.apply_physics_material(self.contact)
        UsdShade.MaterialBindingAPI.SetMaterialBindingStrength(UsdShade.MaterialBindingAPI(self.cart_prim).GetDirectBindingRel(),UsdShade.Tokens.weakerThanDescendants)
        self.cargo=self.world.scene.add(SwitchableCuboid('/World/Cargo',name='cargo',position=np.array([start[0],start[1],1.5]),
            scale=np.array([.58,.38,.56])*self.rig_scale,mass=16.,color=np.array([.58,.43,.27])))
        self.cargo.apply_physics_material(self.contact)
        UsdShade.MaterialBindingAPI.SetMaterialBindingStrength(UsdShade.MaterialBindingAPI(self.cargo.prim).GetDirectBindingRel(),UsdShade.Tokens.weakerThanDescendants)
        self.cargo.prim.SetTypeName('Xform')
        self.cargo.prim.RemoveAPI(UsdPhysics.CollisionAPI)
        self.cargo.prim.RemoveAPI(UsdPhysics.MeshCollisionAPI)
        self.proxy=shape(self.stage,'/World/Cargo/Collider','box',(0,0,0),(1,1,1),self.scene.mats['box'],collision=True)
        UsdGeom.Imageable(self.proxy).MakeInvisible()
        # Child scan is compensated for the rigid cube's nonuniform scale.
        scan_parent=UsdGeom.Xform.Define(self.stage,'/World/Cargo/ScanRoot').GetPrim()
        pose(scan_parent,scale=(1/.58,1/.38,1/.56))
        self.scan=add_scan(self.stage,ROOT/'assets/cargo.usd','/World/Cargo/ScanRoot/Scan',.56)
        self.scan_points=self.collect_scan_points()
        self.cargo_parts={str(q.GetPath()):UsdGeom.Xformable(q).GetLocalTransformation()
                          for q in Usd.PrimRange(self.stage.GetPrimAtPath('/World/Cargo')) if q.IsA(UsdGeom.Xformable)
                          and str(q.GetPath())!='/World/Cargo'}
        self.cargo_seen=None;self.cargo_pending=None
        self.cargo_written=None;self.replay_move=None;self.queued_move=None   # mouse moves in replay
        self.pause_settle=0.                                                   # s of physics after a move while paused
        # Measured cargo surface (replay) lives on the rig so it moves with the vehicle.
        first=self.load_cache(self.cases[0])
        self.surface=DepthSurface(self.stage,rig_root+'/Measured',first['camera'],first['grid'],step=2)
        self.surface_index=-1
        # The same frames as closed boxes, made by the display Pi's own code (cargo_boxes.py, team_display/)
        self.box_model=DisplayBoxes()
        self.box_drawer=BoxDrawer(self.stage,rig_root+'/MeasuredBoxes',vertex_colour_material(self.stage),
                                  PLATE_ABOVE_FLOOR,DECK_TOP,GRID_Y_TO_RIG_X)
        # Belt colours by judgement level and four load-cell bars.
        self.belt_mats=[material(self.stage,'BeltN',(.018,.02,.023),.9),material(self.stage,'BeltS',(.018,.02,.023),.9)]
        for i,strap in enumerate(self.scene.straps):
            bind(strap,self.belt_mats[i])
        self.belt_shaders=[UsdShade.Shader(self.stage.GetPrimAtPath(m.GetPath().AppendChild('Shader'))) for m in self.belt_mats]
        self.belt_levels=[None,None]
        self.cell_mat=material(self.stage,'CellBar',(.1,.8,.9),.4)
        self.cell_shader=UsdShade.Shader(self.stage.GetPrimAtPath(self.cell_mat.GetPath().AppendChild('Shader')))
        self.cell_bars={}
        for name,x,y in [('NW',-.196,.125),('NE',-.196,-.125),('SW',.196,.125),('SE',.196,-.125)]:
            self.cell_bars[name]=shape(self.stage,f'{rig_root}/CellBar_{name}','box',(x,y*1.6,.20),(.03,.03,.02),self.cell_mat)
        self.cog=shape(self.stage,'/World/COG','sphere',(start[0],start[1],.95),(.025,.025,.025),self.scene.mats['yellow'])
        self.cog_mat=material(self.stage,'COGMarker',(1.,.9,.05),.4)
        bind(self.cog,self.cog_mat)
        self.cog_shader=UsdShade.Shader(self.stage.GetPrimAtPath(self.cog_mat.GetPath().AppendChild('Shader')))
        self.imu=self.world.scene.add(IMUSensor('/World/Cart/IMU',frequency=60,translation=self.rig_to_cart([.25,.12,.25])))
        self.camera=Camera('/World/Cart/DepthCamera',resolution=(320,240),frequency=10,translation=cam_pos)
        self.world.reset()
        self.world.stop()
        UsdPhysics.RigidBodyAPI(self.cargo.prim).CreateKinematicEnabledAttr(True)
        self.world.reset()
        self.camera.initialize()
        self.camera.set_local_pose(translation=cam_pos,orientation=cam_rot,camera_axes='world')
        self.camera.add_distance_to_image_plane_to_frame()
        self.camera.set_focal_length(1.93)
        self.camera.set_horizontal_aperture(3.896)
        self.camera.set_clipping_range(.05,100.)
        self.index=next((i for i,c in enumerate(self.cases) if c['id']=='20261015_212007'),0)
        self.playing=True;self.mode=0;self.t=0.;self.distance=0.;self.phase=0.;self.heading_prev=self.yaw
        self.body_roll=0.;self.body_pitch=0.;self.steer_shown=0.
        self.velocity=0.;self.previous_velocity=0.;self.frame_count=0;self.last_ui=0.
        self.cam_image=ui.ByteImageProvider();self.depth_image=ui.ByteImageProvider()
        self.real_depth=False          # depth_image shows the real D435i depth (else the simulated camera's)
        self.last_depth=None;self.latest={};self.last_export='';self.pending_capture=None
        self.recorder=None   # nothing is written until 'Start recording' is pressed
        self.shift=CargoShift()   # cargo-shift judgement from the COG history (cargo_shift.py)
        self.tilt_pivot=np.zeros(3);self.tilt_R=None   # slope / bank pivot (see retilt)
        self.imu_hist=collections.deque();self.imu_last=(np.array([0.,0.,1.]),np.zeros(3));self.settle_until=1.5
        self.belt_T=[0.,0.];self.belt_L0=[None,None];self.belt_L=[None,None]   # rope belts (belt_rope.py)
        self.stick=None;self.stick_pos=None;self.secured=False;self.grip=(0.,0.)   # static friction (secure_cargo)
        self.display=DisplaySender(ROOT/'data'/'live_runs')   # plays recordings on the display Pi
        # Live: the sensor Pi mirrors its display frames here (live_link.py) - the real bed as it is now
        self.live=LiveReceiver();self.live_on=False;self.live_count=0;self.live_cam=None;self.live_ts=None
        self.last_recording=None
        self.api=twin_api.TwinAPI(self)   # remote control (teammates' Claude), share mode
        self.models={};self.labels={};self.reset_requested=True
        self.pending_mode=None;self.export_requested=False;self.hq_state=None
        # Follow camera state: the vehicle displacement is added to the user's own view
        # instead of overwriting the view every frame (that fought Alt-orbit dragging).
        self.cam_offset=None;self.cam_dragging=False
        self.input=carb.input.acquire_input_interface()
        self.mouse=omni.appwindow.get_default_app_window().get_mouse()
        self.mouse_buttons=(carb.input.MouseInput.LEFT_BUTTON,carb.input.MouseInput.RIGHT_BUTTON,carb.input.MouseInput.MIDDLE_BUTTON)
        self.applied={};self.cam_shown=-1;self.perf_t=time.perf_counter();self.perf_frames=0;self.fps=0.
        self.build_ui()
        self.select_case(self.index,initial=True)
        self.apply_mode(0)
        self.set_camera(0)
        self.world.play()
        for name in ('Content','Console','Isaac Assets (Beta)','Stage','Layer','Property','Render Settings','Synthetic Data Recorder','Semantics Schema Editor','Semantic Schema Editor'):
            window=ui.Workspace.get_window(name)
            if window:
                window.visible=False
        viewport_window=ui.Workspace.get_window('Viewport')
        if viewport_window:
            self.window.dock_in(viewport_window,ui.DockPosition.RIGHT,.31)
        else:
            self.window.set_position(850,60)
        get_active_viewport().fill_frame=True
        self.add_logos(viewport_window)
        self.set_quality(DEFAULT_QUALITY)
        self.started=time.perf_counter()
        self.verification=[]
        self.protect_scene()
        print('FOREST_TWIN_READY',flush=True)
        if args.share:
            self.stream_res=args.stream_res
            urls=stream_urls()
            for url in urls:
                print('SHARE_URL',url,flush=True)
            self.last_export='Sharing (1 viewer): '+(urls[0] if urls else f'port {STREAM_PORT}')
            # Teammates download recordings from the same HTTP server as the stream page.
            try:
                import recordings_web
                recordings_web.register(ROOT,lambda:self.recorder['dir'] if self.recorder else None,
                                        to_display=lambda name:self.api.submit('send_to_display',value=name))
                self.api.register(ROOT,Path(__file__).with_name('isaac_twin_mcp.py'))
                base=urls[0].split('/twin/view')[0] if urls else None
                print('CONTROL_URL',base+'/twin/' if base else None,flush=True)
                self.download_url=base+'/recordings/' if base else None
                print('DOWNLOAD_URL',self.download_url,flush=True)
            except Exception:
                traceback.print_exc()
            # What the viewer actually gets (encode size, bitrate the stream's QoS settled on): /twin/state 'stream'
            self.stream_qos={}
            try:
                from omni.kit.livestream import bind as livestream_bind   # not 'import omni...': omni would turn local
                def on_qos(q):
                    self.stream_qos={'encode':f'{q.encode_width}x{q.encode_height}','bitrate_kbps':int(q.qos_bitrate),   # the SDK reports kbps (31000 seen at ~31 Mbps)
                                     
                                     'fps':q.target_streaming_fps,'rtt_ms':q.average_rtd_ms,'t':time.strftime('%H:%M:%S')}
                self.qos_live=livestream_bind.acquire_livestream_interface()
                self.qos_id=self.qos_live.register_qos_status_callback(on_qos)
            except Exception as e:
                print('STREAM_QOS unavailable',repr(e),flush=True)

    # ------------------------------------------------------------------ frames
    def rig_to_cart(self,v):
        """Rig-local (real plate, metres) coordinates -> vehicle-root coordinates (drawn rig_scale x larger)."""
        c,s=self.rig_cs;k=self.rig_scale
        return np.array([self.rig_t[0]+k*(c*v[0]-s*v[1]),self.rig_t[1]+k*(s*v[0]+c*v[1]),self.rig_t[2]+k*v[2]])

    def cart_to_rig(self,v):
        c,s=self.rig_cs
        d=(np.asarray(v,float)-self.rig_t)/self.rig_scale
        return np.array([c*d[0]+s*d[1],-s*d[0]+c*d[1],d[2]])

    def load_cache(self,case):
        if case['id'] not in self.caches:
            self.caches[case['id']]=load_cache(case['camera']['cache'])
        return self.caches[case['id']]

    def protect_scene(self):
        """Mark every scene prim no_delete so a Delete key press (e.g. by a remote viewer) is refused.

        Deleting /World/Cart/Deck invalidated the PhysX tensor views and ended the app."""
        from omni.usd.editor import set_no_delete
        count=0
        for prim in self.stage.Traverse():
            set_no_delete(prim,True)
            count+=1
        print(f'Protected {count} prims from deletion',flush=True)
        # Only the cargo can be selected (and so moved with the Move / Rotate tool): the ground, road, plate and
        # truck could be dragged up and down too (2026-11-01, user: "the floor height can be changed - stop it").
        ctx=omni.usd.get_context()
        self.selection_guard=ctx.get_stage_event_stream().create_subscription_to_pop(self.guard_selection,name='twin_selection_guard')

    MOVABLE=('/World/Cargo',)

    def guard_selection(self,event):
        if event.type!=int(omni.usd.StageEventType.SELECTION_CHANGED):
            return
        sel=omni.usd.get_context().get_selection()
        paths=list(sel.get_selected_prim_paths())
        keep=[p for p in paths if any(p==m or p.startswith(m+'/') for m in self.MOVABLE)]
        if keep!=paths:
            sel.set_selected_prim_paths(keep,False)

    def recover(self):
        """Best-effort restart of the simulation after an unexpected error."""
        self.labels['status'].text='장면 오류 - 시뮬레이션을 리셋했어요. 계속되면 앱을 다시 켜 주세요.'
        try:
            self.world.stop();self.world.reset();self.world.play()
        except Exception:
            pass
        self.reset_requested=True

    # ------------------------------------------------------------------ UI
    def slider(self, key, text, default, lo, hi):
        with ui.HStack(height=22):
            ui.Label(text,width=150)
            widget=ui.FloatSlider(min=lo,max=hi)
            widget.model.set_value(default)
            self.models[key]=widget.model
            self.ranges=getattr(self,'ranges',{});self.ranges[key]=(lo,hi,text)
        return widget.model

    def toggle(self,key,text,default):
        with ui.HStack(height=22):
            ui.Label(text,width=210)
            check=ui.CheckBox(width=24)
            check.model.set_value(default);self.models[key]=check.model
            self.toggles=getattr(self,'toggles',{});self.toggles[key]=text
        return check.model

    def build_ui(self):
        title='적재적소 | 트럭 트윈' if TRUCK else '적재적소 | 대차 트윈'
        self.window=ui.Window(title,width=390,height=1000,position_x=850,position_y=60)
        speed_label='순항 속도 (km/h, - 후진)' if TRUCK else '목표 속도 (m/s)'
        speed_default,speed_lo,speed_hi=(20.,-10.,60.) if TRUCK else (1.4,-2.5,2.5)
        # Korean panel text: isaac_twin_light.kit loads Malgun Gothic for the glyphs listed in
        # assets/fonts/korean_glyphs.txt (omni.ui region file, 64 characters per line) - add new characters there.
        with self.window.frame:
            with ui.ScrollingFrame():
                with ui.VStack(spacing=5,height=0):
                    ui.Label('HYUNDAI XCIENT / 적재적소 화물 센서 트윈' if TRUCK else 'HAND CART / 적재적소 화물 센서 트윈',height=27,style={'font_size':18,'color':0xffc9ead5})
                    with ui.HStack(height=28):
                        ui.Button('측정 재생',clicked_fn=lambda:self.set_mode(0))
                        ui.Button('물리 실험',clicked_fn=lambda:self.set_mode(1))
                    combo=ui.ComboBox(self.index,*[f"{c['id'][4:6]}/{c['id'][6:8]} {c['id'][9:11]}:{c['id'][11:13]}  {c['title']}" for c in self.cases],height=24)
                    combo.model.add_item_changed_fn(lambda m,_:self.select_case(m.get_item_value_model().as_int))
                    self.case_model=combo.model
                    with ui.HStack(height=26):
                        ui.Button('재생 / 일시정지',clicked_fn=self.toggle_play)
                        ui.Button('처음부터',clicked_fn=self.request_reset)
                        ui.Button('내보내기',clicked_fn=self.request_export)
                        ui.Button('고화질 사진',clicked_fn=self.request_hq)
                    with ui.HStack(height=28):
                        self.rec_button=ui.Button('기록 시작',clicked_fn=self.toggle_recording,width=150,
                                                  style={'background_color':0xff2d6b2d})
                        self.labels['rec']=ui.Label('  기록 안 하는 중',style={'color':0xff9a9a9a})
                    with ui.HStack(height=26):
                        self.display_button=ui.Button('디스플레이로 보내기',clicked_fn=self.toggle_display,width=150)
                        self.labels['display']=ui.Label('  디스플레이: 꺼짐',style={'color':0xff9a9a9a},word_wrap=True)
                    with ui.HStack(height=26):
                        self.live_button=ui.Button('실시간 받기',clicked_fn=self.set_live,width=150)
                        self.labels['live']=ui.Label('  실시간: 꺼짐',style={'color':0xff9a9a9a},word_wrap=True)
                    self.slider('time','재생 위치 (초)',0,0,max(self.cases[self.index]['duration'],.1)).add_value_changed_fn(self.seek)
                    self.slider('rate','재생 속도 (배)',1,.1,3)
                    self.toggle('loop','끝나면 반복 재생',True)
                    self.toggle('cargo_boxes','화물을 상자로 (디스플레이 파이 방식)',True)
                    self.labels['source']=ui.Label('',height=37,word_wrap=True)
                    with ui.CollapsableFrame('도로 · 주행',collapsed=False):
                        with ui.VStack(spacing=3):
                            self.slider('grade','도로 경사 (도, + 오르막)',0,-25,25)
                            self.slider('bank','도로 좌우 기울기 (도)',0,-15,15)
                            self.slider('speed',speed_label,speed_default,speed_lo,speed_hi)
                            self.slider('accel','가속 · 제동 (m/s²)',1.2 if TRUCK else .8,.1,5)
                            self.slider('steer','회전 (도/초, + 왼쪽)',0,-50,50)
                            self.slider('lateral','차선 위치 (m, + 왼쪽)',self.lane_default,-6.,6.).add_value_changed_fn(lambda m:self.set_lane(m.as_float))
                            self.toggle('imu_motion','재생: 기록된 IMU대로 달리기',True)
                            self.slider('turn_gain','재생: 회전 반영 비율',0.,0,1)    # 0: only the user turns the truck
                            self.slider('tilt_gain','차체 기울기 (도/G)',4.,0,10)
                            if not TRUCK:
                                self.toggle('pull','대차를 앞에서 끌기',True)
                            with ui.HStack(height=23):
                                ui.Button('출발',clicked_fn=self.go)
                                ui.Button('정지',clicked_fn=self.brake)
                                ui.Button('좌회전',clicked_fn=lambda:self.turn(90))
                                ui.Button('우회전',clicked_fn=lambda:self.turn(-90))
                                ui.Button('직진',clicked_fn=self.straight)
                    with ui.CollapsableFrame('화물 · 벨트 · 물리',collapsed=True):
                        with ui.VStack(spacing=3):
                            self.slider('mass','화물 무게 (kg)',14,.1,30)
                            self.slider('cartmass','차량 무게 (kg)',1800 if TRUCK else 7,1,3000 if TRUCK else 30)
                            self.slider('friction','바닥 마찰 계수',.6,.05,1.5)
                            self.slider('cargo_x','화물 앞뒤 (m, + 뒤쪽)',0,-.3,.3)
                            self.slider('cargo_y','화물 좌우 (m, + 오른쪽)',0,-.2,.2)
                            # Each belt has its own preload and hook, and its own tension cell (c1 front, c2 rear).
                            # 'Strap preload' / 'Straps connected' set both belts at once (and the API keeps them).
                            both=self.slider('pretension','두 벨트 조임 (N)',8,0,TENSION_MAX_N)
                            self.slider('pretension_c1','앞벨트 c1 조임 (N)',8,0,TENSION_MAX_N)
                            self.slider('pretension_c2','뒷벨트 c2 조임 (N)',8,0,TENSION_MAX_N)
                            both.add_value_changed_fn(lambda m:[self.models[k].set_value(m.as_float) for k in ('pretension_c1','pretension_c2')])
                            self.slider('stiffness','벨트 강성 (N/m)',1500,100,5000)
                            both=self.toggle('straps','두 벨트 연결',True)
                            self.toggle('straps_c1','앞벨트 c1 연결',True)
                            self.toggle('straps_c2','뒷벨트 c2 연결',True)
                            both.add_value_changed_fn(lambda m:[self.models[k].set_value(m.as_bool) for k in ('straps_c1','straps_c2')])
                            self.toggle('scan','스캔한 화물 모양 (물리 실험)',True)
                            with ui.HStack(height=23):
                                ui.Button('벨트 모두 풀기',clicked_fn=lambda:self.set_straps(False))
                                ui.Button('화물 제자리에 다시 놓기',clicked_fn=self.request_reset)
                    with ui.CollapsableFrame('판정 · 센서',collapsed=False):
                        with ui.VStack(spacing=2):
                            self.labels['mode']=ui.Label('',height=20)
                            self.labels['overall']=ui.Label('',height=26,style={'font_size':17})
                            for key in ('load','cog','move','belts'):
                                self.labels[key]=ui.Label('',height=22,word_wrap=True)
                            self.labels['cells']=ui.Label('',height=34,word_wrap=True)
                            self.labels['imu']=ui.Label('',height=34,word_wrap=True)
                            self.labels['motion']=ui.Label('',height=34,word_wrap=True)
                            self.labels['camera']=ui.Label('',height=22,word_wrap=True)
                            self.labels['warning']=ui.Label('',height=46,word_wrap=True)
                    with ui.CollapsableFrame('시점',collapsed=False):
                        with ui.VStack(spacing=3):
                            self.toggle('follow','트럭 따라가기',True)
                            with ui.HStack(height=26):
                                for index,label in enumerate(self.VIEWS):
                                    ui.Button(self.VIEWS_KO[index],clicked_fn=lambda n=index:self.set_camera(n))
                            self.slider('zoom','카메라 거리 (m)',getattr(T,'VIEW_DISTANCE',9.) if TRUCK else 4,2,40).add_value_changed_fn(lambda m:self.snap_camera())
                            with ui.HStack(height=26):
                                ui.Label('화질',width=50)
                                for key,label in (('fast','빠름'),('balanced','보통'),('high','고화질')):
                                    ui.Button(label,clicked_fn=lambda k=key:self.set_quality(k))
                            self.labels['perf']=ui.Label('',height=20)
                    with ui.CollapsableFrame('D435i 카메라',collapsed=False):
                        with ui.VStack(spacing=3):
                            self.labels['cam_title']=ui.Label('기록된 컬러 영상 (센서와 같은 시각)',height=20)
                            ui.ImageWithProvider(self.cam_image,height=200,fill_policy=ui.IwpFillPolicy.IWP_PRESERVE_ASPECT_FIT)
                            self.labels['depth_title']=ui.Label('시뮬레이션 깊이 영상 (m)',height=20)
                            ui.ImageWithProvider(self.depth_image,height=180,fill_policy=ui.IwpFillPolicy.IWP_PRESERVE_ASPECT_FIT)
                    self.labels['status']=ui.Label('준비됨',height=48,word_wrap=True)
        self._setting_time=False;self.camera_view=0

    def value(self,key):
        return self.models[key].as_float

    D435I_MIN_SPAN_MM=150      # depth colours span the frame's own 2-98 % range, at least this wide

    def add_d435i_view(self,viewport_window):
        """The 깊이 카메라 view: the real D435i colour and depth pictures (live from the sensor Pi, or the recording)
        over the viewport (2026-11-01, user: "show the D435i's own picture"). Hidden in the other views and when
        there is no real frame (physics mode): then the simulated depth camera's view shows as before."""
        if not viewport_window or not hasattr(viewport_window,'get_frame'):
            return
        self.d435i_frame=viewport_window.get_frame('twin_d435i')
        with self.d435i_frame:
            with ui.ZStack():
                ui.Rectangle(style={'background_color':0xff141414})
                with ui.VStack(spacing=6):
                    ui.Spacer(height=84)                 # below the toolbar and the logos
                    self.labels['d435i_title']=ui.Label('',height=26,alignment=ui.Alignment.CENTER,
                                                        style={'font_size':20,'color':0xffe0e0e0})
                    with ui.HStack(spacing=12):
                        ui.Spacer(width=8)
                        for title,provider in (('컬러 영상',self.cam_image),('깊이 영상 (빨강 = 가까움, 파랑 = 멂)',self.depth_image)):
                            with ui.VStack(spacing=4):
                                ui.Label(title,height=22,alignment=ui.Alignment.CENTER,style={'font_size':16,'color':0xffc8c8c8})
                                ui.ImageWithProvider(provider,fill_policy=ui.IwpFillPolicy.IWP_PRESERVE_ASPECT_FIT)
                        ui.Spacer(width=8)
                    ui.Spacer(height=16)
        self.d435i_frame.visible=False

    def depth_rgba(self,depth_mm):
        """Real depth (mm, 0 = no reading) -> turbo colours, near red / far blue, no reading black. The range is the
        frame's own (the tilted camera sees 250-900 mm across the picture; a fixed scale left the cargo all red)."""
        d=np.asarray(depth_mm,np.float32)
        valid=d[d>0]
        near,far=(np.percentile(valid,(2,98)) if valid.size else (300.,600.))
        far=max(far,near+self.D435I_MIN_SPAN_MM)
        x=np.clip((far-d)/(far-near),0,1)
        rgb=cv2.applyColorMap(np.uint8(x*255),cv2.COLORMAP_TURBO)
        rgb[d<=0]=0
        return cv2.cvtColor(rgb,cv2.COLOR_BGR2RGBA)

    def add_logos(self,viewport_window):
        """Overlay the two transparent logos in the viewport's top corners (window only, not in captures)."""
        if not viewport_window or not hasattr(viewport_window,'get_frame'):
            return
        self.add_d435i_view(viewport_window)       # first, so the logos stay on top of it
        self.logo_frame=viewport_window.get_frame('twin_logos')
        with self.logo_frame:
            with ui.VStack():
                ui.Spacer(height=40)          # below the viewport's own toolbar
                with ui.HStack(height=0):
                    for k,(path,w,h) in enumerate(LOGOS):
                        if k:
                            ui.Spacer()
                        else:
                            ui.Spacer(width=8)
                        if not path.exists():
                            continue
                        ui.Image(str(path),width=w,height=h,fill_policy=ui.FillPolicy.PRESERVE_ASPECT_FIT)
                    ui.Spacer(width=8)
                ui.Spacer()

    def set_quality(self,name):
        """Viewport render quality. The shimmer ('지지직') on the truck's thin parts came from rendering a small image
        (DLSS quality upscales ~0.67 of the viewport resolution) and from 1-sample sampled lighting; 'balanced' uses DLAA
        (DLSS anti-aliasing at native resolution) and 2 light samples per pixel, 'high' also renders 1.5x and shrinks."""
        q=QUALITY.get(name)
        if q is None:
            return False
        settings=carb.settings.get_settings()
        for key,value in q['settings'].items():
            settings.set(key,value)
        try:   # the viewport fills its window; scale > 1 renders a bigger image and shrinks it (supersampling)
            get_active_viewport().resolution_scale=q['scale']
        except Exception as error:
            print('QUALITY resolution scale not set:',error,flush=True)
        self.quality=name
        self.last_export=f'Render quality: {name}'
        print('QUALITY',name,flush=True)
        return True

    def changed(self,key,value):
        if self.applied.get(key)==value:
            return False
        self.applied[key]=value
        return True

    def flag(self,key):
        return self.models[key].as_bool if key in self.models else True

    def toggle_play(self):
        self.playing=not self.playing

    def brake(self):
        # Remember the cruise speed so Go can resume it.
        if abs(self.value('speed'))>.05:
            self.cruise_speed=self.value('speed')
        self.models['speed'].set_value(0)

    def go(self):
        # Go also un-pauses: on a paused twin it only set the cruise speed and the truck stood still
        self.models['speed'].set_value(getattr(self,'cruise_speed',20. if TRUCK else 1.4))
        self.playing=True

    def request_reset(self):
        self.reset_requested=True

    def set_lane(self,value):
        """Put the truck into that lane of the loop (in its travel direction), lined up with the road."""
        self.update_pose()
        self.dir=1 if abs(self.psi)<=90 else -1
        xy,heading=self.road.frenet_to_xy(self.s,self.dir*value)
        self.xy=np.array(xy);self.yaw=math.degrees(heading)+(0. if self.dir>0 else 180.)
        self.turn_target=None;self.yaw_rate_f=0.
        self.update_pose()
        self.place_cargo()

    TURN_RATE=18.     # deg/s of a turn: 90 deg in ~5 s, about how a loaded truck takes a junction
    TURN_A_MAX=2.5    # m/s^2: at speed a turn is no tighter than this lateral acceleration (60 km/h -> ~9 deg/s)

    def turn(self,degrees):
        """Turn left (+) / right (-) by that many degrees, starting now, then drive straight on."""
        self.models['steer'].set_value(0)
        self.turn_target=float(degrees)          # degrees still to turn, counted down as the heading changes
        self.turn_sign=1. if self.turn_target>=0 else -1.
        return {'ok':True,'message':f'turning {float(degrees):+.0f} deg'}

    def straight(self):
        """Stop any turn and keep the current heading."""
        self.models['steer'].set_value(0)
        self.turn_target=None
        return {'ok':True,'message':'straight on'}

    def update_pose(self):
        """Road coordinates (s, d, psi) of the world pose (xy, yaw)."""
        self.s,self.d,th=self.road.project(float(self.xy[0]),float(self.xy[1]))
        self.psi=(self.yaw-math.degrees(th)+540.)%360.-180.

    def seek(self,model):
        if not getattr(self,'_setting_time',False):
            self.t=model.as_float
            self.shift.reset()   # a jump in the recording invalidates the reference

    def select_case(self,index,initial=False):
        if getattr(self,'live_on',False):
            self.set_live(False)      # a recording was picked: back to replay
        self.index=index;self.t=0.;self.reset_requested=True
        case=self.cases[index]
        self.models['time'].set_max(max(case['duration'],.1))
        cache=self.load_cache(case)
        self.surface.set_camera(cache['camera'],cache['grid'])
        self.surface_index=-1
        if case.get('load_kg_mean'):
            self.models['mass'].set_value(float(case['load_kg_mean']))
        if not initial:
            self.labels['status'].text=f"{case['id']}  {case['title']}"

    def set_mode(self,mode):
        # UI callbacks run inside Kit's render update; reset only between frames.
        self.pending_mode=mode

    def apply_mode(self,mode):
        if mode!=0 and self.live_on:
            self.set_live(False)
        self.mode=mode;self.reset_requested=True;self.cam_shown=-1
        algo=self.team.algo if self.team else '트윈 규칙 (team/ 동기화 안 됨)'
        self.labels['mode'].text=(f'측정 재생 - 판정: {algo}' if mode==0 else f'물리 실험 (PhysX) - 판정: {algo}')
        self.labels['source'].text=('기록된 센서값을 그대로 재생. 트럭 움직임은 기록된 IMU를 따름 (회전은 비율만큼).' if mode==0 else
                                   '조건을 바꿔 보는 실험. 센서값은 모델로 계산 (실물 보정 전).')
        self.world.stop()
        UsdPhysics.RigidBodyAPI(self.cargo.prim).GetKinematicEnabledAttr().Set(mode==0)
        self.world.reset()
        self.world.play()
        self.show_measured(mode==0)

    def show_measured(self,measured):
        if self.changed('measured_visible',measured):
            img=UsdGeom.Imageable(self.surface.prim)
            img.MakeVisible() if measured else img.MakeInvisible()
            self.applied.pop('scan',None)   # re-evaluate scan/proxy visibility
            self.applied.pop('cargo_view',None)

    # ------------------------------------------------------------------ cameras
    # Camera presets as offsets in the vehicle frame (x forward, y left, z up) times the distance slider.
    # The default is a high front-quarter view from over the road: cab, driver, bed and cargo are all in
    # sight and nothing on the truck can get between the camera and them.
    VIEWS=('Front','Rear','Side','Aerial','Depth','Cab')     # API names
    VIEWS_KO=('앞','뒤','옆','위','깊이 카메라','운전석')
    # Chase views sit ~20 deg up so the tree line and sky fill the top of the picture (2026-11-01, user: "an angle
    # that always shows nature"; the old 36-40 deg views showed only the pad). They look straight at the truck, and
    # a lens shift (the camera's vertical aperture offset) moves the picture up LENS_SHIFT_DEG instead: aiming at a
    # point above the truck made zooming / orbiting go toward that empty point (user: "odd when zoomed").
    PRESETS={0:(.72,.66,.40),1:(-.82,.5,.42),2:(.05,.95,.30),3:(.02,0.,2.5)}
    LENS_SHIFT_DEG=8.
    DEPTH_VIEW,CAB_VIEW=4,5

    def look_target(self,center,offset):
        """The chase camera always aims at the truck's centre, so Kit's zoom and orbit centre on the truck."""
        return center

    def set_lens_shift(self):
        """Shift the viewport picture up in the chase views (front / rear / side), none from above or the cab."""
        cam=UsdGeom.Camera(self.stage.GetPrimAtPath('/OmniverseKit_Persp'))
        if not cam:
            return
        deg=self.LENS_SHIFT_DEG if self.camera_view in (0,1,2) else 0.
        focal=cam.GetFocalLengthAttr().Get() or 18.147
        attr=cam.GetVerticalApertureOffsetAttr() or cam.CreateVerticalApertureOffsetAttr()
        attr.Set(float(focal*math.tan(math.radians(deg))))

    def set_camera(self,index):
        self.camera_view=index
        if index==self.DEPTH_VIEW:
            get_active_viewport().camera_path='/World/Cart/DepthCamera'
        else:
            get_active_viewport().camera_path='/OmniverseKit_Persp'
            self.snap_camera()

    def cart_center(self):
        return self.world_point(np.array([self.xy[0],self.xy[1],getattr(T,'CENTER_Z',1.3) if TRUCK else .75]))

    def world_to_vehicle(self,v):
        """Inverse of cart_vector: world vector -> vehicle frame (heading, road slope and bank removed)."""
        grade=math.radians(self.value('grade'));bank=math.radians(self.value('bank'))
        rx=np.array([[1,0,0],[0,math.cos(bank),-math.sin(bank)],[0,math.sin(bank),math.cos(bank)]])
        ry=np.array([[math.cos(grade),0,-math.sin(grade)],[0,1,0],[math.sin(grade),0,math.cos(grade)]])
        local=(ry@rx).T@np.asarray(v,float)
        a=math.radians(self.yaw)
        return np.array([[math.cos(a),math.sin(a),0],[-math.sin(a),math.cos(a),0],[0,0,1]])@local

    def place_camera(self,eye,target):
        """Aim the viewport camera without going through the undoable Kit commands (called every frame)."""
        prim=self.stage.GetPrimAtPath('/OmniverseKit_Persp')
        ops=UsdGeom.Xformable(prim).GetOrderedXformOps() if prim else []
        if len(ops)==1 and ops[0].GetOpType()==UsdGeom.XformOp.TypeTransform:
            eye_v=Gf.Vec3d(*map(float,eye));target_v=Gf.Vec3d(*map(float,target))
            ops[0].Set(Gf.Matrix4d(1).SetLookAt(eye_v,target_v,Gf.Vec3d(0,0,1)).GetInverse())
            coi=prim.GetAttribute('omni:kit:centerOfInterest')
            if coi:
                coi.Set(Gf.Vec3d(0,0,-float(np.linalg.norm(np.asarray(target)-np.asarray(eye)))))
        else:
            set_camera_view(eye=eye,target=target,camera_prim_path='/OmniverseKit_Persp')

    def snap_camera(self):
        """Jump to a preset view (view buttons and the distance slider)."""
        if self.camera_view==self.DEPTH_VIEW:
            return
        center=self.cart_center()
        zoom=self.value('zoom') if 'zoom' in self.models else 4.
        if self.camera_view==self.CAB_VIEW and TRUCK:
            eye=self.world_point([*self.xy,0])+self.cart_vector([T.DRIVER['root'][0]+.08,T.DRIVER['root'][1],T.DRIVER['root'][2]+1.22])
            target=eye+self.cart_vector([8.,0,-.6])
            self.cam_offset=None
        else:
            self.cam_offset=np.array(self.PRESETS.get(self.camera_view,self.PRESETS[0]))*zoom
            eye=center+self.cart_vector(self.cam_offset);target=self.look_target(center,self.cam_offset)
        set_camera_view(eye=eye,target=target,camera_prim_path='/OmniverseKit_Persp')
        self.set_lens_shift()
        self.cam_dragging=False

    def user_navigating(self):
        try:
            return any(self.input.get_mouse_value(self.mouse,b)>0 for b in self.mouse_buttons)
        except Exception:
            return False

    def follow_camera(self):
        """Chase camera: the view keeps its offset in the *vehicle* frame, so it turns with the truck and
        a chosen angle (preset or the user's own Alt-orbit) stays clear of the cargo around the whole loop.

        While a mouse button is held the camera belongs to Kit's manipulator; on release the new
        position is adopted as the offset to keep."""
        if self.camera_view==self.DEPTH_VIEW or not self.flag('follow'):
            self.cam_offset=None;self.cam_dragging=False
            return
        if self.user_navigating():
            self.cam_dragging=True
            return
        center=self.cart_center()
        if self.camera_view==self.CAB_VIEW and TRUCK:
            if self.cam_dragging:
                self.camera_view=0            # the user orbited away from the cab: continue as a free chase view
                self.set_lens_shift()
            else:
                eye=self.world_point([*self.xy,0])+self.cart_vector([T.DRIVER['root'][0]+.08,T.DRIVER['root'][1],T.DRIVER['root'][2]+1.22])
                self.place_camera(eye,eye+self.cart_vector([8.,0,-.6]))
                return
        if self.cam_dragging or self.cam_offset is None:
            state=ViewportCameraState('/OmniverseKit_Persp',get_active_viewport())
            offset=self.world_to_vehicle(np.array(state.position_world)-center)
            if np.linalg.norm(offset)<2.5:   # inside the truck or the cargo: back off to the default view
                offset=np.array(self.PRESETS[0])*max(self.value('zoom'),4.)
            self.cam_offset=offset;self.cam_dragging=False
        self.place_camera(center+self.cart_vector(self.cam_offset),self.look_target(center,self.cam_offset))

    def to_world(self,vec):
        grade=math.radians(self.value('grade'));bank=math.radians(self.value('bank'))
        rx=np.array([[1,0,0],[0,math.cos(bank),-math.sin(bank)],[0,math.sin(bank),math.cos(bank)]])
        ry=np.array([[math.cos(grade),0,-math.sin(grade)],[0,1,0],[math.sin(grade),0,math.cos(grade)]])
        return ry@rx@np.asarray(vec)

    def world_point(self,point):
        """Road-frame point -> world: the slope / bank rotation turns about tilt_pivot (the deck), not the origin."""
        return self.to_world(np.asarray(point,float)-self.tilt_pivot)+self.tilt_pivot

    def retilt(self):
        """Apply a new road slope / bank. The world used to turn about the origin, so with the truck ~100 m away
        a 12 deg change moved the deck ~20 m in one frame and threw the strapped cargo off (belts 7500 N).
        Now it turns about the deck, and the cargo is carried along so only the new slope acts on it."""
        from omni.isaac.core.utils.rotations import quat_to_rot_matrix, rot_matrix_to_quat
        new_R=np.column_stack([self.to_world(e) for e in np.eye(3)])
        new_pivot=np.array([self.xy[0],self.xy[1],T.BED_TOP if TRUCK else .5])
        if self.tilt_R is not None and getattr(self,'cargo',None) is not None:
            pos,quat=self.cargo.get_world_pose()
            road=self.tilt_R.T@(np.asarray(pos)-self.tilt_pivot)+self.tilt_pivot
            turn=new_R@self.tilt_R.T
            self.cargo.set_world_pose(new_R@(road-new_pivot)+new_pivot,rot_matrix_to_quat(turn@quat_to_rot_matrix(np.asarray(quat))))
            if self.cargo_dynamic():
                self.cargo.set_linear_velocity(turn@np.asarray(self.cargo.get_linear_velocity()))
                self.cargo.set_angular_velocity(turn@np.asarray(self.cargo.get_angular_velocity()))
        if self.tilt_R is not None and not np.allclose(new_R,self.tilt_R):
            self.shift.rebias()   # the IMU now reads a different gravity component
        self.tilt_R,self.tilt_pivot=new_R,new_pivot
        pose(self.scene.environment,new_pivot-new_R@new_pivot,(self.value('bank'),-self.value('grade'),0))

    def cart_vector(self,vec):
        angle=math.radians(self.yaw)
        local=np.array([[math.cos(angle),-math.sin(angle),0],[math.sin(angle),math.cos(angle),0],[0,0,1]])@np.asarray(vec)
        return self.to_world(local)

    # ------------------------------------------------------------------ simulation
    def reset(self):
        self.shift.reset()
        self.imu_last=(np.array([0.,0.,1.]),np.zeros(3))
        self.imu_hist=collections.deque();self.tilt=None
        self.settle_until=1.5
        self.belt_T=[0.,0.];self.belt_L0=[None,None];self.belt_L=[None,None]
        self.stick=None;self.stick_pos=None;self.secured=False
        # forget the last seen USD pose: in replay the kinematic cargo's USD pose moves every frame, and coming
        # back to the sandbox that change read as a mouse move and teleported the cargo to an old place
        self.cargo_seen=None;self.cargo_pending=None;self.rig_origin=None;self.release_v=None
        if self.mode==1:
            self.set_cargo_kinematic(False)
        if self.team_live is not None:
            self.team_live.reset();self.team_t=-1.;self.team_out={};self.team_clock=0.
        self.t=0.;self.distance=0.;self.velocity=0.
        self.s=self.road.start_s;self.d=self.value('lateral');self.psi=0.
        self.dir=1;self.turn_target=None;self.yaw_rate_f=0.;self.gz_bias=None
        xy,heading=self.road.frenet_to_xy(self.s,self.d)
        self.xy=np.array(xy);self.yaw=math.degrees(heading)
        self.update_pose();self.heading_prev=self.yaw
        self.body_roll=self.body_pitch=0.
        self.place_cargo()
        self.reset_requested=False

    def cargo_desired(self):
        base=self.world_point([*self.xy,0])
        return base+self.cart_vector(self.rig_to_cart([self.value('cargo_x'),self.value('cargo_y'),.515]))

    def vehicle_rotation(self,yaw_deg):
        """World rotation of the vehicle: tilted road (slope / bank, as the environment) x heading x body roll /
        pitch. The Euler form rotation(bank+roll, -grade+pitch, yaw) applied the heading after the road tilt, so on a
        slope the truck was not parallel to the road once it turned and the plate frame drifted from the vehicle
        frame used for the cells (locked cargo read as sliding up to 10 cm in curves)."""
        from omni.isaac.core.utils.rotations import rot_matrix_to_quat
        road=np.column_stack([self.to_world(e) for e in np.eye(3)])
        a=math.radians(yaw_deg);p=math.radians(self.body_pitch);r=math.radians(self.body_roll)
        rz=np.array([[math.cos(a),-math.sin(a),0],[math.sin(a),math.cos(a),0],[0,0,1]])
        ry=np.array([[math.cos(p),0,math.sin(p)],[0,1,0],[-math.sin(p),0,math.cos(p)]])
        rx=np.array([[1,0,0],[0,math.cos(r),-math.sin(r)],[0,math.sin(r),math.cos(r)]])
        return rot_matrix_to_quat(road@rz@ry@rx)

    def body_orientation(self):
        return self.vehicle_rotation(self.yaw)

    def cargo_orientation(self):
        return self.vehicle_rotation(self.yaw+self.rig_yaw)

    @staticmethod
    def as_array(m):
        return np.array([[m[i][j] for j in range(4)] for i in range(4)])

    def collect_scan_points(self,limit=4000):
        """Vertices of the scanned cargo in the cargo body's frame (homogeneous rows), to drape the belts on."""
        cache=UsdGeom.XformCache(Usd.TimeCode.Default())
        to_cargo=self.as_array(cache.GetLocalToWorldTransform(self.stage.GetPrimAtPath('/World/Cargo')).GetInverse())
        chunks=[]
        for prim in Usd.PrimRange(self.scan):
            if prim.IsA(UsdGeom.Mesh):
                pts=UsdGeom.Mesh(prim).GetPointsAttr().Get()
                if pts:
                    pts=np.asarray(pts,float)
                    world=np.hstack([pts,np.ones((len(pts),1))])@self.as_array(cache.GetLocalToWorldTransform(prim))
                    chunks.append(world@to_cargo)
        if not chunks:
            return None
        pts=np.vstack(chunks)
        if len(pts)>limit:
            pts=pts[np.random.default_rng(1).choice(len(pts),limit,replace=False)]
        return pts

    def user_moved_cargo(self):
        """Cargo moved with the mouse in the viewport (Move / Rotate tool) becomes a real move of the PhysX cargo
        when the mouse button is released, so the cells, belts and judgement follow it. Clicking the cargo usually
        selects the scanned boxes (a child of the cargo body): before, moving them only moved the picture, and a
        move of the body itself was overwritten by PhysX on the next step."""
        cache=UsdGeom.XformCache(Usd.TimeCode.Default())
        body=self.stage.GetPrimAtPath('/World/Cargo')
        held=self.user_navigating()
        # PhysX does not write the body's pose back to USD here: the USD transform of /World/Cargo stays where the
        # body was created, so the true body transform comes from PhysX and USD only shows what the user edits.
        body_usd=cache.GetLocalToWorldTransform(body)
        pos,quat=self.cargo.get_world_pose()
        sx,sy,sz=(float(v) for v in self.cargo.get_local_scale())
        body_true=(Gf.Matrix4d().SetScale(Gf.Vec3d(sx,sy,sz))*Gf.Matrix4d().SetRotate(Gf.Quatd(float(quat[0]),Gf.Vec3d(*map(float,quat[1:]))))
                   *Gf.Matrix4d().SetTranslate(Gf.Vec3d(*map(float,pos))))
        # 1. a part of the cargo (scan / collider) was moved: apply the same world motion to the body
        for path,home in self.cargo_parts.items():
            prim=self.stage.GetPrimAtPath(path)
            if not prim:
                continue
            local=UsdGeom.Xformable(prim).GetLocalTransformation()
            if Gf.IsClose(local,home,1e-6) or held:
                continue
            parent=cache.GetParentToWorldTransform(prim)*body_usd.GetInverse()*body_true   # where the part really is
            motion=(home*parent).GetInverse()*(local*parent)          # world-space move the user applied
            target=body_true*motion
            for other,other_home in self.cargo_parts.items():        # put the parts back on the body
                q=self.stage.GetPrimAtPath(other)
                if q and not Gf.IsClose(UsdGeom.Xformable(q).GetLocalTransformation(),other_home,1e-6):
                    UsdGeom.Xformable(q).MakeMatrixXform().Set(other_home)
            self.teleport_cargo(target)
            return
        # 2. the body itself was moved: its USD transform changed since the last frame (USD-to-USD, never against
        # PhysX - the USD pose is stale) and differs from where PhysX has the body
        here=np.array(body_usd.ExtractTranslation())
        if self.cargo_seen is not None and np.linalg.norm(here-self.cargo_seen)>1e-3 and np.linalg.norm(here-np.asarray(pos,float))>.01:
            self.cargo_pending=Gf.Matrix4d(body_usd)
        self.cargo_seen=here
        if self.cargo_pending is not None and not held:
            self.teleport_cargo(self.cargo_pending);self.cargo_pending=None

    def replay_cargo_moved(self):
        """Measured replay plays the recorded load cells, so a cargo moved there with the mouse only moved the
        picture and the COG stayed the recorded one. On release the twin now switches to the physics sandbox and
        puts the cargo at the same place on the plate; the cells, belts, COG and judgement follow its position."""
        held=self.user_navigating()
        cache=UsdGeom.XformCache(Usd.TimeCode.Default())
        body=cache.GetLocalToWorldTransform(self.stage.GetPrimAtPath('/World/Cargo'))
        here=np.array(body.ExtractTranslation())
        for path,home in self.cargo_parts.items():     # a part (scanned boxes) moved: same as user_moved_cargo
            prim=self.stage.GetPrimAtPath(path)
            if not prim or Gf.IsClose(UsdGeom.Xformable(prim).GetLocalTransformation(),home,1e-6):
                continue
            if held:
                return
            parent=cache.GetParentToWorldTransform(prim)
            local=UsdGeom.Xformable(prim).GetLocalTransformation()
            motion=(home*parent).GetInverse()*(local*parent)
            self.replay_move=np.array((body*motion).ExtractTranslation())-here
            for other,other_home in self.cargo_parts.items():
                q=self.stage.GetPrimAtPath(other)
                if q and not Gf.IsClose(UsdGeom.Xformable(q).GetLocalTransformation(),other_home,1e-6):
                    UsdGeom.Xformable(q).MakeMatrixXform().Set(other_home)
            break
        else:
            # the body itself dragged: replay rewrites its pose every frame, so anything else is the user's edit
            if self.cargo_written is not None and np.linalg.norm(here-self.cargo_written)>1e-3:
                self.replay_move=here-self.cargo_written
        if self.replay_move is None or held:
            return
        d=self.rig_matrix().GetInverse().TransformDir(Gf.Vec3d(*map(float,self.replay_move)))
        self.replay_move=None
        self.queued_move=(float(d[1])*1000,float(d[0])*1000)   # rig y = right, rig x = towards the rear belt
        self.pending_mode=1
        self.last_export='Measured replay keeps the recorded sensors: switching to the physics sandbox to move the cargo'
        print('REPLAY_CARGO_MOVED',np.round(self.queued_move,1).tolist(),flush=True)

    def teleport_cargo(self,matrix):
        """Put the PhysX cargo at a world transform (scale ignored), at rest; the belts keep their length, so a
        cargo pushed sideways under them pulls them tighter."""
        m=Gf.Matrix4d(matrix).RemoveScaleShear()
        q=m.ExtractRotationQuat()
        pos=np.array(m.ExtractTranslation())
        quat=np.array([q.GetReal(),*q.GetImaginary()])
        self.pause_settle=1.
        if self.mode==1 and self.stick is not None:
            # a held cargo slid over the plate (same height): held again at once at the new place. Let go as a
            # dynamic body for a frame it lost the deck's speed and slid 17 mm rearward on a driving truck.
            M=self.rig_matrix();inv=M.GetInverse()
            old,_=self.cargo.get_world_pose()
            rise=inv.Transform(Gf.Vec3d(*map(float,pos)))[2]-inv.Transform(Gf.Vec3d(*map(float,old)))[2]
            if abs(rise)<.005:
                kinematic_pose(self.cargo,pos,quat)
                self.stick=Gf.Matrix4d(m)*inv;self.stick_pos=pos
                self.belt_L=[None,None];self.cargo_physx=pos
                self.last_export='Cargo moved with the mouse'
                print('CARGO_MOVED',np.round(pos,3).tolist(),flush=True)
                return
        self.cargo.set_world_pose(position=pos,orientation=quat)
        # it lands moving with the deck: placed at rest on a driving truck it fell behind and slid 2-3 cm rearward
        v=np.asarray(getattr(self,'rig_v',np.zeros(3)),float) if self.playing else np.zeros(3)
        if self.cargo_dynamic():
            self.cargo.set_linear_velocity(v);self.cargo.set_angular_velocity(np.zeros(3))
        self.belt_L=[None,None]
        self.cargo_physx=pos
        self.stick=None;self.release_v=v
        if self.mode==1:
            self.set_cargo_kinematic(False)
        self.last_export='Cargo moved with the mouse'
        print('CARGO_MOVED',np.round(pos,3).tolist(),flush=True)

    # real MPU-6050 on the cart (sensor Pi recordings 2026-10-14/15): bias when parked, 0.1 s-frame noise when
    # parked and when driving (road vibration). G for ax / ay / az, deg/s for gx / gy / gz.
    IMU_BIAS=np.array([-.006,.022,.001])
    IMU_SD_PARKED=(np.array([.0005,.0004,.0006]),np.array([.03,.03,.05]))
    IMU_SD_DRIVING=(np.array([.035,.015,.022]),np.array([1.5,1.5,3.7]))
    IMU_TAU=.08                       # s, correlation time of the vibration

    def sim_imu(self):
        """Specific force (G, body frame: x forward, y left, z up) and body rates (deg/s) of the truck from its
        motion: forward acceleration, centripetal v x yaw rate, gravity through slope / bank / body attitude."""
        from omni.isaac.core.utils.rotations import quat_to_rot_matrix
        a_fwd,a_lat,yaw_rate=getattr(self,'kin',(0.,0.,0.))
        if not self.playing:
            a_fwd=a_lat=yaw_rate=0.
        r=quat_to_rot_matrix(np.asarray(self.body_orientation(),float))    # body -> world
        f=np.array([a_fwd/G,a_lat/G,0.])+r.T@np.array([0.,0.,1.])
        rates=np.array([getattr(self,'roll_rate',0.),getattr(self,'pitch_rate',0.),math.degrees(yaw_rate)])
        return f,rates

    def imu_noise(self,a,w):
        """Add the real sensor's bias and coloured noise; the noise level goes from 'parked' to 'driving' with
        the speed (the cart recordings reach their vibration level at walking pace)."""
        s=float(np.clip(abs(self.velocity)/1.,0.,1.))
        sd_a=self.IMU_SD_PARKED[0]+(self.IMU_SD_DRIVING[0]-self.IMU_SD_PARKED[0])*s
        sd_w=self.IMU_SD_PARKED[1]+(self.IMU_SD_DRIVING[1]-self.IMU_SD_PARKED[1])*s
        dt=max(1e-3,self.t-getattr(self,'noise_t',self.t-1/60.)) if self.playing else 0.
        self.noise_t=self.t
        phi=math.exp(-dt/self.IMU_TAU) if dt else 1.
        rng=getattr(self,'rng',None) or np.random.default_rng(7)
        self.rng=rng
        n_a,n_w=getattr(self,'imu_n',(np.zeros(3),np.zeros(3)))
        if dt:
            q=math.sqrt(max(0.,1-phi*phi))
            n_a=phi*n_a+q*rng.standard_normal(3);n_w=phi*n_w+q*rng.standard_normal(3)
        self.imu_n=(n_a,n_w)
        return a+self.IMU_BIAS+n_a*sd_a,w+n_w*sd_w

    def imu_tilt(self,a,w):
        """Roll / pitch as the Pi computes them: complementary filter (alpha .98) of gyro and accelerometer tilt."""
        acc=np.array([math.degrees(math.atan2(a[1],a[2])),math.degrees(math.atan2(-a[0],math.hypot(a[1],a[2])))])
        dt=max(0.,self.t-getattr(self,'tilt_t',self.t));self.tilt_t=self.t
        prev=getattr(self,'tilt',None)
        self.tilt=acc if prev is None or dt>.5 else .98*(prev+np.array([w[0],w[1]])*dt)+.02*acc
        return self.tilt

    def rig_matrix(self):
        """World transform of the plate (rig) frame - real-cart metres, as drawn (scale, body pitch / roll included)."""
        return UsdGeom.XformCache(Usd.TimeCode.Default()).GetLocalToWorldTransform(self.stage.GetPrimAtPath(self.rig_root))

    def belt_ropes(self,dt):
        """Sandbox lashing: two belts over the cargo behave as ropes (belt_rope.py). Each belt's length follows
        the cargo's cross-section in its plane; tension = preload + stiffness x stretch + damping, never below 0
        (a slack belt carries nothing, a belt can only pull) and it acts along the free legs, at the points where
        the belt leaves the cargo (down and inwards, resisting lifting, tipping and sideways sliding; fore-aft
        only through the friction it adds). Before: an invisible 3D spring pulled the cargo centre to its seat in
        every direction and its damping acted on the world velocity, dragging the cargo backwards while driving.
        The belts are tightened while the cargo settles after a reset (the length then carries the preload)."""
        from omni.isaac.core.utils.rotations import quat_to_rot_matrix
        hooked=self.belts_hooked()
        for i in range(2):
            if not hooked[i]:
                self.belt_T[i]=0.;self.belt_L0[i]=None;self.belt_L[i]=None
        if not any(hooked):
            return
        M=self.rig_matrix();Minv=M.GetInverse()
        scale=Gf.Vec3d(M.GetRow3(0)).GetLength()
        r_rig=np.array([[M[i][j] for j in range(3)] for i in range(3)]).T/scale          # rig axes as columns (world)
        pos,quat=self.cargo.get_world_pose()
        centre=np.array(Minv.Transform(Gf.Vec3d(*map(float,pos))))
        r_cargo=r_rig.T@quat_to_rot_matrix(np.asarray(quat,float))
        force=np.zeros(3);torque=np.zeros(3);paths=[]
        for i,x in enumerate(belt_rope.BELT_X):
            path=belt_rope.upper_path(belt_rope.box_section(centre,r_cargo,belt_rope.CARGO_HALF,x))
            paths.append(path)
            if not hooked[i]:
                continue
            length=belt_rope.length(path);contact=len(path)>2
            if self.belt_L0[i] is None or self.t<self.settle_until:
                self.belt_L0[i]=length                     # tightened onto the settling cargo (or just re-hooked)
            stretch=length-self.belt_L0[i]
            rate=(length-self.belt_L[i])/dt if self.belt_L[i] is not None and dt>0 else 0.
            self.belt_L[i]=length
            preload=self.value(('pretension_c1','pretension_c2')[i])
            tension=max(0.,preload+self.value('stiffness')*stretch+40.*rate) if contact else 0.
            # webbing stretches, it does not snap: limit how fast the tension can change and cap it (ratchet slips),
            # otherwise a belt catching a bottom corner as the cargo slid out from under it threw the cargo 5 m
            step=3000.*max(dt,1e-3)
            self.belt_T[i]=float(np.clip(min(tension,400.),self.belt_T[i]-step,self.belt_T[i]+step))
            for point,f in belt_rope.leg_forces(path,self.belt_T[i]):
                at=np.array(M.Transform(Gf.Vec3d(float(x),float(point[0]),float(point[1]))))
                fw=r_rig@np.array([0.,f[0],f[1]])
                force+=fw;torque+=np.cross(at-np.asarray(pos),fw)
        n=np.linalg.norm(force)
        if n>800.:
            force*=800./n;torque*=800./n
        if self.cargo_dynamic():
            self.cargo._rigid_prim_view.apply_forces_and_torques_at_pos(forces=np.array([force]),torques=np.array([torque]),is_global=True)
        if self.scan_points is not None and self.flag('scan'):
            # draw the belts on the visible scanned cargo (the physics uses its bounding box)
            # cargo body transform from PhysX (USD may not be written back every step): rows scale x R^T, then position
            cargo=np.eye(4);cargo[:3,:3]=np.diag(np.asarray(self.cargo.get_local_scale(),float))@quat_to_rot_matrix(np.asarray(quat,float)).T
            cargo[3,:3]=np.asarray(pos,float)
            rig_pts=(self.scan_points@cargo@self.as_array(Minv))[:,:3]
            self.scene.drape_on_points(rig_pts,band=.02)
        else:
            for i,path in enumerate(paths):
                self.scene.drape_belt(i,path)

    def set_straps(self,on):
        """Hook / unhook both belts (Release cargo, the API, --verify); the per-belt boxes follow."""
        for k in ('straps','straps_c1','straps_c2'):
            self.models[k].set_value(bool(on))

    def belts_hooked(self):
        """[front c1, rear c2] hooked. Each belt has its own 'connected' box; the master box sets both."""
        return [self.flag('straps_c1'),self.flag('straps_c2')]

    def cargo_dynamic(self):
        """True while the cargo is a dynamic PhysX body (sandbox). In replay it is kinematic, and PhysX refuses
        velocities / forces on kinematic bodies ("Body must be non-kinematic!" in the console)."""
        if getattr(self,'kin_changed_at',-1)==getattr(self,'frame_count',0):
            return False                              # switched this frame: PhysX still has the old state
        attr=UsdPhysics.RigidBodyAPI(self.cargo.prim).GetKinematicEnabledAttr()
        return not (attr and attr.Get())

    def set_cargo_kinematic(self,on):
        attr=UsdPhysics.RigidBodyAPI(self.cargo.prim).GetKinematicEnabledAttr()
        if bool(attr.Get())!=bool(on):
            attr.Set(bool(on))
            self.kin_changed_at=self.frame_count      # PhysX applies it at the next step

    def secure_cargo(self,dt):
        """Static friction of a lashed load. The kinematically moved deck (its pose is written every frame, PhysX
        sees no surface velocity) cannot drag the cargo along by friction, so properly strapped cargo crept 4-7 cm
        on slopes and in curves. Now the cargo is fixed to the plate while the friction it can develop holds it:
            demand   = m g |f_plane|                 (in-plane specific force: acceleration, curves, slope, bank)
            capacity = mu (m g f_normal + 2 T_front + 2 T_rear)   (belts press it down with both legs)
        Held, it is a kinematic body moved with the plate (exactly, no per-step contact push: re-placing a dynamic
        body every frame left it 8 mm high on a 12 deg slope and stretched the belts to 55 N). Beyond the capacity
        (low friction, released belts, too steep, too hard a manoeuvre) it becomes dynamic again with the deck's
        velocity and slides; it is held again wherever it comes to rest."""
        if self.mode!=1:
            self.stick=None;self.secured=False
            return
        f,_=self.sim_imu()
        mass=self.value('mass')
        normal=mass*G*max(0.,float(f[2]))+2*sum(T for T,on in zip(self.belt_T,self.belts_hooked()) if on)
        demand=mass*G*math.hypot(float(f[0]),float(f[1]))
        capacity=self.value('friction')*normal
        self.grip=(demand,capacity)
        M=self.rig_matrix()
        origin=np.array(M.ExtractTranslation())
        prev=getattr(self,'rig_origin',None)
        self.rig_v=(origin-prev)/dt if prev is not None and dt>0 and np.linalg.norm(origin-prev)<2. else np.zeros(3)
        self.rig_origin=origin
        pos,quat=self.cargo.get_world_pose()
        local=np.array(M.GetInverse().Transform(Gf.Vec3d(*map(float,pos))))
        on_plate=abs(local[0])<.44 and abs(local[1])<.32 and local[2]>.20 and self.t>=self.settle_until
        holds=on_plate and demand<capacity
        if self.stick is not None and not holds:
            # it slips: dynamic again, moving with the deck (the velocity is set once PhysX has the change)
            self.stick=None;self.secured=False
            self.set_cargo_kinematic(False)
            self.release_v=getattr(self,'deck_v',np.zeros(3))
            return
        if self.stick is None:
            if not self.cargo_dynamic():
                self.set_cargo_kinematic(False)
                return
            if getattr(self,'release_v',None) is not None:
                self.cargo.set_linear_velocity(self.release_v);self.release_v=None
            velocity=np.asarray(self.cargo.get_linear_velocity(),float)
            deck_v=(np.asarray(pos,float)-self.stick_pos)/dt if self.stick_pos is not None and dt>0 else velocity
            self.stick_pos=np.asarray(pos,float)
            if holds and demand<.95*capacity and np.linalg.norm(velocity-deck_v)<.05*self.rig_scale:
                body=Gf.Matrix4d().SetRotate(Gf.Quatd(float(quat[0]),Gf.Vec3d(*map(float,quat[1:]))))
                body.SetTranslateOnly(Gf.Vec3d(*map(float,pos)))
                self.stick=body*M.GetInverse()           # cargo pose in the plate frame, kept while it holds
                self.set_cargo_kinematic(True)
            self.secured=self.stick is not None
            return
        target=Gf.Matrix4d(self.stick)*M
        m=Gf.Matrix4d(target).RemoveScaleShear()
        q=m.ExtractRotationQuat();new=np.array(m.ExtractTranslation())
        self.deck_v=(new-self.stick_pos)/dt if self.stick_pos is not None and dt>0 else np.zeros(3)
        kinematic_pose(self.cargo,new,np.array([q.GetReal(),*q.GetImaginary()]))
        self.stick_pos=new;self.secured=True

    def place_cargo(self):
        self.cargo.set_world_pose(position=self.cargo_desired(),orientation=self.cargo_orientation())
        if self.mode==1 and self.cargo_dynamic():
            self.cargo.set_linear_velocity(np.zeros(3));self.cargo.set_angular_velocity(np.zeros(3))

    def base_speed(self,case):
        speed=self.value('speed')/3.6 if TRUCK else self.value('speed')
        if self.mode==0 and self.flag('imu_motion') and case['motion']['starts_at_rest']:
            return 0.
        return speed

    def advance(self,dt):
        if self.hq_state is not None:
            if self.hq_step():
                return
        self.api.pump()
        if self.mode==1:
            self.cargo_written=None
            self.user_moved_cargo()
            if self.queued_move is not None and self.cargo_dynamic():
                right,rear=self.queued_move;self.queued_move=None
                self.api_action('move_cargo',value=[right,rear])
                self.last_export=f'Cargo moved {right:+.0f} mm right, {rear:+.0f} mm rear - physics sandbox'
        else:
            self.replay_cargo_moved()
        if self.pending_mode is not None:
            mode=self.pending_mode;self.pending_mode=None
            self.apply_mode(mode)
        if self.export_requested:
            self.export_requested=False
            self.export()
        if self.reset_requested:
            self.reset()
        case=self.cases[self.index]
        live=self.live_on and self.mode==0
        if self.playing:
            self.t+=dt*self.value('rate') if self.mode==0 and not live else dt
        if self.mode==0 and not live and self.t>case['duration']:
            if self.flag('loop'):
                self.t=0.    # the vehicle keeps its place on the loop; only the recording restarts
            else:
                self.t=case['duration'];self.playing=False
        f=self.live_row() if live else interpolate(case,self.t)
        active_dt=dt if self.playing else 0
        self.previous_velocity=self.velocity
        replay_imu=self.mode==0 and self.flag('imu_motion') and not live
        if replay_imu:
            # Speed = base speed + recorded speed change. The recorded yaw turns the truck only when the user asks
            # for it (turn_gain, default 0): heading is the user's.
            self.velocity=max(0.,self.base_speed(case)+(f.get('v_imu') or 0.))
            yaw_rate=(f.get('yaw_rate') or 0.)*self.value('turn_gain')
            a_fwd=f.get('a_fwd') or 0.
        else:
            target=self.value('speed')/3.6 if TRUCK else self.value('speed')
            if self.mode==1 and self.t<getattr(self,'settle_until',0.):
                # A freshly placed cargo settles on the parked truck before it drives off (as when loading);
                # driving off at once made the team's analyzer flag the settling as a cargo shift.
                target=0.
            self.velocity+=float(np.clip(target-self.velocity,-self.value('accel')*active_dt,self.value('accel')*active_dt))
            a_fwd=(self.velocity-self.previous_velocity)/dt if active_dt else 0.
            yaw_rate=0.
        # Manual driving (2026-11-01, user: "left / right still follow the lane and turn on their own - the truck
        # must move only as I control it"): no lane keeping and no rejoining the loop. The heading changes only
        # from the steer slider, a turn from the buttons / API, the real rig's gyro in live mode and, if turn_gain
        # is raised, the recording's yaw. At the paved area's outer edge the truck slows and stops instead of
        # steering itself; turning away (it can turn on the spot) lets it drive on.
        command=self.value('steer') if not replay_imu else 0.
        if live:
            gz=f.get('imu1_gz_dps')
            if isinstance(gz,(int,float)):
                # bias follows the gyro while it is quiet; > 6 deg/s above it is a real turn (gz + = left). It
                # also creeps towards a lasting reading (60 s), so a bias first caught while the rig was being
                # turned does not make the truck circle for good.
                if self.gz_bias is None:
                    self.gz_bias=float(gz)
                else:
                    self.gz_bias+=(gz-self.gz_bias)*min(1.,dt/(5. if abs(gz-self.gz_bias)<4. else 60.))
                w=gz-self.gz_bias
                if abs(w)>6.:
                    command+=max(-45.,min(45.,w))
        if self.turn_target is not None:
            other=command+yaw_rate                # what else turns the truck (steer slider, gyro, turn_gain)
            yrf=getattr(self,'yaw_rate_f',0.)
            left=self.turn_target
            # minus what the 0.6 s steering lag below still adds once the command drops (8 deg overshoot without)
            err=left-.6*(yrf+yaw_rate)
            # about a 12 m radius like a junction, but no more than TURN_A_MAX sideways at speed
            v=abs(self.velocity)
            rate=min(max(self.TURN_RATE,math.degrees(v/12.)),math.degrees(self.TURN_A_MAX/max(v,.1)))
            push=max(-rate,min(rate,1.5*err))
            command+=push
            # finish on the exact heading (ending on the lead-corrected error stopped ~1.6 deg short); with other
            # inputs turning too, finish once past the target or once they hold the truck still short of it
            if abs(left)<.3 and abs(yrf+yaw_rate)<1.:
                self.turn_target=None
            elif abs(other)>.5 and (left*self.turn_sign<=0. or
                                    (abs(yrf+yaw_rate)<1. and abs(push)<rate)):
                self.turn_target=None
        # Steering takes time on a real truck: the commanded turn rate is reached over ~0.6 s. A step
        # (Turn left / Straight buttons) yawed the deck instantly and the strapped cargo slid 20-35 mm.
        k=min(1.,active_dt/.6) if active_dt else 0.
        self.yaw_rate_f=getattr(self,'yaw_rate_f',0.)+(command-getattr(self,'yaw_rate_f',0.))*k
        heading_rate=self.yaw_rate_f+yaw_rate
        self.yaw=(self.yaw+heading_rate*active_dt+180.)%360.-180.
        if self.turn_target is not None:
            self.turn_target-=heading_rate*active_dt
        # Edge of the paved area (the guardrail is 1.5 m beyond it): limit the speed so the truck can still stop
        # before it (3 m/s^2) while it heads outwards.
        self.update_pose()
        _,_,th,_=self.road.pose(self.s)
        travel=1. if self.velocity>=0. else -1.
        outward=travel*(math.sin(th)*math.cos(math.radians(self.yaw))-math.cos(th)*math.sin(math.radians(self.yaw)))
        limit=road_path.PAVED_OUT-(2. if TRUCK else 1.)
        self.edge_stop=False
        if outward>.02:
            room=max(0.,(limit+self.d)/outward-3.)          # metres along the travel to the edge, 3 m spare
            v_max=math.sqrt(2*3.*room)
            if abs(self.velocity)>v_max:
                self.velocity=travel*v_max;self.edge_stop=v_max<.3
                a_fwd=(self.velocity-self.previous_velocity)/dt if active_dt else 0.
        step=self.velocity*active_dt
        nxt=self.xy+step*np.array([math.cos(math.radians(self.yaw)),math.sin(math.radians(self.yaw))])
        s_n,d_n,th_n=self.road.project(float(nxt[0]),float(nxt[1]))
        if d_n<-limit:                                     # last resort: never past the edge
            nxt=nxt+(-limit-d_n)*np.array([-math.sin(th_n),math.cos(th_n)])
        self.xy=nxt
        self.distance+=step
        self.update_pose()
        # Lateral acceleration of the simulated path (road curve + relative turning) drives the body roll;
        # recorded roll/pitch are accelerometer tilt and are shown as data instead.
        heading_rate=math.radians((self.yaw-self.heading_prev+540.)%360.-180.)/dt if active_dt else 0.
        self.heading_prev=self.yaw
        a_lat=self.velocity*heading_rate
        self.kin=(float(a_fwd),float(a_lat),float(heading_rate))   # sandbox IMU model (sim_imu)
        gain=self.value('tilt_gain')
        k=1-math.exp(-active_dt/.15) if active_dt else 1.
        # A loaded truck pitches / rolls 1-2 deg under hard braking or cornering; the wheels stay on the road
        # (Truck.suspend). The earlier 10 deg/G up to 6 deg pushed the rear end ~0.5 m into the ground.
        pitch0,roll0=self.body_pitch,self.body_roll
        self.body_pitch+=(max(-3.,min(3.,-gain*a_fwd/G))-self.body_pitch)*k
        self.body_roll+=(max(-3.,min(3.,gain*a_lat/G))-self.body_roll)*k
        self.pitch_rate=(self.body_pitch-pitch0)/dt if active_dt else 0.;self.roll_rate=(self.body_roll-roll0)/dt if active_dt else 0.
        if TRUCK:
            self.truck.roll_wheels(self.distance)
            self.truck.suspend(self.body_pitch,self.body_roll)
            steer=max(-150.,min(150.,math.degrees(heading_rate)*3.5))
            if abs(steer-self.steer_shown)>1.5:
                self.steer_shown=steer
                self.truck.steer(steer)
                if self.driver:
                    self.driver.sit(T.DRIVER,steer)
        else:
            facing=1 if self.flag('pull') or not self.scene.skinned else -1
            self.phase+=(self.velocity*facing if self.scene.skinned else abs(self.velocity))*active_dt*8
        if self.changed('env',(self.value('bank'),self.value('grade'))):
            self.retilt()
        orientation=self.body_orientation()
        base=self.world_point([*self.xy,0])
        kinematic_pose(self.cart,base,orientation)
        # Physics/USD properties are only written when a slider or checkbox changes;
        # rewriting them every frame caused needless PhysX and Hydra updates.
        if self.changed('mass',self.value('mass')):
            self.cargo.set_mass(self.value('mass'))
        if self.changed('friction',self.value('friction')):
            self.contact.set_static_friction(self.value('friction'))
            self.contact.set_dynamic_friction(self.value('friction')*.8)
        if self.changed('cartmass',self.value('cartmass')):
            self.cart.set_mass(self.value('cartmass'))
        show_scan=self.flag('scan') and self.mode==1
        if self.changed('scan',(show_scan,self.mode)):
            UsdGeom.Imageable(self.scan).MakeVisible() if show_scan else UsdGeom.Imageable(self.scan).MakeInvisible()
            # The collision proxy is drawn only in the sandbox when the scan is hidden.
            UsdGeom.Imageable(self.proxy).MakeVisible() if (self.mode==1 and not show_scan) else UsdGeom.Imageable(self.proxy).MakeInvisible()
        hooked=self.belts_hooked()
        if self.changed('straps',tuple(hooked)):
            for strap,on in zip(self.scene.straps,hooked):
                UsdGeom.Imageable(strap).MakeVisible() if on else UsdGeom.Imageable(strap).MakeInvisible()
        desired=self.cargo_desired()
        if self.mode==0:
            kinematic_pose(self.cargo,desired,self.cargo_orientation())
            self.cargo_written=np.asarray(desired,float)
            self.update_live() if live else self.update_measured(case,f)
        else:
            self.apply_cargo_view()
            self.belt_ropes(dt)
            self.secure_cargo(dt)
        if not TRUCK:
            person=base+self.cart_vector([self.scene.operator_offset(self.flag('pull')),0,0])
            person_yaw=self.yaw+self.scene.operator_yaw(self.flag('pull'))
            pose(self.scene.person,person,(self.value('bank'),-self.value('grade'),person_yaw))
            self.scene.walk(self.phase,min(abs(self.velocity)*2,1) if self.playing else 0,self.flag('pull'))
        self.follow_camera()
        if self.playing:
            self.world.step(render=True)
        elif self.mode==1 and self.pause_settle>0:
            # paused, but the cargo was just moved: PhysX has to take the new pose (and let it settle on the parked
            # truck) - without a step the cells kept reading the old place and the COG did not change
            self.pause_settle-=dt
            self.world.step(render=True)
        else:
            app.update()
        self.latest=self.readings(f,desired,dt)
        if self.recorder is not None:
            self.record_row()
        if self.display.live and time.time()-getattr(self,'display_t',0.)>=.1:
            self.display_t=time.time();self.display.offer(self.build_row(self.t))
        self.update_markers(base)
        if self.frame_count%6==0:
            self.refresh_ui()
        self.frame_count+=1
        self.perf_frames+=1
        now=time.perf_counter()
        if now-self.perf_t>=2.:
            self.fps=self.perf_frames/(now-self.perf_t)
            print(f'PERF fps={self.fps:.1f}',flush=True)
            self.perf_t=now;self.perf_frames=0

    def update_measured(self,case,f):
        """Animate the D435i surface and fit the belts over it (replay only)."""
        cache=self.load_cache(case)
        ct=cache['t']
        index=int(np.clip(np.searchsorted(ct,self.t,side='right')-1,0,len(ct)-1))
        if index!=self.surface_index:
            blend=index==self.surface_index+1      # next frame: smooth the depth noise; a seek starts afresh
            self.surface_index=index
            self.surface.update(cache['depth'][index],jpeg_bytes(cache,index),blend=blend)
            if self.flag('cargo_boxes'):
                cam=cache['camera']
                hdr={'session':'replay','run':case['id'],'t_run':float(cache['t'][index]),'grid':cache['grid'],
                     'calib':{'ref_h':cam['ref_h']},'surface':{'intr':cam['intr'],'R':cam['R']},'cam_accel':cam.get('cam_accel')}
                self.update_boxes(frame_from_parts(hdr,cache['depth'][index],jpeg_bytes(cache,index),
                                                   cache['height'][index],cache['colour'][index]))
        self.apply_cargo_view()
        self.drape_belts()

    def update_boxes(self,frame):
        try:
            boxes,floor=self.box_model.update(frame)
            self.box_drawer.draw(boxes,floor)
        except Exception as e:      # never let the box fit stop the twin; the surface is still there
            if time.time()-getattr(self,'box_err_t',0.)>10:
                self.box_err_t=time.time();print('BOXES_ERROR',repr(e),flush=True)

    def apply_cargo_view(self):
        """Measured mode shows either the display-style boxes or the raw depth surface."""
        boxes=self.flag('cargo_boxes') and self.mode==0
        if self.changed('cargo_view',(self.mode,boxes)):
            surf=UsdGeom.Imageable(self.surface.prim)
            surf.MakeVisible() if self.mode==0 and not boxes else surf.MakeInvisible()
            self.box_drawer.set_visible(boxes)
            if boxes:
                self.box_model.reset();self.surface_index=-1;self.live_count=-1   # refit from the next frame

    def drape_belts(self):
        pts=getattr(self.surface,'cargo_points',None)
        if pts is not None and len(pts):
            self.scene.drape_on_points(pts)
        else:
            self.scene.fit_straps(self.surface.cargo_top())

    # ------------------------------------------------------------------ live (sensor Pi)
    def set_live(self,on=None):
        """Show the sensor Pi's live frames (on) or go back to the selected recording (off)."""
        on=(not self.live_on) if on is None else bool(on)
        if on:
            ok,message=self.live.start()
            if not ok:
                self.last_export=f'Live: {message}'
                return {'ok':False,'error':message}
            self.live_on=True;self.live_count=0;self.live_cam=None;self.live_ts=None;self.gz_bias=None
            if self.mode!=0:
                self.set_mode(0)
            self.clear_cargo_view()          # nothing of the replay stays on the plate until the first live frame
            self.playing=True;self.surface_index=-1;self.cam_shown=-1
            self.shift.reset()
            if self.team_live is not None:
                self.team_live.reset();self.team_t=-1.;self.team_out={}
            self.labels['source'].text='센서 파이에서 지금 받는 값 그대로 (화물 모양 · 로드셀 · 벨트 · IMU · 판정). 트럭 움직임은 아래 슬라이더대로.'
            self.last_export=f'Live: {message}'
            return {'ok':True,'message':message}
        self.live.stop();self.live_on=False
        self.t=0.                            # the clock ran on during live: start the recording from its beginning
        cache=self.load_cache(self.cases[self.index])
        self.surface.set_camera(cache['camera'],cache['grid'])
        self.surface_index=-1;self.cam_shown=-1;self.shift.reset()
        self.labels['source'].text='기록된 센서값을 그대로 재생. 트럭 움직임은 기록된 IMU를 따름 (회전은 비율만큼).'
        self.last_export='Live: off'
        return {'ok':True,'message':'live off - back to the recording'}

    def clear_cargo_view(self):
        cache=self.load_cache(self.cases[self.index])
        self.surface.update(np.zeros(cache['depth'][0].shape,np.uint16),None)
        self.box_drawer.draw([],None);self.box_model.reset()

    def live_row(self):
        """The newest sensor row from the Pi (its own column names), timed by the twin's clock. Nothing when the
        link is stale, so an old reading is never judged, recorded or sent on as if it were current."""
        if not self.live.receiving():
            return {'t':self.t}
        frame,_=self.live.latest()
        sensors=(frame or {}).get('sensors') or {}
        f={k:v for k,v in sensors.items() if k!='check_text'}
        f['t']=self.t
        return f

    def update_live(self):
        """Redraw the measured surface from the newest live frame (only when a new one has arrived)."""
        frame,count=self.live.latest()
        if not self.live.receiving():
            if self.live_count!=0 and not getattr(self,'live_cleared',False):
                self.clear_cargo_view();self.live_cleared=True   # link lost: do not keep showing the last cargo
            frame=None
        if frame is not None and count!=self.live_count:
            self.live_cleared=False
            blend=count==self.live_count+1
            cam=json.dumps([frame['camera']['intr'],frame['camera']['R'],frame['camera']['ref_h'],frame['grid']])
            if cam!=self.live_cam:            # first frame, or the Pi re-calibrated: new rays
                self.live_cam=cam;blend=False
                self.surface.set_camera(frame['camera'],frame['grid'])
            self.live_count=count
            self.surface.update(frame['depth'],frame['jpeg'],blend=blend)
            if self.flag('cargo_boxes'):
                self.update_boxes(frame_from_parts(frame['header'],frame['depth'],frame['jpeg'],frame['height'],frame['colour']))
        self.apply_cargo_view()
        self.drape_belts()

    def live_readings(self,f,dt):
        out=dict(f)
        out['recorded_judgement']={k:f.get(k) for k in ('algo','risk','lv_load','lv_cog','lv_move','lv_c1','lv_c2')}
        if f.get('total_g') is None:
            return out                       # no sensor row yet (or the Pi's sensor feed has stopped)
        ts=f.get('t_s')
        if self.team_live is not None and isinstance(ts,(int,float)):
            if self.live_ts is not None and ts<self.live_ts:
                self.team_live.reset();self.team_out={}      # the Pi restarted its clock
            if ts!=self.live_ts:                             # judge each new ~10 Hz sensor row once, on the Pi's clock
                self.live_ts=ts;self.team_out=self.team_live.step(ts,out)
            out.update(self.team_out)
        self.apply_shift(out)
        return out

    def live_status_ko(self):
        r=self.live
        if not r.running:
            return '실시간: 꺼짐'+(f' ({r.error})' if r.error else '')
        if r.receiving():
            hz=r.hz();delay=r.delay
            more=(f' {hz:.1f} Hz' if hz else '')+(f', 지연 {delay:.2f}초' if delay is not None and -1<delay<10 else '')
            return f'실시간: 센서 파이에서 받는 중 ({r.peer}{more})'
        if r.arrivals:
            return '실시간: 연결 끊김 - 센서 파이를 기다리는 중'
        return f'실시간: 센서 파이를 기다리는 중 (포트 {r.port})'

    def update_markers(self,base):
        f=self.latest
        cx=f.get('cog_x_mm');cy=f.get('cog_y_mm')
        if cx is not None and cy is not None:
            h=f.get('h_y_mm') or 190.
            # COG X grows towards the right of the vehicle (rig +y), like the D435i surface (camera x = rig y) and the
            # real cart (left turns, +ay, load the right side: +110 mm/G). The marker used to sit mirrored.
            pos=base+self.cart_vector(self.rig_to_cart([(cy-196)/1000,(cx-125)/1000,DECK_TOP+h/1000]))
            pose(self.cog,pos,scale=(.025,.025,.025))
            UsdGeom.Imageable(self.cog).MakeVisible()
        else:
            UsdGeom.Imageable(self.cog).MakeInvisible()
        if self.changed('cog_level',level_of(f.get('lv_cog'))):
            lv=level_of(f.get('lv_cog'))
            self.cog_shader.GetInput('diffuseColor').Set(Gf.Vec3f(*(LEVEL_RGB[lv] if lv in (1,2,3) else (1.,.9,.05))))
        for i,key in enumerate(('lv_c1','lv_c2')):
            lv=level_of(f.get(key))
            if lv!=self.belt_levels[i]:
                self.belt_levels[i]=lv
                colour=LEVEL_RGB[lv] if lv in (1,2,3) else (.018,.02,.023)
                self.belt_shaders[i].GetInput('diffuseColor').Set(Gf.Vec3f(*colour))
        if self.frame_count%3==0:
            for name,prim in self.cell_bars.items():
                g=f.get(name+'_g')
                h=max(.01,min(.30,(g or 0.)/5000.*.25))
                x,y=prim.GetAttribute('xformOp:translate').Get()[0],prim.GetAttribute('xformOp:translate').Get()[1]
                pose(prim,(x,y,.185+h/2),scale=(.03,.03,h))
            over=any((f.get(n+'_g') or 0)>5000 for n in ('NW','NE','SW','SE'))
            if self.changed('cell_over',over):
                self.cell_shader.GetInput('diffuseColor').Set(Gf.Vec3f(*((1.,.1,.05) if over else (.1,.8,.9))))
        if TRUCK:
            self.truck.set_beacon(level_of(f.get('risk')))

    # ------------------------------------------------------------------ judgement
    def readings(self,f,desired,dt):
        if self.mode==0 and self.live_on:
            return self.live_readings(f,dt)
        if self.mode==0:
            out=dict(f)
            team=self.cases[self.index].get('_team')
            if team:
                out['recorded_judgement']={k:f.get(k) for k in ('algo','risk','lv_load','lv_cog','lv_move','lv_c1','lv_c2')}
                out.update(team[f['_index']])
            self.apply_shift(out)
            return out
        pos,quat=self.cargo.get_world_pose()
        self.cargo_physx=np.asarray(pos,float)
        here=np.asarray(pos,float)
        if not self.secured and self.cargo_dynamic():
            # a free cargo has already been integrated to the next step while the deck is still at this frame's
            # pose: compare them at the same time (else a cargo riding along read v x dt ahead)
            here=here-getattr(self,'rig_v',np.zeros(3))*(1/60.)
        # cargo position in the real plate frame (as drawn: body roll / pitch included) - the frame of the cells
        local=np.array(self.rig_matrix().GetInverse().Transform(Gf.Vec3d(*map(float,here))))
        # Sandbox IMU. Isaac's IMU sensor on this kinematically moved truck missed the centripetal acceleration
        # (turns read ~5 % of v x yaw rate), shook 70x more than the real sensor when parked and had no road
        # vibration when driving. The specific force is therefore computed from the truck's own motion - forward
        # acceleration, v x yaw rate, gravity through road slope / bank and body pitch / roll - and the real
        # MPU-6050's mounting bias, noise and road vibration (from the sensor Pi recordings) are added on top.
        clean,w_clean=self.sim_imu()
        hist=self.imu_hist
        hist.append((self.t,clean,w_clean))
        while hist and hist[0][0]<self.t-.5:
            hist.popleft()
        # The Pi reports each IMU channel as the mean over its ~0.1 s HX711 frame, and its load cells answer about
        # COG_LAG = 0.2 s after the IMU (the cells see the true force: no sensor bias or noise).
        window=lambda t0,t1:[s for s in hist if t0<s[0]<=t1] or [hist[-1]]
        now=window(self.t-.1,self.t);lagged=window(self.t-.3,self.t-.2)
        a=np.mean([s[1] for s in now],axis=0);w=np.mean([s[2] for s in now],axis=0)
        a_cells=np.mean([s[1] for s in lagged],axis=0)
        a,w=self.imu_noise(a,w)
        spike=False
        angles=self.imu_tilt(a,w)
        # Quasi-static 4-corner reaction approximation; not a calibrated load-cell model.
        # Load transfer as the real cells see it: the specific force the IMU measures (acceleration, braking,
        # cornering, slope) acting at the cargo's centre height moves the measured COG by h x a - forward
        # acceleration / uphill towards the rear belt, a left turn to the right. The team's analyzer learns h
        # from exactly this; without it the sandbox cells never moved and h was learnt down to ~25 mm.
        # h = the real cargo's COG height (team measurements 2026-10-04: 155-204 mm; their start value 190 mm),
        # not the sandbox box's 290 mm, so the analyzer is tested on the cargo the cart actually carries.
        h_c=.19 if float(local[2])>.20 else 0.
        # x: fraction towards the NE/SE cells = the vehicle's right (rig +y); y: towards the rear row (rig +x)
        x=float(np.clip(.5+(local[1]+h_c*float(a_cells[1]))/.25,0,1));y=float(np.clip(.5+(local[0]+h_c*float(a_cells[0]))/.392,0,1))
        supported=abs(local[0])<.44 and abs(local[1])<.32 and local[2]>.20
        normal=self.value('mass')*math.cos(math.radians(self.value('grade')))*math.cos(math.radians(self.value('bank'))) if supported else 0
        # Two tension cells, one per belt: each reads its own belt (0 N when that belt is unhooked) plus its own
        # HX711 noise (sigma 0.01 N, the real cells at rest 2026-10-15)
        hooked=self.belts_hooked()
        t_front,t_rear=(float(self.belt_T[i]) if hooked[i] else 0. for i in range(2))
        cell_rng=getattr(self,'cell_rng',None) or np.random.default_rng(11);self.cell_rng=cell_rng
        t_front,t_rear=(max(0.,v+cell_rng.normal(0,.01)) if v>0 else 0. for v in (t_front,t_rear))
        # As on the real cart each belt presses the cargo onto the plate with both legs (2T): the front belt c1 on
        # the front cell row (NW/NE), the rear belt c2 on the rear row (SW/SE) - the team's model (BELT_N_Y = 0,
        # BELT_S_Y = BED_W), which removes exactly this. Without it the load came out ~4T/g too low.
        push_n,push_s=(2*t_front/G,2*t_rear/G) if supported else (0.,0.)
        weights=[normal*(1-x)*(1-y)+push_n*(1-x),normal*x*(1-y)+push_n*x,normal*(1-x)*y+push_s*(1-x),normal*x*y+push_s*x]
        out={'t':self.t,'total_g':sum(weights)*1000,'NW_g':weights[0]*1000,'NE_g':weights[1]*1000,
             'SW_g':weights[2]*1000,'SE_g':weights[3]*1000,'cog_x_mm':x*250,'cog_y_mm':y*392,
             'c1_N':t_front,'c2_N':t_rear,'imu1_roll_deg':float(angles[0]),'imu1_pitch_deg':float(angles[1]),
             'imu1_ax_G':float(a[0]),'imu1_ay_G':float(a[1]),'imu1_az_G':float(a[2]),
             'imu1_gx_dps':float(w[0]),'imu1_gy_dps':float(w[1]),'imu1_gz_dps':float(w[2]),'camera_volume_l':None,
             'h_y_mm':190.,'estimated':True,'flags':'imu_spike:imu1' if spike else '',
             # displacement from its seat along cog_x / cog_y (mm): moves the cargo on the display Pi
             'cargo_offset_mm':[round((local[1]-self.value('cargo_y'))*1000,1),round((local[0]-self.value('cargo_x'))*1000,1)],
             'cargo_on_plate':bool(supported),'cargo_secured':bool(self.secured),
             'friction':self.value('friction'),'grip_demand_N':round(self.grip[0],1),'grip_capacity_N':round(self.grip[1],1)}
        if self.team_live is not None:
            # The Pi keeps judging a parked truck, so the team's clock runs on while the sandbox is paused: a
            # cargo moved while paused kept the last judged COG (the team output overrides cog_x / cog_y).
            self.team_clock+=dt
            if self.team_clock<self.team_t:
                self.team_live.reset()
            if self.team_clock-self.team_t>=.1:       # the Pi judges each ~10 Hz HX711 frame
                self.team_t=self.team_clock;self.team_out=self.team_live.step(self.team_clock,out)
            out.update(self.team_out)
        else:
            out.update(self.judge(out))
        self.apply_shift(out)
        return out

    def apply_shift(self,out):
        """Cargo-shift level from the COG history; the overall level is recombined with it. In replay the
        Pi's own lv_move is kept as lv_move_pi and the overall level never drops below the recorded one."""
        kg=lambda *keys:sum((out.get(k+'_g') or 0.) for k in keys)/1000.
        level,shift_mm,reason=self.shift.update(self.t,out.get('cog_x_mm'),out.get('cog_y_mm'),(out.get('total_g') or 0.)/1000.,
                                                out.get('imu1_ax_G'),out.get('imu1_ay_G'),kg('NW','NE'),kg('SW','SE'))
        out['lv_move_twin']=level if self.shift.reference is not None else None
        out['shift_mm']=shift_mm;out['shift_reason']=reason;out['shift_reason_ko']=self.shift.reason_ko
        if self.team is not None:
            return                                   # the team's judgement stands; the twin rule is for comparison
        out['lv_move_pi']=out.get('lv_move')
        out['lv_move']=out['lv_move_twin']
        # Korean judgement text in the sensor Pi's format ('<level> <check>: <why>'), read by the display Pi.
        parts=[part for part in str(out.get('risk_text') or '').split(' | ') if part and '화물 이동' not in part]
        if level:
            parts.append(f"{KO_LEVELS[level]} 화물 이동: {self.shift.reason_ko}")
        out['risk_text']=' | '.join(parts)
        belts=[lv for lv in (out.get('lv_c1'),out.get('lv_c2')) if lv is not None]
        combined=self.combine([out.get('lv_load'),out.get('lv_cog'),max(belts) if belts else None,out['lv_move']])
        recorded=out.get('risk') if self.mode==0 else None
        out['risk']=max(combined,recorded) if isinstance(recorded,(int,float)) else combined

    @staticmethod
    def combine(levels):
        """Team overall rule: any danger or two warnings -> danger; any warning or three cautions -> warning."""
        parts=[int(lv) for lv in levels if isinstance(lv,(int,float))]
        if any(lv==3 for lv in parts) or sum(lv==2 for lv in parts)>=2:
            return 3
        if any(lv==2 for lv in parts) or sum(lv==1 for lv in parts)>=3:
            return 2
        return 1 if any(lv==1 for lv in parts) else 0

    @staticmethod
    def judge(r):
        """Team rules (analysis/total.py, cog.py, tension.py, overall.py) on estimated readings.
        Load: belt push 2T/g removed, % of RATED_KG at 110/125/140. COG zone: margin to the nearest edge
        in % of the cell spacing at 25/15/7. Belts: EN 12195-1 floor max(2 N, 1 N/kg x share) at 100/75/50 %,
        ceiling 80 N at 50/75/100 %. Cargo shift needs history: apply_shift() adds it."""
        push_g=sum(max(0.,r.get(k) or 0.)*2/G*1000 for k in ('c1_N','c2_N'))
        cargo_kg=max(0.,(r.get('total_g') or 0.)-push_g)/1000.
        pct=100*cargo_kg/RATED_KG
        lv_load=3 if pct>140 else 2 if pct>125 else 1 if pct>110 else 0
        cx,cy=r.get('cog_x_mm'),r.get('cog_y_mm')
        if cargo_kg<.1 or cx is None:
            lv_cog=None;margin=None
        else:
            fx,fy=100*cx/BED_MM[0],100*cy/BED_MM[1]
            margin=min(fx,100-fx,fy,100-fy)
            lv_cog=3 if margin<7 else 2 if margin<15 else 1 if margin<25 else 0
        share_n=min(1.,max(0.,(BED_MM[1]-(cy if cy is not None else BED_MM[1]/2))/BED_MM[1]))
        shares={'c1':share_n,'c2':1-share_n}
        levels={};mins={}
        for key,share in shares.items():
            side_kg=cargo_kg*share
            need=max(2.,1.0*side_kg) if side_kg>=.2 else None
            belt=r.get(key+'_N') or 0.
            low=0
            if need:
                p=100*belt/need
                low=3 if p<50 else 2 if p<75 else 1 if p<100 else 0
            hp=100*belt/TENSION_MAX_N
            high=3 if hp>100 else 2 if hp>75 else 1 if hp>50 else 0
            levels[key]=max(low,high);mins[key]=need
        overall=Twin.combine([lv_load,lv_cog,max(levels.values())])
        return {'load_kg':cargo_kg,'measured_kg':(r.get('total_g') or 0.)/1000.,'lv_load':lv_load,'lv_cog':lv_cog,
                'cog_margin_pct':margin,'lv_c1':levels['c1'],'lv_c2':levels['c2'],'c1_min_N':mins['c1'],'c2_min_N':mins['c2'],
                'lv_move':None,'risk':overall,'rated_kg':RATED_KG,'tension_max_N':TENSION_MAX_N}

    @staticmethod
    def fmt(value, digits=2):
        return 'N/A' if value is None else f'{value:.{digits}f}'

    def refresh_ui(self):
        f=self.latest;fmt=self.fmt
        risk=level_of(f.get('risk'))
        self.labels['overall'].text=f"종합 위험도: {level_ko(risk)}"+('  (시뮬레이션 추정값)' if f.get('estimated') else '')
        self.labels['overall'].style={'font_size':17,'color':level_colour(risk)}
        kg=f.get('load_kg');pct=100*kg/RATED_KG if kg is not None else None
        self.labels['load'].text=f"적재 {fmt(kg)} kg = 적정 {RATED_KG:g} kg 의 {fmt(pct,0)}%  [{level_ko(level_of(f.get('lv_load')))}]"
        self.labels['load'].style={'color':level_colour(level_of(f.get('lv_load')))}
        cx,cy=f.get('cog_x_mm'),f.get('cog_y_mm')
        where=f" ({100*cx/250:.0f}% / {100*cy/392:.0f}%)" if cx is not None and cy is not None else ''
        self.labels['cog'].text=f"무게중심 X {fmt(cx,0)} · Y {fmt(cy,0)} mm{where}  높이 h {fmt(f.get('h_y_mm'),0)} mm  [{level_ko(level_of(f.get('lv_cog')))}]"
        self.labels['cog'].style={'color':level_colour(level_of(f.get('lv_cog')))}
        resid=f"  하중 이동 뺀 무게중심 {fmt(f.get('cog_static_x_mm'),0)} · {fmt(f.get('cog_static_y_mm'),0)} mm" if self.team else ''
        self.labels['move'].text=f"화물 이동 [{level_ko(level_of(f.get('lv_move')))}]{resid}"
        self.labels['move'].style={'color':level_colour(level_of(f.get('lv_move')))}
        self.labels['belts'].text=(f"앞벨트 c1 {fmt(f.get('c1_N'),1)} N (하한 {fmt(f.get('c1_min_N'),1)}) [{level_ko(level_of(f.get('lv_c1')))}]   "
                                   f"뒷벨트 c2 {fmt(f.get('c2_N'),1)} N (하한 {fmt(f.get('c2_min_N'),1)}) [{level_ko(level_of(f.get('lv_c2')))}]")
        self.labels['belts'].style={'color':level_colour(max(level_of(f.get('lv_c1')) or 0,level_of(f.get('lv_c2')) or 0))}
        cell=lambda k: fmt(f.get(k+'_g')/1000 if f.get(k+'_g') is not None else None)
        self.labels['cells'].text=f"로드셀 kg  앞왼 NW {cell('NW')}  앞오 NE {cell('NE')}\n           뒤왼 SW {cell('SW')}  뒤오 SE {cell('SE')}  합 {fmt((f.get('total_g') or 0)/1000)}"
        self.labels['imu'].text=(f"IMU 가속(G) 앞뒤 {fmt(f.get('imu1_ax_G'))} · 좌우 {fmt(f.get('imu1_ay_G'))} · 상하 {fmt(f.get('imu1_az_G'))}  회전 {fmt(f.get('imu1_gz_dps'),1)} 도/초\n"
                                 f"기울기 롤 {fmt(f.get('imu1_roll_deg'),1)} · 피치 {fmt(f.get('imu1_pitch_deg'),1)} 도")
        kmh=self.velocity*3.6
        where=(f"방향 {self.yaw:+.0f} 도  순환로 기준 {self.s:.0f}/{self.road.length:.0f} m, 중심선에서 {self.d:+.1f} m"
               +("\n포장 끝 - 좌회전 / 우회전으로 방향을 바꾸면 다시 출발" if self.edge_stop else ""))
        self.labels['motion'].text=(f"트럭 {kmh:.1f} km/h  {where}\n"
                                    f"차체 롤 {self.body_roll:+.1f} · 피치 {self.body_pitch:+.1f} 도")
        st=None
        live_frame=self.live.latest()[0] if self.live_on and self.mode==0 else None
        if live_frame is not None:
            st=live_frame.get('stats')
        elif self.mode==0:
            cache=self.caches.get(self.cases[self.index]['id'])
            if cache and 0<=self.surface_index<len(cache['stats']):
                st=cache['stats'][self.surface_index]
        self.labels['camera'].text=(f"D435i 부피 {fmt(st.get('volume_l'),1)} L  면적 {fmt(st.get('area_m2'),3)} m²  최고 높이 {fmt(st.get('max_mm'),0)} mm" if st
                                    else 'D435i: 시뮬레이션 깊이 영상만')
        # the team's own Korean warning text, as the sensor Pi / display show it
        text=' | '.join(part for part in str(f.get('risk_text') or '').split(' | ') if self.team or '화물 이동' not in part)
        if self.team is None and f.get('lv_move') and (f.get('shift_reason_ko') or f.get('shift_reason')):
            text=(text+' | ' if text else '')+f"{level_ko(level_of(f.get('lv_move')))} 화물 이동: {f.get('shift_reason_ko') or f['shift_reason']}"
        self.labels['warning'].text=text if text else ('모든 값이 정상 범위' if risk==0 else '')
        self._setting_time=True;self.models['time'].set_value(self.t);self._setting_time=False
        # Real D435i pictures: the live frame, or the recording's frame at the replay time. The depth preview shows
        # the real depth whenever there is a real frame, the simulated camera's otherwise (physics mode).
        real=None
        if live_frame is not None:
            real=('live',self.live_count)
            if self.live_count!=self.cam_shown:
                self.cam_shown=self.live_count
                img=cv2.imdecode(np.frombuffer(live_frame['jpeg'],np.uint8),cv2.IMREAD_COLOR)
                if img is not None:
                    self.upload(self.cam_image,cv2.cvtColor(img,cv2.COLOR_BGR2RGBA))
                self.upload(self.depth_image,self.depth_rgba(live_frame['depth']))
        elif self.mode==0 and not self.live_on:
            cache=self.caches.get(self.cases[self.index]['id'])
            if cache and self.surface_index>=0:
                real=('recording',self.surface_index)
                if self.surface_index!=self.cam_shown:
                    self.cam_shown=self.surface_index
                    img=cv2.imdecode(np.frombuffer(jpeg_bytes(cache,self.surface_index),np.uint8),cv2.IMREAD_COLOR)
                    if img is not None:
                        self.upload(self.cam_image,cv2.cvtColor(img,cv2.COLOR_BGR2RGBA))
                    self.upload(self.depth_image,self.depth_rgba(cache['depth'][self.surface_index]))
        self.real_depth=real is not None
        depth=self.camera.get_current_frame().get('distance_to_image_plane')
        if depth is not None and np.asarray(depth).size:
            depth=np.asarray(depth);self.last_depth=depth
            if not self.real_depth:
                gray=np.uint8(np.clip(np.nan_to_num(depth,nan=0,posinf=3)/3,0,1)*255)
                self.upload(self.depth_image,cv2.cvtColor(cv2.applyColorMap(gray,cv2.COLORMAP_TURBO),cv2.COLOR_BGR2RGBA))
        self.labels['cam_title'].text=('실시간 컬러 영상 (센서 파이)' if real and real[0]=='live' else
                                       '기록된 컬러 영상 (센서와 같은 시각)')
        self.labels['depth_title'].text=('D435i 깊이 영상 (빨강 = 가까움, 파랑 = 멂)' if self.real_depth else '시뮬레이션 깊이 영상 (m)')
        if getattr(self,'d435i_frame',None) is not None:
            self.d435i_frame.visible=self.camera_view==self.DEPTH_VIEW and self.real_depth
            if real:
                self.labels['d435i_title'].text=('D435i 실시간 화면 (센서 파이)' if real[0]=='live' else
                                                 f'D435i 기록 화면 ({self.cases[self.index]["title"]}, {self.t:.1f}초)')
        case=self.cases[self.index]
        if self.live_on and self.mode==0:
            self.labels['status'].text=f"실시간 (센서 파이) | {self.t:.1f}초\n{status_ko(self.last_export)}"
        else:
            self.labels['status'].text=f"{case['id']} {case['title']} | {self.t:.1f}초 / {case['duration']:.1f}초\n{status_ko(self.last_export)}"
        self.labels['live'].text='  '+self.live_status_ko()
        self.live_button.text='실시간 멈춤' if self.live_on else '실시간 받기'
        self.labels['perf'].text=f'화면: {self.fps:.0f} FPS'
        self.labels['display'].text='  '+status_ko(self.display.status)
        self.display_button.text='디스플레이 멈춤' if self.display.busy else '디스플레이로 보내기'
        self.api.snapshot=self.api_snapshot()
        if self.recorder is not None:
            r=self.recorder;elapsed=time.time()-r['start']
            self.labels['rec'].text=f"  ● 기록 중 {int(elapsed//60):02d}:{int(elapsed%60):02d}  {r['rows']}줄"

    @staticmethod
    def upload(provider,rgba):
        provider.set_data_array(np.ascontiguousarray(rgba,dtype=np.uint8),[rgba.shape[1],rgba.shape[0]])

    # ------------------------------------------------------------------ export / verify
    # ------------------------------------------------------------------ recording
    REC_HZ=10.
    REC_FIXED=('rec_time_s','clock','mode','run','replay_time_s','vehicle_kmh','lap_s_m','lane_d_m','heading_deg',
               'road_slope_deg','road_bank_deg','cargo_mass_kg','straps_connected')

    def toggle_recording(self):
        if self.recorder is None:
            self.start_recording()
        else:
            self.stop_recording()

    def start_recording(self):
        dest=ROOT/'recordings'/time.strftime('%Y%m%d_%H%M%S');dest.mkdir(parents=True,exist_ok=True)
        self.recorder={'dir':dest,'start':time.time(),'next':0.,'rows':0,'writer':None,'columns':None,
                       'csv':(dest/'twin_log.csv').open('w',newline='',encoding='utf-8-sig'),
                       'jsonl':(dest/'twin_log.jsonl').open('w',encoding='utf-8'),
                       'runs':set(),'modes':set(),'max_risk':None}
        self.rec_button.text='Stop recording';self.rec_button.style={'background_color':0xff2626b0}
        self.labels['rec'].style={'color':0xff5050ff}
        self.last_export='Recording: recordings/'+dest.name
        print('REC_START',dest,flush=True)

    def build_row(self,rec_time):
        """One flat row of the twin's state and readings (recorder CSV / JSONL and live display frames)."""
        case=self.cases[self.index]
        live=self.live_on and self.mode==0
        row={'rec_time_s':round(rec_time,3),'clock':time.strftime('%H:%M:%S'),
             'mode':'live' if live else 'measured' if self.mode==0 else 'physics',
             # run stays the recording id in live too: display_link takes its camera frames from that recording
             'run':case['id'],'replay_time_s':round(self.t,3),'vehicle_kmh':round(self.velocity*3.6,2),
             'lap_s_m':round(float(self.s),2),'lane_d_m':round(float(self.d),3),'heading_deg':round(float(self.yaw),2),
             'road_slope_deg':self.value('grade'),'road_bank_deg':self.value('bank'),
             'cargo_mass_kg':self.value('mass'),'straps_connected':all(self.belts_hooked()),
             'belt_c1_connected':self.flag('straps_c1'),'belt_c2_connected':self.flag('straps_c2'),
             'preload_c1_N':self.value('pretension_c1'),'preload_c2_N':self.value('pretension_c2')}
        for key,value in self.latest.items():
            if key in ('cargo_offset_mm','cargo_on_plate','cargo_secured'):
                row[key]=value
            elif not str(key).startswith('_') and key not in row:
                row[key]=(json.dumps(value) if isinstance(value,(list,tuple,dict)) else
                          round(value,4) if isinstance(value,float) else value)
        return row

    def record_row(self):
        r=self.recorder;now=time.time()-r['start']
        if now<r['next']:
            return
        r['next']=now+1/self.REC_HZ
        case=self.cases[self.index]
        row=self.build_row(now)
        if r['writer'] is None:
            r['columns']=list(row)
            r['writer']=csv.DictWriter(r['csv'],fieldnames=r['columns'],extrasaction='ignore',restval='')
            r['writer'].writeheader()
        r['writer'].writerow(row)
        r['jsonl'].write(json.dumps(row,ensure_ascii=False,default=str)+chr(10))
        r['rows']+=1;r['runs'].add(case['id']);r['modes'].add(row['mode'])
        risk=self.latest.get('risk')
        if isinstance(risk,(int,float)) and (r['max_risk'] is None or risk>r['max_risk']):
            r['max_risk']=risk

    def stop_recording(self):
        r=self.recorder
        if r is None:
            return
        self.recorder=None;self.last_recording=r['dir']
        r['csv'].close();r['jsonl'].close()
        duration=time.time()-r['start']
        summary={'started':time.strftime('%Y-%m-%d %H:%M:%S',time.localtime(r['start'])),'duration_s':round(duration,1),
                 'rows':r['rows'],'rate_hz':self.REC_HZ,'vehicle':args.vehicle,'runs':sorted(r['runs']),
                 'modes':sorted(r['modes']),'max_risk_level':r['max_risk'],
                 'files':{'twin_log.csv':'one row per 0.1 s (Excel)','twin_log.jsonl':'same rows, all fields'}}
        (r['dir']/'summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2),encoding='utf-8')
        self.rec_button.text='Start recording';self.rec_button.style={'background_color':0xff2d6b2d}
        self.labels['rec'].text=f"  저장됨 {r['rows']}줄 ({duration:.0f}초)";self.labels['rec'].style={'color':0xff9a9a9a}
        self.last_export='Recording saved: recordings/'+r['dir'].name+(
            chr(10)+'Team download: '+self.download_url if getattr(self,'download_url',None) else '')
        print('REC_STOP',r['dir'],r['rows'],flush=True)

    def toggle_display(self):
        if self.display.busy:
            self.display.stop()
        else:
            self.send_to_display()   # the live twin, until Stop display

    def send_to_display(self,name=None):
        """Stream to the display Pi until Stop display: the live twin, or a saved recording (looped) by name."""
        if self.display.busy:
            self.display.stop()
            if self.display.thread:
                self.display.thread.join(timeout=3)
        if not name:
            ok,message=self.display.start_live()
            print('DISPLAY_LIVE',ok,message,flush=True)
            return {'ok':ok,'message':message} if ok else {'ok':False,'error':message}
        folder=ROOT/'recordings'/str(name)
        if not folder.exists():
            return {'ok':False,'error':f'no recording {name}'}
        if self.recorder is not None and Path(self.recorder['dir'])==Path(folder):
            return {'ok':False,'error':'that recording is still running - press Stop recording first'}
        ok,message=self.display.send(folder)
        if not ok:
            self.display.status='Display: '+message
        print('DISPLAY_SEND',folder,ok,message,flush=True)
        return {'ok':ok,'message':message} if ok else {'ok':False,'error':message}

    # ------------------------------------------------------------------ remote control (twin_api.py)
    @staticmethod
    def as_bool(value):
        if isinstance(value,str):
            return value.strip().lower() in ('1','true','on','yes','y','켜','켜기')
        return bool(value)

    def find_case(self,value):
        text=str(value).strip()
        if text.isdigit() and int(text)<len(self.cases):
            return int(text)
        for i,case in enumerate(self.cases):
            if case['id']==text or case['id'].startswith(text):
                return i
        low=text.lower()
        for i,case in enumerate(self.cases):
            if low and (low in case['title'].lower() or low in (case.get('title_en') or '').lower()):
                return i
        return None

    def api_settings(self):
        out={key:{'label':label,'min':lo,'max':hi,'value':round(self.value(key),3)} for key,(lo,hi,label) in self.ranges.items()}
        out.update({key:{'label':label,'value':self.flag(key),'type':'toggle'} for key,label in self.toggles.items()})
        if 'time' in out:
            out['time']['max']=round(self.cases[self.index]['duration'],2)   # the slider range is set for the first run
        return out

    def api_action(self,action,name=None,value=None):
        """One remote action (main thread). Mirrors the panel buttons and sliders."""
        message=action
        if action=='mode':
            mode={'measured':0,'replay':0,'0':0,'physics':1,'sandbox':1,'1':1}.get(str(value).strip().lower())
            if mode is None:
                return {'ok':False,'error':'value must be "measured" or "physics"'}
            self.set_mode(mode);message='mode -> '+('measured replay' if mode==0 else 'physics sandbox')
        elif action=='run':
            index=self.find_case(value)
            if index is None:
                return {'ok':False,'error':f'no recording matches {value!r}; see GET /twin/commands'}
            self.case_model.get_item_value_model().set_value(index)
            message=f"run -> {self.cases[index]['id']} {self.cases[index].get('title_en') or self.cases[index]['title']}"
        elif action in ('play','pause'):
            self.playing=action=='play'
        elif action in ('reset','reseat_cargo'):
            self.request_reset()
        elif action=='go':
            self.go()
        elif action=='brake':
            self.brake()
        elif action in ('turn_left','turn_right'):
            return self.turn((90. if action=='turn_left' else -90.) if value is None else abs(float(value))*(1 if action=='turn_left' else -1))
        elif action=='turn':
            return self.turn(float(value or 0.))
        elif action=='straight':
            return self.straight()
        elif action=='select':
            # what a viewport click would do: only the cargo stays selected (guard_selection)
            # (no value: just report the current selection; the guard runs on the next frame)
            sel=omni.usd.get_context().get_selection()
            if value is not None:
                sel.set_selected_prim_paths([str(p) for p in (value if isinstance(value,(list,tuple)) else [value]) if p],True)
            return {'ok':True,'selected':list(sel.get_selected_prim_paths())}
        elif action=='set':
            key=str(name or '')
            if key in self.ranges:
                lo,hi,_=self.ranges[key]
                number=min(hi,max(lo,float(value)))
                self.models[key].set_value(number);message=f'{key} -> {number:g}'
                if key=='pretension':      # both belts, also when the slider already shows this value
                    for k in ('pretension_c1','pretension_c2'):
                        self.models[k].set_value(number)
            elif key=='straps':
                self.set_straps(self.as_bool(value));message=f'straps (both belts) -> {self.as_bool(value)}'
            elif key in self.toggles:
                self.models[key].set_value(self.as_bool(value));message=f'{key} -> {self.as_bool(value)}'
            else:
                return {'ok':False,'error':f'unknown setting {key!r}; settings: {sorted(list(self.ranges)+list(self.toggles))}'}
        elif action=='view':
            names=[v.lower() for v in self.VIEWS]
            if str(value).strip().lower() not in names:
                return {'ok':False,'error':f'view must be one of {list(self.VIEWS)}'}
            self.set_camera(names.index(str(value).strip().lower()));message=f'view -> {value}'
        elif action=='record_start':
            if self.recorder is not None:
                return {'ok':False,'error':'already recording'}
            self.start_recording();message='recording -> '+self.recorder['dir'].name
        elif action=='record_stop':
            if self.recorder is None:
                return {'ok':False,'error':'not recording'}
            rows=self.recorder['rows'];self.stop_recording()
            message=f'recording saved -> {self.last_recording.name} ({rows} rows)'
        elif action=='release_cargo':
            self.set_straps(False)
        elif action=='export':
            self.request_export()
        elif action=='hq_photo':
            self.request_hq()
        elif action=='send_to_display':
            return self.send_to_display(value or None)
        elif action=='stop_display':
            self.display.stop()
        elif action=='live':
            return self.set_live(None if value is None else self.as_bool(value))
        elif action=='move_cargo':
            try:
                dx,dy=(float(v) for v in (value if isinstance(value,(list,tuple)) else str(value).replace(',',' ').split())[:2])
            except (TypeError,ValueError):
                return {'ok':False,'error':'value: [right_mm, rear_mm] on the plate, e.g. [50, 0] = 5 cm to the right'}
            if self.mode!=1:
                # replay plays recorded sensors: switch to the sandbox, the move follows once the cargo is placed
                self.queued_move=(dx,dy);self.pending_mode=1
                return {'ok':True,'message':f'switching to the physics sandbox, then cargo {dx:+.0f} mm right, {dy:+.0f} mm rear'}
            M=self.rig_matrix()
            pos,quat=self.cargo.get_world_pose()
            shift=np.array(M.TransformDir(Gf.Vec3d(dy/1000.,dx/1000.,0.)))   # rig x = towards the rear belt, rig y = right
            body=Gf.Matrix4d().SetRotate(Gf.Quatd(float(quat[0]),Gf.Vec3d(*map(float,quat[1:]))))
            body.SetTranslateOnly(Gf.Vec3d(*map(float,np.asarray(pos)+shift)))
            self.teleport_cargo(body);message=f'cargo moved {dx:+.0f} mm right, {dy:+.0f} mm rear'
        elif action=='_simulate_gizmo':
            # test hook: the same edit the viewport Move tool makes on the scanned boxes (name 'body': on the body)
            if name=='body':
                op=self.cargo.prim.GetAttribute('xformOp:translate')
                op.Set(op.Get()+Gf.Vec3d(0.,float(value or .05),0.))
                return {'ok':True,'message':'cargo body moved in USD'}
            prim=self.stage.GetPrimAtPath('/World/Cargo/ScanRoot')
            local=UsdGeom.Xformable(prim).GetLocalTransformation()
            local.SetTranslateOnly(local.ExtractTranslation()+Gf.Vec3d(0.,float(value or .05),0.))
            UsdGeom.Xformable(prim).MakeMatrixXform().Set(local)
            return {'ok':True,'message':'scan part moved in USD'}
        elif action=='render_quality':
            if not self.set_quality(str(value).strip().lower()):
                return {'ok':False,'error':f'value must be one of {list(QUALITY)}'}
            message=f'render quality -> {value}'
        elif action=='_screenshot':
            self.pending_capture=capture_viewport_to_file(get_active_viewport(),str(value))
            return {'ok':True}
        else:
            return {'ok':False,'error':f'unknown action {action!r}'}
        self.last_export='Remote control: '+message
        print('API',message,flush=True)
        return {'ok':True,'message':message}

    def api_snapshot(self):
        f=self.latest;case=self.cases[self.index]
        lv=lambda key:level_of(f.get(key))
        name=lambda key:level_name(lv(key))
        cell=lambda key:round(f[key+'_g']/1000,3) if isinstance(f.get(key+'_g'),(int,float)) else None
        r=self.recorder
        return {'ready':True,'clock':time.strftime('%H:%M:%S'),'mode':'measured' if self.mode==0 else 'physics',
                'playing':self.playing,'run':{'index':self.index,'id':case['id'],'title':case.get('title_en') or case['title'],
                                              'title_ko':case['title'],'duration_s':case['duration']},
                'replay_time_s':round(self.t,2),
                'vehicle':{'speed_kmh':round(self.velocity*3.6,1),'lap_m':round(float(self.s),1),'lap_length_m':round(self.road.length,1),
                           'lane_m':round(float(self.d),2),'heading_deg':round(float(self.yaw),1),'steer_dps':self.value('steer'),
                           'driving':'manual','turning':self.turn_target is not None,'edge_stop':bool(self.edge_stop)},
                'judgement':{'overall':name('risk'),'load':name('lv_load'),'cog':name('lv_cog'),'cargo_shift':name('lv_move'),
                             'belt_front_c1':name('lv_c1'),'belt_rear_c2':name('lv_c2'),
                             'levels':{k:lv(k) for k in ('risk','lv_load','lv_cog','lv_move','lv_c1','lv_c2')},
                             'cargo_shift_mm':None if f.get('shift_mm') is None else round(f['shift_mm'],1),
                             'cargo_shift_reason':f.get('shift_reason') or None,'text_ko':f.get('risk_text') or None,
                             'estimated':bool(f.get('estimated'))},
                'sensors':{'cells_kg':{k:cell(k) for k in ('NW','NE','SW','SE')},'total_kg':cell('total'),
                           'load_kg':f.get('load_kg'),'cog_mm':[f.get('cog_x_mm'),f.get('cog_y_mm')],
                           'belts_N':{'c1_front':f.get('c1_N'),'c2_rear':f.get('c2_N')},
                           'imu_G':[f.get('imu1_ax_G'),f.get('imu1_ay_G'),f.get('imu1_az_G')]},
                'recording':{'active':r is not None,'name':r['dir'].name if r else None,'rows':r['rows'] if r else None,
                             'seconds':round(time.time()-r['start'],1) if r else None},
                'last_recording':self.last_recording.name if self.last_recording else None,
                'stream':getattr(self,'stream_qos',None),'display':self.display.status,'live':{'on':self.live_on,'status':self.live.status(),'frames':self.live_count},
                'view':self.VIEWS[self.camera_view],'fps':round(self.fps,1),
                'render_quality':getattr(self,'quality',None),
                'status':self.last_export}

    def request_export(self):
        self.export_requested=True

    def request_hq(self):
        """Path-traced still of the current view: the scene freezes, the renderer accumulates samples for
        about 1.5 s, the image is saved to exports/, then real-time rendering resumes."""
        if self.hq_state is None:
            self.hq_state='start'

    HQ_FRAMES=90

    def hq_step(self):
        """One frame of the HQ export state machine; returns True while the simulation must stay frozen."""
        settings=carb.settings.get_settings()
        if self.hq_state=='start':
            self.hq_restore={k:settings.get(k) for k in ForestScene.path_tracing_settings()}
            for key,value in ForestScene.path_tracing_settings().items():
                settings.set(key,value)
            self.hq_frames=0;self.hq_state='accumulate'
            self.labels['status'].text='고화질 사진: 경로 추적 렌더링 중...'
        elif self.hq_state=='accumulate':
            self.hq_frames+=1
            if self.hq_frames==self.HQ_FRAMES:
                dest=ROOT/'exports'/(time.strftime('%Y%m%d_%H%M%S')+'_hq');dest.mkdir(parents=True,exist_ok=True)
                self.pending_capture=capture_viewport_to_file(get_active_viewport(),str(dest/'photo.png'))
                self.hq_dest=dest;self.hq_state='capture'
        elif self.hq_state=='capture':
            self.hq_frames+=1
            if self.hq_frames>=self.HQ_FRAMES+6:
                for key,value in self.hq_restore.items():
                    if value is not None:
                        settings.set(key,value)
                settings.set('/rtx/rendermode','RayTracedLighting')
                self.last_export='HQ photo: '+self.hq_dest.name
                print('HQ_EXPORT',self.hq_dest,flush=True)
                self.hq_state=None
        app.update()
        return self.hq_state is not None

    def export(self):
        dest=ROOT/'exports'/time.strftime('%Y%m%d_%H%M%S');dest.mkdir(parents=True,exist_ok=True)
        self.stage.Export(str(dest/'truck_twin.usda'))
        case=self.cases[self.index]
        payload={'mode':'measured' if self.mode==0 else 'physics_estimated','run':case['id'],'title':case['title'],
                 'vehicle':args.vehicle,'settings':{k:m.get_value_as_string() for k,m in self.models.items()},
                 'readings':self.latest,'notes':self.data['notes']}
        (dest/'session.json').write_text(json.dumps(payload,ensure_ascii=False,indent=2),encoding='utf-8')
        with (dest/'sensor_snapshot.csv').open('w',newline='',encoding='utf-8-sig') as stream:
            writer=csv.DictWriter(stream,fieldnames=self.latest.keys());writer.writeheader();writer.writerow(self.latest)
        if self.last_depth is not None:
            np.save(dest/'depth_m.npy',self.last_depth)
        self.pending_capture=capture_viewport_to_file(get_active_viewport(),str(dest/'viewport.png'))
        self.last_export='Saved: '+dest.name
        print('EXPORT',dest,flush=True)

    def camera_offset(self):
        state=ViewportCameraState('/OmniverseKit_Persp',get_active_viewport())
        return np.array(state.position_world)-self.cart_center()

    def verify(self):
        if self.frame_count==20:
            # The same command the Delete key runs; protected prims must survive it.
            import omni.kit.commands
            paths=[self.rig_root+'/Deck','/World/Cart','/World/Cart/Truck' if TRUCK else '/World/Operator']
            omni.kit.commands.execute('DeletePrims',paths=paths)
            survived=all(self.stage.GetPrimAtPath(p).IsValid() for p in paths)
            self.verification.append({'mode':'delete_protection','survived':survived})
            print('DELETE_PROTECTION',survived,flush=True)
            self.brake();braked=self.value('speed');self.go();resumed=self.value('speed')
            self.verification.append({'mode':'brake_go','braked':braked,'resumed':resumed})
            print('BRAKE_GO',braked,resumed,flush=True)
            self.surface_snapshot=np.array(self.surface.points.Get());self.x_snapshot=float(self.distance)
            if TRUCK:
                self.wheel_snapshot=tuple(UsdGeom.Xformable(self.truck.wheels[0]).GetOrderedXformOps()[1].Get())
        if self.frame_count==40:
            # A user-chosen view (as after an Alt-orbit) must keep its offset relative to the moving truck.
            center=self.cart_center()
            set_camera_view(eye=center+np.array([2.5,3.,1.4]),target=center,camera_prim_path='/OmniverseKit_Persp')
            self.cam_offset=None;self.cam_dragging=True
            self.follow_check={'start_offset':self.camera_offset().tolist(),'start_cart':center.tolist()}
        if self.frame_count==85:
            offset=self.camera_offset()
            self.follow_check.update(end_offset=offset.tolist(),end_cart=self.cart_center().tolist(),
                                     offset_drift_m=float(np.linalg.norm(offset-np.array(self.follow_check['start_offset']))))
            self.verification.append({'mode':'follow_camera',**self.follow_check})
            changed=float(np.abs(np.array(self.surface.points.Get())-self.surface_snapshot).max())
            self.verification.append({'mode':'replay_motion','surface_point_change_m':changed,
                                      'distance_advance_m':float(self.distance-self.x_snapshot),'speed_mps':self.velocity,
                                      'lap_s_m':self.s,'lane_d_m':self.d,'risk':self.latest.get('risk'),'driver':bool(self.driver)})
            print('REPLAY_MOTION',round(changed,4),round(float(self.distance-self.x_snapshot),3),flush=True)
            if TRUCK:
                now=tuple(UsdGeom.Xformable(self.truck.wheels[0]).GetOrderedXformOps()[1].Get())
                # Rolling must change only the Y (axle) angle: X stays 90 (tilt), Z is the steer angle.
                self.verification.append({'mode':'wheel_roll','before':self.wheel_snapshot,'after':now,
                                          'rolls_about_axle':bool(abs(now[1]-self.wheel_snapshot[1])>1 and abs(now[0]-90)<1e-6)})
                print('WHEEL_ROLL',self.wheel_snapshot,now,flush=True)
        if self.frame_count==30:
            self.rec_dirs_before=sorted(p.name for p in (ROOT/'recordings').glob('*')) if (ROOT/'recordings').exists() else []
            self.verification.append({'mode':'no_recording_at_start','recorder_off':self.recorder is None})
            self.start_recording()
        if self.frame_count==80:
            rec_dir=self.recorder['dir'];self.stop_recording()
            rows=(rec_dir/'twin_log.csv').read_text(encoding='utf-8-sig').splitlines()
            summary=json.loads((rec_dir/'summary.json').read_text(encoding='utf-8'))
            self.verification.append({'mode':'recording','dir':rec_dir.name,'csv_lines':len(rows),'header':rows[0][:200],
                                      'summary':summary,'recorder_off_after_stop':self.recorder is None})
            print('REC_CHECK',rec_dir.name,len(rows),summary['duration_s'],flush=True)
        if self.frame_count==90:
            self.verification.append({'mode':'replay','readings':self.latest,'depth_shape':list(self.last_depth.shape) if self.last_depth is not None else None})
            self.export();self.set_mode(1);self.models['grade'].set_value(12.)
        if self.frame_count==170:
            self.verification.append({'mode':'physics','position':self.cargo.get_world_pose()[0].tolist(),'readings':self.latest})
            self.set_camera(0);self.export()
            self.set_straps(False)
        if self.frame_count==240:
            self.verification.append({'mode':'released','position':self.cargo.get_world_pose()[0].tolist(),
                                      'lv_move':self.latest.get('lv_move'),'shift_reason':self.latest.get('shift_reason')})
            print('SANDBOX_SHIFT',self.latest.get('lv_move'),self.latest.get('shift_reason'),flush=True)
            (ROOT/'verification.json').write_text(json.dumps(self.verification,indent=2,default=str),encoding='utf-8')
            self.models['grade'].set_value(0);self.set_straps(True);self.set_mode(0)
            print('FOREST_VERIFIED',flush=True)
        if self.frame_count==420:
            self.set_camera(0);self.request_hq()
        if self.frame_count==900:
            # Second viewport export once vegetation materials have finished compiling.
            self.set_camera(0);self.export()
            print('FOREST_FINAL_EXPORT',flush=True)
            # Cargo-shift judgement on the recorded 'no rear belt, cargo shift' run (Pi level 0 throughout).
            self.select_case(next(i for i,c in enumerate(self.cases) if c['id']=='20261015_212927'))
            self.models['loop'].set_value(False);self.playing=True
        if self.frame_count==1360:
            entry={'mode':'cargo_shift_replay','run':self.cases[self.index]['id'],'t':self.t,'algo':self.latest.get('algo'),
                   'lv_move':self.latest.get('lv_move'),'lv_move_twin':self.latest.get('lv_move_twin'),
                   'recorded':self.latest.get('recorded_judgement'),'load_kg':self.latest.get('load_kg'),
                   'lv_move_pi':self.latest.get('lv_move_pi'),'shift_mm':self.latest.get('shift_mm'),
                   'reason':self.latest.get('shift_reason'),'risk':self.latest.get('risk')}
            self.verification.append(entry)
            (ROOT/'verification.json').write_text(json.dumps(self.verification,indent=2,default=str),encoding='utf-8')
            print('SHIFT_CHECK',json.dumps(entry,default=str),flush=True)
            # Sandbox: drive off, reach cruise, 12 deg slope with straps on (cargo must stay), then release.
            self.set_mode(1);self.models['grade'].set_value(0);self.set_straps(True);self.playing=True;self.go()
        if self.frame_count==1760:
            self.sandbox_ref=self.latest.get('lv_move');self.models['grade'].set_value(12.)
        if self.frame_count==1940:
            self.sandbox_slope={'lv_move':self.latest.get('lv_move'),'lv_move_twin':self.latest.get('lv_move_twin'),'shift_mm':self.latest.get('shift_mm'),
                                'belt_N':self.latest.get('c1_N'),'load_kg':self.latest.get('load_kg'),'cells_kg':(self.latest.get('total_g') or 0)/1000,
                                'mass_kg':self.value('mass'),'levels':{k:self.latest.get(k) for k in ('risk','lv_load','lv_cog','lv_c1','lv_c2')},
                                'algo':self.latest.get('algo')}
            self.set_straps(False)
        if self.frame_count==2300:
            entry={'mode':'sandbox_slope_release','level_at_cruise':self.sandbox_ref,'strapped_on_12deg':self.sandbox_slope,
                   'after_release':{'lv_move':self.latest.get('lv_move'),'shift_mm':self.latest.get('shift_mm'),'reason':self.latest.get('shift_reason')}}
            self.verification.append(entry)
            (ROOT/'verification.json').write_text(json.dumps(self.verification,indent=2,default=str),encoding='utf-8')
            self.models['grade'].set_value(0);self.set_straps(True);self.set_mode(0)
            print('SANDBOX_SLOPE_CHECK',json.dumps(entry,default=str),flush=True)


try:
    twin=Twin()
    failures=[]
    while app.is_running():
        if len(failures)>=5:
            # Repeated errors: stop driving the scene but keep the window and stream alive.
            app.update()
            continue
        try:
            twin.advance(1/60)
            if args.verify:
                twin.verify()
        except Exception:
            traceback.print_exc()
            now=time.perf_counter()
            failures=[t for t in failures if now-t<60]+[now]
            twin.recover()
            if len(failures)>=5:
                twin.labels['status'].text='오류가 반복되어 장면을 멈췄어요 - 앱을 다시 켜 주세요.'
                print('FOREST_SCENE_STOPPED',flush=True)
except Exception:
    traceback.print_exc()
    raise
finally:
    try:
        twin.stop_recording()
    except Exception:
        pass
    app.close()
