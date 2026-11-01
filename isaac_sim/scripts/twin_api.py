"""Remote-control API for the twin, served by Kit's HTTP server next to the stream (port 8211, share mode).

Teammates' Claude (or curl) drives the twin with the same actions as the panel:
  GET  /twin/              Korean guide page (how to connect Claude), MCP server download
  GET  /twin/state         current state: mode, run, replay time, vehicle, judgement, sensors, recording, display
  GET  /twin/commands      every action with its parameters and ranges
  POST /twin/command       {"action": "...", "name": "...", "value": ...} -> result after it ran
  GET  /twin/screenshot    JPEG of the current view
  GET  /twin/mcp           isaac_twin_mcp.py (stdio MCP server for Claude Code / Claude Desktop)

HTTP handlers run on worker threads; every action is queued and executed by the twin's main loop
(USD / PhysX / omni.ui are not thread-safe), and the handler waits for the result.
"""
import json
import os
import queue
import threading
import time
from pathlib import Path

COMMANDS = {
    'mode': 'value: "measured" (recorded replay) or "physics" (PhysX sandbox)',
    'run': 'value: recording id (e.g. 20261015_212927), list index, or part of its title',
    'play': 'start replay', 'pause': 'pause replay', 'reset': 'restart the recording / re-seat the cargo',
    'go': 'drive at cruise speed', 'brake': 'stop the vehicle',
    'turn_left': 'turn left now (value: degrees, default 90), then drive straight on the paved area',
    'turn_right': 'turn right now (value: degrees, default 90)', 'turn': 'value: degrees to turn now (+ left)',
    'straight': 'stop turning and keep the heading',
    'select': 'value: prim path(s) to select; only the cargo (/World/Cargo...) can be selected / moved',
    'set': 'name: a slider or toggle from GET /twin/commands, value: number (clamped to its range) or true/false',
    'view': 'value: Front, Rear, Side, Aerial, Depth or Cab',
    'record_start': 'start saving data (recordings/<time>/)', 'record_stop': 'stop and save the recording',
    'release_cargo': 'unhook the straps (physics mode)', 'reseat_cargo': 'put the cargo back',
    'export': 'save USD + settings + snapshot to exports/', 'hq_photo': 'path-traced still to exports/',
    'send_to_display': 'no value: stream the live twin to the display Pi until stop_display; '
                       'value: recording name -> loop that recording on the display until stop_display',
    'stop_display': 'stop sending to the display Pi',
    'live': 'value true/false (none = toggle): show the sensor Pi\'s live frames (mirrored to this PC, port 5010) '
            'instead of a recording - cargo surface, sensors and judgement as they are now',
    'render_quality': 'value: fast, balanced or high - viewport resolution / anti-aliasing (high = full HD + DLAA, less shimmer, lower FPS)',
    'move_cargo': 'physics mode: value [right_mm, rear_mm] - push the cargo on the plate (e.g. [50, 0]); cells, belts and judgement follow',
}


class TwinAPI:
    def __init__(self, twin):
        self.twin = twin
        self.jobs = queue.Queue()
        self.snapshot = {}
        self.shot_lock = threading.Lock()

    # ---------------------------------------------------------------- main-thread side
    def pump(self, limit=8):
        """Run queued actions (called once per frame by the twin)."""
        for _ in range(limit):
            try:
                job = self.jobs.get_nowait()
            except queue.Empty:
                return
            try:
                job['result'] = self.twin.api_action(job['action'], job.get('name'), job.get('value'))
            except Exception as error:
                job['result'] = {'ok': False, 'error': f'{error.__class__.__name__}: {error}'}
            job['done'].set()

    def submit(self, action, name=None, value=None, timeout=6.):
        job = {'action': action, 'name': name, 'value': value, 'done': threading.Event()}
        self.jobs.put(job)
        if not job['done'].wait(timeout):
            return {'ok': False, 'error': 'the twin did not answer in time (busy with an HQ photo?)'}
        return job['result']

    # ---------------------------------------------------------------- HTTP side
    def register(self, root, mcp_file):
        from fastapi import Body, HTTPException, Request
        from fastapi.responses import FileResponse, HTMLResponse, JSONResponse, RedirectResponse, Response
        from omni.services.core import main, routers

        router = routers.ServiceAPIRouter()
        api = self

        @router.get('/', response_class=HTMLResponse)
        def guide():
            return GUIDE_HTML

        @router.get('/state')
        def state():
            return JSONResponse(api.snapshot or {'ready': False})

        @router.get('/commands')
        def commands():
            return JSONResponse({'actions': COMMANDS, 'settings': api.twin.api_settings(),
                                 'runs': [{'index': i, 'id': c['id'], 'title': c.get('title_en') or c['title'], 'title_ko': c['title']}
                                          for i, c in enumerate(api.twin.cases)],
                                 'views': list(api.twin.VIEWS)})

        @router.post('/command')
        def command(payload: dict = Body(...)):
            action = str(payload.get('action', ''))
            if action not in COMMANDS and not (action=='_simulate_gizmo' and os.environ.get('TWIN_TEST')=='1'):
                raise HTTPException(status_code=400, detail=f'unknown action {action!r}; see GET /twin/commands')
            return JSONResponse(api.submit(action, payload.get('name'), payload.get('value')))

        @router.get('/screenshot')
        def screenshot():
            with api.shot_lock:
                path = Path(root) / 'exports' / 'api_screenshot.png'
                if path.exists():
                    path.unlink()
                result = api.submit('_screenshot', value=str(path))
                if not result.get('ok'):
                    raise HTTPException(status_code=503, detail=result.get('error'))
                deadline = time.time() + 6
                while time.time() < deadline:
                    if path.exists() and path.stat().st_size > 0:
                        time.sleep(.2)
                        break
                    time.sleep(.1)
                else:
                    raise HTTPException(status_code=504, detail='screenshot was not written')
                import cv2
                image = cv2.imread(str(path))
                ok, jpeg = cv2.imencode('.jpg', image, [cv2.IMWRITE_JPEG_QUALITY, 85])
                return Response(jpeg.tobytes(), media_type='image/jpeg')

        @router.get('/view', response_class=HTMLResponse)
        def view(request: Request):
            # The stock page (/streaming/webrtc-demo/) asks Kit for a 1280x720 window and the player then caps the
            # bitrate from that size (~7-12 Mbps): a small, soft picture. This page asks for 1920x1080 and a higher
            # cap; the player's own QoS still lowers it on a slow link. ?resolution=WxH&maxbitrate=kbps override.
            # 40/60 Mbps caps stuttered over the internet (bursts beyond the upload); 20/30 Mbps still look sharp.
            q = dict(request.query_params)
            res = getattr(api.twin, 'stream_res', '1920x1080')          # the twin's --stream-res
            big = int(res.lower().split('x')[0]) > 2000
            want = {'server': request.url.hostname, 'resolution': res, 'maxbitrate': '30000' if big else '20000'}
            if any(k not in q for k in want):
                from urllib.parse import urlencode
                return RedirectResponse('/twin/view?' + urlencode({**want, **q}))
            w, h = (int(v) for v in q['resolution'].lower().split('x'))
            return VIEW_HTML.replace('__W__', str(w)).replace('__H__', str(h))

        @router.get('/js/{name}')
        def player_js(name: str):
            # kit-player.js starts its worker from ./js/ragnarokworker.js, i.e. /twin/js/ for the page above
            import omni.kit.app
            mgr = omni.kit.app.get_app().get_extension_manager()
            ext = mgr.get_extension_id_by_module('omni.services.streamclient.webrtc')
            path = Path(mgr.get_extension_path(ext)) / 'web' / 'js' / Path(name).name
            if not path.exists():
                raise HTTPException(status_code=404)
            return FileResponse(path, media_type='application/javascript')

        @router.get('/mcp')
        def mcp():
            return FileResponse(mcp_file, filename='isaac_twin_mcp.py', media_type='text/x-python')

        main.register_router(router=router, prefix='/twin', tags=['twin'])


VIEW_HTML = '''<!doctype html><html lang="ko"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>적재적소 트윈 (고화질)</title>
<style>
html,body{margin:0;height:100%;background:#000;color:#ddd;font-family:sans-serif;overflow:hidden}
#wrap{position:absolute;inset:0;display:flex;align-items:center;justify-content:center}
/* keep the element exactly 16:9-ish (W:H) so mouse positions map 1:1 onto the stream */
#remote-video{width:min(100vw,calc(100vh*__W__/__H__))!important;height:auto!important;aspect-ratio:__W__/__H__;background:#000}
#bar{position:absolute;right:10px;bottom:8px;display:flex;gap:8px;opacity:.35}
#bar:hover{opacity:1}
button{background:#333;color:#eee;border:1px solid #555;border-radius:6px;padding:6px 12px;font-size:14px;cursor:pointer}
#overlay{position:absolute;left:50%;top:50%;transform:translate(-50%,-50%)}
#play{font-size:28px;padding:18px 34px;background:#b02a2a}
</style></head><body>
<div id="wrap"><video id="remote-video" width="__W__" height="__H__" playsinline tabindex="-1"></video><audio id="remote-audio"></audio></div>
<div id="overlay"><button id="play" disabled>▶ 연결 중…</button></div>
<div id="bar"><button id="full">전체 화면</button><button id="stop" disabled>■ 끊기</button></div>
<script src="/streaming/webrtc-demo/js/kit-player.js"></script>
<script>
(async()=>{
  const play=document.getElementById('play'),stop=document.getElementById('stop');
  await SetupWebRTCPlayer({videoElementId:'remote-video',audioElementId:'remote-audio',playElement:play,stopElement:stop,
                           videoWidth:__W__,videoHeight:__H__});
  play.disabled=false;play.textContent='▶ 보기';
  const overlay=document.getElementById('overlay');
  play.addEventListener('click',()=>{overlay.style.display='none';stop.disabled=false;});
  stop.addEventListener('click',()=>{overlay.style.display='';play.textContent='▶ 다시 보기';stop.disabled=true;});
  document.getElementById('full').addEventListener('click',()=>document.documentElement.requestFullscreen());
})();
</script></body></html>'''


GUIDE_HTML = '''<!doctype html><html lang="ko"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1"><title>트윈 원격 조종</title>
<style>body{font-family:system-ui,"Malgun Gothic",sans-serif;margin:24px;max-width:900px;background:#16191d;color:#e6e6e6;line-height:1.6}
code,pre{background:#23272e;border-radius:6px;padding:2px 6px}pre{padding:12px;overflow-x:auto}a{color:#7cc4ff}h3{margin-top:28px}</style>
</head><body><h2>적재적소 디지털 트윈 – 내 Claude로 조종하기</h2>
<p>이 주소의 Isaac Sim 트윈은 스트림 화면 없이도 <b>각자의 Claude</b>가 직접 조종할 수 있습니다.
화면을 보고 있는 사람과 동시에 써도 됩니다 (같은 트윈을 움직입니다).</p>
<h3>방법 1. Claude Code / Claude Desktop에 연결 (추천)</h3>
<ol><li><a href="mcp">isaac_twin_mcp.py</a> 를 내려받습니다 (Python 3만 있으면 됨, 추가 설치 없음).</li>
<li>Claude Code: 터미널에서 <pre>claude mcp add isaac-twin -- python 내려받은경로/isaac_twin_mcp.py --url http://HOST</pre></li>
<li>Claude Desktop: 설정 → 개발자 → 구성 편집(claude_desktop_config.json)에
<pre>"mcpServers": {"isaac-twin": {"command": "python", "args": ["내려받은경로/isaac_twin_mcp.py", "--url", "http://HOST"]}}</pre> 넣고 다시 시작.</li>
<li>Claude에게 말하기: <i>"트윈 상태 보여줘"</i>, <i>"뒷벨트없음화물이동 기록 틀고 화면 캡처해줘"</i>,
<i>"물리 모드로 바꾸고 경사 10도, 벨트 풀어서 화물 이동 판정 확인해줘"</i>, <i>"기록 시작하고 20초 달린 뒤 멈추고 디스플레이로 보내줘"</i></li></ol>
<h3>방법 2. Claude에게 주소만 알려주기 (curl)</h3>
<p>MCP 없이도 Claude Code에 "<code>http://HOST/twin/commands</code> 를 읽고 이 트윈을 조종해" 라고 하면 됩니다.</p>
<pre>curl http://HOST/twin/state
curl http://HOST/twin/commands
curl -X POST http://HOST/twin/command -H "Content-Type: application/json" -d "{\\"action\\":\\"go\\"}"
curl -X POST http://HOST/twin/command -H "Content-Type: application/json" -d "{\\"action\\":\\"set\\",\\"name\\":\\"grade\\",\\"value\\":10}"
curl -o view.jpg http://HOST/twin/screenshot</pre>
<h3>할 수 있는 것</h3>
<p>모드 전환 · 기록 선택 · 재생/정지 · 운전(Go/Brake/좌우) · 모든 슬라이더(경사, 속도, 화물 무게, 마찰, 벨트 장력 …) ·
시점 · 데이터 기록 시작/정지 · 디스플레이 파이로 보내기 · 화면 캡처 · 판정/센서 값 읽기.
기록 파일은 <a href="/recordings/">/recordings/</a> 에서 받습니다.</p>
<script>document.body.innerHTML=document.body.innerHTML.replaceAll('http://HOST','http://'+location.host)</script>
</body></html>'''
