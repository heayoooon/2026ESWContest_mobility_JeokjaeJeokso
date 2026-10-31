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
    'go': 'drive at cruise speed', 'brake': 'stop the vehicle', 'turn_left': 'steer left (+25 deg/s)',
    'turn_right': 'steer right (-25 deg/s)', 'straight': 'steer straight',
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
        from fastapi import Body, HTTPException
        from fastapi.responses import FileResponse, HTMLResponse, JSONResponse, Response
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

        @router.get('/mcp')
        def mcp():
            return FileResponse(mcp_file, filename='isaac_twin_mcp.py', media_type='text/x-python')

        main.register_router(router=router, prefix='/twin', tags=['twin'])


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
