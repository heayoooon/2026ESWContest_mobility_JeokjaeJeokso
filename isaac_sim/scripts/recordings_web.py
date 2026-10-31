"""Download page for twin recordings, served by Kit's own HTTP server (port 8211, same as the stream).

Teammates open http://<this PC>:8211/recordings/ and download a recording as one zip or file by file.
Read-only: only folders named YYYYMMDD_HHMMSS inside recordings/ and the three files the recorder
writes can be fetched.
"""
import html
import io
import json
import re
import zipfile
from pathlib import Path

NAME = re.compile(r'^\d{8}_\d{6}$')
FILES = ('twin_log.csv', 'twin_log.jsonl', 'summary.json')
LEVELS = {0: '정상', 1: '주의', 2: '경고', 3: '위험'}


def register(root, active_dir=lambda: None, to_display=None):
    """Mount /recordings on Kit's HTTP server. active_dir() returns the folder being recorded, if any;
    to_display(name) plays a recording on the display Pi and returns {'ok': bool, ...}."""
    from fastapi import HTTPException
    from fastapi.responses import FileResponse, HTMLResponse, JSONResponse, Response
    from omni.services.core import main, routers

    rec = Path(root) / 'recordings'
    router = routers.ServiceAPIRouter()

    def folder(name):
        path = rec / name
        if not NAME.match(name) or not path.is_dir():
            raise HTTPException(status_code=404, detail='recording not found')
        return path

    @router.get('/', response_class=HTMLResponse)
    def index():
        active = active_dir()
        rows = []
        for path in sorted((p for p in rec.glob('*') if p.is_dir() and NAME.match(p.name)), reverse=True):
            info = {}
            try:
                info = json.loads((path / 'summary.json').read_text(encoding='utf-8'))
            except Exception:
                pass
            busy = active is not None and Path(active) == path
            when = f'{path.name[:4]}-{path.name[4:6]}-{path.name[6:8]} {path.name[9:11]}:{path.name[11:13]}:{path.name[13:15]}'
            if busy:
                detail = '<b class="rec">기록 중…</b> (정지 후 다운로드하세요)'
                links = ''
            else:
                risk = info.get('max_risk_level')
                level = LEVELS.get(int(risk), '-') if isinstance(risk, (int, float)) else '-'
                detail = (f"{info.get('duration_s', '?')} s · {info.get('rows', '?')} 줄 · 최고 등급 {level} · "
                          f"{html.escape(', '.join(info.get('modes', [])))}")
                links = (f'<a class="zip" href="{path.name}.zip">ZIP 전체 받기</a> ' +
                         ' '.join(f'<a href="{path.name}/{f}">{f}</a>' for f in FILES if (path / f).exists()) +
                         (f' <button onclick="show(\'{path.name}\')">디스플레이로 보내기</button>' if to_display else ''))
            rows.append(f'<tr><td>{when}</td><td>{detail}</td><td>{links}</td></tr>')
        body = ''.join(rows) or '<tr><td colspan="3">아직 기록이 없습니다. Isaac Sim 패널의 Start recording을 눌러 기록하세요.</td></tr>'
        return f'''<!doctype html><html lang="ko"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1"><title>적재적소 트윈 기록</title>
<style>body{{font-family:system-ui,"Malgun Gothic",sans-serif;margin:24px;background:#16191d;color:#e6e6e6}}
table{{border-collapse:collapse;width:100%}}td,th{{border-bottom:1px solid #333;padding:8px;text-align:left}}
a{{color:#7cc4ff;margin-right:10px}}a.zip{{color:#7dff9a;font-weight:600}}.rec{{color:#ff6b6b}}
p{{color:#9aa}}</style></head><body><h2>적재적소 디지털 트윈 – 기록 다운로드</h2>
<p>CSV는 엑셀로 바로 열립니다. 새로고침하면 목록이 갱신됩니다. '디스플레이로 보내기'는 그 기록을 디스플레이 파이 트윈 화면에서 재생합니다.</p><p id="msg"></p>
<table><tr><th>기록 시작</th><th>내용</th><th>다운로드</th></tr>{body}</table><script>async function show(n){{const r=await fetch(n+'/display',{{method:'POST'}});const j=await r.json();document.getElementById('msg').textContent=(j.ok?'보내는 중: ':'실패: ')+(j.message||j.error||'');}}</script><style>button{{background:#2d6b2d;color:#fff;border:0;border-radius:4px;padding:3px 8px;cursor:pointer}}</style></body></html>'''

    @router.get('/index.json')
    def index_json():
        items = []
        for path in sorted((p for p in rec.glob('*') if p.is_dir() and NAME.match(p.name)), reverse=True):
            info = {}
            try:
                info = json.loads((path / 'summary.json').read_text(encoding='utf-8'))
            except Exception:
                pass
            busy = active_dir() is not None and Path(active_dir()) == path
            items.append({'name': path.name, 'recording_now': busy, **info,
                          'zip': f'/recordings/{path.name}.zip'})
        return JSONResponse(items)

    @router.post('/{name}/display')
    def display(name: str):
        folder(name)
        if to_display is None:
            raise HTTPException(status_code=503, detail='display link not available')
        return JSONResponse(to_display(name))

    @router.get('/{name}.zip')
    def download_zip(name: str):
        path = folder(name)
        if active_dir() is not None and Path(active_dir()) == path:
            raise HTTPException(status_code=409, detail='still recording - press Stop recording first')
        buffer = io.BytesIO()
        with zipfile.ZipFile(buffer, 'w', zipfile.ZIP_DEFLATED) as archive:
            for f in FILES:
                if (path / f).exists():
                    archive.write(path / f, f'{name}/{f}')
        return Response(buffer.getvalue(), media_type='application/zip',
                        headers={'Content-Disposition': f'attachment; filename="twin_{name}.zip"'})

    @router.get('/{name}/{file}')
    def download_file(name: str, file: str):
        path = folder(name)
        if file not in FILES or not (path / file).exists():
            raise HTTPException(status_code=404, detail='file not found')
        return FileResponse(path / file, filename=f'{name}_{file}')

    main.register_router(router=router, prefix='/recordings', tags=['recordings'])
    return router
