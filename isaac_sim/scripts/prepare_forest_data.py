"""Preserve raw channels, source units, missing values and video provenance."""
import bisect
import csv
import json
import math
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
TITLES = {'20261003_165743': 'Constant drive', '20261003_170015': 'Start / brake',
          '20261003_170232': 'Right turn', '20261003_170459': 'Cargo removal'}

def rows(path):
    if not path.exists():
        return []
    with path.open(encoding='utf-8-sig', newline='') as stream:
        return list(csv.DictReader(stream))

def number(row, key):
    try:
        value = float(row.get(key, ''))
        return value if math.isfinite(value) else None
    except (ValueError, TypeError):
        return None

def prepare():
    cases = []
    for path in sorted((ROOT / 'data').glob('2026*.csv')):
        if '_camera' in path.stem:
            continue
        raw = rows(path)
        if not raw or 'total_g' not in raw[0]:
            continue
        cams = rows(path.with_name(path.stem + '_camera.csv'))
        cams = sorted((row for row in cams if number(row, 't_s') is not None), key=lambda row: float(row['t_s']))
        ct = [float(row['t_s']) for row in cams]
        frames = []
        for row in raw:
            t = number(row, 't_s')
            if t is None:
                continue
            f = {key: number(row, key) for key in row if key not in ('time', 'flags', 'risk_text')}
            f.update(t=t, timestamp=row.get('time'), flags=row.get('flags', ''), risk_text=row.get('risk_text', ''))
            if 'cog_x_mm' not in row:
                f['cog_x_mm'] = number(row, 'cog_x_pct') * 2.5 if number(row, 'cog_x_pct') is not None else None
                f['cog_y_mm'] = number(row, 'cog_y_pct') * 3.92 if number(row, 'cog_y_pct') is not None else None
            # Both CSV streams use the same logger t_s origin; tolerate at most 250 ms.
            index = bisect.bisect_left(ct, t)
            candidates = [i for i in (index - 1, index) if 0 <= i < len(ct)]
            nearest = min(candidates, key=lambda i: abs(ct[i] - t)) if candidates else None
            cam = cams[nearest] if nearest is not None and abs(ct[nearest] - t) <= .25 else {}
            for key in ('volume_l', 'valid_pct', 'cx_m', 'cy_m', 'max_mm', 'tilt_deg'):
                f['camera_' + key] = number(cam, key)
            frames.append(f)
        frames.sort(key=lambda f: f['t'])
        if not frames:
            continue
        origin = frames[0]['t']
        for f in frames:
            f['t'] -= origin
        cases.append({'id': path.stem, 'title': TITLES.get(path.stem, path.stem),
                      'source': str(path), 'duration': frames[-1]['t'], 'frames': frames,
                      'source_cog_unit': 'mm' if 'cog_x_mm' in raw[0] else 'percent',
                      'camera_rows': len(cams), 'tension2_available': any(f.get('c2_N') is not None for f in frames)})
    videos = json.loads((ROOT / 'captures/external/metadata.json').read_text())
    notes = ['Motion path and human gait are adjustable approximations, not motion capture.',
             'External videos have no verified sensor-run pairing or clock offset.',
             'Support spacing 250 x 392 mm measured in Pi acquisition/env.sh on 2026-10-04.',
             'Cart outer dimensions, mass, friction, strap stiffness are configurable assumptions.',
             '4 x 5 kg compression cells; 2 x 10 kgf tension cells (98.0665 N each).',
             'Do not clip recorded overloads; missing sensor channels remain null.',
             'No new 2026-10-04 depth image recordings found; camera CSV summaries are available.']
    result = {'cases': cases, 'videos': videos, 'notes': notes, 'support_mm': [250, 392]}
    (ROOT / 'forest_data.json').write_text(json.dumps(result, ensure_ascii=False, allow_nan=False), encoding='utf-8')
    print(json.dumps({'runs': len(cases), 'new_runs': sum(c['id'].startswith('20261004') for c in cases),
                      'frames': sum(len(c['frames']) for c in cases), 'videos': len(videos)}, indent=2))

if __name__ == '__main__':
    prepare()
