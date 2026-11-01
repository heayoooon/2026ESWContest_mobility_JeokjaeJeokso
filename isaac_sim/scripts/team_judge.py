"""Run the team's own judgement (sensor Pi pi/analysis, copied by sync_team_analysis.py) inside the twin.

The twin feeds raw channels exactly as the Pi's FrameSource would - floor cells NW/NE/SW/SE (g), belt
tension c1/c2 (N), IMU (G, deg/s, deg) at ~10 Hz - into analysis.analyzer.Analyzer with the Pi's wiring
configuration (acquisition/env.sh), and turns the Result into the same columns as pi/live/sensor_feed.py
(risk, lv_*, risk_text, check_text, load_kg, cog_*, h_*, c*_min_N ...). Nothing is re-implemented here.
"""
import json
import math
import os
import sys
from pathlib import Path

TEAM = Path(__file__).resolve().parent.parent / 'team'
LEVEL_KO = ('정상', '주의', '경고', '위험')


def version():
    try:
        return json.loads((TEAM / 'TEAM_VERSION.json').read_text(encoding='utf-8'))
    except Exception:
        return None


# Twin-only settings that win over team/acquisition/env.sh (an environment variable of the same name still wins).
# TENSION_MAX: belt / tension-cell upper limit, 300 N in the twin (2026-11-01, user request; team default 80 N).
TWIN_ENV = {'TENSION_MAX': '300'}


def _load_env():
    """Pi wiring / bed geometry from team/acquisition/env.sh ('export KEY=VALUE' lines)."""
    for key, value in TWIN_ENV.items():
        os.environ.setdefault(key, value)
    for line in (TEAM / 'acquisition' / 'env.sh').read_text(encoding='utf-8').splitlines():
        line = line.strip()
        if line.startswith('export ') and '=' in line:
            key, value = line[7:].split('=', 1)
            os.environ.setdefault(key.strip(), value.strip().strip('"\''))


class TeamJudge:
    """One Analyzer per replay / sandbox session. step() takes one ~10 Hz sample."""

    def __init__(self):
        info = version()
        if info is None:
            raise FileNotFoundError('team/ is missing - run scripts/sync_team_analysis.py')
        _load_env()
        if str(TEAM) not in sys.path:
            sys.path.insert(0, str(TEAM))
        from acquisition.config import Config
        from acquisition.frame import Frame, ImuReading
        from analysis.analyzer import Analyzer
        from analysis.overall import overall
        from analysis.risk import name as risk_name
        from analysis import total
        self.Frame, self.ImuReading, self.overall, self.risk_name, self.total = Frame, ImuReading, overall, risk_name, total
        self.Analyzer = Analyzer
        self.cfg = Config()
        self.weight, self.tension, _ = self.cfg.roles()
        self.algo = f"team {info['branch']}@{info['commit'][:7]}"
        self.reset()

    def reset(self):
        self.analyzer = self.Analyzer(self.cfg)
        self.seq = 0

    def rebase(self):
        self.analyzer.rebase()

    def step(self, t_s, sample):
        """sample: dict with the Pi's column names (NW_g ..., c1_N, c2_N, imu1_ax_G ..., flags)."""
        num = lambda key: (float(sample[key]) if isinstance(sample.get(key), (int, float)) and not math.isnan(sample[key])
                           else None)
        imu = {}
        for label in self.cfg.imu_labels:
            ax = num(f'{label}_ax_G')
            imu[label] = None if ax is None else self.ImuReading(
                ax=ax, ay=num(f'{label}_ay_G') or 0., az=num(f'{label}_az_G') or 1., gx=num(f'{label}_gx_dps') or 0.,
                gy=num(f'{label}_gy_dps') or 0., gz=num(f'{label}_gz_dps') or 0., roll=num(f'{label}_roll_deg') or 0.,
                pitch=num(f'{label}_pitch_deg') or 0., ax_peak=ax, ay_peak=num(f'{label}_ay_G') or 0., n=1)
        flags = sample.get('flags')
        frame = self.Frame(seq=self.seq, t_ns=int(t_s * 1e9), t_s=float(t_s), wall=float(sample.get('wall') or 0.),
                           load_g={k: num(k + '_g') for k in self.weight}, tension_n={k: num(k + '_N') for k in self.tension},
                           raw={}, imu=imu, flags=[f for f in str(flags).split('|') if f] if flags else [])
        self.seq += 1
        res = self.analyzer(frame)
        return self.columns(res)

    def columns(self, res):
        """Result -> the sensor_feed.py columns (same names the display Pi reads)."""
        r1 = lambda v, d=1: None if v is None else round(float(v), d)
        tr = res.transfer
        lv = {ch.key: ch.level for ch in res.checks}
        s = dict(algo=self.algo, risk=res.level, risk_overall=self.overall(res.checks)[1],
                 lv_load=lv.get('load'), lv_cog=lv.get('cog'), lv_move=lv.get('move'), lv_c1=lv.get('c1'), lv_c2=lv.get('c2'),
                 risk_text=' | '.join('%s %s: %s' % (self.risk_name(ch.level), ch.name, ch.text) for ch in res.checks if ch.level),
                 check_text={ch.key: ch.text for ch in res.checks},
                 load_kg=r1(res.load_kg, 3), measured_kg=r1(res.measured_kg, 3), rated_kg=self.total.RATED_KG,
                 load_pct=[self.total.CAUTION_PCT, self.total.WARNING_PCT, self.total.DANGER_PCT],
                 cog_x_mm=r1(res.cog.x) if res.cog else None, cog_y_mm=r1(res.cog.y) if res.cog else None,
                 cog_pred_x_mm=r1(tr.pred_x) if tr else None, cog_pred_y_mm=r1(tr.pred_y) if tr else None,
                 cog_static_x_mm=r1(tr.static_x) if tr else None, cog_static_y_mm=r1(tr.static_y) if tr else None,
                 h_x_mm=r1(abs(tr.kx)) if tr and tr.kx is not None else None,
                 h_y_mm=r1(abs(tr.ky)) if tr and tr.ky is not None else None,
                 c1_min_N=r1(res.tension_min.get('c1'), 2), c2_min_N=r1(res.tension_min.get('c2'), 2),
                 tension_max_N=self.cfg.tension_max)
        if s['cog_x_mm'] is not None and self.cfg.bed_l:
            s['cog_x_pct'] = round(s['cog_x_mm'] / self.cfg.bed_l * 100, 1)
            s['cog_y_pct'] = round(s['cog_y_mm'] / self.cfg.bed_w * 100, 1)
        return s


def judge_recording(frames, judge=None):
    """Re-judge a recorded run frame by frame with the team code; returns one column dict per frame."""
    judge = judge or TeamJudge()
    judge.reset()
    return [judge.step(float(f.get('t_s', f.get('t', 0.)) or 0.), f) for f in frames]
