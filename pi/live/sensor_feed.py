#!/usr/bin/env python3
"""SCSS 실시간 센서 공급기 (센서 파이, 시스템 python3).

pi/acquisition 으로 센서(HX711 로드셀 4 · 장력 2, MPU-6050 IMU)를 약 10 Hz 로 읽고,
pi/analysis 위험 판정(4단계: 정상 · 주의 · 경고 · 위험)을 돌린 뒤,
결과를 같은 파이의 live_twin.py 로 UDP JSON 한 줄씩 넘긴다 (127.0.0.1:5020).
같은 값을 멀티캐스트 239.255.42.21:5021 로도 내보낸다 -> 다른 프로그램 · 팀원이 동시에 몇 개든 받아 볼 수 있다
(HX711 은 한 프로그램만 GPIO 를 잡을 수 있으므로, 센서는 여기서만 읽고 값을 나눠 준다. watch_sensors.py 참고).
기록: logs/<시각>_live.csv (센서 · 판정 열, pipeline.py CSV 와 같은 이름)

pipeline.py 와 HX711 을 같이 쓸 수 없다 -> 실시간 트윈을 돌리는 동안은 pipeline.py 를 끈다.

실측 구간 저장: UDP 127.0.0.1:5022 로 'start <폴더>' / 'stop' / 'status' (rec.sh 가 보낸다).
  start 이면 <폴더>/sensors.csv 에 그때부터 stop 까지의 행을 쓰고, stop 때 <폴더>/summary.json 을 남긴다.

usage: python3 sensor_feed.py [--port 5020]   (보통 run_live.sh 가 acquisition/env.sh 를 읽고 띄운다)
"""
import argparse, csv, json, os, socket, sys, threading, time

HERE = os.path.dirname(os.path.abspath(__file__))
PI = os.environ.get('SCSS_PI_DIR', os.path.dirname(HERE))          # 레포의 pi/
sys.path.insert(0, PI)

from acquisition.config import Config                     # noqa: E402  배선 · 보정 · 판정 기준 (env.sh)
from acquisition.source import FrameSource                # noqa: E402
from analysis.analyzer import Analyzer                    # noqa: E402  4단계 판정
from analysis.overall import overall                      # noqa: E402
from analysis.risk import name as risk_name               # noqa: E402
from analysis import total                                # noqa: E402

MCAST = ('239.255.42.21', 5021)   # 누구나 받아 보는 곳 (watch_sensors.py)
r1 = lambda v, d=1: None if v is None else round(float(v), d)  # noqa: E731


JUDGE = ['algo', 'risk', 'risk_overall', 'lv_load', 'lv_cog', 'lv_move', 'lv_c1', 'lv_c2', 'risk_text', 'load_kg', 'measured_kg',
         'rated_kg', 'cog_x_mm', 'cog_y_mm', 'cog_pred_x_mm', 'cog_pred_y_mm', 'cog_static_x_mm', 'cog_static_y_mm',
         'h_x_mm', 'h_y_mm', 'c1_min_N', 'c2_min_N', 'tension_max_N', 'cog_x_pct', 'cog_y_pct',
         'c1_k', 'c1_px_mm', 'c1_py_mm', 'c2_k', 'c2_px_mm', 'c2_py_mm']     # 벨트 누름 계수 · 자리 추정 (belt_fit.py)
IMU_F = ['ax_G', 'ay_G', 'az_G', 'gx_dps', 'gy_dps', 'gz_dps', 'roll_deg', 'pitch_deg']


def fields(cfg):
    """CSV 열 (고정): 처음 행에 없던 센서(IMU 등)가 나중에 들어와도 빠지지 않게."""
    weight, tension, _ = cfg.roles()
    return (['t_s', 'wall'] + [l + '_g' for l in weight] + ['total_g'] + [l + '_N' for l in tension]
            + ['%s_%s' % (lab, k) for lab in cfg.imu_labels for k in IMU_F] + JUDGE + ['flags'])


def row(f, res, cfg):
    """프레임 + 판정 -> 디스플레이가 읽는 sensors 딕셔너리 (pipeline.py CSV 열 이름)."""
    s = {'t_s': round(f.t_s, 3), 'wall': round(f.wall, 3), 'flags': '|'.join(f.flags)}
    for k, g in f.load_g.items():
        s[k + '_g'] = r1(g)
    s['total_g'] = r1(res.total_g)
    for k, n in f.tension_n.items():
        s[k + '_N'] = r1(n, 2)
    for lab, m in f.imu.items():
        if m is None:
            continue
        s.update({f'{lab}_ax_G': r1(m.ax, 4), f'{lab}_ay_G': r1(m.ay, 4), f'{lab}_az_G': r1(m.az, 4),
                  f'{lab}_gx_dps': r1(m.gx, 2), f'{lab}_gy_dps': r1(m.gy, 2), f'{lab}_gz_dps': r1(m.gz, 2),
                  f'{lab}_roll_deg': r1(m.roll, 2), f'{lab}_pitch_deg': r1(m.pitch, 2)})
    tr = res.transfer
    lv = {ch.key: ch.level for ch in res.checks}
    s.update(
        algo='2026-10-08', risk=res.level, risk_overall=overall(res.checks)[1],
        lv_load=lv.get('load'), lv_cog=lv.get('cog'), lv_move=lv.get('move'), lv_c1=lv.get('c1'), lv_c2=lv.get('c2'),
        risk_text=' | '.join('%s %s: %s' % (risk_name(ch.level), ch.name, ch.text) for ch in res.checks if ch.level),
        check_text={ch.key: ch.text for ch in res.checks},
        load_kg=r1(res.load_kg, 3), measured_kg=r1(res.measured_kg, 3), rated_kg=total.RATED_KG,
        load_pct=[total.CAUTION_PCT, total.WARNING_PCT, total.DANGER_PCT],
        cog_x_mm=r1(res.cog.x) if res.cog else None, cog_y_mm=r1(res.cog.y) if res.cog else None,
        cog_pred_x_mm=r1(tr.pred_x) if tr else None, cog_pred_y_mm=r1(tr.pred_y) if tr else None,
        cog_static_x_mm=r1(tr.static_x) if tr else None, cog_static_y_mm=r1(tr.static_y) if tr else None,
        h_x_mm=r1(abs(tr.kx)) if tr and tr.kx is not None else None, h_y_mm=r1(abs(tr.ky)) if tr and tr.ky is not None else None,
        c1_min_N=r1(res.tension_min.get('c1'), 2), c2_min_N=r1(res.tension_min.get('c2'), 2),
        tension_max_N=cfg.tension_max)
    for k, b in res.belts.items():                       # 벨트 누름 계수 · 자리 추정. 위치를 모르는 벨트는 k 만
        s.update({k + '_k': r1(b.k, 3), k + '_px_mm': r1(b.x), k + '_py_mm': r1(b.y)})
    if s.get('cog_x_mm') is not None:                    # 옛 화면 호환: % 무게중심
        s['cog_x_pct'] = round(s['cog_x_mm'] / cfg.bed_l * 100, 1)
        s['cog_y_pct'] = round(s['cog_y_mm'] / cfg.bed_w * 100, 1)
    return s


FIELDS = []
REC = dict(dir=None, fh=None, w=None, n=0, t0=None, wall0=None, want=None, lock=threading.Lock(), last=None)


def control_listener(port=5022):
    """rec.sh 가 보내는 'start <폴더>' / 'stop' / 'status'. 답은 JSON 한 줄."""
    c = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    c.bind(('127.0.0.1', port))
    while True:
        data, addr = c.recvfrom(4096)
        cmd, _, arg = data.decode(errors='ignore').strip().partition(' ')
        with REC['lock']:
            if cmd == 'start' and arg:
                REC['want'] = ('start', arg)
            elif cmd == 'stop':
                REC['want'] = ('stop', None)
        if cmd in ('start', 'stop'):
            time.sleep(0.6)                                   # 다음 프레임에서 바뀐다
        with REC['lock']:
            st = dict(recording=REC['dir'] is not None, dir=REC['dir'], rows=REC['n'],
                      seconds=round(time.time() - REC['wall0'], 1) if REC['wall0'] else None, last=REC['last'])
        c.sendto(json.dumps(st, ensure_ascii=False).encode(), addr)


def rec_apply(flat):
    """프레임마다: 시작 · 끝 요청을 반영하고, 기록 중이면 한 행 쓴다."""
    with REC['lock']:
        want, REC['want'] = REC['want'], None
        if want and want[0] == 'start':
            if REC['fh']:
                rec_close()
            os.makedirs(want[1], exist_ok=True)
            REC.update(dir=want[1], fh=open(os.path.join(want[1], 'sensors.csv'), 'w', newline='', encoding='utf-8-sig'),
                       w=None, n=0, t0=flat.get('t_s'), wall0=time.time())
            print('기록 시작 ->', want[1], flush=True)
        elif want and want[0] == 'stop' and REC['fh']:
            rec_close()
        if REC['fh']:
            if REC['w'] is None:
                REC['w'] = csv.DictWriter(REC['fh'], fieldnames=FIELDS, extrasaction='ignore')
                REC['w'].writeheader()
            REC['w'].writerow(flat)
            REC['n'] += 1
            if REC['n'] % 10 == 0:
                REC['fh'].flush()


def rec_close():
    REC['fh'].close()
    summ = dict(dir=REC['dir'], rows=REC['n'], seconds=round(time.time() - REC['wall0'], 2),
                started=time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(REC['wall0'])),
                ended=time.strftime('%Y-%m-%d %H:%M:%S'))
    with open(os.path.join(REC['dir'], 'summary.json'), 'w', encoding='utf-8') as f:
        json.dump(summ, f, ensure_ascii=False, indent=1)
    print('기록 끝 -> %s (%d 행, %.1f 초)' % (REC['dir'], REC['n'], summ['seconds']), flush=True)
    REC.update(dir=None, fh=None, w=None, wall0=None, last=summ)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--port', type=int, default=5020)
    args = ap.parse_args()
    global FIELDS
    cfg = Config()
    FIELDS = fields(cfg)
    print(cfg.describe(), flush=True)
    analyze = Analyzer(cfg)
    src = FrameSource(cfg)
    out = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    out.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, 1)       # 이 파이 + LAN 까지만
    out.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_LOOP, 1)      # 같은 파이 안에서도 받게
    os.makedirs(os.path.join(HERE, 'logs'), exist_ok=True)
    path = os.path.join(HERE, 'logs', time.strftime('%Y%m%d_%H%M%S') + '_live.csv')
    fh = open(path, 'w', newline='', encoding='utf-8-sig')
    w = None
    last_flush = last_print = 0.0
    n = 0
    print('센서 -> 127.0.0.1:%d, 기록 %s' % (args.port, path), flush=True)
    threading.Thread(target=control_listener, daemon=True).start()
    try:
        for f in src.frames():
            res = analyze(f)
            s = row(f, res, cfg)
            msg = json.dumps(s, ensure_ascii=False, separators=(',', ':')).encode()
            out.sendto(msg, ('127.0.0.1', args.port))
            try:
                out.sendto(msg, MCAST)
            except OSError:
                pass
            flat = {k: v for k, v in s.items() if not isinstance(v, (dict, list))}
            if w is None:
                w = csv.DictWriter(fh, fieldnames=FIELDS, extrasaction='ignore')
                w.writeheader()
            w.writerow(flat)
            rec_apply(flat)
            n += 1
            now = time.monotonic()
            if now - last_flush > 1.0:
                fh.flush()
                last_flush = now
            if now - last_print > 2.0:
                last_print = now
                print('%6.1fs %5d행  적재 %s kg  c1 %s N  c2 %s N  종합 %s  %s' % (
                    f.t_s, n, s.get('load_kg'), s.get('c1_N'), s.get('c2_N'), risk_name(s.get('risk')),
                    s.get('risk_overall') or ''), flush=True)
    except KeyboardInterrupt:
        pass
    finally:
        src.close()
        fh.close()
        with REC['lock']:
            if REC['fh']:
                rec_close()
        print('끝: %d 행 -> %s' % (n, path), flush=True)


if __name__ == '__main__':
    main()
