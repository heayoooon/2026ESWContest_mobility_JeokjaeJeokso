#!/usr/bin/env python3
"""녹화 재생기 (디스플레이 파이). 센서 파이 rec.sh 로 저장한 녹화(runs/<폴더>/twin.tws)를
녹화 때와 같은 속도로 twin_server.py (TCP 5010) 에 보낸다. 서버 · 화면은 실시간과 똑같이 그린다
(띠에 '녹화 재생' + 녹화 이름 + 진행 막대).

녹화의 판정은 팀 레포 최신 pi/analysis 로 다시 계산해 둔 것이다 (PC 의 rejudge_runs.py, runs/index.json 의 algo).
화면의 '일시정지' 단추는 UDP 5011 'pause' / 'resume' 으로 온다.

usage: python3 replay_runs.py [이름 일부 ...] [--once] [--dir runs] [--port 5010]
  이름을 주면 그 글자가 들어간 녹화만 (예: 좌회전 충돌). 없으면 전부를 index.json 순서로 계속 돌린다
"""
import argparse, json, os, socket, struct, sys, threading, time

HERE = os.path.dirname(os.path.abspath(__file__))
CTRL = dict(paused=False, skip=False)


def control_listener(port=5011):
    c = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    c.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    c.bind(('127.0.0.1', port))
    while True:
        m = c.recvfrom(4096)[0].decode(errors='ignore').strip()
        if m in ('pause', 'resume'):
            CTRL['paused'] = m == 'pause'
        elif m in ('restart', 'next'):
            CTRL['skip'] = True
        print('control', m, flush=True)


def read_tws(path):
    frames = []
    with open(path, 'rb') as f:
        while True:
            h = f.read(12)
            if len(h) < 12:
                break
            wall, ln = struct.unpack('!dI', h)
            blob = f.read(ln)
            if len(blob) < ln:
                break
            frames.append((wall, blob))
    return frames


class Link:
    """twin_server 로 [4바이트 길이][본문] 을 보내는 연결 하나. 끊기면 다시 잇는다."""

    START_WAIT, GONE_WAIT = 90.0, 15.0     # 서버를 처음 기다리는 시간 / 쓰다가 서버가 사라지면 기다리는 시간 (s)

    def __init__(self, port):
        self.addr, self.conn, self.ever = ('127.0.0.1', port), None, False

    def send(self, blob):
        packet = struct.pack('!I', len(blob)) + blob
        since = time.time()
        while True:
            try:
                if self.conn is None:
                    self.conn = socket.create_connection(self.addr, timeout=3)
                    self.conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                    self.ever = True
                self.conn.sendall(packet)
                return
            except OSError:
                if self.conn is not None:
                    self.conn.close()
                self.conn = None
                # 키오스크를 닫아 서버가 꺼졌으면 재생기도 끝낸다 (실시간 송신이 나중에 연결할 수 있게)
                if time.time() - since > (self.GONE_WAIT if self.ever else self.START_WAIT):
                    sys.exit('서버(127.0.0.1:%d)가 없다 -> 재생 끝' % self.addr[1])
                time.sleep(1.0)                    # 서버가 아직 안 떴거나 다시 켜지는 중


def play(link, name, frames, seq):
    """녹화 하나를 녹화 때 간격 그대로. 세션 id 를 새로 줘서 서버가 트윈을 처음부터 다시 만들게 한다."""
    session = os.urandom(8).hex()
    t0, start = frames[0][0], time.time()
    paused_for = 0.0
    for wall, blob in frames:
        while True:                                # 녹화 시각에 맞춰 기다린다 (멈춘 동안은 시계도 멈춘다)
            if CTRL['skip']:
                CTRL['skip'] = False
                return seq
            if CTRL['paused']:
                p0 = time.time()
                while CTRL['paused'] and not CTRL['skip']:
                    time.sleep(0.05)
                paused_for += time.time() - p0
                continue
            wait = start + paused_for + (wall - t0) - time.time()
            if wait <= 0:
                break
            time.sleep(min(wait, 0.05))
        hl = struct.unpack('!I', blob[:4])[0]
        hdr = json.loads(blob[4:4 + hl])
        hdr.update(session=session, seq=seq, send_t=time.time())
        hb = json.dumps(hdr, ensure_ascii=False, separators=(',', ':')).encode()
        link.send(struct.pack('!I', len(hb)) + hb + blob[4 + hl:])
        seq += 1
    return seq


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('names', nargs='*')
    ap.add_argument('--dir', default=os.path.join(os.path.dirname(HERE), 'runs'))
    ap.add_argument('--port', type=int, default=5010)
    ap.add_argument('--once', action='store_true')
    ap.add_argument('--gap', type=float, default=1.5, help='녹화 사이 마지막 화면을 두는 시간 (s)')
    args = ap.parse_args()
    try:
        index = json.load(open(os.path.join(args.dir, 'index.json'), encoding='utf-8'))
        dirs = [r['dir'] for r in index['runs']]
        print('판정:', index.get('algo'), flush=True)
    except (OSError, ValueError, KeyError):
        dirs = sorted(d for d in os.listdir(args.dir) if os.path.isfile(os.path.join(args.dir, d, 'twin.tws')))
    if args.names:
        dirs = [d for d in dirs if any(n in d for n in args.names)]
    if not dirs:
        sys.exit('재생할 녹화가 없다: %s %s' % (args.dir, ' '.join(args.names)))
    runs = [(d, read_tws(os.path.join(args.dir, d, 'twin.tws'))) for d in dirs]
    runs = [(d, f) for d, f in runs if f]
    for d, f in runs:
        print('  %-40s %4d 프레임 %5.1f s' % (d, len(f), f[-1][0] - f[0][0]), flush=True)
    threading.Thread(target=control_listener, daemon=True).start()
    link, seq = Link(args.port), 0
    while True:
        for d, frames in runs:
            print(time.strftime('%T'), '재생', d, flush=True)
            seq = play(link, d, frames, seq)
            # 다음 녹화 전 마지막 화면을 잠깐 둔다 (5 s 넘게 비면 서버가 '연결 끊김'으로 본다. 멈춘 동안은 서버가 기다려 준다)
            time.sleep(args.gap)
            while CTRL['paused'] and not CTRL['skip']:
                time.sleep(0.1)
        if args.once:
            break


if __name__ == '__main__':
    main()
