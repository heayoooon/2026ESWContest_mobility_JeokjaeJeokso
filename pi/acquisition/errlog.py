"""오류 · 복구 기록 (디버깅용). 프로그램마다 pi/live/logs/errors_<이름>.log 하나 (1 MB x 5 개로 돌려 쓴다).

    from acquisition.errlog import errlog, Edge
    log = errlog('sensor_feed')            # 잡히지 않은 예외(스레드 포함)도 traceback 으로 남는다
    log.warning('imu1 절전 상태 -> 다시 깨움')
    edge = Edge(log)
    edge.set('load:NW', bad, 'NW 로드셀 값 없음')   # 상태가 바뀔 때만 한 줄 (나빠짐 · 회복 + 걸린 시간)

같은 문제가 계속되면 줄이 넘치지 않게 Edge 로 바뀔 때만 남긴다. 보기: tail -f pi/live/logs/errors_*.log
"""
import logging
import logging.handlers
import os
import sys
import threading
import time

LOG_DIR = os.environ.get('SCSS_ERR_DIR') or os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'live', 'logs')


def errlog(name):
    log = logging.getLogger('scss.' + name)
    if log.handlers:
        return log
    os.makedirs(LOG_DIR, exist_ok=True)
    h = logging.handlers.RotatingFileHandler(os.path.join(LOG_DIR, 'errors_%s.log' % name),
                                             maxBytes=1 << 20, backupCount=5, encoding='utf-8')
    h.setFormatter(logging.Formatter('%(asctime)s.%(msecs)03d %(levelname)-7s ' + name + '  %(message)s',
                                     '%Y-%m-%d %H:%M:%S'))
    log.addHandler(h)
    log.setLevel(logging.INFO)
    log.propagate = False

    def hook(t, v, tb):
        if not issubclass(t, KeyboardInterrupt):
            log.critical('잡히지 않은 예외로 끝남', exc_info=(t, v, tb))
        sys.__excepthook__(t, v, tb)

    def thread_hook(a):
        log.critical('스레드 %s 예외', a.thread.name if a.thread else '?', exc_info=(a.exc_type, a.exc_value, a.exc_traceback))

    sys.excepthook = hook
    threading.excepthook = thread_hook
    log.info('시작 (pid %d)', os.getpid())
    return log


class Edge:
    """상태가 바뀔 때만 남긴다: 나빠지면 WARNING, 돌아오면 INFO (얼마 동안이었는지)."""

    def __init__(self, log):
        self.log = log
        self.bad = {}

    def set(self, key, bad, text, ok_text=None):
        since = self.bad.get(key)
        if bad and since is None:
            self.bad[key] = time.monotonic()
            self.log.warning(text)
        elif not bad and since is not None:
            del self.bad[key]
            self.log.info('%s (%.1f 초 만에)', ok_text or (key + ' 회복'), time.monotonic() - since)
