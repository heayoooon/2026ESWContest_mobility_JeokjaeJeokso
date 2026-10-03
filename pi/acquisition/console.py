"""터미널 출력. SSH 가 끊겨 터미널이 사라져도 기록은 계속되게, 출력 오류는 삼킨다."""
import shutil
import sys
import unicodedata


def say(*args):
    try:
        print(*args, flush=True)
    except OSError:
        pass


def text_width(s):
    """터미널/엑셀에서 차지하는 칸 수. 한글은 2칸."""
    return sum(2 if unicodedata.east_asian_width(ch) in "WF" else 1 for ch in s)


def status(text):
    """한 줄 상태표시. 터미널 폭을 넘기면 줄이 밀려 \\r 이 안 먹으니 잘라서 쓴다."""
    cols = shutil.get_terminal_size((120, 20)).columns - 1
    out, w = "", 0
    for ch in text:
        cw = text_width(ch)
        if w + cw > cols:
            break
        out += ch
        w += cw
    try:
        sys.stdout.write("\r" + out + " " * (cols - w))
        sys.stdout.flush()
    except OSError:
        pass
