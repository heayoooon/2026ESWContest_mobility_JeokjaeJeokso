# D435i 동작 확인 — 뎁스카메라 테스트

Intel RealSense D435i가 제대로 동작하는지 확인하는 스크립트 모음.
라즈베리파이 5(display 노드)에서 실행하는 것을 기준으로 정리했다.

## 검증 완료 환경

| 항목 | 값 |
|---|---|
| 보드 | Raspberry Pi 5 Model B Rev 1.1 (4코어 / 8GB) |
| OS | Debian 13 (trixie), aarch64, labwc Wayland 세션 |
| 디스플레이 | DSI 800x480 (공식 7" 터치스크린) |
| 카메라 | RealSense D435I, 시리얼 261822073176, 펌웨어 5.15.1.55 |
| librealsense | v2.58.4 소스 빌드 (RSUSB 백엔드) |

## 왜 소스 빌드인가

둘 다 막혀서 **소스 빌드 외에 방법이 없다.**

- **apt에 패키지 없음** — Intel은 Ubuntu용 deb만 배포한다. Debian 13 repo에
  `librealsense2-*`가 없다.
- **pip 휠 없음** — `pyrealsense2`는 aarch64 리눅스 휠을 제공하지 않는다.
  (Windows/x86_64만 있다. 참고로 Windows에서도 **Python 3.14용 휠은 없으니**
  3.10~3.12 환경을 써야 한다.)

Pi 5에서 `make -j4` 기준 **약 15~20분** 걸린다.

## 설치

```bash
# 1) 소스 받기 (일반 계정)
git clone --depth 1 -b v2.58.4 https://github.com/IntelRealSense/librealsense.git ~/librealsense

# 2) 의존성 + udev 권한 (sudo 필요, 1회성)
sudo bash rs_deps.sh

# 3) 빌드 설정
mkdir -p ~/librealsense/build && cd ~/librealsense/build
cmake .. \
  -DCMAKE_BUILD_TYPE=Release \
  -DFORCE_RSUSB_BACKEND=ON \
  -DBUILD_EXAMPLES=ON \
  -DBUILD_GRAPHICAL_EXAMPLES=ON \
  -DBUILD_PYTHON_BINDINGS=ON \
  -DPYTHON_EXECUTABLE=/usr/bin/python3 \
  -DBUILD_UNIT_TESTS=OFF \
  -DCHECK_FOR_UPDATES=OFF

# 4) 빌드
nohup make -j4 > ~/rs_build.log 2>&1 &
```

`-DFORCE_RSUSB_BACKEND=ON`이 중요하다. Pi 커널에는 RealSense용 UVC 패치가
없어서 기본 V4L2 백엔드로는 depth 스트림이 제대로 안 잡힌다. libuvc 기반
RSUSB 백엔드를 쓰면 커널 패치 없이 동작하고, 대신 `99-realsense-libusb.rules`
udev 규칙이 있어야 root가 아닌 계정에서 장치를 열 수 있다.

`sudo make install`은 하지 않아도 된다. `rs_viewer.sh` / `rs_py.sh`가 빌드
트리를 직접 가리킨다.

## 파일

| 파일 | 용도 |
|---|---|
| `rs_deps.sh` | 빌드 의존성 apt 설치 + udev 권한 규칙. **sudo 필요, 1회만** |
| `rs_viewer.sh` | `realsense-viewer` GUI를 DSI 화면에 띄운다 |
| `rs_py.sh` | `sudo make install` 없이 빌드 트리의 pyrealsense2를 쓰는 실행 래퍼 |
| `test_d435i.py` | 종합 진단 — 장치 정보, fps, 유효 depth 비율, 거리, IMU |
| `ir_probe.py` | IR 원본·depth를 PNG로 저장. depth가 안 나올 때 원인 판별용 |
| `samples/` | 정상/비정상 IR·depth 영상 비교 샘플 |

## 실행

**GUI 뷰어** (DSI 화면에 뜬다. SSH로 실행해도 로컬 모니터로 간다)

```bash
./rs_viewer.sh
```

뜨고 나면 좌측 패널에서 `Stereo Module` / `RGB Camera` / `Motion Module`
토글을 켜야 영상이 나온다. 기본값은 전부 off다.

**숫자 진단** (화면 없이, SSH만으로)

```bash
./rs_py.sh test_d435i.py --headless --seconds 10
```

**IR 원본 확인** (depth가 안 나올 때)

```bash
./rs_py.sh ir_probe.py --out /tmp
```

**파이썬 라이브 뷰** (OpenCV 필요: `sudo apt install python3-opencv`)

```bash
./rs_py.sh test_d435i.py --max-width 800 --fullscreen --seconds 600
```

800x480에서는 realsense-viewer의 좌측 패널이 화면 절반을 먹는다.
컬러+depth만 크게 보려면 이쪽이 낫다.

## 정상 기준값

`test_d435i.py --headless`를 1~3m 거리의 평범한 실내에서 돌렸을 때:

| 항목 | 정상 | 이상하면 |
|---|---|---|
| USB 타입 | `3.2` | `2.1`이면 케이블/포트 문제. 해상도·fps가 깎인다 |
| 센서 목록 | Stereo + RGB + **Motion** | Motion이 없으면 D435i가 아니라 D435 |
| 유효 depth 픽셀 | **70% 이상** | 아래 "문제 판별" 참고 |
| fps (640x480) | 24~30 | align 연산 때문에 Pi에서 30 미만은 정상 |
| accel 크기 | 9.8 근처 | 벗어나면 IMU 이상 |
| gyro | 정지 시 0 근처 | |

실측 기준선 (필름 제거 후, 책상~화이트보드 촬영):
유효 depth **73.6%**, 거리 0.18~4.69m, fps 24.9, accel 9.76 m/s².

## 문제 판별

**depth 유효 픽셀이 낮을 때** `ir_probe.py`로 IR 원본을 보면 원인이 갈린다.
`samples/` 폴더에 실제 비교 영상이 있다.

| IR 영상 상태 | 원인 |
|---|---|
| 뿌옇고 초점 안 맞음, 표준편차 20 이하 | **렌즈에 뭔가 덮여 있다** |
| 전부 새까맣거나 포화(255) | 렌즈 완전 가림 / 직사광 |
| 선명한데 depth만 없음 | 검은 광택면, 유리, 최소거리(0.28m) 이내 |

### 실제로 겪은 것들

**1. 보호필름** — D435i는 전면 유리에 투명 보호필름이 붙어 출고된다.
눈에 잘 안 보이는데 이게 붙어 있으면 IR 상이 완전히 뭉개진다.
유효 depth 픽셀이 **2.3% → 74.7%**로 바뀌었다. `samples/ir_bad_film_on.png`와
`samples/ir_good_film_off.png`를 비교해보면 차이가 명확하다.

카메라가 이상하다 싶으면 **이것부터 확인할 것.**

**2. 검은 광택면** — 노트북 LCD 화면처럼 검고 광택 있는 면은 IR을 흡수해서
depth가 안 잡힌다. 고장이 아니라 물리적 한계다.
**화물 인식할 때 검은 비닐·광택 포장재에서 같은 현상이 나온다.**

**3. 최소 측정 거리 0.28m** — 이보다 가까우면 전부 0으로 나온다.
값이 0.26~0.29m 부근에만 드문드문 찍히면 피사체가 최소거리 경계에 있는 것이다.

**4. USB 연결** — depth가 갑자기 안 나오면 물리적으로 빠졌는지부터 본다.

```bash
lsusb | grep 8086                                   # 열거 확인
journalctl --since "10 min ago" | grep -i "usb disconnect"
```

## GUI 실행 시 주의 — Wayland 대신 X11

`rs_viewer.sh`는 `WAYLAND_DISPLAY`를 **존재하지 않는 소켓명으로 설정**해서
GLFW가 X11(Xwayland) 백엔드를 쓰도록 강제한다.

`unset WAYLAND_DISPLAY`만으로는 안 된다. libwayland의 `wl_display_connect(NULL)`이
`XDG_RUNTIME_DIR`의 기본 소켓명 `wayland-0`으로 폴백해서 결국 Wayland에 붙는다.
그렇게 붙으면 GLFW가 창 위치·커서를 제어하지 못해 이런 에러를 초당 200줄씩 뱉는다:

```
GLFW Driver Error: Wayland: The platform does not provide the window position
```

동작은 하지만 로그가 시간당 50MB씩 불어난다. X11 강제 후 **1690줄 → 1줄**로
떨어졌다.

## 알려진 경고

뷰어 실행 시 libudev 관련 붉은 경고가 뜬다. 핫플러그 자동 감지가 안 되므로
**카메라를 다시 꽂았으면 뷰어를 재시작**해야 인식한다.
