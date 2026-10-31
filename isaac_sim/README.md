# isaac_sim – Isaac Sim 디지털 트윈

적재적소 화물 센서 장치를 **Isaac Sim 4.2** 안에 옮긴 디지털 트윈입니다. 숲길 순환로를 달리는 현대 엑시언트급 화물 트럭 위에
실제 장치(로드셀 4개 · 벨트 장력 2개 · IMU · D435i 깊이 카메라)를 올려 두고, 판정은 이 저장소의 센서 파이 코드
(`pi/analysis`)를 그대로 돌립니다.

- **측정 재생**: 센서 파이 녹화(`~/scss_live/runs/…`)를 그대로 재생 – 셀 · 벨트 · IMU 값, 화물 모양, 트럭 움직임(기록된 IMU)
- **물리 실험**: PhysX 로 조건(무게 · 마찰 · 벨트 조임 · 경사 · 회전 · 화물 밀기)을 바꿔 보고 센서값 · 판정을 계산
- **실시간 받기**: 센서 파이가 디스플레이 파이로 보내는 프레임을 이 PC 로도 받아(포트 5010) 지금 적재함 모습 · 센서 · 판정을 표시
- **화물을 상자로**: 디스플레이 파이와 같은 코드(`display/twin/server` 의 `cargo_model` · `stable_twin` · `stack_layers`)로 화물을 상자로
- **원격**: 브라우저 스트리밍(WebRTC, 1명) · 원격 조종 API(`/twin/…`) · 녹화 내려받기(`/recordings/`) · Claude 용 MCP
- **디스플레이로 보내기**: 트윈 상태를 디스플레이 파이 트윈 화면으로 보냄

조종 방법은 [CONTROLS_KO.md](CONTROLS_KO.md), 바뀐 내역과 설계 이유는 [FOREST_TWIN.md](FOREST_TWIN.md) 에 있습니다.

## 설치 (Windows, NVIDIA RTX GPU)
1. Isaac Sim 4.2 pip 환경: `conda create -n env_isaacsim42 python=3.10` 후
   `pip install isaacsim==4.2.0.2 isaacsim-extscache-physics==4.2.0.2 isaacsim-extscache-kit==4.2.0.2 isaacsim-extscache-kit-sdk==4.2.0.2 --extra-index-url https://pypi.nvidia.com`
   그리고 `pip install opencv-python pillow`.
2. Kit 설정 설치 (한 번): `python install_kit.py` (그 환경의 python 으로). 한국어 패널 글꼴 경로를 이 폴더로 채워 넣습니다.
3. 팀 코드 가져오기: `python scripts/sync_team_analysis.py --part all` → `team/` (판정 코드) · `team_display/` (상자 코드).
   이 저장소 안에서는 `team_display/` 가 없으면 옆의 `display/twin/server` 를 바로 씁니다.
4. NVIDIA 무료 자산(나무 · 풀 · 하늘 · 사람, 약 600 MB): `python scripts/fetch_nvidia_assets.py` → `assets/nvidia/`.
5. 실행: `Start-ForestTwin.bat` (창) 또는 `Start-ForestTwin-Share.bat` (브라우저 공유 + 원격 API).
   다른 경로의 python 이면 `set ISAAC_PY=…\python.exe` 후 실행.

## 필요한 큰 파일 (저장소에 넣지 않음)
저장소를 가볍게 두려고(센서 파이도 이 저장소를 받습니다) 아래는 올리지 않았습니다. 트윈을 돌리는 PC 에서 만들거나 받습니다.

| 파일 | 크기 | 만드는 법 |
|---|---|---|
| `data/live_runs/<녹화>/` | 녹화마다 수 MB | 센서 파이 `~/scss_live/runs/` 에서 복사 (`rec.sh start/stop` 으로 녹화) |
| `live_data.json`, `data/live_cache/*.npz` | 약 22 MB | `python scripts/prepare_live_runs.py` (위 녹화를 변환) |
| `assets/cargo.usd` | 약 20 MB | 화물 3D 스캔 (`scripts/twn_to_usd.py` / `convert_glb_to_usd.py`) – 물리 실험의 화물 모양 |
| `assets/nvidia/` | 약 600 MB | `scripts/fetch_nvidia_assets.py` |

## 폴더
| 경로 | 내용 |
|---|---|
| `scripts/run_forest_twin.py` | 트윈 본체 (장면 · 모드 · 패널 · 판정 · 기록 · API) |
| `scripts/forest_scene.py`, `truck*.py`, `road_path.py`, `materials_pbr.py` | 숲 · 도로 · 트럭 · 재질 |
| `scripts/measured_surface.py`, `cargo_boxes.py` | 측정 화물 표면 / 디스플레이 방식 상자 |
| `scripts/live_link.py` | 실시간 받기 (센서 파이 → TCP 5010) |
| `scripts/display_link.py` | 디스플레이 파이로 보내기 |
| `scripts/team_judge.py`, `cargo_shift.py`, `sync_team_analysis.py` | 팀 판정 코드 연결 |
| `scripts/twin_api.py`, `recordings_web.py`, `isaac_twin_mcp.py` | 원격 API · 녹화 내려받기 · MCP |
| `isaac_twin_light.kit`, `install_kit.py` | Kit 앱 설정 (가벼운 창 · 스트리밍 · 한국어 글꼴) |
| `assets/fonts`, `assets/branding`, `assets/truck` | 한국어 글자 목록 · 화면 로고 · 트럭 글자/표지판 |

## 실시간 받기를 쓰려면 (센서 파이)
센서 파이 송신기가 같은 프레임을 이 PC 로도 보내야 합니다 (`pi/live/live_twin.py --mirror`, 따로 도는 송신 스레드라 트윈이 꺼져 있어도
디스플레이는 느려지지 않습니다):

```bash
~/scss_live/run_live.sh <디스플레이 파이 주소> <트윈 PC 주소>:5010
```
