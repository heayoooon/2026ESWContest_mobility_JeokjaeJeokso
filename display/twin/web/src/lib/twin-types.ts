/**
 * 백엔드(디스플레이 파이 ~/twin_web/twin_server.py) <-> 화면(React) 데이터 계약.
 *
 * 백엔드는 GET /api/stream 으로 Server-Sent Events 를 약 10 Hz 로 보낸다. 각 메시지 = `data: <TwinState JSON>\n\n`.
 * GET /api/state 는 같은 JSON 한 번. 아래 필드 이름 · 단위를 바꾸면 양쪽을 같이 바꾼다.
 *
 * 좌표계 (월드, 미터): 카메라 설치 보정 기준. x = 오른쪽, y = 앞(카메라에서 멀어지는 쪽), z = 위.
 * z 는 방바닥 = 0. 화물 받침판(적재면) 높이 = base. 트럭 앞 방향과 로드셀 위치는 카메라와 정합된 적이 없다 (도식으로만 그린다).
 */

export type TwinBox = {
  /** 추적 번호 (같은 블록이면 프레임이 바뀌어도 같다). 없을 수 있다 */
  id: number | null
  shape: 'box' | 'cylinder'
  /** 바닥 사각형 (m) */
  x0: number
  x1: number
  y0: number
  y1: number
  /** 아래 · 위 높이 (m, 방바닥 기준) */
  bottom: number
  top: number
  /** 카메라에서 본 대표색 0..1 */
  color: [number, number, number]
  /** 잠깐 안 보여서 직전 모양을 유지 중 */
  held: boolean
  /** 0.5 s 넘게 지속된 형상 변화가 이 블록에 확정됨 (빨갛게 강조) */
  alert: boolean
  /** 'box' = 쌓인 상자 (앞면 깊이로 층을 나눔), 'side' = 상자 옆에 따로 놓인 짐 (뽁뽁이로 싼 물병 등),
   *  'item' = 판이나 상자 위의 작은 물건. 옛 서버는 없음 */
  kind?: 'box' | 'side' | 'item'
}

export type TwinAlert = {
  kind: 'accel' | 'turn' | 'load' | 'cog' | 'shape' | 'cam' | string
  text: string
  /** true = 경고(빨강), false = 주의(주황) */
  severe: boolean
}

export type TwinSensors = {
  /** 총하중 (g) */
  total_g: number | null
  /** 네 모서리 로드셀 (g). NW=앞왼 NE=앞오 SW=뒤왼 SE=뒤오 */
  loads: { NW: number | null; NE: number | null; SW: number | null; SE: number | null }
  /** 로드셀 무게중심 (%, 50/50 = 가운데). x = 좌→우, y = 앞→뒤 */
  cog: { x_pct: number; y_pct: number } | null
  /** 벨트 장력 (N) */
  tension_n: number | null
  tension_stuck: boolean
  /** MPU-6050 (deg, deg/s, G) */
  roll: number | null
  pitch: number | null
  yaw_rate: number | null
  ax: number | null
  ay: number | null
  /** 센서 값이 카메라 프레임보다 얼마나 오래됐나 (s) */
  age_s: number | null
  /** 2026-10-04 부터 (센서 파이 analysis/): 적재 kg 와 최대 적재 kg */
  load_kg?: number | null
  max_kg?: number
  /** 로드셀 간격 (mm): l = 앞왼-앞오(좌우), w = 앞왼-뒤왼(앞뒤). 무게중심 mm 의 기준 */
  bed_mm?: { l: number; w: number }
  /** 무게중심 (mm, 앞왼 셀 원점: x 좌->우, y 앞->뒤) */
  cog_mm?: { x: number | null; y: number | null } | null
  /** 가감속으로 예측한 무게중심 (mm). 실측과 차이가 크면 화물 이동 */
  cog_pred_mm?: { x: number | null; y: number | null } | null
  /** 벨트 장력. c1 = 앞 벨트(앞쪽 두 셀 무게를 잡는다), c2 = 뒤 벨트. min_n = 그 무게로 필요한 최소 장력 */
  belts?: TwinBelt[]
  /** 2026-10-08 판정: 적정 적재중량 (kg) 과 주의 · 경고 · 위험 기준 (%). load_kg = 벨트 누름을 뺀 화물 무게 */
  rated_kg?: number
  load_pct?: number[]
  /** 셀 합 그대로의 무게 (벨트 누름 포함, kg) */
  measured_kg?: number | null
  /** 벨트 장력 상한 (N). 이 50 % 를 넘으면 과조임 */
  tension_max_n?: number
}

export type TwinBelt = { key: 'c1' | 'c2' | string; name: string; n: number | null; min_n: number | null; level: RiskLevel | null }

/** 0 = 정상, 1 = 주의, 2 = 경고, 3 = 위험 (센서 파이 analysis/risk.py, 2026-10-08 부터 4단계).
 *  옛 3단계 녹화(정상 · 주의 · 위험)는 서버가 위험을 3 으로 바꿔 보낸다 */
export type RiskLevel = 0 | 1 | 2 | 3

/** 센서 파이 파이프라인의 위험 판정 그대로. 판정이 없는 옛 녹화면 TwinState.risk = null */
export type TwinRisk = {
  /** 종합 위험도 = 적재 · 무게중심 · 장력 단계의 안전 여유 곱 (팀 overall.py). 화물 이동은 안 들어간다 */
  level: RiskLevel | null
  /** 종합 이유 (예: '경고 적재 · 경고 장력 S(c2) (겹쳐서 한 단계 올림)') */
  overall?: string
  /** 원문 (예: '위험 적재: 과적 16.4kg > 최대 15kg | 주의 장력 c1: 부족 ...') */
  text: string
  /** load 적재 · cog 무게중심 · move 화물 이동 · c1 앞 벨트 · c2 뒤 벨트 */
  checks: { key: 'load' | 'cog' | 'move' | 'c1' | 'c2' | string; name: string; level: RiskLevel | null; reason: string }[]
}

export type TwinInfo = {
  state: 'building' | 'ready'
  /** 기준 형상을 만드는 중이면 몇 프레임째 (10 이면 완료) */
  built: number
  /** 기준 대비 0.5 s 넘게 지속된 변화 (%) */
  changed_pct: number
  pending_pct: number
  /** 거치대 흔들림으로 판정을 멈춘 상태 */
  shaking: boolean
}

export type TwinState = {
  /** 서버 시각 (unix s) */
  t: number
  /** 최근 5 s 안에 센서 프레임을 받았나 */
  connected: boolean
  /** 화면/송신 일시정지 중 */
  paused: boolean
  /** 'replay' = 녹화 재생, 'live' = 실시간, 'none' = 신호 없음 */
  source: 'replay' | 'live' | 'none'
  run: string | null
  /** 시나리오 이름 (예: 정속주행) */
  scenario: string | null
  t_run: number | null
  dur_run: number | null
  /** 화물을 보는 영역 (m) */
  grid: { x0: number; x1: number; y0: number; y1: number } | null
  /** 적재면(받침판) 높이 (m) */
  base: number
  boxes: TwinBox[]
  sensors: TwinSensors
  /** 센서 파이 위험 판정 (없으면 null) */
  risk?: TwinRisk | null
  twin: TwinInfo
  alerts: TwinAlert[]
  link: { hz: number; lost: number; latency_ms: number | null; frames: number }
  /** 새 카메라 사진이 생길 때마다 1 씩 커진다. GET /api/camera.jpg?seq=<n> */
  camera_seq: number
  volume: { level: number; muted: boolean }
  /** 카메라 -> 적재판 위치 맞춤 (m). 판 X = 카메라 x - cx, 판 Z(뒤 +) = -(카메라 y - cy).
   *  서버가 '카메라로 본 화물 부피 중심 = 로드셀 무게중심' 이 되게 낸 값 (가감속 · 화물 이동 없을 때의 중앙값). 옛 서버는 없음 */
  reg?: { cx: number; cy: number; n: number } | null
}

export type ControlCmd = 'pause' | 'resume'
export type VolumeAction = 'up' | 'down' | 'mute' | 'unmute' | 'toggle' | 'set'
