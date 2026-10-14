/**
 * 개발용 모의 데이터 (?mock=1).
 * twin-sample-1004.json (2026-10-04 녹화 한 프레임) 을 바탕으로, 실제 녹화의 다섯 장면 + 정상 + 판정 없음을
 * 7 s 씩 돌려 가며 10 Hz 로 TwinState 를 만든다. 위험 판정 문구는 센서 파이(analysis/risk.py)의 실제 문구 모양을 따른다.
 * 백엔드 호출(제어 · 볼륨 · 종료 · 통계)은 여기서 흉내만 낸다. ?mockSignal=0 이면 신호 없음.
 */
import { useEffect, useState } from 'react'
import sampleJson from './twin-sample-1004.json'
import type { ControlCmd, RiskLevel, TwinAlert, TwinRisk, TwinState, VolumeAction } from './twin-types'

const SAMPLE = sampleJson as unknown as TwinState
const PHASE_S = 7
const DT = 0.1

type Phase = {
  run: string
  scenario: string
  dur: number
  kg: number
  /** 무게중심 (mm) 과 예측 */
  cog: [number, number]
  pred: [number, number]
  c1: [number, number]
  c2: [number, number]
  roll: number
  pitch: number
  shake: number
  judged: boolean
  checks: Partial<Record<'load' | 'cog' | 'move' | 'c1' | 'c2', [RiskLevel, string]>>
}

// 판정 문구 · 단계는 팀 레포 2026-10-08 판정(pi/analysis)을 10-04 녹화에 다시 돌린 결과에서 옮김 (0 정상 1 주의 2 경고 3 위험)
const PHASES: Phase[] = [
  { run: 'cruise', scenario: '정속 주행', dur: 12, kg: 9.6, cog: [122, 201], pred: [121, 199], c1: [9.4, 5.1], c2: [14.8, 9.9], roll: 0.4, pitch: -0.6, shake: 0.2, judged: true, checks: {} },
  {
    run: 'load', scenario: '과적 적재', dur: 15.8, kg: 13.9, cog: [113, 268], pred: [112, 265], c1: [5.2, 5.9], c2: [8.3, 12.9], roll: 0.5, pitch: 0.4, shake: 0.3, judged: true,
    checks: {
      load: [2, '과적 13.9kg = 적정 10kg 의 139% (125% 넘음)'],
      c1: [1, '부족 5.2N = 하한 5.9N 의 89% (100% 밑) · NW+NE 4.4kg'],
      c2: [2, '부족 8.3N = 하한 12.9N 의 64% (75% 밑) · SW+SE 9.6kg'],
    },
  },
  {
    run: 'brake', scenario: '급출발·급정거', dur: 7, kg: 13.9, cog: [123, 302], pred: [121, 298], c1: [5.3, 6.0], c2: [8.7, 12.9], roll: 0.3, pitch: 4.5, shake: 1, judged: true,
    checks: {
      load: [2, '과적 13.9kg = 적정 10kg 의 139% (125% 넘음)'],
      cog: [1, '뒤쪽 쏠림 23% (X 49% Y 77%)'],
      c1: [1, '부족 5.3N = 하한 6.0N 의 88% (100% 밑) · NW+NE 4.5kg'],
      c2: [2, '부족 8.7N = 하한 12.9N 의 67% (75% 밑) · SW+SE 9.6kg'],
    },
  },
  {
    run: 'turn', scenario: '선회', dur: 19, kg: 13.8, cog: [104, 268], pred: [108, 266], c1: [4.6, 5.6], c2: [8.6, 12.8], roll: -3.8, pitch: -1.2, shake: 0.5, judged: true,
    checks: {
      load: [2, '과적 13.8kg = 적정 10kg 의 138% (125% 넘음)'],
      c1: [1, '부족 4.6N = 하한 5.6N 의 82% (100% 밑) · NW+NE 4.2kg'],
      c2: [2, '부족 8.6N = 하한 12.8N 의 67% (75% 밑) · SW+SE 9.5kg'],
    },
  },
  {
    run: 'rear_loose', scenario: '뒤 벨트 풀림', dur: 8.3, kg: 12.7, cog: [120, 168], pred: [124, 250], c1: [4.7, 6.5], c2: [0.9, 12.7], roll: 1.2, pitch: 2.4, shake: 0.6, judged: true,
    checks: {
      load: [2, '과적 12.7kg = 적정 10kg 의 127% (125% 넘음)'],
      move: [3, '가감속으로 설명 안 되는 이동 앞쪽 92mm (허용 44mm) 확정 1.8초'],
      c1: [2, '부족 4.7N = 하한 6.5N 의 73% (75% 밑) · NW+NE 4.8kg'],
      c2: [3, '풀림 0.9N = 하한 12.7N 의 7% (50% 밑) · SW+SE 9.4kg'],
    },
  },
  {
    run: 'both_loose', scenario: '양쪽 벨트 풀림', dur: 10.5, kg: 14.0, cog: [115, 259], pred: [117, 257], c1: [1.5, 6.2], c2: [1.2, 12.9], roll: 0.9, pitch: 0.6, shake: 0.5, judged: true,
    checks: {
      load: [2, '과적 14.0kg = 적정 10kg 의 140% (125% 넘음)'],
      c1: [3, '풀림 1.5N = 하한 6.2N 의 24% (50% 밑) · NW+NE 4.6kg'],
      c2: [3, '풀림 1.2N = 하한 12.9N 의 9% (50% 밑) · SW+SE 9.6kg'],
    },
  },
  { run: 'old', scenario: '옛 녹화 (판정 없음)', dur: 10, kg: 11.8, cog: [128, 190], pred: [128, 190], c1: [7.2, 5.0], c2: [11.5, 9.5], roll: 0.2, pitch: -0.3, shake: 0.2, judged: false, checks: {} },
]

/** 팀 overall.py: 적재 · 무게중심 · 장력(높은 쪽) 단계의 안전 여유 곱. 화물 이동은 안 넣는다 */
function combine(levels: number[]) {
  let left = 1
  for (const lv of levels) left *= 1 - lv / 4
  return Math.floor(4 * (1 - left) + 1e-9)
}

// 정규분포 잡음 (Box-Muller)
function randn() {
  const u = 1 - Math.random()
  const v = Math.random()
  return Math.sqrt(-2 * Math.log(u)) * Math.cos(2 * Math.PI * v)
}
const NAMES = { load: '적재', cog: '무게중심', move: '화물 이동', c1: '앞 벨트', c2: '뒤 벨트' } as const

type Listener = (s: TwinState) => void

class MockEngine {
  simT = 0
  frames = 0
  paused = false
  volume = { level: 0.8, muted: false }
  signal = new URLSearchParams(location.search).get('mockSignal') !== '0'
  /** ?mockPhase=4 이면 그 장면에 머문다 */
  hold = (() => {
    const v = new URLSearchParams(location.search).get('mockPhase')
    return v === null ? null : Math.max(0, Math.min(PHASES.length - 1, Number(v) || 0))
  })()
  private listeners = new Set<Listener>()
  private timer: number | undefined
  private last: TwinState | null = null
  private lastTick = performance.now()

  subscribe(fn: Listener) {
    this.listeners.add(fn)
    if (this.last) fn(this.last)
    if (this.timer === undefined) {
      this.lastTick = performance.now()
      this.timer = window.setInterval(() => this.tick(), DT * 1000)
    }
    return () => {
      this.listeners.delete(fn)
      if (!this.listeners.size) {
        window.clearInterval(this.timer)
        this.timer = undefined
      }
    }
  }

  private tick() {
    const now = performance.now()
    const dt = Math.min(1, Math.max(0, (now - this.lastTick) / 1000))
    this.lastTick = now
    if (!this.paused) {
      this.simT += dt
      this.frames += 1
    }
    this.last = this.paused && this.last ? { ...this.last, t: Date.now() / 1000, paused: true, volume: { ...this.volume } } : this.build()
    for (const fn of this.listeners) fn(this.last)
  }

  private build(): TwinState {
    const idx = this.hold ?? Math.floor(this.simT / PHASE_S) % PHASES.length
    const ph = PHASES[idx]
    const inPhase = this.simT % PHASE_S
    const wob = Math.sin(this.simT * 2.3) * ph.shake
    const j = (v: number, sd: number) => v + randn() * sd
    const bed = SAMPLE.sensors.bed_mm ?? { l: 250, w: 392 }
    const cx = j(ph.cog[0] + wob * 3, 0.8)
    const cy = j(ph.cog[1] + wob * 8, 0.8)
    // 무게중심 -> 네 셀 몫
    const fx = Math.max(0, Math.min(1, cx / bed.l))
    const fy = Math.max(0, Math.min(1, cy / bed.w))
    const g = ph.kg * 1000
    const loads = {
      NW: j(g * (1 - fx) * (1 - fy), 20),
      NE: j(g * fx * (1 - fy), 20),
      SW: j(g * (1 - fx) * fy, 20),
      SE: j(g * fx * fy, 20),
    }
    const lvBelt = (k: 'c1' | 'c2') => (ph.judged ? (ph.checks[k]?.[0] ?? 0) : null)
    const belts = (['c1', 'c2'] as const).map((k) => ({
      key: k,
      name: NAMES[k],
      n: Math.round(j(ph[k][0], 0.05) * 100) / 100,
      min_n: ph[k][1],
      level: lvBelt(k) as RiskLevel | null,
    }))
    // 판정은 장면 시작 0.5 s 뒤부터 (실제 녹화도 처음 잠깐은 정상)
    const settled = inPhase > 0.5
    let risk: TwinRisk | null = null
    if (ph.judged) {
      const checks = (Object.keys(NAMES) as (keyof typeof NAMES)[]).map((key) => {
        const c = settled ? ph.checks[key] : undefined
        return { key, name: NAMES[key], level: (c?.[0] ?? 0) as RiskLevel, reason: c?.[1] ?? '' }
      })
      const lv = (k: string) => checks.find((c) => c.key === k)?.level ?? 0
      const level = combine([lv('load'), lv('cog'), Math.max(lv('c1'), lv('c2'))]) as RiskLevel
      risk = {
        level,
        text: checks.filter((c) => c.level).map((c) => `${['', '주의', '경고', '위험'][c.level]} ${c.name}: ${c.reason}`).join(' | '),
        checks,
      }
    }
    const alerts: TwinAlert[] =
      this.signal && risk ? risk.checks.filter((c) => c.level).map((c) => ({ kind: c.key, text: `${c.name} · ${c.reason}`, severe: (c.level ?? 0) >= 2 })) : []
    const boxes = SAMPLE.boxes.map((b) => ({ ...b, alert: false }))
    // 뒤 벨트가 풀린 장면: 화물이 앞으로 밀린다
    if (ph.run === 'rear_loose' && settled) for (const b of boxes) (b.y0 += 0.03), (b.y1 += 0.03)
    return {
      ...SAMPLE,
      t: Date.now() / 1000,
      connected: this.signal,
      paused: this.paused,
      source: this.signal ? 'replay' : 'none',
      run: ph.run,
      scenario: ph.scenario,
      t_run: (inPhase / PHASE_S) * ph.dur,
      dur_run: ph.dur,
      boxes,
      sensors: {
        ...SAMPLE.sensors,
        total_g: loads.NW + loads.NE + loads.SW + loads.SE,
        loads,
        cog: { x_pct: (cx / bed.l) * 100, y_pct: (cy / bed.w) * 100 },
        tension_n: belts[0].n,
        roll: j(ph.roll + wob * 0.6, 0.05),
        pitch: j(ph.pitch + wob * 0.9, 0.05),
        yaw_rate: j(ph.run === 'turn' ? -32 : 0, 0.5),
        ax: j(ph.run === 'brake' ? 0.18 * Math.sin(this.simT * 1.4) : 0.01, 0.004),
        ay: j(ph.run === 'turn' ? -0.12 : 0, 0.004),
        load_kg: Math.round(j(ph.kg, 0.02) * 1000) / 1000,
        max_kg: 10,
        rated_kg: 10,
        load_pct: [110, 125, 140],
        bed_mm: bed,
        cog_mm: { x: cx, y: cy },
        cog_pred_mm: { x: j(ph.pred[0], 0.5), y: j(ph.pred[1], 0.5) },
        belts,
      },
      risk,
      alerts,
      link: { hz: this.signal ? 9.6 + Math.random() * 0.6 : 0, lost: 0, latency_ms: this.signal ? 0.3 : null, frames: this.frames },
      camera_seq: 0,
      volume: { ...this.volume },
    }
  }

  control(cmd: ControlCmd) {
    if (cmd === 'pause') this.paused = true
    else if (cmd === 'resume') this.paused = false
    return Promise.resolve({ ok: true })
  }

  setVolume(action: VolumeAction, value?: number) {
    const v = this.volume
    if (action === 'up') v.level = Math.min(1.5, v.level + 0.05)
    else if (action === 'down') v.level = Math.max(0, v.level - 0.05)
    else if (action === 'mute') v.muted = true
    else if (action === 'unmute') v.muted = false
    else if (action === 'toggle') v.muted = !v.muted
    else if (action === 'set' && value !== undefined) v.level = Math.max(0, Math.min(1.5, value))
    v.level = Math.round(v.level * 100) / 100
    return Promise.resolve({ ...v })
  }
}

let engine: MockEngine | null = null
const getEngine = () => (engine ??= new MockEngine())

/** useTwinState() 와 같은 모양: { state, online } */
export function useMockTwinState() {
  const [state, setState] = useState<TwinState | null>(null)
  useEffect(() => getEngine().subscribe(setState), [])
  return { state, online: state !== null }
}

/** twin-api 의 함수들과 같은 모양의 모의 함수 */
export const mockApi = {
  sendControl: (cmd: ControlCmd) => getEngine().control(cmd),
  setVolume: (action: VolumeAction, value?: number) => getEngine().setVolume(action, value),
  exitApp: () => Promise.resolve({ ok: true, mock: true }),
  reportStats: (stats: Record<string, number | string>) => {
    if (new URLSearchParams(location.search).has('debug')) console.info('[stats]', stats)
    return Promise.resolve()
  },
}
