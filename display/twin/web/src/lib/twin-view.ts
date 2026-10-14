/**
 * TwinState -> 화면에 보일 말과 숫자 (판정 · 이유 · 다섯 칸).
 * 판정은 센서 파이(state.risk)의 것을 그대로 쓴다. 여기서는 사람이 읽기 쉬운 말로만 바꾼다.
 * risk 가 없는 옛 녹화면 판정 없이 센서 값만 보여 준다.
 */
import type { RiskLevel, TwinBelt, TwinState } from './twin-types'

export type Level = RiskLevel | null
export type CheckKey = 'load' | 'cog' | 'move' | 'c1' | 'c2'

export type CheckView = {
  key: CheckKey
  name: string
  level: Level
  /** 센서 파이 원문 */
  reason: string
  /** 칸의 큰 숫자 · 단위 */
  value: string
  unit: string
  /** 상태 한 낱말 (정상 · 과적 · 풀림 ...) */
  word: string
  /** 쉬운 문장 (위쪽 띠 · 자세히) */
  sentence: string
  /** 문장 뒤 숫자 설명 */
  detail: string
  /** 3D 이름표 (짧게) */
  tag: string
  /** 0..1 막대 (값 / 기준). 없으면 null */
  ratio: number | null
  /** 막대의 기준 눈금 위치 0..1 */
  mark: number | null
}

export type Verdict = {
  level: Level
  word: string
  headline: string
  detail: string
  /** 맨 위 이유 말고도 정상이 아닌 항목 수 */
  more: number
  top: CheckKey | null
  /** 화물 이동은 종합에 안 들어가고 따로 알린다 (정상이면 null) */
  move: { level: RiskLevel; word: string } | null
}

export type TwinView = { connected: boolean; judged: boolean; verdict: Verdict; checks: CheckView[] }

export const LEVEL_WORD = ['정상', '주의', '경고', '위험'] as const
export const CHECK_ORDER: CheckKey[] = ['load', 'cog', 'move', 'c1', 'c2']
const NAMES: Record<CheckKey, string> = { load: '적재', cog: '무게중심', move: '화물 이동', c1: '앞 벨트', c2: '뒤 벨트' }

const num = (v: unknown): number | null => (typeof v === 'number' && Number.isFinite(v) ? v : null)
const f1 = (v: number) => (Math.abs(v) < 0.05 ? '0.0' : v.toFixed(1))
const pick = (re: RegExp, s: string, i = 1) => {
  const m = re.exec(s)
  return m ? Number(m[i]) : null
}

/** 받침 판 크기 (mm). 없으면 기본값 */
export function bedOf(state: TwinState | null) {
  return state?.sensors?.bed_mm ?? { l: 250, w: 392 }
}

function loadView(state: TwinState, level: Level, reason: string): CheckView {
  const s = state.sensors
  const kg = num(s.load_kg) ?? (num(s.total_g) !== null ? s.total_g! / 1000 : null)
  // 2026-10-08 판정: 적정 적재중량(rated_kg) 의 110 / 125 / 140 %. 옛 녹화는 최대 적재(max_kg) 하나
  const rated = num(s.rated_kg) ?? num(s.max_kg) ?? 15
  const pcts = s.load_pct && s.load_pct.length === 3 ? s.load_pct : null
  const base = { key: 'load' as const, name: NAMES.load, level, reason }
  const value = kg === null ? '—' : f1(kg)
  const pct = kg === null ? null : Math.round((kg / rated) * 100)
  const top = pcts ? pcts[2] / 100 + 0.1 : 1.2
  const common = { value, unit: `/ ${rated} kg`, ratio: kg === null ? null : Math.max(0, kg / (rated * top)), mark: (pcts ? pcts[0] / 100 : 1) / top }
  const detail = pct === null ? '' : `${value} kg · ${pcts ? '적정' : '최대'} ${rated} kg 의 ${pct}%`
  if (level && pcts) {
    const words = ['', '적재 초과', '과적', '심한 과적']
    const sent = ['', '화물이 적정 무게보다 무겁습니다', '과적입니다', '심한 과적입니다']
    return { ...base, ...common, word: words[level], sentence: sent[level], detail, tag: words[level] }
  }
  if (level && /과적/.test(reason)) {
    return { ...base, ...common, word: '과적', sentence: '화물이 너무 무겁습니다', detail, tag: '과적' }
  }
  if (level) {
    return { ...base, ...common, word: '한도 근접', sentence: '적재 무게가 한도에 가깝습니다', detail, tag: '적재 한도 근접' }
  }
  return { ...base, ...common, word: level === 0 ? '정상' : '', sentence: '적재 무게 정상', detail, tag: '적재' }
}

/** 무게중심: 판 가운데에서 어느 쪽으로 몇 mm */
export function cogOffset(state: TwinState | null) {
  const c = state?.sensors?.cog_mm
  const bed = bedOf(state)
  if (!c || c.x === null || c.y === null) return null
  const dx = c.x - bed.l / 2
  const dy = c.y - bed.w / 2
  const side = Math.abs(dy) >= Math.abs(dx) ? (dy > 0 ? '뒤' : '앞') : dx > 0 ? '오른' : '왼'
  return { dx, dy, side, mm: Math.max(Math.abs(dx), Math.abs(dy)), dist: Math.hypot(dx, dy) }
}

function cogView(state: TwinState, level: Level, reason: string): CheckView {
  const off = cogOffset(state)
  const base = { key: 'cog' as const, name: NAMES.cog, level, reason, ratio: null, mark: null }
  if (!off) return { ...base, value: '—', unit: '', word: '', sentence: '무게중심', detail: '', tag: '무게중심' }
  const centred = off.mm < 12
  const value = centred ? '가운데' : `${Math.round(off.mm)}`
  const unit = centred ? '' : `mm ${off.side}쪽`
  if (level) {
    const dir = /(앞|뒤|왼|오른)쪽/.exec(reason)?.[1] ?? off.side
    // 2026-10-08: '왼쪽 쏠림 12% (X 12% Y 50%)' = 가장자리에서 셀 간격의 12 %. 옛: '... 쏠림 25%'
    const edge = pick(/(?:쏠림|끝)\s*(-?\d+)\s*%/, reason)
    const words = ['', '쏠림', '많이 쏠림', '가장자리']
    const sent = ['', `무게가 ${dir}쪽으로 쏠렸습니다`, `무게가 ${dir}쪽으로 많이 쏠렸습니다`, `무게가 ${dir}쪽 가장자리에 있습니다`]
    return {
      ...base,
      value,
      unit,
      word: words[level],
      sentence: sent[level],
      detail: edge !== null ? `가장자리까지 ${edge}% · 가운데에서 ${Math.round(off.dist)} mm` : `가운데에서 ${Math.round(off.dist)} mm`,
      tag: `${dir}쪽 ${words[level]}`,
    }
  }
  return {
    ...base,
    value,
    unit,
    word: level === 0 ? '정상' : '',
    sentence: '무게중심 정상',
    detail: centred ? '판 가운데' : `가운데에서 ${off.side}쪽 ${Math.round(off.mm)} mm`,
    tag: '무게중심',
  }
}

function moveView(state: TwinState, level: Level, reason: string): CheckView {
  const c = state.sensors.cog_mm
  const p = state.sensors.cog_pred_mm
  const gap = c && p && c.x !== null && c.y !== null && p.x !== null && p.y !== null ? Math.hypot(c.x - p.x, c.y - p.y) : null
  const mm = pick(/([\d.]+)\s*mm/, reason)
  const allow = pick(/허용\s*([\d.]+)\s*mm/, reason)
  const shown = level && mm !== null ? mm : gap
  const value = shown === null ? '—' : `${Math.round(shown)}`
  const base = {
    key: 'move' as const,
    name: NAMES.move,
    level,
    reason,
    value,
    unit: 'mm',
    ratio: shown === null ? null : Math.max(0, shown / ((allow ?? 40) * 2)),
    mark: 0.5,
  }
  if (level) {
    const dir = /(앞|뒤|왼|오른)쪽/.exec(reason)?.[0] ?? ''
    // 2026-10-08: '... 이동 앞쪽 86mm (허용 44mm) 의심 0.2초' / '확정 1.8초'. 옛: '확인 중'
    const checking = /의심|확인 중/.test(reason)
    const sec = pick(/(?:의심|확정)\s*([\d.]+)\s*초/, reason)
    const where = dir ? `${dir}으로 ` : ''
    return {
      ...base,
      word: checking ? '의심' : '밀림',
      sentence: checking ? `화물이 ${where}움직이는 것 같습니다` : `화물이 ${where}밀렸습니다`,
      detail: `${value} mm${allow !== null ? ` · 허용 ${allow} mm` : ''}${sec !== null && sec > 0 ? ` · ${sec.toFixed(1)}초째` : ''}`,
      tag: checking ? '화물 움직임 의심' : '화물 밀림',
    }
  }
  return { ...base, word: level === 0 ? '정상' : '', sentence: '화물 이동 없음', detail: `예측과 차이 ${value} mm`, tag: '화물 이동' }
}

function beltView(state: TwinState, key: 'c1' | 'c2', level: Level, reason: string): CheckView {
  const b: TwinBelt | undefined = state.sensors.belts?.find((x) => x.key === key)
  const name = NAMES[key]
  const n = num(b?.n) ?? pick(/([-\d.]+)\s*N/, reason)
  const min = num(b?.min_n) ?? pick(/하한\s*([\d.]+)\s*N/, reason)
  const shownN = n === null ? null : Math.max(0, n)
  const value = shownN === null ? (key === 'c1' && state.sensors.tension_n !== null ? f1(state.sensors.tension_n) : '—') : f1(shownN)
  const base = {
    key,
    name,
    level,
    reason,
    value,
    unit: min !== null ? `/ ${f1(min)} N` : 'N',
    ratio: shownN !== null && min ? Math.max(0, shownN / (min * 1.5)) : null,
    mark: min ? 1 / 1.5 : null,
  }
  const need = min !== null ? ` · 필요 ${f1(min)} N` : ''
  // 2026-10-08: 하한(EN 12195-1)의 % 로 부족 · 풀림, 상한(벨트 · 셀 한계)의 % 로 과조임 · 상한 넘음
  const pct = pick(/하한\s*[\d.]+\s*N\s*의\s*(-?\d+)\s*%/, reason)
  const detail = `${value} N${need}${pct !== null ? ` (${Math.max(0, pct)}%)` : ''}`
  if (level && /상한 넘음/.test(reason)) {
    return { ...base, word: '상한 넘음', sentence: `${name}가 너무 세게 조여 있습니다`, detail, tag: `${name} 상한 넘음` }
  }
  if (level && /과조임/.test(reason) && !/부족|풀림/.test(reason)) {
    return { ...base, word: '과조임', sentence: `${name}가 세게 조여 있습니다`, detail, tag: `${name} 과조임` }
  }
  if (level && /풀림/.test(reason)) {
    return { ...base, word: '풀림', sentence: `${name}가 풀렸습니다`, detail, tag: `${name} 풀림` }
  }
  if (level) {
    return { ...base, word: '장력 부족', sentence: `${name} 장력이 부족합니다`, detail, tag: `${name} 장력 부족` }
  }
  return { ...base, word: level === 0 ? '정상' : '', sentence: `${name} 정상`, detail, tag: name }
}

/** 같은 위험 단계면 무엇을 먼저 말할까 (벨트 풀림 > 과적 > 쏠림 > 장력 부족) */
function weight(c: CheckView) {
  const lv = c.level ?? 0
  const kind = c.word === '풀림' ? 5 : c.key === 'load' ? 3 : c.key === 'cog' ? 2 : 1
  return lv * 10 + kind
}

export function buildView(state: TwinState | null, connected: boolean): TwinView {
  const risk = state?.risk ?? null
  const judged = !!risk && risk.level !== null && risk.level !== undefined
  const checks: CheckView[] = CHECK_ORDER.map((key) => {
    const rc = risk?.checks?.find((c) => c.key === key)
    let level: Level = judged ? ((rc?.level ?? 0) as Level) : null
    // 벨트는 센서 쪽 level 도 있다 (판정이 없을 때 쓰지 않는다)
    if (judged && rc === undefined && (key === 'c1' || key === 'c2')) {
      level = (state?.sensors.belts?.find((b) => b.key === key)?.level ?? 0) as Level
    }
    const reason = rc?.reason ?? ''
    if (!state) {
      return { key, name: NAMES[key], level: null, reason: '', value: '—', unit: '', word: '', sentence: '', detail: '', tag: NAMES[key], ratio: null, mark: null }
    }
    if (key === 'load') return loadView(state, level, reason)
    if (key === 'cog') return cogView(state, level, reason)
    if (key === 'move') return moveView(state, level, reason)
    return beltView(state, key, level, reason)
  })

  // 화물 이동은 종합 위험도에 안 들어가고 따로 알린다 (팀 analysis/overall.py)
  const moveCheck = checks.find((c) => c.key === 'move')
  const move = judged && moveCheck && (moveCheck.level ?? 0) > 0 ? { level: moveCheck.level as RiskLevel, word: moveCheck.word } : null
  let verdict: Verdict
  if (!connected || !state) {
    verdict = { level: null, word: '대기', headline: '센서 신호를 기다리는 중', detail: '', more: 0, top: null, move: null }
  } else if (!judged) {
    // 실시간: 카메라는 오는데 판정이 없다. 센서 값이 일부라도 오면 어떤 센서가 안 읽히는지 알려 준다
    const sn = state.sensors
    const names: [string, number | null][] = [
      ['앞왼', sn.loads.NW], ['앞오', sn.loads.NE], ['뒤왼', sn.loads.SW], ['뒤오', sn.loads.SE],
      ['앞 벨트', sn.belts?.find((b) => b.key === 'c1')?.n ?? sn.tension_n], ['뒤 벨트', sn.belts?.find((b) => b.key === 'c2')?.n ?? null],
    ]
    const missing = names.filter(([, v]) => v === null || v === undefined).map(([n]) => n)
    const any = names.some(([, v]) => v !== null && v !== undefined)
    verdict = any
      ? { level: null, word: '판정 불가', headline: '센서 일부가 읽히지 않습니다', detail: missing.length ? `안 읽힘: ${missing.join(' · ')}` : '판정에 필요한 값이 모자랍니다', more: 0, top: null, move: null }
      : { level: null, word: '판정 대기', headline: '센서 값을 기다리는 중입니다', detail: '카메라 화면만 들어오고 있어요', more: 0, top: null, move: null }
  } else {
    const bad = checks.filter((c) => c.key !== 'move' && (c.level ?? 0) > 0).sort((a, b) => weight(b) - weight(a))
    // 종합 = 센서 파이 값 (안전 여유 곱: 경고 둘이면 위험). 없으면 가장 나쁜 판정
    const level = (risk!.level ?? Math.max(0, ...bad.map((c) => c.level ?? 0))) as RiskLevel
    const bumped = bad.length > 0 && level > (bad[0].level ?? 0)
    const why = bumped ? ' · 겹쳐서 한 단계 올림' : ''
    if (!bad.length) {
      const load = checks[0]
      verdict = {
        level,
        word: LEVEL_WORD[level],
        headline: move ? '화물 이동을 확인하세요' : '화물이 안전하게 고정되어 있습니다',
        detail: `${load.value} kg · 벨트 장력 정상`,
        more: 0,
        top: move ? 'move' : null,
        move,
      }
    } else {
      const top = bad[0]
      const pair = bad.find((c) => c !== top && c.word === '풀림' && top.word === '풀림' && c.level === top.level)
      if (pair) {
        const [a, b] = top.key === 'c1' ? [top, pair] : [pair, top]
        verdict = {
          level,
          word: LEVEL_WORD[level],
          headline: '앞 · 뒤 벨트가 모두 풀렸습니다',
          detail: `앞 ${a.value} N · 뒤 ${b.value} N${why}`,
          more: bad.length - 2,
          top: top.key,
          move,
        }
      } else {
        verdict = { level, word: LEVEL_WORD[level], headline: top.sentence, detail: top.detail + why, more: bad.length - 1, top: top.key, move }
      }
    }
  }
  return { connected: connected && !!state, judged, verdict, checks }
}

/** 단계별 색 (화면 · 3D 같이 씀): 정상 초록 · 주의 노랑 · 경고 주황 · 위험 빨강 */
export const LEVEL_COLOR = {
  0: '#3fd59a',
  1: '#ffd23f',
  2: '#ff9640',
  3: '#ff5a52',
  none: '#9aa3ad',
} as const
export const levelColor = (l: Level) => (l === null ? LEVEL_COLOR.none : LEVEL_COLOR[l])
