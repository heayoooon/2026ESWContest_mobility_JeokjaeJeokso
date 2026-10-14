/**
 * 트윈 화면의 작은 조각들: 위쪽 판정 띠, 아래 다섯 칸, 모서리 버튼, 볼륨, 기울기, 종료 확인.
 */
import { useCallback, useEffect, useRef, useState, type ReactNode } from 'react'
import { Crosshair, House, Link2, Minus, Move, Package, Pause, Play, Plus, RotateCcw, Rotate3d, Volume2, VolumeX } from 'lucide-react'
import type { TwinState, VolumeAction } from '@/lib/twin-types'
import { LEVEL_WORD, type CheckKey, type CheckView, type Level, type TwinView } from '@/lib/twin-view'

const lvClass = (l: Level) => (l === null ? 'lvn' : `lv${l}`)

/** 경고 · 주의 삼각 표지 (정상이 아니면 깜박인다) / 정상은 동그라미 체크 */
export function StatusSign({ level }: { level: Level }) {
  if (level !== null && level > 0) {
    return (
      <svg className={`tw-sign blink${level}`} viewBox="0 0 48 44" aria-hidden>
        <path d="M24 3.5 45 40.5H3z" fill="currentColor" stroke="currentColor" strokeWidth="3.5" strokeLinejoin="round" />
        <path d="M24 16v12" stroke="#16130f" strokeWidth="4.6" strokeLinecap="round" />
        <circle cx="24" cy="34" r="2.7" fill="#16130f" />
      </svg>
    )
  }
  if (level === 0) {
    return (
      <svg className="tw-sign" viewBox="0 0 48 48" aria-hidden>
        <circle cx="24" cy="24" r="20" fill="currentColor" />
        <path d="m15 24.5 6 6 12-13" fill="none" stroke="#0d1a14" strokeWidth="4.4" strokeLinecap="round" strokeLinejoin="round" />
      </svg>
    )
  }
  return (
    <svg className="tw-sign" viewBox="0 0 48 48" aria-hidden>
      <circle cx="24" cy="24" r="19" fill="none" stroke="currentColor" strokeWidth="3" />
      <path d="M16 24h16" stroke="currentColor" strokeWidth="3.4" strokeLinecap="round" />
    </svg>
  )
}

function sourceText(state: TwinState | null, connected: boolean, paused: boolean) {
  if (!state || !connected) return { cls: 'none', text: '신호 없음' }
  if (paused) return { cls: 'paused', text: '일시정지' }
  if (state.source === 'live') return { cls: 'live', text: '실시간' }
  return { cls: 'replay', text: '녹화 재생' }
}

export function Banner({ view, state, paused }: { view: TwinView; state: TwinState | null; paused: boolean }) {
  const v = view.verdict
  const src = sourceText(state, view.connected, paused)
  const prog = view.connected && state?.t_run != null && state?.dur_run ? Math.max(0, Math.min(1, state.t_run / state.dur_run)) : 0
  const hz = view.connected && state ? state.link.hz.toFixed(1) : '—'
  return (
    <header className={`tw-banner ${lvClass(v.level)}`}>
      <StatusSign level={v.level} />
      <div className="tw-verdict">{v.word}</div>
      <div className="tw-reason">
        <strong>{v.headline}</strong>
        {(v.detail || v.more > 0 || v.move) && (
          <span>
            {v.detail}
            {v.more > 0 && <em>외 {v.more}건</em>}
            {v.move && <em className={`mv lv${v.move.level}`}>화물 이동 {LEVEL_WORD[v.move.level]}</em>}
          </span>
        )}
      </div>
      {/* 색 뜻: 화면 전체(띠 · 칸 · 3D · 판)에 같은 색을 쓴다 */}
      <div className="tw-legend-top" aria-label="색 뜻">
        <span className="lv0"><i />정상</span>
        <span className="lv1"><i />주의</span>
        <span className="lv2"><i />경고</span>
        <span className="lv3"><i />위험</span>
      </div>
      <div className="tw-meta">
        <div className="tw-scn">{view.connected ? (state?.scenario ?? state?.run ?? '—') : '—'}</div>
        <div className="tw-prog">
          <i style={{ transform: `scaleX(${prog})` }} />
        </div>
        <div className={`tw-src ${src.cls}`}>
          <i />
          {src.text}
          <span>{hz} Hz</span>
        </div>
      </div>
    </header>
  )
}

const ICONS: Record<CheckKey, ReactNode> = {
  load: <Package size={17} strokeWidth={2} />,
  cog: <Crosshair size={17} strokeWidth={2} />,
  move: <Move size={17} strokeWidth={2} />,
  c1: <Link2 size={17} strokeWidth={2} />,
  c2: <Link2 size={17} strokeWidth={2} />,
}

export function CheckTiles({ view, focus, onPick }: { view: TwinView; focus: CheckKey | null; onPick: (k: CheckKey) => void }) {
  return (
    <nav className="tw-tiles" aria-label="점검 항목">
      {view.checks.map((c) => (
        <Tile key={c.key} c={c} on={focus === c.key} dim={!view.connected} onPick={onPick} />
      ))}
    </nav>
  )
}

function Tile({ c, on, dim, onPick }: { c: CheckView; on: boolean; dim: boolean; onPick: (k: CheckKey) => void }) {
  const lv = dim ? null : c.level
  const word = dim ? '' : c.level === null ? '' : c.level === 0 ? '정상' : c.word
  return (
    <button type="button" className={`tw-tile ${lvClass(lv)}${on ? ' sel' : ''}`} aria-pressed={on} onClick={() => onPick(c.key)}>
      <div className="tw-tile-head">
        {ICONS[c.key]}
        <span className="nm">{c.name}</span>
        {word && <span className="wd">{word}</span>}
      </div>
      <div className="tw-tile-val">
        <b>{dim ? '—' : c.value}</b>
        {!dim && c.unit && <small>{c.unit}</small>}
      </div>
      <div className="tw-tile-bar">
        {!dim && c.ratio !== null && <i style={{ transform: `scaleX(${Math.min(1, c.ratio)})` }} />}
        {!dim && c.mark !== null && <s style={{ left: `${c.mark * 100}%` }} />}
      </div>
    </button>
  )
}

/** 고른 항목 자세히 (3D 아래쪽 가운데) */
type Loads = TwinState['sensors']['loads']

export function DetailCard({ c, judged, loads, onClose }: { c: CheckView; judged: boolean; loads?: Loads | null; onClose: () => void }) {
  const tot = loads ? [loads.NW, loads.NE, loads.SW, loads.SE].reduce<number>((a, g) => a + Math.max(0, g ?? 0), 0) : 0
  const cell = (id: keyof Loads, name: string) => {
    const g = loads?.[id] ?? null
    return (
      <div className="cell" key={id}>
        <span>{name}</span>
        <b>{g === null ? '—' : (g / 1000).toFixed(1)}</b>
        <small>kg · {g === null || tot <= 0 ? '—' : Math.round((Math.max(0, g) / tot) * 100)}%</small>
      </div>
    )
  }
  const word = c.level === null ? (judged ? '' : '판정 없음') : LEVEL_WORD[c.level]
  return (
    <div className={`tw-detail ${lvClass(c.level)}`} role="status" onClick={onClose}>
      <div className="tw-detail-head">
        <span className="nm">{c.name}</span>
        {word && <span className="wd">{word}</span>}
      </div>
      <strong>{c.sentence}</strong>
      <span className="dt">{c.detail}</span>
      {c.reason && <span className="raw">센서 판정: {c.reason}</span>}
      {loads && (c.key === 'load' || c.key === 'cog') && (
        <div className="tw-cells" aria-label="네 모서리 로드셀 (위가 앞)">
          {cell('NW', '앞 왼쪽')}
          {cell('NE', '앞 오른쪽')}
          {cell('SW', '뒤 왼쪽')}
          {cell('SE', '뒤 오른쪽')}
        </div>
      )}
    </div>
  )
}

export function IconButton({ label, onClick, children, className = '', pressed }: { label: string; onClick: () => void; children: ReactNode; className?: string; pressed?: boolean }) {
  return (
    <button type="button" className={`tw-btn ${className}`} aria-label={label} aria-pressed={pressed} onClick={onClick}>
      {children}
    </button>
  )
}

export function TopControls({
  presetLabel,
  paused,
  onPreset,
  onPause,
  onRestart,
}: {
  presetLabel: string
  paused: boolean
  onPreset: () => void
  onPause: () => void
  /** 녹화 재생 때만 (실시간에는 없다) */
  onRestart?: () => void
}) {
  return (
    <div className="tw-topctl">
      <button type="button" className="tw-btn tw-view" onClick={onPreset} aria-label={`시점 바꾸기 (지금 ${presetLabel})`}>
        <Rotate3d size={18} strokeWidth={1.9} />
        <span>{presetLabel}</span>
      </button>
      <IconButton label={paused ? '재생' : '일시정지'} onClick={onPause} pressed={paused}>
        {paused ? <Play size={18} strokeWidth={2} fill="currentColor" /> : <Pause size={18} strokeWidth={2} fill="currentColor" />}
      </IconButton>
      {onRestart && (
        <IconButton label="처음부터" onClick={onRestart}>
          <RotateCcw size={18} strokeWidth={2} />
        </IconButton>
      )}
    </div>
  )
}

export function HomeButton({ onPress }: { onPress: () => void }) {
  return (
    <IconButton label="홈 (끝내기)" onClick={onPress} className="tw-home">
      <House size={19} strokeWidth={1.9} />
    </IconButton>
  )
}

/** 기울기: 작은 물방울 수평계 + 숫자 */
export function TiltBadge({ roll, pitch, dim }: { roll: number | null; pitch: number | null; dim: boolean }) {
  const r = roll ?? 0
  const p = pitch ?? 0
  const mag = Math.hypot(r, p)
  const k = 1.6 // px / deg
  const bx = Math.max(-13, Math.min(13, r * k))
  const by = Math.max(-13, Math.min(13, -p * k))
  return (
    <div className={`tw-tilt${dim || roll === null ? ' dim' : ''}${mag > 8 ? ' big' : ''}`}>
      <div className="tw-bubble">
        <i style={{ transform: `translate(${bx.toFixed(1)}px, ${by.toFixed(1)}px)` }} />
      </div>
      <div className="tw-tilt-txt">
        <b><small>기울기 </small>{roll === null ? '—' : `${mag.toFixed(1)}°`}</b>
      </div>
    </div>
  )
}

// ── 볼륨: [음소거] [−] 막대 · % [+] ──
type Vol = { level: number; muted: boolean }
const VMAX = 1.5
const STEP = 0.05
const HOLD_MS = 1500

export function VolumePill({ volume, setVolume }: { volume: Vol | null | undefined; setVolume: (a: VolumeAction, v?: number) => Promise<Vol> }) {
  const [vol, setVol] = useState<Vol>(volume ?? { level: 1, muted: false })
  const touched = useRef(-Infinity)
  const volRef = useRef(vol)
  volRef.current = vol
  // 서버 값에 맞추기 (방금 손댄 게 아니면)
  useEffect(() => {
    if (!volume || performance.now() - touched.current < HOLD_MS) return
    setVol((v) => (v.level === volume.level && v.muted === volume.muted ? v : { ...volume }))
  }, [volume])
  const act = useCallback(
    (a: VolumeAction) => {
      touched.current = performance.now()
      const v = volRef.current
      if (a === 'up' || a === 'down') {
        const level = Math.round(Math.max(0, Math.min(VMAX, v.level + (a === 'up' ? STEP : -STEP))) * 100) / 100
        if (a === 'up' && v.muted) setVolume('unmute').catch(() => {})
        setVol({ level, muted: a === 'up' ? false : v.muted })
      } else if (a === 'toggle') setVol({ ...v, muted: !v.muted })
      setVolume(a)
        .then((r) => {
          if (r && typeof r.level === 'number') {
            touched.current = performance.now()
            setVol({ level: r.level, muted: !!r.muted })
          }
        })
        .catch(() => {})
    },
    [setVolume]
  )
  const pct = Math.round(vol.level * 100)
  const segs = 6
  const on = vol.muted ? 0 : Math.round((vol.level / VMAX) * segs)
  return (
    <div className={`tw-vol${vol.muted ? ' muted' : ''}`} role="group" aria-label="소리 크기">
      <button type="button" className="tw-vbtn" aria-label={vol.muted ? '소리 켜기' : '소리 끄기'} aria-pressed={vol.muted} onClick={() => act('toggle')}>
        {vol.muted ? <VolumeX size={18} strokeWidth={2} /> : <Volume2 size={18} strokeWidth={2} />}
      </button>
      <button type="button" className="tw-vbtn" aria-label="작게" onClick={() => act('down')}>
        <Minus size={17} strokeWidth={2.2} />
      </button>
      <div className="tw-vlevel" aria-live="polite">
        <em>음량</em>
        <div className="bars">
          {Array.from({ length: segs }, (_, i) => (
            <i key={i} className={i < on ? 'on' : ''} style={{ height: 6 + i * 2 }} />
          ))}
        </div>
        <span>{vol.muted ? '꺼짐' : `${pct}%`}</span>
      </div>
      <button type="button" className="tw-vbtn" aria-label="크게" onClick={() => act('up')}>
        <Plus size={17} strokeWidth={2.2} />
      </button>
    </div>
  )
}

export function ConfirmSheet({ open, onCancel, onConfirm }: { open: boolean; onCancel: () => void; onConfirm: () => Promise<void> }) {
  const [busy, setBusy] = useState(false)
  const [err, setErr] = useState(false)
  useEffect(() => {
    if (!open) {
      setBusy(false)
      setErr(false)
    }
  }, [open])
  if (!open) return null
  return (
    <div className="tw-sheet-bg" onClick={busy ? undefined : onCancel}>
      <div className="tw-sheet" role="dialog" aria-modal="true" onClick={(e) => e.stopPropagation()}>
        <strong>트윈 화면을 끝낼까요?</strong>
        <span>{err ? '서버에 닿지 않았습니다. 다시 눌러 주세요.' : '바탕화면(홈)으로 돌아갑니다.'}</span>
        <div className="row">
          <button type="button" className="tw-sbtn" onClick={onCancel} disabled={busy}>
            취소
          </button>
          <button
            type="button"
            className="tw-sbtn go"
            disabled={busy}
            onClick={() => {
              setBusy(true)
              setErr(false)
              onConfirm().catch(() => {
                setBusy(false)
                setErr(true)
              })
            }}
          >
            {busy ? '끝내는 중…' : '끝내기'}
          </button>
        </div>
      </div>
    </div>
  )
}

export function Waiting() {
  return (
    <div className="tw-wait">
      <i />
      <strong>센서 신호를 기다리는 중</strong>
      <span>연결되면 바로 화물 상태를 보여 드립니다</span>
    </div>
  )
}

/** 보기 모드 (홈 단추 옆): 기본 · 투시 · 분해 · 센서 */
export function ModeSwitch<T extends string>({ modes, value, onChange }: { modes: { id: T; label: string; icon?: ReactNode }[]; value: T; onChange: (m: T) => void }) {
  return (
    <div className="tw-modes" role="radiogroup" aria-label="보기 모드">
      {modes.map((m) => (
        <button key={m.id} type="button" role="radio" aria-checked={value === m.id} className={`tw-btn tw-mode${value === m.id ? ' on' : ''}`} onClick={() => onChange(m.id)}>
          {m.icon}
          <span>{m.label}</span>
        </button>
      ))}
    </div>
  )
}

/** 색 뜻 (왼쪽 아래, 작게) */
export function LevelLegend() {
  return (
    <div className="tw-legend" aria-label="색 뜻">
      <span className="lv1"><i />주의</span>
      <span className="lv2"><i />경고</span>
      <span className="lv3"><i />위험</span>
    </div>
  )
}

/** 무게중심 지도 (왼쪽 막대): 판을 위에서 본 모양. 팀 판정의 구역(가장자리까지 셀 간격의 25 · 15 · 7 %)을 색 띠로,
 *  지금 무게중심을 점으로 (점 색 = 무게중심 단계). 아래에 기울기 숫자 */
export function CogMap({ state, level, roll, pitch, dim }: { state: TwinState | null; level: Level; roll: number | null; pitch: number | null; dim: boolean }) {
  const bed = state?.sensors.bed_mm ?? { l: 250, w: 392 }
  const c = state?.sensors.cog_mm
  const has = !dim && !!c && c.x !== null && c.y !== null
  const L = bed.l
  const W = bed.w
  const band = (pct: number) => ({ x: (L * pct) / 100, y: (W * pct) / 100, w: L * (1 - (2 * pct) / 100), h: W * (1 - (2 * pct) / 100) })
  const zones = [
    { p: 0, cls: 'z3' },
    { p: 7, cls: 'z2' },
    { p: 15, cls: 'z1' },
    { p: 25, cls: 'z0' },
  ]
  const x = has ? Math.max(0, Math.min(L, c!.x!)) : L / 2
  const y = has ? Math.max(0, Math.min(W, c!.y!)) : W / 2
  const tilt = roll === null ? null : Math.hypot(roll, pitch ?? 0)
  return (
    <div className={`tw-cogmap ${lvClass(has ? level : null)}`} aria-label="무게중심 지도">
      <svg viewBox={`-14 -30 ${L + 28} ${W + 44}`}>
        <text x={L / 2} y={-8} textAnchor="middle" className="front">앞</text>
        {zones.map((z) => {
          const b = band(z.p)
          return <rect key={z.p} className={z.cls} x={b.x} y={b.y} width={b.w} height={b.h} rx={10} />
        })}
        <line className="cross" x1={L / 2} y1={0} x2={L / 2} y2={W} />
        <line className="cross" x1={0} y1={W / 2} x2={L} y2={W / 2} />
        {has && (
          <>
            <circle className="halo" cx={x} cy={y} r={34} />
            <circle className="dot" cx={x} cy={y} r={20} />
          </>
        )}
      </svg>
      <b>무게중심</b>
      <small>기울기 {tilt === null ? '—' : `${tilt.toFixed(1)}°`}</small>
    </div>
  )
}
