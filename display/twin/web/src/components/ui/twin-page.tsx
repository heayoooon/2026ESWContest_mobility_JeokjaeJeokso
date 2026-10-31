import { Suspense, lazy, useCallback, useEffect, useLayoutEffect, useMemo, useRef, useState, type ReactNode } from 'react'
import { Box, Gauge, Layers, ScanEye } from 'lucide-react'
import {
  Banner,
  CheckTiles,
  ConfirmSheet,
  DetailCard,
  HomeButton,
  CogMap,
  ModeSwitch,
  TopControls,
  VolumePill,
  Waiting,
} from '@/components/ui/twin-controls'
import { useFpsReport } from '@/hooks/use-fps-report'
import { useTwinSource } from '@/hooks/use-twin-source'
import type { DriveMode, TwinState } from '@/lib/twin-types'
import { buildView, type CheckKey } from '@/lib/twin-view'
import './twin-page.css'

/** 3D 장면(three.js)은 따로 받아 온다 (인트로가 빨리 뜨도록). 인트로 중에 미리 받아 둔다 */
const loadScene = () => import('@/components/ui/twin-scene-3d')
const TwinScene3D = lazy(loadScene)
export const preloadTwin3D = () => {
  loadScene().catch(() => {})
}

type Preset = 'main' | 'top' | 'side'
const PRESETS: { id: Preset; label: string }[] = [
  { id: 'main', label: '기본' },
  { id: 'top', label: '위' },
  { id: 'side', label: '옆' },
]
const FOCUS_MS = 15000
/** 막대 폭 + 틈 (px, twin-page.css .tw-rail 과 맞춘다). 왼쪽은 무게중심 지도가 들어가 조금 넓다 */
const RAIL_L = 84
const RAIL_R = 68
type ViewMode = 'normal' | 'xray' | 'explode' | 'sensor'
const VIEW_MODES: { id: ViewMode; label: string; icon: ReactNode }[] = [
  { id: 'normal', label: '기본', icon: <Box size={17} strokeWidth={1.9} /> },
  { id: 'xray', label: '투시', icon: <ScanEye size={17} strokeWidth={1.9} /> },
  { id: 'explode', label: '분해', icon: <Layers size={17} strokeWidth={1.9} /> },
  { id: 'sensor', label: '센서', icon: <Gauge size={17} strokeWidth={1.9} /> },
]

/**
 * 디지털 트윈 화면 (800x480 기준).
 *  위: 판정 띠 (정상 · 주의 · 위험 + 가장 중요한 이유)
 *  가운데: 3D 적재 판 · 화물 · 벨트 · 무게중심 (문제 있는 곳에 이름표)
 *  아래: 다섯 칸 (적재 · 무게중심 · 화물 이동 · 앞 벨트 · 뒤 벨트). 누르면 그곳을 가까이 보고 자세한 이유
 *  모서리: 홈 · 시점 · 일시정지 · 적재/주행 · 볼륨
 * onExited: 모의 모드에서 '끝내기' 를 누르면 (닫을 브라우저가 없으므로) 부른다.
 */
export default function TwinPage({ onExited }: { onExited?: () => void }) {
  const { state, online, api, mock } = useTwinSource()
  const connected = online && !!state?.connected
  // 서버와 끊기면 마지막 상태가 남는다 -> 장면에는 '신호 없음'으로 바꿔 넘긴다
  const sceneState = useMemo<TwinState | null>(() => {
    if (!state || online) return state
    return { ...state, connected: false, source: 'none', alerts: [], link: { ...state.link, hz: 0 } }
  }, [state, online])
  const view = useMemo(() => buildView(sceneState, connected), [sceneState, connected])

  // ── 일시정지: 누르면 바로 바꾸고, 서버 값이 따라오면 맞춘다 ──
  const [paused, setPaused] = useState(false)
  const pausedRef = useRef(paused)
  const pauseHold = useRef(0)
  useEffect(() => {
    if (!state || performance.now() < pauseHold.current) return
    if (state.paused !== pausedRef.current) {
      pausedRef.current = state.paused
      setPaused(state.paused)
    }
  }, [state])
  const onPause = useCallback(() => {
    const next = !pausedRef.current
    pausedRef.current = next
    pauseHold.current = performance.now() + 1500
    setPaused(next)
    api.sendControl(next ? 'pause' : 'resume').catch(() => {})
  }, [api])

  // ── 적재 · 주행: 누르면 글자를 바로 바꾸고, 센서 파이가 보낸 모드(state.mode)가 따라오면 맞춘다 ──
  //    화면은 두 모드가 같다. 적재 모드에서는 서버가 경고음 · 주의음만 내지 않는다
  const [driveMode, setDriveMode] = useState<DriveMode | null>(null)
  const driveRef = useRef(driveMode)
  const driveHold = useRef(0)
  useEffect(() => {
    if (!state || performance.now() < driveHold.current) return
    const m = connected ? state.mode ?? null : null
    if (m !== driveRef.current) {
      driveRef.current = m
      setDriveMode(m)
    }
  }, [state, connected])
  const onDriveMode = useCallback(() => {
    if (!driveRef.current) return
    const next: DriveMode = driveRef.current === 'load' ? 'drive' : 'load'
    driveRef.current = next
    driveHold.current = performance.now() + 1500
    setDriveMode(next)
    api.sendMode(next).catch(() => {})
  }, [api])

  // ── 짧은 알림 (처음부터 · 시점) ──
  const [toast, setToast] = useState<string | null>(null)
  const toastTimer = useRef<number | undefined>(undefined)
  const say = useCallback((t: string) => {
    setToast(t)
    window.clearTimeout(toastTimer.current)
    toastTimer.current = window.setTimeout(() => setToast(null), 1600)
  }, [])
  useEffect(() => () => window.clearTimeout(toastTimer.current), [])

  // ── 보기 모드 ──
  const [mode, setMode] = useState<ViewMode>('normal')
  const onMode = useCallback((m: ViewMode) => setMode(m), [])

  // ── 시점 · 고른 항목 ──
  const [preset, setPreset] = useState<Preset>('main')
  const [focus, setFocus] = useState<CheckKey | null>(null)
  const onPreset = useCallback(() => {
    setFocus(null)
    setPreset((p) => {
      const next = PRESETS[(PRESETS.findIndex((x) => x.id === p) + 1) % PRESETS.length]
      say(`시점: ${next.label}`)
      return next.id
    })
  }, [say])
  const onPick = useCallback((k: CheckKey) => setFocus((f) => (f === k ? null : k)), [])
  useEffect(() => {
    if (!focus) return
    const id = window.setTimeout(() => setFocus(null), FOCUS_MS)
    return () => window.clearTimeout(id)
  }, [focus])
  // 시점 단추를 다시 누르면 고른 항목은 풀리고, 고른 항목을 풀면 원래 시점으로
  const focusCheck = focus ? view.checks.find((c) => c.key === focus) ?? null : null

  // ── 3D 영역 크기 (띠와 칸 사이) ──
  const stageRef = useRef<HTMLDivElement>(null)
  const [stage, setStage] = useState({ x: 0, y: 72, w: 800, h: 312 })
  useLayoutEffect(() => {
    const el = stageRef.current
    if (!el) return
    const read = () => {
      const r = el.getBoundingClientRect()
      setStage((s) =>
        s.x === r.left && s.y === r.top && s.w === r.width && s.h === r.height ? s : { x: r.left, y: r.top, w: r.width, h: r.height }
      )
    }
    read()
    const ro = new ResizeObserver(read)
    ro.observe(el)
    return () => ro.disconnect()
  }, [])

  // ── 프레임 속도 보고 (5 s) ──
  useFpsReport(api.reportStats, 'twin', 5000)

  // ── 끝내기 ──
  const [askExit, setAskExit] = useState(false)
  const doExit = useCallback(async () => {
    if (!mock) {
      try {
        await api.exitApp()
      } catch (e) {
        // 서버가 죽어 있으면 브라우저가 스스로 닫는다
        window.close()
        throw e
      }
      window.setTimeout(() => window.close(), 5000)
      return
    }
    // 모의 모드: 닫을 키오스크가 없으므로 잠깐 보여 주고 처음 화면으로
    await api.exitApp()
    await new Promise((r) => window.setTimeout(r, 700))
    setAskExit(false)
    onExited?.()
  }, [api, mock, onExited])

  // 자세히 카드가 왼쪽에 뜨면 3D 는 남은 오른쪽 영역 가운데로
  const showDetail = !!focusCheck && view.connected
  // 3D 는 양쪽 막대 사이에만 (막대 RAIL px + 틈)
  const sceneStage = useMemo(() => {
    const x = stage.x + RAIL_L + (showDetail ? 256 : 0)
    return { x, y: stage.y, w: Math.max(100, stage.x + stage.w - RAIL_R - x), h: stage.h }
  }, [showDetail, stage])
  const level = view.connected ? view.verdict.level : null
  const presetLabel = PRESETS.find((p) => p.id === preset)!.label

  return (
    <div className={`tw-page edge${level ?? 'n'}`}>
      <div className="tw-bg" />
      <Suspense fallback={null}>
        <TwinScene3D state={sceneState} view={view} preset={preset} mode={mode} focus={focus} stage={sceneStage} />
      </Suspense>
      <div className="tw-vignette" />
      <div className="tw-layout">
        <Banner view={view} state={sceneState} paused={paused} />
        <div className="tw-stage" ref={stageRef}>
          {/* 왼쪽 막대: 홈 · 보기 모드 · 색 뜻 · 기울기 (3D 를 가리지 않게 모서리에) */}
          <aside className="tw-rail left">
            <HomeButton onPress={() => setAskExit(true)} />
            <ModeSwitch modes={VIEW_MODES} value={mode} onChange={onMode} />
            <CogMap
              state={sceneState}
              level={view.checks.find((c) => c.key === 'cog')?.level ?? null}
              roll={sceneState?.sensors.roll ?? null}
              pitch={sceneState?.sensors.pitch ?? null}
              dim={!view.connected}
            />
          </aside>
          {/* 오른쪽 막대: 시점 · 일시정지 · 적재/주행 · 음량 */}
          <aside className="tw-rail right">
            <TopControls
              presetLabel={presetLabel}
              paused={paused}
              driveMode={driveMode}
              onPreset={onPreset}
              onPause={onPause}
              onDriveMode={onDriveMode}
            />
            <VolumePill volume={state?.volume} setVolume={api.setVolume} />
          </aside>
          {showDetail && focusCheck && <DetailCard c={focusCheck} judged={view.judged} loads={sceneState?.sensors.loads} onClose={() => setFocus(null)} />}
          {paused && view.connected && <div className="tw-paused">일시정지 중 · ▶ 를 누르면 이어서 재생</div>}
          {toast && <div className="tw-toast">{toast}</div>}
          {!view.connected && <Waiting />}
        </div>
        <CheckTiles view={view} focus={focus} onPick={onPick} />
      </div>
      <ConfirmSheet open={askExit} onCancel={() => setAskExit(false)} onConfirm={doExit} />
    </div>
  )
}
