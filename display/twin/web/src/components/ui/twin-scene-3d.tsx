/**
 * 3D 장면 (twin-scene.ts) 을 React 에 붙이는 얇은 껍데기. 페이지가 따로(lazy) 받아 온다.
 * 장면은 한 번만 만들고, 상태 · 시점 · 고른 항목이 바뀌면 손잡이(handle)로 넘긴다.
 */
import { useEffect, useRef, useState } from 'react'
import type { TwinState } from '@/lib/twin-types'
import type { CheckKey, TwinView } from '@/lib/twin-view'
import { initTwinScene, loadFonts, type SceneHandle, type ScenePreset, type ViewMode } from './twin-scene'

export type TwinScene3DProps = {
  state: TwinState | null
  view: TwinView
  preset: ScenePreset
  /** 보기 모드 (기본 · 투시 · 분해 · 센서) */
  mode?: ViewMode
  focus: CheckKey | null
  /** 3D 가 들어갈 화면 영역 (px, 창 기준) */
  stage: { x: number; y: number; w: number; h: number }
  onReady?: () => void
}

export default function TwinScene3D({ state, view, preset, mode = 'normal', focus, stage, onReady }: TwinScene3DProps) {
  const hostRef = useRef<HTMLDivElement>(null)
  const tagRef = useRef<HTMLDivElement>(null)
  const handleRef = useRef<SceneHandle | null>(null)
  const latest = useRef({ state, view, preset, mode, focus, stage, onReady })
  latest.current = { state, view, preset, mode, focus, stage, onReady }
  const [error, setError] = useState<string | null>(null)

  useEffect(() => {
    const host = hostRef.current
    const tags = tagRef.current
    if (!host || !tags) return
    let cancelled = false
    let handle: SceneHandle | null = null
    loadFonts().then(() => {
      if (cancelled) return
      try {
        handle = initTwinScene(host, tags, { onReady: () => latest.current.onReady?.(), onError: setError })
      } catch {
        setError('3D 화면을 시작하지 못했습니다')
        return
      }
      handleRef.current = handle
      const l = latest.current
      handle.setStage(l.stage)
      handle.setPreset(l.preset)
      handle.setMode(l.mode)
      handle.focus(l.focus)
      handle.update(l.state, l.view)
    })
    return () => {
      cancelled = true
      handle?.dispose()
      handleRef.current = null
    }
  }, [])

  useEffect(() => {
    handleRef.current?.update(state, view)
  }, [state, view])
  useEffect(() => {
    handleRef.current?.setPreset(preset)
  }, [preset])
  useEffect(() => {
    handleRef.current?.focus(focus)
  }, [focus])
  useEffect(() => {
    handleRef.current?.setMode(mode)
  }, [mode])
  const { x, y, w, h } = stage
  useEffect(() => {
    handleRef.current?.setStage({ x, y, w, h })
  }, [x, y, w, h])

  return (
    <>
      <div ref={hostRef} className="tw-canvas" role="img" aria-label="화물 적재 3D 트윈. 한 손가락으로 돌리고 두 손가락으로 확대합니다." />
      <div ref={tagRef} className="tw-tags" />
      {error && (
        <div className="tw-error" role="alert">
          <strong>{error}</strong>
          <button type="button" onClick={() => location.reload()}>
            다시 시도
          </button>
        </div>
      )}
    </>
  )
}
