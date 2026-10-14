import { useEffect, useRef, useState } from 'react'
import type { ControlCmd, TwinState, VolumeAction } from './twin-types'

/** 백엔드 주소. 같은 서버에서 서빙되므로 상대 경로 */
const API = '/api'

/** SSE /api/stream 구독. 끊기면 1 s 뒤 다시 연결한다. 마지막 상태와 연결 여부를 돌려준다. */
export function useTwinState() {
  const [state, setState] = useState<TwinState | null>(null)
  const [online, setOnline] = useState(false)
  const esRef = useRef<EventSource | null>(null)

  useEffect(() => {
    let stopped = false
    let retry: number | undefined
    const open = () => {
      if (stopped) return
      const es = new EventSource(`${API}/stream`)
      esRef.current = es
      es.onopen = () => setOnline(true)
      es.onmessage = (ev) => {
        try {
          setState(JSON.parse(ev.data) as TwinState)
        } catch {
          /* 깨진 메시지는 건너뛴다 */
        }
      }
      es.onerror = () => {
        setOnline(false)
        es.close()
        retry = window.setTimeout(open, 1000)
      }
    }
    open()
    return () => {
      stopped = true
      window.clearTimeout(retry)
      esRef.current?.close()
    }
  }, [])

  return { state, online }
}

async function post(path: string, body: unknown) {
  const r = await fetch(`${API}/${path}`, {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify(body),
  })
  if (!r.ok) throw new Error(`${path}: HTTP ${r.status}`)
  return r.json().catch(() => ({}))
}

/** 일시정지 · 재개 · 처음부터 (보내는 쪽 UDP 5011 까지 전달된다) */
export const sendControl = (cmd: ControlCmd) => post('control', { cmd })

/** 시스템 볼륨 (wpctl). value 는 action='set' 일 때 0..1.5 */
export const setVolume = (action: VolumeAction, value?: number) =>
  post('volume', value === undefined ? { action } : { action, value }) as Promise<{ level: number; muted: boolean }>

/** 키오스크 브라우저를 닫고 바탕화면(홈)으로 */
export const exitApp = () => post('exit', {})

/** 화면 프레임 속도 등을 백엔드 로그로 (진단용) */
export const reportStats = (stats: Record<string, number | string>) => post('stats', stats).catch(() => {})

export const cameraUrl = (seq: number) => `${API}/camera.jpg?seq=${seq}`
