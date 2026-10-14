import { exitApp, reportStats, sendControl, setVolume, useTwinState } from '@/lib/twin-api'
import { mockApi, useMockTwinState } from '@/lib/twin-mock'
import type { TwinState } from '@/lib/twin-types'

/** 주소창 옵션 (?mock=1, ?skipIntro=1 ...) — 페이지를 열 때 한 번만 읽는다 */
const params = new URLSearchParams(location.search)
const flag = (name: string) => ['1', 'true', 'yes', 'on'].includes((params.get(name) ?? '').toLowerCase())

/** 모의 데이터는 개발 서버(npm run dev)에서만. 키오스크용 빌드는 센서 파이 실시간 입력만 쓴다 */
export const IS_MOCK = import.meta.env.DEV && flag('mock')
export const SKIP_INTRO = flag('skipIntro')

export type TwinApi = {
  sendControl: typeof sendControl
  setVolume: typeof setVolume
  exitApp: typeof exitApp
  reportStats: typeof reportStats
}

const realApi: TwinApi = { sendControl, setVolume, exitApp, reportStats }

export type TwinSource = { state: TwinState | null; online: boolean; api: TwinApi; mock: boolean }

function useRealSource(): TwinSource {
  const { state, online } = useTwinState()
  return { state, online, api: realApi, mock: false }
}

function useMockSource(): TwinSource {
  const { state, online } = useMockTwinState()
  return { state, online, api: mockApi as TwinApi, mock: true }
}

/** 실제 백엔드(SSE) 또는 모의 데이터. 페이지 수명 동안 바뀌지 않으므로 훅 규칙에 어긋나지 않는다 */
export const useTwinSource: () => TwinSource = IS_MOCK ? useMockSource : useRealSource
