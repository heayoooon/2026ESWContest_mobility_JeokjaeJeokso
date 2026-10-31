import { useCallback, useEffect, useState } from 'react'
import CargoIntro from '@/components/ui/cargo-intro'
import TwinPage, { preloadTwin3D } from '@/components/ui/twin-page'
import { SKIP_INTRO } from '@/hooks/use-twin-source'
import DisplayFrame from '@/components/ui/viewer-frame'

/**
 * 화면 흐름: 인트로 영상(저절로 재생) -> 끝날 즈음 트윈을 아래에 미리 띄움 -> 인트로가 점점 옅어지며 트윈이 드러남.
 * ?skipIntro=1 이면 바로 트윈, ?mock=1 이면 모의 데이터.
 */
type Phase = 'intro' | 'under' | 'blend' | 'twin'

/** 인트로가 옅어지는 시간 (cargo-intro 의 transition 과 같게) */
const BLEND_MS = 1800

export default function App() {
  const [phase, setPhase] = useState<Phase>(SKIP_INTRO ? 'twin' : 'intro')

  // 영상 끝 무렵: 트윈을 인트로 아래에 먼저 올려 둔다 (3D 가 준비될 시간)
  const prepare = useCallback(() => setPhase((p) => (p === 'intro' ? 'under' : p)), [])
  const enter = useCallback(() => setPhase((p) => (p === 'intro' || p === 'under' ? 'blend' : p)), [])
  const backToIntro = useCallback(() => setPhase('intro'), [])

  // 다 옅어지면 인트로를 내린다
  useEffect(() => {
    if (phase !== 'blend') return
    const id = window.setTimeout(() => setPhase('twin'), BLEND_MS + 100)
    return () => window.clearTimeout(id)
  }, [phase])

  // 인트로를 보는 동안 3D 장면 코드를 미리 받아 둔다
  useEffect(() => {
    if (phase !== 'intro') return
    const id = window.setTimeout(preloadTwin3D, 1500)
    return () => window.clearTimeout(id)
  }, [phase])

  return (
    <div
      style={{ position: 'fixed', inset: 0, overflow: 'hidden', background: '#05070d' }}
      // 길게 누르면 뜨는 메뉴 막기 (터치 키오스크)
      onContextMenu={(e) => e.preventDefault()}
    >
      {/* 트윈은 어느 화면에서든 키오스크(800 x 480)와 같은 비율로 (모니터에서는 통째로 키운다) */}
      {phase !== 'intro' && (
        <DisplayFrame>
          <TwinPage onExited={backToIntro} />
        </DisplayFrame>
      )}
      {phase !== 'twin' && <CargoIntro leaving={phase === 'blend'} onPrepare={prepare} onEnter={enter} />}
    </div>
  )
}
