import { useCallback, useEffect, useRef, useState } from 'react'
import MetroHero from '@/components/ui/scroll-locked-video-hero'

/**
 * 첫 화면: 사용자가 준 MetroHero 컴포넌트를 그대로 쓰되, 스크롤 없이 저절로 재생한다.
 *  - 영상: 컴포넌트 기본 영상(원격). 못 읽으면(인터넷 없음 · 오류 · 8 s 안에 첫 프레임 없음) 조용히 /intro/cargo_intro.mp4
 *  - 제목 자리에 SCSS 로고 (public/intro/scss_logo.png, 배경 투명). 끝 문구는 없다
 *  - 영상 끝 무렵 onPrepare (트윈을 아래에 띄움) -> 끝나면 onEnter -> 이 화면이 점점 옅어지며 트윈이 드러난다
 */

const FONT = "'Nanum Gothic', sans-serif"
const LOCAL_VIDEO = '/intro/cargo_intro.mp4'
const REMOTE_TIMEOUT_MS = 8000
/** 영상 전체를 이 시간에 맞춰 재생 */
const PLAY_SECONDS = 9
/** 이 진행도에서 트윈을 아래에 미리 올리고, 이 진행도에서 옅어지기 시작 */
const PREPARE_AT = 0.72
const FADE_AT = 0.96
/** 옅어지는 시간 (App 의 BLEND_MS 와 같게) */
const FADE_MS = 1800

export default function CargoIntro({
  onEnter,
  onPrepare,
  leaving = false,
}: {
  onEnter: () => void
  onPrepare?: () => void
  leaving?: boolean
}) {
  // ── 영상: 원격(기본) -> 실패하면 우리 영상 ──
  const [local, setLocal] = useState(false)
  const readyRef = useRef(false)
  useEffect(() => {
    if (local) return
    const id = window.setTimeout(() => {
      if (!readyRef.current) setLocal(true)
    }, REMOTE_TIMEOUT_MS)
    return () => window.clearTimeout(id)
  }, [local])
  const onVideoReady = useCallback(() => {
    readyRef.current = true
  }, [])
  const onVideoError = useCallback(() => setLocal(true), [])

  // ── 끝 무렵: 트윈 준비 -> 옅어지기 ──
  const prepared = useRef(false)
  const entered = useRef(false)
  const onProgress = useCallback(
    (p: number) => {
      if (p >= PREPARE_AT && !prepared.current) {
        prepared.current = true
        onPrepare?.()
      }
      if (p >= FADE_AT && !entered.current) {
        entered.current = true
        onEnter()
      }
    },
    [onEnter, onPrepare]
  )

  return (
    <div
      style={{
        position: 'fixed',
        inset: 0,
        zIndex: 20,
        background: '#05070d',
        opacity: leaving ? 0 : 1,
        transition: `opacity ${FADE_MS}ms cubic-bezier(.4,0,.2,1)`,
        pointerEvents: leaving ? 'none' : 'auto',
      }}
      // 작가 표시 링크는 남기되 키오스크에서 바깥 사이트로 나가지 않게 막는다
      onClickCapture={(e) => {
        if ((e.target as Element).closest?.('a')) e.preventDefault()
      }}
    >
      <MetroHero
        videoSrc={local ? LOCAL_VIDEO : undefined}
        fontFamily={FONT}
        title={
          <img
            src="/intro/scss_logo.png"
            alt="SCSS"
            draggable={false}
            style={{ display: 'block', width: 'min(72vw, 600px)', height: 'auto', filter: 'drop-shadow(0 4px 24px rgba(0,0,0,.55))' }}
          />
        }
        tagline={null}
        autoPlaySeconds={PLAY_SECONDS}
        onProgress={onProgress}
        onVideoReady={onVideoReady}
        onVideoError={onVideoError}
      />
    </div>
  )
}
