import { useEffect, useState, type ReactNode } from 'react'
import { frameFor } from '@/lib/display-frame'

/**
 * 트윈 화면을 설계 크기(800 x 480) 비율 그대로 화면에 맞춘다 (lib/display-frame.ts).
 * 모니터에서는 통째로 키우고, 휴대폰에서는 줄인다. 키오스크(800 x 480)에서는 감싸지 않고 그대로 그린다.
 */
export default function DisplayFrame({ children }: { children: ReactNode }) {
  const [vp, setVp] = useState({ w: window.innerWidth, h: window.innerHeight })
  useEffect(() => {
    const on = () => setVp({ w: window.innerWidth, h: window.innerHeight })
    window.addEventListener('resize', on)
    window.addEventListener('orientationchange', on)
    return () => {
      window.removeEventListener('resize', on)
      window.removeEventListener('orientationchange', on)
    }
  }, [])
  const f = frameFor(vp.w, vp.h)
  if (f.s === 1 && f.W === vp.w && f.H === vp.h) return <>{children}</>
  return (
    <div style={{ position: 'fixed', inset: 0, background: '#1a2129', overflow: 'hidden' }}>
      <div
        style={{
          position: 'absolute',
          left: f.left,
          top: f.top,
          width: f.W,
          height: f.H,
          transform: `scale(${f.s})`,
          transformOrigin: '0 0',
          overflow: 'hidden',
        }}
      >
        {children}
      </div>
      {vp.h > vp.w && <div className="tw-rotate-hint">휴대폰을 가로로 돌리면 크게 보입니다</div>}
    </div>
  )
}
