import { useEffect, useRef } from 'react'

/** 화면 프레임 속도를 재서 period 마다 report 로 보낸다 (기본 5 s) */
export function useFpsReport(report: (stats: Record<string, number | string>) => unknown, page = 'twin', period = 5000) {
  const reportRef = useRef(report)
  useEffect(() => {
    reportRef.current = report
  }, [report])

  useEffect(() => {
    let frames = 0
    let raf = 0
    let t0 = performance.now()
    const loop = () => {
      frames += 1
      raf = requestAnimationFrame(loop)
    }
    raf = requestAnimationFrame(loop)
    const iv = window.setInterval(() => {
      const now = performance.now()
      const fps = (frames * 1000) / Math.max(1, now - t0)
      frames = 0
      t0 = now
      // 탭이 숨겨져 있으면 rAF 가 멈추므로 보내지 않는다
      if (document.visibilityState === 'visible') reportRef.current({ page, fps: Math.round(fps * 10) / 10 })
    }, period)
    return () => {
      cancelAnimationFrame(raf)
      window.clearInterval(iv)
    }
  }, [page, period])
}
