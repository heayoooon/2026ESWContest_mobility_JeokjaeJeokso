/**
 * 화면 배율: 디자인은 7인치 키오스크 800 x 480 (px) 기준이다. 모니터 · 노트북 · 휴대폰에서도 같은 모양이 되게
 * 화면을 '설계 크기에 가까운 가상 화면'으로 배치한 뒤 통째로 키우거나 줄인다 (글자 · 칸 · 단추 비율이 모두 같다).
 *
 *  - 가상 화면 높이는 480 으로 두고 폭만 화면 비율을 따른다 (16:9 모니터 -> 853 x 480 을 2.25 배).
 *  - 800 x 480 보다 세로가 긴 화면(4:3, 16:10 등)은 폭 800 에 높이를 늘린다 (최대 4:3 = 800 x 600).
 *  - 그보다 더 길거나 넓은 화면(세로 휴대폰, 초광폭)은 남는 곳을 남색으로 비운다.
 *  - 키오스크(800 x 480)에서는 배율 1 이라 아무것도 바뀌지 않는다.
 */
export const DESIGN_W = 800
export const DESIGN_H = 480
const MAX_ASPECT = 2.4 // 이보다 넓으면 양옆을 비운다
const MIN_ASPECT = 4 / 3 // 이보다 좁으면(세로로 길면) 위아래를 비운다

export type Frame = { W: number; H: number; s: number; left: number; top: number }

export function frameFor(vw: number, vh: number): Frame {
  const a = vw / Math.max(1, vh)
  let W: number
  let H: number
  if (a >= DESIGN_W / DESIGN_H) {
    H = DESIGN_H
    W = Math.round(H * Math.min(a, MAX_ASPECT))
  } else {
    W = DESIGN_W
    H = Math.round(W / Math.max(a, MIN_ASPECT))
  }
  const s = Math.min(vw / W, vh / H)
  return { W, H, s, left: (vw - W * s) / 2, top: (vh - H * s) / 2 }
}

export const currentFrame = () => frameFor(window.innerWidth, window.innerHeight)

/**
 * 3D 를 그리는 해상도 배율. 화면을 키운 만큼 3D 도 촘촘히 그려야 흐리지 않다.
 * 라즈베리파이(ARM)는 무거우니 1.5 배까지, PC 는 2 배까지. 주소에 ?pr=1.25 처럼 주면 그 값.
 */
export function renderPixelRatio(): number {
  const q = Number(new URLSearchParams(location.search).get('pr'))
  if (q > 0) return Math.min(3, Math.max(0.5, q))
  const want = currentFrame().s * (window.devicePixelRatio || 1)
  const arm = /aarch64|armv|arm64/i.test(`${navigator.userAgent} ${navigator.platform}`)
  return Math.min(Math.max(1, want), arm ? 1.5 : 2)
}
