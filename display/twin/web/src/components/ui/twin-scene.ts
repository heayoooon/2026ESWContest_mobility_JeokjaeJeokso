// 화물 적재 디지털 트윈 — three.js 장면 (agentic-factory-3d 의 만듦새: 방 조명 PMREM, ACES, 부드러운 그림자, 접지 그림자, 금속 판)
//
// 좌표 (미터, Y 가 위). X = 오른쪽, Z = 뒤(+Z, 기본 시점 쪽), 앞 = -Z.
//   로드셀 NW=앞 왼쪽(-X,-Z) NE=앞 오른쪽(+X,-Z) SW=뒤 왼쪽(-X,+Z) SE=뒤 오른쪽(+X,+Z). 간격 = sensors.bed_mm (250 x 392 mm)
//   무게중심 cog_mm (앞왼 셀 원점, x 좌->우, y 앞->뒤) -> X = -l/2 + x, Z = -w/2 + y
//   카메라 블록 (월드 x 오른쪽, y 앞, z 위) -> X = x - CAM_CX, Z = -(y - CAM_CY), Y = 판 윗면 + (z - base)
//   카메라와 판 · 로드셀 · 벨트의 위치 관계는 잰 적이 없다 (도식). CAM_CX/CY 는 화물이 판 가운데쯤 오게 고른 값.
import * as THREE from 'three'
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js'
import { RoundedBoxGeometry } from 'three/examples/jsm/geometries/RoundedBoxGeometry.js'
import { RoomEnvironment } from 'three/examples/jsm/environments/RoomEnvironment.js'
import type { TwinBox, TwinState } from '@/lib/twin-types'
import { HIDDEN_ITEMS, type HiddenItem } from '@/lib/cargo-inventory'
import { LEVEL_COLOR, bedOf, type CheckKey, type Level, type TwinView } from '@/lib/twin-view'

export type ScenePreset = 'main' | 'top' | 'side'
/** 보기 모드: 기본 · 투시(상자를 유리처럼, 무게중심 빛줄기) · 분해(층을 위아래로 벌리고 높이) · 센서(모든 값 + 모서리 무게, 상자 흐리게) */
export type ViewMode = 'normal' | 'xray' | 'explode' | 'sensor'
export const MODES: { id: ViewMode; label: string }[] = [
  { id: 'normal', label: '기본' },
  { id: 'xray', label: '투시' },
  { id: 'explode', label: '분해' },
  { id: 'sensor', label: '센서' },
]
export const PRESETS: { id: ScenePreset; label: string }[] = [
  { id: 'main', label: '기본' },
  { id: 'top', label: '위' },
  { id: 'side', label: '옆' },
]

export type SceneHandle = {
  update: (state: TwinState | null, view: TwinView) => void
  setPreset: (p: ScenePreset) => void
  setMode: (m: ViewMode) => void
  focus: (key: CheckKey | null) => void
  /** 화면에서 3D 가 들어갈 영역 (px) */
  setStage: (r: { x: number; y: number; w: number; h: number }) => void
  dispose: () => void
}

export type SceneOptions = { onReady?: () => void; onError?: (msg: string) => void }

const FONT = `'Nanum Gothic', sans-serif`
const CAM_CX = -0.03
const CAM_CY = 0.3
const FOOT_H = 0.05
const PLATE_T = 0.014
const PLATE_TOP = FOOT_H + PLATE_T
const MARGIN = 0.03 // 셀 바깥 판 여유
const STRAP_W = 0.025
const N_STRAP = 120

/** 나눔고딕을 실제로 불러온 뒤 (캔버스 글씨). 인터넷이 없어도 timeoutMs 뒤엔 넘어간다 */
export function loadFonts(timeoutMs = 2500): Promise<void> {
  const fonts = (document as Document & { fonts?: FontFaceSet }).fonts
  if (!fonts) return Promise.resolve()
  const loads = ['400', '700', '800'].map((w) => fonts.load(`${w} 24px "Nanum Gothic"`, '앞뒤 취급주의 0123').catch(() => []))
  return Promise.race([Promise.all(loads).then(() => undefined), new Promise<void>((r) => window.setTimeout(r, timeoutMs))])
}

const col = (hex: string) => new THREE.Color(hex)
const COLORS = {
  ok: col(LEVEL_COLOR[0]),
  warn: col(LEVEL_COLOR[1]),
  alert: col(LEVEL_COLOR[2]),
  bad: col(LEVEL_COLOR[3]),
  calm: col('#cfe9ff'),
  ivory: col('#f4f1ea'),
}
/** 0 정상 초록 · 1 주의 노랑 · 2 경고 주황 · 3 위험 빨강 */
const levelCol = (l: Level) => (l === 3 ? COLORS.bad : l === 2 ? COLORS.alert : l === 1 ? COLORS.warn : COLORS.ok)
const approach = (cur: number, goal: number, dt: number, tau: number) => cur + (goal - cur) * (1 - Math.exp(-dt / tau))

type BlockMesh = {
  group: THREE.Group
  /** 크기만큼 늘린 단위 상자들 (몸통 · 위험 빛 · 테두리) */
  sg: THREE.Group
  body: THREE.Mesh
  /** 위험 빛 (히트맵) 껍질 */
  heat: THREE.Mesh
  edges: THREE.LineSegments
  edgeMat: THREE.LineBasicMaterial
  /** 상자마다 따로 (카메라 색으로 색조) */
  mats: THREE.MeshStandardMaterial[]
  /** 작은 물건 재료 (흰 소포 또는 카메라 색) */
  itemMat: THREE.MeshStandardMaterial
  /** 옆 짐 안의 물병 */
  extra: THREE.Group
  key: string
  thin: boolean
  kind: 'box' | 'side' | 'item'
  tint: THREE.Color
  /** 지금 화면에 보이는 모양 (목표를 향해 따라감) */
  cur: ShapeExtent | null
}
type ShapeExtent = { x0: number; x1: number; z0: number; z1: number; top: number; bottom: number; thin: boolean; kind: 'box' | 'side' | 'item'; color: [number, number, number]; alert: boolean }
type Strap = {
  key: 'c1' | 'c2'
  ribbon: THREE.Mesh
  tail: THREE.Mesh
  mat: THREE.MeshStandardMaterial
  buckle: THREE.Group
  hooks: THREE.Mesh[]
  slack: number
  slackGoal: number
  z: number
  top: THREE.Vector3
  level: Level
}
type Tag = { el: HTMLDivElement; line: SVGLineElement; dot: SVGCircleElement; html: string; on: boolean; w: number; h: number; side: 1 | -1; x: number; y: number; ax: number; ay: number }

/** 이름표 아이콘 (상황마다 다르게). 24x24 선 아이콘 (lucide 모양) */
const ICON_PATH = {
  weight: '<circle cx="12" cy="5" r="3"/><path d="M6.5 8a2 2 0 0 0-1.9 1.46L2.1 18.5A2 2 0 0 0 4 21h16a2 2 0 0 0 1.93-2.54L19.4 9.5A2 2 0 0 0 17.48 8Z"/>',
  target: '<circle cx="12" cy="12" r="9"/><circle cx="12" cy="12" r="3.5"/><path d="M12 1v4M12 19v4M1 12h4M19 12h4"/>',
  move: '<path d="M12 2v20M2 12h20"/><path d="m15 19-3 3-3-3M19 9l3 3-3 3M5 9l-3 3 3 3M9 5l3-3 3 3"/>',
  link: '<path d="M10 13a5 5 0 0 0 7.54.54l3-3a5 5 0 0 0-7.07-7.07l-1.72 1.71"/><path d="M14 11a5 5 0 0 0-7.54-.54l-3 3a5 5 0 0 0 7.07 7.07l1.71-1.71"/>',
  unlink: '<path d="m18.84 12.25 1.72-1.71a5 5 0 0 0-7.07-7.07l-1.72 1.71"/><path d="m5.17 11.75-1.71 1.71a5 5 0 0 0 7.07 7.07l1.71-1.71"/><path d="M8 2v3M2 8h3M16 19v3M19 16h3"/>',
  gauge: '<path d="m12 14 4-4"/><path d="M3.34 19a10 10 0 1 1 17.32 0"/>',
  check: '<path d="M20 6 9 17l-5-5"/>',
  ruler: '<path d="M21.3 15.3a2.4 2.4 0 0 1 0 3.4l-2.6 2.6a2.4 2.4 0 0 1-3.4 0L2.7 8.7a2.41 2.41 0 0 1 0-3.4l2.6-2.6a2.41 2.41 0 0 1 3.4 0Z"/><path d="m14.5 12.5 2-2M11.5 9.5l2-2M8.5 6.5l2-2M17.5 15.5l2-2"/>',
} as const
const ICON_SVG = (k: keyof typeof ICON_PATH) =>
  `<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2.6" stroke-linecap="round" stroke-linejoin="round">${ICON_PATH[k]}</svg>`
function tagIcon(key: CheckKey, word: string) {
  const k =
    key === 'load' ? 'weight'
    : key === 'cog' ? 'target'
    : key === 'move' ? 'move'
    : word === '풀림' ? 'unlink'
    : word === '과조임' || word === '상한 넘음' ? 'gauge'
    : word === '정상' || word === '' ? 'check'
    : 'link'
  return `<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2.6" stroke-linecap="round" stroke-linejoin="round">${ICON_PATH[k]}</svg>`
}

export function initTwinScene(host: HTMLElement, tagLayer: HTMLElement, opts: SceneOptions = {}): SceneHandle {
  const cleanups: (() => void)[] = []
  const disposables: { dispose: () => void }[] = []
  const track = <T extends { dispose: () => void }>(o: T) => {
    disposables.push(o)
    return o
  }

  let renderer: THREE.WebGLRenderer
  try {
    renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true, powerPreference: 'high-performance' })
  } catch (e) {
    opts.onError?.('WebGL 을 쓸 수 없습니다')
    throw e
  }
  renderer.setPixelRatio(1) // 라즈베리파이: 픽셀 비율 1 고정
  renderer.setClearColor(0x000000, 0)
  renderer.outputColorSpace = THREE.SRGBColorSpace
  renderer.toneMapping = THREE.ACESFilmicToneMapping
  renderer.toneMappingExposure = 1.08
  renderer.shadowMap.enabled = true
  renderer.shadowMap.type = THREE.PCFShadowMap
  renderer.shadowMap.autoUpdate = false
  host.appendChild(renderer.domElement)
  renderer.domElement.style.cssText = 'position:absolute;inset:0;width:100%;height:100%;display:block;touch-action:none;outline:none'

  const scene = new THREE.Scene()
  const camera = new THREE.PerspectiveCamera(30, 800 / 480, 0.05, 20)

  // ── 빛: 방 환경 + 위쪽 주광(그림자) + 뒤 테두리광 + 따뜻한 보조광 ──
  const pmrem = new THREE.PMREMGenerator(renderer)
  const room = new RoomEnvironment()
  const env = pmrem.fromScene(room, 0.04)
  scene.environment = env.texture
  scene.environmentIntensity = 0.55
  room.dispose()
  pmrem.dispose()
  track(env.texture)
  scene.add(new THREE.HemisphereLight(0xdbe5f4, 0x221c16, 1.6))
  const key = new THREE.DirectionalLight(0xfff1d8, 3.6)
  key.position.set(-0.9, 2.4, 1.3)
  key.castShadow = true
  key.shadow.mapSize.set(1024, 1024)
  Object.assign(key.shadow.camera, { left: -0.55, right: 0.55, top: 0.55, bottom: -0.55, near: 0.5, far: 5 })
  key.shadow.normalBias = 0.01
  key.shadow.bias = -0.0004
  key.shadow.radius = 4
  scene.add(key)
  const rim = new THREE.DirectionalLight(0xc4d4ed, 2.4)
  rim.position.set(1.2, 1.4, -2)
  scene.add(rim)
  const warm = new THREE.PointLight(0xffc27a, 1.2, 4, 2)
  warm.position.set(-0.8, 0.7, 0.9)
  scene.add(warm)

  // ── 재료 ──
  const std = (color: number, metalness: number, roughness: number, extra: THREE.MeshStandardMaterialParameters = {}) =>
    track(new THREE.MeshStandardMaterial({ color, metalness, roughness, ...extra }))
  const M = {
    plateEdge: std(0x8b949f, 0.9, 0.24),
    plateBody: std(0x272d35, 0.8, 0.34),
    alu: std(0xc8cdd3, 0.92, 0.2),
    aluDark: std(0x5d656e, 0.85, 0.3),
    rubber: std(0x0b0e11, 0.05, 0.78),
    chrome: std(0xdde2e6, 1, 0.13),
    dark: std(0x14181d, 0.4, 0.42),
  }
  const geos = new Map<string, THREE.BufferGeometry>()
  const geo = <T extends THREE.BufferGeometry>(k: string, make: () => T): T => {
    if (!geos.has(k)) geos.set(k, make())
    return geos.get(k) as T
  }
  const rbox = (w: number, h: number, d: number, r: number): THREE.BufferGeometry =>
    geo(`rb${w},${h},${d},${r}`, () =>
      r > 0 ? new RoundedBoxGeometry(w, h, d, 2, Math.min(r, w / 3, h / 3, d / 3)) : new THREE.BoxGeometry(w, h, d)
    )
  const cylG = (r: number, h: number, seg = 28, r2 = r) => geo(`cy${r},${h},${seg},${r2}`, () => new THREE.CylinderGeometry(r, r2, h, seg))
  const mesh = (g: THREE.BufferGeometry, m: THREE.Material | THREE.Material[], parent: THREE.Object3D, x = 0, y = 0, z = 0, shadow = true) => {
    const o = new THREE.Mesh(g, m)
    o.position.set(x, y, z)
    o.castShadow = shadow
    o.receiveShadow = shadow
    parent.add(o)
    return o
  }
  const canvasTex = (w: number, h: number, draw: (c: CanvasRenderingContext2D, w: number, h: number) => void) => {
    const cv = document.createElement('canvas')
    cv.width = w
    cv.height = h
    draw(cv.getContext('2d')!, w, h)
    const t = track(new THREE.CanvasTexture(cv))
    t.colorSpace = THREE.SRGBColorSpace
    t.anisotropy = Math.min(4, renderer.capabilities.getMaxAnisotropy())
    return t
  }

  // ── 바닥: 그림자 받이 + 접지 그림자 + 옅은 제도선 + '앞' 표시 ──
  const floor = mesh(new THREE.PlaneGeometry(6, 6), track(new THREE.ShadowMaterial({ opacity: 0.32 })), scene, 0, 0, 0, false)
  floor.rotation.x = -Math.PI / 2
  floor.receiveShadow = true
  const contactTex = canvasTex(128, 128, (c, w, h) => {
    const g = c.createRadialGradient(w / 2, h / 2, 8, w / 2, h / 2, w / 2)
    g.addColorStop(0, 'rgba(0,0,0,.85)')
    g.addColorStop(0.5, 'rgba(0,0,0,.45)')
    g.addColorStop(1, 'rgba(0,0,0,0)')
    c.fillStyle = g
    c.fillRect(0, 0, w, h)
  })
  const contact = mesh(
    new THREE.PlaneGeometry(0.62, 0.78),
    track(new THREE.MeshBasicMaterial({ map: contactTex, transparent: true, depthWrite: false, opacity: 0.7 })),
    scene,
    0,
    0.0008,
    0,
    false
  )
  contact.rotation.x = -Math.PI / 2
  const decalTex = canvasTex(1024, 1024, (c, w, h) => {
    const cx = w / 2
    const cy = h / 2
    c.strokeStyle = 'rgba(200,210,222,.16)'
    c.lineWidth = 2
    for (const r of [372, 392]) {
      c.beginPath()
      c.ellipse(cx, cy, r * 0.8, r, 0, 0, Math.PI * 2)
      c.stroke()
    }
    for (let i = 0; i < 72; i++) {
      const a = (i / 72) * Math.PI * 2
      const r0 = 392
      const r1 = r0 + (i % 6 === 0 ? 16 : 7)
      c.beginPath()
      c.moveTo(cx + Math.cos(a) * r0 * 0.8, cy + Math.sin(a) * r0)
      c.lineTo(cx + Math.cos(a) * r1 * 0.8, cy + Math.sin(a) * r1)
      c.stroke()
    }
    // 앞 (캔버스 위 = 장면 -Z)
    c.fillStyle = 'rgba(232,236,242,.55)'
    c.beginPath()
    c.moveTo(cx, 58)
    c.lineTo(cx - 22, 92)
    c.lineTo(cx + 22, 92)
    c.closePath()
    c.fill()
    c.font = `800 40px ${FONT}`
    c.textAlign = 'center'
    c.textBaseline = 'middle'
    c.fillText('앞', cx, 128)
    c.fillStyle = 'rgba(232,236,242,.28)'
    c.font = `700 32px ${FONT}`
    c.fillText('뒤', cx, h - 92)
  })
  const decal = mesh(
    new THREE.PlaneGeometry(1.5, 1.5),
    track(new THREE.MeshBasicMaterial({ map: decalTex, transparent: true, depthWrite: false })),
    scene,
    0,
    0.001,
    0,
    false
  )
  decal.rotation.x = -Math.PI / 2

  // ── 적재 판: 항상 수평으로 고정 (판이 기울어 보이면 전복으로 읽힌다. 기울기는 숫자 배지로만) ──
  const tilt = new THREE.Group()
  tilt.position.y = PLATE_TOP
  scene.add(tilt)
  const bed = new THREE.Group()
  bed.position.y = -PLATE_TOP
  tilt.add(bed)

  let L = 0.25
  let W = 0.392
  const plateGroup = new THREE.Group()
  bed.add(plateGroup)
  const treadTex = canvasTex(256, 256, (c, w, h) => {
    c.fillStyle = '#2a3038'
    c.fillRect(0, 0, w, h)
    // 미끄럼 방지 무늬 (마름모 돌기)
    for (let y = 0; y < h; y += 16)
      for (let x = (y / 16) % 2 ? 8 : 0; x < w; x += 16) {
        c.save()
        c.translate(x, y)
        c.rotate(((y / 16) % 2 ? 1 : -1) * 0.7)
        c.fillStyle = 'rgba(255,255,255,.07)'
        c.fillRect(-5, -1.2, 10, 2.4)
        c.fillStyle = 'rgba(0,0,0,.22)'
        c.fillRect(-5, 1.2, 10, 1.2)
        c.restore()
      }
  })
  treadTex.wrapS = treadTex.wrapT = THREE.RepeatWrapping
  const plateTop = std(0xffffff, 0.75, 0.42, { map: treadTex })
  const feet: { group: THREE.Group; arc: THREE.Mesh; arcMat: THREE.MeshBasicMaterial; ring: THREE.Mesh; share: number; goal: number; id: 'NW' | 'NE' | 'SW' | 'SE' }[] = []
  const trackGeo = track(new THREE.RingGeometry(0.03, 0.036, 64, 1))
  const trackMat = track(new THREE.MeshBasicMaterial({ color: 0xffffff, transparent: true, opacity: 0.08, depthWrite: false }))
  for (const id of ['NW', 'NE', 'SW', 'SE'] as const) {
    const g = new THREE.Group()
    bed.add(g)
    // 로드셀 받침: 고무 발 + 알루미늄 몸통 + 윗 볼트
    mesh(cylG(0.022, 0.006), M.rubber, g, 0, 0.003, 0)
    mesh(cylG(0.017, FOOT_H - 0.012, 28), M.alu, g, 0, 0.006 + (FOOT_H - 0.012) / 2, 0)
    mesh(cylG(0.0175, 0.004, 28), M.aluDark, g, 0, FOOT_H * 0.55, 0)
    mesh(cylG(0.009, 0.006, 16), M.chrome, g, 0, FOOT_H - 0.003, 0)
    const ring = mesh(trackGeo, trackMat, g, 0, 0.0015, 0, false)
    ring.rotation.x = -Math.PI / 2
    const arcMat = track(new THREE.MeshBasicMaterial({ color: COLORS.calm.clone(), transparent: true, opacity: 0.85, depthWrite: false, toneMapped: false }))
    // 고리마다 따로 만든다 (drawRange 로 호 길이를 바꾼다)
    const arc = mesh(track(new THREE.RingGeometry(0.03, 0.036, 64, 1, Math.PI / 2, Math.PI * 2)), arcMat, g, 0, 0.002, 0, false)
    arc.rotation.x = -Math.PI / 2
    feet.push({ group: g, arc, arcMat, ring, share: 0.25, goal: 0.25, id })
  }
  const frontMat = track(
    new THREE.MeshBasicMaterial({
      transparent: true,
      depthWrite: false,
      toneMapped: false,
      map: canvasTex(256, 64, (c, w, h) => {
        c.fillStyle = 'rgba(236,240,245,.8)'
        c.beginPath()
        c.moveTo(70, 12)
        c.lineTo(56, 40)
        c.lineTo(84, 40)
        c.closePath()
        c.fill()
        c.fillRect(20, 52, w - 40, 3)
        c.font = `800 34px ${FONT}`
        c.textBaseline = 'middle'
        c.fillText('앞', 104, 28)
        void h
      }),
    })
  )
  let plateMeshes: THREE.Object3D[] = []
  function buildPlate() {
    for (const o of plateMeshes) plateGroup.remove(o)
    plateMeshes = []
    const pw = L + MARGIN * 2 + 0.03
    const pd = W + MARGIN * 2 + 0.03
    const add = (o: THREE.Object3D) => (plateMeshes.push(o), o)
    // 두 겹 판: 아래 밝은 모서리 + 위 무늬 판 (모서리를 깎은 느낌)
    add(mesh(rbox(pw, PLATE_T * 0.55, pd, 0.004), M.plateEdge, plateGroup, 0, FOOT_H + PLATE_T * 0.275, 0))
    const top = mesh(rbox(pw - 0.008, PLATE_T * 0.5, pd - 0.008, 0.003), plateTop, plateGroup, 0, FOOT_H + PLATE_T * 0.75, 0)
    add(top)
    treadTex.repeat.set(pw / 0.09, pd / 0.09)
    add(mesh(rbox(pw - 0.03, 0.006, pd - 0.03, 0.002), M.dark, plateGroup, 0, FOOT_H - 0.002, 0))
    // 판 위 위험 빛 (히트맵)
    const heatPlate = mesh(geo('hp' + pw + ',' + pd, () => new THREE.PlaneGeometry(pw, pd)), heatPlateMat, plateGroup, 0, PLATE_TOP + 0.0009, 0, false)
    heatPlate.rotation.x = -Math.PI / 2
    heatPlate.renderOrder = 2
    add(heatPlate)
    // 판 둘레 빛의 막 4장 (아래가 진하고 위로 갈수록 옅다)
    for (const [w, x, z, ry] of [
      [pw, 0, -pd / 2, 0],
      [pw, 0, pd / 2, Math.PI],
      [pd, -pw / 2, 0, Math.PI / 2],
      [pd, pw / 2, 0, -Math.PI / 2],
    ] as [number, number, number, number][]) {
      const c = mesh(geo('hc' + w, () => new THREE.PlaneGeometry(w, CURTAIN_H)), heatCurtainMat, plateGroup, x, PLATE_TOP + CURTAIN_H / 2, z, false)
      c.rotation.y = ry
      c.renderOrder = 5
      add(c)
    }
    // 셀 자리 볼트
    for (const sx of [-1, 1])
      for (const sz of [-1, 1]) {
        add(mesh(cylG(0.0065, 0.003, 14), M.chrome, plateGroup, (sx * L) / 2, PLATE_TOP + 0.0012, (sz * W) / 2, false))
        add(mesh(cylG(0.0035, 0.0034, 6), M.dark, plateGroup, (sx * L) / 2, PLATE_TOP + 0.0016, (sz * W) / 2, false))
      }
    // 앞쪽 가장자리: '▲ 앞' 표시 (어느 시점에서든 트럭 앞이 어디인지)
    const front = mesh(geo('frontMark', () => new THREE.PlaneGeometry(0.1, 0.025)), frontMat, plateGroup, 0, PLATE_TOP + 0.0006, -pd / 2 + 0.017, false)
    front.rotation.x = -Math.PI / 2
    add(front)
    feet.forEach((f) => f.group.position.set(((f.id.endsWith('E') ? 1 : -1) * L) / 2, 0, ((f.id.startsWith('S') ? 1 : -1) * W) / 2))
    shadowDirty = true
  }

  // ── 상자 (골판지) ──
  const kraft = (kind: 'top' | 'side' | 'end' | 'front' | 'plain') =>
    canvasTex(512, 512, (c, w, h) => {
      const g = c.createLinearGradient(0, 0, 0, h)
      g.addColorStop(0, '#c09466')
      g.addColorStop(1, '#ad8256')
      c.fillStyle = g
      c.fillRect(0, 0, w, h)
      // 섬유 결
      let seed = kind.length * 977
      const rnd = () => ((seed = (seed * 16807) % 2147483647) / 2147483647)
      for (let i = 0; i < 2600; i++) {
        const x = rnd() * w
        const y = rnd() * h
        c.strokeStyle = rnd() > 0.5 ? 'rgba(90,60,30,.10)' : 'rgba(255,235,200,.08)'
        c.lineWidth = 1
        c.beginPath()
        c.moveTo(x, y)
        c.lineTo(x + (rnd() - 0.5) * 10, y + (rnd() - 0.5) * 3)
        c.stroke()
      }
      if (kind !== 'top') {
        c.fillStyle = 'rgba(80,52,25,.035)'
        for (let x = 0; x < w; x += 7) c.fillRect(x, 0, 2, h)
      }
      // 모서리 어둡게
      const e = c.createLinearGradient(0, 0, w, 0)
      e.addColorStop(0, 'rgba(60,38,18,.16)')
      e.addColorStop(0.06, 'rgba(60,38,18,0)')
      e.addColorStop(0.94, 'rgba(60,38,18,0)')
      e.addColorStop(1, 'rgba(60,38,18,.16)')
      c.fillStyle = e
      c.fillRect(0, 0, w, h)
      const tape = (x: number, y: number, tw: number, th: number) => {
        c.fillStyle = 'rgba(222,180,118,.62)'
        c.fillRect(x, y, tw, th)
        c.fillStyle = 'rgba(255,244,222,.22)'
        c.fillRect(x + tw * 0.12, y, tw * 0.1, th)
        c.fillStyle = 'rgba(120,80,40,.18)'
        c.fillRect(x, y, 2, th)
        c.fillRect(x + tw - 2, y, 2, th)
      }
      if (kind === 'top') {
        // 덮개 맞닿는 선 + 테이프
        c.fillStyle = 'rgba(70,45,20,.35)'
        c.fillRect(w / 2 - 1, 0, 2, h)
        tape(w / 2 - 46, 0, 92, h)
      }
      if (kind === 'end') {
        tape(w / 2 - 46, 0, 92, h * 0.3)
        // 운송장 (흰 종이 + 바코드)
        c.fillStyle = 'rgba(244,242,236,.95)'
        c.fillRect(w * 0.58, h * 0.52, w * 0.32, h * 0.3)
        c.fillStyle = 'rgba(30,30,30,.85)'
        let x = w * 0.6
        while (x < w * 0.87) {
          const bw = 1 + Math.floor(rnd() * 4)
          c.fillRect(x, h * 0.66, bw, h * 0.12)
          x += bw + 1 + Math.floor(rnd() * 3)
        }
        c.fillRect(w * 0.6, h * 0.56, w * 0.18, 5)
        c.fillRect(w * 0.6, h * 0.6, w * 0.24, 4)
      }
      if (kind === 'front') tape(w / 2 - 46, 0, 92, h * 0.3)
      if (kind === 'side' || kind === 'end' || kind === 'front') {
        // 이쪽 위로 화살표 + 취급주의
        c.strokeStyle = 'rgba(52,34,18,.72)'
        c.fillStyle = 'rgba(52,34,18,.72)'
        c.lineWidth = 6
        const ax = kind === 'side' ? w * 0.16 : w * 0.14
        for (const dx of [0, 34]) {
          c.beginPath()
          c.moveTo(ax + dx, h * 0.66)
          c.lineTo(ax + dx, h * 0.5)
          c.stroke()
          c.beginPath()
          c.moveTo(ax + dx - 12, h * 0.52)
          c.lineTo(ax + dx, h * 0.46)
          c.lineTo(ax + dx + 12, h * 0.52)
          c.closePath()
          c.fill()
        }
        c.fillRect(ax - 16, h * 0.69, 66, 5)
        if (kind === 'side') {
          c.font = `800 40px ${FONT}`
          c.textBaseline = 'middle'
          c.fillText('취급주의', w * 0.42, h * 0.58)
          c.font = `700 22px ${FONT}`
          c.fillStyle = 'rgba(52,34,18,.5)'
          c.fillText('CARGO 01 · 15 kg MAX', w * 0.42, h * 0.66)
        }
      }
    })
  const texTop = kraft('top')
  const texSide = kraft('side')
  const texEnd = kraft('end')
  const texFront = kraft('front')
  const texPlain = kraft('plain')
  const boxMat = (t: THREE.Texture) => std(0xffffff, 0, 0.86, { map: t })
  const boxTex = [texSide, texSide, texTop, texPlain, texEnd, texFront]
  const flapMat = boxMat(texPlain)
  // 옆 짐: 에어캡(뽁뽁이)으로 싼 물병 묶음. 반투명 포장 + 안에 누운 병
  const bubbleTex = canvasTex(256, 256, (c, w, h) => {
    c.fillStyle = 'rgba(232,240,247,.5)'
    c.fillRect(0, 0, w, h)
    for (let y = 0; y < h + 22; y += 22)
      for (let x = (y / 22) % 2 ? 11 : 0; x < w + 22; x += 22) {
        const g = c.createRadialGradient(x - 3, y - 3, 1, x, y, 9)
        g.addColorStop(0, 'rgba(255,255,255,.95)')
        g.addColorStop(0.6, 'rgba(220,232,242,.32)')
        g.addColorStop(1, 'rgba(150,172,195,.55)')
        c.fillStyle = g
        c.beginPath()
        c.arc(x, y, 8.5, 0, Math.PI * 2)
        c.fill()
      }
  })
  bubbleTex.wrapS = bubbleTex.wrapT = THREE.RepeatWrapping
  bubbleTex.repeat.set(2.5, 2.5)
  const wrapMat = std(0xffffff, 0, 0.2, { map: bubbleTex, transparent: true, opacity: 0.6, depthWrite: false, side: THREE.DoubleSide })
  const bottleMat = std(0xa8d8ff, 0.05, 0.1, { transparent: true, opacity: 0.78, emissive: 0x0a2034 })
  const capMat = std(0x2f7de1, 0.15, 0.4)

  // ── 위험 빛 (히트맵): 위험 = 빨강, 주의 = 노랑, 안전 = 초록. 상자 · 받침 판 위에 은은하게 비춘다 ──
  // 위험 근원(벨트 줄, 쏠린 쪽, 밀린 상자 ...)마다 반경 안에서 단계(0..2)가 퍼지고, 겹치면 큰 값. 적재(과적)는 화물 전체 바탕값.
  // 판은 고정이라 판 좌표 = 월드 좌표.
  const N_SRC = 6
  const heatU = {
    uSrc: { value: Array.from({ length: N_SRC }, () => new THREE.Vector4(0, -9, 0, 0.01)) },
    uMask: { value: Array.from({ length: N_SRC }, () => new THREE.Vector3(1, 1, 1)) },
    uLv: { value: new Array<number>(N_SRC).fill(0) },
    uBase: { value: 0 },
    uTime: { value: 0 },
    uOn: { value: 0 },
    /** 적재판 네 구역(로드셀 자리)의 단계 0..3: x = 앞왼(NW) y = 앞오(NE) z = 뒤왼(SW) w = 뒤오(SE) */
    uQuad: { value: new THREE.Vector4(0, 0, 0, 0) },
  }
  const HEAT_VS = /* glsl */ `
    varying vec3 vW; varying vec3 vN; varying vec2 vUv;
    void main() {
      vec4 w = modelMatrix * vec4(position, 1.0);
      vW = w.xyz; vN = normalize(mat3(modelMatrix) * normal); vUv = uv;
      gl_Position = projectionMatrix * viewMatrix * w;
    }`
  type HeatMode = 'box' | 'plate' | 'curtain'
  const heatFS = (mode: HeatMode) => /* glsl */ `
    uniform vec4 uSrc[${N_SRC}]; uniform vec3 uMask[${N_SRC}]; uniform float uLv[${N_SRC}];
    uniform float uBase; uniform float uTime; uniform float uOn; uniform vec4 uQuad;
    varying vec3 vW; varying vec3 vN; varying vec2 vUv;
    float hash(vec3 p) { p = fract(p * 0.3183099 + 0.1); p *= 17.0; return fract(p.x * p.y * p.z * (p.x + p.y + p.z)); }
    float noise(vec3 x) {
      vec3 i = floor(x); vec3 f = fract(x); f = f * f * (3.0 - 2.0 * f);
      return mix(mix(mix(hash(i), hash(i + vec3(1, 0, 0)), f.x), mix(hash(i + vec3(0, 1, 0)), hash(i + vec3(1, 1, 0)), f.x), f.y),
                 mix(mix(hash(i + vec3(0, 0, 1)), hash(i + vec3(1, 0, 1)), f.x), mix(hash(i + vec3(0, 1, 1)), hash(i + vec3(1, 1, 1)), f.x), f.y), f.z);
    }
    void main() {
${
        mode === 'box'
          ? `float r = uBase;
      for (int i = 0; i < ${N_SRC}; i++) {
        float d = length((vW - uSrc[i].xyz) * uMask[i]);
        r = max(r, uLv[i] * (1.0 - smoothstep(uSrc[i].w * 0.3, uSrc[i].w, d)));
      }
      // 들판 히트맵처럼 고르지 않은 결
      r = clamp(r + (noise(vW * 42.0) - 0.5) * 0.26 + (noise(vW * 11.0 + uTime * 0.15) - 0.5) * 0.18, 0.0, 3.0);`
          : // 판 · 빛의 막: 네 구역(앞왼 · 앞오 · 뒤왼 · 뒤오)마다 따로. 경계는 1.5 cm 로 살짝만 섞는다
            `float qx = smoothstep(-0.015, 0.015, vW.x);
      float qz = smoothstep(-0.015, 0.015, vW.z);
      float r = mix(mix(uQuad.x, uQuad.y, qx), mix(uQuad.z, uQuad.w, qx), qz);
      r = clamp(r + (noise(vW * 30.0 + uTime * 0.1) - 0.5) * 0.12, 0.0, 3.0);`
      }
      // 0 정상 초록 · 1 주의 노랑 · 2 경고 주황 · 3 위험 빨강
      vec3 green = vec3(0.16, 0.88, 0.42), yellow = vec3(1.0, 0.84, 0.18), orange = vec3(1.0, 0.52, 0.16), red = vec3(1.0, 0.2, 0.18);
      vec3 col = r < 1.0 ? mix(green, yellow, smoothstep(0.0, 1.0, r)) : r < 2.0 ? mix(yellow, orange, smoothstep(1.0, 2.0, r)) : mix(orange, red, smoothstep(2.0, 2.9, r));
      float hot = smoothstep(2.25, 2.9, r);
      col *= 1.0 + hot * 0.35 * sin(uTime * 6.2832 * 1.1);
      ${
        mode === 'plate'
          ? // 판: 진하게 + 가장자리는 형광펜처럼 밝은 띠
            `float e = min(min(vUv.x, 1.0 - vUv.x), min(vUv.y, 1.0 - vUv.y));
             float rim = 1.0 - smoothstep(0.0, 0.035, e);
             // 구역 나누는 선 (판 가운데 십자)
             float cross = 1.0 - smoothstep(0.0015, 0.004, min(abs(vW.x), abs(vW.z)));
             rim = max(rim, cross * 0.55);
             col = mix(col, vec3(1.0), rim * 0.35);
             float a = (0.66 + 0.2 * hot) * smoothstep(0.0, 0.05, e) + 0.95 * rim;`
          : mode === 'curtain'
          ? // 판 둘레에서 위로 번지는 빛의 막 (위에서 봐도 색이 보이게). vUv.y 0 = 판, 1 = 위 끝
            `float a = pow(1.0 - vUv.y, 1.7) * (0.5 + 0.25 * hot);
             col *= 1.25;`
          : `float a = 0.36 + 0.12 * hot + 0.1 * smoothstep(0.75, 1.0, abs(vN.y));`
      }
      gl_FragColor = vec4(col, a * uOn);
    }`
  const heatMat = (mode: HeatMode) =>
    track(
      new THREE.ShaderMaterial({
        uniforms: heatU,
        vertexShader: HEAT_VS,
        fragmentShader: heatFS(mode),
        transparent: true,
        side: mode === 'curtain' ? THREE.DoubleSide : THREE.FrontSide,
        blending: mode === 'curtain' ? THREE.AdditiveBlending : THREE.NormalBlending,
        depthWrite: false,
        toneMapped: false,
        polygonOffset: true,
        polygonOffsetFactor: -2,
        polygonOffsetUnits: -2,
      })
    )
  const heatBoxMat = heatMat('box')
  const heatPlateMat = heatMat('plate')
  const heatCurtainMat = heatMat('curtain')
  const CURTAIN_H = 0.13
  /** 셰이더와 같은 값 (결 없이) — 테두리 · 이름표 색 */
  function riskAt(x: number, y: number, z: number) {
    let r = heatU.uBase.value
    for (let i = 0; i < N_SRC; i++) {
      const s = heatU.uSrc.value[i]
      const m = heatU.uMask.value[i]
      const d = Math.hypot((x - s.x) * m.x, (y - s.y) * m.y, (z - s.z) * m.z)
      const k = 1 - THREE.MathUtils.smoothstep(d, s.w * 0.3, s.w)
      r = Math.max(r, heatU.uLv.value[i] * k)
    }
    return r
  }

  const cargo = new THREE.Group()
  bed.add(cargo)
  const blocks: BlockMesh[] = []
  type Extent = ShapeExtent
  let extents: Extent[] = []
  let cargoBox: { x0: number; x1: number; z0: number; z1: number; top: number } | null = null
  let sceneR = 0.3
  let lastBoxes: TwinBox[] = []
  let boxesSeenAt = 0

  /** 카메라 색 -> 골판지 색조 (흰 상자는 밝게, 갈색 상자는 진하게) */
  function kraftTint(c: [number, number, number], out: THREE.Color) {
    const m = Math.max(0.05, c[0], c[1], c[2])
    out.setRGB(c[0] / m, c[1] / m, c[2] / m, THREE.SRGBColorSpace).lerp(COLORS.ivory, 0.3)
    const lum = 0.299 * c[0] + 0.587 * c[1] + 0.114 * c[2]
    return out.multiplyScalar(THREE.MathUtils.clamp(0.55 + lum * 1.4, 0.72, 1.1))
  }

  function buildBottles(b: BlockMesh, w: number, h: number, d: number) {
    for (const o of [...b.extra.children]) b.extra.remove(o)
    const along = d >= w ? 'z' : 'x'
    const len = (along === 'z' ? d : w) * 0.9
    const across = along === 'z' ? w : d
    const dia = Math.min(h * 0.86, across * 0.9, 0.075)
    const nx = Math.max(1, Math.floor(across / (dia * 1.04)))
    const ny = Math.max(1, Math.floor(h / (dia * 1.04)))
    const r = +(dia / 2).toFixed(3)
    for (let i = 0; i < nx; i++)
      for (let j = 0; j < ny; j++) {
        const u = (i - (nx - 1) / 2) * dia * 1.04
        const v = -h / 2 + dia / 2 + j * dia * 1.02 + 0.002
        const bottle = mesh(cylG(r, +(len - r * 1.6).toFixed(3), 20), bottleMat, b.extra, 0, 0, 0, true)
        const neck = mesh(cylG(r * 0.42, r * 0.9, 14, r * 0.9), bottleMat, b.extra, 0, 0, 0, false)
        const cap = mesh(cylG(r * 0.45, r * 0.45, 14), capMat, b.extra, 0, 0, 0, false)
        const end = len / 2 - r * 0.8
        for (const [o, off] of [
          [bottle, 0],
          [neck, end + r * 0.45],
          [cap, end + r * 0.9 + r * 0.22],
        ] as [THREE.Mesh, number][]) {
          if (along === 'z') {
            o.rotation.x = Math.PI / 2
            o.position.set(u, v, off)
          } else {
            o.rotation.z = Math.PI / 2
            o.position.set(off, v, u)
          }
        }
      }
  }

  /** 카메라 -> 판 위치 맞춤 (서버가 로드셀 무게중심으로 낸 값). 없으면 옛 고정값 */
  let reg = { cx: CAM_CX, cy: CAM_CY }
  /** 서버에서 온 목표 모양 (판 좌표). 화면은 이걸 향해 부드럽게 움직인다 */
  let targets: Extent[] = []
  const unitBox = track(new THREE.BoxGeometry(1, 1, 1))
  const unitEdges = track(new THREE.EdgesGeometry(unitBox))
  const KIND_ORDER = { box: 0, side: 1, item: 2 } as const
  const SHAPE_TAU = 0.22 // 초. 모양이 바뀌면 이 시정수로 따라간다 (튀지 않고, 실제 변화는 0.5 초 안에 거의 다 따라감)

  function setBlocks(boxes: TwinBox[], base: number) {
    // 블록 -> 판 좌표 (카메라 x 오른쪽, y 앞 -> 판 X, Z(뒤 +))
    const ex: Extent[] = boxes.map((b) => ({
      x0: b.x0 - reg.cx,
      x1: b.x1 - reg.cx,
      z0: -(b.y1 - reg.cy),
      z1: -(b.y0 - reg.cy),
      bottom: PLATE_TOP + Math.max(0, b.bottom - base),
      top: PLATE_TOP + Math.max(0.005, b.top - base),
      thin: (b.kind ?? 'box') === 'box' && b.top - b.bottom < 0.06,
      kind: b.kind === 'side' || b.kind === 'item' ? b.kind : 'box',
      color: b.color,
      alert: b.alert,
    }))
    // 큰 상자 위에 얹힌 얇은 덮개는 상자 윗면에 올린다 (겹쳐 깜빡이지 않게)
    const main = ex.filter((e) => !e.thin && e.kind === 'box')
    for (const e of ex) {
      if (!e.thin) continue
      const under = main.filter((m) => m.x0 < e.x1 && m.x1 > e.x0 && m.z0 < e.z1 && m.z1 > e.z0)
      const t = Math.max(PLATE_TOP, ...under.map((m) => m.top))
      e.bottom = Math.max(e.bottom, t + 0.0005)
      e.top = Math.max(e.top, e.bottom + 0.006)
    }
    // 블록 자리는 '종류 · 아래부터' 순서로 고정 -> 같은 상자는 늘 같은 블록 (서버 순서가 바뀌어도 다른 상자로 바뀌어 보이지 않게)
    ex.sort((a, b) => KIND_ORDER[a.kind] - KIND_ORDER[b.kind] || Number(a.thin) - Number(b.thin) || a.bottom - b.bottom || a.x0 - b.x0)
    targets = ex
    while (blocks.length < ex.length) {
      const group = new THREE.Group()
      cargo.add(group)
      const sg = new THREE.Group()
      group.add(sg)
      const mats = boxTex.map((t) => boxMat(t))
      const body = mesh(unitBox, mats, sg)
      const heat = mesh(unitBox, heatBoxMat, sg, 0, 0, 0, false)
      heat.renderOrder = 3
      const edgeMat = track(new THREE.LineBasicMaterial({ color: 0xffffff, transparent: true, opacity: 0.3, toneMapped: false }))
      const edges = new THREE.LineSegments(unitEdges, edgeMat)
      edges.renderOrder = 4
      sg.add(edges)
      const extra = new THREE.Group()
      group.add(extra)
      const itemMat = std(0xeceae4, 0, 0.62)
      blocks.push({ group, sg, body, heat, edges, edgeMat, mats, itemMat, extra, key: '', thin: false, kind: 'box', tint: new THREE.Color(1, 1, 1), cur: null })
    }
    if (mode !== 'normal') applyModeMaterials()
    blocks.forEach((b, i) => {
      const e = ex[i]
      if (!e) {
        b.group.visible = false
        b.cur = null
        return
      }
      if (b.kind !== e.kind || b.thin !== e.thin || !b.cur) {
        b.body.material = e.thin ? flapMat : e.kind === 'side' ? wrapMat : e.kind === 'item' ? b.itemMat : b.mats
        b.body.castShadow = e.kind !== 'side'
        b.thin = e.thin
        b.kind = e.kind
        b.key = ''
        if (e.kind !== 'side') for (const o of [...b.extra.children]) b.extra.remove(o)
      }
      if (!b.cur) b.cur = { ...e } // 새로 나타난 상자는 바로 그 자리
      b.cur.color = e.color
      b.cur.alert = e.alert
      b.heat.visible = !e.thin
      if (e.kind === 'item') {
        // 작은 물건: 카메라 색이 있으면 그 색, 없으면(회색 0.47) 흰 소포
        const grey = Math.abs(e.color[0] - e.color[1]) < 0.02 && Math.abs(e.color[1] - e.color[2]) < 0.02 && Math.abs(e.color[0] - 0.47) < 0.03
        if (grey) b.itemMat.color.set(0xeceae4)
        else b.itemMat.color.setRGB(e.color[0], e.color[1], e.color[2], THREE.SRGBColorSpace).lerp(COLORS.ivory, 0.35)
      }
      if (!e.thin && e.kind === 'box') {
        kraftTint(e.color, b.tint)
        b.mats.forEach((m) => m.color.copy(b.tint))
        if (!ex.slice(0, i).some((q) => !q.thin && q.kind === 'box')) flapMat.color.copy(b.tint)
      }
      b.group.visible = true
    })
    animateBlocks(1) // 모양 목록이 바뀐 직후 한 번 맞춰 둔다 (k=1 이면 바로)
  }

  /** 매 프레임: 보이는 모양을 목표로 부드럽게. dt<0 이 아니라 k(0..1) 를 받는다 */
  const shapeKeys = ['x0', 'x1', 'z0', 'z1', 'bottom', 'top'] as const
  function animateBlocks(k: number) {
    const shown: Extent[] = []
    // 분해 순서: 쌓인 상자를 아래부터 0, 1, ... (얹힌 덮개는 맨 위 다음)
    const rank = new Map<BlockMesh, number>()
    if (explodeT > 0.001) {
      const order = blocks.filter((b, i) => targets[i] && b.cur && b.kind === 'box').sort((a, b2) => Number(a.thin) - Number(b2.thin) || a.cur!.bottom - b2.cur!.bottom)
      order.forEach((b, i) => rank.set(b, i))
    }
    let moved = false
    blocks.forEach((b, i) => {
      const e = targets[i]
      if (!e || !b.cur) return
      const c = b.cur
      for (const key of shapeKeys) {
        const d = e[key] - c[key]
        if (Math.abs(d) > 2e-4) {
          c[key] += d * k
          moved = true
        } else c[key] = e[key]
      }
      const w = Math.max(0.01, c.x1 - c.x0)
      const h = Math.max(0.004, c.top - c.bottom)
      const dd = Math.max(0.01, c.z1 - c.z0)
      b.group.position.set((c.x0 + c.x1) / 2, (c.bottom + c.top) / 2 + explodeT * EXPLODE_GAP * (rank.get(b) ?? 0), (c.z0 + c.z1) / 2)
      b.sg.scale.set(w, h, dd)
      b.heat.scale.set((w + 0.005) / w, (h + 0.005) / h, (dd + 0.005) / dd)
      b.edges.scale.set((w + 0.003) / w, (h + 0.003) / h, (dd + 0.003) / dd)
      if (b.kind === 'side') {
        // 물병은 1 cm 단위로 바뀔 때만 다시 놓는다
        const key = `${Math.round(w * 100)},${Math.round(h * 100)},${Math.round(dd * 100)}`
        if (key !== b.key) {
          b.key = key
          buildBottles(b, w, h, dd)
        }
      }
      shown.push(c)
    })
    if (moved) shadowDirty = true
    extents = shown
    if (shown.length) {
      cargoBox = {
        x0: Math.min(...shown.map((e) => e.x0)),
        x1: Math.max(...shown.map((e) => e.x1)),
        z0: Math.min(...shown.map((e) => e.z0)),
        z1: Math.max(...shown.map((e) => e.z1)),
        top: Math.max(...shown.map((e) => e.top)),
      }
    } else cargoBox = null
    const R = THREE.MathUtils.clamp(Math.hypot(0.22, ((cargoBox ? cargoBox.top : PLATE_TOP + 0.2) - PLATE_TOP) / 2 + 0.03), 0.3, 0.46)
    placeGhosts()
    if (Math.abs(R - sceneR) > 0.015) {
      sceneR = R
      if (sized) layout()
    }
  }

  // ── 카메라에 다 안 보이는 작은 화물 ──
  // 크기는 2026-10-03 녹화에서 잰 값(cargo-inventory.ts). 10-04 깊이에 앞면 일부가 잡힌 것은 그 자리(cam 좌표)에,
  // 나머지는 판 위 '카메라 사각지대' = 쌓인 화물 너머(-Z, 앞쪽) 빈 곳에 왼쪽부터 놓고 평평한 것은 큰 것 위에 쌓는다.
  // 진짜 물건처럼 그린다 (불투명 · 그림자 · 겉 무늬). 화물이 1 cm 넘게 바뀔 때만 다시 놓는다 (흔들리지 않게).
  const ghostGroup = new THREE.Group()
  bed.add(ghostGroup)
  const itemTex = {
    rpi: canvasTex(256, 256, (c, w, h) => {
      c.fillStyle = '#f4f5f6'
      c.fillRect(0, 0, w, h)
      c.fillStyle = '#c51a4a' // 라즈베리 색 로고
      c.beginPath()
      c.arc(w * 0.3, h * 0.42, 26, 0, Math.PI * 2)
      c.fill()
      c.fillStyle = '#2a8f3c'
      c.fillRect(w * 0.3 - 14, h * 0.42 - 40, 28, 10)
      c.fillStyle = '#2b2b2b'
      c.font = `800 26px ${FONT}`
      c.fillText('Raspberry Pi', w * 0.08, h * 0.78)
      c.fillStyle = 'rgba(0,0,0,.08)'
      c.fillRect(0, h - 10, w, 10)
    }),
    window: canvasTex(256, 256, (c, w, h) => {
      c.fillStyle = '#f2f4f6'
      c.fillRect(0, 0, w, h)
      c.fillStyle = 'rgba(160,200,225,.55)' // 투명 창
      c.fillRect(w * 0.18, h * 0.2, w * 0.64, h * 0.5)
      c.fillStyle = '#2f7de1'
      c.beginPath()
      c.arc(w * 0.5, h * 0.45, 22, 0, Math.PI * 2)
      c.fill()
      c.fillStyle = '#333'
      c.fillRect(w * 0.2, h * 0.8, w * 0.6, 8)
    }),
    thin: canvasTex(256, 256, (c, w, h) => {
      c.fillStyle = '#eceff2'
      c.fillRect(0, 0, w, h)
      c.fillStyle = '#ffd400' // 가격표
      c.fillRect(w * 0.08, h * 0.62, w * 0.22, h * 0.2)
      c.fillStyle = '#222'
      c.font = `800 22px ${FONT}`
      c.fillText('5,000', w * 0.34, h * 0.78)
      c.fillRect(w * 0.08, h * 0.2, w * 0.7, 6)
    }),
    base: canvasTex(128, 128, (c, w, h) => {
      c.fillStyle = '#cdb68e'
      c.fillRect(0, 0, w, h)
      c.fillStyle = 'rgba(90,60,30,.12)'
      for (let x = 0; x < w; x += 6) c.fillRect(x, 0, 2, h)
    }),
  }
  const itemMat = {
    rpi: std(0xffffff, 0, 0.55, { map: itemTex.rpi }),
    window: std(0xffffff, 0.02, 0.4, { map: itemTex.window }),
    thin: std(0xffffff, 0, 0.55, { map: itemTex.thin }),
    base: std(0xffffff, 0, 0.8, { map: itemTex.base }),
  }
  let ghostKey = ''
  /** 놓인 작은 화물 (판 좌표) */
  let ghosts: { item: HiddenItem; x0: number; x1: number; z0: number; z1: number; bottom: number; top: number }[] = []
  function placeGhosts() {
    const stack = extents.filter((e) => !e.thin)
    const hx = (L + MARGIN * 2 + 0.03) / 2 - 0.006
    const hz = (W + MARGIN * 2 + 0.03) / 2 - 0.006
    const zFront = stack.length ? Math.min(...stack.map((e) => e.z0)) - 0.006 : -hz
    const key = stack.length ? `${Math.round(zFront * 100)},${Math.round(hx * 100)},${Math.round(hz * 100)},${Math.round(reg.cx * 100)},${Math.round(reg.cy * 100)}` : 'none'
    if (key === ghostKey) return
    ghostKey = key
    for (const o of [...ghostGroup.children]) ghostGroup.remove(o)
    ghosts = []
    if (!stack.length) return
    const depth = zFront - -hz
    type Pile = { x0: number; x1: number; z0: number; z1: number; top: number }
    const piles: Pile[] = []
    let cursor = -hx
    const taken: { x0: number; x1: number; z0: number; z1: number }[] = []
    const free = (x0: number, x1: number, z0: number, z1: number) =>
      ![...extents, ...taken].some((e) => e.x0 < x1 && e.x1 > x0 && e.z0 < z1 && e.z1 > z0)
    // 깊이에 잡힌 자리가 있는 것부터
    const items = [...HIDDEN_ITEMS].sort((a, b) => Number(!!b.cam) - Number(!!a.cam) || b.w * b.d - a.w * a.d)
    for (const it of items) {
      let placed: { x0: number; x1: number; z0: number; z1: number; bottom: number } | null = null
      if (it.cam) {
        // 10-04 카메라 좌표 -> 판 좌표 (상자와 같은 맞춤)
        placed = { x0: it.cam.x0 - reg.cx, x1: it.cam.x1 - reg.cx, z0: -(it.cam.y1 - reg.cy), z1: -(it.cam.y0 - reg.cy), bottom: PLATE_TOP }
        taken.push(placed)
      }
      // 1) 판 위 새 자리 (화물에 붙여서, 왼쪽부터). 두 방향 다 해 본다
      for (const [w, d] of [
        [it.w, it.d],
        [it.d, it.w],
      ]) {
        if (placed || d > depth || cursor + w > hx) continue
        const p = { x0: cursor, x1: cursor + w, z0: zFront - d, z1: zFront }
        if (!free(p.x0, p.x1, p.z0, p.z1)) continue
        placed = { ...p, bottom: PLATE_TOP }
        piles.push({ ...p, top: PLATE_TOP + it.h })
        cursor += w + 0.006
      }
      // 2) 자리가 없으면 발판이 넉넉한(2 cm 까지 삐져나와도 됨) 더미 중 가장 낮은 것 위에
      if (!placed) {
        const fits = piles
          .filter((p) => p.x1 - p.x0 + 0.02 >= Math.min(it.w, it.d) && p.z1 - p.z0 + 0.02 >= Math.min(it.w, it.d) && Math.max(it.w, it.d) <= Math.max(p.x1 - p.x0, p.z1 - p.z0) + 0.02)
          .sort((a, b) => a.top - b.top)
        const p = fits[0]
        if (p) {
          const wide = p.x1 - p.x0 >= p.z1 - p.z0
          const [w, d] = wide === it.w >= it.d ? [it.w, it.d] : [it.d, it.w]
          const cx = (p.x0 + p.x1) / 2
          const cz = (p.z0 + p.z1) / 2
          placed = { x0: cx - w / 2, x1: cx + w / 2, z0: cz - d / 2, z1: cz + d / 2, bottom: p.top }
          p.top += it.h
        }
      }
      if (!placed) continue // 놓을 데가 없으면 그리지 않는다
      const g = { item: it, ...placed, top: placed.bottom + it.h }
      ghosts.push(g)
      const w = g.x1 - g.x0
      const d = g.z1 - g.z0
      const cx = (g.x0 + g.x1) / 2
      const cz = (g.z0 + g.z1) / 2
      if (it.shape === 'bottle') {
        const along = d >= w
        const r = Math.min(it.h, along ? w : d) / 2
        const len = Math.max(w, d)
        const body = mesh(cylG(r, len * 0.78, 20), bottleMat, ghostGroup, cx, g.bottom + r, cz, true)
        const neck = mesh(cylG(r * 0.45, len * 0.1, 14, r * 0.9), bottleMat, ghostGroup, cx, g.bottom + r, cz, false)
        const cap = mesh(cylG(r * 0.47, len * 0.1, 14), capMat, ghostGroup, cx, g.bottom + r, cz, true)
        if (along) {
          body.rotation.x = neck.rotation.x = cap.rotation.x = -Math.PI / 2
          neck.position.z = cz - len * 0.43
          cap.position.z = cz - len * 0.5
        } else {
          body.rotation.z = neck.rotation.z = cap.rotation.z = Math.PI / 2
          neck.position.x = cx - len * 0.43
          cap.position.x = cx - len * 0.5
        }
      } else {
        const mat = itemMat[it.id as keyof typeof itemMat] ?? itemMat.base
        const m = mesh(unitBox, mat, ghostGroup, cx, (g.bottom + g.top) / 2, cz, true)
        m.scale.set(w, it.h, d)
        // 다른 화물과 같이 위험 색이 비친다
        const heat = mesh(unitBox, heatBoxMat, ghostGroup, cx, (g.bottom + g.top) / 2, cz, false)
        heat.scale.set(w + 0.004, it.h + 0.004, d + 0.004)
        heat.renderOrder = 3
      }
    }
    shadowDirty = true
  }

  // ── 벨트 (웨빙 + 버클) ──
  const webTex = canvasTex(64, 256, (c, w, h) => {
    c.fillStyle = '#23272d'
    c.fillRect(0, 0, w, h)
    for (let x = 0; x < w; x += 3) {
      c.fillStyle = x % 6 ? 'rgba(255,255,255,.05)' : 'rgba(0,0,0,.25)'
      c.fillRect(x, 0, 1, h)
    }
    for (let y = 0; y < h; y += 4) {
      c.fillStyle = 'rgba(255,255,255,.03)'
      c.fillRect(0, y, w, 1)
    }
    c.fillStyle = 'rgba(210,216,224,.32)'
    c.fillRect(5, 0, 2, h)
    c.fillRect(w - 7, 0, 2, h)
  })
  webTex.wrapS = webTex.wrapT = THREE.RepeatWrapping
  // 주의 · 위험 때 빛나는 곳: 양쪽 가장자리 줄은 밝게, 가운데는 은은하게 (벨트가 테이프처럼 보이지 않게)
  const glowEdge = canvasTex(64, 4, (c, w, h) => {
    c.fillStyle = '#505050'
    c.fillRect(0, 0, w, h)
    c.fillStyle = '#ffffff'
    c.fillRect(2, 0, 7, h)
    c.fillRect(w - 9, 0, 7, h)
  })
  glowEdge.wrapS = glowEdge.wrapT = THREE.RepeatWrapping
  const straps: Strap[] = []
  const hookGeo = rbox(0.012, 0.01, STRAP_W + 0.008, 0.002)
  function ribbonGeometry(n: number) {
    const g = new THREE.BufferGeometry()
    g.setAttribute('position', new THREE.BufferAttribute(new Float32Array(n * 2 * 3), 3))
    g.setAttribute('normal', new THREE.BufferAttribute(new Float32Array(n * 2 * 3), 3))
    g.setAttribute('uv', new THREE.BufferAttribute(new Float32Array(n * 2 * 2), 2))
    const idx: number[] = []
    for (let i = 0; i < n - 1; i++) {
      const a = i * 2
      idx.push(a, a + 1, a + 2, a + 1, a + 3, a + 2)
    }
    g.setIndex(idx)
    return track(g)
  }
  for (const k of ['c1', 'c2'] as const) {
    const mat = std(0xffffff, 0.05, 0.72, { map: webTex, side: THREE.DoubleSide, emissive: 0x000000, emissiveMap: glowEdge })
    const ribbon = mesh(ribbonGeometry(N_STRAP), mat, bed, 0, 0, 0, true)
    ribbon.receiveShadow = false
    const tail = mesh(ribbonGeometry(24), mat, bed, 0, 0, 0, true)
    tail.receiveShadow = false
    const buckle = new THREE.Group()
    bed.add(buckle)
    mesh(rbox(0.008, 0.04, STRAP_W + 0.01, 0.0025), M.chrome, buckle, 0.004, 0, 0)
    mesh(rbox(0.006, 0.03, STRAP_W + 0.004, 0.002), M.dark, buckle, 0.011, 0.004, 0)
    mesh(cylG(0.0025, STRAP_W + 0.014, 10), M.aluDark, buckle, 0.006, -0.018, 0).rotation.x = Math.PI / 2
    const hooks = [mesh(hookGeo, M.chrome, bed), mesh(hookGeo, M.chrome, bed)]
    straps.push({ key: k, ribbon, tail, mat, buckle, hooks, slack: 0, slackGoal: 0, z: 0, top: new THREE.Vector3(), level: null })
  }
  const tmpA = new THREE.Vector3()
  const tmpB = new THREE.Vector3()
  const UP = new THREE.Vector3(0, 1, 0)

  type Layer = { x0: number; x1: number; bottom: number; top: number }
  /** 벨트 길 (팽팽할 때): 쌓인 층의 옆모습을 따라 감고 맨 위를 가로지른다. 느슨함 s(0..1) 만큼 부풀린다 */
  function strapPath(prof: Layer[], top: number, s: number, t: number, phase: number) {
    const base = PLATE_TOP
    const xa = L / 2 + MARGIN + 0.01
    const g = 0.0025
    const rc = 0.006
    const pts: [number, number, number][] = [] // x, y, kind(0 끝 · 턱, 1 옆, 2 위)
    const line = (ax: number, ay: number, bx: number, by: number, n: number, kind: number) => {
      for (let i = 0; i < n; i++) pts.push([ax + ((bx - ax) * i) / n, ay + ((by - ay) * i) / n, kind])
    }
    const arc = (cx: number, cy: number, r: number, a0: number, a1: number, n: number) => {
      for (let i = 0; i < n; i++) {
        const a = a0 + ((a1 - a0) * i) / n
        pts.push([cx + Math.cos(a) * r, cy + Math.sin(a) * r, 2])
      }
    }
    const P = prof.length ? prof : [{ x0: -0.05, x1: 0.05, bottom: base, top }]
    const first = P[0]
    const last = P[P.length - 1]
    const n = P.length
    const sideN = (i: number) => Math.max(4, Math.round(((i === n - 1 ? top : P[i].top) - (i ? P[i - 1].top : base)) / 0.006))
    // 왼쪽: 고리 -> 층마다 위로 (층 사이 턱에서 안/밖으로)
    line(-xa, base + 0.003, first.x0 - g, base + 0.014, 10, 0)
    for (let i = 0; i < n; i++) {
      const y0 = i ? P[i - 1].top + g : base + 0.014
      const y1 = i === n - 1 ? top - rc : P[i].top + g
      line(P[i].x0 - g, y0, P[i].x0 - g, y1, sideN(i), 1)
      if (i < n - 1) line(P[i].x0 - g, y1, P[i + 1].x0 - g, y1, 4, 0)
    }
    arc(last.x0 + rc - 0.002, top - rc, rc + g - 0.002, Math.PI, Math.PI / 2, 6)
    line(last.x0 + rc, top + g, last.x1 - rc, top + g, 44, 2)
    arc(last.x1 - rc + 0.002, top - rc, rc + g - 0.002, Math.PI / 2, 0, 6)
    // 오른쪽: 위에서 아래로
    for (let i = n - 1; i >= 0; i--) {
      const y1 = i === n - 1 ? top - rc : P[i].top + g
      const y0 = i ? P[i - 1].top + g : base + 0.014
      line(P[i].x1 + g, y1, P[i].x1 + g, y0, sideN(i), 1)
      if (i) line(P[i].x1 + g, y0, P[i - 1].x1 + g, y0, 4, 0)
    }
    line(first.x1 + g, base + 0.014, xa, base + 0.003, 10, 0)
    pts.push([xa, base + 0.003, 0])
    const h = Math.max(0.02, top - base)
    const sway = Math.sin(t * 1.4 + phase) * s
    const x0 = last.x0
    const x1 = last.x1
    return pts.map(([x, y, kind]) => {
      if (s <= 0.001) return [x, y]
      if (kind === 1) {
        // 옆면: 가운데가 바깥으로 불룩
        const u = Math.max(0, Math.min(1, (y - base) / h))
        const out = Math.sin(Math.PI * u) * (0.036 * s + 0.005 * sway)
        return [x + (x < 0 ? -out : out), y]
      }
      if (kind === 2) {
        const u = Math.max(0, Math.min(1, (x - x0) / Math.max(0.01, x1 - x0)))
        return [x, y + Math.sin(Math.PI * u) * 0.018 * s + Math.sin(u * Math.PI * 5 + t * 2 + phase) * 0.003 * s]
      }
      return [x, y]
    })
  }
  function writeRibbon(m: THREE.Mesh, pts: number[][], z: number) {
    const g = m.geometry as THREE.BufferGeometry
    const pos = g.getAttribute('position') as THREE.BufferAttribute
    const nor = g.getAttribute('normal') as THREE.BufferAttribute
    const uv = g.getAttribute('uv') as THREE.BufferAttribute
    const n = pos.count / 2
    let len = 0
    for (let i = 0; i < n; i++) {
      const p = pts[Math.min(i, pts.length - 1)]
      const q = pts[Math.min(i + 1, pts.length - 1)]
      const o = pts[Math.max(0, i - 1)]
      const tx = q[0] - o[0]
      const ty = q[1] - o[1]
      const tl = Math.hypot(tx, ty) || 1
      // 리본 면의 법선 = 길 방향 x Z
      const nx = ty / tl
      const ny = -tx / tl
      if (i > 0) len += Math.hypot(p[0] - pts[i - 1][0], p[1] - pts[i - 1][1])
      pos.setXYZ(i * 2, p[0], p[1], z - STRAP_W / 2)
      pos.setXYZ(i * 2 + 1, p[0], p[1], z + STRAP_W / 2)
      nor.setXYZ(i * 2, -nx, -ny, 0)
      nor.setXYZ(i * 2 + 1, -nx, -ny, 0)
      uv.setXY(i * 2, 0, len / 0.06)
      uv.setXY(i * 2 + 1, 1, len / 0.06)
    }
    pos.needsUpdate = true
    nor.needsUpdate = true
    uv.needsUpdate = true
    g.computeBoundingSphere()
  }
  /** 벨트 길을 고른 간격으로 N 개 점으로 */
  function resample(pts: number[][], n: number) {
    const cum = [0]
    for (let i = 1; i < pts.length; i++) cum.push(cum[i - 1] + Math.hypot(pts[i][0] - pts[i - 1][0], pts[i][1] - pts[i - 1][1]))
    const total = cum[cum.length - 1]
    const out: number[][] = []
    let j = 0
    for (let i = 0; i < n; i++) {
      const d = (total * i) / (n - 1)
      while (j < cum.length - 2 && cum[j + 1] < d) j++
      const k = (d - cum[j]) / Math.max(1e-9, cum[j + 1] - cum[j])
      out.push([pts[j][0] + (pts[j + 1][0] - pts[j][0]) * k, pts[j][1] + (pts[j + 1][1] - pts[j][1]) * k])
    }
    return out
  }
  function layoutStraps(t: number) {
    for (const s of straps) {
      const vis = !!cargoBox
      s.ribbon.visible = s.tail.visible = s.buckle.visible = vis
      s.hooks.forEach((h) => (h.visible = vis))
      if (!cargoBox) continue
      // 벨트는 쌓인 상자만 감는다 (옆 짐 제외)
      const stack = extents.filter((e) => e.kind === 'box' && !e.thin)
      const cb = stack.length
        ? { z0: Math.min(...stack.map((e) => e.z0)), z1: Math.max(...stack.map((e) => e.z1)), x0: Math.min(...stack.map((e) => e.x0)), x1: Math.max(...stack.map((e) => e.x1)), top: Math.max(...stack.map((e) => e.top)) }
        : cargoBox
      const depth = cb.z1 - cb.z0
      const z = s.key === 'c1' ? cb.z0 + depth * 0.24 : cb.z1 - depth * 0.24
      // 이 줄을 지나는 층들 (아래부터). 맨 위 = 덮개까지 포함한 가장 높은 곳
      const prof = stack.filter((e) => e.z0 <= z && e.z1 >= z).sort((p, q) => p.bottom - q.bottom)
      const top = Math.max(...extents.filter((e) => e.kind === 'box' && e.z0 <= z && e.z1 >= z).map((e) => e.top), prof.length ? 0 : cb.top)
      const x0 = prof.length ? prof[prof.length - 1].x0 : cb.x0
      const x1 = prof.length ? prof[prof.length - 1].x1 : cb.x1
      const xr = prof.length ? Math.min(...prof.map((p) => p.x1)) : x1
      s.z = z
      const raw = strapPath(prof.length ? prof : [{ x0, x1, bottom: PLATE_TOP, top }], top, s.slack, t, s.key === 'c1' ? 0 : 1.7)
      const pts = resample(raw, N_STRAP)
      writeRibbon(s.ribbon, pts, z)
      s.top.set(x0 + (x1 - x0) * (s.key === 'c1' ? 0.3 : 0.7), top + 0.004, z)
      // 버클: 오른쪽 옆면 45 % 높이
      const yb = PLATE_TOP + (top - PLATE_TOP) * 0.45
      let bi = 0
      let best = 1e9
      for (let i = N_STRAP / 2; i < N_STRAP; i++) {
        const dd = Math.abs(pts[i][1] - yb) + (pts[i][0] < xr - 0.01 ? 1 : 0)
        if (dd < best) {
          best = dd
          bi = i
        }
      }
      const p = pts[bi]
      const q = pts[Math.min(N_STRAP - 1, bi + 1)]
      s.buckle.position.set(p[0], p[1], z)
      tmpA.set(p[0] - q[0], p[1] - q[1], 0).normalize()
      s.buckle.quaternion.setFromUnitVectors(UP, tmpA)
      // 남는 끝자락: 버클에서 아래로 늘어진다 (풀리면 더 길고 흔들린다)
      const tl = 0.04 + s.slack * 0.07
      const tail: number[][] = []
      for (let i = 0; i < 24; i++) {
        const u = i / 23
        const sw = Math.sin(t * 1.8 + u * 2) * 0.01 * s.slack * u
        tail.push([p[0] + 0.009 + u * u * 0.012 * (1 + s.slack) + sw, p[1] - 0.016 - u * tl])
      }
      writeRibbon(s.tail, tail, z + 0.0012)
      s.hooks[0].position.set(-(L / 2 + MARGIN + 0.01), PLATE_TOP + 0.002, z)
      s.hooks[1].position.set(L / 2 + MARGIN + 0.01, PLATE_TOP + 0.002, z)
    }
  }

  // ── 무게중심 표적 (화물 위에 떠 있는 판 모양 표시) ──
  const reticle = new THREE.Group()
  bed.add(reticle)
  const glowTex = canvasTex(64, 64, (c, w, h) => {
    const g = c.createRadialGradient(w / 2, h / 2, 0, w / 2, h / 2, w / 2)
    g.addColorStop(0, 'rgba(255,255,255,1)')
    g.addColorStop(0.35, 'rgba(255,255,255,.35)')
    g.addColorStop(1, 'rgba(255,255,255,0)')
    c.fillStyle = g
    c.fillRect(0, 0, w, h)
  })
  const zoneMat = track(new THREE.MeshBasicMaterial({ color: COLORS.ok.clone(), transparent: true, opacity: 0.1, depthWrite: false, side: THREE.DoubleSide, toneMapped: false }))
  const zone = mesh(track(new THREE.CircleGeometry(1, 64)), zoneMat, reticle, 0, 0, 0, false)
  zone.rotation.x = -Math.PI / 2
  const zoneLineMat = track(new THREE.LineBasicMaterial({ color: COLORS.ok.clone(), transparent: true, opacity: 0.85, toneMapped: false }))
  const ellipsePts: THREE.Vector3[] = []
  for (let i = 0; i <= 96; i++) ellipsePts.push(new THREE.Vector3(Math.cos((i / 96) * Math.PI * 2), 0, Math.sin((i / 96) * Math.PI * 2)))
  const zoneLine = new THREE.Line(track(new THREE.BufferGeometry().setFromPoints(ellipsePts)), zoneLineMat)
  reticle.add(zoneLine)
  const frameMat = track(new THREE.LineDashedMaterial({ color: 0xdfe6ee, transparent: true, opacity: 0.32, dashSize: 0.008, gapSize: 0.006, toneMapped: false }))
  const frame = new THREE.LineSegments(track(new THREE.BufferGeometry()), frameMat)
  reticle.add(frame)
  const dropMat = track(new THREE.LineBasicMaterial({ color: 0xdfe6ee, transparent: true, opacity: 0.1, toneMapped: false }))
  const drops = new THREE.LineSegments(track(new THREE.BufferGeometry()), dropMat)
  bed.add(drops)
  const dotMat = track(new THREE.MeshStandardMaterial({ color: 0xffffff, emissive: 0xffffff, emissiveIntensity: 0.6, roughness: 0.3 }))
  const dot = mesh(track(new THREE.SphereGeometry(0.012, 20, 14)), dotMat, reticle, 0, 0.006, 0, false)
  const haloMat = track(new THREE.SpriteMaterial({ map: glowTex, color: COLORS.ok.clone(), transparent: true, depthWrite: false, opacity: 0.9, toneMapped: false }))
  const halo = new THREE.Sprite(haloMat)
  halo.scale.setScalar(0.05)
  dot.add(halo)
  const plumbMat = track(new THREE.LineBasicMaterial({ color: 0xffffff, transparent: true, opacity: 0.35, toneMapped: false }))
  // 투시 모드: 판에서 무게중심까지 빛줄기
  const beamMat = track(new THREE.MeshBasicMaterial({ color: 0xffffff, transparent: true, opacity: 0.85, depthWrite: false, toneMapped: false, blending: THREE.AdditiveBlending }))
  const beam = mesh(cylG(0.004, 1, 12), beamMat, bed, 0, 0, 0, false)
  beam.visible = false
  const beamGlow = mesh(cylG(0.012, 1, 16), track(new THREE.MeshBasicMaterial({ color: 0xffffff, transparent: true, opacity: 0.18, depthWrite: false, toneMapped: false, blending: THREE.AdditiveBlending })), bed, 0, 0, 0, false)
  beamGlow.visible = false
  const plumb = new THREE.Line(track(new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(), new THREE.Vector3(0, -1, 0)])), plumbMat)
  dot.add(plumb)
  const predMat = track(new THREE.MeshBasicMaterial({ color: 0xffffff, transparent: true, opacity: 0.55, depthWrite: false, side: THREE.DoubleSide, toneMapped: false }))
  const pred = mesh(track(new THREE.RingGeometry(0.007, 0.0095, 32)), predMat, reticle, 0, 0.002, 0, false)
  pred.rotation.x = -Math.PI / 2
  const arrowMat = track(new THREE.MeshBasicMaterial({ color: COLORS.warn.clone(), transparent: true, opacity: 0.95, depthWrite: false, side: THREE.DoubleSide, toneMapped: false }))
  const arrow = new THREE.Group()
  reticle.add(arrow)
  const shaft = mesh(track(new THREE.PlaneGeometry(1, 0.004)), arrowMat, arrow, 0, 0, 0, false)
  shaft.rotation.x = -Math.PI / 2
  const headShape = new THREE.Shape()
  headShape.moveTo(0, 0)
  headShape.lineTo(-0.016, 0.009)
  headShape.lineTo(-0.016, -0.009)
  headShape.closePath()
  const head = mesh(track(new THREE.ShapeGeometry(headShape)), arrowMat, arrow, 0, 0, 0, false)
  head.rotation.x = -Math.PI / 2
  let reticleY = PLATE_TOP + 0.05
  const cogNow = new THREE.Vector2(0, 0)
  const cogGoal = new THREE.Vector2(0, 0)
  const predNow = new THREE.Vector2(0, 0)
  const predGoal = new THREE.Vector2(0, 0)
  let hasCog = false
  let hasPred = false

  function buildFrame() {
    const hx = L / 2
    const hz = W / 2
    const p = [
      [-hx, -hz, hx, -hz],
      [hx, -hz, hx, hz],
      [hx, hz, -hx, hz],
      [-hx, hz, -hx, -hz],
      [-hx, 0, hx, 0],
      [0, -hz, 0, hz],
    ]
    const v: THREE.Vector3[] = []
    for (const [a, b, c2, d] of p) v.push(new THREE.Vector3(a, 0, b), new THREE.Vector3(c2, 0, d))
    frame.geometry.dispose()
    frame.geometry = new THREE.BufferGeometry().setFromPoints(v)
    frame.computeLineDistances()
    zone.scale.set(L * 0.25, W * 0.25, 1)
    zoneLine.scale.set(L * 0.25, 1, W * 0.25)
  }

  // ── 카메라 · 손가락 조작 ──
  const controls = new OrbitControls(camera, renderer.domElement)
  controls.enableDamping = true
  controls.dampingFactor = 0.07
  controls.enablePan = false
  controls.rotateSpeed = 0.55
  controls.zoomSpeed = 0.7
  controls.minPolarAngle = 0.001
  controls.maxPolarAngle = Math.PI * 0.47
  let stage = { x: 0, y: 64, w: 800, h: 320 }
  let width = 800
  let height = 480
  let baseDist = 2
  let preset: ScenePreset = 'main'
  let mode: ViewMode = 'normal'
  let explodeT = 0
  const EXPLODE_GAP = 0.08 // 분해: 층마다 이만큼 띄운다 (m)
  /** 투시 · 센서 모드에서 상자 재질을 비치게 */
  function applyModeMaterials() {
    const alpha = mode === 'xray' ? 0.28 : mode === 'sensor' ? 0.45 : 1
    const list: THREE.MeshStandardMaterial[] = [flapMat, ...Object.values(itemMat)]
    for (const b of blocks) list.push(...b.mats, b.itemMat)
    for (const m of list) {
      const tr = alpha < 1
      if (m.transparent !== tr || m.opacity !== alpha) {
        m.transparent = tr
        m.opacity = alpha
        m.depthWrite = !tr
        m.needsUpdate = true
      }
    }
    shadowDirty = true
  }
  let focusKey: CheckKey | null = null
  const goalPos = new THREE.Vector3()
  const goalTarget = new THREE.Vector3()
  let animating = true
  let dragging = false
  let lastInput = -1e9
  let sized = false

  function layout() {
    width = host.clientWidth || 800
    height = host.clientHeight || 480
    renderer.setSize(width, height, false)
    camera.aspect = width / height
    // 3D 영역(stage) 가운데에 오도록 화면을 비켜 그린다
    const ox = width / 2 - (stage.x + stage.w / 2)
    const oy = height / 2 - (stage.y + stage.h / 2)
    camera.setViewOffset(width, height, ox, oy, width, height)
    camera.updateProjectionMatrix()
    // 반지름 R 짜리 장면이 stage 에 들어가는 거리 (쌓은 높이에 따라)
    const R = sceneR
    const vfov = THREE.MathUtils.degToRad(camera.fov)
    const fitH = R / Math.tan(vfov / 2) / (stage.h / height)
    const fitW = R / (Math.tan(vfov / 2) * camera.aspect) / (stage.w / width)
    baseDist = Math.max(fitH, fitW) * 1.02
    controls.minDistance = baseDist * 0.45
    controls.maxDistance = baseDist * 1.8
    computeGoal()
    if (!sized) {
      sized = true
      camera.position.copy(goalPos)
      controls.target.copy(goalTarget)
      controls.update()
    }
    animating = true
  }
  const local = (x: number, y: number, z: number) => bed.localToWorld(tmpB.set(x, y, z)).clone()
  function computeGoal() {
    bed.updateWorldMatrix(true, false)
    const cb = cargoBox
    const top = cb ? cb.top : PLATE_TOP + 0.1
    const centre = local(0, (PLATE_TOP + top) / 2 + 0.02, 0)
    let dir = new THREE.Vector3(0.62, 0.56, 1)
    let dist = baseDist
    let target = centre
    if (focusKey) {
      dist = baseDist * 0.72
      if (focusKey === 'load' && cb) {
        target = local((cb.x0 + cb.x1) / 2, (PLATE_TOP + cb.top) / 2, (cb.z0 + cb.z1) / 2)
        dir = new THREE.Vector3(0.75, 0.5, 0.9)
        dist = baseDist * 0.86
      } else if (focusKey === 'cog' || focusKey === 'move') {
        target = local(0, reticleY, 0)
        dir = new THREE.Vector3(0.25, 1, 0.5)
        dist = baseDist * 0.8
      } else if (focusKey === 'c1') {
        const s = straps[0]
        target = local(0, s.top.y - 0.04, s.z)
        dir = new THREE.Vector3(-0.7, 0.55, -0.8)
      } else if (focusKey === 'c2') {
        const s = straps[1]
        target = local(0, s.top.y - 0.04, s.z)
        dir = new THREE.Vector3(0.7, 0.55, 0.85)
      }
    } else if (preset === 'top') {
      dir = new THREE.Vector3(0, 1, 0.16)
      dist = baseDist * 1.08
      target = local(0, PLATE_TOP, 0)
    } else if (preset === 'side') {
      dir = new THREE.Vector3(-1, 0.3, 0.06)
      dist = baseDist * 0.98
    }
    goalTarget.copy(target)
    goalPos.copy(target).addScaledVector(dir.normalize(), dist)
  }

  controls.addEventListener('start', () => {
    dragging = true
    animating = false
    lastInput = performance.now()
  })
  controls.addEventListener('end', () => {
    dragging = false
    lastInput = performance.now()
  })

  // ── 이름표 (DOM): 정상 아닌 항목만 + 고른 항목 ──
  const svgNS = 'http://www.w3.org/2000/svg'
  const svg = document.createElementNS(svgNS, 'svg')
  svg.setAttribute('class', 'tw-tag-lines')
  tagLayer.appendChild(svg)
  const tags = new Map<string, Tag>()
  function tagFor(id: string) {
    let t = tags.get(id)
    if (!t) {
      const el = document.createElement('div')
      el.className = 'tw-tag'
      tagLayer.appendChild(el)
      const line = document.createElementNS(svgNS, 'line')
      const dotEl = document.createElementNS(svgNS, 'circle')
      dotEl.setAttribute('r', '3.2')
      svg.appendChild(line)
      svg.appendChild(dotEl)
      t = { el, line, dot: dotEl, html: '', on: false, w: 0, h: 0, side: 1, x: 0, y: 0, ax: 0, ay: 0 }
      tags.set(id, t)
    }
    return t
  }

  // ── 데이터 반영 ──
  let view: TwinView | null = null
  let st: TwinState | null = null
  let shadowDirty = true
  const heatGoal = new Array<number>(6).fill(0)
  let lastShadow = 0
  const levelOf = (k: CheckKey): Level => view?.checks.find((c) => c.key === k)?.level ?? null

  function update(state: TwinState | null, v: TwinView) {
    st = state
    view = v
    const bedmm = bedOf(state)
    if (Math.abs(bedmm.l / 1000 - L) > 1e-4 || Math.abs(bedmm.w / 1000 - W) > 1e-4) {
      L = bedmm.l / 1000
      W = bedmm.w / 1000
      buildPlate()
      buildFrame()
    }
    if (!state) return
    if (state.reg) reg = { cx: state.reg.cx, cy: state.reg.cy }
    // 블록: 잠깐(1.5 s) 비면 직전 모양을 유지
    const now = performance.now()
    if (state.boxes.length) {
      lastBoxes = state.boxes
      boxesSeenAt = now
    }
    const boxes = state.boxes.length ? state.boxes : now - boxesSeenAt < 1500 ? lastBoxes : []
    setBlocks(boxes, state.base)

    const s = state.sensors

    // 로드셀 몫
    const parts = [s.loads.NW, s.loads.NE, s.loads.SW, s.loads.SE].map((g) => Math.max(0, g ?? 0))
    const tot = parts.reduce((a, b) => a + b, 0)
    feet.forEach((f, i) => (f.goal = tot > 200 ? parts[i] / tot : 0.25))

    // 무게중심 (mm -> m, 판 가운데 원점)
    const c = s.cog_mm
    hasCog = !!c && c.x !== null && c.y !== null
    if (hasCog) cogGoal.set(-L / 2 + c!.x! / 1000, -W / 2 + c!.y! / 1000)
    else if (s.cog) {
      hasCog = true
      cogGoal.set(-L / 2 + (s.cog.x_pct / 100) * L, -W / 2 + (s.cog.y_pct / 100) * W)
    }
    const p = s.cog_pred_mm
    hasPred = !!p && p.x !== null && p.y !== null
    if (hasPred) predGoal.set(-L / 2 + p!.x! / 1000, -W / 2 + p!.y! / 1000)

    // 벨트 느슨함: 필요 장력의 절반 아래로 내려갈수록
    for (const sp of straps) {
      const b = s.belts?.find((x) => x.key === sp.key)
      const lv = levelOf(sp.key)
      sp.level = lv
      let slack = 0
      if (b && b.n !== null && b.min_n) slack = THREE.MathUtils.clamp(1 - b.n / (0.5 * b.min_n), 0, 1)
      const word = view.checks.find((x) => x.key === sp.key)?.word
      if (word === '풀림') slack = Math.max(slack, 0.55)
      sp.slackGoal = slack
    }
  }

  // ── 매 프레임 ──
  const projV = new THREE.Vector3()
  let raf = 0
  let last = performance.now()
  let tSim = 0
  let readySent = false
  const pulse = (t: number, hz: number) => 0.5 + 0.5 * Math.sin(t * Math.PI * 2 * hz)

  /** 네 구역(NW NE SW SE) 단계 목표.
   *  적재: 그 모서리가 평균(25 %)보다 무거우면 적재 단계 그대로, 가벼우면 한 단계 아래 (과적 때 짐이 몰린 쪽이 더 빨갛게)
   *  벨트: 앞(NW NE) = 앞 벨트, 뒤(SW SE) = 뒤 벨트.  무게중심: 쏠린 쪽 두 구역.  화물 이동: 밀린 쪽 두 구역 */
  const QUAD_IDS = ['NW', 'NE', 'SW', 'SE'] as const
  function quadGoal(on: boolean, lvLoad: Level, heavy: string | null, lvCog: Level, lvMove: Level): number[] {
    if (!on || !st) return [0, 0, 0, 0]
    const loads = st.sensors.loads
    const parts = QUAD_IDS.map((k) => Math.max(0, loads[k] ?? 0))
    const tot = parts.reduce((a, b) => a + b, 0)
    const c1 = levelOf('c1') ?? 0
    const c2 = levelOf('c2') ?? 0
    const moveReason = view?.checks.find((c) => c.key === 'move')?.reason ?? ''
    const moveDir = /앞쪽/.test(moveReason) ? 'N' : /뒤쪽/.test(moveReason) ? 'S' : /왼쪽/.test(moveReason) ? 'W' : /오른쪽/.test(moveReason) ? 'E' : null
    return QUAD_IDS.map((id, i) => {
      let lv = 0
      if (lvLoad) lv = tot > 200 && parts[i] / tot >= 0.25 ? lvLoad : Math.max(0, lvLoad - 1)
      lv = Math.max(lv, id.startsWith('N') ? c1 : c2)
      if (lvCog && heavy && id.includes(heavy)) lv = Math.max(lv, lvCog)
      if (lvMove && moveDir && id.includes(moveDir)) lv = Math.max(lv, lvMove)
      return lv
    })
  }

  function frameLoop(now: number) {
    raf = requestAnimationFrame(frameLoop)
    const dt = Math.min(0.05, Math.max(0, (now - last) / 1000))
    last = now
    tSim += dt
    const t = tSim

    {
      const e0 = explodeT
      explodeT = approach(explodeT, mode === 'explode' ? 1 : 0, dt, 0.25)
      if (Math.abs(explodeT - e0) > 1e-4) shadowDirty = true
    }
    animateBlocks(1 - Math.exp(-dt / SHAPE_TAU))

    const connected = !!view?.connected
    const lvLoad = levelOf('load')
    const lvCog = levelOf('cog')
    const lvMove = levelOf('move')

    // 로드셀 고리: 몫만큼 호를 그린다 (넷이 고르면 각 1/4)
    const heavySide = (() => {
      if (!lvCog || !hasCog) return null
      const dx = cogGoal.x
      const dz = cogGoal.y
      return Math.abs(dz) / W >= Math.abs(dx) / L ? (dz > 0 ? 'S' : 'N') : dx > 0 ? 'E' : 'W'
    })()
    for (const f of feet) {
      f.share = approach(f.share, connected ? f.goal : 0.25, dt, 0.25)
      const segs = Math.max(1, Math.round(Math.min(1, f.share) * 64))
      f.arc.geometry.setDrawRange(0, segs * 6)
      let c = COLORS.calm
      let op = 0.75
      if (lvLoad && lvLoad > 0) c = levelCol(lvLoad)
      if (heavySide && f.id.includes(heavySide)) c = levelCol(lvCog)
      if (c !== COLORS.calm) op = 0.65 + 0.35 * pulse(t, (lvLoad ?? 0) >= 2 ? 1.2 : 0.6)
      if (!connected) op = 0.25
      f.arcMat.color.copy(c)
      f.arcMat.opacity = op
    }

    // 벨트
    let strapMoved = false
    for (const s of straps) {
      const prev = s.slack
      s.slack = approach(s.slack, s.slackGoal, dt, 0.5)
      if (Math.abs(prev - s.slack) > 1e-4 || s.slack > 0.01) strapMoved = true
      const lv = connected ? s.level : null
      if (lv !== null && lv >= 2) {
        s.mat.emissive.copy(levelCol(lv))
        s.mat.emissiveIntensity = 0.55 + 0.75 * pulse(t, lv === 3 ? 1.1 : 0.8)
        s.mat.color.setRGB(1, 0.75, 0.72)
      } else if (lv === 1) {
        s.mat.emissive.copy(COLORS.warn)
        s.mat.emissiveIntensity = 0.35 + 0.3 * pulse(t, 0.55)
        s.mat.color.setRGB(1, 0.9, 0.78)
      } else {
        s.mat.emissive.setRGB(0, 0, 0)
        s.mat.emissiveIntensity = 0
        s.mat.color.setRGB(1, 1, 1)
      }
    }
    layoutStraps(t)
    if (explodeT > 0.02)
      for (const s of straps) {
        s.ribbon.visible = s.tail.visible = s.buckle.visible = false
        s.hooks.forEach((h) => (h.visible = false))
      }
    {
      const on = mode === 'xray' && hasCog
      beam.visible = beamGlow.visible = on
      if (on) {
        const h = Math.max(0.01, reticleY - PLATE_TOP)
        for (const m of [beam, beamGlow]) {
          m.position.set(cogNow.x, PLATE_TOP + h / 2, cogNow.y)
          m.scale.set(1, h, 1)
        }
        const lc = levelCol(connected ? levelOf('cog') : null)
        beamMat.color.copy(lc)
        ;(beamGlow.material as THREE.MeshBasicMaterial).color.copy(lc)
      }
    }

    // 위험 빛 (히트맵): 자리는 고정 (0 앞 벨트 · 1 뒤 벨트 · 2 쏠린 쪽 · 3 밀린 상자 · 4,5 카메라가 본 형상 변화)
    {
      const on = connected && !!view?.judged
      heatU.uOn.value = approach(heatU.uOn.value, on ? 1 : 0, dt, 0.4)
      heatU.uTime.value = t
      heatU.uBase.value = approach(heatU.uBase.value, on ? (lvLoad ?? 0) : 0, dt, 0.3)
      // 적재판 네 구역: 그 모서리의 무게 몫 · 그쪽 벨트 · 무게중심이 쏠린 쪽 · 화물이 밀린 쪽 중 가장 나쁜 단계
      {
        const goal = quadGoal(on, lvLoad, heavySide, levelOf('cog'), lvMove)
        const q = heatU.uQuad.value
        q.set(approach(q.x, goal[0], dt, 0.35), approach(q.y, goal[1], dt, 0.35), approach(q.z, goal[2], dt, 0.35), approach(q.w, goal[3], dt, 0.35))
      }
      const cb = cargoBox
      const midY = (PLATE_TOP + (cb ? cb.top : PLATE_TOP + 0.1)) / 2
      const setSrc = (i: number, x: number, y: number, z: number, r: number, m: [number, number, number], lv: Level) => {
        heatU.uSrc.value[i].set(x, y, z, r)
        heatU.uMask.value[i].set(m[0], m[1], m[2])
        heatGoal[i] = on && lv ? lv : 0
      }
      setSrc(0, 0, midY, straps[0].z, 0.15, [0, 0, 1], levelOf('c1'))
      setSrc(1, 0, midY, straps[1].z, 0.15, [0, 0, 1], levelOf('c2'))
      const cl = Math.hypot(cogGoal.x, cogGoal.y)
      const ux = cl > 1e-4 ? cogGoal.x / cl : 0
      const uz = cl > 1e-4 ? cogGoal.y / cl : 0
      setSrc(2, cogGoal.x + ux * 0.1, midY, cogGoal.y + uz * 0.1, 0.26, [1, 0, 1], hasCog ? lvCog : null)
      const solid = extents.filter((e) => e.kind === 'box' && !e.thin)
      const topBox = solid.length ? solid.reduce((p, q) => (q.top > p.top ? q : p)) : null
      if (topBox) {
        const r = Math.max(topBox.x1 - topBox.x0, topBox.z1 - topBox.z0, topBox.top - topBox.bottom) * 0.8
        setSrc(3, (topBox.x0 + topBox.x1) / 2, (topBox.bottom + topBox.top) / 2, (topBox.z0 + topBox.z1) / 2, r, [1, 1, 1], lvMove)
      } else setSrc(3, 0, -9, 0, 0.01, [1, 1, 1], null)
      const al = extents.filter((e) => e.alert && !e.thin)
      for (let k = 0; k < 2; k++) {
        const e = al[k]
        if (e) setSrc(4 + k, (e.x0 + e.x1) / 2, (e.bottom + e.top) / 2, (e.z0 + e.z1) / 2, Math.max(e.x1 - e.x0, e.z1 - e.z0) * 0.7, [1, 1, 1], 1)
        else setSrc(4 + k, 0, -9, 0, 0.01, [1, 1, 1], null)
      }
      for (let i = 0; i < N_SRC; i++) heatU.uLv.value[i] = approach(heatU.uLv.value[i], heatGoal[i], dt, 0.35)
      // 상자 테두리: 그 상자 자리의 위험 색
      blocks.forEach((b, i) => {
        const e = extents[i]
        if (!e || !b.group.visible) return
        const r = riskAt((e.x0 + e.x1) / 2, (e.bottom + e.top) / 2, (e.z0 + e.z1) / 2)
        const lv: Level = heatU.uOn.value > 0.5 ? (r >= 2.5 ? 3 : r >= 1.5 ? 2 : r >= 0.6 ? 1 : 0) : null
        b.edgeMat.color.copy(lv === null ? COLORS.calm : levelCol(lv))
        b.edgeMat.opacity = e.thin ? 0 : lv === 3 ? 0.6 + 0.4 * pulse(t, 1.1) : lv === 2 ? 0.6 + 0.3 * pulse(t, 0.7) : lv === 1 ? 0.55 : 0.3
      })
    }
    if (strapMoved) shadowDirty = true

    // 무게중심 표적
    const cb = cargoBox
    reticleY = approach(reticleY, (cb ? cb.top : PLATE_TOP) + 0.045, dt, 0.3)
    reticle.position.y = reticleY
    reticle.visible = hasCog
    cogNow.x = approach(cogNow.x, cogGoal.x, dt, 0.18)
    cogNow.y = approach(cogNow.y, cogGoal.y, dt, 0.18)
    predNow.x = approach(predNow.x, predGoal.x, dt, 0.18)
    predNow.y = approach(predNow.y, predGoal.y, dt, 0.18)
    dot.position.set(cogNow.x, 0.004, cogNow.y)
    const cogLv = connected ? lvCog : null
    const zc = cogLv ? levelCol(cogLv) : view?.judged ? COLORS.ok : COLORS.calm
    zoneMat.color.copy(zc)
    zoneLineMat.color.copy(zc)
    zoneMat.opacity = cogLv ? 0.16 + 0.1 * pulse(t, 0.6) : 0.13
    haloMat.color.copy(cogLv ? levelCol(cogLv) : COLORS.ivory)
    halo.scale.setScalar(0.075 + (cogLv ? 0.02 * pulse(t, 0.8) : 0))
    plumb.scale.y = Math.max(0.001, reticleY - (cb ? cb.top : PLATE_TOP))
    pred.visible = hasPred
    pred.position.set(predNow.x, 0.002, predNow.y)
    const gap = Math.hypot(cogNow.x - predNow.x, cogNow.y - predNow.y)
    const moveLv = connected ? lvMove : null
    arrow.visible = hasPred && gap > 0.012 && !!moveLv
    if (arrow.visible) {
      arrowMat.color.copy(levelCol(moveLv))
      arrow.position.set(predNow.x, 0.003, predNow.y)
      arrow.rotation.y = -Math.atan2(cogNow.y - predNow.y, cogNow.x - predNow.x)
      const len = Math.max(0.001, gap - 0.006)
      shaft.scale.x = Math.max(0.001, len - 0.014)
      shaft.position.x = (len - 0.014) / 2
      head.position.x = len
    }
    // 판 모서리 -> 로드셀 옅은 내림선
    {
      const hx = L / 2
      const hz = W / 2
      const arr: number[] = []
      for (const [sx, sz] of [[-1, -1], [1, -1], [-1, 1], [1, 1]])
        arr.push(sx * hx, reticleY, sz * hz, sx * hx, FOOT_H, sz * hz)
      const g = drops.geometry
      const a = g.getAttribute('position') as THREE.BufferAttribute | undefined
      if (!a || a.count !== 8) g.setAttribute('position', new THREE.BufferAttribute(new Float32Array(arr), 3))
      else {
        ;(a.array as Float32Array).set(arr)
        a.needsUpdate = true
      }
      drops.visible = hasCog
    }

    // 카메라
    const idle = now - lastInput
    if (!dragging && idle > 20000 && !animating) {
      animating = true
    }
    if (animating && !dragging) {
      computeGoal()
      // 가만히 두면 기본 시점에서 아주 천천히 좌우로 흔들린다
      if (!focusKey && preset === 'main' && idle > 8000) {
        const a = Math.sin(t * ((Math.PI * 2) / 28)) * 0.2
        goalPos.sub(goalTarget).applyAxisAngle(UP, a).add(goalTarget)
      }
      const k = 1 - Math.exp(-dt * 2.6)
      camera.position.lerp(goalPos, k)
      controls.target.lerp(goalTarget, k)
    }
    controls.update(dt)

    // 그림자는 바뀔 때만 (최대 15 Hz)
    if (shadowDirty && now - lastShadow > 66) {
      renderer.shadowMap.needsUpdate = true
      shadowDirty = false
      lastShadow = now
    }
    renderer.render(scene, camera)
    placeTags()
    if (!readySent) {
      readySent = true
      opts.onReady?.()
    }
  }

  // ── 이름표 배치 ──
  function placeTags() {
    const want: { id: string; html: string; lv: Level; anchor: THREE.Vector3; side: 1 | -1 }[] = []
    const v = view
    if (v?.connected) {
      for (const c of v.checks) {
        // 고른 항목이 있으면 그것만, 없으면 정상이 아닌 것들
        const show = focusKey ? focusKey === c.key : mode === 'sensor' || (c.level ?? 0) > 0
        if (!show) continue
        let anchor: THREE.Vector3 | null = null
        let side: 1 | -1 = 1
        const cb = cargoBox
        if (c.key === 'load' && cb) {
          anchor = local(cb.x1, cb.top, cb.z1)
          side = 1
        } else if (c.key === 'cog' && hasCog) {
          anchor = local(cogNow.x, reticleY, cogNow.y)
          side = -1
        } else if (c.key === 'move' && hasCog) {
          anchor = local(cogNow.x, reticleY + 0.004, cogNow.y)
          side = 1
        } else if (c.key === 'c1' && cb) {
          anchor = local(straps[0].top.x, straps[0].top.y, straps[0].z)
          side = -1
        } else if (c.key === 'c2' && cb) {
          anchor = local(straps[1].top.x, straps[1].top.y, straps[1].z)
          side = 1
        }
        if (!anchor) continue
        const title = mode === 'sensor' ? `${c.name} <em>${c.value}${c.unit ? ' ' + c.unit : ''}</em>` : c.tag
        want.push({ id: c.key, lv: c.level, anchor, side, html: `<i class="ic">${tagIcon(c.key, c.word)}</i><b>${title}</b>` })
      }
    }
    // 센서 모드: 네 모서리 로드셀 무게
    if (v?.connected && mode === 'sensor' && st) {
      const names = { NW: '앞왼', NE: '앞오', SW: '뒤왼', SE: '뒤오' } as const
      for (const f of feet) {
        const g = st.sensors.loads[f.id]
        if (g === null || g === undefined) continue
        const p = f.group.position
        want.push({ id: f.id, lv: null, anchor: local(p.x, FOOT_H * 0.5, p.z), side: f.id.endsWith('E') ? 1 : -1, html: `<i class="ic">${ICON_SVG('weight')}</i><b>${names[f.id]} <em>${(Math.max(0, g) / 1000).toFixed(1)} kg</em></b>` })
      }
    }
    // 분해 모드: 층마다 높이
    if (mode === 'explode' && explodeT > 0.5) {
      const order = blocks.filter((b, i) => targets[i] && b.cur && b.kind === 'box' && !b.thin).sort((a, b2) => a.cur!.bottom - b2.cur!.bottom)
      order.forEach((b, i) => {
        const c = b.cur!
        const nm = order.length === 1 ? '상자' : i === 0 ? '아래 상자' : i === order.length - 1 ? '위 상자' : `${i + 1}층 상자`
        const p = b.group.position
        want.push({ id: `h${i}`, lv: null, anchor: local(c.x1, p.y + (c.top - c.bottom) / 2, (c.z0 + c.z1) / 2), side: 1, html: `<i class="ic">${ICON_SVG('ruler')}</i><b>${nm} <em>높이 ${Math.round((c.top - c.bottom) * 100)} cm</em></b>` })
      })
    }
    const seen = new Set(want.map((w) => w.id))
    for (const [id, t] of tags) {
      if (!seen.has(id) && t.on) {
        t.on = false
        t.el.classList.remove('on')
        t.line.style.opacity = '0'
        t.dot.style.opacity = '0'
      }
    }
    // 화면 좌표
    const placed: { t: Tag; ax: number; ay: number; side: 1 | -1 }[] = []
    for (const w of want) {
      const t = tagFor(w.id)
      const cls = `tw-tag on lv${w.lv ?? 'n'}${w.id.length === 2 && w.id !== 'c1' && w.id !== 'c2' ? ' corner' : ''}`
      if (t.html !== w.html || t.el.className !== cls) {
        t.html = w.html
        t.el.innerHTML = w.html
        t.el.className = cls
        t.w = t.el.offsetWidth
        t.h = t.el.offsetHeight
      }
      if (!t.on) {
        t.on = true
        t.w = t.el.offsetWidth
        t.h = t.el.offsetHeight
      }
      projV.copy(w.anchor).project(camera)
      const ax = (projV.x * 0.5 + 0.5) * width
      const ay = (-projV.y * 0.5 + 0.5) * height
      placed.push({ t, ax, ay, side: w.side })
    }
    // 화물이 화면에서 차지하는 좌우 범위 (이름표는 그 바깥에 둔다)
    let bl = width / 2
    let br = width / 2
    const cb = cargoBox
    if (cb) {
      bl = 1e9
      br = -1e9
      for (const x of [cb.x0, cb.x1])
        for (const y of [PLATE_TOP, reticleY])
          for (const z of [cb.z0, cb.z1]) {
            projV.copy(local(x, y, z)).project(camera)
            const sx = (projV.x * 0.5 + 0.5) * width
            bl = Math.min(bl, sx)
            br = Math.max(br, sx)
          }
    }
    // 같은 쪽끼리 위아래로 겹치지 않게. 위 · 아래 모서리 버튼 줄은 피한다
    const minY = stage.y + 54
    const maxY = stage.y + stage.h - 54
    for (const side of [-1, 1] as const) {
      const group = placed.filter((p) => p.side === side).sort((a, b) => a.ay - b.ay)
      let y = -1e9
      for (const p of group) {
        let ty = Math.max(p.ay - p.t.h - 10, y + 6)
        ty = Math.min(Math.max(ty, minY), maxY - p.t.h)
        p.t.y = ty
        y = ty + p.t.h
        const corner = p.t.el.classList.contains('corner')
        let tx = side < 0 ? Math.min(p.ax - 36, corner ? p.ax - 36 : bl - 18) - p.t.w : Math.max(p.ax + 36, corner ? p.ax + 36 : br + 18)
        tx = Math.min(Math.max(tx, stage.x + 6), stage.x + stage.w - 6 - p.t.w)
        p.t.x = tx
        p.t.ax = p.ax
        p.t.ay = p.ay
      }
      // 아래로 밀려 넘쳤으면 위로 다시 당긴다
      let yb = maxY
      for (let i = group.length - 1; i >= 0; i--) {
        const p = group[i]
        if (p.t.y + p.t.h > yb) p.t.y = Math.max(minY, yb - p.t.h)
        yb = p.t.y - 6
      }
    }
    for (const p of placed) {
      const t = p.t
      t.el.style.transform = `translate3d(${Math.round(t.x)}px, ${Math.round(t.y)}px, 0)`
      const ex = t.x + t.w / 2 < t.ax ? t.x + t.w : t.x
      const ey = t.y + t.h / 2
      t.line.setAttribute('x1', ex.toFixed(1))
      t.line.setAttribute('y1', ey.toFixed(1))
      t.line.setAttribute('x2', t.ax.toFixed(1))
      t.line.setAttribute('y2', t.ay.toFixed(1))
      t.dot.setAttribute('cx', t.ax.toFixed(1))
      t.dot.setAttribute('cy', t.ay.toFixed(1))
      const cls = t.el.className.includes('lv3') ? 'lv3' : t.el.className.includes('lv2') ? 'lv2' : t.el.className.includes('lv1') ? 'lv1' : t.el.className.includes('lv0') ? 'lv0' : 'lvn'
      t.line.setAttribute('class', cls)
      t.dot.setAttribute('class', cls)
      t.line.style.opacity = '1'
      t.dot.style.opacity = '1'
    }
  }

  // ── 크기 ──
  const ro = new ResizeObserver(() => layout())
  ro.observe(host)
  cleanups.push(() => ro.disconnect())
  const onLost = (e: Event) => e.preventDefault()
  renderer.domElement.addEventListener('webglcontextlost', onLost)
  cleanups.push(() => renderer.domElement.removeEventListener('webglcontextlost', onLost))

  buildPlate()
  buildFrame()
  layout()
  raf = requestAnimationFrame(frameLoop)

  const handle: SceneHandle = {
    update,
    setMode(m) {
      if (m === mode) return
      mode = m
      applyModeMaterials()
      animating = true
    },
    setPreset(p) {
      preset = p
      focusKey = null
      animating = true
      lastInput = -1e9
    },
    focus(k) {
      focusKey = k
      animating = true
      lastInput = -1e9
    },
    setStage(r) {
      stage = r
      layout()
    },
    dispose() {
      cancelAnimationFrame(raf)
      for (const fn of cleanups) fn()
      controls.dispose()
      for (const g of geos.values()) g.dispose()
      for (const d of disposables) d.dispose()
      renderer.dispose()
      renderer.domElement.remove()
      svg.remove()
      for (const t of tags.values()) t.el.remove()
    },
  }
  // 시험용 (브라우저 콘솔)
  ;(window as unknown as { __twin?: unknown }).__twin = {
    preset: (p: ScenePreset) => handle.setPreset(p),
    focus: (k: CheckKey | null) => handle.focus(k),
    cam: () => ({ pos: camera.position.toArray(), target: controls.target.toArray(), dist: camera.position.distanceTo(controls.target), baseDist, stage, width, height, animating }),
    alive: () => !!renderer.domElement.parentNode,
    st: () => ({ hasState: !!st, conn: view?.connected, judged: view?.judged, boxes: st?.boxes.length, ext: extents.length }),
    heat: () => ({ on: heatU.uOn.value, base: heatU.uBase.value, lv: [...heatU.uLv.value], src: heatU.uSrc.value.map((v) => v.toArray()) }),
    info: () => ({ calls: renderer.info.render.calls, tris: renderer.info.render.triangles, geos: renderer.info.memory.geometries, tex: renderer.info.memory.textures }),
  }
  return handle
}
