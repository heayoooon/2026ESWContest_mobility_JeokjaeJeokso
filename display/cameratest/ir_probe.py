#!/usr/bin/env python3
"""IR 원본 영상을 받아 밝기 통계를 내고 PNG로 저장한다.

렌즈 가림 / 물체 근접 / IR 에미터 고장을 구분하기 위한 진단.
depth가 안 나올 때 "카메라 고장"인지 "볼 게 없는 것"인지 가르는 용도다.
OpenCV 없이 numpy + zlib만으로 PNG를 쓴다 (Pi에 cv2가 없어서).

사용법:
    ./rs_py.sh ir_probe.py            # 현재 디렉터리에 저장
    ./rs_py.sh ir_probe.py --out /tmp
"""

import argparse
import os
import struct
import sys
import zlib

import numpy as np
import pyrealsense2 as rs


def write_png_gray(path, arr):
    """8bit 그레이스케일 numpy 배열을 PNG로 저장."""
    arr = np.ascontiguousarray(arr, dtype=np.uint8)
    h, w = arr.shape

    raw = b"".join(b"\x00" + arr[y].tobytes() for y in range(h))

    def chunk(tag, data):
        c = struct.pack(">I", len(data)) + tag + data
        return c + struct.pack(">I", zlib.crc32(tag + data) & 0xFFFFFFFF)

    png = b"\x89PNG\r\n\x1a\n"
    png += chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 0, 0, 0, 0))
    png += chunk(b"IDAT", zlib.compress(raw, 6))
    png += chunk(b"IEND", b"")
    with open(path, "wb") as f:
        f.write(png)


def main():
    ap = argparse.ArgumentParser(description="D435i IR/depth 원본 진단")
    ap.add_argument("--out", default=".", help="PNG 저장 디렉터리 (기본: 현재 위치)")
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)

    pipe = rs.pipeline()
    cfg = rs.config()
    cfg.enable_stream(rs.stream.infrared, 1, 640, 480, rs.format.y8, 30)
    cfg.enable_stream(rs.stream.infrared, 2, 640, 480, rs.format.y8, 30)
    cfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

    profile = pipe.start(cfg)
    dev = profile.get_device()
    ds = dev.first_depth_sensor()

    # IR 에미터 상태 확인
    if ds.supports(rs.option.emitter_enabled):
        print(f"IR 에미터: {'켜짐' if ds.get_option(rs.option.emitter_enabled) else '꺼짐'}")
    if ds.supports(rs.option.laser_power):
        rng = ds.get_option_range(rs.option.laser_power)
        print(f"레이저 출력: {ds.get_option(rs.option.laser_power)} (범위 {rng.min}~{rng.max})")

    try:
        for _ in range(30):          # 자동노출 안정화
            pipe.wait_for_frames(timeout_ms=5000)

        frames = pipe.wait_for_frames(timeout_ms=5000)
        ir1 = np.asanyarray(frames.get_infrared_frame(1).get_data())
        ir2 = np.asanyarray(frames.get_infrared_frame(2).get_data())
        depth = np.asanyarray(frames.get_depth_frame().get_data())
    finally:
        pipe.stop()

    for name, ir in (("좌 IR", ir1), ("우 IR", ir2)):
        print(
            f"{name}: 평균밝기 {ir.mean():6.1f} / 표준편차 {ir.std():6.1f} / "
            f"최소 {ir.min():3d} / 최대 {ir.max():3d} / "
            f"포화(255) {np.count_nonzero(ir == 255) / ir.size * 100:5.1f}% / "
            f"암부(<10) {np.count_nonzero(ir < 10) / ir.size * 100:5.1f}%"
        )

    valid = np.count_nonzero(depth)
    print(f"depth 유효: {valid / depth.size * 100:.1f}%")
    if valid:
        nz = depth[depth > 0] * 0.001
        print(
            f"유효 픽셀 거리: 최소 {nz.min():.3f}m / 중앙값 {np.median(nz):.3f}m / "
            f"최대 {nz.max():.3f}m"
        )

    paths = [os.path.join(args.out, n) for n in ("ir_left.png", "ir_right.png", "depth_gray.png")]
    write_png_gray(paths[0], ir1)
    write_png_gray(paths[1], ir2)

    # depth를 눈으로 보기 쉽게: 0=검정, 0.2~3m를 회색조로
    d = depth.astype(np.float32) * 0.001
    vis = np.clip((d - 0.2) / (3.0 - 0.2), 0, 1) * 255
    vis[d == 0] = 0
    write_png_gray(paths[2], vis.astype(np.uint8))
    print("저장:", " ".join(paths))


if __name__ == "__main__":
    sys.exit(main())
