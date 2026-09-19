#!/usr/bin/env python3
"""D435i 동작 확인 스크립트.

용도
----
1) 카메라가 붙었는지 / USB3로 붙었는지 / 펌웨어가 뭔지
2) Depth·Color 프레임이 실제로 오는지, 몇 fps인지
3) Depth 품질 - 유효 픽셀 비율, 중앙 거리 측정값
4) IMU(accel/gyro)가 나오는지  <- D435i와 D435를 구분하는 지점

사용법
------
    python viz/test_d435i.py            # 진단 + 라이브 창 (q 종료, s 스냅샷)
    python viz/test_d435i.py --headless # 창 없이 숫자만 (SSH/RPi용)
    python viz/test_d435i.py --seconds 5

라즈베리파이 DSI 800x480에 띄울 때:
    python3 viz/test_d435i.py --max-width 800 --fullscreen --seconds 600
"""

import argparse
import sys
import time

import numpy as np

try:
    import pyrealsense2 as rs
except ImportError:
    sys.exit(
        "pyrealsense2가 없다. 설치:\n"
        "  py -3.10 -m pip install pyrealsense2 opencv-python numpy"
    )


def list_devices():
    """연결된 RealSense 장치 정보를 출력하고 개수를 리턴."""
    ctx = rs.context()
    devices = list(ctx.query_devices())

    print("=" * 60)
    print(f"[1] 장치 검색 - {len(devices)}개 발견")
    print("=" * 60)

    if not devices:
        print("  [X] 장치 없음.")
        print("    - USB 케이블 확인 (D435i는 USB-C, 반드시 데이터용 케이블)")
        print("    - USB 3.0 포트(파란색)에 직결. 허브/연장선은 일단 빼고 테스트")
        print("    - 장치관리자에 'Intel(R) RealSense(TM) Depth Camera 435i'가 뜨는지 확인")
        return 0

    for d in devices:
        name = d.get_info(rs.camera_info.name)
        serial = d.get_info(rs.camera_info.serial_number)
        fw = d.get_info(rs.camera_info.firmware_version)
        try:
            usb = d.get_info(rs.camera_info.usb_type_descriptor)
        except RuntimeError:
            usb = "unknown"

        print(f"  모델      : {name}")
        print(f"  시리얼    : {serial}")
        print(f"  펌웨어    : {fw}")
        if usb.startswith("2"):
            print(f"  USB 타입  : {usb}   [!] USB2로 붙었다 - 해상도/fps 제한됨. 포트/케이블 교체 권장")
        else:
            print(f"  USB 타입  : {usb}   [OK]")

        sensors = [s.get_info(rs.camera_info.name) for s in d.query_sensors()]
        print(f"  센서      : {', '.join(sensors)}")
        has_imu = any("Motion" in s for s in sensors)
        print(f"  IMU       : {'[OK] 있음 (D435i 맞음)' if has_imu else '[X] 없음 -> D435i가 아니라 D435일 수 있음'}")
        print("-" * 60)

    return len(devices)


def run_test(seconds, headless, width, height, fps, max_width=0, fullscreen=False):
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, width, height, rs.format.z16, fps)
    config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)

    imu_ok = False
    try:
        config.enable_stream(rs.stream.accel)
        config.enable_stream(rs.stream.gyro)
        imu_ok = True
    except RuntimeError:
        print("  [!] IMU 스트림 활성화 실패 - depth/color만 테스트한다")

    print("=" * 60)
    print(f"[2] 스트리밍 시작 - {width}x{height} @ {fps}fps, {seconds}초")
    print("=" * 60)

    try:
        profile = pipeline.start(config)
    except RuntimeError as e:
        print(f"  [X] 파이프라인 시작 실패: {e}")
        print("    다른 프로그램(realsense-viewer 등)이 카메라를 잡고 있지 않은지 확인")
        return False

    cv2 = None
    if not headless:
        try:
            import cv2 as _cv2

            cv2 = _cv2
        except ImportError:
            print("  [!] opencv-python 없음 -> headless로 진행")

    depth_sensor = profile.get_device().first_depth_sensor()
    depth_scale = depth_sensor.get_depth_scale()
    print(f"  depth scale : {depth_scale} (raw 1 -> {depth_scale * 1000:.2f} mm)")

    align = rs.align(rs.stream.color)
    colorizer = rs.colorizer()

    WIN = "D435i test  (q: quit, s: snapshot)"
    if cv2 is not None:
        if fullscreen:
            cv2.namedWindow(WIN, cv2.WINDOW_NORMAL)
            cv2.setWindowProperty(WIN, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
        else:
            cv2.namedWindow(WIN, cv2.WINDOW_AUTOSIZE)

    n_frames = 0
    valid_ratios = []
    center_dists = []
    accel_last = gyro_last = None
    t0 = time.time()
    snapshot_idx = 0

    try:
        while time.time() - t0 < seconds:
            try:
                frames = pipeline.wait_for_frames(timeout_ms=5000)
            except RuntimeError:
                print("  [X] 5초간 프레임이 안 온다 - 케이블/전원 확인")
                break

            if imu_ok:
                accel_f = frames.first_or_default(rs.stream.accel)
                gyro_f = frames.first_or_default(rs.stream.gyro)
                if accel_f:
                    accel_last = accel_f.as_motion_frame().get_motion_data()
                if gyro_f:
                    gyro_last = gyro_f.as_motion_frame().get_motion_data()

            aligned = align.process(frames)
            depth_frame = aligned.get_depth_frame()
            color_frame = aligned.get_color_frame()
            if not depth_frame or not color_frame:
                continue

            depth = np.asanyarray(depth_frame.get_data())
            color = np.asanyarray(color_frame.get_data())

            # depth 0 = "측정 실패" 픽셀. 유효 비율이 품질 지표가 된다
            valid_ratios.append(float(np.count_nonzero(depth)) / depth.size)
            h, w = depth.shape
            center_dists.append(depth_frame.get_distance(w // 2, h // 2))
            n_frames += 1

            if cv2 is not None:
                depth_vis = np.asanyarray(colorizer.colorize(depth_frame).get_data())
                cv2.circle(depth_vis, (w // 2, h // 2), 5, (255, 255, 255), 2)
                cv2.putText(
                    depth_vis,
                    f"{center_dists[-1]:.3f} m",
                    (w // 2 + 12, h // 2 - 8),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (255, 255, 255),
                    2,
                )
                view = np.hstack((color, depth_vis))
                # DSI 800x480 같은 작은 화면에 맞춰 축소 (스냅샷은 원본 해상도로 저장)
                shown = view
                if max_width and view.shape[1] > max_width:
                    s = max_width / view.shape[1]
                    shown = cv2.resize(view, None, fx=s, fy=s, interpolation=cv2.INTER_AREA)
                cv2.imshow(WIN, shown)
                key = cv2.waitKey(1) & 0xFF
                if key == ord("q"):
                    break
                if key == ord("s"):
                    fn = f"d435i_snapshot_{snapshot_idx}.png"
                    cv2.imwrite(fn, view)
                    print(f"  -> 저장: {fn}")
                    snapshot_idx += 1
    finally:
        pipeline.stop()
        if cv2 is not None:
            cv2.destroyAllWindows()

    elapsed = time.time() - t0

    print("=" * 60)
    print("[3] 결과")
    print("=" * 60)
    if n_frames == 0:
        print("  [X] 프레임 0장. 스트리밍 실패.")
        return False

    measured_fps = n_frames / elapsed
    mean_valid = float(np.mean(valid_ratios)) * 100
    center = np.array(center_dists)
    center_nz = center[center > 0]

    tail = "[OK]" if measured_fps > fps * 0.7 else f"[!] 설정({fps})보다 많이 낮다"
    print(f"  프레임 수      : {n_frames}장 / {elapsed:.1f}초 -> {measured_fps:.1f} fps  {tail}")

    if mean_valid > 80:
        tail = "[OK]"
    elif mean_valid > 50:
        tail = "[!] 낮음 - 피사체가 너무 가깝거나(<0.2m) 검은색/반사면일 수 있다"
    else:
        tail = "[X] 심각하게 낮음 - 렌즈 가림, 유리 너머 촬영, 강한 직사광 확인"
    print(f"  유효 depth 픽셀: {mean_valid:.1f}%  {tail}")

    if len(center_nz):
        print(
            f"  중앙 거리      : 평균 {center_nz.mean():.3f} m / "
            f"표준편차 {center_nz.std() * 1000:.1f} mm "
            f"(측정성공 {len(center_nz)}/{n_frames})"
        )
        if center_nz.std() * 1000 > 20:
            print("                   [!] 흔들림이 크다. 카메라 고정 후 재측정")
    else:
        print("  중앙 거리      : [X] 전부 측정 실패 - 0.3~2m 거리에 평평한 벽을 두고 다시")

    if accel_last is not None:
        mag = (accel_last.x ** 2 + accel_last.y ** 2 + accel_last.z ** 2) ** 0.5
        tail = "[OK] (정지 상태면 9.8 근처)" if 8.5 < mag < 11.0 else "[!] 9.8에서 벗어남"
        print(
            f"  accel          : x={accel_last.x:+.2f} y={accel_last.y:+.2f} "
            f"z={accel_last.z:+.2f} |a|={mag:.2f} m/s^2  {tail}"
        )
    if gyro_last is not None:
        print(
            f"  gyro           : x={gyro_last.x:+.3f} y={gyro_last.y:+.3f} "
            f"z={gyro_last.z:+.3f} rad/s  (정지 시 0 근처)"
        )

    print("=" * 60)
    return True


def main():
    p = argparse.ArgumentParser(description="RealSense D435i 동작 확인")
    p.add_argument("--seconds", type=float, default=10, help="측정 시간(초)")
    p.add_argument("--headless", action="store_true", help="창 없이 숫자만 출력")
    p.add_argument("--width", type=int, default=640)
    p.add_argument("--height", type=int, default=480)
    p.add_argument("--fps", type=int, default=30)
    p.add_argument(
        "--max-width", type=int, default=0,
        help="창 표시 폭 상한(px). DSI 800x480이면 800. 0=축소 안 함",
    )
    p.add_argument("--fullscreen", action="store_true", help="전체화면으로 띄운다")
    args = p.parse_args()

    if list_devices() == 0:
        sys.exit(1)

    ok = run_test(
        args.seconds, args.headless, args.width, args.height, args.fps,
        args.max_width, args.fullscreen,
    )
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
