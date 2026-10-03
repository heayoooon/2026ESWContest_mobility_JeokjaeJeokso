"""트윈 쪽(PC/디스플레이 Pi) 수신 예제: 패킷을 풀어 높이맵 numpy 배열 + 센서값으로 복원.

usage: python3 twin/twin_receiver.py [seconds]      (UDP 5005)
"""
import json, socket, struct, sys, time, zlib
import numpy as np

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("0.0.0.0", 5005)); sock.settimeout(5)
end = time.time() + (float(sys.argv[1]) if len(sys.argv) > 1 else 1e9)
n = 0
while time.time() < end:
    try:
        pkt, _ = sock.recvfrom(65535)
    except socket.timeout:
        continue
    hl = struct.unpack("!I", pkt[:4])[0]
    hdr = json.loads(pkt[4:4 + hl])
    g = hdr["grid"]
    hm = np.frombuffer(zlib.decompress(pkt[4 + hl:]), np.int16).reshape(g["ny"], g["nx"])
    n += 1
    if n % 30 == 1:
        valid = hm != g["nodata"]
        print(f"recv seq {hdr['seq']:5d}  heightmap {hm.shape} int16 mm, valid {valid.mean()*100:4.1f}%, "
              f"max {hm[valid].max()} mm, imu tilt change {hdr['imu']['tilt_change_deg']:.2f}°, "
              f"lag {(time.time()-hdr['t'])*1000:.1f} ms, sensors {hdr['sensors']}")
print("received", n, "packets")
