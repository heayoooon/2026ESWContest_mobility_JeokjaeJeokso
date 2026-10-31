import csv
import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
DATA = ROOT / "data"
OUT = ROOT / "scenarios.json"

CASES = [
    {
        "id": "constant_drive",
        "title": "정속주행",
        "sensor": "20261003_165743.csv",
        "camera": "20261003_165743_camera.csv",
        "bag": "20261003_165741_d435i.db3",
    },
    {
        "id": "hard_start_stop",
        "title": "급출발 급정거",
        "sensor": "20261003_170015.csv",
        "camera": "20261003_170015_camera.csv",
        "bag": "20261003_170013_d435i.db3",
    },
    {
        "id": "right_turn",
        "title": "우회전",
        "sensor": "20261003_170232.csv",
        "camera": "20261003_170232_camera.csv",
        "bag": "20261003_170231_d435i.db3",
    },
    {
        "id": "cargo_drop",
        "title": "화물 떨어짐",
        "sensor": "20261003_170459.csv",
        "camera": "20261003_170459_camera.csv",
        "bag": "20261003_170458_d435i.db3",
    },
]


def read_rows(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as f:
        return list(csv.DictReader(f))


def number(row: dict[str, str], key: str, default: float = 0.0) -> float:
    value = row.get(key, "")
    if value == "":
        return default
    try:
        return float(value)
    except ValueError:
        return default


def sample_frames(sensor_rows: list[dict[str, str]], camera_rows: list[dict[str, str]]) -> list[dict[str, float]]:
    camera_by_t = [(number(row, "t_s"), row) for row in camera_rows]
    frames = []
    for row in sensor_rows:
        t = number(row, "t_s")
        cam = min(camera_by_t, key=lambda item: abs(item[0] - t))[1] if camera_by_t else {}
        frames.append(
            {
                "t": t,
                "total_g": number(row, "total_g"),
                "cog_x_pct": number(row, "cog_x_pct", 50.0),
                "cog_y_pct": number(row, "cog_y_pct", 50.0),
                "roll_deg": number(row, "imu1_roll_deg"),
                "pitch_deg": number(row, "imu1_pitch_deg"),
                "cam_volume_l": number(cam, "volume_l"),
                "cam_cx_m": number(cam, "cx_m"),
                "cam_cy_m": number(cam, "cy_m"),
                "cam_tilt_deg": number(cam, "tilt_deg"),
            }
        )
    return frames


def summarize(frames: list[dict[str, float]]) -> dict[str, float]:
    def values(key: str) -> list[float]:
        return [frame[key] for frame in frames]

    return {
        "duration_s": frames[-1]["t"] if frames else 0.0,
        "total_g_min": min(values("total_g")),
        "total_g_max": max(values("total_g")),
        "cog_x_min": min(values("cog_x_pct")),
        "cog_x_max": max(values("cog_x_pct")),
        "cog_y_min": min(values("cog_y_pct")),
        "cog_y_max": max(values("cog_y_pct")),
        "roll_min": min(values("roll_deg")),
        "roll_max": max(values("roll_deg")),
        "pitch_min": min(values("pitch_deg")),
        "pitch_max": max(values("pitch_deg")),
    }


def main() -> None:
    scenarios = []
    for case in CASES:
        sensor_rows = read_rows(DATA / case["sensor"])
        camera_rows = read_rows(DATA / case["camera"])
        frames = sample_frames(sensor_rows, camera_rows)
        scenarios.append({**case, "summary": summarize(frames), "frames": frames})
    OUT.write_text(json.dumps({"cases": scenarios}, ensure_ascii=False, indent=2), encoding="utf-8")
    print(f"wrote {OUT}")


if __name__ == "__main__":
    main()
