#!/usr/bin/env python3
"""
Trajectory Comparison Tool
===========================
실제 차량 궤적 (ego_state.jsonl / fix.jsonl) vs 사전 매핑 경로 (route CSV) 비교 시각화

실행 방법:
    python3 compare_trajectory.py                        # 기본 경로 자동 탐색
    python3 compare_trajectory.py \
        --route-dir ~/white_ws/gps_data \
        --bag-dir   ~/white_ws/ros2bag  \
        --out-dir   ~/white_ws/results  \
        --no-show                        # 창 표시 없이 이미지만 저장

의존성:
    pip install pandas matplotlib numpy scipy
"""

import argparse
import json
import sys
from pathlib import Path

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from matplotlib.collections import LineCollection
from scipy.spatial import KDTree


# ══════════════════════════════════════════════════════
#  기본 경로
# ══════════════════════════════════════════════════════
DEFAULT_ROUTE_DIR = Path("/home/dohyeon/white_ws/gps_data")
DEFAULT_BAG_DIR   = Path("/home/dohyeon/white_ws/ros2bag")

W = 60  # 터미널 박스 너비


# ══════════════════════════════════════════════════════
#  터미널 UI 헬퍼
# ══════════════════════════════════════════════════════

def box_top(title: str):
    print(f"\n╔{'═' * W}╗")
    pad = (W - len(title)) // 2
    print(f"║{' ' * pad}{title}{' ' * (W - pad - len(title))}║")
    print(f"╠{'═' * W}╣")


def box_row(text: str):
    # 한글 포함 시 글자 폭 보정 (한글 1자 = 2칸)
    display_len = sum(2 if ord(c) > 0x7F else 1 for c in text)
    pad = max(0, W - 2 - display_len)
    print(f"║  {text}{' ' * pad}║")


def box_sep():
    print(f"╠{'─' * W}╣")


def box_bot():
    print(f"╚{'═' * W}╝")


def prompt_int(prompt: str, lo: int, hi: int) -> int:
    """lo ~ hi 범위의 정수를 입력받을 때까지 반복."""
    while True:
        try:
            raw = input(f"\n  {prompt} [{lo}~{hi}]: ").strip()
            val = int(raw)
            if lo <= val <= hi:
                return val
            print(f"  ⚠  {lo}~{hi} 사이의 숫자를 입력하세요.")
        except (ValueError, EOFError):
            print("  ⚠  숫자를 입력하세요.")


# ══════════════════════════════════════════════════════
#  파일 선택 대화형 로직
# ══════════════════════════════════════════════════════

def select_route(route_dir: Path) -> Path:
    """gps_data 폴더에서 route CSV 선택."""
    csvs = sorted(route_dir.glob("route_*.csv"))
    if not csvs:
        sys.exit(f"\n오류: {route_dir} 에 route_*.csv 파일이 없습니다.")

    box_top("STEP 1 / 2  —  사전 매핑 경로 파일 선택")
    for i, p in enumerate(csvs, 1):
        size_kb = p.stat().st_size // 1024
        box_row(f"[{i:2d}]  {p.name:<40}  {size_kb:>4} KB")
    box_bot()

    idx = prompt_int("번호 입력", 1, len(csvs))
    chosen = csvs[idx - 1]
    print(f"\n  ✔  선택됨: {chosen.name}")
    return chosen


def select_bag_folder(bag_dir: Path) -> Path:
    """ros2bag 폴더에서 extracted_* 폴더 선택."""
    folders = sorted(
        p for p in bag_dir.iterdir()
        if p.is_dir() and p.name.startswith("extracted_")
    )
    if not folders:
        sys.exit(f"\n오류: {bag_dir} 에 extracted_* 폴더가 없습니다.")

    box_top("STEP 2 / 2  —  추출된 ROS2 bag 폴더 선택")
    box_row(f"{'':5}{'폴더명':<44}  ego  fix")
    box_sep()
    for i, p in enumerate(folders, 1):
        ego_ok = "✓" if (p / "ego_state.jsonl").exists() else "✗"
        fix_ok = "✓" if (p / "fix.jsonl").exists()       else "✗"
        mark   = "▶" if ego_ok == "✓" and fix_ok == "✓" else " "
        box_row(f"{mark}[{i:2d}]  {p.name:<44}   {ego_ok}    {fix_ok}")
    box_bot()

    idx = prompt_int("번호 입력", 1, len(folders))
    chosen = folders[idx - 1]
    print(f"\n  ✔  선택됨: {chosen.name}")
    return chosen


def resolve_paths(bag_dir: Path, route_dir: Path):
    """대화형으로 세 파일 경로를 확정하고 반환."""
    route_path = select_route(route_dir)
    bag_folder = select_bag_folder(bag_dir)

    ego_path = bag_folder / "ego_state.jsonl"
    fix_path = bag_folder / "fix.jsonl"

    missing = [str(p) for p in [ego_path, fix_path] if not p.exists()]
    if missing:
        sys.exit("\n오류: 다음 파일이 없습니다:\n  " + "\n  ".join(missing))

    print(f"\n{'─' * W}")
    print(f"  route      : {route_path.name}")
    print(f"  ego_state  : {ego_path}")
    print(f"  fix        : {fix_path}")
    print(f"{'─' * W}\n")

    return ego_path, fix_path, route_path


# ══════════════════════════════════════════════════════
#  데이터 로드
# ══════════════════════════════════════════════════════

def load_ego_state(path: Path) -> pd.DataFrame:
    """ego_state.jsonl  →  data.data = [lat, lon, vx, vy, heading, ?, speed]"""
    records = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            obj = json.loads(line)
            ts  = obj["timestamp"]
            d   = obj["data"]["data"]
            # ego_state 규약:
            #   data[0]=lat, data[1]=lon, data[2]=fused_x, data[3]=fused_y,
            #   data[4]=heading, data[5]=speed, data[6]=steer placeholder
            # 기존 map_check.py는 data[6]을 speed로 읽어서 속도 그래프가 항상 0으로 표시됐다.
            speed = d[5] if len(d) > 5 else (d[6] if len(d) > 6 else 0.0)
            records.append({
                "timestamp_ns": ts,
                "timestamp_s":  ts * 1e-9,
                "lat":          d[0],
                "lon":          d[1],
                "x_m":          d[2],
                "y_m":          d[3],
                "heading":      d[4],
                "speed":        speed,
            })
    df = pd.DataFrame(records).sort_values("timestamp_s").reset_index(drop=True)
    df["elapsed_s"] = df["timestamp_s"] - df["timestamp_s"].iloc[0]
    return df


def load_fix(path: Path) -> pd.DataFrame:
    """fix.jsonl  →  GPS NavSatFix"""
    records = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            obj = json.loads(line)
            ts  = obj["timestamp"]
            d   = obj["data"]
            records.append({
                "timestamp_ns": ts,
                "timestamp_s":  ts * 1e-9,
                "lat":          d["latitude"],
                "lon":          d["longitude"],
                "altitude":     d["altitude"],
                "gps_status":   d["status"]["status"],
            })
    df = pd.DataFrame(records).sort_values("timestamp_s").reset_index(drop=True)
    df["elapsed_s"] = df["timestamp_s"] - df["timestamp_s"].iloc[0]
    return df


def load_route(path: Path) -> pd.DataFrame:
    """route CSV  →  latitude, longitude, heading, speed, steer"""
    df = pd.read_csv(path)
    df = df.rename(columns={"latitude": "lat", "longitude": "lon"})
    return df.reset_index(drop=True)


# ══════════════════════════════════════════════════════
#  분석 함수
# ══════════════════════════════════════════════════════

def latlon_to_meters(lat, lon, lat0, lon0):
    R = 6_371_000.0
    x = np.radians(lon - lon0) * R * np.cos(np.radians(lat0))
    y = np.radians(lat - lat0) * R
    return x, y


def _to_meters_xy(lat, lon, lat0):
    R  = 6_371_000.0
    sx = np.radians(1) * R * np.cos(np.radians(lat0))
    sy = np.radians(1) * R
    return np.column_stack([lon * sx, lat * sy])


def compute_cross_track_error(ego_lat, ego_lon, route_lat, route_lon):
    lat0    = route_lat.mean()
    route_m = _to_meters_xy(route_lat, route_lon, lat0)
    ego_m   = _to_meters_xy(ego_lat,   ego_lon,   lat0)
    dist, _ = KDTree(route_m).query(ego_m)
    return dist


def compute_heading_error(ego_heading, route, ego_lat, ego_lon):
    lat0    = route["lat"].mean()
    route_m = _to_meters_xy(route["lat"].values, route["lon"].values, lat0)
    ego_m   = _to_meters_xy(ego_lat, ego_lon, lat0)
    _, idx  = KDTree(route_m).query(ego_m)
    diff    = ego_heading - route["heading"].values[idx]
    return (diff + 180) % 360 - 180


def print_stats(label: str, arr: np.ndarray, unit: str = "m"):
    print(f"  {label}")
    print(f"    mean={np.mean(arr):.4f} {unit}  "
          f"std={np.std(arr):.4f}  "
          f"max={np.max(arr):.4f}  "
          f"95th%={np.percentile(arr,95):.4f} {unit}")


# ══════════════════════════════════════════════════════
#  시각화
# ══════════════════════════════════════════════════════

def _colormap_line(ax, x, y, c, cmap="YlOrRd", lw=2, label=""):
    pts  = np.array([x, y]).T.reshape(-1, 1, 2)
    segs = np.concatenate([pts[:-1], pts[1:]], axis=1)
    norm = plt.Normalize(c.min(), c.max())
    lc   = LineCollection(segs, cmap=cmap, norm=norm, linewidth=lw, label=label)
    lc.set_array(c[:-1])
    ax.add_collection(lc)
    return lc


def plot_comparison(ego, fix, route, cte, hdg_err,
                    output_path: str, title_tag: str, show: bool):

    lat0 = route["lat"].mean()
    lon0 = route["lon"].mean()

    rx, ry = latlon_to_meters(route["lat"].values, route["lon"].values, lat0, lon0)
    ex, ey = latlon_to_meters(ego["lat"].values,   ego["lon"].values,   lat0, lon0)
    fx, fy = latlon_to_meters(fix["lat"].values,   fix["lon"].values,   lat0, lon0)

    fig = plt.figure(figsize=(18, 14))
    fig.suptitle(f"Trajectory Comparison  ·  {title_tag}",
                 fontsize=13, fontweight="bold", y=0.98)
    gs = gridspec.GridSpec(3, 3, figure=fig,
                           hspace=0.44, wspace=0.38,
                           left=0.06, right=0.97,
                           top=0.93, bottom=0.06)

    # (A) 궤적 오버레이
    ax = fig.add_subplot(gs[0:2, 0:2])
    ax.plot(rx, ry, color="#2563eb", lw=2.5, zorder=3,
            label="Mapped route", alpha=0.9)
    lc = _colormap_line(ax, ex, ey, cte, label="ego_state (CTE color)")
    plt.colorbar(lc, ax=ax, label="Cross-track error (m)", shrink=0.7)
    ax.scatter(fx, fy, c="#16a34a", s=4, alpha=0.4, zorder=2, label="GPS fix")
    ax.scatter(ex[0],  ey[0],  c="lime", s=80, zorder=5, marker="^", label="Start")
    ax.scatter(ex[-1], ey[-1], c="red",  s=80, zorder=5, marker="s", label="End")
    ax.set_xlabel("East (m)"); ax.set_ylabel("North (m)")
    ax.set_title("(A) Trajectory Overlay")
    ax.legend(fontsize=8); ax.set_aspect("equal"); ax.grid(True, alpha=0.3)

    # (B) CTE 시계열 (수정: .values 추가)
    ax = fig.add_subplot(gs[0, 2])
    ax.plot(ego["elapsed_s"].values, cte, color="#dc2626", lw=1.2)
    ax.axhline(np.mean(cte), color="#7f1d1d", ls="--", lw=1,
               label=f"mean {np.mean(cte):.3f} m")
    ax.fill_between(ego["elapsed_s"].values, 0, cte, alpha=0.15, color="#dc2626")
    ax.set_xlabel("Elapsed time (s)"); ax.set_ylabel("CTE (m)")
    ax.set_title("(B) Cross-Track Error")
    ax.legend(fontsize=8); ax.grid(True, alpha=0.3)

    # (C) Heading error (수정: .values 추가)
    ax = fig.add_subplot(gs[1, 2])
    ax.plot(ego["elapsed_s"].values, hdg_err, color="#7c3aed", lw=1.2)
    ax.axhline(0, color="gray", ls="--", lw=0.8)
    ax.fill_between(ego["elapsed_s"].values, 0, hdg_err,
                    where=(hdg_err >= 0), alpha=0.15, color="#7c3aed")
    ax.fill_between(ego["elapsed_s"].values, 0, hdg_err,
                    where=(hdg_err <  0), alpha=0.15, color="#db2777")
    ax.set_xlabel("Elapsed time (s)"); ax.set_ylabel("Heading error (°)")
    ax.set_title("(C) Heading Error vs Route"); ax.grid(True, alpha=0.3)

    # (D) 속도 프로파일 (수정: .values 추가)
    ax = fig.add_subplot(gs[2, 0])
    ax.plot(ego["elapsed_s"].values, np.abs(ego["speed"].values),
            color="#0891b2", lw=1.2, label="ego |speed|")
    if "speed" in route.columns:
        ax.axhline(route["speed"].abs().mean(), color="#0e7490", ls="--", lw=1.2,
                   label=f"route mean {route['speed'].abs().mean():.1f} m/s")
    ax.set_xlabel("Elapsed time (s)"); ax.set_ylabel("Speed (m/s)")
    ax.set_title("(D) Speed Profile")
    ax.legend(fontsize=8); ax.grid(True, alpha=0.3)

    # (E) CTE 히스토그램
    ax = fig.add_subplot(gs[2, 1])
    ax.hist(cte, bins=40, color="#f97316", edgecolor="white",
            linewidth=0.4, alpha=0.85)
    ax.axvline(np.mean(cte),           color="#7c2d12", ls="--", lw=1.2,
               label=f"mean={np.mean(cte):.3f} m")
    ax.axvline(np.percentile(cte, 95), color="#dc2626", ls=":",  lw=1.2,
               label=f"95th%={np.percentile(cte,95):.3f} m")
    ax.set_xlabel("CTE (m)"); ax.set_ylabel("Count")
    ax.set_title("(E) CTE Distribution")
    ax.legend(fontsize=8); ax.grid(True, alpha=0.3)

    # (F) GPS fix vs ego 잔차
    ax   = fig.add_subplot(gs[2, 2])
    R    = 6_371_000.0
    midx = np.searchsorted(ego["timestamp_s"].values,
                           fix["timestamp_s"].values).clip(0, len(ego) - 1)
    dlat = (fix["lat"].values - ego["lat"].values[midx]) * np.radians(1) * R
    dlon = (fix["lon"].values - ego["lon"].values[midx]) * np.radians(1) * R \
           * np.cos(np.radians(lat0))
    rms  = np.sqrt(np.mean(dlat**2 + dlon**2))
    sc   = ax.scatter(dlon, dlat, c=fix["elapsed_s"].values,
                      cmap="viridis", s=8, alpha=0.6)
    plt.colorbar(sc, ax=ax, label="Elapsed time (s)", shrink=0.7)
    ax.axhline(0, color="gray", lw=0.6, ls="--")
    ax.axvline(0, color="gray", lw=0.6, ls="--")
    ax.set_xlabel("ΔEast (m)  [fix − ego]")
    ax.set_ylabel("ΔNorth (m)  [fix − ego]")
    ax.set_title(f"(F) GPS fix vs ego_state  RMS={rms:.4f} m")
    ax.set_aspect("equal"); ax.grid(True, alpha=0.3)

    plt.savefig(output_path, dpi=150, bbox_inches="tight")
    print(f"\n  그림 저장 → {output_path}")
    if show:
        print("  (창을 닫으면 종료됩니다)")
        plt.show()
    plt.close()


# ══════════════════════════════════════════════════════
#  CSV 저장
# ══════════════════════════════════════════════════════

def save_stats_csv(ego, cte, hdg_err, output_path: str):
    pd.DataFrame({
        "timestamp_s":       ego["timestamp_s"].values,
        "elapsed_s":         ego["elapsed_s"].values,
        "lat":               ego["lat"].values,
        "lon":               ego["lon"].values,
        "heading_deg":       ego["heading"].values,
        "speed_mps":         ego["speed"].values,
        "cross_track_err_m": cte,
        "heading_err_deg":   hdg_err,
    }).to_csv(output_path, index=False)
    print(f"  통계 CSV 저장 → {output_path}")


# ══════════════════════════════════════════════════════
#  main
# ══════════════════════════════════════════════════════

def main():
    parser = argparse.ArgumentParser(
        description="실제 차량 궤적 vs 사전 매핑 경로 비교 도구"
    )
    parser.add_argument("--route-dir", default=str(DEFAULT_ROUTE_DIR),
                        help=f"route CSV 폴더 (default: {DEFAULT_ROUTE_DIR})")
    parser.add_argument("--bag-dir",   default=str(DEFAULT_BAG_DIR),
                        help=f"ROS2 bag 루트 폴더 (default: {DEFAULT_BAG_DIR})")
    parser.add_argument("--out-dir",   default=None,
                        help="출력 디렉터리 (default: 선택한 bag 폴더 내부)")
    parser.add_argument("--no-show",   action="store_true",
                        help="matplotlib 창 표시 없이 이미지만 저장")
    args = parser.parse_args()

    route_dir = Path(args.route_dir).expanduser()
    bag_dir   = Path(args.bag_dir).expanduser()

    print(f"\n{'═' * W}")
    print(f"  Trajectory Comparison Tool  (ROS2 Humble / Ubuntu 22.04)")
    print(f"{'═' * W}")

    ego_path, fix_path, route_path = resolve_paths(bag_dir, route_dir)

    out_dir = Path(args.out_dir).expanduser() if args.out_dir \
              else ego_path.parent
    out_dir.mkdir(parents=True, exist_ok=True)

    # 데이터 로드
    print("데이터 로드 중...")
    ego   = load_ego_state(ego_path)
    fix   = load_fix(fix_path)
    route = load_route(route_path)
    print(f"  ego_state : {len(ego):,} rows  ({ego['elapsed_s'].iloc[-1]:.1f} s)")
    print(f"  fix       : {len(fix):,} rows")
    print(f"  route     : {len(route):,} waypoints")

    # 지표 계산
    print("\n지표 계산 중...")
    cte     = compute_cross_track_error(
                  ego["lat"].values, ego["lon"].values,
                  route["lat"].values, route["lon"].values)
    hdg_err = compute_heading_error(
                  ego["heading"].values, route,
                  ego["lat"].values, ego["lon"].values)

    R      = 6_371_000.0
    lat0   = route["lat"].mean()
    midx   = np.searchsorted(ego["timestamp_s"].values,
                             fix["timestamp_s"].values).clip(0, len(ego) - 1)
    dlat_m = (fix["lat"].values - ego["lat"].values[midx]) * np.radians(1) * R
    dlon_m = (fix["lon"].values - ego["lon"].values[midx]) * np.radians(1) * R \
             * np.cos(np.radians(lat0))
    dist_m = np.sqrt(dlat_m**2 + dlon_m**2)

    print(f"\n{'─' * W}")
    print("  분석 결과")
    print(f"{'─' * W}")
    print_stats("Cross-Track Error  (ego vs route)",   cte,             "m")
    print_stats("Heading Error      (ego vs route)",   np.abs(hdg_err), "°")
    print_stats("GPS fix vs ego_state  position diff", dist_m,          "m")
    print(f"{'─' * W}")

    tag         = ego_path.parent.name.replace("extracted_rosbag2_", "")
    plot_output = str(out_dir / f"trajectory_comparison_{tag}.png")
    csv_output  = str(out_dir / f"trajectory_stats_{tag}.csv")
    title_tag   = f"{route_path.stem}  ←→  {ego_path.parent.name}"

    print("\n시각화 생성 중...")
    plot_comparison(ego, fix, route, cte, hdg_err,
                    plot_output, title_tag, show=(not args.no_show))
    save_stats_csv(ego, cte, hdg_err, csv_output)
    print("\n완료!")


if __name__ == "__main__":
    main()