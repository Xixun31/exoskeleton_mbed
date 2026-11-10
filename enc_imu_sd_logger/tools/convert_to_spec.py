#!/usr/bin/env python3
"""把 enc_imu_sd_logger 的紀錄檔轉成 data_format_spec.md 的原始資料格式。

用法:
    python3 convert_to_spec.py run009.csv --subject S01 --trial T01
    python3 convert_to_spec.py run009.csv --subject S01 --trial T01 --out data --date 2026-11-06

輸入 (同一個資料夾):
    runXXX.csv          encoder + 3 顆 IMU，100 Hz (每列一個取樣週期)
    runXXX_foot.csv     足底，每列一包 (左右腳交錯，各約 19 Hz)
    runXXX_origin.csv   原點 3 秒的統計 (有的話，用來填 static_trial_s)

輸出:
    <out>/<S>/raw/<S>_<T>_raw.csv       IMU + encoder，100 Hz
    <out>/<S>/raw/<S>_<T>_raw_fsr.csv   足底，每包一列 (另一隻腳的欄位留空)
    <out>/<S>/<S>_<T>_meta.json         實驗資訊；量不到的欄位是 null，要手動填

轉換規則:
  - time 從錄製開始 (第一個週期) 算 0，所有檔案用同一個 t0，所以時間可以直接對齊。
  - 每個感測器另有自己的時間戳欄 (*_time)：IMU = 封包第一個 byte 收到的時間，
    encoder = CS 下降緣 (角度鎖存)，足底 = STM32 收到封包的時間。
  - 缺值寫 NaN，不填 0：
      IMU 最新封包超過 25 ms 沒更新 → 那顆 IMU 的值全部 NaN (原檔是沿用上一筆)
      encoder 通訊錯誤 (同位 / OCF / COF / 起始 bit，err & 0x27) → NaN (原檔 raw 是錯的讀值)
  - 四元數由 Euler 角換算 (Xsens Euler 是 Z-Y-X 順序：q = qz(yaw)·qy(pitch)·qx(roll))。
    pitch = ±90° 時 Euler 本身有萬向鎖，換算出來的四元數也一樣不可靠。
只用 Python 標準函式庫。
"""
import argparse
import csv
import json
import math
import os
import sys

# ---------------- 感測器對應 (裝的位置換了改這裡) ----------------
IMU_MAP = {"pelvis": "", "L_shank": "imuA_", "R_shank": "imuB_"}  # 腰背 = SPI IMU；左 / 右腳踝 = UART IMU A / B
ENC_MAP = {"L_hip_enc": "enc2", "R_hip_enc": "enc1"}
IMU_SOURCE = {"pelvis": "SPI IMU", "L_shank": "UART IMU A", "R_shank": "UART IMU B"}

ENC_COUNTS_PER_REV = 4096  # AS5145 12 bit
ENC_COMM_ERR = 0x27        # main.cpp parse_encoder：同位 / OCF / COF / 起始 bit
IMU_STALE_S = 0.025        # IMU 最新封包超過這麼久沒更新 (掉 2 包以上) 才寫 NaN
FOOT_N = 18
NAN = "NaN"

# 18 個足底感測點的幾何中心 (mm，相對 FOOT_ORIGIN)，與 main.cpp 的 FOOT_X / FOOT_Y 相同
FOOT_X = [1943.9325, 1940.0133, 1973.6715, 1972.9752, 1965.7372, 1957.7049, 1963.0637, 1955.6792, 1995.8721,
          1996.5690, 1983.1920, 1980.9491, 1982.3130, 1970.9490, 2000.9966, 2003.7015, 2001.4963, 1988.7695]
FOOT_Y = [2600.1604, 2637.9524, 2444.2083, 2482.2769, 2521.7471, 2561.9473, 2599.0265, 2642.2744, 2444.2212,
          2482.2769, 2520.6611, 2559.8462, 2599.0265, 2641.6030, 2521.0309, 2560.0963, 2597.5737, 2633.6218]
FOOT_ORIGIN = (1984.7721, 2415.3761)


def read_csv(path):
    with open(path, newline="", encoding="utf-8") as f:
        r = csv.DictReader(f)
        return r.fieldnames or [], [row for row in r if None not in row.values()]


def fmt(v, nd):
    return NAN if v is None else f"{v:.{nd}f}"


def euler_to_quat(roll, pitch, yaw):
    """Xsens Euler (deg, Z-Y-X) → (w, x, y, z)。"""
    r, p, y = (math.radians(a) / 2 for a in (roll, pitch, yaw))
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return (cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy)


def convert_main(rows, fields, t0):
    imus = {name: pre for name, pre in IMU_MAP.items() if f"{pre}ax" in fields}
    for name in IMU_MAP:
        if name not in imus:
            print(f"注意：{os.path.basename(args.run)} 沒有 {IMU_SOURCE[name]} 的欄位，{name} 不輸出", file=sys.stderr)
    head = ["time"]
    for name in imus:
        head += [f"{name}_time"] + [f"{name}_acc_{a}" for a in "xyz"] + [f"{name}_gyr_{a}" for a in "xyz"]
        head += [f"{name}_quat_{a}" for a in "wxyz"] + [f"{name}_roll", f"{name}_pitch", f"{name}_yaw"]
    head += ["enc_time"]
    for name in ENC_MAP:
        head += [name, f"{name}_err"]
    head += ["phase"]

    out, stats = [], {"imu_nan": {n: 0 for n in imus}, "enc_nan": {n: 0 for n in ENC_MAP}}
    for row in rows:
        line = [fmt(float(row["t_s"]) - t0, 6)]
        for name, pre in imus.items():
            t_col = f"{pre}t_s" if pre else "imu_t_s"
            new_col = f"{pre}new" if pre else "imu_new"
            # new = 0 常常只是 IMU 跟 MCU 的 100 Hz 剛好對在週期邊界 (下一週期就收到 2 包)，值只舊了 <10 ms；
            # 最新封包超過 IMU_STALE_S 沒更新才當作缺值
            t_imu = float(row.get(t_col) or 0)
            if t_imu == 0 or float(row["t_s"]) - t_imu > IMU_STALE_S:
                stats["imu_nan"][name] += 1
                line += [NAN] * 14
                continue
            g = lambda k: float(row[f"{pre}{k}"])
            roll, pitch, yaw = (g("roll"), g("pitch"), g("yaw"))
            line += [fmt(float(row[t_col]) - t0, 6)]
            line += [fmt(g(k), 4) for k in ("ax", "ay", "az")] + [fmt(g(k), 3) for k in ("gx", "gy", "gz")]
            line += [fmt(q, 6) for q in euler_to_quat(roll, pitch, yaw)]
            line += [fmt(roll, 3), fmt(pitch, 3), fmt(yaw, 3)]
        line += [fmt(float(row["enc_t_s"]) - t0, 6) if row.get("enc_t_s") else NAN]
        for name, src in ENC_MAP.items():
            err = int(row[f"{src}_err"])
            if err & ENC_COMM_ERR:
                stats["enc_nan"][name] += 1
                line += [NAN, str(err)]
            else:
                line += [row[f"{src}_raw"], str(err)]
        line += [row.get("phase", "")]
        out.append(line)
    return head, out, stats


def convert_foot(rows, t0):
    head = ["time", "foot", "esp_seq"] + [f"L_fsr_{i}" for i in range(1, FOOT_N + 1)]
    head += [f"R_fsr_{i}" for i in range(1, FOOT_N + 1)] + ["phase"]
    out, n = [], {"L": 0, "R": 0}
    t_first, t_last = {}, {}
    for row in rows:
        side = row["foot"]
        t = float(row["t_rx_s"]) - t0
        p = [row[f"P{i}"] for i in range(1, FOOT_N + 1)]
        blank = [""] * FOOT_N
        line = [fmt(t, 6), side, row.get("esp_seq", "")] + (p + blank if side == "L" else blank + p) + [row["phase"]]
        out.append(line)
        n[side] += 1
        t_first.setdefault(side, t)
        t_last[side] = t
    rate = {s: round((n[s] - 1) / (t_last[s] - t_first[s]), 1) if n[s] > 1 else None for s in n}
    return head, out, rate


def write_csv(path, head, rows):
    with open(path, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f, lineterminator="\n")
        w.writerow(head)
        w.writerows(rows)


def origin_interval(path, t0):
    if not os.path.exists(path):
        return None
    _, rows = read_csv(path)
    ts = [(float(r["t_first_s"]), float(r["t_last_s"])) for r in rows if r.get("t_first_s")]
    if not ts:
        return None
    return [round(min(a for a, _ in ts) - t0, 3), round(max(b for _, b in ts) - t0, 3)]


ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
ap.add_argument("run", help="runXXX.csv")
ap.add_argument("--subject", required=True, help="受試者編號，例如 S01")
ap.add_argument("--trial", required=True, help="趟次，例如 T01")
ap.add_argument("--out", default="data", help="輸出的 data 資料夾 (預設 ./data)")
ap.add_argument("--date", default=None, help="實驗日期 YYYY-MM-DD (SD 卡檔案沒有正確日期)")
args = ap.parse_args()

base = args.run[:-4] if args.run.endswith(".csv") else args.run
fields, rows = read_csv(args.run)
if not rows:
    sys.exit(f"{args.run} 沒有資料")
for need in ("t_s", "enc1_raw", "enc2_raw", "enc1_err", "enc2_err"):
    if need not in fields:
        sys.exit(f"{args.run} 缺少欄位 {need}，不是 enc_imu_sd_logger 的格式？")
t0 = float(rows[0]["t_s"])
tag = f"{args.subject}_{args.trial}"
raw_dir = os.path.join(args.out, args.subject, "raw")
os.makedirs(raw_dir, exist_ok=True)

head, out, stats = convert_main(rows, fields, t0)
main_path = os.path.join(raw_dir, f"{tag}_raw.csv")
write_csv(main_path, head, out)
dt = [float(rows[i + 1]["t_s"]) - float(rows[i]["t_s"]) for i in range(len(rows) - 1)]
dropped = int(rows[-1].get("dropped") or 0)
print(f"{main_path}: {len(out)} 列, {len(out) / (float(rows[-1]['t_s']) - t0):.1f} Hz, "
      f"最大間隔 {max(dt) * 1000:.0f} ms, dropped {dropped}")
for name, k in stats["imu_nan"].items():
    print(f"  {name}: {k} 列沒有新封包 → NaN")
for name, k in stats["enc_nan"].items():
    print(f"  {name}: {k} 列通訊錯誤 → NaN")

fsr_rate = None
foot_path = base + "_foot.csv"
if os.path.exists(foot_path):
    _, frows = read_csv(foot_path)
    fhead, fout, fsr_rate = convert_foot(frows, t0)
    fsr_out = os.path.join(raw_dir, f"{tag}_raw_fsr.csv")
    write_csv(fsr_out, fhead, fout)
    print(f"{fsr_out}: {len(fout)} 列, L {fsr_rate['L']} Hz, R {fsr_rate['R']} Hz")
else:
    print(f"注意：找不到 {foot_path}，沒有足底資料", file=sys.stderr)

meta_path = os.path.join(args.out, args.subject, f"{tag}_meta.json")
static = origin_interval(base + "_origin.csv", t0)
notes = [f"source: {os.path.basename(args.run)}", f"dropped samples: {dropped}"]
notes += [f"{n}: {k} rows encoder comm error (NaN)" for n, k in stats["enc_nan"].items() if k]
meta = {
    "subject": args.subject,
    "trial": args.trial,
    "date": args.date,
    "height_cm": None,
    "weight_kg": None,
    "treadmill_speed_mps": None,
    "condition": None,
    "sample_rate_hz": {"imu": 100, "encoder": 100, "fsr": fsr_rate},
    "imu_range": {"acc_g": None, "gyr_dps": None},
    "encoder_counts_per_rev": ENC_COUNTS_PER_REV,
    "fsr_layout": [f"P{i + 1} ({FOOT_X[i] - FOOT_ORIGIN[0]:.1f}, {FOOT_Y[i] - FOOT_ORIGIN[1]:.1f}) mm"
                   for i in range(FOOT_N)],
    "fsr_calibration": None,
    "imu_mounting": {name: f"{IMU_SOURCE[name]}，座標軸朝向待填" for name in IMU_MAP},
    "encoder_mounting": {name: src for name, src in ENC_MAP.items()},
    "static_trial_s": static,
    "sync_event_time_s": {"exo": None, "mocap": None},
    "angle_convention": None,
    "notes": "; ".join(notes),
}
if os.path.exists(meta_path):
    print(f"{meta_path} 已存在，不覆蓋 (要重新產生請先刪掉)")
else:
    with open(meta_path, "w", encoding="utf-8") as f:
        json.dump(meta, f, ensure_ascii=False, indent=2)
        f.write("\n")
    todo = [k for k, v in meta.items() if v is None]
    print(f"{meta_path}: 還要手動填 {', '.join(todo)}，以及 imu_range、imu_mounting 的軸向、sync_event_time_s.exo")
