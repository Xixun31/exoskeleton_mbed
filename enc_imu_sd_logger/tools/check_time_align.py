#!/usr/bin/env python3
"""檢查 enc_imu_sd_logger 紀錄檔的時間對齊。

用法:
    python3 check_time_align.py run001.csv            # 自動找同資料夾的 run001_foot.csv
    python3 check_time_align.py run001.csv --plot     # 另外存一張 run001_time_align.png

所有 *_t_s / t_rx_s 都是同一個 MCU µs 時鐘，可以直接相減。
IMU 的 imu_stf (0.1 ms) 與 ESP32 的 esp_us 是各自裝置的時鐘，這裡用直線擬合
(MCU 時間 = a × 裝置時間 + b) 算出時鐘漂移 (ppm) 與延遲抖動 (殘差)。
只需要 numpy；--plot 需要 matplotlib。
"""
import argparse
import csv
import os
import sys

import numpy as np


def load_csv(path):
    with open(path, newline="") as f:
        rows = list(csv.reader(f))
    if not rows:
        return {}
    head, body = rows[0], [r for r in rows[1:] if len(r) == len(rows[0])]
    cols = {}
    for i, name in enumerate(head):
        vals = [r[i] for r in body]
        try:
            cols[name] = np.array([float(v) if v != "" else np.nan for v in vals])
        except ValueError:
            cols[name] = np.array(vals)
    return cols


def unwrap(x, bits):
    """把會繞回的計數器 (例如 32 bit µs) 展開成遞增的序列。"""
    x = np.asarray(x, dtype=np.float64)
    if len(x) == 0:
        return x
    period = float(2**bits)
    d = np.diff(x)
    jumps = np.concatenate([[0.0], np.cumsum(np.where(d < -period / 2, period, 0.0))])
    return x + jumps


def stats_ms(x_s):
    x = np.asarray(x_s) * 1e3
    if len(x) == 0:
        return "(沒有資料)"
    return f"平均 {x.mean():.3f} ms, 標準差 {x.std():.3f} ms, 最小 {x.min():.3f} ms, 最大 {x.max():.3f} ms"


def clock_fit(dev_s, mcu_s):
    """MCU 時間 = a × 裝置時間 + b。回傳 (漂移 ppm, 殘差秒, 斜率, 截距)。"""
    a, b = np.polyfit(dev_s, mcu_s, 1)
    resid = mcu_s - (a * dev_s + b)
    return (a - 1.0) * 1e6, resid, a, b


def report_main(m):
    print("=" * 70)
    print("主檔 (encoder + IMU)")
    print("=" * 70)
    n = len(m["cycle"])
    t = m["t_s"]
    dur = t[-1] - t[0] if n > 1 else 0
    print(f"行數 {n}，時間長度 {dur:.2f} 秒，平均 {(n - 1) / dur if dur else 0:.2f} Hz")
    gaps = np.diff(m["cycle"])
    print(f"cycle 跳號: {int(np.sum(gaps != 1))} 次 (少了 {int(np.sum(gaps[gaps > 1] - 1))} 個週期)；"
          f"dropped (SD 太慢丟掉) 最後值 = {int(m['dropped'][-1])}")
    print(f"週期間隔 (t_s): {stats_ms(np.diff(t))}")

    if "enc_t_s" in m:
        enc_t = m["enc_t_s"]
        print(f"encoder 取樣間隔 (enc_t_s): {stats_ms(np.diff(enc_t))}")

    if "imu_t_s" in m:
        new = m["imu_new"] > 0
        print(f"IMU: {int(new.sum())}/{n} 行有新封包，{int(np.sum(m['imu_new'] > 1))} 行一次讀到 2 包以上 (舊的被覆蓋)，"
              f"錯誤封包 {int(np.nansum(m['imu_bad']))}")
        imu_t = m["imu_t_s"][new]
        if len(imu_t) > 2:
            print(f"IMU 封包間隔 (imu_t_s): {stats_ms(np.diff(imu_t))}")
            pc = m["imu_pc"][new]
            dpc = np.mod(np.diff(pc), 65536)
            print(f"IMU PacketCounter 跳號: {int(np.sum(dpc != 1))} 次 (少了 {int(np.sum(dpc[dpc > 1] - 1))} 包)")
            stf = unwrap(m["imu_stf"][new], 32) * 1e-4
            if np.ptp(stf) > 0:
                ppm, resid, _, _ = clock_fit(stf, imu_t)
                print(f"IMU 時鐘 vs MCU 時鐘: 漂移 {ppm:+.1f} ppm；MCU 收到 IMU 封包的時間抖動 "
                      f"(殘差) 標準差 {resid.std() * 1e3:.3f} ms, 最大 {np.abs(resid).max() * 1e3:.3f} ms")
        if "enc_t_s" in m:
            d = (m["enc_t_s"] - m["imu_t_s"])[new]
            print(f"同一行 encoder 取樣 - IMU 封包時間: {stats_ms(d)}")
    print()


def report_foot(f, m, baud):
    print("=" * 70)
    print("足底檔")
    print("=" * 70)
    pkt_ms = 47 * 10 / baud * 1e3
    print(f"(UART {baud} baud 傳一包 47 bytes 約 {pkt_ms:.2f} ms；t_rx_s 是最後一個 byte 收到的時間)")
    result = {}
    for side in ("L", "R"):
        sel = f["foot"] == side
        cnt = int(sel.sum())
        if cnt == 0:
            print(f"[{side}] 沒有資料")
            continue
        t = f["t_rx_s"][sel]
        dur = t[-1] - t[0] if cnt > 1 else 0
        print(f"[{side}] {cnt} 包，平均 {(cnt - 1) / dur if dur else 0:.2f} Hz")
        if cnt > 1:
            print(f"[{side}] STM32 收到間隔 (t_rx_s): {stats_ms(np.diff(t))}")
        seq = f["seq"][sel]
        print(f"[{side}] STM32 編號 seq 跳號: {int(np.sum(np.diff(seq) != 1))} 次 "
              "(只在 STM32 佇列滿時發生)")
        esp = f["esp_us"][sel]
        has = ~np.isnan(esp)
        if has.sum() > 2:
            eseq = f["esp_seq"][sel][has]
            de = np.mod(np.diff(eseq), 65536)
            print(f"[{side}] ESP32 編號 esp_seq 跳號: {int(np.sum(de != 1))} 次 (少了 {int(np.sum(de[de > 1] - 1))} 包，"
                  "= ESP32→STM32 UART 掉包或 checksum 錯)")
            esp_s = unwrap(esp[has], 32) * 1e-6
            print(f"[{side}] ESP32 收到 BLE 的間隔 (esp_us): {stats_ms(np.diff(esp_s))}")
            ppm, resid, a, b = clock_fit(esp_s, t[has])
            print(f"[{side}] ESP32 時鐘 vs MCU 時鐘: 漂移 {ppm:+.1f} ppm；ESP32→STM32 延遲抖動 (殘差) "
                  f"標準差 {resid.std() * 1e3:.3f} ms, 最大 {np.abs(resid).max() * 1e3:.3f} ms")
            result[side] = (esp_s, t[has], resid)
        else:
            print(f"[{side}] 沒有 ESP32 時間戳 (ESP32 用舊程式或 SEND_TIMESTAMP = false)")
        # 對照主檔：每包足底資料距離最近的 encoder 取樣差多少
        if m is not None and "enc_t_s" in m:
            enc_t = m["enc_t_s"]
            tin = t[(t >= enc_t[0]) & (t <= enc_t[-1])]  # 只看主檔紀錄期間
            idx = np.clip(np.searchsorted(enc_t, tin), 1, len(enc_t) - 1)
            near = np.where(np.abs(enc_t[idx] - tin) < np.abs(enc_t[idx - 1] - tin), idx, idx - 1)
            d = tin - enc_t[near]
            print(f"[{side}] 足底收到時間 - 最近的 encoder 取樣: {stats_ms(d)} "
                  "(主檔 100 Hz，正常在 ±5 ms 內；超過代表主檔那段有缺週期)")
    if m is not None and "footL_cnt" in m:
        for side, col in (("L", "footL_cnt"), ("R", "footR_cnt")):
            last = int(m[col][-1]) if len(m[col]) else 0
            got = int(np.max(f["seq"][f["foot"] == side])) if np.any(f["foot"] == side) else 0
            print(f"[{side}] 主檔最後 {col} = {last}，足底檔最大 seq = {got}")
    print()
    return result


def plot(m, f, foot_fit, out):
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(4, 1, figsize=(11, 12), sharex=True)
    t0 = m["t_s"][0] if m is not None else 0
    if m is not None:
        ax[0].plot(m["t_s"][1:] - t0, np.diff(m["t_s"]) * 1e3, ".", ms=2, label="cycle (t_s)")
        if "imu_t_s" in m:
            new = m["imu_new"] > 0
            it = m["imu_t_s"][new]
            ax[0].plot(it[1:] - t0, np.diff(it) * 1e3, ".", ms=2, label="IMU (imu_t_s)")
        ax[0].set_ylabel("interval [ms]")
        ax[0].set_title("encoder / IMU sample interval")
        ax[0].legend()
        if "imu_t_s" in m:
            stf = unwrap(m["imu_stf"][new], 32) * 1e-4
            if len(stf) > 2 and np.ptp(stf) > 0:
                _, resid, _, _ = clock_fit(stf, it)
                ax[1].plot(it - t0, resid * 1e3, ".", ms=2)
        ax[1].set_ylabel("residual [ms]")
        ax[1].set_title("IMU: MCU timestamp vs IMU clock (SampleTimeFine), after linear fit")
    if f is not None:
        for side in ("L", "R"):
            sel = f["foot"] == side
            t = f["t_rx_s"][sel]
            if len(t) > 1:
                ax[2].plot(t[1:] - t0, np.diff(t) * 1e3, ".", ms=3, label=f"foot {side}")
            if side in foot_fit:
                _, tr, resid = foot_fit[side]
                ax[3].plot(tr - t0, resid * 1e3, ".", ms=3, label=f"foot {side}")
        ax[2].set_ylabel("interval [ms]")
        ax[2].set_title("foot sensor receive interval (t_rx_s)")
        ax[2].legend()
        ax[3].set_ylabel("residual [ms]")
        ax[3].set_title("foot: ESP32 -> STM32 latency jitter (t_rx vs esp_us, after linear fit)")
        if foot_fit:
            ax[3].legend()
    ax[3].set_xlabel("MCU time [s]")
    for a in ax:
        a.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(out, dpi=120)
    print(f"圖存到 {out}")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("csv", help="runXXX.csv (主檔)")
    ap.add_argument("--foot", help="足底檔 (預設 runXXX_foot.csv)")
    ap.add_argument("--baud", type=int, default=115200, help="足底 UART 鮑率 (預設 115200)")
    ap.add_argument("--plot", action="store_true", help="存一張 runXXX_time_align.png")
    args = ap.parse_args()

    stem = args.csv[:-4] if args.csv.endswith(".csv") else args.csv
    m = load_csv(args.csv)
    if not m or len(m.get("cycle", [])) < 2:
        sys.exit(f"{args.csv} 沒有資料")
    report_main(m)

    foot_path = args.foot or stem + "_foot.csv"
    f = load_csv(foot_path) if os.path.exists(foot_path) else None
    foot_fit = {}
    if f and len(f.get("seq", [])) > 0:
        foot_fit = report_foot(f, m, args.baud)
    else:
        print(f"足底檔 {foot_path} 不存在或沒有資料")
        f = None

    if args.plot:
        plot(m, f, foot_fit, stem + "_time_align.png")


if __name__ == "__main__":
    main()
