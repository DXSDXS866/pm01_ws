#!/usr/bin/env python3
"""VAD 能量阈值调参工具：录 N 秒，算 RMS，给能量阈值建议。

在机器人计算单元上运行（先安静不说话，再正常说话）：
  python3 tune_vad.py --seconds 3          # 测环境噪声（安静）
  python3 tune_vad.py --seconds 3 --talk  # 测说话音量

输出会给出推荐的 energy_threshold（设为其 1.5~2 倍），写入 config/params.yaml。
"""
import argparse
import numpy as np
import pyaudio

CHUNK = 3200
SR = 16000


def record_rms(seconds: int, talk: bool):
    pa = pyaudio.PyAudio()
    dev = pa.get_default_input_device_info()
    print(f"使用设备 [{dev['index']}] {dev['name']}")
    stream = pa.open(format=pyaudio.paInt16, channels=1, rate=SR,
                     input=True, frames_per_buffer=CHUNK)
    print(f"{'说话' if talk else '保持安静'}，录制 {seconds} 秒...")
    rms_list = []
    for _ in range(int(seconds * SR / CHUNK)):
        data = stream.read(CHUNK, exception_on_overflow=False)
        audio = np.frombuffer(data, dtype=np.int16).astype(np.float32)
        rms = float(np.sqrt(np.mean(audio ** 2)))
        rms_list.append(rms)
    stream.close()
    pa.terminate()

    rms_arr = np.array(rms_list)
    print(f"\nRMS 统计：")
    print(f"  最小 {rms_arr.min():.0f}  最大 {rms_arr.max():.0f}  均值 {rms_arr.mean():.0f}")
    if talk:
        rec = rms_arr.mean()
        print(f"  → 说话 RMS 均值 ≈ {rec:.0f}")
        print(f"  → 建议 energy_threshold = {int(rec * 0.5)}~{int(rec * 0.7)}")
    else:
        noise = rms_arr.mean()
        print(f"  → 环境噪声 RMS 均值 ≈ {noise:.0f}")
        print(f"  → 建议 energy_threshold = {int(noise * 2)}~{int(noise * 3)}")
    print(f"\n写入 config/params.yaml 的 energy_threshold 即可。")


if __name__ == '__main__':
    p = argparse.ArgumentParser()
    p.add_argument('--seconds', type=int, default=3)
    p.add_argument('--talk', action='store_true', help='说话测试')
    args = p.parse_args()
    record_rms(args.seconds, args.talk)
