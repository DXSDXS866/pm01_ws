#!/usr/bin/env python3
"""列出所有 pyaudio 音频设备，标出输入设备索引。

在机器人计算单元上运行：
  python3 list_audio_devices.py

输出形如：
  [0] USB Audio Device  (输入 2ch/16kHz)  ← 这是 mic_device_index
  [1] HDMI Output       (输出)
"""
import pyaudio

pa = pyaudio.PyAudio()
print(f"共 {pa.get_device_count()} 个设备：")
for i in range(pa.get_device_count()):
    info = pa.get_device_info_by_index(i)
    name = info['name']
    max_in = info['maxInputChannels']
    max_out = info['maxOutputChannels']
    sr = int(info['defaultSampleRate'])
    if max_in > 0:
        print(f"  [{i}] {name}  (输入 {max_in}ch/{sr}Hz)  ← 可作 mic_device_index")
    elif max_out > 0:
        print(f"  [{i}] {name}  (输出 {max_out}ch)")
    else:
        print(f"  [{i}] {name}  (无通道)")
pa.terminate()
