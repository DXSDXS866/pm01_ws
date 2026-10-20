"""基于短时能量的 VAD，检测说话起止。

可用于持续监听场景：检测到说话开始→启动ASR，静音超时→判定一句话结束。
可选引入 webrtcvad 做更精准的判定（pip install webrtcvad）。
"""
import numpy as np

try:
    import webrtcvad  # type: ignore
    _HAS_WEBRTC = True
except ImportError:
    _HAS_WEBRTC = False


class SimpleVAD:
    """能量阈值 + 静音超时断句。

    Args:
        energy_threshold: 短时 RMS 能量阈值，高于视为有人说话。麦克风增益和环境
            噪声不同需要现场调（先录几秒环境噪声看 RMS，阈值设在其 2~3 倍）。
        silence_duration_ms: 句尾静音时长，超过则判定一句话结束（默认 800ms）。
        sample_rate / chunk_ms: 与麦克风采集参数对齐。
    """

    def __init__(self, energy_threshold=500, silence_duration_ms=800,
                 sample_rate=16000, chunk_ms=100, use_webrtc=False):
        self.energy_threshold = energy_threshold
        self.silence_frames = max(1, silence_duration_ms // chunk_ms)
        self._silence_count = 0
        self._speaking = False
        self._use_webrtc = use_webrtc and _HAS_WEBRTC
        if self._use_webrtc:
            # aggressiveness 0~3，3 最激进
            self._webrtc = webrtcvad.Vad(3)
        self._chunk_samples = int(sample_rate * chunk_ms / 1000)

    def process(self, chunk: bytes) -> str:
        """返回 'speaking' / 'silence' / 'end_of_utterance'。"""
        audio = np.frombuffer(chunk, dtype=np.int16).astype(np.float32)
        energy = float(np.sqrt(np.mean(audio ** 2))) if len(audio) else 0.0
        is_voice_energy = energy > self.energy_threshold

        if self._use_webrtc and len(audio) >= 480:
            is_voice_webrtc = self._is_speech_webrtc(chunk)
            is_voice = is_voice_energy or is_voice_webrtc
        else:
            is_voice = is_voice_energy

        if is_voice:
            self._silence_count = 0
            self._speaking = True
            return 'speaking'
        else:
            if self._speaking:
                self._silence_count += 1
                if self._silence_count >= self.silence_frames:
                    self._speaking = False
                    self._silence_count = 0
                    return 'end_of_utterance'
                return 'speaking'
            return 'silence'

    def _is_speech_webrtc(self, chunk: bytes) -> bool:
        """按 30ms 帧送检 webrtcvad（只接受 10/20/30ms 帧），过半帧为语音才算有人说话。

        异常时返回 False，避免误判成"一直说话"导致 end_of_utterance 永不触发。
        """
        frame_bytes = int(16000 * 0.03) * 2  # 30ms * 16kHz * 16bit 单声道
        try:
            votes = 0
            total = 0
            for i in range(0, len(chunk) - frame_bytes + 1, frame_bytes):
                if self._webrtc.is_speech(chunk[i:i + frame_bytes], 16000):
                    votes += 1
                total += 1
            return total > 0 and votes * 2 > total
        except Exception:
            return False

    def reset(self):
        self._silence_count = 0
        self._speaking = False
