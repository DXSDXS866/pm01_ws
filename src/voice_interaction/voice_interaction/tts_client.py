"""阿里云 CosyVoice TTS -> PM01 Jetson 本地扬声器播放。"""

import os
import time

import dashscope
from dashscope.audio.tts_v2 import SpeechSynthesizer

from .audio_io import SpeakerPlayer


class TTSClient:
    """文字 -> CosyVoice -> MP3 -> 本机扬声器。"""

    def __init__(
        self,
        node,
        model='cosyvoice-v2',
        voice='longshu_v2',
    ):
        self.logger = node.get_logger()

        self.player = SpeakerPlayer()

        self.model = model
        self.voice = voice

    def stop(self):
        """立即停止当前本地 TTS 播放。"""
        self.player.stop()

    def speak(self, text: str) -> float:
        text = text.strip()

        if not text:
            return 0.0

        api_key = os.getenv('DASHSCOPE_API_KEY')

        if not api_key:
            self.logger.error('❌ 没有设置 DASHSCOPE_API_KEY')
            return 0.0

        # ----------------------------------------------------
        # 网络抖动时 dashscope websocket 握手硬超时只有 5 秒，
        # 失败后重试一次，避免演示时偶发 TTS 报错
        # ----------------------------------------------------

        for attempt in (1, 2):

            try:

                dashscope.api_key = api_key

                self.logger.info(
                    f'🔊 CosyVoice 开始合成: {text[:60]}'
                )

                synthesizer = SpeechSynthesizer(
                    model=self.model,
                    voice=self.voice,
                )

                audio = synthesizer.call(text)

                break

            except Exception as e:

                if attempt == 2:

                    self.logger.error(
                        f'❌ TTS失败: '
                        f'{type(e).__name__}: {e}'
                    )

                    return 0.0

                self.logger.warning(
                    f'↻ TTS连接失败，1秒后重试: '
                    f'{type(e).__name__}: {e}'
                )

                time.sleep(1.0)

        if not audio:

            self.logger.error('❌ CosyVoice 返回空音频')

            return 0.0

        self.logger.info(
            f'✅ TTS 合成成功: {len(audio)} bytes'
        )

        self.logger.info('🔊 开始本地播放...')

        self.player.play_mp3(audio)

        self.logger.info('✅ TTS 播放结束')

        return