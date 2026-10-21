"""阿里云 CosyVoice TTS -> PM01 Jetson 本地扬声器播放。

默认使用流式播放：DashScope 回调一返回第一块 MP3，就写入 mpg123 的
stdin 开始出声，不必等整段合成完再转 WAV。系统没有 mpg123 时自动退回
原来的“整段合成 + ffmpeg + aplay”路径。
"""

import os
import queue
import shutil
import threading
import time

import dashscope
from dashscope.audio.tts_v2 import ResultCallback, SpeechSynthesizer

from .audio_io import SpeakerPlayer

# 播放线程的结束哨兵
_STREAM_END = object()


class _TTSStreamCallback(ResultCallback):

    def __init__(self, chunks):
        self.chunks = chunks
        self.first_data_at = None
        self.error = None
        self.complete = threading.Event()
        self.opened = threading.Event()

    def on_open(self):
        self.opened.set()

    def on_data(self, data):
        if not data:
            return
        if self.first_data_at is None:
            self.first_data_at = time.monotonic()
        self.chunks.put(data)

    def on_complete(self):
        self.complete.set()

    def on_close(self):
        self.complete.set()

    def on_error(self, message):
        self.error = str(message)
        self.complete.set()


class TTSClient:
    """文字 -> CosyVoice -> 本机扬声器。"""

    def __init__(
        self,
        node,
        model='cosyvoice-v2',
        voice='longshu_v2',
        streaming=True,
    ):
        self.logger = node.get_logger()

        self.player = SpeakerPlayer()

        self.model = model
        self.voice = voice

        # 流式播放依赖 mpg123；没有就退回整段播放
        self.streaming = bool(streaming) and shutil.which('mpg123') is not None

        self._synth_lock = threading.Lock()
        self._synth = None
        self._stream_stop = None

    def stop(self):
        """立即停止当前 TTS 合成与本地播放。"""
        with self._synth_lock:
            stop_event = self._stream_stop
            synth = self._synth

        if stop_event is not None:
            stop_event.set()

        self.player.stop()

        # 取消服务端剩余合成，避免打断后还在后台收音频。
        # streaming_cancel 会等 task-finished，不能堵调用线程。
        if synth is not None:
            def _cancel():
                try:
                    synth.streaming_cancel(complete_timeout_millis=300)
                except Exception:
                    pass

            threading.Thread(target=_cancel, daemon=True).start()

    def speak(self, text: str) -> float:
        text = text.strip()

        if not text:
            return 0.0

        api_key = os.getenv('DASHSCOPE_API_KEY')

        if not api_key:
            self.logger.error('❌ 没有设置 DASHSCOPE_API_KEY')
            return 0.0

        if self.streaming:
            return self._speak_streaming(text, api_key)

        return self._speak_blocking(text, api_key)

    # --------------------------------------------------------
    # 流式：首包即播
    # --------------------------------------------------------

    def _speak_streaming(self, text, api_key):
        for attempt in (1, 2):

            chunks = queue.Queue()
            callback = _TTSStreamCallback(chunks)
            stop_event = threading.Event()
            player_done = threading.Event()

            with self._synth_lock:
                self._stream_stop = stop_event

            def _player_worker():
                started = False
                fallback = bytearray()

                while True:
                    data = chunks.get()

                    if data is _STREAM_END:
                        break

                    if stop_event.is_set():
                        # 已被新的唤醒/指令打断，丢弃剩余音频
                        continue

                    if not started:
                        if self.player.start_mp3_stream():
                            started = True
                            self.player.write_mp3_stream(data)
                        else:
                            # mpg123 起不来：先缓存，最后退回整段播放
                            fallback.extend(data)
                    else:
                        self.player.write_mp3_stream(data)

                if started:
                    self.player.finish_mp3_stream()
                elif fallback:
                    try:
                        self.player.play_mp3(bytes(fallback))
                    except Exception as e:
                        self.logger.warning(f'↻ TTS 回退播放失败: {e}')

                player_done.set()

            player_thread = threading.Thread(
                target=_player_worker, daemon=True)
            player_thread.start()

            synthesizer = None
            error = None
            t_start = time.monotonic()

            try:
                dashscope.api_key = api_key

                self.logger.info(
                    f'🔊 CosyVoice 流式合成: {text[:60]}')

                synthesizer = SpeechSynthesizer(
                    model=self.model,
                    voice=self.voice,
                    callback=callback,
                )

                with self._synth_lock:
                    self._synth = synthesizer

                # 注意：callback 模式下 call() 在 WebSocket 建立后就返回，
                # 不会等音频收完。必须等 on_complete/on_error，
                # 期间 _player_worker 已经在消费首包并出声。
                synthesizer.call(text)

                deadline = time.monotonic() + 30.0
                while not callback.complete.is_set():
                    # 唤醒词打断后不再死等合成收尾
                    if stop_event.is_set():
                        break
                    if time.monotonic() > deadline:
                        raise RuntimeError('TTS 合成超时')
                    callback.complete.wait(0.05)

                if stop_event.is_set():
                    # 被打断：不报错、不重试，播完当前已缓冲的部分即返回
                    pass
                elif callback.error:
                    raise RuntimeError(callback.error)
                elif callback.first_data_at is None:
                    raise RuntimeError('TTS 没有返回音频')

            except Exception as e:
                error = e

            finally:
                with self._synth_lock:
                    if self._synth is synthesizer:
                        self._synth = None
                    if self._stream_stop is stop_event:
                        self._stream_stop = None
                chunks.put(_STREAM_END)

            if error is None:
                if callback.first_data_at is not None:
                    first_audio = callback.first_data_at - t_start
                    self.logger.info(
                        f'⏱ TTS 首音 {first_audio:.2f}s'
                        f'（合成总耗时 {time.monotonic() - t_start:.2f}s）'
                    )
                player_done.wait()
                return 0.0

            # 还没出过声才重试，避免同一句话重复播放
            if (attempt == 1
                    and callback.first_data_at is None
                    and not stop_event.is_set()):
                self.logger.warning(
                    f'↻ TTS连接失败，1秒后重试: '
                    f'{type(error).__name__}: {error}')
                player_done.wait(timeout=1.0)
                time.sleep(1.0)
                continue

            if not stop_event.is_set():
                self.logger.error(
                    f'❌ TTS失败: {type(error).__name__}: {error}')
            player_done.wait()
            return 0.0

    # --------------------------------------------------------
    # 兼容路径：整段合成 + ffmpeg + aplay
    # --------------------------------------------------------

    def _speak_blocking(self, text, api_key):
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

        return 0.0
