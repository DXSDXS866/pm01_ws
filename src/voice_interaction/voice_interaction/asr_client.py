"""阿里云 Paraformer 流式语音识别（ASR）。"""

import os
import threading

import dashscope
from dashscope.audio.asr import (
    Recognition,
    RecognitionCallback,
    RecognitionResult,
)

dashscope.api_key = os.getenv('DASHSCOPE_API_KEY')


class ASRClient:
    """一次会话对应一次 start/stop。"""

    def __init__(self):
        self._partial = ''
        self._finals = []
        self._lock = threading.Lock()
        self._complete = threading.Event()
        self._recognition = None
        self._cb = _ASRCallback(self)

    def start(self):
        # 每次启动重新读环境变量，兼容服务化部署时后配 key 的场景
        api_key = os.getenv('DASHSCOPE_API_KEY')
        if not api_key:
            raise RuntimeError('未设置 DASHSCOPE_API_KEY 环境变量')
        dashscope.api_key = api_key

        with self._lock:
            self._partial = ''
            self._finals = []
        self._complete.clear()

        self._recognition = Recognition(
            model='paraformer-realtime-v2',
            format='pcm',
            sample_rate=16000,
            language_hints=['zh', 'en'],
            callback=self._cb,
        )

        self._recognition.start()

    def send_audio(self, pcm_chunk: bytes):
        """发送 16kHz / 16bit / mono PCM 音频。"""
        if self._recognition is not None:
            self._recognition.send_audio_frame(pcm_chunk)

    def stop(self) -> str:
        """停止识别并返回最终文字。"""
        rec = self._recognition
        self._recognition = None

        if rec is not None:
            try:
                rec.stop()
            except Exception as e:
                print(f'[ASR STOP ERROR] {e}', flush=True)
            # 等服务端把剩余结果（含最后一句 sentence_end）回传完，
            # 否则可能丢掉最后一句。与官方 CLI 实现一致。
            self._complete.wait(timeout=5.0)

        with self._lock:
            return (''.join(self._finals) + self._partial).strip()

    def get_partial(self) -> str:
        """返回当前累计的全部文字（已完成句 + 当前进行中的句子）。

        覆盖整个会话的累计文本，便于在机器人说话时检测唤醒词打断。
        """
        with self._lock:
            return ''.join(self._finals) + self._partial


class _ASRCallback(RecognitionCallback):

    def __init__(self, parent):
        self.parent = parent

    def on_open(self):
        print('[ASR] WebSocket 已连接', flush=True)

    def on_complete(self):
        print('[ASR] 识别任务完成', flush=True)
        self.parent._complete.set()

    def on_error(self, result):
        print(f'[ASR ERROR] {result}', flush=True)
        self.parent._complete.set()

    def on_close(self):
        print('[ASR] WebSocket 已关闭', flush=True)
        self.parent._complete.set()

    def on_event(self, result: RecognitionResult):
        try:
            sentence = result.get_sentence()

            print(f'[ASR EVENT] {sentence!r}', flush=True)

            if not isinstance(sentence, dict):
                return

            text = str(sentence.get('text') or '').strip()

            if not text:
                return

            print(f'[ASR TEXT] {text}', flush=True)

            with self.parent._lock:
                # 一句话结束：把完成的句子累积进列表，并清空当前句缓存；
                # 否则只更新当前句的实时 partial。
                #
                # 关键：完成的句子必须【追加】而不是覆盖，
                # 否则多句会话只会保留最后一句（旧 bug）。
                if RecognitionResult.is_sentence_end(sentence):
                    self.parent._finals.append(text)
                    self.parent._partial = ''
                else:
                    self.parent._partial = text

        except Exception as e:
            print(
                f'[ASR CALLBACK ERROR] {type(e).__name__}: {e}',
                flush=True,
            )
