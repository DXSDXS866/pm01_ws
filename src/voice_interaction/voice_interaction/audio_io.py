
import os
import pyaudio
import subprocess
import tempfile
import threading


CHUNK = 1600
SAMPLE_RATE = 16000
CHANNELS = 1
FORMAT = pyaudio.paInt16


class MicrophoneRecorder:

    def __init__(self, device_index=None):
        self.pa = pyaudio.PyAudio()
        self._device_index = device_index
        self._stream = None

    def start(self):
        if self._stream is not None:
            return

        self._stream = self.pa.open(
            format=FORMAT,
            channels=CHANNELS,
            rate=SAMPLE_RATE,
            input=True,
            input_device_index=self._device_index,
            frames_per_buffer=CHUNK,
        )

    def read_chunk(self) -> bytes:
        if self._stream is None:
            return b'\x00' * CHUNK * 2

        return self._stream.read(
            CHUNK,
            exception_on_overflow=False,
        )

    def stop(self):
        if self._stream is not None:
            self._stream.stop_stream()
            self._stream.close()
            self._stream = None

    def close(self):
        self.stop()
        self.pa.terminate()


class SpeakerPlayer:

    def __init__(self):
        self._proc = None
        self._lock = threading.Lock()
        self._mp3_path = None
        self._wav_path = None
        self._interrupted = False

    # --------------------------------------------------------
    # 流式 MP3 播放
    #
    # mpg123 可以直接从 stdin 解码 MP3，TTS 收到第一块音频就能
    # 开始出声，不必等整段合成完再 ffmpeg 转 WAV。
    # --------------------------------------------------------

    def start_mp3_stream(self) -> bool:
        """启动 mpg123 流式播放；系统没有 mpg123 时返回 False。"""
        import shutil

        if shutil.which('mpg123') is None:
            return False

        # 先停掉正在播放的上一段（TTS 或内置 MP3）
        self.stop()
        self._interrupted = False

        try:
            proc = subprocess.Popen(
                ['mpg123', '-q', '-'],
                stdin=subprocess.PIPE,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
        except OSError as e:
            print(f'[Speaker] mpg123 启动失败: {e}', flush=True)
            return False

        with self._lock:
            self._proc = proc

        return True

    def write_mp3_stream(self, data: bytes) -> bool:
        """写入一块 MP3；进程已被打断/退出时返回 False。"""
        if not data:
            return True

        with self._lock:
            proc = self._proc
            interrupted = self._interrupted

        if proc is None or interrupted or proc.poll() is not None:
            return False

        try:
            proc.stdin.write(data)
            proc.stdin.flush()
            return True
        except (BrokenPipeError, OSError, ValueError):
            return False

    def finish_mp3_stream(self):
        """关闭 stdin，等 mpg123 把已缓冲的音频播完。

        等待期间保留 self._proc，这样唤醒词打断时 stop() 还能 kill
        正在出声的进程。
        """
        with self._lock:
            proc = self._proc
            interrupted = self._interrupted

        if proc is None:
            return

        try:
            if proc.stdin is not None:
                proc.stdin.close()
        except (BrokenPipeError, OSError, ValueError):
            pass

        if not interrupted:
            try:
                proc.wait()
            except Exception:
                pass

        with self._lock:
            if self._proc is proc:
                self._proc = None

    def play_mp3(self, mp3_bytes: bytes):
        self.stop()
        self._interrupted = False

        with tempfile.NamedTemporaryFile(
            suffix='.mp3',
            delete=False
        ) as f:
            f.write(mp3_bytes)
            self._mp3_path = f.name

        self._wav_path = self._mp3_path + '.wav'

        try:
            print(
                f'[Speaker] MP3保存: {self._mp3_path} '
                f'({len(mp3_bytes)} bytes)',
                flush=True,
            )

            print('[Speaker] ffmpeg 转 WAV...', flush=True)
            result = subprocess.run(
                [
                    'ffmpeg',
                    '-y',
                    '-loglevel', 'error',
                    '-i', self._mp3_path,
                    self._wav_path,
                ],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
            )

            if result.returncode != 0:
                raise RuntimeError(
                    'ffmpeg 转换失败:\n' + result.stderr
                )

            print(f'[Speaker] WAV生成成功: {self._wav_path}', flush=True)
            print('[Speaker] 调用 aplay 播放...', flush=True)

            proc = subprocess.Popen(['aplay', self._wav_path])
            with self._lock:
                self._proc = proc

            rc = proc.wait()

            if rc != 0 and not self._interrupted:
                raise RuntimeError(f'aplay 播放失败，返回码={rc}')

            if self._interrupted:
                print('[Speaker] 播放已被唤醒词打断', flush=True)
            else:
                print('[Speaker] 播放完成', flush=True)

        finally:
            with self._lock:
                if self._proc is not None and self._proc.poll() is not None:
                    self._proc = None
            self._cleanup()

    def stop(self):
        with self._lock:
            proc = self._proc
            if proc is None:
                return
            if proc.poll() is not None:
                self._proc = None
            else:
                self._interrupted = True

        if proc.poll() is None:
            try:
                proc.kill()
                proc.wait(timeout=1)
            except Exception:
                pass

        try:
            if proc.stdin is not None:
                proc.stdin.close()
        except (BrokenPipeError, OSError, ValueError):
            pass

        with self._lock:
            if self._proc is proc:
                self._proc = None

    def is_playing(self) -> bool:
        with self._lock:
            proc = self._proc
        return proc is not None and proc.poll() is None

    def _cleanup(self):
        for path in [self._mp3_path, self._wav_path]:
            if path and os.path.exists(path):
                try:
                    os.remove(path)
                except OSError:
                    pass

        self._mp3_path = None
        self._wav_path = None
