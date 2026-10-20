"""本地 MP4 播放器，支持随时停止。"""
import os
import signal
import subprocess
import threading


class MediaPlayer:
    def __init__(self, logger):
        self.logger = logger
        self._proc = None
        self._lock = threading.Lock()

    def play(self, path: str):
        """异步全屏播放 MP4。"""
        if not os.path.exists(path):
            self.logger.error(f'❌ 视频不存在: {path}')
            return False

        self.stop()
        try:
            proc = subprocess.Popen(
                [
                    'ffplay',
                    '-fs',
                    '-autoexit',
                    '-loglevel', 'error',
                    path,
                ],
                stdin=subprocess.DEVNULL,
                start_new_session=True,
            )
            with self._lock:
                self._proc = proc
            self.logger.info(f'🎬 开始播放: {path}')
            return True
        except Exception as e:
            self.logger.error(f'❌ 视频播放失败: {e}')
            return False

    def stop(self):
        """停止当前视频。"""
        with self._lock:
            proc = self._proc
            self._proc = None

        if proc is None or proc.poll() is not None:
            return

        try:
            os.killpg(proc.pid, signal.SIGTERM)
            proc.wait(timeout=1.0)
        except Exception:
            try:
                os.killpg(proc.pid, signal.SIGKILL)
            except Exception:
                pass
        self.logger.info('⏹ 视频已停止')

    def is_playing(self) -> bool:
        with self._lock:
            proc = self._proc
        return proc is not None and proc.poll() is None
