"""Minimal TensorRT runner for the end-to-end YOLO person detector.

Why this exists: the desktop torch wheel on this Jetson Orin lacks sm_87 kernels,
so *any* torch op on the GPU crashes ("no kernel image available for execution").
Ultralytics' TensorRT backend moves the input to a torch CUDA tensor and casts it
there, so it cannot be used. This module instead drives the TensorRT engine
directly via pycuda: pre/post stays in numpy on the CPU, only the engine runs on
the GPU (native sm_87, no torch kernels involved).

Engine I/O is FP32. The output tensor is (1, 300, 6) = already NMS'd end-to-end
detections, each row [x1, y1, x2, y2, conf, cls] in letterboxed input coords.
"""

from __future__ import annotations

import numpy as np
import cv2
import tensorrt as trt
import pycuda.driver as cuda
import pycuda.autoinit  # noqa: F401  (creates/owns the primary CUDA context)


class TRTPersonDetector:
    """Run a TensorRT engine for person detection without touching torch GPU."""

    def __init__(self, engine_path: str, conf: float = 0.5, class_filter: int = 0):
        self.conf = conf
        self.class_filter = class_filter

        logger = trt.Logger(trt.Logger.WARNING)
        with open(engine_path, "rb") as f:
            engine = trt.Runtime(logger).deserialize_cuda_engine(f.read())
        self.context = engine.create_execution_context()
        self.stream = cuda.Stream()

        # Enumerate I/O tensors in engine index order (TRT10 named-tensor API).
        self.io = []
        for i in range(engine.num_io_tensors):
            name = engine.get_tensor_name(i)
            dtype = trt.nptype(engine.get_tensor_dtype(name))
            shape = tuple(engine.get_tensor_shape(name))
            is_input = engine.get_tensor_mode(name) == trt.TensorIOMode.INPUT
            host = cuda.pagelocked_empty(int(np.prod(shape)), dtype)
            device = cuda.mem_alloc(host.nbytes)
            self.io.append(
                dict(name=name, is_input=is_input, dtype=dtype,
                     shape=shape, host=host, device=device)
            )
        self.in_binding = next(b for b in self.io if b["is_input"])
        self.out_bindings = [b for b in self.io if not b["is_input"]]

        _, _, self.in_h, self.in_w = self.in_binding["shape"]  # (1, 3, H, W)
        self.in_dtype = self.in_binding["dtype"]

        # TRT 10/11 use execute_async_v3 with named tensor addresses; older
        # versions use execute_async_v2 with an ordered pointer list.
        self._trt10 = hasattr(self.context, "execute_async_v3")
        if self._trt10:
            for b in self.io:
                self.context.set_tensor_address(b["name"], int(b["device"]))

    def _letterbox(self, img: np.ndarray):
        """Resize-with-padding to the engine input size; return (canvas, r, left, top)."""
        h, w = img.shape[:2]
        r = min(self.in_h / h, self.in_w / w)
        nh, nw = int(round(h * r)), int(round(w * r))
        resized = cv2.resize(img, (nw, nh), interpolation=cv2.INTER_LINEAR)
        canvas = np.full((self.in_h, self.in_w, 3), 114, dtype=np.uint8)
        top = (self.in_h - nh) // 2
        left = (self.in_w - nw) // 2
        canvas[top:top + nh, left:left + nw] = resized
        return canvas, r, left, top

    def detect(self, img: np.ndarray):
        """Detect people in a BGR uint8 (H, W, 3) image.

        Returns a list of (x1, y1, x2, y2, conf) in the *original* image coords.
        """
        canvas, r, left, top = self._letterbox(img)
        x = canvas[:, :, ::-1].astype(self.in_dtype) / 255.0          # BGR->RGB, 0-1
        x = np.ascontiguousarray(x.transpose(2, 0, 1)[None])         # (1, 3, H, W)
        np.copyto(self.in_binding["host"], x.reshape(-1))
        cuda.memcpy_htod_async(self.in_binding["device"],
                               self.in_binding["host"], self.stream)

        if self._trt10:
            self.context.execute_async_v3(self.stream.handle)
        else:
            self.context.execute_async_v2(
                [int(b["device"]) for b in self.io], self.stream.handle)

        for b in self.out_bindings:
            cuda.memcpy_dtoh_async(b["host"], b["device"], self.stream)
        self.stream.synchronize()

        out = self.out_bindings[0]["host"].reshape(self.out_bindings[0]["shape"])[0]
        boxes = []
        for d in out:
            x1, y1, x2, y2, conf, cls = (float(v) for v in d)
            if conf < self.conf:
                continue
            if self.class_filter is not None and int(cls) != self.class_filter:
                continue
            boxes.append(((x1 - left) / r, (y1 - top) / r,
                          (x2 - left) / r, (y2 - top) / r, conf))
        return boxes
