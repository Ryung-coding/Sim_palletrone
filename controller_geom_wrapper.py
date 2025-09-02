# controller_geom_wrapper.py
import ctypes as C
import numpy as np
import os
import sys
from typing import Optional, Sequence

class ControllerGeom:
  def __init__(self, lib_path: Optional[str] = None):
    default_name = {
      "darwin": "libcontroller_geom.dylib",
      "win32": "controller_geom.dll",
    }.get(sys.platform, "libcontroller_geom.so")

    here = os.path.dirname(os.path.abspath(__file__))
    candidates = []
    if lib_path is not None: candidates.append(lib_path)
    candidates.append(os.path.join(here, "controller_geom", default_name))
    candidates.append(os.path.join(here, default_name))
    candidates.append(default_name)

    last_err = None
    self._lib = None
    for p in candidates:
      try:
        if os.path.exists(p) or p == default_name:
          self._lib = C.CDLL(p)
          self._lib_path = p
          break
      except OSError as e:
        last_err = e

    if self._lib is None:
      raise FileNotFoundError(
        f"Failed to load controller_geom shared library.\n"
        f"Tried paths:\n  " + "\n  ".join(candidates) +
        (f"\nLast error: {last_err}" if last_err else "")
      )

    Handle = C.c_void_p
    self._Handle = Handle
    self._lib.controller_create.restype = Handle
    self._lib.controller_destroy.argtypes = [Handle]
    
    self._lib.controller_state_update.argtypes = [
      Handle,
      C.POINTER(C.c_double),  # pos_des3
      C.c_double,             # heading_yaw
      C.POINTER(C.c_double),  # pos3
      C.POINTER(C.c_double),  # vel3
      C.POINTER(C.c_double),  # acc3
      C.POINTER(C.c_double),  # quat_wxyz4
      C.POINTER(C.c_double),  # gyro3
      C.POINTER(C.c_double),  # inertia9
      C.POINTER(C.c_double),  # arm_pos20
    ]

    self._lib.controller_compute_pwm.argtypes = [
      Handle,
      C.POINTER(C.c_double),  # wrench4
      C.POINTER(C.c_double),  # pwm_out4
    ]
    self._lib.controller_compute_pwm.restype = None

    h = self._lib.controller_create()
    if not h: raise RuntimeError("controller_create() returned NULL handle.")
    self._h = h

    self._pwm_buf = (C.c_double * 4)()
    self._wrench_buf = (C.c_double * 4)()

  def close(self) -> None:
    """Explicitly destroy the native handle."""
    if getattr(self, "_h", None):
      try:
        self._lib.controller_destroy(self._h)
      finally:
        self._h = None

  def __enter__(self):
    return self

  def __exit__(self, exc_type, exc, tb):
    self.close()

  def __del__(self):
    try:
      if getattr(self, "_lib", None) and getattr(self, "_h", None):
        self._lib.controller_destroy(self._h)
    except Exception:
      pass
    finally:
      self._h = None

  @property
  def handle(self) -> C.c_void_p:
    return self._h

  # ---------- Utils ----------
  @staticmethod
  def _as_ptr(arr: np.ndarray) -> "C.POINTER(C.c_double)":
    return arr.ctypes.data_as(C.POINTER(C.c_double))

  @staticmethod
  def _ensure_shape(data: Sequence[float], shape) -> np.ndarray:
    a = np.asarray(data, dtype=np.float64)
    if not a.flags["C_CONTIGUOUS"]: a = np.ascontiguousarray(a)
    if np.prod(shape) != a.size: raise ValueError(f"Expected {np.prod(shape)} elements, got {a.size}")
    if tuple(a.shape) != tuple(shape): a = a.reshape(shape)
    return a

  def state_update(
    self,
    pos_des3: Sequence[float],
    heading_yaw: float,
    pos3: Sequence[float],
    vel3: Sequence[float],
    acc3: Sequence[float],
    quat_wxyz4: Sequence[float],
    gyro3: Sequence[float],
    inertia: Sequence[float],
    arm_pos: Sequence[float],
  ) -> None:
    pos_des3 = self._ensure_shape(pos_des3, (3,))
    pos3     = self._ensure_shape(pos3, (3,))
    vel3     = self._ensure_shape(vel3, (3,))
    acc3     = self._ensure_shape(acc3, (3,))
    quat     = self._ensure_shape(quat_wxyz4, (4,))
    gyro3    = self._ensure_shape(gyro3, (3,))

    inertia  = np.asarray(inertia, dtype=np.float64)
    if inertia.shape == (3, 3): inertia = np.ascontiguousarray(inertia.reshape(9,))
    else: inertia = self._ensure_shape(inertia, (9,))

    arm_pos  = np.asarray(arm_pos, dtype=np.float64)
    if arm_pos.shape == (4, 5): arm_pos = np.ascontiguousarray(arm_pos.reshape(20,))
    else: arm_pos = self._ensure_shape(arm_pos, (20,))

    self._lib.controller_state_update(
      self._h,
      self._as_ptr(pos_des3),
      C.c_double(float(heading_yaw)),
      self._as_ptr(pos3),
      self._as_ptr(vel3),
      self._as_ptr(acc3),
      self._as_ptr(quat),
      self._as_ptr(gyro3),
      self._as_ptr(inertia),
      self._as_ptr(arm_pos),
    )

  def compute_pwm(self) -> np.ndarray:
    self._lib.controller_compute_pwm(self._h, self._wrench_buf, self._pwm_buf)
    return np.ctypeslib.as_array(self._wrench_buf).copy(), np.ctypeslib.as_array(self._pwm_buf).copy()