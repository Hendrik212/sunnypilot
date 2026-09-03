import time

import numpy as np

from openpilot.cereal.visionipc import VisionStreamType
from msgq.visionipc import VisionIpcServer
from openpilot.cereal import messaging

from openpilot.tools.sim.lib.common import W, H

# Wide (extra) is stamped slightly behind narrow (main) so modeld pairs the two frames from
# the SAME send cycle. modeld's extra-drain loop keeps pulling wide frames until
#   meta_main.timestamp_sof < meta_extra.timestamp_sof + 25 ms
# i.e. until  extra.sof > main.sof - 25 ms.  At exactly -25 ms the comparison is false, so it
# over-drains and pairs narrow N with wide N+1 -- the "frames out of sync" spam, one whole
# sim frame of skew. Staying strictly inside the 25 ms window (and within the 10 ms
# out-of-sync tolerance) makes the same-cycle wide the one it stops on.
WIDE_TIMESTAMP_OFFSET_NS = 5_000_000


def rgb_to_nv12(rgb):
  """Convert RGB image to NV12 (YUV420) format using BT.601 coefficients."""
  h, w = rgb.shape[:2]
  r = rgb[:, :, 0].astype(np.int32)
  g = rgb[:, :, 1].astype(np.int32)
  b = rgb[:, :, 2].astype(np.int32)

  # Y plane - BT.601 coefficients (matches original OpenCL kernel)
  y = (((b * 13 + g * 65 + r * 33) + 64) >> 7) + 16
  y = np.clip(y, 0, 255).astype(np.uint8)

  # Subsample RGB for UV (2x2 box filter)
  r_sub = (r[0::2, 0::2] + r[0::2, 1::2] + r[1::2, 0::2] + r[1::2, 1::2] + 2) >> 2
  g_sub = (g[0::2, 0::2] + g[0::2, 1::2] + g[1::2, 0::2] + g[1::2, 1::2] + 2) >> 2
  b_sub = (b[0::2, 0::2] + b[0::2, 1::2] + b[1::2, 0::2] + b[1::2, 1::2] + 2) >> 2

  # U and V planes
  u = np.clip((b_sub * 56 - g_sub * 37 - r_sub * 19 + 0x8080) >> 8, 0, 255).astype(np.uint8)
  v = np.clip((r_sub * 56 - g_sub * 47 - b_sub * 9 + 0x8080) >> 8, 0, 255).astype(np.uint8)

  # Interleave UV for NV12 format
  uv = np.empty((h // 2, w), dtype=np.uint8)
  uv[:, 0::2] = u
  uv[:, 1::2] = v

  return np.concatenate([y.ravel(), uv.ravel()]).tobytes()


class Camerad:
  """Simulates the camerad daemon"""
  def __init__(self, dual_camera):
    self.pm = messaging.PubMaster(['narrowRoadCameraState', 'wideRoadCameraState'])

    self.frame_road_id = 0
    self.frame_wide_id = 0
    self.vipc_server = VisionIpcServer("camerad")

    self.vipc_server.create_buffers(VisionStreamType.VISION_STREAM_NARROW_ROAD, 5, W, H)
    if dual_camera:
      self.vipc_server.create_buffers(VisionStreamType.VISION_STREAM_WIDE_ROAD, 5, W, H)

    self.vipc_server.start_listener()

  def cam_send_yuv_road(self, yuv, eof_ns=None):
    self._send_yuv(yuv, self.frame_road_id, 'narrowRoadCameraState', VisionStreamType.VISION_STREAM_NARROW_ROAD, 0, eof_ns)
    self.frame_road_id += 1

  def cam_send_yuv_wide_road(self, yuv, eof_ns=None, frame_id=None):
    self._send_yuv(yuv, self.frame_wide_id, 'wideRoadCameraState', VisionStreamType.VISION_STREAM_WIDE_ROAD, WIDE_TIMESTAMP_OFFSET_NS, eof_ns)
    self.frame_wide_id += 1

  def rgb_to_yuv(self, rgb):
    """Convert RGB to NV12 YUV format."""
    assert rgb.shape == (H, W, 3), f"{rgb.shape}"
    assert rgb.dtype == np.uint8
    return rgb_to_nv12(rgb)

  def _send_yuv(self, yuv, frame_id, pub_type, yuv_type, ts_offset_ns=0, eof_ns=None):
    # Use a shared monotonic timestamp (passed from send_camera_images) so the narrow/wide
    # offset is exact; fall back to the clock if not provided.
    eof = (eof_ns if eof_ns is not None else int(time.monotonic() * 1e9)) - ts_offset_ns
    sof = eof - int(0.05 * 1e9)
    self.vipc_server.send(yuv_type, yuv, frame_id, eof, sof)

    dat = messaging.new_message(pub_type, valid=True)
    msg = {
      "frameId": frame_id,
      "transform": [1.0, 0.0, 0.0,
                    0.0, 1.0, 0.0,
                    0.0, 0.0, 1.0]
    }
    setattr(dat, pub_type, msg)
    self.pm.send(pub_type, dat)
